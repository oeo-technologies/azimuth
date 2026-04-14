/*
 * Multi-Protocol Scanner — Implementation
 * Part of Azimuth GPS Tracker firmware
 * 
 * Supports Meshtastic (protobuf + AES-CTR) and MeshCore (binary adverts)
 * Now with custom channel key support
 */

#include "meshtastic_scanner.h"
#include <RadioLib.h>
#include <mbedtls/aes.h>
#include <mbedtls/sha256.h>
#include <mbedtls/base64.h>

// -----------------------------------------------------------------------------
// External references (defined in main firmware)
// -----------------------------------------------------------------------------
extern SX1262* radio;
extern bool loraReady;

// Current Azimuth LoRa config (set by main firmware before scanner init)
// Scanner will save and restore these values
float    azimuthLoraFreq = 868.1f;
float    azimuthLoraBW   = 125.0f;
uint8_t  azimuthLoraSF   = 9;
uint8_t  azimuthLoraCR   = 5;
int8_t   azimuthLoraPwr  = 14;

// -----------------------------------------------------------------------------
// Module State
// -----------------------------------------------------------------------------

static MeshtasticScannerMode scannerMode = SCANNER_OFF;
static MeshtasticScannerState scannerState = SCAN_STATE_IDLE;
static MeshtasticPeer peers[SCANNER_MAX_PEERS];
static MeshtasticChannel channels[SCANNER_MAX_CHANNELS];
static LoraConfigBackup savedConfig;
static MeshtasticStats stats = {0};

static uint32_t lastScanStartMs = 0;
static uint32_t scanStartedMs = 0;
static uint8_t currentProtocol = 0;  // 0=Meshtastic, 1=MeshCore for AUTO mode

// Include default AQ== channel
bool meshtasticIncludeDefaultChannel = true;

// Meshtastic default channel key: "AQ==" = 0x01, expanded to 16 bytes
static const uint8_t MESHTASTIC_DEFAULT_KEY[16] = {
  0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
  0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};
// Note: This is the actual expanded key from "AQ==" after Meshtastic's key derivation

// Alternative: Simple key (if above doesn't work, try this)
static const uint8_t MESHTASTIC_SIMPLE_KEY[16] = {
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// -----------------------------------------------------------------------------
// Forward Declarations
// -----------------------------------------------------------------------------

static void saveLoraConfig();
static void switchToMeshtasticConfig();
static void switchToMeshCoreConfig();
static void restoreLoraConfig();
static bool processMeshtasticPacket(uint8_t* data, int len, int16_t rssi, int8_t snr);
static bool processMeshCorePacket(uint8_t* data, int len, int16_t rssi, int8_t snr);
static int  decryptPayloadMultiKey(uint8_t* encrypted, int len, uint32_t fromNode, uint32_t packetId, uint8_t* out);
static bool decryptWithKey(uint8_t* encrypted, int len, uint32_t fromNode, uint32_t packetId, 
                           const uint8_t* key, int keyLen, uint8_t* out);
static bool parseDataMessage(uint8_t* data, int len, uint32_t* portnum, uint8_t** payload, int* payloadLen);
static bool parsePosition(uint8_t* data, int len, double* lat, double* lon, float* alt);
static uint32_t decodeVarint(uint8_t* data, int* offset, int maxLen);
static void skipProtoField(uint8_t* data, int* offset, int maxLen, uint8_t wireType);
static void addOrUpdatePeer(uint32_t nodeId, double lat, double lon, float alt, 
                            int16_t rssi, int8_t snr, ExternalPeerSource source, const char* name, int8_t channelIdx);
static void expireOldPeers();
static void formatNodeId(uint32_t nodeId, char* out);
static bool expandMeshtasticKey(const char* pskBase64, uint8_t* expandedKey, uint8_t* keyLen);

// -----------------------------------------------------------------------------
// Public API Implementation
// -----------------------------------------------------------------------------

void meshtasticScannerInit() {
  memset(peers, 0, sizeof(peers));
  memset(channels, 0, sizeof(channels));
  memset(&stats, 0, sizeof(stats));
  scannerMode = SCANNER_OFF;
  scannerState = SCAN_STATE_IDLE;
  lastScanStartMs = 0;
  meshtasticIncludeDefaultChannel = true;
  Serial.println("[MESH] Scanner initialized");
}

void meshtasticScannerLoop() {
  if (scannerMode == SCANNER_OFF || !loraReady || !radio) {
    return;
  }

  uint32_t now = millis();

  switch (scannerState) {
    case SCAN_STATE_IDLE:
      // Check if it's time for a scan
      if (now - lastScanStartMs >= SCANNER_SCAN_INTERVAL_MS) {
        saveLoraConfig();
        lastScanStartMs = now;
        stats.scanCount++;
        stats.lastScanMs = now;
        
        // Determine which protocol(s) to scan
        if (scannerMode == SCANNER_MESHTASTIC) {
          scannerState = SCAN_STATE_SWITCHING_MESHTASTIC;
        } else if (scannerMode == SCANNER_MESHCORE) {
          scannerState = SCAN_STATE_SWITCHING_MESHCORE;
        } else if (scannerMode == SCANNER_AUTO) {
          // Alternate between protocols each scan
          if (currentProtocol == 0) {
            scannerState = SCAN_STATE_SWITCHING_MESHTASTIC;
          } else {
            scannerState = SCAN_STATE_SWITCHING_MESHCORE;
          }
          currentProtocol = (currentProtocol + 1) % 2;
        }
        Serial.println("[SCAN] Starting scan...");
      }
      break;

    case SCAN_STATE_SWITCHING_MESHTASTIC:
      switchToMeshtasticConfig();
      scanStartedMs = now;
      scannerState = SCAN_STATE_LISTENING_MESHTASTIC;
      Serial.println("[SCAN] Listening for Meshtastic packets...");
      break;

    case SCAN_STATE_LISTENING_MESHTASTIC:
      {
        if (loraRxFlag) {
          loraRxFlag = false;
          
          uint8_t rxBuf[256];
          int state = radio->readData(rxBuf, sizeof(rxBuf));
          if (state == RADIOLIB_ERR_NONE) {
            int len = radio->getPacketLength();
            if (len > 0 && len <= (int)sizeof(rxBuf)) {
              int16_t rssi = radio->getRSSI();
              int8_t snr = (int8_t)radio->getSNR();
              stats.meshtasticPackets++;
              Serial.printf("[SCAN] Meshtastic: %d bytes, RSSI=%d, SNR=%d\n", len, rssi, snr);
              
              if (processMeshtasticPacket(rxBuf, len, rssi, snr)) {
                stats.meshtasticDecoded++;
              }
            }
          }
          radio->startReceive();
        }
      }
      
      if (now - scanStartedMs >= SCANNER_SCAN_DURATION_MS) {
        // For AUTO mode, continue to MeshCore; otherwise restore
        if (scannerMode == SCANNER_AUTO && currentProtocol == 1) {
          scannerState = SCAN_STATE_SWITCHING_MESHCORE;
        } else {
          scannerState = SCAN_STATE_RESTORING;
        }
      }
      break;

    case SCAN_STATE_SWITCHING_MESHCORE:
      switchToMeshCoreConfig();
      scanStartedMs = now;
      scannerState = SCAN_STATE_LISTENING_MESHCORE;
      Serial.println("[SCAN] Listening for MeshCore packets...");
      break;

    case SCAN_STATE_LISTENING_MESHCORE:
      {
        if (loraRxFlag) {
          loraRxFlag = false;
          
          uint8_t rxBuf[256];
          int state = radio->readData(rxBuf, sizeof(rxBuf));
          if (state == RADIOLIB_ERR_NONE) {
            int len = radio->getPacketLength();
            if (len > 0 && len <= (int)sizeof(rxBuf)) {
              int16_t rssi = radio->getRSSI();
              int8_t snr = (int8_t)radio->getSNR();
              stats.meshcorePackets++;
              Serial.printf("[SCAN] MeshCore: %d bytes, RSSI=%d, SNR=%d\n", len, rssi, snr);
              
              if (processMeshCorePacket(rxBuf, len, rssi, snr)) {
                stats.meshcoreDecoded++;
              }
            }
          }
          radio->startReceive();
        }
      }
      
      if (now - scanStartedMs >= SCANNER_SCAN_DURATION_MS) {
        scannerState = SCAN_STATE_RESTORING;
      }
      break;

    case SCAN_STATE_RESTORING:
      restoreLoraConfig();
      expireOldPeers();
      scannerState = SCAN_STATE_IDLE;
      Serial.printf("[SCAN] Complete. Meshtastic: %lu, MeshCore: %lu decoded\n", 
                    stats.meshtasticDecoded, stats.meshcoreDecoded);
      break;
  }
}

void meshtasticScannerSetMode(MeshtasticScannerMode mode) {
  if (mode != scannerMode) {
    scannerMode = mode;
    if (mode == SCANNER_OFF && scannerState != SCAN_STATE_IDLE) {
      // Abort current scan
      restoreLoraConfig();
      scannerState = SCAN_STATE_IDLE;
    }
    Serial.printf("[MESH] Scanner mode set to %d\n", mode);
  }
}

MeshtasticScannerMode meshtasticScannerGetMode() {
  return scannerMode;
}

bool meshtasticScannerIsActive() {
  return (scannerState == SCAN_STATE_LISTENING_MESHTASTIC || 
          scannerState == SCAN_STATE_SWITCHING_MESHTASTIC ||
          scannerState == SCAN_STATE_LISTENING_MESHCORE ||
          scannerState == SCAN_STATE_SWITCHING_MESHCORE);
}

int meshtasticGetPeers(MeshtasticPeer* out, int maxPeers) {
  int count = 0;
  for (int i = 0; i < SCANNER_MAX_PEERS && count < maxPeers; i++) {
    if (peers[i].valid) {
      out[count++] = peers[i];
    }
  }
  return count;
}

int meshtasticGetPeerCount() {
  int count = 0;
  for (int i = 0; i < SCANNER_MAX_PEERS; i++) {
    if (peers[i].valid) count++;
  }
  return count;
}

void meshtasticClearPeers() {
  memset(peers, 0, sizeof(peers));
  Serial.println("[MESH] Peers cleared");
}

void meshtasticForceScan() {
  if (scannerMode != SCANNER_OFF && scannerState == SCAN_STATE_IDLE) {
    lastScanStartMs = 0;  // Trigger immediate scan
    Serial.println("[MESH] Forced scan triggered");
  }
}

const MeshtasticStats& meshtasticGetStats() {
  return stats;
}

// -----------------------------------------------------------------------------
// Custom Channel Management
// -----------------------------------------------------------------------------

int meshtasticAddChannel(const char* name, const char* pskBase64) {
  if (!name || !pskBase64 || strlen(name) == 0 || strlen(pskBase64) == 0) {
    Serial.println("[MESH] Invalid channel parameters");
    return -1;
  }
  
  // Check if channel with same name exists — update it
  for (int i = 0; i < SCANNER_MAX_CHANNELS; i++) {
    if (channels[i].enabled && strcmp(channels[i].name, name) == 0) {
      // Update existing
      strncpy(channels[i].pskBase64, pskBase64, MESH_CHANNEL_KEY_LEN - 1);
      channels[i].pskBase64[MESH_CHANNEL_KEY_LEN - 1] = '\0';
      if (!expandMeshtasticKey(pskBase64, channels[i].expandedKey, &channels[i].keyLen)) {
        Serial.printf("[MESH] Failed to expand key for channel %s\n", name);
        return -1;
      }
      channels[i].packetCount = 0;  // Reset count
      Serial.printf("[MESH] Updated channel %d: %s\n", i, name);
      return i;
    }
  }
  
  // Find empty slot
  for (int i = 0; i < SCANNER_MAX_CHANNELS; i++) {
    if (!channels[i].enabled) {
      strncpy(channels[i].name, name, MESH_CHANNEL_NAME_LEN - 1);
      channels[i].name[MESH_CHANNEL_NAME_LEN - 1] = '\0';
      strncpy(channels[i].pskBase64, pskBase64, MESH_CHANNEL_KEY_LEN - 1);
      channels[i].pskBase64[MESH_CHANNEL_KEY_LEN - 1] = '\0';
      
      if (!expandMeshtasticKey(pskBase64, channels[i].expandedKey, &channels[i].keyLen)) {
        Serial.printf("[MESH] Failed to expand key for channel %s\n", name);
        return -1;
      }
      
      channels[i].enabled = true;
      channels[i].packetCount = 0;
      Serial.printf("[MESH] Added channel %d: %s (key %d bytes)\n", i, name, channels[i].keyLen);
      return i;
    }
  }
  
  Serial.println("[MESH] No free channel slots");
  return -1;
}

bool meshtasticRemoveChannel(int idx) {
  if (idx < 0 || idx >= SCANNER_MAX_CHANNELS) return false;
  if (!channels[idx].enabled) return false;
  
  Serial.printf("[MESH] Removed channel %d: %s\n", idx, channels[idx].name);
  memset(&channels[idx], 0, sizeof(MeshtasticChannel));
  return true;
}

bool meshtasticSetChannelEnabled(int idx, bool enabled) {
  if (idx < 0 || idx >= SCANNER_MAX_CHANNELS) return false;
  channels[idx].enabled = enabled;
  return true;
}

int meshtasticGetChannels(MeshtasticChannel* out, int maxChannels) {
  int count = 0;
  for (int i = 0; i < SCANNER_MAX_CHANNELS && count < maxChannels; i++) {
    if (channels[i].enabled) {
      out[count++] = channels[i];
    }
  }
  return count;
}

int meshtasticGetChannelCount() {
  int count = 0;
  for (int i = 0; i < SCANNER_MAX_CHANNELS; i++) {
    if (channels[i].enabled) count++;
  }
  return count;
}

void meshtasticClearChannels() {
  memset(channels, 0, sizeof(channels));
  Serial.println("[MESH] All custom channels cleared");
}

// Parse Meshtastic share URL
// Format: https://meshtastic.org/e/#CgMSAQESBHRlc3QaBAgBOAE=
// The base64 part is a protobuf-encoded ChannelSet
bool meshtasticParseShareUrl(const char* url, char* nameOut, int nameMax, char* pskOut, int pskMax) {
  // Look for "#" or "e/#" in URL
  const char* hashPos = strstr(url, "#");
  if (!hashPos) {
    // Maybe just the base64 part was passed
    hashPos = url - 1;  // Will be incremented below
  }
  
  const char* b64Start = hashPos + 1;
  if (strlen(b64Start) < 4) {
    Serial.println("[MESH] URL too short");
    return false;
  }
  
  // URL-safe base64 to standard base64
  char b64Clean[256];
  int j = 0;
  for (int i = 0; b64Start[i] && j < 255; i++) {
    char c = b64Start[i];
    if (c == '-') c = '+';
    else if (c == '_') c = '/';
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || 
        (c >= '0' && c <= '9') || c == '+' || c == '/' || c == '=') {
      b64Clean[j++] = c;
    }
  }
  b64Clean[j] = '\0';
  
  // Add padding if needed
  while (j % 4 != 0) {
    b64Clean[j++] = '=';
    b64Clean[j] = '\0';
  }
  
  // Decode base64
  uint8_t decoded[192];
  size_t decodedLen = 0;
  
  int ret = mbedtls_base64_decode(decoded, sizeof(decoded), &decodedLen, 
                                   (const uint8_t*)b64Clean, strlen(b64Clean));
  if (ret != 0) {
    Serial.printf("[MESH] Base64 decode failed: %d\n", ret);
    return false;
  }
  
  // Parse protobuf ChannelSet
  // Field 1 (settings) is a repeated ChannelSettings
  // ChannelSettings has: field 2 = name (string), field 1 = psk (bytes)
  
  int offset = 0;
  bool foundName = false;
  bool foundPsk = false;
  
  while (offset < (int)decodedLen) {
    uint8_t tag = decoded[offset++];
    uint8_t fieldNum = tag >> 3;
    uint8_t wireType = tag & 0x07;
    
    if (fieldNum == 1 && wireType == 2) {
      // settings - length-delimited (ChannelSettings)
      uint32_t len = decodeVarint(decoded, &offset, decodedLen);
      int innerEnd = offset + len;
      
      // Parse inner ChannelSettings
      while (offset < innerEnd && offset < (int)decodedLen) {
        uint8_t innerTag = decoded[offset++];
        uint8_t innerField = innerTag >> 3;
        uint8_t innerWire = innerTag & 0x07;
        
        if (innerField == 1 && innerWire == 2) {
          // psk - bytes
          uint32_t pskLen = decodeVarint(decoded, &offset, decodedLen);
          if (pskLen > 0 && pskLen <= 32 && offset + pskLen <= decodedLen) {
            // Encode PSK to base64
            size_t outLen = 0;
            mbedtls_base64_encode((uint8_t*)pskOut, pskMax, &outLen, 
                                   &decoded[offset], pskLen);
            pskOut[outLen] = '\0';
            foundPsk = true;
          }
          offset += pskLen;
        } else if (innerField == 2 && innerWire == 2) {
          // name - string
          uint32_t nameLen = decodeVarint(decoded, &offset, decodedLen);
          if (nameLen > 0 && (int)nameLen < nameMax && offset + nameLen <= decodedLen) {
            memcpy(nameOut, &decoded[offset], nameLen);
            nameOut[nameLen] = '\0';
            foundName = true;
          }
          offset += nameLen;
        } else {
          // Skip field
          skipProtoField(decoded, &offset, decodedLen, innerWire);
        }
      }
      
      // Only need first channel
      if (foundPsk) break;
      
    } else {
      skipProtoField(decoded, &offset, decodedLen, wireType);
    }
  }
  
  // Use default name if not found
  if (!foundName && foundPsk) {
    strncpy(nameOut, "Imported", nameMax - 1);
    nameOut[nameMax - 1] = '\0';
    foundName = true;
  }
  
  Serial.printf("[MESH] Parsed URL: name=%s psk=%s\n", 
                foundName ? nameOut : "(none)", 
                foundPsk ? pskOut : "(none)");
  
  return foundName && foundPsk;
}

// -----------------------------------------------------------------------------
// Key Expansion
// -----------------------------------------------------------------------------

static bool expandMeshtasticKey(const char* pskBase64, uint8_t* expandedKey, uint8_t* keyLen) {
  // Convert URL-safe base64 to standard base64
  char stdBase64[48];
  int j = 0;
  for (int i = 0; pskBase64[i] && j < 46; i++) {
    char c = pskBase64[i];
    if (c == '-') c = '+';
    else if (c == '_') c = '/';
    stdBase64[j++] = c;
  }
  stdBase64[j] = '\0';
  
  // Remove existing padding first, then re-add correct amount
  while (j > 0 && stdBase64[j-1] == '=') {
    j--;
    stdBase64[j] = '\0';
  }
  
  // Add correct padding
  while (j % 4 != 0 && j < 46) {
    stdBase64[j++] = '=';
    stdBase64[j] = '\0';
  }
  
  Serial.printf("[MESH] Base64 input: %s -> %s\n", pskBase64, stdBase64);
  
  // Decode base64 PSK
  uint8_t rawKey[33];
  size_t rawLen = 0;
  
  int ret = mbedtls_base64_decode(rawKey, sizeof(rawKey) - 1, &rawLen,
                                   (const uint8_t*)stdBase64, strlen(stdBase64));
  if (ret != 0) {
    Serial.printf("[MESH] Base64 decode failed: %d\n", ret);
    return false;
  }
  
  if (rawLen == 0) {
    Serial.println("[MESH] Base64 decoded to zero bytes");
    return false;
  }
  
  Serial.printf("[MESH] Raw key length: %d bytes\n", rawLen);
  
  // Meshtastic key expansion:
  // - 1 byte (0x01 = AQ==): use the known Meshtastic default expanded key
  // - 16 bytes: use directly as AES-128
  // - 32 bytes: use directly as AES-256 (we'll use first 16 for AES-128)
  // - Other: hash to get key
  
  if (rawLen == 1 && rawKey[0] == 0x01) {
    // Special case: AQ== is the default key, use Meshtastic's known expansion
    memcpy(expandedKey, MESHTASTIC_DEFAULT_KEY, 16);
    *keyLen = 16;
  } else if (rawLen == 16) {
    // Use directly
    memcpy(expandedKey, rawKey, 16);
    *keyLen = 16;
  } else if (rawLen == 32) {
    // Use first 16 bytes for AES-128 (we don't support AES-256)
    memcpy(expandedKey, rawKey, 16);
    *keyLen = 16;
  } else {
    // Hash the key material using SHA256
    uint8_t hash[32];
    mbedtls_sha256(rawKey, rawLen, hash, 0);  // 0 = SHA-256 (not SHA-224)
    memcpy(expandedKey, hash, 16);
    *keyLen = 16;
  }
  
  Serial.printf("[MESH] Expanded key: %02X%02X%02X%02X...\n", 
                expandedKey[0], expandedKey[1], expandedKey[2], expandedKey[3]);
  
  return true;
}

// -----------------------------------------------------------------------------
// LoRa Configuration
// -----------------------------------------------------------------------------

static void saveLoraConfig() {
  // Save current Azimuth config (set by main firmware)
  savedConfig.frequency = azimuthLoraFreq;
  savedConfig.bandwidth = azimuthLoraBW;
  savedConfig.spreadingFactor = azimuthLoraSF;
  savedConfig.codingRate = azimuthLoraCR;
  savedConfig.syncWord = 0x12;           // Azimuth private sync word (RADIOLIB_SX126X_SYNC_WORD_PRIVATE)
  savedConfig.power = azimuthLoraPwr;
  savedConfig.preamble = 8;
  
  Serial.printf("[SCAN] Azimuth config saved: %.1f MHz SF%d BW%.0f\n", 
                savedConfig.frequency, savedConfig.spreadingFactor, savedConfig.bandwidth);
}

static void switchToMeshtasticConfig() {
  radio->standby();
  
  radio->setFrequency(MESHTASTIC_FREQ_EU868);
  radio->setBandwidth(MESHTASTIC_BW);
  radio->setSpreadingFactor(MESHTASTIC_SF);
  radio->setCodingRate(MESHTASTIC_CR);
  radio->setSyncWord(MESHTASTIC_SYNC_WORD);
  radio->setPreambleLength(MESHTASTIC_PREAMBLE);
  radio->setOutputPower(MESHTASTIC_POWER);
  
  // Setup interrupt and start receiving
  extern void loraRxISR();
  radio->setDio1Action(loraRxISR);
  loraRxFlag = false;
  radio->startReceive();
  
  Serial.printf("[SCAN] Meshtastic config: %.3f MHz, SF%d, BW%.0f\n",
                MESHTASTIC_FREQ_EU868, MESHTASTIC_SF, MESHTASTIC_BW);
}

static void switchToMeshCoreConfig() {
  radio->standby();
  
  radio->setFrequency(MESHCORE_FREQ_EU868);
  radio->setBandwidth(MESHCORE_BW);
  radio->setSpreadingFactor(MESHCORE_SF);
  radio->setCodingRate(MESHCORE_CR);
  radio->setSyncWord(MESHCORE_SYNC_WORD);
  radio->setPreambleLength(MESHCORE_PREAMBLE);
  radio->setOutputPower(MESHCORE_POWER);
  
  // Setup interrupt and start receiving
  extern void loraRxISR();
  radio->setDio1Action(loraRxISR);
  loraRxFlag = false;
  radio->startReceive();
  
  Serial.printf("[SCAN] MeshCore config: %.3f MHz, SF%d, BW%.1f\n",
                MESHCORE_FREQ_EU868, MESHCORE_SF, MESHCORE_BW);
}

static void restoreLoraConfig() {
  radio->standby();
  
  radio->setFrequency(savedConfig.frequency);
  radio->setBandwidth(savedConfig.bandwidth);
  radio->setSpreadingFactor(savedConfig.spreadingFactor);
  radio->setCodingRate(savedConfig.codingRate);
  radio->setSyncWord(savedConfig.syncWord);
  radio->setPreambleLength(savedConfig.preamble);
  radio->setOutputPower(savedConfig.power);
  
  // Re-setup interrupt and start receiving for Azimuth
  extern void loraRxISR();
  radio->setDio1Action(loraRxISR);
  loraRxFlag = false;
  radio->startReceive();
  
  Serial.printf("[SCAN] Azimuth config restored: %.1f MHz SF%d BW%.0f\n",
                savedConfig.frequency, savedConfig.spreadingFactor, savedConfig.bandwidth);
}

// -----------------------------------------------------------------------------
// Meshtastic Packet Processing
// -----------------------------------------------------------------------------

static bool processMeshtasticPacket(uint8_t* data, int len, int16_t rssi, int8_t snr) {
  // Meshtastic packet structure:
  // Bytes 0-3:   to (destination node ID, 0xFFFFFFFF = broadcast)
  // Bytes 4-7:   from (source node ID)
  // Bytes 8-11:  id (packet ID, used as nonce)
  // Byte 12:     flags (hop_limit, hop_start, want_ack, via_mqtt)
  // Byte 13:     channel hash
  // Bytes 14-15: reserved/padding
  // Bytes 16+:   encrypted payload
  
  if (len < 20) {
    return false;
  }
  
  // Parse header (little-endian)
  uint32_t toNode, fromNode, packetId;
  memcpy(&toNode, &data[0], 4);
  memcpy(&fromNode, &data[4], 4);
  memcpy(&packetId, &data[8], 4);
  uint8_t flags = data[12];
  uint8_t channelHash = data[13];
  
  // Deduplicate by packet ID
  static uint32_t lastPacketId = 0;
  if (packetId == lastPacketId) {
    return false;  // Already processed
  }
  lastPacketId = packetId;
  
  // Encrypted payload starts at byte 16
  int payloadLen = len - 16;
  if (payloadLen <= 0) {
    return false;
  }
  
  // Decrypt payload — try all keys
  uint8_t decrypted[240];
  int channelIdx = decryptPayloadMultiKey(&data[16], payloadLen, fromNode, packetId, decrypted);
  if (channelIdx < -1) {
    stats.decryptFails++;
    return false;
  }
  
  if (channelIdx >= 0) {
    channels[channelIdx].packetCount++;
  }
  
  // Parse Data message to get portnum and inner payload
  uint32_t portnum = 0;
  uint8_t* innerPayload = nullptr;
  int innerLen = 0;
  
  if (!parseDataMessage(decrypted, payloadLen, &portnum, &innerPayload, &innerLen)) {
    stats.parseFails++;
    return false;
  }
  
  // Only process position packets (portnum 3)
  if (portnum != MESHTASTIC_PORTNUM_POSITION) {
    return false;
  }
  
  // Parse Position protobuf
  double lat = 0, lon = 0;
  float alt = 0;
  
  if (!parsePosition(innerPayload, innerLen, &lat, &lon, &alt)) {
    stats.parseFails++;
    Serial.println("[SCAN] Failed to parse Position");
    return false;
  }
  
  Serial.printf("[SCAN] Meshtastic position: lat=%.6f lon=%.6f alt=%.0f\n", lat, lon, alt);
  
  // Add/update peer with channel info
  addOrUpdatePeer(fromNode, lat, lon, alt, rssi, snr, PEER_SOURCE_MESHTASTIC, nullptr, (int8_t)channelIdx);
  
  return true;
}

// -----------------------------------------------------------------------------
// MeshCore Packet Processing
// -----------------------------------------------------------------------------

static bool processMeshCorePacket(uint8_t* data, int len, int16_t rssi, int8_t snr) {
  // MeshCore packet structure:
  // Byte 0: Header
  //   Bits 0-1: Route type (0=FLOOD, 1=DIRECT, etc.)
  //   Bits 2-5: Payload type (0x04 = ADVERT)
  //   Bits 6-7: Protocol version
  // Byte 1+: Variable depending on packet type
  //
  // For ADVERT packets (payload type 0x04):
  // After header: [path_len:1] [path:0-64] [pubkey:32] [lat:4] [lon:4] [name:var]
  
  if (len < 2) {
    Serial.println("[SCAN] MeshCore packet too short");
    return false;
  }
  
  uint8_t header = data[0];
  uint8_t routeType = header & 0x03;
  uint8_t payloadType = (header >> 2) & 0x0F;
  uint8_t protoVersion = (header >> 6) & 0x03;
  
  Serial.printf("[SCAN] MeshCore: route=%d payload=%d ver=%d len=%d\n", 
                routeType, payloadType, protoVersion, len);
  
  // We're only interested in ADVERT packets
  if (payloadType != MESHCORE_PAYLOAD_ADVERT) {
    return false;
  }
  
  // Parse ADVERT packet
  // Based on companion protocol docs:
  // Adverts contain: pubkey (32 bytes), lat/lon (each 32-bit LE / 1e6), name
  
  int offset = 1;  // Skip header
  
  // Check for path length (optional in some versions)
  if (offset >= len) return false;
  uint8_t pathLen = data[offset++];
  
  // Skip path data
  if (pathLen > 0 && pathLen <= 64) {
    offset += pathLen;
  }
  
  // Need at least: pubkey (32) + lat (4) + lon (4) = 40 bytes
  if (offset + 40 > len) {
    Serial.println("[SCAN] MeshCore advert too short for position");
    return false;
  }
  
  // Skip pubkey (32 bytes) — we'll use a hash of it for nodeId
  uint32_t nodeId = 0;
  for (int i = 0; i < 4; i++) {
    nodeId ^= ((uint32_t)data[offset + i] << (i * 8));
  }
  nodeId |= 0xC0000000;  // Mark as MeshCore node
  offset += 32;
  
  // Parse lat/lon (32-bit signed, little-endian, scaled by 1e6)
  int32_t latI, lonI;
  memcpy(&latI, &data[offset], 4); offset += 4;
  memcpy(&lonI, &data[offset], 4); offset += 4;
  
  double lat = latI / 1e6;
  double lon = lonI / 1e6;
  float alt = 0;  // MeshCore adverts don't include altitude
  
  // Optional: parse name if present
  char name[12] = "";
  if (offset < len) {
    int nameLen = len - offset;
    if (nameLen > 11) nameLen = 11;
    memcpy(name, &data[offset], nameLen);
    name[nameLen] = '\0';
  }
  
  Serial.printf("[SCAN] MeshCore advert: lat=%.6f lon=%.6f name=%s\n", lat, lon, name);
  
  // Add/update peer
  addOrUpdatePeer(nodeId, lat, lon, alt, rssi, snr, PEER_SOURCE_MESHCORE, name[0] ? name : nullptr, -1);
  
  return true;
}

// -----------------------------------------------------------------------------
// AES-CTR Decryption — Multi-Key
// -----------------------------------------------------------------------------

// Returns channel index that successfully decrypted, -1 for default key, -2 for failure
static int decryptPayloadMultiKey(uint8_t* encrypted, int len, uint32_t fromNode, uint32_t packetId, uint8_t* out) {
  // Try custom channels first
  for (int i = 0; i < SCANNER_MAX_CHANNELS; i++) {
    if (channels[i].enabled) {
      if (decryptWithKey(encrypted, len, fromNode, packetId, 
                         channels[i].expandedKey, channels[i].keyLen, out)) {
        if (out[0] == 0x08) {
          return i;
        }
      }
    }
  }
  
  // Try default key if enabled
  if (meshtasticIncludeDefaultChannel) {
    if (decryptWithKey(encrypted, len, fromNode, packetId, 
                       MESHTASTIC_DEFAULT_KEY, 16, out)) {
      if (out[0] == 0x08) {
        return -1;
      }
    }
    
    // Also try simple key (0x01 zero-padded)
    if (decryptWithKey(encrypted, len, fromNode, packetId,
                       MESHTASTIC_SIMPLE_KEY, 16, out)) {
      if (out[0] == 0x08) {
        return -1;
      }
    }
  }
  
  return -2;  // Failed
}

static bool decryptWithKey(uint8_t* encrypted, int len, uint32_t fromNode, uint32_t packetId,
                           const uint8_t* key, int keyLen, uint8_t* out) {
  // Build nonce for AES-CTR
  // Meshtastic nonce construction (from CryptoEngine.cpp):
  // Bytes 0-7:   packetId (little-endian, zero-padded to 64 bits)
  // Bytes 8-15:  fromNode (little-endian, zero-padded to 64 bits)
  
  uint8_t nonce[16];
  memset(nonce, 0, 16);
  memcpy(&nonce[0], &packetId, 4);   // Bytes 0-3: packetId
  memcpy(&nonce[8], &fromNode, 4);   // Bytes 8-11: fromNode
  
  uint8_t streamBlock[16];
  size_t ncOff = 0;
  
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, key, keyLen * 8);  // keyLen in bits
  
  int ret = mbedtls_aes_crypt_ctr(&aes, len, &ncOff, nonce, streamBlock, encrypted, out);
  mbedtls_aes_free(&aes);
  
  return (ret == 0);
}

// -----------------------------------------------------------------------------
// Protobuf Parsing (Manual, minimal implementation)
// -----------------------------------------------------------------------------

static uint32_t decodeVarint(uint8_t* data, int* offset, int maxLen) {
  uint32_t result = 0;
  int shift = 0;
  
  while (*offset < maxLen) {
    uint8_t b = data[(*offset)++];
    result |= (uint32_t)(b & 0x7F) << shift;
    if ((b & 0x80) == 0) break;
    shift += 7;
    if (shift >= 35) break;  // Prevent infinite loop
  }
  
  return result;
}

static int32_t decodeSignedVarint(uint8_t* data, int* offset, int maxLen) {
  uint32_t raw = decodeVarint(data, offset, maxLen);
  // ZigZag decode for sint32
  return (int32_t)((raw >> 1) ^ -(int32_t)(raw & 1));
}

static void skipProtoField(uint8_t* data, int* offset, int maxLen, uint8_t wireType) {
  switch (wireType) {
    case 0: // Varint
      decodeVarint(data, offset, maxLen);
      break;
    case 1: // 64-bit
      *offset += 8;
      break;
    case 2: // Length-delimited
      {
        uint32_t len = decodeVarint(data, offset, maxLen);
        *offset += len;
      }
      break;
    case 5: // 32-bit
      *offset += 4;
      break;
    default:
      // Unknown wire type, can't skip reliably
      break;
  }
  
  if (*offset > maxLen) *offset = maxLen;
}

static bool parseDataMessage(uint8_t* data, int len, uint32_t* portnum, uint8_t** payload, int* payloadLen) {
  // Meshtastic Data message:
  // field 1: portnum (varint)
  // field 2: payload (bytes)
  
  int offset = 0;
  *portnum = 0;
  *payload = nullptr;
  *payloadLen = 0;
  
  while (offset < len) {
    if (offset >= len) break;
    
    uint8_t tag = data[offset++];
    uint8_t fieldNum = tag >> 3;
    uint8_t wireType = tag & 0x07;
    
    if (fieldNum == 1 && wireType == 0) {
      // portnum - varint
      *portnum = decodeVarint(data, &offset, len);
    } else if (fieldNum == 2 && wireType == 2) {
      // payload - length-delimited
      *payloadLen = decodeVarint(data, &offset, len);
      if (offset + *payloadLen <= len) {
        *payload = &data[offset];
      }
      offset += *payloadLen;
    } else {
      // Skip unknown field
      skipProtoField(data, &offset, len, wireType);
    }
  }
  
  return (*portnum != 0 && *payload != nullptr);
}

static bool parsePosition(uint8_t* data, int len, double* lat, double* lon, float* alt) {
  // Meshtastic Position message:
  // field 1: latitude_i (sfixed32) - degrees * 1e7
  // field 2: longitude_i (sfixed32) - degrees * 1e7
  // field 3: altitude (int32) - meters
  
  int offset = 0;
  int32_t latI = 0, lonI = 0, altI = 0;
  bool hasLat = false, hasLon = false;
  
  while (offset < len) {
    if (offset >= len) break;
    
    uint8_t tag = data[offset++];
    uint8_t fieldNum = tag >> 3;
    uint8_t wireType = tag & 0x07;
    
    if (fieldNum == 1 && wireType == 5) {
      // latitude_i - sfixed32 (4 bytes, little-endian)
      if (offset + 4 <= len) {
        memcpy(&latI, &data[offset], 4);
        hasLat = true;
      }
      offset += 4;
    } else if (fieldNum == 2 && wireType == 5) {
      // longitude_i - sfixed32
      if (offset + 4 <= len) {
        memcpy(&lonI, &data[offset], 4);
        hasLon = true;
      }
      offset += 4;
    } else if (fieldNum == 3 && wireType == 0) {
      // altitude - int32 varint
      altI = (int32_t)decodeVarint(data, &offset, len);
    } else {
      // Skip unknown field
      skipProtoField(data, &offset, len, wireType);
    }
  }
  
  if (hasLat && hasLon) {
    *lat = latI / 1e7;
    *lon = lonI / 1e7;
    *alt = (float)altI;
    return true;
  }
  
  return false;
}

// -----------------------------------------------------------------------------
// Peer Management
// -----------------------------------------------------------------------------

static void formatNodeId(uint32_t nodeId, char* out) {
  // Format as !AABBCCDD
  sprintf(out, "!%08x", nodeId);
}

static void addOrUpdatePeer(uint32_t nodeId, double lat, double lon, float alt, 
                            int16_t rssi, int8_t snr, ExternalPeerSource source, const char* name, int8_t channelIdx) {
  // Check if peer already exists
  for (int i = 0; i < SCANNER_MAX_PEERS; i++) {
    if (peers[i].valid && peers[i].nodeId == nodeId) {
      // Update existing
      peers[i].lat = lat;
      peers[i].lon = lon;
      peers[i].alt = alt;
      peers[i].lastSeenMs = millis();
      peers[i].rssi = rssi;
      peers[i].snr = snr;
      peers[i].source = source;
      peers[i].channelIdx = channelIdx;
      // Update name if provided
      if (name && name[0]) {
        strncpy(peers[i].shortName, name, sizeof(peers[i].shortName) - 1);
        peers[i].shortName[sizeof(peers[i].shortName) - 1] = '\0';
      }
      Serial.printf("[SCAN] Updated peer %s (%s)\n", peers[i].shortName,
                    source == PEER_SOURCE_MESHTASTIC ? "Meshtastic" : "MeshCore");
      return;
    }
  }
  
  // Find empty slot
  for (int i = 0; i < SCANNER_MAX_PEERS; i++) {
    if (!peers[i].valid) {
      peers[i].valid = true;
      peers[i].nodeId = nodeId;
      peers[i].source = source;
      peers[i].channelIdx = channelIdx;
      if (name && name[0]) {
        strncpy(peers[i].shortName, name, sizeof(peers[i].shortName) - 1);
        peers[i].shortName[sizeof(peers[i].shortName) - 1] = '\0';
      } else {
        formatNodeId(nodeId, peers[i].shortName);
      }
      peers[i].lat = lat;
      peers[i].lon = lon;
      peers[i].alt = alt;
      peers[i].lastSeenMs = millis();
      peers[i].rssi = rssi;
      peers[i].snr = snr;
      Serial.printf("[SCAN] Added new peer %s (%s)\n", peers[i].shortName,
                    source == PEER_SOURCE_MESHTASTIC ? "Meshtastic" : "MeshCore");
      return;
    }
  }
  
  // No empty slot - replace oldest
  uint32_t oldestTime = UINT32_MAX;
  int oldestIdx = 0;
  for (int i = 0; i < SCANNER_MAX_PEERS; i++) {
    if (peers[i].lastSeenMs < oldestTime) {
      oldestTime = peers[i].lastSeenMs;
      oldestIdx = i;
    }
  }
  
  peers[oldestIdx].valid = true;
  peers[oldestIdx].nodeId = nodeId;
  peers[oldestIdx].source = source;
  peers[oldestIdx].channelIdx = channelIdx;
  if (name && name[0]) {
    strncpy(peers[oldestIdx].shortName, name, sizeof(peers[oldestIdx].shortName) - 1);
    peers[oldestIdx].shortName[sizeof(peers[oldestIdx].shortName) - 1] = '\0';
  } else {
    formatNodeId(nodeId, peers[oldestIdx].shortName);
  }
  peers[oldestIdx].lat = lat;
  peers[oldestIdx].lon = lon;
  peers[oldestIdx].alt = alt;
  peers[oldestIdx].lastSeenMs = millis();
  peers[oldestIdx].rssi = rssi;
  peers[oldestIdx].snr = snr;
  Serial.printf("[SCAN] Replaced oldest peer with %s (%s)\n", peers[oldestIdx].shortName,
                source == PEER_SOURCE_MESHTASTIC ? "Meshtastic" : "MeshCore");
}

static void expireOldPeers() {
  uint32_t now = millis();
  for (int i = 0; i < SCANNER_MAX_PEERS; i++) {
    if (peers[i].valid && (now - peers[i].lastSeenMs) > SCANNER_PEER_TIMEOUT_MS) {
      Serial.printf("[SCAN] Expired peer %s\n", peers[i].shortName);
      peers[i].valid = false;
    }
  }
}
