/*
 * Multi-Protocol Scanner — Passive listener for Meshtastic & MeshCore
 * Part of Azimuth GPS Tracker firmware
 * 
 * Receives and decodes position packets from:
 * - Meshtastic devices on default "LongFast" channel (public key AQ==)
 * - Meshtastic devices on custom channels (user-provided PSK)
 * - MeshCore devices via advert broadcasts (unencrypted)
 * 
 * Receive-only, no transmission to external networks.
 */

#ifndef MESHTASTIC_SCANNER_H
#define MESHTASTIC_SCANNER_H

#include <Arduino.h>

// External reference to LoRa RX interrupt flag from main firmware
extern volatile bool loraRxFlag;

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

#define SCANNER_SCAN_INTERVAL_MS      30000   // Scan every 30 seconds
#define SCANNER_SCAN_DURATION_MS      8000    // Listen for 8 seconds per protocol
#define SCANNER_PEER_TIMEOUT_MS       600000  // Expire peers after 10 minutes
#define SCANNER_MAX_PEERS             10      // Maximum external peers to track
#define SCANNER_MAX_CHANNELS          5       // Maximum custom Meshtastic channels

// Meshtastic EU868 LongFast radio parameters
#define MESHTASTIC_FREQ_EU868         869.525f
#define MESHTASTIC_BW                 250.0f
#define MESHTASTIC_SF                 11
#define MESHTASTIC_CR                 5
#define MESHTASTIC_SYNC_WORD          0x2B
#define MESHTASTIC_PREAMBLE           16
#define MESHTASTIC_POWER              14

// MeshCore EU868 radio parameters (regional default)
#define MESHCORE_FREQ_EU868           869.618f
#define MESHCORE_BW                   62.5f
#define MESHCORE_SF                   8
#define MESHCORE_CR                   8
#define MESHCORE_SYNC_WORD            0x12    // Private sync word
#define MESHCORE_PREAMBLE             8
#define MESHCORE_POWER                14

// Meshtastic port numbers
#define MESHTASTIC_PORTNUM_POSITION   3
#define MESHTASTIC_PORTNUM_NODEINFO   4
#define MESHTASTIC_PORTNUM_TEXT       1

// MeshCore packet types (header bits 2-5)
#define MESHCORE_PAYLOAD_ADVERT       0x04    // Advertisement packet

// Channel name/key limits
#define MESH_CHANNEL_NAME_LEN         24      // Max channel name length
#define MESH_CHANNEL_KEY_LEN          32      // Max base64 key length (before expansion)

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------

// Scanner operating modes
enum MeshtasticScannerMode {
  SCANNER_OFF = 0,
  SCANNER_MESHTASTIC = 1,
  SCANNER_MESHCORE = 2,      // Future
  SCANNER_AUTO = 3           // Cycles through enabled protocols
};

// Scanner state machine states
enum MeshtasticScannerState {
  SCAN_STATE_IDLE = 0,
  SCAN_STATE_SWITCHING_MESHTASTIC,
  SCAN_STATE_LISTENING_MESHTASTIC,
  SCAN_STATE_SWITCHING_MESHCORE,
  SCAN_STATE_LISTENING_MESHCORE,
  SCAN_STATE_RESTORING
};

// External peer source
enum ExternalPeerSource {
  PEER_SOURCE_MESHTASTIC = 0,
  PEER_SOURCE_MESHCORE = 1
};

// External peer from Meshtastic/MeshCore network
struct MeshtasticPeer {
  uint32_t nodeId;           // Node ID (from header)
  char     shortName[12];    // Node name if available, else "!AABBCCDD"
  double   lat;
  double   lon;
  float    alt;
  uint32_t lastSeenMs;       // millis() timestamp of last packet
  int16_t  rssi;
  int8_t   snr;
  bool     valid;            // Slot in use
  ExternalPeerSource source; // Which protocol this peer came from
  int8_t   channelIdx;       // Which channel decoded this peer (-1 = default)
};

// Custom Meshtastic channel
struct MeshtasticChannel {
  char     name[MESH_CHANNEL_NAME_LEN];   // User-friendly name
  char     pskBase64[MESH_CHANNEL_KEY_LEN]; // Base64-encoded PSK
  uint8_t  expandedKey[32];               // Expanded key (16 or 32 bytes)
  uint8_t  keyLen;                        // 16 or 32 bytes
  bool     enabled;                       // Channel active
  uint32_t packetCount;                   // Packets decoded on this channel
};

// Saved LoRa configuration (to restore after scan)
struct LoraConfigBackup {
  float    frequency;
  float    bandwidth;
  uint8_t  spreadingFactor;
  uint8_t  codingRate;
  uint8_t  syncWord;
  int8_t   power;
  uint16_t preamble;
};

// -----------------------------------------------------------------------------
// Azimuth LoRa Config — set by main firmware before scanner init
// -----------------------------------------------------------------------------
extern float    azimuthLoraFreq;
extern float    azimuthLoraBW;
extern uint8_t  azimuthLoraSF;
extern uint8_t  azimuthLoraCR;
extern int8_t   azimuthLoraPwr;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

// Initialize scanner (call once in setup())
void meshtasticScannerInit();

// Main loop function (call every loop iteration)
void meshtasticScannerLoop();

// Enable/disable scanner
void meshtasticScannerSetMode(MeshtasticScannerMode mode);
MeshtasticScannerMode meshtasticScannerGetMode();

// Check if currently scanning (radio config changed)
bool meshtasticScannerIsActive();

// Get peer list for display
// Returns number of valid peers copied to output array
int meshtasticGetPeers(MeshtasticPeer* out, int maxPeers);

// Get peer count
int meshtasticGetPeerCount();

// Clear all external peers
void meshtasticClearPeers();

// Force immediate scan (for testing)
void meshtasticForceScan();

// -----------------------------------------------------------------------------
// Custom Channel Management
// -----------------------------------------------------------------------------

// Add or update a channel (returns channel index, or -1 on failure)
int meshtasticAddChannel(const char* name, const char* pskBase64);

// Remove a channel by index
bool meshtasticRemoveChannel(int idx);

// Enable/disable a channel
bool meshtasticSetChannelEnabled(int idx, bool enabled);

// Get channel list
int meshtasticGetChannels(MeshtasticChannel* out, int maxChannels);

// Get channel count (enabled channels only)
int meshtasticGetChannelCount();

// Clear all custom channels
void meshtasticClearChannels();

// Include default "AQ==" channel in decryption attempts
extern bool meshtasticIncludeDefaultChannel;

// Parse Meshtastic share URL (meshtastic.org/e/#...) and extract channel info
// Returns true if valid, populates name and pskBase64
bool meshtasticParseShareUrl(const char* url, char* nameOut, int nameMax, char* pskOut, int pskMax);

// -----------------------------------------------------------------------------
// Statistics
// -----------------------------------------------------------------------------

struct MeshtasticStats {
  uint32_t scanCount;              // Total scans performed
  uint32_t meshtasticPackets;      // Meshtastic packets received
  uint32_t meshtasticDecoded;      // Successfully decoded Meshtastic positions
  uint32_t meshcorePackets;        // MeshCore packets received
  uint32_t meshcoreDecoded;        // Successfully decoded MeshCore adverts
  uint32_t decryptFails;           // Decryption failures
  uint32_t parseFails;             // Parse failures
  uint32_t lastScanMs;             // Timestamp of last scan
};

const MeshtasticStats& meshtasticGetStats();

#endif // MESHTASTIC_SCANNER_H
