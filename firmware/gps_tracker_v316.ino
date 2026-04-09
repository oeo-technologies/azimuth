/*
 * Azimuth GPS Tracker Firmware — OEO Technologies
 * =====================================================================
 * Version : v3.16
 * Changes :
 *   v3.16 — Mesh status, config portal, lockdown mode, WiFi QR code
 *          New /status endpoint: self-contained HTML status page showing
 *          device info, battery, GPS, uptime, LoRa status, peer table with
 *          name, distance, last seen, RSSI/SNR signal bars, battery.
 *          New /config endpoint: full device configuration via browser.
 *          Settings: name, speed/elevation units, timeout, theme, LoRa
 *          region, broadcast interval, relay mode. No BLE required.
 *          New /api/mesh endpoint: JSON API for device + peer data.
 *          Lockdown mode: Settings > Lock blocks all remote config (BLE/WiFi).
 *          Unlock via OLED Settings page. SOS/messages still allowed when locked.
 *          WiFi QR code: after saving settings with Hotspot on, displays QR code
 *          that phones can scan to auto-join the device AP. Stays on screen
 *          (ignores timeout) until button press dismisses it.
 *          LoraPeer struct extended with rssi/snr fields.
 *          RSSI/SNR captured from RadioLib on packet receive.
 *   v3.15 — Relay mode and store-and-forward messaging
 *          Optional relay mode (Settings > Relay) forwards packets for
 *          other devices — useful when stationary at checkpoint/base.
 *          Relays position broadcasts, SOS, and group messages.
 *          Deduplication via packet hashing (30s window).
 *          Random delay (50-300ms) before relay to avoid collisions.
 *          Store-and-forward queues messages for offline peers (max 8,
 *          1hr expiry) and delivers when peer broadcasts position.
 *          Relay count tracked per session.
 *   v3.14 — Animated splash: mountains scroll up, AZIMUTH wipes down
 *   v313 — On-device Groups page with stored groups
 *          New OLED Groups page shows stored groups, pending requests.
 *          Groups stored in NVS (max 5) with name, key, leader info.
 *          Key share packet now includes group name (71 bytes).
 *          Button interactions: Short=cycle, Long=select/share, Dbl=delete.
 *          New command: SETGROUPNAME:name (set before KEYSHARE).
 *          Accept saves group to device, activates it automatically.
 *   v312 — Key share consent flow
 *          Receiving device shows "Join [leader]'s network?" overlay.
 *          Long press = accept (saves key, sends 0xB9 ACK), single = decline.
 *          Leader can broadcast 0xB8 cancel to clear pending requests.
 *          Updated 0xB7 packet includes leader name (39 bytes).
 *          Web app notified of pending requests and ACKs.
 *          New commands: KEYACCEPT, KEYDECLINE.
 *   v311 — Group tracking system for events and team coordination
 *          Groups stored in FFat (/groups.json) with templates:
 *          general, fell_race, sar. Each group has own network key.
 *          Key share broadcast (0xB7, UNENCRYPTED) for device onboarding.
 *          New commands: KEYSHARE, GETGROUPS, SETGROUPS, SETACTIVEGROUP
 *          WiFi API: /api/groups endpoint, cmd handlers for all above.
 *          Supports offline operation — CSV roster stored on device.
 *   v310 — Device registration protocol for fell race / event scenarios
 *          Leader broadcasts 0xB2 discover, devices respond 0xB3 with
 *          random 50-550ms backoff to avoid collisions.
 *          Leader assigns names via 0xB5, devices ACK with 0xB6.
 *            New BLE commands: REGDISCOVER, REGASSIGN:DEVICEID:Name
 *            WiFi API mirrors BLE commands for AP mode operation.
 *            Supports naming 50+ hire devices in under 60 seconds.
 *   v3.0.9 — Shared waypoint system (wayfinding mode)
 *            Web app pushes up to 10 named waypoints via BLE (SETWP JSON).
 *            Device broadcasts waypoints to all peers over LoRa (0xAF packets).
 *            Receiving devices store and persist waypoints to FFat (/waypoints.bin).
 *            Navigate page extended: long press cycles through waypoints,
 *            compass needle and distance update per waypoint. Shows wp name + N/M.
 *            Route Log page: long press now clears all saved routes from device.
 *            BLE confirms waypoint receipt with {"wps":N} notify.
 *            waypointLoad/Save/Clear functions mirror route persistence pattern.
 *   v3.0.8 — Route log persistence (FFat filesystem)
 *            Route log entries now survive reboots and power cycles.
 *            Saved to /routes/route_N.bin on FFat (9.9MB partition).
 *            Format-on-first-use with [FFAT] serial diagnostics.
 *            routeLogSave() writes to flash; routeLogClear() removes files.
 *            All logs loaded back into heap on boot via routeLogLoad().
 *            Weather fix: fetchWeather() now runs on approximate GPS fix
 *            (IP geolocation) — no longer requires a real GPS fix.
 *            Acquiring screen: map pin graphic with signal arcs replacing
 *            satellite icon. Battery indicator shifted 4px right.
 *            Splash screen: Azimuth bearing mark (Option B) — degree ring,
 *            219 SW bearing line, dashed arc, AZIMUTH wordmark. Splash
 *            still being refined — on the list for further tweaks.
 *   v3.0.7 — Peer limit raised to 20 (LORA_MAX_PEERS).
 *            Broadcast message repeat x3 with 5s gaps for reliability.
 *            SOS function: double press Power page broadcasts 0xAE packet
 *            6x at 4s intervals, rapid LED flash, OLED status screen.
 *            Receiving devices flash LED 8x and notify BLE with SOS JSON.
 *            Stale peer alert: OLED flash + BLE notify after 5 min silence.
 *            TX power maximised: US/AU raised to 22dBm (SX1262 hardware max).
 *            EU868 remains at 14dBm (regulatory maximum).
 *            PIN overlay on long press page 0 (GPS/Acquiring).
 *   v3.0.6 — TX power maximised for US/AU regions.
 *   v3.0.5 — Acquiring screen animation, bat=255 USB sentinel,
 *            stats overlay battery, peer 0,0 fix.
 *   v3.0.4 — Route export UI, buildSettingsJSON with logCount+rec.
 *   v3.0.3 — Peer distance line, follow mode, peer track, ETA, bearing arrow.
 *   v3.0.2 — AES-128 encryption, peer sharing, extended position packet.
 *   v3.0.1 — Page reordering, conditional pages system.
 *   v3.0.0 — LoRa crash fix (RadioLib include inside ENABLE_LORA guard).
 * =====================================================================
 * Board   : Heltec WiFi LoRa 32(V4)
 * GPS     : AT6558R via Serial1 RX=39 TX=38
 * Display : Onboard SSD1306 128x64 OLED SDA=17 SCL=18 RST=21
 * BLE     : NimBLE-Arduino 1.4.2
 * LoRa    : RadioLib (SX1262)
 *
 * Libraries (Arduino Library Manager):
 *  1. Heltec ESP32 Dev-Boards  (Heltec Automation) — 3.0.3
 *  2. Adafruit SSD1306         (Adafruit)
 *  3. Adafruit GFX Library     (Adafruit)
 *  4. TinyGPSPlus              (Mikal Hart)
 *  5. NimBLE-Arduino 1.4.2     (h2zero)
 *  6. RadioLib                 (jgromes) — 6.6.0
 *
 * Board settings:
 *  Board           : WiFi LoRa 32(V4)
 *  USB CDC On Boot : Enabled
 *  Flash Size      : 16MB
 *  Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
 *  Upload Speed    : 921600
 */

// ============================================================
// INCLUDES
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansOblique12pt7b.h>
#include <TinyGPSPlus.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <FFat.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
// QR code generation embedded — no external library needed
#include "webapp_gz.h"  // gzipped web app embedded in firmware

// LoRa SPI bus — instantiated inside loraBegin() to avoid global constructor issues

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define VEXT_CTRL_PIN   36
#define VGNSS_CTRL_PIN  34
#define OLED_SDA        17
#define OLED_SCL        18
#define OLED_RST        21
#define OLED_ADDRESS    0x3C
#define SCREEN_W        128
#define SCREEN_H        64
#define BAT_ADC_PIN     1
#define BAT_CTRL_PIN    37
#define BAT_DIVIDER     0.204
#define BAT_ADC_REF     3.3
#define BAT_ADC_RES     4095
#define BAT_MAX_V       4.2
#define BAT_MIN_V       3.3
#define BAT_READ_MS     30000
#define BAT_NO_BAT_V    3.80  // charger IC idle rail ~3.85V — above this = no battery
#define BAT_FULL_V      4.15  // above this on USB = real battery fully charged

#define LED_PIN         35    // White LED, active HIGH
#define LED_FLASH_MS    100   // flash duration ms
#define LED_INTERVAL_MS 10000 // flash every 10s when recording

#define BTN_PIN         0
#define BTN_LONG_MS     1000
#define BTN_POWER_MS    3000
#define OLED_TIMEOUT_MS 10000

// OLED layout constants - nav dots at y=58, so content must stay above
#define OLED_CONTENT_MAX_Y  56  // Last safe y for content (nav dots start at 58)
#define OLED_HINT_Y         46  // Standard y position for hint text

#define GPS_RX_PIN      39  // GPS module TX -> ESP32 RX (confirmed by Meshtastic)
#define GPS_TX_PIN      38  // ESP32 TX -> GPS module RX
#define GPS_BAUD        9600

// ============================================================
// BLE
// ============================================================
// Device name — generated on first boot, stored in NVS
char bleDeviceName[32] = "Azimuth";
#define BLE_SERVICE_UUID     "12345678-1234-1234-1234-123456789abc"
#define BLE_GPS_CHAR_UUID    "12345678-1234-1234-1234-123456789abd"
#define BLE_ROUTE_CHAR_UUID  "12345678-1234-1234-1234-123456789abe"
#define BLE_SCRN_CHAR_UUID   "12345678-1234-1234-1234-123456789abf"
#define BLE_SET_CHAR_UUID    "12345678-1234-1234-1234-123456789ac0"
#define BLE_SCAN_CHAR_UUID   "12345678-1234-1234-1234-123456789ac1"
#define BLE_RES_CHAR_UUID    "12345678-1234-1234-1234-123456789ac2"  // read-only results
#define BLE_PEER_CHAR_UUID   "12345678-1234-1234-1234-123456789ac3"  // LoRa peer notify

// ============================================================
// LORA — set to 1 to enable, 0 to disable (safe mode)
// ============================================================
#define ENABLE_LORA  1  // set to 1 to enable LoRa

#if ENABLE_LORA
#include <RadioLib.h>
#include <SPI.h>

// Regional presets: { frequency MHz, SF, BW kHz, CR denominator, maxPower dBm }
// EU868: 14 dBm EIRP is the regulatory maximum under ETSI EN 300 220 (1% duty cycle)
// US915: FCC Part 15.247 allows up to 30 dBm EIRP; SX1262 hardware max is 22 dBm
// AU915: ACMA limit matches US915; SX1262 hardware max is 22 dBm
struct LoRaRegionPreset {
  const char* name;
  float       freqMHz;
  uint8_t     sf;
  float       bwKHz;
  uint8_t     cr;
  int8_t      power;
};
static const LoRaRegionPreset LORA_REGIONS[] = {
  { "EU868",  868.1f, 9, 125.0f, 5, 14 },  // 14 dBm — regulatory max for EU868
  { "US915",  910.5f, 7,  62.5f, 5, 22 },  // 22 dBm — SX1262 hardware max (within FCC limit)
  { "AU915",  916.8f, 7,  62.5f, 5, 22 },  // 22 dBm — SX1262 hardware max (within ACMA limit)
};
static const int LORA_REGION_COUNT = 3;

#define LORA_MAGIC          0xA9  // position broadcast
#define LORA_MAGIC_MSG_BC   0xAB  // broadcast message (to all)
#define LORA_MAGIC_MSG_DM   0xAC  // direct message (to specific device)
#define LORA_MAX_PEERS      20
#define LORA_MAX_MSGS       5
#define LORA_MSG_MAX_LEN    48
#define LORA_PEER_EXPIRE_MS 300000UL
#define LORA_MIN_INTERVAL_S 10
#define LORA_MAX_INTERVAL_S 300
#define LORA_KEY_LEN        16
#define LORA_NONCE_LEN      4   // bytes prepended to encrypted payload
// Default network key — all Azimuth devices share this unless changed
static const uint8_t LORA_DEFAULT_KEY[LORA_KEY_LEN] = {
  'A','z','i','m','u','t','h','N','e','t','w','o','r','k','0','1'
};
#include <mbedtls/aes.h>

struct LoraPeer {
  char     id[8];
  char     name[24];
  double   lat, lon;
  float    alt;
  uint16_t hdg;
  uint8_t  spd;
  uint8_t  bat;
  uint32_t lastSeenMs;
  bool     active;
  // Extended fields
  uint8_t  status;       // bit0=recording, bit1=waypointSaved, bit2=gpsFix
  uint16_t routePts;     // number of route points recorded
  uint32_t routeDistM;   // route distance in metres
  double   wpLat, wpLon; // peer's saved waypoint (valid if status bit1 set)
  // Signal quality (updated on each received packet)
  int16_t  rssi;         // RSSI in dBm (typically -120 to 0)
  int8_t   snr;          // SNR in dB (typically -20 to +10)
};

struct LoraMsg {
  char     fromId[8];
  char     fromName[24];
  char     text[LORA_MSG_MAX_LEN + 1];
  uint32_t rxMs;
  bool     read;
  bool     active;
};
#endif // ENABLE_LORA

// ============================================================
// SIMULATION MODE — comment out when real GPS arrives
// ============================================================
// // #define SIMULATE_GPS  // uncomment to simulate GPS data for testing

// ============================================================
// GLOBALS
// ============================================================
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RST);
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(1);
Preferences      prefs;

NimBLEServer*         pServer   = nullptr;
NimBLECharacteristic* pGpsChar  = nullptr;
NimBLECharacteristic* pRouteChar = nullptr;
NimBLECharacteristic* pScrnChar  = nullptr;
NimBLECharacteristic* pSetChar   = nullptr;
NimBLECharacteristic* pScanChar  = nullptr;  // WiFi scan results (read-only)
NimBLECharacteristic* pResChar   = nullptr;  // Settings/name results (read-only)
NimBLECharacteristic* pPeerChar  = nullptr;  // LoRa peer notify (read-only)
bool bleConnected       = false;
bool blePushLogPending  = false;  // push route log on next loop
bool weatherReceived    = false;  // true once weather data is valid (WiFi or LoRa push)
bool apEnabled          = false;  // true when AP mode is active
bool clientSubscribed = false;

// LoRa globals
#if ENABLE_LORA
SPIClass* loraSPI = nullptr;
SX1262*   radio   = nullptr;
bool     loraReady        = false;
uint8_t  loraRegionIdx    = 0;
uint16_t loraIntervalSec  = 30;
uint32_t lastLoraTxMs     = 0;

// Message repeat — broadcast sent 3× with 5s gaps
uint8_t  msgRepeatCount   = 0;      // how many more repeats to send
uint32_t msgRepeatNextMs  = 0;      // when to send next repeat
uint8_t  msgRepeatBuf[80];          // copy of packet to repeat
int      msgRepeatLen     = 0;

// SOS state
bool     sosActive        = false;   // true while SOS is broadcasting
uint32_t sosStartMs       = 0;
uint8_t  sosBroadcastCount = 0;      // repeats sent so far
uint32_t sosNextTxMs      = 0;
#define  SOS_REPEAT_COUNT  6         // send SOS packet 6× total
#define  SOS_REPEAT_GAP_MS 4000      // 4s between SOS repeats
#define  SOS_LED_FLASH_MS  80        // rapid LED flash rate during SOS
#define  LORA_MAGIC_SOS    0xAE      // SOS magic byte
#define  LORA_MAGIC_WP     0xAF      // waypoint broadcast
#define  LORA_MAGIC_WP_ACK 0xB0      // waypoint acknowledgement
#define  LORA_MAGIC_WP_CLR 0xB1      // waypoint clear broadcast
#define  LORA_MAGIC_REG_DISCOVER  0xB2  // registration: "who's there?" broadcast
#define  LORA_MAGIC_REG_RESPONSE  0xB3  // registration: device response with ID+name
#define  LORA_MAGIC_WEATHER 0xB4        // weather push to specific device
#define  LORA_MAGIC_REG_ASSIGN    0xB5  // registration: assign name to specific device
#define  LORA_MAGIC_REG_ACK       0xB6  // registration: device confirms name assignment
#define  LORA_MAGIC_KEY_SHARE     0xB7  // key share request (UNENCRYPTED, includes leader name)
#define  LORA_MAGIC_KEY_CANCEL    0xB8  // key share cancel (UNENCRYPTED)
#define  LORA_MAGIC_KEY_ACK       0xB9  // key share accepted ACK (encrypted)
#define  LORA_MAGIC_GROUP_LEAVE   0xBA  // device leaving group (encrypted, sent before key change)

// ── Repeater / Relay settings ───────────────────────────────────
#define RELAY_MAX_SEEN      32    // track recently seen packet hashes to prevent loops
#define RELAY_SEEN_EXPIRE   30000 // ms to remember seen packets (30s)
#define RELAY_DELAY_MIN     50    // min random delay before relay (ms)
#define RELAY_DELAY_MAX     300   // max random delay before relay (ms)
#define RELAY_MAX_HOPS      3     // maximum relay hops

// ── Store-and-forward settings ──────────────────────────────────
#define STORE_FWD_MAX       8     // max pending messages to store
#define STORE_FWD_EXPIRE    3600000 // expire after 1 hour (ms)

// ── Waypoint system ──────────────────────────────────────────
#define WP_MAX       10
#define WP_NAME_LEN  12

struct Waypoint {
  float lat, lon;
  char  name[WP_NAME_LEN + 1];
};

// ── Group tracking system ────────────────────────────────────
// Groups stored in FFat as /groups.json
// Max limits to bound memory usage
#define GROUP_MAX           10   // max groups
#define GROUP_DEVICE_MAX    50   // max devices per group
#define GROUP_NAME_LEN      32   // max group name length
#define GROUP_DEVICE_NAME   20   // max device name length
#define GROUP_ROLE_LEN      12   // max role string length
#define GROUP_ID_LEN        12   // group id length

struct GroupDevice {
  char id[8];                    // device LoRa ID (6 hex chars + null)
  char name[GROUP_DEVICE_NAME];  // assigned name
  char role[GROUP_ROLE_LEN];     // role: runner, marshal, sweep, etc
};

struct Group {
  char id[GROUP_ID_LEN];         // unique group id
  char name[GROUP_NAME_LEN];     // display name
  char key[LORA_KEY_LEN + 1];    // network key (16 chars + null)
  char templ[16];                // template: general, fell_race, sar
  bool rolesEnabled;
  bool overdueEnabled;
  uint16_t overdueThresholdMins;
  uint8_t deviceCount;
  GroupDevice devices[GROUP_DEVICE_MAX];
};

Group    groups[GROUP_MAX];
int      groupCount     = 0;
int      activeGroupIdx = -1;  // -1 = personal mode (default key)

// ── Pending key share request (consent flow) ────────────────────────────
// When a key share (0xB7) is received, store it here for user approval
struct PendingKeyShare {
  bool     active;              // true if request pending
  char     fromId[8];           // leader device ID
  char     leaderName[20];      // leader device name (shown on OLED)
  char     groupName[32];       // group name
  uint8_t  key[LORA_KEY_LEN];   // proposed network key
  uint32_t receivedMs;          // when received (for potential timeout)
};
PendingKeyShare pendingKeyShare = {false};

// Key share state (leader side)
bool     keyShareActive   = false;  // true after sending key share, until cancelled
uint8_t  keyShareAckCount = 0;      // count of devices that accepted
uint32_t keyShareSentMs   = 0;      // when key share was sent
char     keyShareGroupName[32] = "";  // group name to include in broadcast

// ── Stored groups (on-device, persisted to NVS) ────────────────────────────
// These are groups the device knows about (joined via key share or web app)
#define MAX_STORED_GROUPS 5
struct StoredGroup {
  char     name[32];       // Group name
  uint8_t  key[LORA_KEY_LEN]; // Network key
  char     leaderName[20]; // Who shared the key (empty if we created it)
  bool     isLeader;       // Did we create this group?
  bool     canLeave;       // Can members leave? (fell race can lock this)
};
StoredGroup storedGroups[MAX_STORED_GROUPS];
int  storedGroupCount = 0;
int  storedGroupSel   = 0;      // Currently selected in list (0 = Personal)
int  activeStoredGroup = -1;    // Which is active (-1 = personal/default key)
bool groupMenuOpen = false;     // Sub-menu open for selected group
int  groupMenuSel  = 0;         // Selected menu item (0=Activate, 1=Share, 2=Leave, 3=Back)
char originalDeviceName[20] = ""; // Stored when joining fell race group

Waypoint waypoints[WP_MAX];
int      waypointCount   = 0;
int      waypointCurrent = 0;  // which waypoint Navigate is showing
volatile bool loraRxFlag  = false;
char     loraDeviceId[8]  = "";
LoraPeer loraPeers[LORA_MAX_PEERS];
int      loraPeerCount    = 0;
LoraMsg  loraMsgs[LORA_MAX_MSGS];
int      loraMsgCount     = 0;
int      loraMsgUnread    = 0;
int      loraMsgScroll    = 0;  // scroll offset for message viewing
uint8_t  loraKey[LORA_KEY_LEN]; // AES-128 network key
void IRAM_ATTR loraRxISR() { loraRxFlag = true; }

// ── Lockdown mode ──────────────────────────────────────────────
// When enabled, blocks all remote config (BLE/WiFi) — requires OLED unlock
bool     deviceLocked     = false;

// ── Relay mode state ────────────────────────────────────────────
bool     relayEnabled     = false;  // user setting — relay packets for others
uint32_t relaySeenHashes[RELAY_MAX_SEEN];  // hashes of recently seen packets
uint32_t relaySeenTimes[RELAY_MAX_SEEN];   // when each was seen
int      relaySeenCount   = 0;
uint32_t relayLastTx      = 0;  // last relay transmission time
uint16_t relayCount       = 0;  // packets relayed this session

// Last received packet signal quality (updated before loraHandlePacket)
int16_t  loraLastRssi     = 0;  // RSSI in dBm
int8_t   loraLastSnr      = 0;  // SNR in dB

// ── Store-and-forward state ─────────────────────────────────────
struct StoredMsg {
  bool     active;
  char     toId[8];       // destination device ID
  char     fromId[8];     // source device ID
  char     fromName[20];  // source name
  char     text[LORA_MSG_MAX_LEN + 1];
  uint32_t storedAt;      // millis() when stored
  uint8_t  attempts;      // delivery attempts
};
StoredMsg storedMsgs[STORE_FWD_MAX];
int       storedMsgCount  = 0;
#else
uint8_t  loraRegionIdx    = 0;
uint16_t loraIntervalSec  = 30;
#endif

// UI state
// Base/Nav counts depend on routeLogCount too — computed in pageCount()
#define PAGE_COUNT_BASE  10  // +1 for settings page — wifi page added dynamically
int      currentPage    = 0;
bool     screenOn       = true;
uint32_t lastActivityMs = 0;
bool     btnLastState   = HIGH;
uint32_t btnPressedMs   = 0;
bool     btnLongFired   = false;
bool     btnPendingSingle  = false; // waiting to see if double press follows
uint32_t btnPendingMs      = 0;
#define  BTN_DOUBLE_MS  500       // window for double press (ms)


// Saved location
bool   locSaved = false;
double savedLat = 0.0, savedLon = 0.0;

// Last known position — saved to NVS for warm start
double   lastKnownLat  = 0.0;
double   lastKnownLon  = 0.0;
float    lastKnownAlt  = 0.0;
uint32_t lastPosSaveMs = 0;
#define  POS_SAVE_INTERVAL_MS 30000  // save position every 30s when fixed

// Cached GPS
bool     cachedFix  = false;
bool     cachedFixIsApprox = false;  // true = IP geolocation, false = real GPS
float    cachedApproxAccM  = 5000.0f; // accuracy radius in metres for approx position
double   cachedLat  = 53.259171, cachedLon = -2.518049;
double   cachedSpd  = 0.0, cachedHdg = 0.0;
float    cachedAlt  = 0.0;
uint32_t cachedSats = 0;

// Timing
uint32_t lastGpsUpdate  = 0;
uint32_t lastOledUpdate = 0;
uint32_t lastBleNotify  = 0;
uint32_t lastBatRead    = 0;

// Battery
float    batVoltage  = 0.0;
int      batPercent  = 0;

// Timezone
int8_t   tzOffset    = 0;
bool     tzAdjustMode = false;

// Route recording
#define ROUTE_INTERVAL_MS  5000
#define ROUTE_MAX_POINTS   3600
struct RoutePoint { double lat, lon; float spd, alt; uint32_t ms; };
RoutePoint* routePoints    = nullptr;
int         routeCount     = 0;
bool        routeRecording = false;
uint32_t    lastRouteMs    = 0;
uint32_t    routeStartMs   = 0;
float       routeTotalDist = 0.0;

// Route log — stores completed routes
#define ROUTE_LOG_MAX  10

struct RouteLog {
  RoutePoint* points;
  int         count;
  float       distM;
};

RouteLog  routeLog[ROUTE_LOG_MAX];
int       routeLogCount  = 0;
bool      logClearWarn   = false;  // showing "route in progress" warning

// Save current route buffer into log
void routeLogSave() {
  if (routeCount == 0) return;

  // If log is full, free oldest and shift
  if (routeLogCount >= ROUTE_LOG_MAX) {
    free(routeLog[0].points);
    for (int i = 0; i < ROUTE_LOG_MAX - 1; i++)
      routeLog[i] = routeLog[i + 1];
    routeLogCount--;
    Serial.println("[LOG] Oldest route dropped to make room");
  }

  // Allocate and copy points
  RoutePoint* pts = (RoutePoint*)malloc(routeCount * sizeof(RoutePoint));
  if (!pts) { Serial.println("[LOG] malloc failed"); return; }
  memcpy(pts, routePoints, routeCount * sizeof(RoutePoint));

  routeLog[routeLogCount].points = pts;
  routeLog[routeLogCount].count  = routeCount;
  routeLog[routeLogCount].distM  = routeTotalDist;
  routeLogCount++;

  Serial.printf("[LOG] Route %d saved: %d pts, %.0fm\n",
                routeLogCount, routeCount, routeTotalDist);

  // Persist all routes to FFat so they survive reboot
  routeLogFlushAll();
}

// Clear all logged routes and free memory
void routeLogClear() {
  for (int i = 0; i < routeLogCount; i++)
    if (routeLog[i].points) { free(routeLog[i].points); routeLog[i].points = nullptr; }
  routeLogCount = 0;
  logClearWarn  = false;
  Serial.println("[LOG] All routes cleared");
  // Remove all route files from flash
  if (FFat.begin(false)) {
    for (int i = 0; i < ROUTE_LOG_MAX; i++) {
      char path[32];
      snprintf(path, sizeof(path), "/route_%d.bin", i);
      if (FFat.exists(path)) FFat.remove(path);
    }
    Serial.println("[FFAT] Route files deleted");
  }
}

// ── FFat route persistence ────────────────────────────────────────────────────
// File format per route: [count:4][distM:4][RoutePoint * count]
// Files: /route_0.bin .. /route_9.bin

void routeLogFlushAll() {
  // Write all in-memory route logs to FFat
  if (!FFat.begin(false)) {
    Serial.println("[FFAT] Mount failed — routes not saved");
    return;
  }
  for (int i = 0; i < routeLogCount; i++) {
    char path[32];
    snprintf(path, sizeof(path), "/route_%d.bin", i);
    File f = FFat.open(path, FILE_WRITE);
    if (!f) { Serial.printf("[FFAT] Cannot write %s\n", path); continue; }
    f.write((uint8_t*)&routeLog[i].count, 4);
    f.write((uint8_t*)&routeLog[i].distM, 4);
    f.write((uint8_t*)routeLog[i].points, routeLog[i].count * sizeof(RoutePoint));
    f.close();
    Serial.printf("[FFAT] Saved %s (%d pts, %.0fm)\n", path, routeLog[i].count, routeLog[i].distM);
  }
}

void routeLogLoad() {
  // Format FFat if this is first use
  if (!FFat.begin(false)) {
    Serial.println("[FFAT] Formatting FAT partition (first use)...");
    if (!FFat.begin(true)) {
      Serial.println("[FFAT] Format failed — route persistence disabled");
      return;
    }
    Serial.println("[FFAT] Formatted OK");
  }
  Serial.printf("[FFAT] Mounted — %.0f KB free\n", FFat.freeBytes() / 1024.0f);

  // Load each route file back into routeLog[]
  routeLogCount = 0;
  for (int i = 0; i < ROUTE_LOG_MAX; i++) {
    char path[32];
    snprintf(path, sizeof(path), "/route_%d.bin", i);
    if (!FFat.exists(path)) continue;
    File f = FFat.open(path, FILE_READ);
    if (!f) continue;
    int   count; float distM;
    if (f.read((uint8_t*)&count, 4) != 4 || f.read((uint8_t*)&distM, 4) != 4 || count <= 0 || count > ROUTE_MAX_POINTS) {
      f.close(); FFat.remove(path); continue;  // corrupt file
    }
    RoutePoint* pts = (RoutePoint*)malloc(count * sizeof(RoutePoint));
    if (!pts) { f.close(); continue; }
    if ((int)f.read((uint8_t*)pts, count * sizeof(RoutePoint)) != count * (int)sizeof(RoutePoint)) {
      free(pts); f.close(); FFat.remove(path); continue;
    }
    f.close();
    routeLog[routeLogCount].points = pts;
    routeLog[routeLogCount].count  = count;
    routeLog[routeLogCount].distM  = distM;
    routeLogCount++;
    Serial.printf("[FFAT] Loaded %s (%d pts, %.0fm)\n", path, count, distM);
  }
  Serial.printf("[FFAT] %d route(s) restored\n", routeLogCount);
}

// ── FFat waypoint persistence ─────────────────────────────────────────────────
// File: /waypoints.bin — [count:1][Waypoint × count]

void waypointSave() {
  if (!FFat.begin(false)) return;
  File f = FFat.open("/waypoints.bin", FILE_WRITE);
  if (!f) { Serial.println("[FFAT] Cannot write /waypoints.bin"); return; }
  uint8_t cnt = (uint8_t)waypointCount;
  f.write(&cnt, 1);
  f.write((uint8_t*)waypoints, waypointCount * sizeof(Waypoint));
  f.close();
  Serial.printf("[FFAT] Saved %d waypoint(s)\n", waypointCount);
}

void waypointLoad() {
  if (!FFat.begin(false)) return;
  if (!FFat.exists("/waypoints.bin")) return;
  File f = FFat.open("/waypoints.bin", FILE_READ);
  if (!f) return;
  uint8_t cnt = 0;
  if (f.read(&cnt, 1) != 1 || cnt > WP_MAX) { f.close(); return; }
  if ((int)f.read((uint8_t*)waypoints, cnt * sizeof(Waypoint)) != cnt * (int)sizeof(Waypoint)) {
    f.close(); waypointCount = 0; return;
  }
  f.close();
  waypointCount   = cnt;
  waypointCurrent = 0;
  Serial.printf("[FFAT] Loaded %d waypoint(s)\n", waypointCount);
}

void waypointClear() {
  waypointCount   = 0;
  waypointCurrent = 0;
  memset(waypoints, 0, sizeof(waypoints));
  if (FFat.begin(false)) {
    if (FFat.exists("/waypoints.bin")) FFat.remove("/waypoints.bin");
  }
  Serial.println("[WP] Waypoints cleared");
}

// ── FFat group persistence ────────────────────────────────────────────────────
// File: /groups.json — JSON format for easy web app interop
// Called on boot, after group changes, and when web app syncs

// Forward declaration — loraSaveKey defined later in LoRa section
#if ENABLE_LORA
void loraSaveKey(const uint8_t* key);
#endif

void groupsSave() {
  if (!FFat.begin(false)) {
    Serial.println("[FFAT] Cannot mount for groups save");
    return;
  }
  
  // Build JSON document
  JsonDocument doc;
  doc["activeGroupIdx"] = activeGroupIdx;
  JsonArray groupsArr = doc["groups"].to<JsonArray>();
  
  for (int g = 0; g < groupCount; g++) {
    JsonObject grp = groupsArr.add<JsonObject>();
    grp["id"] = groups[g].id;
    grp["name"] = groups[g].name;
    grp["key"] = groups[g].key;
    grp["templ"] = groups[g].templ;
    grp["rolesEnabled"] = groups[g].rolesEnabled;
    grp["overdueEnabled"] = groups[g].overdueEnabled;
    grp["overdueThresholdMins"] = groups[g].overdueThresholdMins;
    
    JsonArray devArr = grp["devices"].to<JsonArray>();
    for (int d = 0; d < groups[g].deviceCount; d++) {
      JsonObject dev = devArr.add<JsonObject>();
      dev["id"] = groups[g].devices[d].id;
      dev["name"] = groups[g].devices[d].name;
      dev["role"] = groups[g].devices[d].role;
    }
  }
  
  File f = FFat.open("/groups.json", FILE_WRITE);
  if (!f) {
    Serial.println("[FFAT] Cannot write /groups.json");
    return;
  }
  serializeJson(doc, f);
  f.close();
  Serial.printf("[FFAT] Saved %d group(s)\n", groupCount);
}

void groupsLoad() {
  if (!FFat.begin(false)) return;
  if (!FFat.exists("/groups.json")) {
    Serial.println("[FFAT] No groups.json found");
    return;
  }
  
  File f = FFat.open("/groups.json", FILE_READ);
  if (!f) return;
  
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  
  if (err) {
    Serial.printf("[FFAT] groups.json parse error: %s\n", err.c_str());
    return;
  }
  
  activeGroupIdx = doc["activeGroupIdx"] | -1;
  JsonArray groupsArr = doc["groups"].as<JsonArray>();
  groupCount = 0;
  
  for (JsonObject grp : groupsArr) {
    if (groupCount >= GROUP_MAX) break;
    Group& g = groups[groupCount];
    strlcpy(g.id, grp["id"] | "", sizeof(g.id));
    strlcpy(g.name, grp["name"] | "", sizeof(g.name));
    strlcpy(g.key, grp["key"] | "", sizeof(g.key));
    strlcpy(g.templ, grp["templ"] | "general", sizeof(g.templ));
    g.rolesEnabled = grp["rolesEnabled"] | false;
    g.overdueEnabled = grp["overdueEnabled"] | false;
    g.overdueThresholdMins = grp["overdueThresholdMins"] | 10;
    
    JsonArray devArr = grp["devices"].as<JsonArray>();
    g.deviceCount = 0;
    for (JsonObject dev : devArr) {
      if (g.deviceCount >= GROUP_DEVICE_MAX) break;
      GroupDevice& d = g.devices[g.deviceCount];
      strlcpy(d.id, dev["id"] | "", sizeof(d.id));
      strlcpy(d.name, dev["name"] | "", sizeof(d.name));
      strlcpy(d.role, dev["role"] | "", sizeof(d.role));
      g.deviceCount++;
    }
    groupCount++;
  }
  
  Serial.printf("[FFAT] Loaded %d group(s), active=%d\n", groupCount, activeGroupIdx);
}

void groupsClear() {
  groupCount = 0;
  activeGroupIdx = -1;
  memset(groups, 0, sizeof(groups));
  if (FFat.begin(false)) {
    if (FFat.exists("/groups.json")) FFat.remove("/groups.json");
  }
  Serial.println("[GROUPS] All groups cleared");
}

// Build groups JSON for BLE/WiFi response
String buildGroupsJSON() {
  JsonDocument doc;
  doc["activeGroupIdx"] = activeGroupIdx;
  JsonArray groupsArr = doc["groups"].to<JsonArray>();
  
  for (int g = 0; g < groupCount; g++) {
    JsonObject grp = groupsArr.add<JsonObject>();
    grp["id"] = groups[g].id;
    grp["name"] = groups[g].name;
    grp["key"] = groups[g].key;
    grp["templ"] = groups[g].templ;
    grp["rolesEnabled"] = groups[g].rolesEnabled;
    grp["overdueEnabled"] = groups[g].overdueEnabled;
    grp["overdueThresholdMins"] = groups[g].overdueThresholdMins;
    
    JsonArray devArr = grp["devices"].to<JsonArray>();
    for (int d = 0; d < groups[g].deviceCount; d++) {
      JsonObject dev = devArr.add<JsonObject>();
      dev["id"] = groups[g].devices[d].id;
      dev["name"] = groups[g].devices[d].name;
      dev["role"] = groups[g].devices[d].role;
    }
  }
  
  String result;
  serializeJson(doc, result);
  return result;
}

// Parse and apply groups JSON from web app
bool groupsParseAndSave(const char* json) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.printf("[GROUPS] Parse error: %s\n", err.c_str());
    return false;
  }
  
  activeGroupIdx = doc["activeGroupIdx"] | -1;
  JsonArray groupsArr = doc["groups"].as<JsonArray>();
  groupCount = 0;
  
  for (JsonObject grp : groupsArr) {
    if (groupCount >= GROUP_MAX) break;
    Group& g = groups[groupCount];
    strlcpy(g.id, grp["id"] | "", sizeof(g.id));
    strlcpy(g.name, grp["name"] | "", sizeof(g.name));
    strlcpy(g.key, grp["key"] | "", sizeof(g.key));
    strlcpy(g.templ, grp["templ"] | "general", sizeof(g.templ));
    g.rolesEnabled = grp["rolesEnabled"] | false;
    g.overdueEnabled = grp["overdueEnabled"] | false;
    g.overdueThresholdMins = grp["overdueThresholdMins"] | 10;
    
    JsonArray devArr = grp["devices"].as<JsonArray>();
    g.deviceCount = 0;
    for (JsonObject dev : devArr) {
      if (g.deviceCount >= GROUP_DEVICE_MAX) break;
      GroupDevice& d = g.devices[g.deviceCount];
      strlcpy(d.id, dev["id"] | "", sizeof(d.id));
      strlcpy(d.name, dev["name"] | "", sizeof(d.name));
      strlcpy(d.role, dev["role"] | "", sizeof(d.role));
      g.deviceCount++;
    }
    groupCount++;
  }
  
  groupsSave();
  Serial.printf("[GROUPS] Synced %d group(s) from web app\n", groupCount);
  return true;
}

// Switch active group and update LoRa key
void groupSetActive(int idx) {
  if (idx < -1 || idx >= groupCount) return;
  activeGroupIdx = idx;
  
#if ENABLE_LORA
  if (idx >= 0 && strlen(groups[idx].key) == LORA_KEY_LEN) {
    // Switch to group's network key
    uint8_t newKey[LORA_KEY_LEN];
    memcpy(newKey, groups[idx].key, LORA_KEY_LEN);
    loraSaveKey(newKey);
    Serial.printf("[GROUPS] Switched to group '%s' key\n", groups[idx].name);
  } else {
    // Switch back to default key
    uint8_t defaultKey[LORA_KEY_LEN] = {'A','z','i','m','u','t','h','N','e','t','w','o','r','k','0','1'};
    loraSaveKey(defaultKey);
    Serial.println("[GROUPS] Switched to personal mode (default key)");
  }
#endif
  
  groupsSave();
}

// Forward declaration — defined in globals below
extern bool wifiEnabled;
extern int  routeLogCount;

// ============================================================
// PAGE INDEX HELPERS
// Base order (no optional pages):
//  0  Position (GPS / Acquiring)
//  1  Compass
//  2  Track
//  3  Waypoint / Navigate
//  4  Elevation
//  5  Clock
//  6  Countdown
//  7  Stopwatch
//  8  Device Info
//  9  Settings
// 10  Power Off
// Optional pages shift everything after them +1:
//  +Messages (after GPS, when loraMsgCount > 0)
//  +Route Log (after Elevation, when routeLogCount > 0)
//  +Weather (after Route Log slot, when wifiEnabled)
// ============================================================
#if ENABLE_LORA
int pageMsgsOffset()  { return loraMsgCount > 0 ? 1 : 0; }
int pageMessages()    { return loraMsgCount > 0 ? 1 : -1; }
#else
int pageMsgsOffset()  { return 0; }
#endif
int pageLogOffset()   { return routeLogCount > 0 ? 1 : 0; }
int pageWifiOffset()  { return (wifiEnabled || weatherReceived) ? 1 : 0; }  // must match pageWeather condition

// Base sequence (no optional pages): 0=GPS,1=Compass,2=Track,3=Waypoint,4=Elevation,5=Groups,6=Clock,7=Countdown,8=Stopwatch,9=Info,10=Settings,11=Power
// Each optional page (msgs, routelog, weather) shifts everything after it by +1
// With ENABLE_LORA, Groups page is always present at index 5+msgOffset
int pageCompass()     { return 1 + pageMsgsOffset(); }
int pageTrack()       { return 2 + pageMsgsOffset(); }
int pageWaypoint()    { return 3 + pageMsgsOffset(); }
int pageElevation()   { return 4 + pageMsgsOffset(); }
#if ENABLE_LORA
int pageGroups()      { return 5 + pageMsgsOffset(); }
int pageRouteLog()    { return routeLogCount > 0 ? 6 + pageMsgsOffset() : -1; }
int pageWeather()     { return (wifiEnabled || weatherReceived) ? 6 + pageMsgsOffset() + pageLogOffset() : -1; }
int pageClock()       { return 6 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageCountdown()   { return 7 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageStopwatch()   { return 8 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageInfo()        { return 9 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageSettings()    { return 10 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pagePower()       { return 11 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
#else
int pageRouteLog()    { return routeLogCount > 0 ? 5 + pageMsgsOffset() : -1; }
int pageWeather()     { return (wifiEnabled || weatherReceived) ? 5 + pageMsgsOffset() + pageLogOffset() : -1; }
int pageClock()       { return 5 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageCountdown()   { return 6 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageStopwatch()   { return 7 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageInfo()        { return 8 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pageSettings()    { return 9 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
int pagePower()       { return 10 + pageMsgsOffset() + pageLogOffset() + pageWifiOffset(); }
#endif

int pageCount() {
  int base = 11; // pages 0-10 always present (without LoRa)
#if ENABLE_LORA
  base = 12; // +1 for Groups page
  if (loraMsgCount > 0) base++;
#endif
  if (routeLogCount > 0) base++;
  if (wifiEnabled || weatherReceived) base++;  // must match pageWeather condition
  return base;
}

// Legacy aliases
int pageRoute()       { return pageTrack(); }
int pageSaveLoc()     { return pageWaypoint(); }

// Countdown timer
const uint32_t CDT_PRESETS[]   = {300,600,900,1200,1800,2700,3600}; // secs: 5,10,15,20,30,45,60min
const int      CDT_PRESET_COUNT = 7;
int      cdtPresetIdx  = 0;       // current preset index
uint32_t cdtSetSecs    = 300;     // currently set duration in seconds
uint32_t cdtRemMs      = 300000;  // remaining milliseconds
uint32_t cdtStartMs    = 0;       // millis() when started
bool     cdtRunning    = false;
bool     cdtDone       = false;   // reached zero
bool     cdtSetMode    = false;   // in time-set mode (long press to enter/exit)
bool     cdtAlerting   = false;   // currently flashing
uint32_t cdtAlertStart = 0;
uint8_t  cdtAlertFlash = 0;       // flash counter

// Elevation history — circular buffer, 1 sample/sec, 128 entries = ~2min rolling
#define ELEV_BUF_SIZE  128
float    elevBuf[ELEV_BUF_SIZE] = {0};
float    trackAltBuf[128]       = {0};  // temp buffer for completed track display
int      elevBufHead  = 0;     // next write index
int      elevBufCount = 0;     // how many valid entries
uint32_t lastElevMs   = 0;
bool     elevLineMode  = false; // false=bar, true=line — toggled by long press

// Settings
bool     setSpeedMph   = false;  // false=km/h, true=mph
bool     setElevFt     = false;  // false=m, true=ft
uint8_t  setTimeoutIdx = 1;      // index into timeout presets
bool     setBrightFull = true;   // true=full, false=dim (contrast)
bool     settingsSel     = false;  // true when a setting row is being edited
uint8_t  settingsRow     = 0;      // 0-3 which row is selected
uint32_t settingsIdleMs  = 0;      // last interaction time — auto-exit after 5s
#define  SETTINGS_IDLE_MS  5000    // abandon edit after 5s inactivity
bool     settingsSaved   = false;  // true for one draw cycle to show saved message
uint32_t settingsSavedMs = 0;      // when saved message was shown
bool     qrCodeActive    = false;  // true while showing WiFi QR code (blocks timeout)

// WiFi & connected mode
char     wifiSSID[64]    = "";
char     wifiPass[64]    = "";
char     owmApiKey[40]   = "";  // OpenWeatherMap API key
bool     wifiEnabled     = false;
bool     wifiConnected   = false;

// ── AP / Web server ──────────────────────────────────────────────────────────
WebServer  webServer(80);
DNSServer  dnsServer;
char       apSSID[32]    = "";      // e.g. "Azimuth-Proto1"
char       apPass[20]    = "azimuth1";  // default AP password
bool     wifiScanPending   = false;  // async scan requested from BLE
bool     wifiRefreshPending  = false; // async WiFi fetch requested from BLE
bool     wpBroadcastPending  = false; // async waypoint broadcast requested from BLE
bool     wpClearPending      = false; // async waypoint clear broadcast requested from BLE
bool     wxPushPending       = false; // async weather push requested from BLE
char     wxPendingJson[256]  = {0};   // weather JSON held for main-loop parsing
char     wxPendingPeerId[8]  = {0};   // peer ID to forward weather to via LoRa (empty = self only)
bool     oledRedrawPending   = false; // request oledDraw() from main loop
char     wpPendingJson[512]  = {0};   // raw JSON held for main-loop parsing
int      wifiLastFetchHour   = -1;    // hour of last successful fetch
int      wifiLastFetchMin    = -1;    // minute of last successful fetch
bool     weatherDetailActive = false; // true while long-pressing on weather page
String   wifiScanResult  = "";     // last scan result JSON
uint32_t wifiLastFetchMs = 0;
#define  WIFI_FETCH_INTERVAL_MS  3600000UL  // 1 hour
#define  WIFI_CONNECT_TIMEOUT_MS 10000      // 10s to connect

// Fetched data
struct WeatherData {
  bool    valid        = false;
  float   tempC        = 0;
  float   feelsLikeC   = 0;
  char    desc[24]     = "";
  float   windKmh      = 0;
  float   windGustKmh  = 0;
  char    windDir[4]   = "";
  int     humidity     = 0;
  float   tempMinC     = 0;
  float   tempMaxC     = 0;
  int     weatherId    = 0;   // OWM condition code
  char    locationName[24] = "";  // city name from OWM
} weather;

struct SunData {
  bool    valid        = false;
  char    sunrise[6]   = "";  // HH:MM
  char    sunset[6]    = "";
  char    golden[6]    = "";  // golden hour end (morning)
  char    goldenEve[6] = "";  // golden hour start (evening)
  char    dayLength[9] = "";  // e.g. "12h 27m"
} sunData;

struct MoonData {
  bool    valid       = false;
  float   phase       = 0;   // 0-1
  char    phaseName[16] = "";
  float   illumination = 0;  // 0-100%
} moonData;

float    magDeclination = 0.0;  // magnetic declination in degrees
bool     magDecValid    = false;

// BLE pairing
bool     pairingActive   = false;  // true while PIN is being displayed
bool     pinOverlayActive = false;  // true while manual PIN overlay held on GPS page
bool     pairingPending    = false;  // true when unbound device connected, awaiting PIN entry
bool     sessionAuthorised = false;  // true after correct PIN sent via VERPIN
uint32_t pairingPin      = 0;      // current 6-digit PIN
int      prePairingPage  = 0;      // page to return to after pairing
const uint16_t TIMEOUT_PRESETS[] = {5,10,30,60,0}; // seconds (0=never)
#define  TIMEOUT_PRESET_COUNT 5

// Stopwatch
bool     swRunning   = false;
uint32_t swStartMs   = 0;
uint32_t swElapsedMs = 0;

uint32_t swCurrentMs() {
  return swRunning ? swElapsedMs + (millis() - swStartMs) : swElapsedMs;
}

void swStart() { swStartMs = millis(); swRunning = true; }
void swStop()  { swElapsedMs = swCurrentMs(); swRunning = false; }
void swClear() { swRunning = false; swElapsedMs = 0; }

// Simulation
#ifdef SIMULATE_GPS
float simAngle = 0.0;
#endif

// ICONS — 8x8 bitmaps
// ============================================================
static const uint8_t PROGMEM ICON_GPS[] = {
  0b00111000,0b01111100,0b11111110,0b11111110,
  0b11111110,0b01111100,0b00111000,0b00010000
};
static const uint8_t PROGMEM ICON_SAT[] = {
  0b10000001,0b01000010,0b00100100,0b00011000,
  0b00011000,0b00100100,0b01000010,0b10000001
};
static const uint8_t PROGMEM ICON_SPD[] = {
  0b00111100,0b01000010,0b10110101,0b10000001,
  0b10000001,0b01000010,0b00111100,0b00000000
};
static const uint8_t PROGMEM ICON_HDG[] = {
  0b00010000,0b00111000,0b01111100,0b00010000,
  0b00010000,0b00010000,0b00010000,0b00000000
};
static const uint8_t PROGMEM ICON_BT[] = {
  0b00011000,0b00011100,0b01011010,0b00111000,
  0b00111000,0b01011010,0b00011100,0b00011000
};
static const uint8_t PROGMEM ICON_PWR[] = {
  0b00111100,0b01000010,0b10011001,0b10100101,
  0b10100101,0b10011001,0b01000010,0b00111100
};
static const uint8_t PROGMEM ICON_CLK[] = {
  0b00111100,0b01000010,0b10010101,0b10110001,
  0b10000001,0b01000010,0b00111100,0b00000000
};
static const uint8_t PROGMEM ICON_CLOUD[] = {
  0b00011000,
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00000000,
  0b00000000
};

// ── Weather icons 24x24 px ──────────────────────────────────────────────────
static const uint8_t PROGMEM ICON_WX_CLEAR[] = {
  0x00,0x00,0x00,0x00,0x18,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x08,0x7E,0x20,0x05,0x81,0xC0,
  0x02,0x00,0x40,0x04,0x00,0x20,
  0x04,0x00,0x20,0x08,0x00,0x10,
  0x08,0x00,0x10,0x48,0x00,0x12,
  0x48,0x00,0x12,0x08,0x00,0x10,
  0x08,0x00,0x10,0x04,0x00,0x20,
  0x04,0x00,0x20,0x02,0x00,0x40,
  0x05,0x81,0xC0,0x08,0x7E,0x20,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x18,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_WX_FEW[] = {
  0x10,0x00,0x00,0x00,0x00,0x00,
  0x3E,0x00,0x00,0xC1,0x00,0x00,
  0xC1,0x00,0x00,0x3E,0x00,0x00,
  0x00,0x00,0x00,0x10,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0xF0,0x00,0x01,0x08,0x00,
  0x06,0x06,0x00,0x08,0x01,0x00,
  0x08,0x01,0x00,0x0F,0xFF,0x00,
  0x07,0xFE,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_WX_CLOUD[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xFC,0x00,
  0x03,0x03,0x00,0x04,0x00,0x80,
  0x08,0x00,0x40,0x10,0x00,0x20,
  0x10,0x00,0x20,0x20,0x00,0x10,
  0x20,0x00,0x10,0x3F,0xFF,0xF0,
  0x1F,0xFF,0xE0,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_WX_RAIN[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xFC,0x00,
  0x03,0x03,0x00,0x04,0x00,0x80,
  0x08,0x00,0x40,0x10,0x00,0x20,
  0x1F,0xFF,0xE0,0x0F,0xFF,0xC0,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x04,0x04,0x00,0x04,0x44,0x40,
  0x04,0x44,0x40,0x00,0x50,0x40,
  0x01,0x11,0x00,0x01,0x11,0x00,
  0x01,0x01,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_WX_DRIZZLE[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xFC,0x00,
  0x03,0x03,0x00,0x04,0x00,0x80,
  0x08,0x00,0x40,0x10,0x00,0x20,
  0x1F,0xFF,0xE0,0x0F,0xFF,0xC0,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x04,0x04,0x00,0x00,0x40,0x40,
  0x00,0x00,0x00,0x01,0x01,0x00,
  0x00,0x10,0x00,0x00,0x00,0x00,
  0x04,0x04,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_WX_SNOW[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xFC,0x00,
  0x03,0x03,0x00,0x04,0x00,0x80,
  0x08,0x00,0x40,0x10,0x00,0x20,
  0x1F,0xFF,0xE0,0x0F,0xFF,0xC0,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x04,0x10,0x40,0x0E,0x38,0xE0,
  0x04,0x10,0x40,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x82,0x00,
  0x01,0xC7,0x00,0x00,0x82,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_WX_MIST[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x3F,0xFF,0xF8,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x3F,0xFF,0xF8,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x0F,0xFF,0xF0,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x3F,0xFF,0xF8,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_WX_STORM[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0xFC,0x00,0x03,0x03,0x00,
  0x04,0x00,0x80,0x08,0x00,0x40,
  0x10,0x00,0x20,0x1F,0xFF,0xE0,
  0x0F,0xFF,0xC0,0x00,0x00,0x00,
  0x00,0x1C,0x00,0x00,0x60,0x00,
  0x0F,0x80,0x00,0x00,0xF0,0x00,
  0x00,0x60,0x00,0x01,0x80,0x00,
  0x06,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t PROGMEM ICON_CHK[] = {
  0b00000001,0b00000011,0b00000110,0b10001100,
  0b11011000,0b01110000,0b00100000,0b00000000
};
static const uint8_t PROGMEM ICON_CRS[] = {
  0b10000001,0b11000011,0b01100110,0b00111100,
  0b00111100,0b01100110,0b11000011,0b10000001
};
static const uint8_t PROGMEM ICON_CMP[] = {
  0b00111100,0b01000010,0b10011001,0b10011001,
  0b10011001,0b10011001,0b01000010,0b00111100
};
static const uint8_t PROGMEM ICON_ROUTE[] = {
  0b10000001,0b11000011,0b10100101,0b10011001,
  0b10000001,0b01000010,0b00100100,0b00011000
};
static const uint8_t PROGMEM ICON_COMPASS[] = {
  0b00111100,
  0b01000010,
  0b10011001,
  0b10111101,
  0b10000001,
  0b01000010,
  0b00111100,
  0b00000000,
};
static const uint8_t PROGMEM ICON_PIN[] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00011000,
  0b00011000,
};
static const uint8_t PROGMEM ICON_CDT[] = {
  0b00111100,
  0b01000010,
  0b10100101,
  0b10010001,
  0b10011101,
  0b01000010,
  0b00111100,
  0b00000000,
};
static const uint8_t PROGMEM ICON_ELEV[] = {
  0b00000000,
  0b00001000,
  0b00011100,
  0b00111110,
  0b01110111,
  0b11100011,
  0b00000001,
  0b11111111,
};
static const uint8_t PROGMEM ICON_PWRBTN[] = {
  0b00011000,
  0b00011000,
  0b01111110,
  0b11000011,
  0b10000001,
  0b10000001,
  0b01111110,
  0b00000000,
};
static const uint8_t PROGMEM ICON_COG[] = {
  0b00010000,
  0b01111100,
  0b10010010,
  0b10111010,
  0b10010010,
  0b01111100,
  0b00010000,
  0b00000000,
};
static const uint8_t PROGMEM ICON_CRS2[] = {
  0b00111100,
  0b01000010,
  0b10010001,
  0b10111101,
  0b10010001,
  0b01000010,
  0b00111100,
  0b00000000,
};
static const uint8_t PROGMEM ICON_TIMER[] = {
  0b00111100,0b01000010,0b10000101,0b10000111,
  0b10000001,0b01000010,0b00111100,0b00000000
};
static const uint8_t PROGMEM ICON_TIME[] = {
  0b00111100,0b01000010,0b10010101,0b10110001,
  0b10000001,0b01000010,0b00111100,0b00000000
};

// ============================================================
// NAVIGATION HELPERS
// ============================================================
float haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

float bearingTo(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1))*sin(radians(lat2)) -
            sin(radians(lat1))*cos(radians(lat2))*cos(dLon);
  float b = degrees(atan2(y, x));
  return fmod(b + 360.0, 360.0);
}

const char* getCardinal(float deg) {
  const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  return dirs[(int)((deg + 22.5) / 45.0) % 8];
}

void drawNeedle(int cx, int cy, int len, float deg) {
  float rad = (deg - 90.0) * DEG_TO_RAD;
  float perpRad = rad + PI/2.0;
  int tipX = cx + (int)(cos(rad) * len);
  int tipY = cy + (int)(sin(rad) * len);
  int bx   = cx - (int)(cos(rad) * len * 0.5);
  int by   = cy - (int)(sin(rad) * len * 0.5);
  int wx   = (int)(cos(perpRad) * 3);
  int wy   = (int)(sin(perpRad) * 3);
  display.fillTriangle(tipX, tipY, bx+wx, by+wy, bx-wx, by-wy, SSD1306_WHITE);
}

// ============================================================
// BATTERY
// ============================================================
bool     batOnUSB    = false;  // true when USB only, no battery
bool     batCharging = false;  // true when USB + real battery present

void readBattery() {
  // --- ADC battery voltage read ---
  pinMode(BAT_CTRL_PIN, OUTPUT);
  digitalWrite(BAT_CTRL_PIN, HIGH);  // HIGH enables voltage divider (V4 datasheet)
  delay(10);
  int raw = 0;
  for (int i = 0; i < 8; i++) { raw += analogRead(BAT_ADC_PIN); delay(2); }
  raw /= 8;
  batVoltage = (raw / (float)BAT_ADC_RES) * BAT_ADC_REF / BAT_DIVIDER;
  digitalWrite(BAT_CTRL_PIN, LOW);   // LOW disables — saves power between reads

  // --- USB detection via charger IC behaviour ---
  // No battery: charger IC idles at ~3.85V (stable, mid-range float)
  // Real battery: reads actual cell voltage — below 3.80V or above 4.15V
  bool usbPhysical = (bool)Serial;  // true when USB host is connected
  bool inFloatBand = (batVoltage >= BAT_NO_BAT_V && batVoltage < BAT_FULL_V);
  batOnUSB = usbPhysical && inFloatBand;  // USB + no battery = float band
  batCharging = usbPhysical && !inFloatBand;  // USB + real battery present

  // --- Determine battery percentage ---
  // If on USB with no real battery, ADC reads charger IC idle output (~3.5-4.1V) — ignore it
  bool batteryPresent = !batOnUSB || (batVoltage > BAT_MAX_V * 0.98) || (batVoltage < 3.4);
  // ↑ battery likely present if: not on USB, OR voltage is at/above full charge, OR suspiciously low

  if (batOnUSB) {
    // Charger IC idle float — no real battery connected
    batVoltage = 0.0;
    batPercent = 0;
  } else {
    float clamped = constrain(batVoltage, BAT_MIN_V, BAT_MAX_V);
    batPercent = (int)((clamped - BAT_MIN_V) / (BAT_MAX_V - BAT_MIN_V) * 100.0);
  }


  Serial.printf("[BAT] %.2fV  %d%%%s%s\n", batVoltage, batPercent,
                batOnUSB ? " (USB only)" : "",
                batCharging ? " (USB+bat)" : "");
}

void drawBatIcon(int x, int y, int pct) {
  display.drawRect(x, y, 14, 7, SSD1306_WHITE);
  display.fillRect(x+14, y+2, 2, 3, SSD1306_WHITE);
  int segs = (pct>=80)?4:(pct>=55)?3:(pct>=30)?2:(pct>=10)?1:0;
  for (int s=0; s<segs; s++) display.fillRect(x+2+s*3, y+2, 2, 3, SSD1306_WHITE);
}

// ============================================================
// TIMEZONE
// ============================================================
void loadTimezone() {
  prefs.begin("azimuth", true);
  tzOffset      = prefs.getChar("tz", 0);
  elevLineMode  = prefs.getBool("elevLine", false);
  // Device name — generate unique name if not yet set
  prefs.getString("devName", bleDeviceName, sizeof(bleDeviceName));
  // Load original name if saved (for restoring when leaving group)
  prefs.getString("origName", originalDeviceName, sizeof(originalDeviceName));
  if (strlen(originalDeviceName) > 0) {
    Serial.printf("[BLE] Original name saved: %s\n", originalDeviceName);
  }
  // Generate if empty or still the plain default (no suffix)
  bool needsName = (strlen(bleDeviceName) == 0 ||
                    strcmp(bleDeviceName, "Azimuth") == 0 ||
                    strncmp(bleDeviceName, "Azimuth-", 8) != 0);
  if (needsName) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(bleDeviceName, sizeof(bleDeviceName),
      "Azimuth-%02X%02X", mac[4], mac[5]);
    prefs.begin("azimuth", false);
    prefs.putString("devName", bleDeviceName);
    prefs.end();
    Serial.printf("[BLE] Generated device name: %s\n", bleDeviceName);
  } else {
    Serial.printf("[BLE] Device name: %s\n", bleDeviceName);
  }

  // WiFi credentials
  prefs.getString("wifiSSID", wifiSSID, sizeof(wifiSSID));
  prefs.getString("wifiPass", wifiPass, sizeof(wifiPass));
  prefs.getString("owmKey",   owmApiKey, sizeof(owmApiKey));
  wifiEnabled = (strlen(wifiSSID) > 0);
  // Restore AP mode if it was enabled before reboot
  // Disabled - AP should not auto-start, user must enable manually
  // bool savedAp = prefs.getBool("apEnabled", false);
  bool savedAp = false;
  setElevFt     = prefs.getBool("elevFt", false);
  setTimeoutIdx = prefs.getUChar("timeoutIdx", 1);
  setBrightFull = prefs.getBool("brightFull", true);
  lastKnownLat = prefs.getDouble("gpsLat", 0.0);
  lastKnownLon = prefs.getDouble("gpsLon", 0.0);
  lastKnownAlt = prefs.getFloat("gpsAlt",  0.0);
  // Restore saved pin (navigate waypoint)
  savedLat = prefs.getDouble("pinLat", 0.0);
  savedLon = prefs.getDouble("pinLon", 0.0);
  locSaved = prefs.getBool("pinSaved", false) && (savedLat != 0.0 || savedLon != 0.0);
#if ENABLE_LORA
  loraRegionIdx   = prefs.getUChar("loraRegion", 0);
  loraIntervalSec = prefs.getUShort("loraInterval", 30);
  relayEnabled    = prefs.getBool("relayMode", false);
  deviceLocked    = prefs.getBool("locked", false);
  if (loraRegionIdx >= LORA_REGION_COUNT) loraRegionIdx = 0;
  if (loraIntervalSec < LORA_MIN_INTERVAL_S) loraIntervalSec = LORA_MIN_INTERVAL_S;
  if (loraIntervalSec > LORA_MAX_INTERVAL_S) loraIntervalSec = LORA_MAX_INTERVAL_S;
#endif
  prefs.end();
  if (savedAp) apStart();
  if (lastKnownLat != 0.0)
    Serial.printf("[GPS] Last pos: %.6f, %.6f\n", lastKnownLat, lastKnownLon);
}
void saveTimezone() {
  prefs.begin("azimuth", false);
  prefs.putChar("tz", tzOffset);
  prefs.end();
}

void saveSettings() {
  prefs.begin("azimuth", false);
  prefs.putBool("speedMph",   setSpeedMph);
  prefs.putBool("elevFt",     setElevFt);
  prefs.putUChar("timeoutIdx",setTimeoutIdx);
  prefs.putBool("brightFull", setBrightFull);
#if ENABLE_LORA
  prefs.putBool("relayMode",  relayEnabled);
  prefs.putBool("locked",     deviceLocked);
#endif
  prefs.end();
  // Apply brightness — full=normal, dim=inverted display (more noticeable than contrast)
  display.invertDisplay(!setBrightFull);
}
void saveLastPosition() {
  prefs.begin("azimuth", false);
  prefs.putDouble("gpsLat", cachedLat);
  prefs.putDouble("gpsLon", cachedLon);
  prefs.putFloat("gpsAlt",  cachedAlt);
  prefs.end();
  Serial.printf("[GPS] Position saved: %.6f, %.6f\n", cachedLat, cachedLon);
}

void saveElevMode() {
  prefs.begin("azimuth", false);
  prefs.putBool("elevLine", elevLineMode);
  prefs.end();
}

// ── Pending key share NVS persistence ─────────────────────────────────────
// Saves pending request so it survives reboot
void pendingKeyShareSave() {
  prefs.begin("azimuth", false);
  prefs.putBool("pksActive", pendingKeyShare.active);
  if (pendingKeyShare.active) {
    prefs.putBytes("pksFromId", pendingKeyShare.fromId, 8);
    prefs.putBytes("pksLeader", pendingKeyShare.leaderName, 20);
    prefs.putBytes("pksGroup", pendingKeyShare.groupName, 32);
    prefs.putBytes("pksKey", pendingKeyShare.key, LORA_KEY_LEN);
  }
  prefs.end();
}

void pendingKeyShareLoad() {
  prefs.begin("azimuth", true);
  pendingKeyShare.active = prefs.getBool("pksActive", false);
  if (pendingKeyShare.active) {
    prefs.getBytes("pksFromId", pendingKeyShare.fromId, 8);
    prefs.getBytes("pksLeader", pendingKeyShare.leaderName, 20);
    prefs.getBytes("pksGroup", pendingKeyShare.groupName, 32);
    prefs.getBytes("pksKey", pendingKeyShare.key, LORA_KEY_LEN);
    pendingKeyShare.receivedMs = millis();
    Serial.printf("[NVS] Restored pending key share from %s group '%s'\n", pendingKeyShare.leaderName, pendingKeyShare.groupName);
  }
  prefs.end();
}

void pendingKeyShareClear() {
  pendingKeyShare.active = false;
  prefs.begin("azimuth", false);
  prefs.putBool("pksActive", false);
  prefs.end();
}

// ── Stored groups NVS persistence ─────────────────────────────────────────
void storedGroupsSave() {
  prefs.begin("azimuth", false);
  prefs.putInt("sgCount", storedGroupCount);
  prefs.putInt("sgActive", activeStoredGroup);
  for (int i = 0; i < storedGroupCount && i < MAX_STORED_GROUPS; i++) {
    char keyName[16], keyKey[16], keyLeader[16], keyIsLeader[16], keyCanLeave[16];
    snprintf(keyName, sizeof(keyName), "sg%dName", i);
    snprintf(keyKey, sizeof(keyKey), "sg%dKey", i);
    snprintf(keyLeader, sizeof(keyLeader), "sg%dLeader", i);
    snprintf(keyIsLeader, sizeof(keyIsLeader), "sg%dIsLdr", i);
    snprintf(keyCanLeave, sizeof(keyCanLeave), "sg%dCanLv", i);
    prefs.putBytes(keyName, storedGroups[i].name, 32);
    prefs.putBytes(keyKey, storedGroups[i].key, LORA_KEY_LEN);
    prefs.putBytes(keyLeader, storedGroups[i].leaderName, 20);
    prefs.putBool(keyIsLeader, storedGroups[i].isLeader);
    prefs.putBool(keyCanLeave, storedGroups[i].canLeave);
  }
  prefs.end();
  Serial.printf("[NVS] Saved %d stored groups, active=%d\n", storedGroupCount, activeStoredGroup);
}

void storedGroupsLoad() {
  prefs.begin("azimuth", true);
  storedGroupCount = prefs.getInt("sgCount", 0);
  activeStoredGroup = prefs.getInt("sgActive", -1);
  if (storedGroupCount > MAX_STORED_GROUPS) storedGroupCount = MAX_STORED_GROUPS;
  for (int i = 0; i < storedGroupCount; i++) {
    char keyName[16], keyKey[16], keyLeader[16], keyIsLeader[16], keyCanLeave[16];
    snprintf(keyName, sizeof(keyName), "sg%dName", i);
    snprintf(keyKey, sizeof(keyKey), "sg%dKey", i);
    snprintf(keyLeader, sizeof(keyLeader), "sg%dLeader", i);
    snprintf(keyIsLeader, sizeof(keyIsLeader), "sg%dIsLdr", i);
    snprintf(keyCanLeave, sizeof(keyCanLeave), "sg%dCanLv", i);
    prefs.getBytes(keyName, storedGroups[i].name, 32);
    prefs.getBytes(keyKey, storedGroups[i].key, LORA_KEY_LEN);
    prefs.getBytes(keyLeader, storedGroups[i].leaderName, 20);
    storedGroups[i].isLeader = prefs.getBool(keyIsLeader, false);
    storedGroups[i].canLeave = prefs.getBool(keyCanLeave, true);
  }
  prefs.end();
  if (storedGroupCount > 0) {
    Serial.printf("[NVS] Loaded %d stored groups, active=%d\n", storedGroupCount, activeStoredGroup);
  }
}

// Add a group (from key share acceptance or web app)
bool storedGroupAdd(const char* name, const uint8_t* key, const char* leaderName, bool isLeader, bool canLeave = true) {
  if (storedGroupCount >= MAX_STORED_GROUPS) return false;
  // Check for duplicate key
  for (int i = 0; i < storedGroupCount; i++) {
    if (memcmp(storedGroups[i].key, key, LORA_KEY_LEN) == 0) {
      // Update existing
      strncpy(storedGroups[i].name, name, 31);
      strncpy(storedGroups[i].leaderName, leaderName, 19);
      storedGroups[i].canLeave = canLeave;
      storedGroupsSave();
      return true;
    }
  }
  // Add new
  strncpy(storedGroups[storedGroupCount].name, name, 31);
  memcpy(storedGroups[storedGroupCount].key, key, LORA_KEY_LEN);
  strncpy(storedGroups[storedGroupCount].leaderName, leaderName, 19);
  storedGroups[storedGroupCount].isLeader = isLeader;
  storedGroups[storedGroupCount].canLeave = canLeave;
  storedGroupCount++;
  storedGroupsSave();
  return true;
}

// Delete a group by index
void storedGroupDelete(int idx) {
  if (idx < 0 || idx >= storedGroupCount) return;
  
  // If this is the active group, broadcast leave notification BEFORE changing key
  bool wasActive = (activeStoredGroup == idx);
  if (wasActive) {
    loraGroupLeave();  // Broadcast on current key before switching
  }
  
  // Shift remaining
  for (int i = idx; i < storedGroupCount - 1; i++) {
    storedGroups[i] = storedGroups[i + 1];
  }
  storedGroupCount--;
  // Adjust active index
  if (wasActive) {
    activeStoredGroup = -1;
    // Revert to default key
    memcpy(loraKey, LORA_DEFAULT_KEY, LORA_KEY_LEN);
    loraSaveKey(loraKey);
    
    // Restore original device name if we have one saved
    Serial.printf("[GROUPS] Checking originalDeviceName: '%s' (len=%d)\n", originalDeviceName, strlen(originalDeviceName));
    if (strlen(originalDeviceName) > 0) {
      strncpy(bleDeviceName, originalDeviceName, sizeof(bleDeviceName) - 1);
      bleDeviceName[sizeof(bleDeviceName) - 1] = 0;
      prefs.begin("azimuth", false);
      prefs.putString("devName", bleDeviceName);
      prefs.remove("origName");  // Clear saved original
      prefs.end();
      originalDeviceName[0] = 0;  // Clear RAM copy
      Serial.printf("[GROUPS] Restored original name: %s\n", bleDeviceName);
    }
  } else if (activeStoredGroup > idx) {
    activeStoredGroup--;
  }
  if (storedGroupSel >= storedGroupCount && storedGroupSel > 0) {
    storedGroupSel--;
  }
  storedGroupsSave();
  
  // Notify BLE web app that groups changed
  if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
    char json[32];
    snprintf(json, sizeof(json), "{\"groupsChanged\":1}");
    pPeerChar->setValue((uint8_t*)json, strlen(json));
    pPeerChar->notify();
  }
}

// Activate a group (switch to its key)
void storedGroupActivate(int idx) {
  if (idx < 0 || idx >= storedGroupCount) {
    // Personal mode
    activeStoredGroup = -1;
    memcpy(loraKey, LORA_DEFAULT_KEY, LORA_KEY_LEN);
  } else {
    activeStoredGroup = idx;
    memcpy(loraKey, storedGroups[idx].key, LORA_KEY_LEN);
  }
  loraSaveKey(loraKey);
  storedGroupsSave();
  Serial.printf("[GROUPS] Activated group %d\n", activeStoredGroup);
}

void getLocalTime(int &h, int &m, int &s, int &d, int &mo, int &y) {
  h  = gps.time.isValid() ? gps.time.hour()   : 0;
  m  = gps.time.isValid() ? gps.time.minute() : 0;
  s  = gps.time.isValid() ? gps.time.second() : 0;
  d  = gps.date.isValid() ? gps.date.day()    : 1;
  mo = gps.date.isValid() ? gps.date.month()  : 1;
  y  = gps.date.isValid() ? gps.date.year()   : 2026;
  h += tzOffset;
  if (h >= 24) { h -= 24; d++; }
  if (h <   0) { h += 24; d--; }
}

// ============================================================
// TIMER HELPERS
// ============================================================
void formatTime(uint32_t ms, char* buf, int len) {
  uint32_t tot = ms/1000;
  uint32_t h = tot/3600, mn = (tot%3600)/60, s = tot%60;
  if (h>0) snprintf(buf,len,"%02lu:%02lu:%02lu",h,mn,s);
  else      snprintf(buf,len,"%02lu:%02lu",mn,s);
}

// ============================================================
// ROUTE HELPERS
// ============================================================
void routeAddPoint() {
  if (!routeRecording || !cachedFix) return;
  if (routeCount >= ROUTE_MAX_POINTS) return;
  if (routePoints == nullptr) {
    routePoints = (RoutePoint*)malloc(ROUTE_MAX_POINTS * sizeof(RoutePoint));
    if (!routePoints) { Serial.println("[ROUTE] malloc failed"); return; }
  }
  if (routeCount > 0)
    routeTotalDist += haversineDistance(routePoints[routeCount-1].lat,
                                        routePoints[routeCount-1].lon,
                                        cachedLat, cachedLon);
  routePoints[routeCount] = {cachedLat, cachedLon, (float)cachedSpd,
                              cachedAlt,
                              millis() - routeStartMs};
  routeCount++;
}
void clearRoute() {
  routeCount     = 0;
  routeTotalDist = 0.0;
  routeRecording = false;
}

// ============================================================
// BLE STRING BUILDER
// ============================================================
String buildJSON() {
  char buf[256];
  int bat = batOnUSB ? 255 : constrain(batPercent, 0, 100);
  snprintf(buf, sizeof(buf),
    "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"spd\":%.2f,\"hdg\":%.1f,\"sat\":%u,\"fix\":%d,\"rec\":%d,\"approx\":%d,\"acc\":%.0f,\"bat\":%d}",
    cachedLat, cachedLon, cachedAlt, cachedSpd, cachedHdg, cachedSats,
    cachedFix?1:0, routeRecording?1:0, cachedFixIsApprox?1:0, cachedApproxAccM, bat);
  return String(buf);
}

// ============================================================
// LORA HELPERS
// ============================================================
#if ENABLE_LORA
void loraInitDeviceId() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(loraDeviceId, sizeof(loraDeviceId), "%02X%02X%02X", mac[3], mac[4], mac[5]);
}

// AES-128 CTR encrypt/decrypt — same function for both directions
// nonce: 4-byte value prepended to form the 16-byte counter block
// in/out may point to the same buffer (in-place)
void loraCrypt(const uint8_t* in, uint8_t* out, int len, uint32_t nonce) {
  uint8_t ctr[16] = {0};
  memcpy(ctr, &nonce, 4);  // nonce in first 4 bytes, rest zero
  uint8_t stream[16];
  size_t nc_off = 0;
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, loraKey, 128);
  mbedtls_aes_crypt_ctr(&aes, len, &nc_off, ctr, stream, in, out);
  mbedtls_aes_free(&aes);
}

// Load key from NVS, fall back to default
void loraLoadKey() {
  prefs.begin("azimuth", false);  // read-write to save default if needed
  size_t len = prefs.getBytesLength("loraKey");
  if (len == LORA_KEY_LEN) {
    prefs.getBytes("loraKey", loraKey, LORA_KEY_LEN);
    Serial.printf("[LORA] Key loaded from NVS: %.16s\n", (char*)loraKey);
  } else {
    // No key in NVS — use and save default
    memcpy(loraKey, LORA_DEFAULT_KEY, LORA_KEY_LEN);
    prefs.putBytes("loraKey", loraKey, LORA_KEY_LEN);
    Serial.printf("[LORA] Default key saved to NVS: %.16s\n", (char*)loraKey);
  }
  prefs.end();
}

void loraSaveKey(const uint8_t* key) {
  memcpy(loraKey, key, LORA_KEY_LEN);
  prefs.begin("azimuth", false);
  prefs.putBytes("loraKey", loraKey, LORA_KEY_LEN);
  prefs.end();
}

// Initialise or re-initialise SX1262 with current region preset
bool loraBegin() {
  const LoRaRegionPreset& p = LORA_REGIONS[loraRegionIdx];
  if (!loraSPI) { loraSPI = new SPIClass(HSPI); }
  loraSPI->begin(9, 11, 10, 8);
  if (!radio) { radio = new SX1262(new Module(8, 14, 12, 13, *loraSPI)); }
  int state = radio->begin(p.freqMHz, p.bwKHz, p.sf, p.cr,
                           RADIOLIB_SX126X_SYNC_WORD_PRIVATE, p.power, 8, 1.8f, false);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LORA] Init failed: %d\n", state);
    loraReady = false;
    return false;
  }
  radio->setDio2AsRfSwitch(true);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraReady = true;
  Serial.printf("[LORA] Ready — %s %.1fMHz SF%d BW%.0f CR4/%d %ddBm\n",
    p.name, p.freqMHz, p.sf, p.bwKHz, p.cr, p.power);
  return true;
}

// Build Azimuth LoRa broadcast packet
// Format: magic(1) id(6) name(up to 20, null-term) lat(4) lon(4) alt(2) hdg(2) spd(1) bat(1)
// Total: 1(magic)+6(id)+4(nonce)+encrypted(name+pos+alt+hdg+spd+bat+status+routePts+routeDist+wpLat+wpLon)
int loraBuildPacket(uint8_t* buf, int bufSize) {
  int i = 0;
  buf[i++] = 0xA9;
  memcpy(buf + i, loraDeviceId, 6); i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(buf + i, &nonce, 4); i += 4;
  int payloadStart = i;
  uint8_t plain[72] = {0};
  int pi = 0;
  // Name
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  uint8_t nameLen = (uint8_t)min((int)strlen(rawName), 20);
  memcpy(plain + pi, rawName, nameLen); plain[pi + nameLen] = 0; pi += nameLen + 1;
  // Position
  int32_t latI = (int32_t)(cachedLat * 1e6);
  int32_t lonI = (int32_t)(cachedLon * 1e6);
  memcpy(plain + pi, &latI, 4); pi += 4;
  memcpy(plain + pi, &lonI, 4); pi += 4;
  int16_t altI = (int16_t)constrain(cachedAlt, -1000, 32000);
  memcpy(plain + pi, &altI, 2); pi += 2;
  uint16_t hdgI = (uint16_t)(cachedHdg * 10);
  memcpy(plain + pi, &hdgI, 2); pi += 2;
  plain[pi++] = (uint8_t)constrain((int)cachedSpd, 0, 255);
  // Battery: 255 = USB power (no battery), 0–100 = battery %
  plain[pi++] = batOnUSB ? 255 : (uint8_t)constrain(batPercent, 0, 100);
  // Status bitfield: bit0=recording, bit1=waypointSaved, bit2=gpsFix
  uint8_t status = 0;
  if (routeRecording) status |= 0x01;
  if (locSaved)       status |= 0x02;
  if (cachedFix)      status |= 0x04;
  plain[pi++] = status;
  // Route stats
  uint16_t rPts = (uint16_t)min(routeCount, 65535);
  uint32_t rDist = (uint32_t)constrain((int)routeTotalDist, 0, 4294967295UL);
  memcpy(plain + pi, &rPts, 2);  pi += 2;
  memcpy(plain + pi, &rDist, 4); pi += 4;
  // Waypoint (only meaningful if status bit1 set)
  int32_t wpLatI = locSaved ? (int32_t)(savedLat * 1e6) : 0;
  int32_t wpLonI = locSaved ? (int32_t)(savedLon * 1e6) : 0;
  memcpy(plain + pi, &wpLatI, 4); pi += 4;
  memcpy(plain + pi, &wpLonI, 4); pi += 4;
  loraCrypt(plain, buf + payloadStart, pi, nonce);
  i += pi;
  return i;
}

// Send a LoRa message — broadcast (toId=nullptr) or direct (toId=6-char hex)
void loraSendMessage(const char* text, const char* toId) {
  if (!loraReady || !radio) return;
  uint8_t pkt[80];
  int i = 0;
  bool isDirect = (toId != nullptr && strlen(toId) == 6);
  pkt[i++] = isDirect ? LORA_MAGIC_MSG_DM : LORA_MAGIC_MSG_BC;
  // Device ID — unencrypted
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  if (isDirect) { memcpy(pkt + i, toId, 6); i += 6; }
  // Nonce — unencrypted
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4); i += 4;
  // Encrypted payload: name + null + text + null
  int payloadStart = i;
  uint8_t plain[64];
  int pi = 0;
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  uint8_t nameLen = (uint8_t)min((int)strlen(rawName), 20);
  memcpy(plain + pi, rawName, nameLen); plain[pi + nameLen] = 0; pi += nameLen + 1;
  uint8_t txtLen = (uint8_t)min((int)strlen(text), (int)LORA_MSG_MAX_LEN);
  memcpy(plain + pi, text, txtLen); plain[pi + txtLen] = 0; pi += txtLen + 1;
  loraCrypt(plain, pkt + payloadStart, pi, nonce);
  i += pi;

  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[LORA] MSG sent (%s): %s\n", isDirect ? toId : "BC", text);
    // Queue 2 repeats for broadcast (3 total) — improves delivery reliability
    if (!isDirect && i <= (int)sizeof(msgRepeatBuf)) {
      memcpy(msgRepeatBuf, pkt, i);
      msgRepeatLen    = i;
      msgRepeatCount  = 2;
      msgRepeatNextMs = millis() + 5000;
    }
  } else {
    Serial.printf("[LORA] MSG send failed: %d\n", state);
  }
}

// ── SOS ──────────────────────────────────────────────────────────────────────
void oledDraw(); // forward declaration — defined later in file
void oledDrawPage(); // forward declaration — for transitions
// Packet: [0xAE][fromId:6][nonce:4][encrypted: name+null + lat(4) + lon(4)]
// ── LoRa waypoint broadcast ───────────────────────────────────────────────────
// Sends all waypoints in packets of 2, sequentially with 200ms gaps.
// Packet: [0xAF][seq:1][total_wps:1][wp_lat:4][wp_lon:4][wp_name:13] × 2 = 45 bytes
void loraWaypointBroadcast() {
  if (!loraReady || !radio || waypointCount == 0) return;
  int packets = (waypointCount + 1) / 2;  // ceil(count/2)

  // Broadcast 3 times with 2s gap — improves reach for devices at marginal range
  for (int repeat = 0; repeat < 3; repeat++) {
    for (int seq = 0; seq < packets; seq++) {
      uint8_t pkt[48] = {0};
      int i = 0;
      pkt[i++] = LORA_MAGIC_WP;
      pkt[i++] = (uint8_t)seq;
      pkt[i++] = (uint8_t)waypointCount;
      for (int w = 0; w < 2; w++) {
        int idx = seq * 2 + w;
        if (idx >= waypointCount) break;
        memcpy(pkt + i, &waypoints[idx].lat,  4); i += 4;
        memcpy(pkt + i, &waypoints[idx].lon,  4); i += 4;
        memcpy(pkt + i, waypoints[idx].name, WP_NAME_LEN + 1); i += WP_NAME_LEN + 1;
      }
      radio->transmit(pkt, i);
      Serial.printf("[LORA] WP packet %d/%d (repeat %d/3) sent (%d bytes)\n", seq+1, packets, repeat+1, i);
      if (seq < packets - 1) delay(200);
    }
    if (repeat < 2) {
      // Re-enter receive mode between repeats so we can receive ACKs
      radio->startReceive();
      delay(2000);
    }
  }
  // Re-enter receive mode
  radio->startReceive();
}

void loraWaypointClearBroadcast() {
  if (!loraReady || !radio) return;
  uint8_t pkt[8] = {0};
  pkt[0] = LORA_MAGIC_WP_CLR;
  memcpy(pkt + 1, loraDeviceId, 6);
  radio->transmit(pkt, 7);
  Serial.println("[LORA] WP clear broadcast sent");
  radio->startReceive();
}

void loraSendWeather(const char* toId) {
  // Binary weather packet — compact and reliable
  // [0xB4][fromId:6][toId:6][nonce:4][t_x10:2][f_x10:2][mn_x10:2][mx_x10:2][h:1][w_x10:2][id:2][desc:24]
  // Total: 1+6+6+4+2+2+2+2+1+2+2+24 = 54 bytes
  if (!loraReady || !radio || !weather.valid) return;
  uint8_t pkt[64]; int i = 0;
  pkt[i++] = LORA_MAGIC_WEATHER;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  memcpy(pkt + i, toId, 6);         i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4);       i += 4;
  // Build binary payload
  uint8_t plain[40] = {0}; int pi = 0;
  int16_t t  = (int16_t)(weather.tempC      * 10);
  int16_t f  = (int16_t)(weather.feelsLikeC * 10);
  int16_t mn = (int16_t)(weather.tempMinC   * 10);
  int16_t mx = (int16_t)(weather.tempMaxC   * 10);
  int16_t w  = (int16_t)(weather.windKmh    * 10);
  uint16_t id = (uint16_t)weather.weatherId;
  uint8_t h   = (uint8_t)constrain(weather.humidity, 0, 100);
  memcpy(plain + pi, &t,  2); pi += 2;
  memcpy(plain + pi, &f,  2); pi += 2;
  memcpy(plain + pi, &mn, 2); pi += 2;
  memcpy(plain + pi, &mx, 2); pi += 2;
  plain[pi++] = h;
  memcpy(plain + pi, &w,  2); pi += 2;
  memcpy(plain + pi, &id, 2); pi += 2;
  strncpy((char*)(plain + pi), weather.desc, 23); plain[pi + 23] = 0; pi += 24;
  loraCrypt(plain, pkt + i, pi, nonce); i += pi;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  Serial.printf("[LORA] Weather sent to %s: %.1fC %s (%d bytes)\n", toId, weather.tempC, weather.desc, i);
}

void loraSOSBuild(uint8_t* pkt, int& len) {
  int i = 0;
  pkt[i++] = LORA_MAGIC_SOS;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4); i += 4;
  uint8_t plain[32] = {0};
  int pi = 0;
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  uint8_t nameLen = (uint8_t)min((int)strlen(rawName), 20);
  memcpy(plain + pi, rawName, nameLen); plain[pi + nameLen] = 0; pi += nameLen + 1;
  int32_t latI = (int32_t)(cachedLat * 1e6);
  int32_t lonI = (int32_t)(cachedLon * 1e6);
  memcpy(plain + pi, &latI, 4); pi += 4;
  memcpy(plain + pi, &lonI, 4); pi += 4;
  loraCrypt(plain, pkt + i, pi, nonce);
  i += pi;
  len = i;
}

void loraSOSStart() {
  if (sosActive) return;
  sosActive        = true;
  sosStartMs       = millis();
  sosBroadcastCount = 0;
  sosNextTxMs      = 0; // send immediately
  Serial.println("[LORA] SOS ACTIVATED");
  // Show SOS on OLED immediately
  oledDraw();
}

void loraSOSCancel() {
  if (!sosActive) return;
  sosActive = false;
  Serial.println("[LORA] SOS cancelled");
  oledDraw();
}

// Store incoming message in ring buffer and notify BLE
void loraStoreMessage(const char* fromId, const char* fromName, const char* text) {
  // Ring buffer — overwrite oldest
  int slot = loraMsgCount % LORA_MAX_MSGS;
  strncpy(loraMsgs[slot].fromId,   fromId,   sizeof(loraMsgs[slot].fromId)   - 1);
  strncpy(loraMsgs[slot].fromName, fromName, sizeof(loraMsgs[slot].fromName) - 1);
  strncpy(loraMsgs[slot].text,     text,     sizeof(loraMsgs[slot].text)     - 1);
  loraMsgs[slot].rxMs   = millis();
  loraMsgs[slot].read   = false;
  loraMsgs[slot].active = true;
  loraMsgCount++;
  loraMsgUnread++;
  loraMsgScroll = 0;  // reset scroll to show newest

  Serial.printf("[LORA] MSG from %s (%s): %s\n", fromId, fromName, text);

  // Notify BLE
  if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
    char json[192];
    snprintf(json, sizeof(json),
      "{\"msg\":1,\"from\":\"%s\",\"name\":\"%s\",\"text\":\"%s\"}",
      fromId, fromName, text);
    pPeerChar->setValue((uint8_t*)json, strlen(json));
    pPeerChar->notify();
  }
}

// ══════════════════════════════════════════════════════════════════
// RELAY MODE — forward packets for other devices
// ══════════════════════════════════════════════════════════════════

// Simple hash of packet for deduplication (FNV-1a)
uint32_t relayHashPacket(uint8_t* buf, int len) {
  uint32_t hash = 2166136261u;
  for (int i = 0; i < len; i++) {
    hash ^= buf[i];
    hash *= 16777619u;
  }
  return hash;
}

// Check if we've seen this packet recently
bool relayHaveSeen(uint32_t hash) {
  uint32_t now = millis();
  for (int i = 0; i < relaySeenCount; i++) {
    if (relaySeenHashes[i] == hash && (now - relaySeenTimes[i]) < RELAY_SEEN_EXPIRE) {
      return true;
    }
  }
  return false;
}

// Mark packet as seen
void relayMarkSeen(uint32_t hash) {
  uint32_t now = millis();
  // Find expired slot or use next slot
  int slot = -1;
  for (int i = 0; i < relaySeenCount; i++) {
    if ((now - relaySeenTimes[i]) >= RELAY_SEEN_EXPIRE) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    if (relaySeenCount < RELAY_MAX_SEEN) {
      slot = relaySeenCount++;
    } else {
      // Overwrite oldest
      uint32_t oldest = 0;
      int oldestIdx = 0;
      for (int i = 0; i < RELAY_MAX_SEEN; i++) {
        uint32_t age = now - relaySeenTimes[i];
        if (age > oldest) { oldest = age; oldestIdx = i; }
      }
      slot = oldestIdx;
    }
  }
  relaySeenHashes[slot] = hash;
  relaySeenTimes[slot] = now;
}

// Relay a packet with random delay to avoid collisions
// Returns true if packet was queued for relay
bool relayPacket(uint8_t* buf, int len, uint8_t magic) {
  if (!relayEnabled || !loraReady || !radio) return false;
  if (len < 7 || len > 200) return false;
  
  // Only relay certain packet types
  // Position (0xA9), Broadcast msg (0xAB), SOS (0xAE)
  if (magic != LORA_MAGIC && magic != LORA_MAGIC_MSG_BC && magic != LORA_MAGIC_SOS) {
    return false;
  }
  
  // Check if from us (don't relay own packets)
  char fromId[8] = {0};
  memcpy(fromId, buf + 1, 6);
  if (strcmp(fromId, loraDeviceId) == 0) return false;
  
  // Check deduplication
  uint32_t hash = relayHashPacket(buf, len);
  if (relayHaveSeen(hash)) {
    Serial.printf("[RELAY] Skip duplicate from %s\n", fromId);
    return false;
  }
  relayMarkSeen(hash);
  
  // Random delay before retransmit
  uint32_t delayMs = RELAY_DELAY_MIN + (esp_random() % (RELAY_DELAY_MAX - RELAY_DELAY_MIN));
  delay(delayMs);
  
  // Retransmit the packet unchanged
  radio->clearDio1Action();
  int state = radio->transmit(buf, len);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  
  if (state == RADIOLIB_ERR_NONE) {
    relayCount++;
    relayLastTx = millis();
    Serial.printf("[RELAY] Forwarded %d bytes from %s (delay %lums)\n", len, fromId, delayMs);
    return true;
  }
  return false;
}

// ══════════════════════════════════════════════════════════════════
// STORE-AND-FORWARD — queue messages for offline peers
// ══════════════════════════════════════════════════════════════════

// Store a message for later delivery to an offline peer
bool storeFwdAdd(const char* toId, const char* fromId, const char* fromName, const char* text) {
  if (!relayEnabled) return false;  // only store when in relay mode
  
  // Find empty slot or expire oldest
  int slot = -1;
  uint32_t now = millis();
  uint32_t oldest = 0;
  int oldestIdx = 0;
  
  for (int i = 0; i < STORE_FWD_MAX; i++) {
    if (!storedMsgs[i].active) {
      slot = i;
      break;
    }
    // Check for expired
    if ((now - storedMsgs[i].storedAt) >= STORE_FWD_EXPIRE) {
      slot = i;
      break;
    }
    // Track oldest for overwrite
    uint32_t age = now - storedMsgs[i].storedAt;
    if (age > oldest) { oldest = age; oldestIdx = i; }
  }
  
  if (slot < 0) {
    // Overwrite oldest
    slot = oldestIdx;
    Serial.printf("[STORE] Overwriting oldest message for %s\n", storedMsgs[slot].toId);
  }
  
  storedMsgs[slot].active = true;
  strncpy(storedMsgs[slot].toId,     toId,     7);
  strncpy(storedMsgs[slot].fromId,   fromId,   7);
  strncpy(storedMsgs[slot].fromName, fromName, 19);
  strncpy(storedMsgs[slot].text,     text,     LORA_MSG_MAX_LEN);
  storedMsgs[slot].storedAt = now;
  storedMsgs[slot].attempts = 0;
  storedMsgCount++;
  
  Serial.printf("[STORE] Queued msg for %s from %s: %s\n", toId, fromId, text);
  return true;
}

// Check if we have stored messages for a peer and deliver them
void storeFwdCheckDeliver(const char* peerId) {
  if (!relayEnabled || !loraReady || !radio) return;
  
  uint32_t now = millis();
  for (int i = 0; i < STORE_FWD_MAX; i++) {
    if (!storedMsgs[i].active) continue;
    if ((now - storedMsgs[i].storedAt) >= STORE_FWD_EXPIRE) {
      storedMsgs[i].active = false;
      continue;
    }
    if (strcmp(storedMsgs[i].toId, peerId) != 0) continue;
    
    // Found a message for this peer — deliver it
    Serial.printf("[STORE] Delivering queued msg to %s from %s\n", 
      peerId, storedMsgs[i].fromId);
    
    // Small delay to let peer's position packet complete
    delay(100 + (esp_random() % 150));
    
    // Send as direct message
    loraSendMessage(storedMsgs[i].text, peerId);
    
    storedMsgs[i].attempts++;
    if (storedMsgs[i].attempts >= 2) {
      // Mark delivered after 2 attempts
      storedMsgs[i].active = false;
      storedMsgCount--;
    }
  }
}

// Check if a direct message target is offline — if so, store it
bool storeFwdCheckStore(const char* toId, const char* fromId, const char* fromName, const char* text) {
  if (!relayEnabled) return false;
  
  // Check if target is in our peer list and recently seen
  uint32_t now = millis();
  for (int p = 0; p < loraPeerCount; p++) {
    if (strcmp(loraPeers[p].id, toId) == 0) {
      // Peer known — check if recently seen (within 2 minutes)
      if ((now - loraPeers[p].lastSeenMs) < 120000) {
        return false;  // Peer is online, don't store
      }
    }
  }
  
  // Peer not seen recently — store for later
  return storeFwdAdd(toId, fromId, fromName, text);
}

// ══════════════════════════════════════════════════════════════════

// ── Device Registration Protocol ─────────────────────────────────
// For fell race / event scenarios where devices are handed out on the day
// Leader broadcasts discover (0xB2), devices respond (0xB3) with random backoff
// Leader assigns names (0xB5), devices ACK (0xB6)

// Send registration discover request — "who's there?"
// Packet: [0xB2][fromId:6]
void loraRegDiscover() {
  uint8_t pkt[8];
  int i = 0;
  pkt[i++] = LORA_MAGIC_REG_DISCOVER;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  Serial.printf("[LORA] Sent registration discover (%d bytes)\n", i);
}

// Schedule registration response with random backoff (called when 0xB2 received)
uint32_t regResponseScheduledMs = 0;
bool     regResponsePending     = false;

void loraRegScheduleResponse() {
  // Random backoff 50-550ms to avoid collisions when many devices respond
  regResponseScheduledMs = millis() + 50 + (esp_random() % 500);
  regResponsePending = true;
  Serial.printf("[LORA] Scheduling reg response in %lums\n", 
    regResponseScheduledMs - millis());
}

// Send registration response — device ID + current name
// Packet: [0xB3][deviceId:6][nonce:4][encrypted: name+null]
void loraRegResponse() {
  regResponsePending = false;
  uint8_t pkt[40];
  int i = 0;
  pkt[i++] = LORA_MAGIC_REG_RESPONSE;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4); i += 4;
  // Encrypt name (strip Azimuth- prefix)
  uint8_t plain[24] = {0};
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  uint8_t nameLen = (uint8_t)min((int)strlen(rawName), 16);
  memcpy(plain, rawName, nameLen); plain[nameLen] = 0;
  loraCrypt(plain, pkt + i, nameLen + 1, nonce);
  i += nameLen + 1;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  Serial.printf("[LORA] Sent reg response: %s (%d bytes)\n", rawName, i);
}

// Send name assignment to specific device
// Packet: [0xB5][fromId:6][toId:6][nonce:4][encrypted: newName+null]
void loraRegAssignName(const char* toId, const char* newName) {
  uint8_t pkt[48];
  int i = 0;
  pkt[i++] = LORA_MAGIC_REG_ASSIGN;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  memcpy(pkt + i, toId, 6); i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4); i += 4;
  // Encrypt new name
  uint8_t plain[24] = {0};
  uint8_t nameLen = (uint8_t)min((int)strlen(newName), 16);
  memcpy(plain, newName, nameLen); plain[nameLen] = 0;
  loraCrypt(plain, pkt + i, nameLen + 1, nonce);
  i += nameLen + 1;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  Serial.printf("[LORA] Sent name assign to %s: %s (%d bytes)\n", toId, newName, i);
}

// Send name assignment ACK — confirms name was saved
// Packet: [0xB6][deviceId:6][nonce:4][encrypted: confirmedName+null]
void loraRegAck(const char* confirmedName) {
  uint8_t pkt[40];
  int i = 0;
  pkt[i++] = LORA_MAGIC_REG_ACK;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4); i += 4;
  // Encrypt confirmed name
  uint8_t plain[24] = {0};
  uint8_t nameLen = (uint8_t)min((int)strlen(confirmedName), 16);
  memcpy(plain, confirmedName, nameLen); plain[nameLen] = 0;
  loraCrypt(plain, pkt + i, nameLen + 1, nonce);
  i += nameLen + 1;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  Serial.printf("[LORA] Sent reg ACK: %s (%d bytes)\n", confirmedName, i);
}

// ── Key Share — UNENCRYPTED broadcast ────────────────────────────────────────
// Used to share network key with devices that don't yet have it
// Recipient shows consent prompt; they must accept to save the key
// Packet: [0xB7][fromId:6][key:16][leaderName:16][groupName:32] = 71 bytes (UNENCRYPTED)
void loraKeyShare() {
  uint8_t pkt[80];
  int i = 0;
  pkt[i++] = LORA_MAGIC_KEY_SHARE;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  memcpy(pkt + i, loraKey, LORA_KEY_LEN); i += LORA_KEY_LEN;
  // Add leader name (strip Azimuth- prefix)
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  char nameBuf[17] = {0};
  strncpy(nameBuf, rawName, 16);
  memcpy(pkt + i, nameBuf, 16); i += 16;
  // Add group name
  char groupBuf[33] = {0};
  strncpy(groupBuf, keyShareGroupName, 32);
  memcpy(pkt + i, groupBuf, 32); i += 32;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  // Track state for Groups page
  keyShareActive = true;
  keyShareAckCount = 0;
  keyShareSentMs = millis();
  Serial.printf("[LORA] KEY SHARE broadcast (%d bytes) — key sent in clear!\n", i);
}

// Cancel a pending key share request — sent by leader if wrong key etc.
// Packet: [0xB8][fromId:6] = 7 bytes (UNENCRYPTED)
void loraKeyShareCancel() {
  uint8_t pkt[8];
  int i = 0;
  pkt[i++] = LORA_MAGIC_KEY_CANCEL;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  // Clear state
  keyShareActive = false;
  keyShareAckCount = 0;
  Serial.printf("[LORA] KEY SHARE CANCEL broadcast (%d bytes)\n", i);
}

// Send key share ACK — device accepted the key
// Packet: [0xB9][deviceId:6][nonce:4][encrypted: name+null]
void loraKeyShareAck() {
  uint8_t pkt[40];
  int i = 0;
  pkt[i++] = LORA_MAGIC_KEY_ACK;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4); i += 4;
  // Encrypt name (use the NEW key that was just saved)
  uint8_t plain[24] = {0};
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  uint8_t nameLen = (uint8_t)min((int)strlen(rawName), 16);
  memcpy(plain, rawName, nameLen); plain[nameLen] = 0;
  loraCrypt(plain, pkt + i, nameLen + 1, nonce);
  i += nameLen + 1;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  Serial.printf("[LORA] KEY SHARE ACK sent (%d bytes)\n", i);
}

// Broadcast that this device is leaving the group (sent before key change)
void loraGroupLeave() {
  uint8_t pkt[40];
  int i = 0;
  pkt[i++] = LORA_MAGIC_GROUP_LEAVE;
  memcpy(pkt + i, loraDeviceId, 6); i += 6;
  uint32_t nonce = (uint32_t)millis();
  memcpy(pkt + i, &nonce, 4); i += 4;
  // Encrypt name so others know who left
  uint8_t plain[24] = {0};
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  uint8_t nameLen = (uint8_t)min((int)strlen(rawName), 16);
  memcpy(plain, rawName, nameLen); plain[nameLen] = 0;
  loraCrypt(plain, pkt + i, nameLen + 1, nonce);
  i += nameLen + 1;
  radio->clearDio1Action();
  int state = radio->transmit(pkt, i);
  radio->setDio1Action(loraRxISR);
  radio->startReceive();
  loraRxFlag = false;
  Serial.printf("[LORA] GROUP LEAVE broadcast sent (%d bytes)\n", i);
}

// Accept pending key share — save key, send ACK, clear pending
void keyShareAccept() {
  if (!pendingKeyShare.active) return;
  
  // Add to stored groups
  storedGroupAdd(pendingKeyShare.groupName, pendingKeyShare.key, pendingKeyShare.leaderName, false);
  
  // Find the new group index and activate it
  for (int i = 0; i < storedGroupCount; i++) {
    if (memcmp(storedGroups[i].key, pendingKeyShare.key, LORA_KEY_LEN) == 0) {
      storedGroupActivate(i);
      break;
    }
  }
  
  Serial.printf("[LORA] Key share ACCEPTED — joined '%s' from %s\n", pendingKeyShare.groupName, pendingKeyShare.leaderName);
  // Send ACK to leader
  loraKeyShareAck();
  
  // Show confirmation on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println("Joined group!");
  display.println();
  display.println(pendingKeyShare.groupName);
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("Via: ");
  display.println(pendingKeyShare.leaderName);
  display.display();
  delay(2000);
  
  // Clear pending (RAM + NVS)
  pendingKeyShareClear();
  
  // Stay on Groups page to show the new group
  oledDraw();
  
  // Notify BLE
  if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
    char json[100];
    snprintf(json, sizeof(json), "{\"keyAccepted\":1,\"leader\":\"%s\",\"group\":\"%s\"}", pendingKeyShare.leaderName, pendingKeyShare.groupName);
    pPeerChar->setValue((uint8_t*)json, strlen(json));
    pPeerChar->notify();
  }
}

// Decline pending key share — just clear it
void keyShareDecline() {
  if (!pendingKeyShare.active) return;
  Serial.printf("[LORA] Key share DECLINED from %s\n", pendingKeyShare.leaderName);
  pendingKeyShareClear();  // clear RAM + NVS
  oledDraw();
  // Notify BLE
  if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
    pPeerChar->setValue("{\"keyDeclined\":1}");
    pPeerChar->notify();
  }
}

// Parse an incoming LoRa packet — position or message
void loraHandlePacket(uint8_t* buf, int len) {
  if (len < 7) return;
  uint8_t magic = buf[0];

  // ── Waypoint packets ────────────────────────────────────────
  if (magic == LORA_MAGIC_WP) {
    if (len < 3) return;
    int i = 1;
    uint8_t seq      = buf[i++];
    uint8_t totalWps = buf[i++];
    if (totalWps == 0 || totalWps > WP_MAX) return;
    // Parse up to 2 waypoints from this packet
    bool changed = false;
    while (i + 21 <= len) {
      int idx = seq * 2 + (i == 3 ? 0 : 1);
      if (idx >= WP_MAX || idx >= totalWps) break;
      float lat, lon;
      memcpy(&lat, buf + i, 4); i += 4;
      memcpy(&lon, buf + i, 4); i += 4;
      waypoints[idx].lat = lat;
      waypoints[idx].lon = lon;
      memcpy(waypoints[idx].name, buf + i, WP_NAME_LEN + 1);
      waypoints[idx].name[WP_NAME_LEN] = 0;  // ensure null terminated
      i += WP_NAME_LEN + 1;
      changed = true;
    }
    // On last packet — update count, save, notify BLE
    int lastSeq = (totalWps + 1) / 2 - 1;
    if ((int)seq == lastSeq && changed) {
      waypointCount   = totalWps;
      waypointCurrent = 0;
      waypointSave();
      Serial.printf("[LORA] Waypoints received: %d — sending ACK\n", waypointCount);
      // Send ACK back: [0xB0][deviceId:6][wpCount:1]
      uint8_t ack[8] = {0};
      ack[0] = LORA_MAGIC_WP_ACK;
      memcpy(ack + 1, loraDeviceId, 6);
      ack[7] = (uint8_t)waypointCount;
      radio->startReceive();
      delay(50 + random(200));  // small random delay to avoid collisions
      radio->transmit(ack, 8);
      radio->startReceive();
      // Notify BLE
      if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
        char json[32];
        snprintf(json, sizeof(json), "{\"wps\":%d}", waypointCount);
        pPeerChar->setValue((uint8_t*)json, strlen(json));
        pPeerChar->notify();
      }
    }
    return;
  }

  // ── Waypoint clear ──────────────────────────────────────────────────────────
  if (magic == LORA_MAGIC_WP_CLR) {
    if (len < 7) return;
    char fromId[8] = {0};
    memcpy(fromId, buf + 1, 6); fromId[6] = 0;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    waypointClear();
    Serial.printf("[LORA] WP clear from %s — waypoints cleared\n", fromId);
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      pPeerChar->setValue("{\"wps\":0}");
      pPeerChar->notify();
    }
    return;
  }

  // ── Weather push (0xB4) ─────────────────────────────────────────────────────
  if (magic == LORA_MAGIC_WEATHER) {
    // Binary format: [0xB4][fromId:6][toId:6][nonce:4][encrypted binary payload]
    if (len < 30) return;
    int i = 1;
    char fromId[8] = {0}; memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    char toId[8]   = {0}; memcpy(toId,   buf + i, 6); toId[6]   = 0; i += 6;
    if (strcmp(toId, loraDeviceId) != 0) return;
    if (strcmp(fromId, loraDeviceId) == 0) return;
    uint32_t nonce; memcpy(&nonce, buf + i, 4); i += 4;
    int payloadLen = len - i;
    uint8_t plain[40] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    // Unpack binary fields
    int pi = 0;
    int16_t t, f, mn, mx, w; uint16_t id; uint8_t h;
    memcpy(&t,  plain + pi, 2); pi += 2;
    memcpy(&f,  plain + pi, 2); pi += 2;
    memcpy(&mn, plain + pi, 2); pi += 2;
    memcpy(&mx, plain + pi, 2); pi += 2;
    h = plain[pi++];
    memcpy(&w,  plain + pi, 2); pi += 2;
    memcpy(&id, plain + pi, 2); pi += 2;
    weather.tempC       = t  / 10.0f;
    weather.feelsLikeC  = f  / 10.0f;
    weather.tempMinC    = mn / 10.0f;
    weather.tempMaxC    = mx / 10.0f;
    weather.windKmh     = w  / 10.0f;
    weather.weatherId   = id;
    weather.humidity    = h;
    strncpy(weather.desc, (char*)(plain + pi), sizeof(weather.desc) - 1);
    weather.desc[sizeof(weather.desc) - 1] = 0;
    weather.valid = true; weatherReceived = true;
    if (gps.time.isValid()) {
      int fh, fm, fs, fd, fmo, fy;
      getLocalTime(fh, fm, fs, fd, fmo, fy);
      wifiLastFetchHour = fh;
      wifiLastFetchMin  = fm;
    }
    Serial.printf("[LORA] Weather from %s: %.1fC %s\n", fromId, weather.tempC, weather.desc);
    oledRedrawPending = true;
    return;
  }

  // ── Waypoint ACK ────────────────────────────────────────────────────────────
  if (magic == LORA_MAGIC_WP_ACK) {
    if (len < 8) return;
    char fromId[8] = {0};
    memcpy(fromId, buf + 1, 6); fromId[6] = 0;
    uint8_t wpsConfirmed = buf[7];
    Serial.printf("[LORA] WP ACK from %s (%d wps)\n", fromId, wpsConfirmed);
    // Notify web app so it can update the confirmation list
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      char json[64];
      snprintf(json, sizeof(json), "{\"wpAck\":\"%s\",\"wps\":%d}", fromId, wpsConfirmed);
      pPeerChar->setValue((uint8_t*)json, strlen(json));
      pPeerChar->notify();
    }
    return;
  }

  // ── Registration: Discover request (0xB2) ─────────────────────────────────────
  // Leader asking "who's there?" — schedule response with random backoff
  if (magic == LORA_MAGIC_REG_DISCOVER) {
    if (len < 7) return;
    char fromId[8] = {0};
    memcpy(fromId, buf + 1, 6); fromId[6] = 0;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    Serial.printf("[LORA] Reg discover from %s — scheduling response\n", fromId);
    loraRegScheduleResponse();
    return;
  }

  // ── Registration: Response (0xB3) ─────────────────────────────────────────────
  // Device responding with ID + name — forward to web app
  if (magic == LORA_MAGIC_REG_RESPONSE) {
    if (len < 11) return;
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    uint32_t nonce;
    memcpy(&nonce, buf + i, 4); i += 4;
    // Decrypt name
    int payloadLen = len - i;
    uint8_t plain[24] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    char name[20] = {0};
    int nameEnd = 0;
    while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 16) nameEnd++;
    memcpy(name, plain, nameEnd);
    Serial.printf("[LORA] Reg response from %s: %s\n", fromId, name);
    // Notify web app
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      char json[80];
      snprintf(json, sizeof(json), "{\"regResp\":\"%s\",\"name\":\"%s\"}", fromId, name);
      pPeerChar->setValue((uint8_t*)json, strlen(json));
      pPeerChar->notify();
    }
    return;
  }

  // ── Registration: Assign name (0xB5) ──────────────────────────────────────────
  // Leader assigning new name to this device
  if (magic == LORA_MAGIC_REG_ASSIGN) {
    if (len < 17) return;
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    char toId[8] = {0};
    memcpy(toId, buf + i, 6); toId[6] = 0; i += 6;
    if (strcmp(toId, loraDeviceId) != 0) return; // not for us
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    uint32_t nonce;
    memcpy(&nonce, buf + i, 4); i += 4;
    // Decrypt new name
    int payloadLen = len - i;
    uint8_t plain[24] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    char newName[20] = {0};
    int nameEnd = 0;
    while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 16) nameEnd++;
    memcpy(newName, plain, nameEnd);
    Serial.printf("[LORA] Reg assign from %s: new name = %s\n", fromId, newName);
    // Save original name if not already saved (for restoring when leaving group)
    if (strlen(originalDeviceName) == 0) {
      strncpy(originalDeviceName, bleDeviceName, sizeof(originalDeviceName) - 1);
      prefs.begin("azimuth", false);
      prefs.putString("origName", originalDeviceName);
      prefs.end();
      Serial.printf("[LORA] Original name saved: %s\n", originalDeviceName);
    }
    // Save new name with Azimuth- prefix
    snprintf(bleDeviceName, sizeof(bleDeviceName), "Azimuth-%s", newName);
    prefs.begin("azimuth", false);
    prefs.putString("devName", bleDeviceName);
    prefs.end();
    // Send ACK
    loraRegAck(newName);
    // Show confirmation on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("Name assigned:");
    display.println(bleDeviceName);
    display.display();
    delay(1500);
    oledDraw();
    return;
  }

  // ── Key Share (0xB7) — UNENCRYPTED ────────────────────────────────────────────
  // Leader sharing network key — store as pending, require user consent
  // Packet: [0xB7][fromId:6][key:16][leaderName:16][groupName:32] = 71 bytes
  if (magic == LORA_MAGIC_KEY_SHARE) {
    if (len < 23) return;  // minimum: 1 + 6 + 16
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    // Extract key (unencrypted)
    uint8_t newKey[LORA_KEY_LEN];
    memcpy(newKey, buf + i, LORA_KEY_LEN); i += LORA_KEY_LEN;
    // Extract leader name if present
    char leaderName[20] = {0};
    if (i + 16 <= len) {
      memcpy(leaderName, buf + i, 16);
      leaderName[16] = 0;
      i += 16;
    } else {
      strncpy(leaderName, fromId, sizeof(leaderName) - 1); // fallback to ID
    }
    // Extract group name if present
    char groupName[32] = {0};
    if (i + 32 <= len) {
      memcpy(groupName, buf + i, 32);
      groupName[31] = 0;
    } else {
      strcpy(groupName, "Group"); // fallback
    }
    // Store as pending request
    pendingKeyShare.active = true;
    strncpy(pendingKeyShare.fromId, fromId, sizeof(pendingKeyShare.fromId) - 1);
    strncpy(pendingKeyShare.leaderName, leaderName, sizeof(pendingKeyShare.leaderName) - 1);
    strncpy(pendingKeyShare.groupName, groupName, sizeof(pendingKeyShare.groupName) - 1);
    memcpy(pendingKeyShare.key, newKey, LORA_KEY_LEN);
    pendingKeyShare.receivedMs = millis();
    pendingKeyShareSave();  // persist to NVS
    Serial.printf("[LORA] KEY SHARE request from %s (%s) group '%s' — awaiting user consent\n", fromId, leaderName, groupName);
    // Wake screen and jump to Groups page
    wakeScreen();
    currentPage = pageGroups();
    oledDraw();
    // Notify BLE if connected
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      char json[120];
      snprintf(json, sizeof(json), "{\"keyReq\":1,\"from\":\"%s\",\"leader\":\"%s\",\"group\":\"%s\"}", fromId, leaderName, groupName);
      pPeerChar->setValue((uint8_t*)json, strlen(json));
      pPeerChar->notify();
    }
    return;
  }

  // ── Key Share Cancel (0xB8) — UNENCRYPTED ─────────────────────────────────────
  // Leader cancelling a pending key share request
  if (magic == LORA_MAGIC_KEY_CANCEL) {
    if (len < 7) return;
    char fromId[8] = {0};
    memcpy(fromId, buf + 1, 6); fromId[6] = 0;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    // Only clear if from same leader
    if (pendingKeyShare.active && strcmp(pendingKeyShare.fromId, fromId) == 0) {
      Serial.printf("[LORA] KEY SHARE CANCELLED by %s\n", fromId);
      pendingKeyShareClear();  // clear RAM + NVS
      oledDraw();
      // Notify BLE
      if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
        pPeerChar->setValue("{\"keyCancelled\":1}");
        pPeerChar->notify();
      }
    }
    return;
  }

  // ── Key Share ACK (0xB9) — device accepted ────────────────────────────────────
  // Forward to web app so leader can track who accepted
  if (magic == LORA_MAGIC_KEY_ACK) {
    if (len < 11) return;
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    uint32_t nonce;
    memcpy(&nonce, buf + i, 4); i += 4;
    // Decrypt name
    int payloadLen = len - i;
    uint8_t plain[24] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    char name[20] = {0};
    int nameEnd = 0;
    while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 16) nameEnd++;
    memcpy(name, plain, nameEnd);
    Serial.printf("[LORA] KEY ACK from %s: %s accepted\n", fromId, name);
    // Track ACK count for Groups page
    keyShareAckCount++;
    // Notify web app
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      char json[80];
      snprintf(json, sizeof(json), "{\"keyAck\":\"%s\",\"name\":\"%s\"}", fromId, name);
      pPeerChar->setValue((uint8_t*)json, strlen(json));
      pPeerChar->notify();
    }
    return;
  }

  // ── Group Leave (0xBA) — device leaving group ────────────────────────────────────
  // Forward to web app so it can update device list
  if (magic == LORA_MAGIC_GROUP_LEAVE) {
    if (len < 11) return;
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    uint32_t nonce;
    memcpy(&nonce, buf + i, 4); i += 4;
    // Decrypt name
    int payloadLen = len - i;
    uint8_t plain[24] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    char name[20] = {0};
    int nameEnd = 0;
    while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 16) nameEnd++;
    memcpy(name, plain, nameEnd);
    Serial.printf("[LORA] GROUP LEAVE from %s: %s\n", fromId, name);
    // Notify web app
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      char json[80];
      snprintf(json, sizeof(json), "{\"groupLeave\":\"%s\",\"name\":\"%s\"}", fromId, name);
      pPeerChar->setValue((uint8_t*)json, strlen(json));
      pPeerChar->notify();
    }
    return;
  }

  // ── Registration: ACK (0xB6) ──────────────────────────────────────────────────
  // Device confirming name assignment — forward to web app
  if (magic == LORA_MAGIC_REG_ACK) {
    if (len < 11) return;
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    uint32_t nonce;
    memcpy(&nonce, buf + i, 4); i += 4;
    // Decrypt confirmed name
    int payloadLen = len - i;
    uint8_t plain[24] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    char name[20] = {0};
    int nameEnd = 0;
    while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 16) nameEnd++;
    memcpy(name, plain, nameEnd);
    Serial.printf("[LORA] Reg ACK from %s: %s\n", fromId, name);
    // Notify web app
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      char json[80];
      snprintf(json, sizeof(json), "{\"regAck\":\"%s\",\"name\":\"%s\"}", fromId, name);
      pPeerChar->setValue((uint8_t*)json, strlen(json));
      pPeerChar->notify();
    }
    return;
  }

  // ── SOS packet ─────────────────────────────────────────────
  if (magic == LORA_MAGIC_SOS) {
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    if (strcmp(fromId, loraDeviceId) == 0) return; // own packet
    if (i + LORA_NONCE_LEN > len) return;
    uint32_t nonce;
    memcpy(&nonce, buf + i, LORA_NONCE_LEN); i += LORA_NONCE_LEN;
    int payloadLen = len - i;
    uint8_t plain[32] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    // Parse name and position
    int pi = 0;
    char fromName[24] = {0};
    int nameEnd = 0;
    while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 20) nameEnd++;
    memcpy(fromName, plain, nameEnd); pi = nameEnd + 1;
    double lat = 0, lon = 0;
    if (pi + 8 <= payloadLen) {
      int32_t latI, lonI;
      memcpy(&latI, plain + pi, 4); pi += 4;
      memcpy(&lonI, plain + pi, 4); pi += 4;
      lat = latI / 1e6; lon = lonI / 1e6;
    }
    Serial.printf("[LORA] SOS from %s (%s) lat=%.5f lon=%.5f\n", fromId, fromName, lat, lon);
    // Flash LED rapidly to alert
    for (int f = 0; f < 8; f++) {
      digitalWrite(LED_PIN, HIGH); delay(SOS_LED_FLASH_MS);
      digitalWrite(LED_PIN, LOW);  delay(SOS_LED_FLASH_MS);
    }
    // Notify BLE — special SOS JSON
    if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
      char json[160];
      snprintf(json, sizeof(json),
        "{\"sos\":1,\"peer\":\"%s\",\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f}",
        fromId, fromName, lat, lon);
      pPeerChar->setValue((uint8_t*)json, strlen(json));
      pPeerChar->notify();
    }
    // Relay SOS packets (high priority)
    relayPacket(buf, len, magic);
    return;
  }

  // ── Message packets ─────────────────────────────────────────
  if (magic == LORA_MAGIC_MSG_BC || magic == LORA_MAGIC_MSG_DM) {
    int i = 1;
    char fromId[8] = {0};
    memcpy(fromId, buf + i, 6); fromId[6] = 0; i += 6;
    if (strcmp(fromId, loraDeviceId) == 0) return;  // own packet
    if (magic == LORA_MAGIC_MSG_DM) {
      char toId[8] = {0};
      memcpy(toId, buf + i, 6); toId[6] = 0; i += 6;
      if (strcmp(toId, loraDeviceId) != 0) return;  // not for us
    }
    // Read nonce then decrypt remainder
    if (i + LORA_NONCE_LEN > len) return;
    uint32_t nonce;
    memcpy(&nonce, buf + i, LORA_NONCE_LEN); i += LORA_NONCE_LEN;

    // Deduplicate repeated broadcasts — same fromId + nonce = repeat, ignore
    static char   lastMsgId[8]   = {0};
    static uint32_t lastMsgNonce = 0;
    if (strcmp(fromId, lastMsgId) == 0 && nonce == lastMsgNonce) return;
    strncpy(lastMsgId, fromId, 7); lastMsgNonce = nonce;

    int payloadLen = len - i;
    uint8_t plain[72] = {0};
    loraCrypt(buf + i, plain, payloadLen, nonce);
    // Parse from plaintext
    int pi = 0;
    char fromName[24] = {0};
    int nameEnd = 0;
    while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 20) nameEnd++;
    memcpy(fromName, plain, nameEnd); pi = nameEnd + 1;
    char text[LORA_MSG_MAX_LEN + 1] = {0};
    int txtEnd = pi;
    while (txtEnd < payloadLen && plain[txtEnd] != 0 && (txtEnd - pi) < LORA_MSG_MAX_LEN) txtEnd++;
    memcpy(text, plain + pi, txtEnd - pi);
    loraStoreMessage(fromId, fromName, text);
    // Relay broadcast messages
    if (magic == LORA_MAGIC_MSG_BC) {
      relayPacket(buf, len, magic);
    }
    return;
  }

  // ── Position packets ───────────────────────────────────────
  if (magic != LORA_MAGIC) return;
  if (len < 15) return;
  int i = 1;
  // Device ID — unencrypted
  char id[8] = {0};
  memcpy(id, buf + i, 6); id[6] = 0; i += 6;
  if (strcmp(id, loraDeviceId) == 0) return;  // own packet
  // Nonce — unencrypted
  if (i + LORA_NONCE_LEN > len) return;
  uint32_t nonce;
  memcpy(&nonce, buf + i, LORA_NONCE_LEN); i += LORA_NONCE_LEN;
  // Decrypt remainder
  int payloadLen = len - i;
  uint8_t plain[48] = {0};
  loraCrypt(buf + i, plain, payloadLen, nonce);
  // Parse from plaintext
  int pi = 0;
  char name[24] = {0};
  int nameEnd = 0;
  while (nameEnd < payloadLen && plain[nameEnd] != 0 && nameEnd < 20) nameEnd++;
  memcpy(name, plain, nameEnd); pi = nameEnd + 1;
  if (pi + 14 > payloadLen) return;
  int32_t latI, lonI;
  memcpy(&latI, plain + pi, 4); pi += 4;
  memcpy(&lonI, plain + pi, 4); pi += 4;
  double lat = latI / 1e6;
  double lon = lonI / 1e6;
  int16_t altI;
  memcpy(&altI, plain + pi, 2); pi += 2;
  uint16_t hdgI;
  memcpy(&hdgI, plain + pi, 2); pi += 2;
  uint8_t spd = (pi < payloadLen) ? plain[pi++] : 0;
  uint8_t bat = (pi < payloadLen) ? plain[pi++] : 0;
  // Extended fields (v302+) — gracefully absent from older firmware
  uint8_t  status    = (pi < payloadLen) ? plain[pi++] : 0;
  uint16_t routePts  = 0;
  uint32_t routeDist = 0;
  double   wpLat = 0, wpLon = 0;
  if (pi + 6 <= payloadLen) {
    memcpy(&routePts,  plain + pi, 2); pi += 2;
    memcpy(&routeDist, plain + pi, 4); pi += 4;
  }
  if ((status & 0x02) && pi + 8 <= payloadLen) {
    int32_t wpLatI, wpLonI;
    memcpy(&wpLatI, plain + pi, 4); pi += 4;
    memcpy(&wpLonI, plain + pi, 4); pi += 4;
    wpLat = wpLatI / 1e6;
    wpLon = wpLonI / 1e6;
  }

  // Find existing peer slot or claim empty one
  int slot = -1;
  for (int p = 0; p < LORA_MAX_PEERS; p++) {
    if (loraPeers[p].active && strcmp(loraPeers[p].id, id) == 0) { slot = p; break; }
  }
  if (slot < 0) {
    for (int p = 0; p < LORA_MAX_PEERS; p++) {
      if (!loraPeers[p].active) { slot = p; break; }
    }
    if (slot < 0) {
      uint32_t oldest = millis();
      for (int p = 0; p < LORA_MAX_PEERS; p++) {
        if (loraPeers[p].lastSeenMs < oldest) { oldest = loraPeers[p].lastSeenMs; slot = p; }
      }
    }
    loraPeerCount = min(loraPeerCount + 1, LORA_MAX_PEERS);
  }

  // Update peer record
  strncpy(loraPeers[slot].id,   id,   sizeof(loraPeers[slot].id)   - 1);
  strncpy(loraPeers[slot].name, name, sizeof(loraPeers[slot].name) - 1);
  loraPeers[slot].lat        = lat;
  loraPeers[slot].lon        = lon;
  loraPeers[slot].alt        = (float)altI;
  loraPeers[slot].hdg        = hdgI;
  loraPeers[slot].spd        = spd;
  loraPeers[slot].bat        = bat;
  loraPeers[slot].lastSeenMs = millis();
  loraPeers[slot].active     = true;
  loraPeers[slot].status     = status;
  loraPeers[slot].routePts   = routePts;
  loraPeers[slot].routeDistM = routeDist;
  loraPeers[slot].wpLat      = wpLat;
  loraPeers[slot].wpLon      = wpLon;
  loraPeers[slot].rssi       = loraLastRssi;
  loraPeers[slot].snr        = loraLastSnr;

  Serial.printf("[LORA] Peer %s (%s) lat=%.5f lon=%.5f rssi=%d snr=%d\n",
    id, name, lat, lon, loraLastRssi, loraLastSnr);

  // Notify BLE
  if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
    char json[256];
    snprintf(json, sizeof(json),
      "{\"peer\":\"%s\",\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,"
      "\"alt\":%d,\"hdg\":%.1f,\"spd\":%d,\"bat\":%d,"
      "\"rec\":%d,\"wp\":%d,\"rpts\":%d,\"rdist\":%lu,"
      "\"wlat\":%.6f,\"wlon\":%.6f}",
      id, name, lat, lon, (int)altI, hdgI / 10.0f, spd, bat,
      (status&0x01)?1:0, (status&0x02)?1:0, routePts, (unsigned long)routeDist,
      wpLat, wpLon);
    pPeerChar->setValue((uint8_t*)json, strlen(json));
    pPeerChar->notify();
  }
  
  // Relay position packets
  relayPacket(buf, len, magic);
  
  // Check if we have stored messages for this peer
  storeFwdCheckDeliver(id);
}

// Expire stale peers and recount
void loraExpirePeers() {
  uint32_t now = millis();
  loraPeerCount = 0;
  for (int p = 0; p < LORA_MAX_PEERS; p++) {
    if (loraPeers[p].active) {
      if (now - loraPeers[p].lastSeenMs > LORA_PEER_EXPIRE_MS) {
        loraPeers[p].active = false;
        Serial.printf("[LORA] Peer %s expired\n", loraPeers[p].id);
      } else {
        loraPeerCount++;
      }
    }
  }
}

// Build GETLORA response JSON
String buildLoraJSON() {
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"loraRegion\":%d,\"loraInterval\":%d}",
    loraRegionIdx, loraIntervalSec);
  return String(buf);
}
#endif // ENABLE_LORA

// ============================================================
// BLE CALLBACKS
// ============================================================
void saveWifi() {
  prefs.begin("azimuth", false);
  prefs.putString("wifiSSID", wifiSSID);
  prefs.putString("wifiPass", wifiPass);
  prefs.putString("owmKey",   owmApiKey);
  prefs.end();
}

// Helper — build current settings JSON
String buildSettingsJSON() {
  char buf[160];
#if ENABLE_LORA
  snprintf(buf, sizeof(buf),
    "{\"spd\":%d,\"elv\":%d,\"tmo\":%d,\"brt\":%d,\"loraRegion\":%d,\"loraInterval\":%d,\"logCount\":%d,\"rec\":%d}",
    setSpeedMph ? 1 : 0, setElevFt ? 1 : 0, setTimeoutIdx,
    setBrightFull ? 1 : 0, loraRegionIdx, loraIntervalSec,
    routeLogCount, routeRecording ? 1 : 0);
#else
  snprintf(buf, sizeof(buf),
    "{\"spd\":%d,\"elv\":%d,\"tmo\":%d,\"brt\":%d,\"logCount\":%d,\"rec\":%d}",
    setSpeedMph ? 1 : 0, setElevFt ? 1 : 0, setTimeoutIdx, setBrightFull ? 1 : 0,
    routeLogCount, routeRecording ? 1 : 0);
#endif
  return String(buf);
}

// Forward declarations — callbacks call functions defined later in file
void drawPairingScreen();
void oledDraw();
void oledDrawPage();
void wifiConnectAndFetch();
void loraSOSStart();
void loraSOSCancel();
void blePushRouteLog();
String buildSettingsJSON();


class SetCharCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string val = c->getValue();
      bool isAuthCmd = (val == "GETPIN" || val.find("VERPIN:") == 0);
    if (!isAuthCmd && !sessionAuthorised) {
      c->setValue("{\"err\":\"auth\"}");
      c->notify();
      return;
    }
    // Lockdown mode — block config commands (read + safety commands allowed)
    if (deviceLocked) {
      bool allowed = (val == "GETPIN" || val.find("VERPIN:") == 0 ||
                      val == "GETSET" || val == "GETGROUPS" || val == "GETRELAY" ||
                      val == "SOS" || val == "CANCELSOS" || 
                      val.find("SENDMSG") == 0);
      if (!allowed) {
        c->setValue("{\"err\":\"locked\"}");
        c->notify();
        return;
      }
    }
    if (val == "GETPIN") {
      char pinJson[32];
      snprintf(pinJson, sizeof(pinJson), "{\"pin\":%06lu}", pairingPin);
      c->setValue(pinJson);
      c->notify();
      return;
    } else if (val.find("VERPIN:") == 0) {
      String submitted = String(val.c_str()).substring(7);
      char expected[8];
      snprintf(expected, sizeof(expected), "%06lu", pairingPin);
      bool correct = (submitted == String(expected));
      c->setValue(correct ? "{\"auth\":1}" : "{\"auth\":0}");
      c->notify();
      if (correct) sessionAuthorised = true;
      Serial.printf("[BLE] PIN verify: %s\n", correct ? "OK" : "FAIL");
      return;
    } else if (val == "GETSET") {
      // Return current settings as JSON
      String j = buildSettingsJSON();
      c->setValue(j.c_str());
      c->notify();
      if (pResChar) pResChar->setValue((uint8_t*)j.c_str(), j.length());
      return;
    }
    // Parse SET:{...} payload
    if (val.find("SET:") == 0) {
      String json = String(val.c_str()).substring(4);
      // Simple key:value parser — no heap allocation
      auto getInt = [&](const char* key) -> int {
        String k = String("\"") + key + "\":";
        int idx = json.indexOf(k);
        if (idx < 0) return -1;
        return json.substring(idx + k.length()).toInt();
      };
      int spd = getInt("spd");
      int elv = getInt("elv");
      int tmo = getInt("tmo");
      int brt = getInt("brt");
      if (spd >= 0) setSpeedMph   = (spd == 1);
      if (elv >= 0) setElevFt     = (elv == 1);
      if (tmo >= 0 && tmo < TIMEOUT_PRESET_COUNT) setTimeoutIdx = (uint8_t)tmo;
      if (brt >= 0) setBrightFull = (brt == 1);
      saveSettings();
      // Echo back updated settings
      String j = buildSettingsJSON();
      c->setValue(j.c_str());
      c->notify();
    } else if (val == "GETNAME") {
      char nameJson[64];
      snprintf(nameJson, sizeof(nameJson), "{\"name\":\"%s\"}", bleDeviceName);
      c->setValue(nameJson);
      c->notify();
      if (pResChar) pResChar->setValue((uint8_t*)nameJson, strlen(nameJson));
      return;
    } else if (val == "GETAP") {
      char apJson[128];
      snprintf(apJson, sizeof(apJson),
        "{\"ap\":%d,\"ssid\":\"%s\",\"ip\":\"192.168.4.1\",\"pass\":\"azimuth1\"}",
        apEnabled ? 1 : 0, bleDeviceName);
      c->setValue(apJson);
      c->notify();
      return;
    } else if (val.find("SETNAME:") == 0) {
      String newName = String(val.c_str()).substring(8);
      newName.trim();
      // Ensure Azimuth- prefix
      if (!newName.startsWith("Azimuth-")) newName = "Azimuth-" + newName;
      if (newName.length() > 0 && newName.length() < 30) {
        newName.toCharArray(bleDeviceName, sizeof(bleDeviceName));
        prefs.begin("azimuth", false);
        prefs.putString("devName", bleDeviceName);
        prefs.end();
        Serial.printf("[BLE] Device renamed to: %s — rebooting\n", bleDeviceName);
        if (pSetChar) {
          char conf[48];
          snprintf(conf, sizeof(conf), "{\"name\":\"%s\"}", bleDeviceName);
          pSetChar->setValue(conf);
          pSetChar->notify();
          delay(500);
        }
        ESP.restart();
      }
      return;
    } else if (val.find("SETWIFI:") == 0) {
      String payload = String(val.c_str()).substring(8);
      int sep1 = payload.indexOf('|');
      int sep2 = payload.lastIndexOf('|');
      if (sep1 > 0 && sep2 > sep1) {
        payload.substring(0, sep1).toCharArray(wifiSSID, sizeof(wifiSSID));
        payload.substring(sep1+1, sep2).toCharArray(wifiPass, sizeof(wifiPass));
        payload.substring(sep2+1).toCharArray(owmApiKey, sizeof(owmApiKey));
        wifiEnabled = (strlen(wifiSSID) > 0);
        saveWifi();
        Serial.printf("[WIFI] Credentials saved: %s\n", wifiSSID);
        if (pSetChar) {
          pSetChar->setValue("{\"wifi\":1}");
          pSetChar->notify();
        }
      }
      return;
    } else if (val == "GETSCAN") {
      // Return last scan result
      if (wifiScanResult.length() > 0) {
        c->setValue(wifiScanResult.c_str());
      } else {
        c->setValue("{\"nets\":[]}");
      }
      c->notify();
      return;
    } else if (val.find("SETOWM:") == 0) {
      // Save OWM API key only — WiFi credentials unchanged
      String key = String(val.c_str()).substring(7);
      key.trim();
      if (key.length() > 0) {
        key.toCharArray(owmApiKey, sizeof(owmApiKey));
        prefs.begin("azimuth", false);
        prefs.putString("owmKey", owmApiKey);
        prefs.end();
        Serial.printf("[WIFI] OWM key saved\n");
        c->setValue("{\"owm\":1}");
        c->notify();
      }
      return;
    } else if (val == "WIFISCAN") {
      // Trigger async scan from main loop — BLE stack stays free
      wifiScanResult = "";  // clear previous result
      wifiScanPending = true;
      return;
    } else if (val == "WIFIREFRESH") {
      wifiRefreshPending = true;
      return;
#if ENABLE_LORA
    } else if (val == "GETLORA") {
      String j = buildLoraJSON();
      c->setValue(j.c_str());
      c->notify();
      if (pResChar) pResChar->setValue((uint8_t*)j.c_str(), j.length());
      return;
    } else if (val.find("SETLORA:") == 0) {
      String payload = String(val.c_str()).substring(8);
      int sep = payload.indexOf('|');
      if (sep > 0) {
        int region   = payload.substring(0, sep).toInt();
        int interval = payload.substring(sep + 1).toInt();
        if (region >= 0 && region < LORA_REGION_COUNT) loraRegionIdx = (uint8_t)region;
        interval = constrain(interval, LORA_MIN_INTERVAL_S, LORA_MAX_INTERVAL_S);
        loraIntervalSec = (uint16_t)interval;
        prefs.begin("azimuth", false);
        prefs.putUChar("loraRegion",   loraRegionIdx);
        prefs.putUShort("loraInterval", loraIntervalSec);
        prefs.end();
        if (loraReady) loraBegin();
        Serial.printf("[LORA] Settings updated: region=%d interval=%ds\n",
          loraRegionIdx, loraIntervalSec);
        String j = buildLoraJSON();
        c->setValue(j.c_str());
        c->notify();
      }
      return;
    } else if (val == "GETRELAY") {
      // Return relay status and stats
      char buf[128];
      uint32_t lastTxAgo = (relayLastTx > 0) ? (millis() - relayLastTx) / 1000 : 0;
      snprintf(buf, sizeof(buf),
        "{\"relay\":%d,\"relayCount\":%d,\"stored\":%d,\"lastTx\":%lu}",
        relayEnabled ? 1 : 0, relayCount, storedMsgCount, lastTxAgo);
      c->setValue(buf);
      c->notify();
      return;
    } else if (val.find("SETRELAY:") == 0) {
      // Enable/disable relay: SETRELAY:1 or SETRELAY:0
      String payload = String(val.c_str()).substring(9);
      int newState = payload.toInt();
      relayEnabled = (newState == 1);
      // Save to NVS
      prefs.begin("azimuth", false);
      prefs.putBool("relayMode", relayEnabled);
      prefs.end();
      Serial.printf("[RELAY] %s via BLE\n", relayEnabled ? "Enabled" : "Disabled");
      // Return updated status
      char buf[128];
      uint32_t lastTxAgo = (relayLastTx > 0) ? (millis() - relayLastTx) / 1000 : 0;
      snprintf(buf, sizeof(buf),
        "{\"relay\":%d,\"relayCount\":%d,\"stored\":%d,\"lastTx\":%lu}",
        relayEnabled ? 1 : 0, relayCount, storedMsgCount, lastTxAgo);
      c->setValue(buf);
      c->notify();
      oledRedrawPending = true;
      return;
#endif // ENABLE_LORA
#if ENABLE_LORA
    } else if (val.find("SENDMSG:") == 0) {
      // Broadcast message to all: SENDMSG:hello world
      String text = String(val.c_str()).substring(8);
      text.trim();
      if (text.length() > 0) {
        loraSendMessage(text.c_str(), nullptr);
      }
      return;
    } else if (val.find("SENDMSGDM:") == 0) {
      // Direct message: SENDMSGDM:84F044:hello world
      String payload = String(val.c_str()).substring(10);
      int sep = payload.indexOf(':');
      if (sep == 6) {
        String toId = payload.substring(0, 6);
        String text = payload.substring(7);
        text.trim();
        if (text.length() > 0) {
          loraSendMessage(text.c_str(), toId.c_str());
        }
      }
      return;
    } else if (val.find("SETLORAKEY:") == 0) {
      // Set network key: SETLORAKEY:16charpassword!!
      String keyStr = String(val.c_str()).substring(11);
      if (keyStr.length() == LORA_KEY_LEN) {
        uint8_t newKey[LORA_KEY_LEN];
        memcpy(newKey, keyStr.c_str(), LORA_KEY_LEN);
        loraSaveKey(newKey);
        c->setValue("{\"keySet\":1}");
        c->notify();
        Serial.println("[LORA] Network key updated");
      } else {
        c->setValue("{\"keySet\":0,\"err\":\"key must be 16 chars\"}");
        c->notify();
      }
      return;
    } else if (val == "REGDISCOVER") {
      // Broadcast registration discover — leader asking "who's there?"
      loraRegDiscover();
      c->setValue("{\"regDiscover\":1}");
      c->notify();
      return;
    } else if (val.find("REGASSIGN:") == 0) {
      // Assign name to device: REGASSIGN:84F044:RunnerName
      String payload = String(val.c_str()).substring(10);
      int sep = payload.indexOf(':');
      if (sep == 6 && payload.length() > 7) {
        String toId = payload.substring(0, 6);
        String name = payload.substring(7);
        name.trim();
        if (name.length() > 0 && name.length() <= 16) {
          loraRegAssignName(toId.c_str(), name.c_str());
          char resp[80];
          snprintf(resp, sizeof(resp), "{\"regAssign\":\"%s\",\"name\":\"%s\"}", toId.c_str(), name.c_str());
          c->setValue(resp);
          c->notify();
        } else {
          c->setValue("{\"regAssign\":0,\"err\":\"name 1-16 chars\"}");
          c->notify();
        }
      } else {
        c->setValue("{\"regAssign\":0,\"err\":\"format: REGASSIGN:DEVICEID:Name\"}");
        c->notify();
      }
      return;
    } else if (val.find("SETKEY:") == 0) {
      // Set network key directly: SETKEY:16charkey
      String keyStr = String(val.c_str()).substring(7);
      if (keyStr.length() == LORA_KEY_LEN) {
        uint8_t newKey[LORA_KEY_LEN];
        memcpy(newKey, keyStr.c_str(), LORA_KEY_LEN);
        loraSaveKey(newKey);
        c->setValue("{\"keySet\":1}");
        Serial.printf("[LORA] Key set via BLE: %.16s\n", keyStr.c_str());
      } else {
        c->setValue("{\"keySet\":0,\"err\":\"key must be 16 chars\"}");
      }
      c->notify();
      return;
    } else if (val.rfind("SETGROUPNAME:", 0) == 0) {
      // Set group name for key share broadcast
      std::string name = val.substr(13);
      // Trim whitespace
      size_t start = name.find_first_not_of(" \t\r\n");
      size_t end = name.find_last_not_of(" \t\r\n");
      if (start != std::string::npos) {
        name = name.substr(start, end - start + 1);
      }
      strncpy(keyShareGroupName, name.c_str(), 31);
      keyShareGroupName[31] = 0;
      Serial.printf("[BLE] Group name set: %s\n", keyShareGroupName);
      c->setValue("{\"groupName\":1}");
      c->notify();
      return;
    } else if (val.rfind("ADDGROUP:", 0) == 0) {
      // Add/update stored group: ADDGROUP:name:16charkey
      // Also activates it
      std::string params = val.substr(9);
      size_t colonPos = params.rfind(':');
      if (colonPos != std::string::npos && params.length() - colonPos - 1 == 16) {
        std::string name = params.substr(0, colonPos);
        std::string key = params.substr(colonPos + 1);
        uint8_t keyBytes[LORA_KEY_LEN];
        memcpy(keyBytes, key.c_str(), LORA_KEY_LEN);
        // Add as leader (we created it)
        storedGroupAdd(name.c_str(), keyBytes, "", true);
        // Find and activate
        for (int i = 0; i < storedGroupCount; i++) {
          if (memcmp(storedGroups[i].key, keyBytes, LORA_KEY_LEN) == 0) {
            storedGroupActivate(i);
            storedGroupSel = i;
            break;
          }
        }
        // Also set keyShareGroupName for broadcasts
        strncpy(keyShareGroupName, name.c_str(), 31);
        c->setValue("{\"addGroup\":1}");
        Serial.printf("[BLE] Added group: %s\n", name.c_str());
        oledDraw();
      } else {
        c->setValue("{\"addGroup\":0,\"err\":\"format: ADDGROUP:name:16charkey\"}");
      }
      c->notify();
      return;
    } else if (val.rfind("DELGROUP:", 0) == 0) {
      // Delete stored group by index
      int idx = atoi(val.substr(9).c_str());
      if (idx >= 0 && idx < storedGroupCount) {
        storedGroupDelete(idx);
        c->setValue("{\"delGroup\":1}");
        Serial.printf("[BLE] Deleted group %d\n", idx);
      } else {
        c->setValue("{\"delGroup\":0,\"err\":\"invalid index\"}");
      }
      c->notify();
      return;
    } else if (val == "CLEARGROUPS") {
      // Clear all stored groups
      storedGroupCount = 0;
      activeStoredGroup = -1;
      storedGroupSel = 0;
      memcpy(loraKey, LORA_DEFAULT_KEY, LORA_KEY_LEN);
      loraSaveKey(loraKey);
      storedGroupsSave();
      c->setValue("{\"clearGroups\":1}");
      Serial.println("[BLE] Cleared all stored groups");
      c->notify();
      return;
    } else if (val == "GETGROUPS") {
      // Return stored groups as JSON
      char json[256];
      int len = snprintf(json, sizeof(json), "{\"groups\":[");
      for (int i = 0; i < storedGroupCount && i < MAX_STORED_GROUPS; i++) {
        if (i > 0) len += snprintf(json + len, sizeof(json) - len, ",");
        len += snprintf(json + len, sizeof(json) - len, 
          "{\"name\":\"%s\",\"leader\":\"%s\",\"active\":%s}",
          storedGroups[i].name, storedGroups[i].leaderName,
          (i == activeStoredGroup) ? "true" : "false");
      }
      snprintf(json + len, sizeof(json) - len, "],\"count\":%d}", storedGroupCount);
      c->setValue(json);
      c->notify();
      return;
    } else if (val == "KEYSHARE") {
      // Broadcast network key (UNENCRYPTED) — for device registration at events
      loraKeyShare();
      c->setValue("{\"keyShare\":1}");
      c->notify();
      return;
    } else if (val == "KEYCANCEL") {
      // Cancel pending key share broadcast
      loraKeyShareCancel();
      c->setValue("{\"keyCancel\":1}");
      c->notify();
      return;
    } else if (val == "KEYACCEPT") {
      // Accept pending key share (from web app)
      if (pendingKeyShare.active) {
        keyShareAccept();
        c->setValue("{\"keyAccepted\":1}");
      } else {
        c->setValue("{\"keyAccepted\":0,\"err\":\"no pending request\"}");
      }
      c->notify();
      return;
    } else if (val == "KEYDECLINE") {
      // Decline pending key share (from web app)
      if (pendingKeyShare.active) {
        keyShareDecline();
        c->setValue("{\"keyDeclined\":1}");
      } else {
        c->setValue("{\"keyDeclined\":0,\"err\":\"no pending request\"}");
      }
      c->notify();
      return;
#endif // ENABLE_LORA
    } else if (val == "GETGROUPS") {
      // Return stored groups JSON
      String json = buildGroupsJSON();
      // BLE MTU limit — may need chunking for large group lists
      if (json.length() < 500) {
        c->setValue(json.c_str());
        c->notify();
      } else {
        // TODO: implement chunked transfer for large payloads
        c->setValue("{\"err\":\"groups too large for BLE\"}");
        c->notify();
      }
      return;
    } else if (val.find("SETGROUPS:") == 0) {
      // Sync groups from web app: SETGROUPS:{json}
      String json = String(val.c_str()).substring(10);
      if (groupsParseAndSave(json.c_str())) {
        c->setValue("{\"groupsSet\":1}");
        c->notify();
      } else {
        c->setValue("{\"groupsSet\":0,\"err\":\"parse failed\"}");
        c->notify();
      }
      return;
    } else if (val.find("SETACTIVEGROUP:") == 0) {
      // Set active group by index: SETACTIVEGROUP:0 or SETACTIVEGROUP:-1 (personal)
      int idx = String(val.c_str()).substring(15).toInt();
      groupSetActive(idx);
      char resp[32];
      snprintf(resp, sizeof(resp), "{\"activeGroup\":%d}", activeGroupIdx);
      c->setValue(resp);
      c->notify();
      return;
    } else if (val.find("SETWP:") == 0) {
      // Defer all JSON parsing to main loop to avoid BLE stack overflow
      String json = String(val.c_str()).substring(6);
      strncpy(wpPendingJson, json.c_str(), sizeof(wpPendingJson) - 1);
      wpPendingJson[sizeof(wpPendingJson) - 1] = 0;
      wpBroadcastPending = true;
      // Immediate ACK so web app doesn't time out
      c->setValue("{\"wps\":-1}"); c->notify();
      return;
    } else if (val == "CLEARWP") {
      waypointClear();
      wpClearPending = true;  // broadcast clear to all peers from main loop
      c->setValue("{\"wps\":0}"); c->notify();
      Serial.println("[WP] Waypoints cleared via BLE");
      return;
    } else if (val.find("APMODE:") == 0) {
      // Toggle AP web server: APMODE:1 to start, APMODE:0 to stop
      String v = String(val.c_str()).substring(7);
      if (v == "1") {
        apStart();
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"ap\":1,\"ssid\":\"%s\",\"ip\":\"%s\",\"pass\":\"%s\"}",
          apSSID, WiFi.softAPIP().toString().c_str(), apPass);
        c->setValue(resp); c->notify();
      } else {
        apStop();
        c->setValue("{\"ap\":0}"); c->notify();
      }
      return;
      // Return OWM key — only to authorised session, used by web app for weather push
      char resp[64];
      if (strlen(owmApiKey) > 0) {
        snprintf(resp, sizeof(resp), "{\"owmKey\":\"%s\"}", owmApiKey);
      } else {
        snprintf(resp, sizeof(resp), "{\"owmKey\":\"\"}");
      }
      c->setValue(resp); c->notify();
      return;
      // Return current waypoints as JSON
      String json = "{\"wps\":[";
      for (int i = 0; i < waypointCount; i++) {
        if (i > 0) json += ",";
        char entry[80];
        snprintf(entry, sizeof(entry), "{\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f}",
          waypoints[i].name, waypoints[i].lat, waypoints[i].lon);
        json += entry;
      }
      json += "]}";
      c->setValue(json.c_str()); c->notify();
      return;
    } else if (val.find("SETWX:") == 0) {
      // Weather push from web app: SETWX:<peerId>:<json>  or  SETWX:self:<json>
      // Defer JSON parsing to main loop — same pattern as SETWP
      String rest = String(val.c_str()).substring(6);
      int sep = rest.indexOf(':');
      if (sep < 0) return;
      String peerId = rest.substring(0, sep);
      String json   = rest.substring(sep + 1);
      if (peerId == "self") {
        memset(wxPendingPeerId, 0, sizeof(wxPendingPeerId));
      } else {
        strncpy(wxPendingPeerId, peerId.c_str(), sizeof(wxPendingPeerId) - 1);
        wxPendingPeerId[sizeof(wxPendingPeerId) - 1] = 0;
      }
      strncpy(wxPendingJson, json.c_str(), sizeof(wxPendingJson) - 1);
      wxPendingJson[sizeof(wxPendingJson) - 1] = 0;
      wxPushPending = true;
      return;
    }
  }
};


class SecurityCB : public NimBLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { return 0; }
  void onPassKeyNotify(uint32_t pass_key) override {}
  bool onSecurityRequest() override { return true; }

  void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
    if (desc->sec_state.authenticated) {
      Serial.println("[BLE] Pairing successful — device bonded");
      pairingActive     = false;
      pairingPending    = false;
      blePushLogPending = true;  // now safe to push data
      // Push settings
      if (pSetChar) {
        String j = buildSettingsJSON();
        pSetChar->setValue(j.c_str());
        pSetChar->notify();
      }
      currentPage = prePairingPage;
      oledDraw();
    } else {
      Serial.println("[BLE] Pairing failed");
      pairingActive  = false;
      pairingPending = false;
      // Show brief failure message
      display.clearDisplay();
      display.setCursor(20, 28); display.println("Pairing failed");
      display.display(); delay(1500);
      currentPage = prePairingPage;
      oledDraw();
    }
  }

  bool onConfirmPIN(uint32_t pin) override { return true; }
};

class BLEServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
    bleConnected = true;
    // Check if this is an unbound device — keep PIN visible
    if (!desc->sec_state.bonded) {
      pairingPending = true;
      blePushLogPending = false;  // wait until bonded before pushing data
      Serial.printf("[BLE] Unbound device connected — PIN: %06lu\n", pairingPin);
    } else {
      pairingPending = false;
      blePushLogPending = true;  // bonded — safe to push
      // Push settings
      if (pSetChar) {
        String j = buildSettingsJSON();
        pSetChar->setValue(j.c_str());
        pSetChar->notify();
      }
    }
    // Request longer connection interval — improves stability on Windows 11
    s->updateConnParams(desc->conn_handle, 24, 48, 0, 600);
    Serial.printf("[BLE] Connected handle=%d\n", desc->conn_handle);
  }
  void onDisconnect(NimBLEServer* s) override {
    bleConnected      = false;
    clientSubscribed  = false;
    blePushLogPending = false;
    pairingPending     = false;
    sessionAuthorised  = false;  // require re-auth on next connect
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Disconnected — advertising restarted");
  }
  void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
    // Windows 11 initiates pairing — we accept the connection but
    // decline bonding so no persistent pairing is stored
    if (!desc->sec_state.encrypted) {
    } else {
    }
  }
};
class GpsCharCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* c, ble_gap_conn_desc* d, uint16_t v) override {
    clientSubscribed = (v > 0);
    Serial.println(clientSubscribed ? "[BLE] Subscribed" : "[BLE] Unsubscribed");
  }
};
class RouteCharCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    bool isReadCmd = (val == "GET" || val == "get");
    if (!isReadCmd && !sessionAuthorised) {
      return;
    }
    if (val == "GETLOG" || val == "getlog") {
      blePushRouteLog();
    } else if (val == "GET" || val == "get") {
      Serial.printf("[ROUTE] Export: %d pts\n", routeCount);
      for (int i = 0; i < routeCount; i++) {
        char buf[96];
        snprintf(buf, sizeof(buf),
          "{\"i\":%d,\"t\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"alt\":%.1f}",
          i, routePoints[i].ms,
          routePoints[i].lat, routePoints[i].lon, routePoints[i].spd, routePoints[i].alt);
        pChar->setValue(buf);
        pChar->notify();
        delay(20);
      }
      char done[48];
      snprintf(done, sizeof(done), "{\"done\":true,\"total\":%d}", routeCount);
      pChar->setValue(done);
      pChar->notify();
    } else if (val == "CLR" || val == "clr") {
      clearRoute();
      Serial.println("[ROUTE] Cleared via BLE");
    } else if (val == "STARTREC" || val == "startrec") {
      if (!routeRecording) {
        clearRoute();
        routeStartMs = millis();
        lastRouteMs = 0;
        routeRecording = true;
        Serial.println("[ROUTE] Started via BLE");
        oledRedrawPending = true;
      }
    } else if (val == "STOPREC" || val == "stoprec") {
      if (routeRecording) {
        routeRecording = false;
        routeLogSave();
        Serial.printf("[ROUTE] Stopped via BLE. %d pts\n", routeCount);
        oledRedrawPending = true;
      }
    } else if (val == "SCREENSHOT" || val == "screenshot") {
      if (!pScrnChar) return;
      const uint8_t* buf = display.getBuffer();
      for (int chunk = 0; chunk < 16; chunk++) {
        uint8_t pkt[66];
        pkt[0] = chunk;
        pkt[1] = 16;
        memcpy(&pkt[2], buf + (chunk * 64), 64);
        pScrnChar->setValue(pkt, 66);
        pScrnChar->notify();
        delay(30);
      }
      Serial.println("[BLE] Screenshot sent");
    }
  }
};

// Push all logged routes to web app via BLE route characteristic
void blePushRouteLog() {
  if (!pRouteChar) return;
  if (routeLogCount == 0) {
    // No routes — send empty log_done so web app knows
    char msg[32];
    snprintf(msg, sizeof(msg), "{\"type\":\"log_done\",\"total\":0}");
    pRouteChar->setValue(msg); pRouteChar->notify();
    Serial.println("[BLE] No routes to push");
    return;
  }
  for (int r = 0; r < routeLogCount; r++) {
    // Send route start header
    char hdr[64];
    snprintf(hdr, sizeof(hdr),
      "{\"type\":\"route_start\",\"route\":%d,\"total\":%d}",
      r, routeLog[r].count);
    pRouteChar->setValue(hdr); pRouteChar->notify(); delay(20);
    // Send all points
    for (int i = 0; i < routeLog[r].count; i++) {
      char buf[96];
      snprintf(buf, sizeof(buf),
        "{\"i\":%d,\"t\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"alt\":%.1f}",
        i, routeLog[r].points[i].ms,
        routeLog[r].points[i].lat, routeLog[r].points[i].lon,
        routeLog[r].points[i].spd, routeLog[r].points[i].alt);
      pRouteChar->setValue(buf); pRouteChar->notify(); delay(20);
    }
    // Send route end
    char ftr[48];
    snprintf(ftr, sizeof(ftr),
      "{\"type\":\"route_end\",\"route\":%d}", r);
    pRouteChar->setValue(ftr); pRouteChar->notify(); delay(30);
  }
  // All done
  char done[32];
  snprintf(done, sizeof(done),
    "{\"type\":\"log_done\",\"total\":%d}", routeLogCount);
  pRouteChar->setValue(done); pRouteChar->notify();
  Serial.println("[BLE] Route log push complete");
}

// ============================================================
// ============================================================
// WIFI / CONNECTED MODE
// ============================================================

// Moon phase calculation — no API needed
void calcMoonPhase() {
  // Days since known new moon (Jan 6 2000)
  uint32_t now = millis() / 1000;  // approximate — use GPS time if available
  // Use a fixed reference: Jan 6 2000 = JD 2451549.5, new moon
  // Current date approximation from GPS UTC if available
  int year = 2026, month = 3, day = 22;  // will be overridden by GPS
  if (cachedFix) {
    // approximate from GPS — we store UTC in cached values
  }
  // Julian Day Number
  int a = (14 - month) / 12;
  int y = year + 4800 - a;
  int m = month + 12 * a - 3;
  long jdn = day + (153*m+2)/5 + 365*y + y/4 - y/100 + y/400 - 32045;
  float daysSinceNew = fmod((float)(jdn - 2451549L), 29.53059f);
  if (daysSinceNew < 0) daysSinceNew += 29.53059f;
  moonData.phase = daysSinceNew / 29.53059f;
  moonData.illumination = (1.0f - cos(moonData.phase * 2.0f * M_PI)) / 2.0f * 100.0f;
  // Phase name
  if      (daysSinceNew < 1.85)  strcpy(moonData.phaseName, "New Moon");
  else if (daysSinceNew < 7.38)  strcpy(moonData.phaseName, "Waxing Cres");
  else if (daysSinceNew < 9.22)  strcpy(moonData.phaseName, "First Qtr");
  else if (daysSinceNew < 14.77) strcpy(moonData.phaseName, "Waxing Gibb");
  else if (daysSinceNew < 16.61) strcpy(moonData.phaseName, "Full Moon");
  else if (daysSinceNew < 22.15) strcpy(moonData.phaseName, "Waning Gibb");
  else if (daysSinceNew < 23.99) strcpy(moonData.phaseName, "Last Qtr");
  else if (daysSinceNew < 29.53) strcpy(moonData.phaseName, "Waning Cres");
  moonData.valid = true;
  Serial.printf("[MOON] %s %.0f%%\n", moonData.phaseName, moonData.illumination);
}

// Convert unix timestamp to HH:MM string
void unixToHHMM(long ts, int tzOff, char* out) {
  long t = ts + (long)tzOff * 3600;
  int h = (t % 86400L) / 3600;
  int m = (t % 3600) / 60;
  snprintf(out, 6, "%02d:%02d", h, m);
}

// Fetch weather from OpenWeatherMap
bool fetchWeather() {
  if (strlen(owmApiKey) == 0) return false;
  if (!cachedFix && !cachedFixIsApprox) return false;  // need at least an approximate position
  char url[200];
  snprintf(url, sizeof(url),
    "https://api.openweathermap.org/data/2.5/weather?lat=%.6f&lon=%.6f&appid=%s&units=metric",
    cachedLat, cachedLon, owmApiKey);
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.setTimeout(8000);
  int code = http.GET();
  Serial.printf("[WIFI] Weather HTTP %d\n", code);
  if (code != 200) { http.end(); return false; }
  String body = http.getString();
  http.end();
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) { Serial.printf("[WIFI] Weather JSON error: %s\n", err.c_str()); return false; }
  weather.tempC    = doc["main"]["temp"];
  weather.humidity = doc["main"]["humidity"];
  weather.tempMinC = doc["main"]["temp_min"];
  weather.tempMaxC = doc["main"]["temp_max"];
  weather.weatherId = doc["weather"][0]["id"];
  strlcpy(weather.desc, doc["weather"][0]["description"] | "", sizeof(weather.desc));
  float windMs = doc["wind"]["speed"];
  weather.windKmh      = windMs * 3.6f;
  float gustMs = doc["wind"]["gust"] | 0.0f;
  weather.windGustKmh  = gustMs * 3.6f;
  weather.feelsLikeC   = doc["main"]["feels_like"];
  strlcpy(weather.locationName, doc["name"] | "Unknown", sizeof(weather.locationName));
  // Wind direction
  int deg = doc["wind"]["deg"];
  const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW","N"};
  strlcpy(weather.windDir, dirs[(int)((deg + 22.5f) / 45.0f) % 8], sizeof(weather.windDir));
  weather.valid = true; weatherReceived = true;
  Serial.printf("[WIFI] Weather: %.1fC %s\n", weather.tempC, weather.desc);
  return true;
}

// Fetch sunrise/sunset from sunrise-sunset.org (free, no key)
bool fetchSunData() {
  if (!cachedFix) return false;
  char url[180];
  snprintf(url, sizeof(url),
    "https://api.sunrise-sunset.org/json?lat=%.6f&lng=%.6f&formatted=0",
    cachedLat, cachedLon);
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.setTimeout(8000);
  int code = http.GET();
  if (code != 200) { http.end(); Serial.printf("[WIFI] Sun HTTP %d\n", code); return false; }
  String body = http.getString();
  http.end();
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, body)) return false;
  if (String(doc["status"].as<const char*>()) != "OK") return false;
  // Parse ISO8601 timestamps — extract unix time
  String srStr = doc["results"]["sunrise"];
  String ssStr = doc["results"]["sunset"];
  // sunrise-sunset returns UTC ISO8601 — parse hour/min directly
  // Format: "2026-03-22T06:23:45+00:00"
  int srH = srStr.substring(11,13).toInt();
  int srM = srStr.substring(14,16).toInt();
  int ssH = ssStr.substring(11,13).toInt();
  int ssM = ssStr.substring(14,16).toInt();
  // Apply timezone offset
  srH = (srH + tzOffset + 24) % 24;
  ssH = (ssH + tzOffset + 24) % 24;
  snprintf(sunData.sunrise, 6, "%02d:%02d", srH, srM);
  snprintf(sunData.sunset,  6, "%02d:%02d", ssH, ssM);
  // Golden hour = ~1hr after sunrise, ~1hr before sunset
  int ghH = (srH + 1) % 24;
  int geH = (ssH - 1 + 24) % 24;
  snprintf(sunData.golden,    6, "%02d:%02d", ghH, srM);
  snprintf(sunData.goldenEve, 6, "%02d:%02d", geH, ssM);
  // Calculate day length from sunrise/sunset minutes
  int riseH = String(sunData.sunrise).substring(0,2).toInt();
  int riseM = String(sunData.sunrise).substring(3,5).toInt();
  int setH  = String(sunData.sunset).substring(0,2).toInt();
  int setM  = String(sunData.sunset).substring(3,5).toInt();
  int dayMins = (setH*60+setM) - (riseH*60+riseM);
  if (dayMins > 0) snprintf(sunData.dayLength, sizeof(sunData.dayLength), "%dh %02dm", dayMins/60, dayMins%60);
  sunData.valid = true;
  Serial.printf("[WIFI] Sun: rise=%s set=%s\n", sunData.sunrise, sunData.sunset);
  return true;
}

// Fetch magnetic declination from NOAA
bool fetchMagDeclination() {
  if (!cachedFix) return false;
  char url[200];
  snprintf(url, sizeof(url),
    "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination?lat1=%.4f&lon1=%.4f&key=zNEw7&resultFormat=json",
    cachedLat, cachedLon);
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.setTimeout(8000);
  int code = http.GET();
  if (code != 200) { http.end(); Serial.printf("[WIFI] Mag HTTP %d\n", code); return false; }
  String body = http.getString();
  http.end();
  // Response: {"result":[{"declination":X.XX,...}]}
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, body)) return false;
  magDeclination = doc["result"][0]["declination"];
  magDecValid = true;
  Serial.printf("[WIFI] Mag declination: %.2f deg\n", magDeclination);
  return true;
}

// Main WiFi connect and fetch
void wifiConnectAndFetch() {
  if (!wifiEnabled || strlen(wifiSSID) == 0) return;

  // Pre-scan for BeaconDB before connecting (passive scan, no association needed)
  Serial.println("[WIFI] Pre-scanning networks for BeaconDB...");
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(false, true);
  String beaconPayload = "";
  if (n > 0) {
    beaconPayload = "{\"wifiAccessPoints\":[";
    int sent = 0;
    for (int i = 0; i < n && sent < 10; i++) {
      String bssid = WiFi.BSSIDstr(i);
      int rssi = WiFi.RSSI(i);
      if (sent > 0) beaconPayload += ",";
      beaconPayload += "{\"macAddress\":\"" + bssid + "\",\"signalStrength\":" + rssi + "}";
      sent++;
    }
    beaconPayload += "]}";
    Serial.printf("[WIFI] Captured %d networks for BeaconDB\n", n);
  }
  WiFi.scanDelete();

  Serial.printf("[WIFI] Connecting to %s...\n", wifiSSID);
  WiFi.begin(wifiSSID, wifiPass);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_CONNECT_TIMEOUT_MS) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Connect failed");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }
  wifiConnected = true;
  Serial.printf("[WIFI] Connected, IP: %s\n", WiFi.localIP().toString().c_str());

  // If no GPS fix, try IP geolocation then refine with BeaconDB
  if (!cachedFix) {
    // Step 1: IP geolocation (fast, city-level)
    Serial.println("[WIFI] No GPS fix — trying IP geolocation...");
    HTTPClient http;
    http.begin("http://ip-api.com/json/?fields=lat,lon,status");
    http.setTimeout(5000);
    int code = http.GET();
    if (code == 200) {
      String body = http.getString();
      int latIdx = body.indexOf("\"lat\":"); 
      int lonIdx = body.indexOf("\"lon\":"); 
      if (latIdx > 0 && lonIdx > 0 && body.indexOf("\"success\"") > 0) {
        cachedLat = body.substring(latIdx + 6).toFloat();
        cachedLon = body.substring(lonIdx + 6).toFloat();
        cachedFixIsApprox = true;
        cachedApproxAccM  = 5000.0f;
        Serial.printf("[WIFI] IP geolocation: %.4f, %.4f\n", cachedLat, cachedLon);
        if (bleConnected && clientSubscribed && sessionAuthorised && pGpsChar) {
          String json = buildJSON();
          pGpsChar->setValue((uint8_t*)json.c_str(), json.length());
          pGpsChar->notify();
        }
      }
    }
    http.end();

    // Step 2: BeaconDB WiFi geolocation (better accuracy, ~50-200m)
    if (beaconPayload.length() > 0) {
      Serial.println("[WIFI] Querying BeaconDB...");
      WiFiClientSecure client;
      client.setInsecure();
      HTTPClient https;
      https.begin(client, "https://api.beacondb.net/v1/geolocate");
      https.addHeader("Content-Type", "application/json");
      https.addHeader("User-Agent", "Azimuth-GPS-Tracker/1.3");
      https.setTimeout(8000);
      int hcode = https.POST(beaconPayload);
      if (hcode == 200) {
        String resp = https.getString();
        int latI = resp.indexOf("\"lat\":");
        int lngI = resp.indexOf("\"lng\":");
        int accI = resp.indexOf("\"accuracy\":");
        if (latI > 0 && lngI > 0) {
          float bLat = resp.substring(latI + 6).toFloat();
          float bLon = resp.substring(lngI + 6).toFloat();
          float bAcc = accI > 0 ? resp.substring(accI + 11).toFloat() : 999999.0f;
          if (bAcc < cachedApproxAccM) {
            cachedLat = bLat;
            cachedLon = bLon;
            cachedApproxAccM  = bAcc;
            cachedFixIsApprox = true;
            Serial.printf("[WIFI] BeaconDB: %.5f, %.5f acc=%.0fm (improved)\n", bLat, bLon, bAcc);
            if (bleConnected && clientSubscribed && sessionAuthorised && pGpsChar) {
              String json = buildJSON();
              pGpsChar->setValue((uint8_t*)json.c_str(), json.length());
              pGpsChar->notify();
            }
          } else {
            Serial.printf("[WIFI] BeaconDB: %.5f, %.5f acc=%.0fm (worse than IP, ignored)\n", bLat, bLon, bAcc);
          }
        }
      } else {
        Serial.printf("[WIFI] BeaconDB failed: %d\n", hcode);
      }
      https.end();
    }
  }

  // Fetch all data
  calcMoonPhase();
  bool wOk = fetchWeather();
  bool sOk = fetchSunData();
  bool dOk = fetchMagDeclination();
  wifiLastFetchMs = millis();
  // Save wall clock time of fetch for display
  if (gps.time.isValid()) {
    int fh, fm, fs, fd, fmo, fy;
    getLocalTime(fh, fm, fs, fd, fmo, fy);
    wifiLastFetchHour = fh;
    wifiLastFetchMin  = fm;
  }

  // Disconnect to free radio for BLE
  WiFi.disconnect(true);
  // If AP mode is active, restore AP+STA or AP-only mode
  if (apEnabled) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPass);
  } else {
    WiFi.mode(WIFI_OFF);
  }
  wifiConnected = false;
  Serial.println("[WIFI] Fetch complete, WiFi off");
}

// ── AP / Web server ───────────────────────────────────────────────────────────

void webServerHandleRoot() {
  webServer.sendHeader("Content-Encoding", "gzip");
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send_P(200, "text/html", (const char*)WEBAPP_GZ, WEBAPP_GZ_LEN);
}

void webServerHandleNotFound() {
  Serial.printf("[AP] 404: %s\n", webServer.uri().c_str());
  webServer.sendHeader("Location", "/", true);
  webServer.send(302, "text/plain", "");
}

// ── WiFi API handlers ─────────────────────────────────────────────────────────

void apiHandleState() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Build peers JSON using String to avoid buffer overflow with many peers
  String peersJson = "[";
  bool firstPeer = true;
#if ENABLE_LORA
  for (int p = 0; p < LORA_MAX_PEERS; p++) {
    if (!loraPeers[p].active) continue;
    if (!firstPeer) peersJson += ",";
    char peerBuf[180];
    snprintf(peerBuf, sizeof(peerBuf),
      "{\"id\":\"%s\",\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,"
      "\"hdg\":%d,\"spd\":%d,\"bat\":%d,\"ts\":%lu,\"status\":%d}",
      loraPeers[p].id, loraPeers[p].name,
      loraPeers[p].lat, loraPeers[p].lon, loraPeers[p].alt,
      loraPeers[p].hdg, loraPeers[p].spd, loraPeers[p].bat,
      loraPeers[p].lastSeenMs, loraPeers[p].status);
    peersJson += peerBuf;
    firstPeer = false;
  }
#endif
  peersJson += "]";

  // Waypoints JSON
  char wpJson[512] = "[";
  for (int i = 0; i < waypointCount; i++) {
    if (i > 0) strncat(wpJson, ",", sizeof(wpJson)-strlen(wpJson)-1);
    char wpBuf[80];
    snprintf(wpBuf, sizeof(wpBuf), "{\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f}",
      waypoints[i].name, waypoints[i].lat, waypoints[i].lon);
    strncat(wpJson, wpBuf, sizeof(wpJson)-strlen(wpJson)-1);
  }
  strncat(wpJson, "]", sizeof(wpJson)-strlen(wpJson)-1);

  // Weather JSON
  char wxJson[128] = "null";
  if (weather.valid) {
    snprintf(wxJson, sizeof(wxJson),
      "{\"t\":%.1f,\"f\":%.1f,\"mn\":%.1f,\"mx\":%.1f,\"h\":%d,"
      "\"w\":%.1f,\"wd\":\"%s\",\"id\":%d,\"d\":\"%s\"}",
      weather.tempC, weather.feelsLikeC, weather.tempMinC, weather.tempMaxC,
      weather.humidity, weather.windKmh, weather.windDir,
      weather.weatherId, weather.desc);
  }

  // Build full response using String for safety
  String resp = "{";
  char buf[256];
  snprintf(buf, sizeof(buf),
    "\"fix\":%d,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,"
    "\"spd\":%.1f,\"hdg\":%.1f,\"sats\":%d,"
    "\"bat\":%d,\"batV\":%.2f,\"usb\":%d,"
    "\"rec\":%d,\"sos\":%d,"
    "\"name\":\"%s\",\"id\":\"%s\",\"ts\":%lu,",
    cachedFix ? 1 : 0, cachedLat, cachedLon, cachedAlt,
    (float)cachedSpd, (float)cachedHdg, cachedSats,
    batPercent, batVoltage, batOnUSB ? 1 : 0,
    routeRecording ? 1 : 0, sosActive ? 1 : 0,
    bleDeviceName, loraDeviceId, millis());
  resp += buf;

  // Settings
  snprintf(buf, sizeof(buf),
    "\"settings\":{\"spd\":%d,\"elv\":%d,\"tmo\":%d,\"brt\":%d,"
    "\"loraRegion\":%d,\"loraInterval\":%d},",
    setSpeedMph ? 1 : 0, setElevFt ? 1 : 0, setTimeoutIdx, setBrightFull ? 1 : 0,
    loraRegionIdx, loraIntervalSec);
  resp += buf;

  resp += "\"peers\":";   resp += peersJson; resp += ",";
  resp += "\"wps\":";     resp += wpJson;    resp += ",";
  resp += "\"wx\":";      resp += wxJson;
  resp += "}";

  webServer.send(200, "application/json", resp);
}

void apiHandleCmd() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  if (!webServer.hasArg("plain")) {
    webServer.send(400, "application/json", "{\"err\":\"no body\"}");
    return;
  }
  String body = webServer.arg("plain");
  // Parse command — reuse same format as BLE commands
  Serial.printf("[API] CMD: %s\n", body.c_str());

  // Lockdown mode — block all config commands (SOS/messages still allowed)
  if (deviceLocked) {
    // Allow safety-critical commands
    if (body != "SOS" && body != "CANCELSOS" && !body.startsWith("SENDMSG")) {
      webServer.send(403, "application/json", "{\"err\":\"locked\"}");
      return;
    }
  }

  // Handle commands inline (no BLE stack concerns on main thread)
  if (body == "SOS") {
    loraSOSStart();
  } else if (body == "CANCELSOS") {
    loraSOSCancel();
  } else if (body == "CLEARWP") {
    waypointClear();
    wpClearPending = true;
  } else if (body.startsWith("SENDMSG:")) {
    String text = body.substring(8);
    text.trim();
    if (text.length() > 0) loraSendMessage(text.c_str(), nullptr);
  } else if (body.startsWith("SENDMSGDM:")) {
    int sep = body.indexOf(':', 10);
    if (sep > 10) {
      String toId = body.substring(10, sep);
      String text = body.substring(sep + 1);
      text.trim();
      if (text.length() > 0) loraSendMessage(text.c_str(), toId.c_str());
    }
  } else if (body.startsWith("SETWP:")) {
    String jsonStr = body.substring(6);
    strncpy(wpPendingJson, jsonStr.c_str(), sizeof(wpPendingJson) - 1);
    wpBroadcastPending = true;
  } else if (body.startsWith("SETNAME:")) {
    String name = body.substring(8);
    name.trim();
    if (name.length() > 0 && name.length() <= 20) {
      snprintf(bleDeviceName, sizeof(bleDeviceName), "Azimuth-%s", name.c_str());
      prefs.begin("azimuth", false);
      prefs.putString("devName", bleDeviceName);
      prefs.end();
    }
  } else if (body.startsWith("SETLORA:")) {
    // SETLORA:region:interval
    String rest = body.substring(8);
    int sep = rest.indexOf(':');
    if (sep >= 0) {
      int region = rest.substring(0, sep).toInt();
      int interval = rest.substring(sep + 1).toInt();
      if (region >= 0 && region < LORA_REGION_COUNT) loraRegionIdx = region;
      if (interval >= LORA_MIN_INTERVAL_S && interval <= LORA_MAX_INTERVAL_S) loraIntervalSec = interval;
      saveSettings();
    }
  } else if (body.startsWith("SETSET:")) {
    // SETSET:spd:elv:tmo:brt
    String rest = body.substring(7);
    int s[4]; int idx = 0;
    while (rest.length() > 0 && idx < 4) {
      int sep = rest.indexOf(':');
      s[idx++] = (sep >= 0 ? rest.substring(0, sep) : rest).toInt();
      if (sep < 0) break;
      rest = rest.substring(sep + 1);
    }
    if (idx == 4) {
      setSpeedMph   = s[0] == 1;
      setElevFt     = s[1] == 1;
      setTimeoutIdx = constrain(s[2], 0, 4);
      setBrightFull = s[3] == 1;
      display.invertDisplay(!setBrightFull);
      saveSettings();
    }
  } else if (body.startsWith("APMODE:")) {
    String v = body.substring(7);
    if (v == "0") {
      apStop();
      webServer.send(200, "application/json", "{\"ap\":0}");
      return;
    } else {
      apStart();
      char resp[128];
      snprintf(resp, sizeof(resp), "{\"ap\":1,\"ssid\":\"%s\",\"ip\":\"%s\",\"pass\":\"%s\"}",
        apSSID, WiFi.softAPIP().toString().c_str(), apPass);
      webServer.send(200, "application/json", resp);
      return;
    }
  } else if (body == "REGDISCOVER") {
    // Broadcast registration discover — leader asking "who's there?"
    loraRegDiscover();
    webServer.send(200, "application/json", "{\"regDiscover\":1}");
    return;
  } else if (body.startsWith("REGASSIGN:")) {
    // Assign name to device: REGASSIGN:84F044:RunnerName
    String payload = body.substring(10);
    int sep = payload.indexOf(':');
    if (sep == 6 && payload.length() > 7) {
      String toId = payload.substring(0, 6);
      String name = payload.substring(7);
      name.trim();
      if (name.length() > 0 && name.length() <= 16) {
        loraRegAssignName(toId.c_str(), name.c_str());
        char resp[80];
        snprintf(resp, sizeof(resp), "{\"regAssign\":\"%s\",\"name\":\"%s\"}", toId.c_str(), name.c_str());
        webServer.send(200, "application/json", resp);
        return;
      }
    }
    webServer.send(400, "application/json", "{\"err\":\"format: REGASSIGN:DEVICEID:Name\"}");
    return;
  } else if (body.startsWith("SETKEY:")) {
    // Set network key directly: SETKEY:16charkey
    String keyStr = body.substring(7);
    if (keyStr.length() == LORA_KEY_LEN) {
      uint8_t newKey[LORA_KEY_LEN];
      memcpy(newKey, keyStr.c_str(), LORA_KEY_LEN);
      loraSaveKey(newKey);
      webServer.send(200, "application/json", "{\"keySet\":1}");
      Serial.printf("[LORA] Key set via WiFi: %.16s\n", keyStr.c_str());
    } else {
      webServer.send(400, "application/json", "{\"keySet\":0,\"err\":\"key must be 16 chars\"}");
    }
    return;
  } else if (body == "KEYSHARE") {
    // Broadcast network key (UNENCRYPTED) — for device registration at events
    loraKeyShare();
    webServer.send(200, "application/json", "{\"keyShare\":1}");
    return;
  } else if (body.startsWith("SETGROUPNAME:")) {
    // Set group name for key share broadcast
    String name = body.substring(13);
    name.trim();
    strncpy(keyShareGroupName, name.c_str(), 31);
    keyShareGroupName[31] = 0;
    Serial.printf("[WIFI] Group name set: %s\n", keyShareGroupName);
    webServer.send(200, "application/json", "{\"groupName\":1}");
    return;
  } else if (body == "KEYCANCEL") {
    // Cancel pending key share broadcast
    loraKeyShareCancel();
    webServer.send(200, "application/json", "{\"keyCancel\":1}");
    return;
  } else if (body == "KEYACCEPT") {
    // Accept pending key share (from web app)
    if (pendingKeyShare.active) {
      keyShareAccept();
      webServer.send(200, "application/json", "{\"keyAccepted\":1}");
    } else {
      webServer.send(400, "application/json", "{\"keyAccepted\":0,\"err\":\"no pending request\"}");
    }
    return;
  } else if (body == "KEYDECLINE") {
    // Decline pending key share (from web app)
    if (pendingKeyShare.active) {
      keyShareDecline();
      webServer.send(200, "application/json", "{\"keyDeclined\":1}");
    } else {
      webServer.send(400, "application/json", "{\"keyDeclined\":0,\"err\":\"no pending request\"}");
    }
    return;
  } else if (body == "GETGROUPS") {
    // Return stored groups JSON
    String json = buildGroupsJSON();
    webServer.send(200, "application/json", json);
    return;
  } else if (body.startsWith("SETGROUPS:")) {
    // Sync groups from web app: SETGROUPS:{json}
    String json = body.substring(10);
    if (groupsParseAndSave(json.c_str())) {
      webServer.send(200, "application/json", "{\"groupsSet\":1}");
    } else {
      webServer.send(400, "application/json", "{\"groupsSet\":0,\"err\":\"parse failed\"}");
    }
    return;
  } else if (body.startsWith("SETACTIVEGROUP:")) {
    // Set active group by index: SETACTIVEGROUP:0 or SETACTIVEGROUP:-1 (personal)
    int idx = body.substring(15).toInt();
    groupSetActive(idx);
    char resp[32];
    snprintf(resp, sizeof(resp), "{\"activeGroup\":%d}", activeGroupIdx);
    webServer.send(200, "application/json", resp);
    return;
  }

  webServer.send(200, "application/json", "{\"ok\":1}");
}

// ── Mesh Status API ─────────────────────────────────────────────────────────
// Returns JSON with device info and peer list for the status page
void apiHandleMesh() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  
  // Build JSON response
  String json = "{";
  
  // Device info
  json += "\"device\":{";
  json += "\"name\":\"" + String(bleDeviceName) + "\",";
  json += "\"firmware\":\"v3.1.6\",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"battery\":" + String(batPercent) + ",";
  json += "\"charging\":" + String(batOnUSB ? 1 : 0) + ",";
  json += "\"gps\":" + String(gps.location.isValid() ? 1 : 0) + ",";
  json += "\"sats\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  json += "\"lat\":" + String(gps.location.isValid() ? gps.location.lat() : 0.0, 6) + ",";
  json += "\"lon\":" + String(gps.location.isValid() ? gps.location.lng() : 0.0, 6) + ",";
  json += "\"alt\":" + String(gps.altitude.isValid() ? (int)gps.altitude.meters() : 0) + ",";
  json += "\"hdop\":" + String(gps.hdop.isValid() ? gps.hdop.hdop() : 99.9, 1);
#if ENABLE_LORA
  json += ",\"lora\":{";
  json += "\"ready\":" + String(loraReady ? 1 : 0) + ",";
  json += "\"region\":\"" + String(LORA_REGIONS[loraRegionIdx].name) + "\",";
  json += "\"interval\":" + String(loraIntervalSec) + ",";
  json += "\"relay\":" + String(relayEnabled ? 1 : 0) + ",";
  json += "\"relayCount\":" + String(relayCount) + ",";
  json += "\"stored\":" + String(storedMsgCount);
  json += "}";
#endif
  json += "},";
  
  // Peers array
  json += "\"peers\":[";
#if ENABLE_LORA
  bool first = true;
  uint32_t now = millis();
  for (int p = 0; p < LORA_MAX_PEERS; p++) {
    if (!loraPeers[p].active) continue;
    if (!first) json += ",";
    first = false;
    
    uint32_t seenAgo = (now - loraPeers[p].lastSeenMs) / 1000;
    
    // Calculate distance if we have GPS
    float distKm = 0;
    float bearing = 0;
    if (gps.location.isValid() && loraPeers[p].lat != 0) {
      double dLat = radians(loraPeers[p].lat - gps.location.lat());
      double dLon = radians(loraPeers[p].lon - gps.location.lng());
      double a = sin(dLat/2)*sin(dLat/2) + 
                 cos(radians(gps.location.lat()))*cos(radians(loraPeers[p].lat))*
                 sin(dLon/2)*sin(dLon/2);
      double c = 2 * atan2(sqrt(a), sqrt(1-a));
      distKm = 6371.0 * c;
      
      double y = sin(dLon) * cos(radians(loraPeers[p].lat));
      double x = cos(radians(gps.location.lat()))*sin(radians(loraPeers[p].lat)) -
                 sin(radians(gps.location.lat()))*cos(radians(loraPeers[p].lat))*cos(dLon);
      bearing = fmod((degrees(atan2(y, x)) + 360), 360);
    }
    
    json += "{";
    json += "\"id\":\"" + String(loraPeers[p].id) + "\",";
    json += "\"name\":\"" + String(loraPeers[p].name) + "\",";
    json += "\"lat\":" + String(loraPeers[p].lat, 6) + ",";
    json += "\"lon\":" + String(loraPeers[p].lon, 6) + ",";
    json += "\"alt\":" + String((int)loraPeers[p].alt) + ",";
    json += "\"hdg\":" + String(loraPeers[p].hdg) + ",";
    json += "\"spd\":" + String(loraPeers[p].spd) + ",";
    json += "\"bat\":" + String(loraPeers[p].bat) + ",";
    json += "\"seen\":" + String(seenAgo) + ",";
    json += "\"dist\":" + String(distKm, 2) + ",";
    json += "\"bearing\":" + String((int)bearing) + ",";
    json += "\"rssi\":" + String(loraPeers[p].rssi) + ",";
    json += "\"snr\":" + String(loraPeers[p].snr);
    json += "}";
  }
#endif
  json += "],";
  
  // Message count
  int msgCount = 0;
#if ENABLE_LORA
  for (int i = 0; i < LORA_MAX_MSGS; i++) {
    if (loraMsgs[i].active) msgCount++;
  }
#endif
  json += "\"msgCount\":" + String(msgCount);
  
  json += "}";
  webServer.send(200, "application/json", json);
}

// ── Mesh Status HTML Page ───────────────────────────────────────────────────
// Self-contained status page served at /status - no external dependencies
void handleStatusPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Azimuth Mesh Status</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0d1117;color:#c9d1d9;padding:16px;min-height:100vh}
h1{font-size:20px;color:#58a6ff;margin-bottom:16px;display:flex;align-items:center;gap:8px}
h1::before{content:'';display:inline-block;width:8px;height:8px;background:#238636;border-radius:50%}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:16px;margin-bottom:16px}
.card h2{font-size:14px;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;margin-bottom:12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:12px}
.stat{text-align:center}
.stat .val{font-size:24px;font-weight:600;color:#f0f6fc}
.stat .lbl{font-size:11px;color:#8b949e;text-transform:uppercase;margin-top:2px}
.ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:8px 6px;text-align:left;border-bottom:1px solid #21262d}
th{color:#8b949e;font-weight:500;font-size:11px;text-transform:uppercase}
tr:hover{background:#1c2128}
.sig{display:inline-block;width:40px;height:8px;background:#21262d;border-radius:4px;overflow:hidden;vertical-align:middle}
.sig-bar{height:100%;border-radius:4px}
.empty{color:#484f58;text-align:center;padding:32px}
.update{font-size:11px;color:#484f58;text-align:center;margin-top:16px}
@media(max-width:480px){.grid{grid-template-columns:1fr 1fr}table{font-size:12px}th,td{padding:6px 4px}}
</style>
</head>
<body>
<h1>Azimuth Mesh Status</h1>

<div class="card">
<h2>Device</h2>
<div class="grid">
<div class="stat"><div class="val" id="d-name">-</div><div class="lbl">Name</div></div>
<div class="stat"><div class="val" id="d-bat">-</div><div class="lbl">Battery</div></div>
<div class="stat"><div class="val" id="d-gps">-</div><div class="lbl">GPS</div></div>
<div class="stat"><div class="val" id="d-sats">-</div><div class="lbl">Satellites</div></div>
<div class="stat"><div class="val" id="d-uptime">-</div><div class="lbl">Uptime</div></div>
<div class="stat"><div class="val" id="d-lora">-</div><div class="lbl">LoRa</div></div>
</div>
</div>

<div class="card">
<h2>Peers</h2>
<table>
<thead><tr><th>Name</th><th>Dist</th><th>Seen</th><th>Signal</th><th>Bat</th></tr></thead>
<tbody id="peers"><tr><td colspan="5" class="empty">Loading...</td></tr></tbody>
</table>
</div>

<div class="update">Updates every 5s &bull; <span id="ts">-</span></div>

<script>
function fmt(s){return s<60?s+'s':s<3600?Math.floor(s/60)+'m':Math.floor(s/3600)+'h'}
function sig(rssi){
  let p=Math.max(0,Math.min(100,(rssi+120)/70*100));
  let c=p>60?'#3fb950':p>30?'#d29922':'#f85149';
  return '<div class="sig"><div class="sig-bar" style="width:'+p+'%;background:'+c+'"></div></div> '+rssi;
}
function update(){
  fetch('/api/mesh').then(r=>r.json()).then(d=>{
    document.getElementById('d-name').textContent=d.device.name;
    document.getElementById('d-bat').innerHTML=d.device.battery+'%'+(d.device.charging?' ⚡':'');
    document.getElementById('d-gps').innerHTML=d.device.gps?'<span class="ok">Fix</span>':'<span class="warn">No fix</span>';
    document.getElementById('d-sats').textContent=d.device.sats;
    document.getElementById('d-uptime').textContent=fmt(d.device.uptime);
    if(d.device.lora){
      let l=d.device.lora;
      document.getElementById('d-lora').innerHTML=l.ready?'<span class="ok">'+l.region+'</span>':'<span class="err">Off</span>';
    }else{
      document.getElementById('d-lora').innerHTML='<span class="warn">N/A</span>';
    }
    let tb=document.getElementById('peers');
    if(d.peers.length===0){
      tb.innerHTML='<tr><td colspan="5" class="empty">No peers detected</td></tr>';
    }else{
      tb.innerHTML=d.peers.map(p=>
        '<tr><td><strong>'+p.name+'</strong></td>'+
        '<td>'+(p.dist<1?(p.dist*1000).toFixed(0)+'m':p.dist.toFixed(1)+'km')+'</td>'+
        '<td>'+fmt(p.seen)+'</td>'+
        '<td>'+sig(p.rssi)+'</td>'+
        '<td>'+p.bat+'%</td></tr>'
      ).join('');
    }
    document.getElementById('ts').textContent=new Date().toLocaleTimeString();
  }).catch(e=>{
    document.getElementById('peers').innerHTML='<tr><td colspan="5" class="empty err">Connection error</td></tr>';
  });
}
update();
setInterval(update,5000);
</script>
</body>
</html>
)rawliteral";
  webServer.send(200, "text/html", html);
}

// ── Config Portal HTML Page ─────────────────────────────────────────────────
// Full device configuration via web browser — no BLE needed
void handleConfigPage() {
  // Check lockdown mode
  if (deviceLocked) {
    webServer.send(200, "text/html", R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Azimuth - Locked</title>
<style>body{font-family:-apple-system,sans-serif;background:#0d1117;color:#c9d1d9;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0;padding:20px;box-sizing:border-box;}
.box{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:32px;text-align:center;max-width:320px;}
h1{color:#f85149;font-size:20px;margin:0 0 12px;}
p{color:#8b949e;font-size:14px;line-height:1.5;margin:0;}</style></head>
<body><div class="box"><h1>🔒 Device Locked</h1><p>Configuration is disabled. To unlock, go to Settings → Lock on the device screen and set to Off.</p></div></body></html>
)rawliteral");
    return;
  }

  // Extract device name without "Azimuth-" prefix
  const char* nameOnly = bleDeviceName;
  if (strncmp(nameOnly, "Azimuth-", 8) == 0) nameOnly += 8;

  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Azimuth Config</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0d1117;color:#c9d1d9;padding:16px;min-height:100vh}
h1{font-size:20px;color:#58a6ff;margin-bottom:16px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:16px;margin-bottom:16px}
.card h2{font-size:13px;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;margin-bottom:12px;border-bottom:1px solid #21262d;padding-bottom:8px}
.row{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-bottom:1px solid #21262d}
.row:last-child{border-bottom:none}
.row label{font-size:14px;color:#c9d1d9}
.row select,.row input[type="text"],.row input[type="number"]{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:8px 12px;color:#c9d1d9;font-size:14px;min-width:120px}
.row select:focus,.row input:focus{outline:none;border-color:#58a6ff}
.toggle{position:relative;width:44px;height:24px;cursor:pointer}
.toggle input{opacity:0;width:0;height:0}
.toggle span{position:absolute;inset:0;background:#21262d;border-radius:12px;transition:.2s}
.toggle input:checked+span{background:#238636}
.toggle span:before{content:'';position:absolute;width:18px;height:18px;left:3px;top:3px;background:#c9d1d9;border-radius:50%;transition:.2s}
.toggle input:checked+span:before{transform:translateX(20px)}
.btn{width:100%;padding:12px;background:#238636;border:none;border-radius:8px;color:#fff;font-size:14px;font-weight:600;cursor:pointer;margin-top:8px}
.btn:hover{background:#2ea043}
.btn:disabled{background:#21262d;color:#484f58;cursor:not-allowed}
.msg{padding:12px;border-radius:6px;margin-bottom:12px;font-size:13px;display:none}
.msg.ok{display:block;background:#0d2818;border:1px solid #238636;color:#3fb950}
.msg.err{display:block;background:#2d1216;border:1px solid #f85149;color:#f85149}
.hint{font-size:11px;color:#6e7681;margin-top:4px}
</style>
</head>
<body>
<h1>⚙️ Azimuth Config</h1>

<div id="msg" class="msg"></div>

<div class="card">
<h2>Device</h2>
<div class="row">
<label>Name</label>
<input type="text" id="name" maxlength="20" value=")rawliteral";
  html += nameOnly;
  html += R"rawliteral(">
</div>
</div>

<div class="card">
<h2>Display</h2>
<div class="row">
<label>Speed units</label>
<select id="spd">
<option value="0")rawliteral";
  html += setSpeedMph ? "" : " selected";
  html += R"rawliteral(>km/h</option>
<option value="1")rawliteral";
  html += setSpeedMph ? " selected" : "";
  html += R"rawliteral(>mph</option>
</select>
</div>
<div class="row">
<label>Elevation units</label>
<select id="elv">
<option value="0")rawliteral";
  html += setElevFt ? "" : " selected";
  html += R"rawliteral(>metres</option>
<option value="1")rawliteral";
  html += setElevFt ? " selected" : "";
  html += R"rawliteral(>feet</option>
</select>
</div>
<div class="row">
<label>Screen timeout</label>
<select id="tmo">
<option value="0")rawliteral";
  html += (setTimeoutIdx == 0) ? " selected" : "";
  html += R"rawliteral(>10s</option>
<option value="1")rawliteral";
  html += (setTimeoutIdx == 1) ? " selected" : "";
  html += R"rawliteral(>30s</option>
<option value="2")rawliteral";
  html += (setTimeoutIdx == 2) ? " selected" : "";
  html += R"rawliteral(>60s</option>
<option value="3")rawliteral";
  html += (setTimeoutIdx == 3) ? " selected" : "";
  html += R"rawliteral(>Never</option>
</select>
</div>
<div class="row">
<label>Theme</label>
<select id="brt">
<option value="1")rawliteral";
  html += setBrightFull ? " selected" : "";
  html += R"rawliteral(>Dark</option>
<option value="0")rawliteral";
  html += setBrightFull ? "" : " selected";
  html += R"rawliteral(>Light</option>
</select>
</div>
</div>

<div class="card">
<h2>LoRa Network</h2>
<div class="row">
<label>Region</label>
<select id="region">
<option value="0")rawliteral";
  html += (loraRegionIdx == 0) ? " selected" : "";
  html += R"rawliteral(>EU868</option>
<option value="1")rawliteral";
  html += (loraRegionIdx == 1) ? " selected" : "";
  html += R"rawliteral(>US915</option>
<option value="2")rawliteral";
  html += (loraRegionIdx == 2) ? " selected" : "";
  html += R"rawliteral(>AU915</option>
</select>
</div>
<div class="row">
<label>Broadcast interval</label>
<select id="interval">
<option value="10")rawliteral";
  html += (loraIntervalSec == 10) ? " selected" : "";
  html += R"rawliteral(>10s</option>
<option value="30")rawliteral";
  html += (loraIntervalSec == 30) ? " selected" : "";
  html += R"rawliteral(>30s</option>
<option value="60")rawliteral";
  html += (loraIntervalSec == 60) ? " selected" : "";
  html += R"rawliteral(>60s</option>
<option value="120")rawliteral";
  html += (loraIntervalSec == 120) ? " selected" : "";
  html += R"rawliteral(>2min</option>
<option value="300")rawliteral";
  html += (loraIntervalSec == 300) ? " selected" : "";
  html += R"rawliteral(>5min</option>
</select>
</div>
<div class="row">
<label>Relay mode</label>
<label class="toggle"><input type="checkbox" id="relay")rawliteral";
  html += relayEnabled ? " checked" : "";
  html += R"rawliteral(><span></span></label>
</div>
<div class="hint">Relay forwards packets for other devices. Enable at checkpoints or stationary positions.</div>
</div>

<button class="btn" onclick="saveConfig()">Save Settings</button>

<script>
async function saveConfig(){
  const btn=document.querySelector('.btn');
  const msg=document.getElementById('msg');
  btn.disabled=true;btn.textContent='Saving...';
  msg.className='msg';msg.style.display='none';
  try{
    // Device name
    const name=document.getElementById('name').value.trim();
    if(name)await fetch('/api/cmd',{method:'POST',body:'SETNAME:'+name});
    // Display settings
    const spd=document.getElementById('spd').value;
    const elv=document.getElementById('elv').value;
    const tmo=document.getElementById('tmo').value;
    const brt=document.getElementById('brt').value;
    await fetch('/api/cmd',{method:'POST',body:'SETSET:'+spd+':'+elv+':'+tmo+':'+brt});
    // LoRa settings
    const region=document.getElementById('region').value;
    const interval=document.getElementById('interval').value;
    await fetch('/api/cmd',{method:'POST',body:'SETLORA:'+region+':'+interval});
    // Relay mode
    const relay=document.getElementById('relay').checked?1:0;
    await fetch('/api/cmd',{method:'POST',body:'SETRELAY:'+relay});
    msg.className='msg ok';msg.textContent='✓ Settings saved successfully';msg.style.display='block';
  }catch(e){
    msg.className='msg err';msg.textContent='✗ Failed to save: '+e.message;msg.style.display='block';
  }
  btn.disabled=false;btn.textContent='Save Settings';
}
</script>
</body>
</html>
)rawliteral";
  webServer.send(200, "text/html", html);
}

void apiHandleMessages() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  char json[1024] = "[";
  bool first = true;
#if ENABLE_LORA
  for (int i = 0; i < LORA_MAX_MSGS; i++) {
    if (!loraMsgs[i].active) continue;
    if (!first) strncat(json, ",", sizeof(json)-strlen(json)-1);
    char entry[160];
    snprintf(entry, sizeof(entry),
      "{\"from\":\"%s\",\"name\":\"%s\",\"text\":\"%s\",\"ts\":%lu}",
      loraMsgs[i].fromId, loraMsgs[i].fromName, loraMsgs[i].text, loraMsgs[i].rxMs);
    strncat(json, entry, sizeof(json)-strlen(json)-1);
    first = false;
  }
#endif
  strncat(json, "]", sizeof(json)-strlen(json)-1);
  webServer.send(200, "application/json", json);
}

void apStart() {
  if (apEnabled) return;
  // Build SSID from device name
  const char* rawName = bleDeviceName;
  if (strncmp(rawName, "Azimuth-", 8) == 0) rawName += 8;
  snprintf(apSSID, sizeof(apSSID), "Azimuth-%s", rawName);
  // Use AP_STA if we have WiFi credentials, AP only otherwise
  WiFiMode_t mode = wifiEnabled ? WIFI_AP_STA : WIFI_AP;
  WiFi.mode(mode);
  WiFi.softAP(apSSID, apPass);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("[AP] Started: SSID=%s IP=%s\n", apSSID, apIP.toString().c_str());
  // DNS server — redirect all domains to captive portal
  dnsServer.start(53, "*", apIP);
  // Web server routes
  webServer.on("/", HTTP_GET, webServerHandleRoot);
  webServer.on("/index.html", HTTP_GET, webServerHandleRoot);
  webServer.on("/favicon.ico", HTTP_GET, [](){webServer.send(204);});
  webServer.on("/manifest.json", HTTP_GET, [](){
    webServer.send(200, "application/json",
      "{\"name\":\"Azimuth\",\"short_name\":\"Azimuth\","
      "\"start_url\":\"/\",\"display\":\"standalone\","
      "\"background_color\":\"#0d1117\",\"theme_color\":\"#238636\"}");
  });
  webServer.on("/generate_204", HTTP_GET, webServerHandleRoot);
  webServer.on("/hotspot-detect.html", HTTP_GET, webServerHandleRoot);
  // API routes
  webServer.on("/api/state",    HTTP_GET,  apiHandleState);
  webServer.on("/api/cmd",      HTTP_POST, apiHandleCmd);
  webServer.on("/api/messages", HTTP_GET,  apiHandleMessages);
  webServer.on("/api/mesh",     HTTP_GET,  apiHandleMesh);
  webServer.on("/api/groups",   HTTP_GET,  [](){
    webServer.sendHeader("Access-Control-Allow-Origin", "*");
    webServer.send(200, "application/json", buildGroupsJSON());
  });
  // Status page (standalone HTML, no PWA required)
  webServer.on("/status", HTTP_GET, handleStatusPage);
  // Config portal (full device setup via browser)
  webServer.on("/config", HTTP_GET, handleConfigPage);
  webServer.onNotFound(webServerHandleNotFound);
  webServer.begin();
  // mDNS — device accessible at azimuth.local
  if (MDNS.begin("azimuth")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[AP] mDNS: azimuth.local");
  }
  apEnabled = true;
  // Save AP enabled state to NVS
  prefs.begin("azimuth", false);
  prefs.putBool("apEnabled", true);
  prefs.end();
  oledDraw();
}

void apStop() {
  if (!apEnabled) return;
  webServer.stop();
  dnsServer.stop();
  MDNS.end();
  WiFi.softAPdisconnect(true);
  if (!wifiEnabled) WiFi.mode(WIFI_OFF);
  apEnabled = false;
  prefs.begin("azimuth", false);
  prefs.putBool("apEnabled", false);
  prefs.end();
  Serial.println("[AP] Stopped");
  oledDraw();
}

// ── Unit helpers ──────────────────────────────────────────────────────────
float displaySpd(float kmh) { return setSpeedMph ? kmh * 0.621371f : kmh; }
const char* spdUnit()       { return setSpeedMph ? "mph" : "km/h"; }
float displayAlt(float m)   { return setElevFt  ? m * 3.28084f  : m; }
const char* altUnit()       { return setElevFt  ? "ft"  : "m"; }
uint32_t timeoutMs() {
  uint16_t s = TIMEOUT_PRESETS[setTimeoutIdx];
  return s == 0 ? 0xFFFFFFFF : (uint32_t)s * 1000;
}

// COMPASS ROSE DRAW (shared by splash and page)
// ============================================================
void drawCompassRose(int cx, int cy, int cr) {
  display.drawCircle(cx, cy, cr,   SSD1306_WHITE);
  display.drawCircle(cx, cy, cr-6, SSD1306_WHITE);
  display.drawCircle(cx, cy, 3,    SSD1306_WHITE);
  display.drawLine(cx, cy-cr,   cx, cy-cr+8, SSD1306_WHITE);
  display.drawLine(cx, cy+cr,   cx, cy+cr-8, SSD1306_WHITE);
  display.drawLine(cx-cr, cy,   cx-cr+8, cy, SSD1306_WHITE);
  display.drawLine(cx+cr, cy,   cx+cr-8, cy, SSD1306_WHITE);
  int td = (int)(cr * 0.707);
  display.drawLine(cx-td,cy-td, cx-td+5,cy-td+5, SSD1306_WHITE);
  display.drawLine(cx+td,cy-td, cx+td-5,cy-td+5, SSD1306_WHITE);
  display.drawLine(cx-td,cy+td, cx-td+5,cy+td-5, SSD1306_WHITE);
  display.drawLine(cx+td,cy+td, cx+td-5,cy+td-5, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(cx-3, cy-cr-9); display.print("N");
  display.setCursor(cx-3, cy+cr+2); display.print("S");
  display.setCursor(cx-cr-8, cy-4); display.print("W");
  display.setCursor(cx+cr+2, cy-4); display.print("E");
}

void drawCompassFrame(int cx, int cy, int cr, float deg) {
  float rad = (deg-90.0)*DEG_TO_RAD;
  float perp = rad + PI/2.0;
  int tipX = cx+(int)(cos(rad)*(cr-4));
  int tipY = cy+(int)(sin(rad)*(cr-4));
  int bx   = cx-(int)(cos(rad)*(cr*0.55));
  int by   = cy-(int)(sin(rad)*(cr*0.55));
  int wx   = (int)(cos(perp)*3);
  int wy   = (int)(sin(perp)*3);
  display.clearDisplay();
  drawCompassRose(cx, cy, cr);
  display.fillTriangle(tipX,tipY, cx+wx,cy+wy, cx-wx,cy-wy, SSD1306_WHITE);
  display.drawTriangle(bx,by,     cx+wx,cy+wy, cx-wx,cy-wy, SSD1306_WHITE);
  display.display();
}

// ============================================================
// TITLE SCREEN
// ============================================================
// ============================================================
// TITLE SCREEN — Mountain peaks + AZIMUTH wipe animation
// Mountains scroll up, hold, slide back down, then AZIMUTH drops in
// Tight bounding box around text only — mountains visible on edges
// ============================================================

// Mountain peak coordinates (relative to baseY)
const int mtPeakX[] = {0, 10, 20, 32, 44, 56, 68, 82, 96, 108, 120, 128};
const float mtPeakH[] = {0, 0.56, 0.25, 1.0, 0.44, 0.81, 0.38, 0.94, 0.5, 0.75, 0.31, 0.5};
const int mtPeakCount = 12;

void drawMountains(int baseY, int height) {
  for (int i = 0; i < mtPeakCount - 1; i++) {
    int y1 = baseY - (int)(height * mtPeakH[i]);
    int y2 = baseY - (int)(height * mtPeakH[i + 1]);
    display.drawLine(mtPeakX[i], y1, mtPeakX[i + 1], y2, SSD1306_BLACK);
  }
}

// Draw mountains, but skip any pixels inside the clip rectangle
void drawMountainsClipped(int baseY, int height, int clipLeft, int clipTop, int clipRight, int clipBottom) {
  for (int i = 0; i < mtPeakCount - 1; i++) {
    int x0 = mtPeakX[i];
    int y0 = baseY - (int)(height * mtPeakH[i]);
    int x1 = mtPeakX[i + 1];
    int y1 = baseY - (int)(height * mtPeakH[i + 1]);
    
    // Draw line pixel by pixel, skipping clip region
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
      // Only draw if outside clip rectangle
      if (x0 < clipLeft || x0 >= clipRight || y0 < clipTop || y0 >= clipBottom) {
        display.drawPixel(x0, y0, SSD1306_BLACK);
      }
      if (x0 == x1 && y0 == y1) break;
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx) { err += dx; y0 += sy; }
    }
  }
}

// Draw final frame with mountains clipped around text
void drawTitleFinal() {
  display.fillScreen(SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  
  const int mtBaseY = 52;
  const int mtHeight = 35;
  const int textY = 18;
  
  // Text dimensions
  const int azimuthX = 23;
  const int azimuthW = 82;
  const int versionX = 37;
  const int versionW = 53;
  const int padL = 3, padR = 3, padT = 2, padB = 2;
  
  int boxLeft = min(azimuthX, versionX) - padL;
  int boxRight = max(azimuthX + azimuthW, versionX + versionW) + padR;
  int boxTop = textY - padT;
  int boxBottom = textY + 14 + 4 + 7 + padB;  // AZIMUTH + gap + version
  
  // Draw mountains, clipped to avoid text area
  drawMountainsClipped(mtBaseY, mtHeight, boxLeft, boxTop, boxRight, boxBottom);
  
  // Draw text
  display.setTextSize(2);
  display.setCursor(azimuthX, textY);
  display.print("AZIMUTH");
  
  display.setTextSize(1);
  display.setCursor(versionX, textY + 18);
  display.print("OEO v3.15");
  
  display.display();
}

void drawTitleScreen() {
  drawTitleFinal();
}

void showSplashCompass() {
  const int cx=64, cy=33, cr=21;
  for (int i=0; i<9; i++) {
    drawCompassFrame(cx, cy, cr, i*45.0);
    delay(450);
  }
}

void showSplashTitle() {
  display.clearDisplay();
  display.display();
  delay(150);
  
  const int mtBaseY = 52;
  const int mtHeight = 35;
  
  // Phase 1: Mountains scroll up from bottom (no clipping needed yet)
  const int scrollSteps = 14;
  const int scrollDelay = 45;
  for (int i = scrollSteps; i >= 0; i--) {
    int yOffset = i * 5;
    display.fillScreen(SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    drawMountains(mtBaseY + yOffset, mtHeight);
    display.display();
    delay(scrollDelay);
  }
  
  // Hold mountains
  delay(900);
  
  // Phase 2: AZIMUTH + version drop in from top, over the mountains
  const int textY = 18;
  const int wipeSteps = 16;
  const int wipeDelay = 35;
  
  // Text dimensions for tight bounding box
  const int azimuthX = 23;
  const int azimuthW = 82;
  const int versionX = 37;
  const int versionW = 53;
  const int padL = 3, padR = 3, padT = 2, padB = 2;
  int boxLeft = min(azimuthX, versionX) - padL;
  int boxRight = max(azimuthX + azimuthW, versionX + versionW) + padR;
  
  for (int i = 0; i <= wipeSteps; i++) {
    float progress = (float)i / wipeSteps;
    float eased = 1.0f - (1.0f - progress) * (1.0f - progress);
    int currentY = -20 + (int)((textY + 20) * eased);
    
    display.fillScreen(SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    
    // Calculate clip box for current text position
    int boxTop = currentY - padT;
    int boxBottom = currentY + 14 + 4 + 7 + padB;
    if (boxTop < 0) boxTop = 0;
    if (boxBottom > 64) boxBottom = 64;
    
    // Draw mountains, clipped around the text area
    drawMountainsClipped(mtBaseY, mtHeight, boxLeft, boxTop, boxRight, boxBottom);
    
    // Draw text
    display.setTextSize(2);
    display.setCursor(azimuthX, currentY);
    display.print("AZIMUTH");
    
    if (currentY >= 10) {
      display.setTextSize(1);
      display.setCursor(versionX, currentY + 18);
      display.print("OEO v3.15");
    }
    
    display.display();
    delay(wipeDelay);
  }
  
  // Hold final frame
  delay(2500);
  display.setTextSize(1);
}

void showSplash() {
  display.clearDisplay(); display.display();
  showSplashCompass();
  showSplashTitle();
}

// ============================================================
// NAV DOTS
// ============================================================
void drawNavDots() {
  int total = pageCount();
  int dotW  = 5, gap = 2;
  int rowW  = total*(dotW+gap)-gap;
  int startX = (128-rowW)/2;
  int y = 58;
  for (int i=0; i<total; i++) {
    int x = startX + i*(dotW+gap);
    if (i == currentPage) display.fillRect(x, y, dotW, dotW, SSD1306_WHITE);
    else                  display.drawRect(x, y, dotW, dotW, SSD1306_WHITE);
  }
}

// ============================================================
// PAGE TRANSITIONS
// ============================================================
// Buffer to store current screen for transitions
uint8_t screenBuffer[128 * 64 / 8];

void captureScreen() {
  memcpy(screenBuffer, display.getBuffer(), sizeof(screenBuffer));
}

void drawBufferOffset(int xOffset) {
  // Draw the captured buffer at an X offset
  for (int y = 0; y < 64; y++) {
    for (int x = 0; x < 128; x++) {
      int srcX = x - xOffset;
      if (srcX >= 0 && srcX < 128) {
        int byteIdx = srcX + (y / 8) * 128;
        int bit = (screenBuffer[byteIdx] >> (y & 7)) & 1;
        if (bit) display.drawPixel(x, y, SSD1306_WHITE);
      }
    }
  }
}

// Forward declaration
void oledDrawPage();

void slideToPage(int newPage, bool slideLeft) {
  // Capture current screen
  captureScreen();
  int oldPage = currentPage;
  currentPage = newPage;
  
  // Render new page to get its content
  display.clearDisplay();
  oledDrawPage();
  uint8_t newBuffer[128 * 64 / 8];
  memcpy(newBuffer, display.getBuffer(), sizeof(newBuffer));
  
  // Animate slide (4 frames, ~80ms total)
  const int steps = 4;
  const int stepDelay = 20;
  
  for (int i = 1; i <= steps; i++) {
    float progress = (float)i / steps;
    // Ease out
    float eased = 1.0f - (1.0f - progress) * (1.0f - progress);
    int offset = (int)(128 * eased);
    
    display.clearDisplay();
    
    if (slideLeft) {
      // Old page slides left (negative offset), new slides in from right
      // Draw old page
      for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 128; x++) {
          int srcX = x + offset;
          if (srcX >= 0 && srcX < 128) {
            int byteIdx = srcX + (y / 8) * 128;
            int bit = (screenBuffer[byteIdx] >> (y & 7)) & 1;
            if (bit) display.drawPixel(x - offset, y, SSD1306_WHITE);
          }
        }
      }
      // Draw new page
      for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 128; x++) {
          int destX = x + (128 - offset);
          if (destX >= 0 && destX < 128) {
            int byteIdx = x + (y / 8) * 128;
            int bit = (newBuffer[byteIdx] >> (y & 7)) & 1;
            if (bit) display.drawPixel(destX, y, SSD1306_WHITE);
          }
        }
      }
    } else {
      // Old page slides right, new slides in from left
      for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 128; x++) {
          int destX = x + offset;
          if (destX >= 0 && destX < 128) {
            int byteIdx = x + (y / 8) * 128;
            int bit = (screenBuffer[byteIdx] >> (y & 7)) & 1;
            if (bit) display.drawPixel(destX, y, SSD1306_WHITE);
          }
        }
      }
      for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 128; x++) {
          int srcX = x - (128 - offset);
          if (srcX >= 0 && srcX < 128) {
            int byteIdx = srcX + (y / 8) * 128;
            int bit = (newBuffer[byteIdx] >> (y & 7)) & 1;
            if (bit) display.drawPixel(x, y, SSD1306_WHITE);
          }
        }
      }
    }
    
    display.display();
    delay(stepDelay);
  }
  
  // Final frame — ensure clean render
  display.clearDisplay();
  oledDrawPage();
  display.display();
}

void nextPage() {
  int newPage = (currentPage + 1) % pageCount();
  slideToPage(newPage, true);  // slide left
}

void prevPage() {
  int newPage = (currentPage - 1 + pageCount()) % pageCount();
  slideToPage(newPage, false);  // slide right
}

// ============================================================
// PAGE DRAWS
// ============================================================

void drawPageGPS() {
  char buf[32];

  // Battery top-right — moved 4px right
  char batBuf[8];
  if (batOnUSB)
    snprintf(batBuf, sizeof(batBuf), "USB");
  else if (batCharging)
    snprintf(batBuf, sizeof(batBuf), "+%d%%", batPercent);
  else
    snprintf(batBuf, sizeof(batBuf), "%d%%", batPercent);
  display.setCursor(87, 1); display.print(batBuf);
  drawBatIcon(111, 1, batPercent);

  if (!cachedFix) {
    // ── No fix — map pin with expanding signal arcs ──
    const int cx = 63;   // horizontal centre

    // ── Map pin ──
    // Pin body: circle top, teardrop point at bottom
    // Circle centre at (cx, py), radius pr, point at (cx, py + pr + ptail)
    const int py   = 30;  // pin circle centre y
    const int pr   = 9;   // pin circle radius
    const int ptail = 6;  // length of tail below circle bottom

    // Filled circle (pin head)
    display.fillCircle(cx, py, pr, SSD1306_WHITE);

    // Filled triangle for the tail (teardrop point)
    // Triangle: left=(cx-pr+2, py+pr-2), right=(cx+pr-2, py+pr-2), tip=(cx, py+pr+ptail)
    display.fillTriangle(
      cx - pr + 2, py + pr - 2,
      cx + pr - 2, py + pr - 2,
      cx,          py + pr + ptail,
      SSD1306_WHITE
    );

    // Hollow inner circle (the white hole in the pin)
    const int ir = 6;
    display.fillCircle(cx, py, ir, SSD1306_BLACK);

    // ── Signal arcs expanding upward from pin circle top ──
    // Arc origin = top of pin circle
    const int ax = cx;
    const int ay = py - pr;   // top of pin circle
    uint8_t arcPhase = (millis() / 350) % 4;

    if (arcPhase >= 1) {
      for (int deg = 200; deg <= 340; deg += 4) {
        float r = deg * M_PI / 180.0f;
        int apx = ax + (int)(8  * cosf(r));
        int apy = ay + (int)(8  * sinf(r));
        if (apx >= 0 && apx < 128 && apy >= 0 && apy < 58) display.drawPixel(apx, apy, SSD1306_WHITE);
      }
    }
    if (arcPhase >= 2) {
      for (int deg = 207; deg <= 333; deg += 3) {
        float r = deg * M_PI / 180.0f;
        int apx = ax + (int)(14 * cosf(r));
        int apy = ay + (int)(14 * sinf(r));
        if (apx >= 0 && apx < 128 && apy >= 0 && apy < 58) display.drawPixel(apx, apy, SSD1306_WHITE);
      }
    }
    if (arcPhase >= 3) {
      for (int deg = 212; deg <= 328; deg += 3) {
        float r = deg * M_PI / 180.0f;
        int apx = ax + (int)(20 * cosf(r));
        int apy = ay + (int)(20 * sinf(r));
        if (apx >= 0 && apx < 128 && apy >= 0 && apy < 58) display.drawPixel(apx, apy, SSD1306_WHITE);
      }
    }

    // "Acquiring Signal..." centred, 5px above nav dots (nav at y=58, text at y=49)
    display.setCursor(10, 47);
    display.print("Acquiring Signal");
    uint8_t dots = (millis() / 500) % 3 + 1;
    for (uint8_t d = 0; d < dots; d++) display.print(".");

  } else {
    // ── Fix acquired — Position page ────────────────────────────
    // Header — crosshair icon + title + shortened divider (stops before battery)
    display.drawBitmap(2, 1, ICON_CRS2, 8, 8, SSD1306_WHITE);
    display.setCursor(13, 1); display.println("Position");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    // Data rows
    display.drawBitmap(2, 13, ICON_GPS, 8, 8, SSD1306_WHITE);
    snprintf(buf, sizeof(buf), "%.6f", cachedLat);
    display.setCursor(13, 14); display.println(buf);

    display.drawBitmap(2, 24, ICON_GPS, 8, 8, SSD1306_WHITE);
    snprintf(buf, sizeof(buf), "%.6f", cachedLon);
    display.setCursor(13, 25); display.println(buf);

#if ENABLE_LORA
    // Unread message indicator — to the right of lon row
    if (loraMsgUnread > 0) {
      char msgBuf[6];
      snprintf(msgBuf, sizeof(msgBuf), "M:%d", loraMsgUnread);
      display.setCursor(92, 25); display.print(msgBuf);
    }
#endif

    display.drawBitmap(2, 36, ICON_SPD, 8, 8, SSD1306_WHITE);
    snprintf(buf, sizeof(buf), "%.1f %s", displaySpd(cachedSpd), spdUnit());
    display.setCursor(13, 36); display.println(buf);

    display.drawBitmap(2, 46, ICON_HDG, 8, 8, SSD1306_WHITE);
    snprintf(buf, sizeof(buf), "%.0fdeg %s", cachedHdg, getCardinal(cachedHdg));
    display.setCursor(13, 47); display.println(buf);
    // Altitude — fixed right side of same row
    snprintf(buf, sizeof(buf), "%.0f%s", displayAlt(cachedAlt), altUnit());
    display.setCursor(92, 47); display.println(buf);
  }
}

// ── Hidden About screen — triggered by long press on Device Information ──────
#define FW_VERSION  "v3.16"
#define FW_BUILD    "Apr 2026"
#define OEO_WEB     "oeo.dev/azimuth"
#define OEO_COPY    "2026 OEO"

void drawAboutScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(2, 2);  display.print("Version: "); display.println(FW_VERSION);
  display.setCursor(2, 12); display.print("Build:   "); display.println(FW_BUILD);
  display.setCursor(2, 22); display.println("By OEO Technologies");
  display.setCursor(2, 32); display.println(OEO_WEB);
  display.setCursor(2, 42); display.print("(c) "); display.println(OEO_COPY);

  display.display();
}
void drawPageInfo() {
  char buf[32];

  // Header — cog icon + title + BLE connected indicator top-right
  display.drawBitmap(2, 1, ICON_COG, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1); display.println(bleDeviceName);
  if (bleConnected) display.drawBitmap(119, 1, ICON_BT, 8, 8, SSD1306_WHITE);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Row 1 — BLE PIN
  display.drawBitmap(2, 13, ICON_BT, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 14);
  if (pairingPending) {
    // Keep showing PIN until pairing is complete
    snprintf(buf, sizeof(buf), "PIN: %06lu", pairingPin);
    display.println(buf);
  } else if (bleConnected) {
    display.println(bleDeviceName);
  } else {
    snprintf(buf, sizeof(buf), "PIN: %06lu", pairingPin);
    display.println(buf);
  }

  // Row 2 — AP status or uptime
  display.drawBitmap(2, 24, ICON_CLK, 8, 8, SSD1306_WHITE);
  if (apEnabled) {
    snprintf(buf, sizeof(buf), "%s", apSSID);
    display.setCursor(13, 25); display.println(buf);
  } else {    snprintf(buf, sizeof(buf), "Up: %lus", millis() / 1000);
    display.setCursor(13, 25); display.println(buf);
  }

  // Row 3 — AP IP or satellites
  display.drawBitmap(2, 35, ICON_SAT, 8, 8, SSD1306_WHITE);
  if (apEnabled) {
    snprintf(buf, sizeof(buf), "azimuth.local");
    display.setCursor(13, 36); display.println(buf);
  } else {
    snprintf(buf, sizeof(buf), "Sats: %u", cachedSats);
    display.setCursor(13, 36); display.println(buf);
  }

  // Row 4 — Battery, left-aligned with other rows (no separate bat icon)
  display.drawBitmap(2, 47, ICON_PWR, 8, 8, SSD1306_WHITE);
  if (batOnUSB)
    snprintf(buf, sizeof(buf), "USB power");
  else
    snprintf(buf, sizeof(buf), "%d%%  %.2fV", batPercent, batVoltage);
  display.setCursor(13, 48); display.println(buf);
}

void drawPageCompass() {
  // Header
  display.drawBitmap(2, 1, ICON_COMPASS, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1); display.println("Compass");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Compass rose — resized cr=18, cy=38 (1px gap from header divider)
  const int cx=30, cy=38, cr=18;
  display.drawCircle(cx,cy,cr,SSD1306_WHITE);
  display.drawLine(cx,cy-cr,   cx,cy-cr+4,   SSD1306_WHITE);
  display.drawLine(cx,cy+cr,   cx,cy+cr-4,   SSD1306_WHITE);
  display.drawLine(cx-cr,cy,   cx-cr+4,cy,   SSD1306_WHITE);
  display.drawLine(cx+cr,cy,   cx+cr-4,cy,   SSD1306_WHITE);
  display.setCursor(cx-2,cy-cr-7); display.println("N");
  display.setCursor(cx-2,cy+cr+1); display.println("S");
  display.setCursor(cx-cr-6,cy-3); display.println("W");
  display.setCursor(cx+cr+2,cy-3); display.println("E");

  if (cachedFix && cachedSpd > 1.5) drawNeedle(cx,cy,cr-4,cachedHdg);
  else { display.setCursor(cx-4,cy-3); display.println("--"); }

  // Vertical divider — starts below header line
  display.drawLine(62, 11, 62, 63, SSD1306_WHITE);

  // Right panel — all items shifted +1px down, +2px right (x=67)
  char buf[16];
  if (cachedFix && cachedSpd > 1.5) {
    snprintf(buf, sizeof(buf), "HDG:%.0f", cachedHdg);
    display.setCursor(67, 15); display.println(buf);
    snprintf(buf, sizeof(buf), "BRG:%s", getCardinal(cachedHdg));
    display.setCursor(67, 25); display.println(buf);
    snprintf(buf, sizeof(buf), "SPD:%.1f", displaySpd(cachedSpd));
    display.setCursor(67, 35); display.println(buf);
  } else {
    display.setCursor(67, 15); display.println("HDG:---");
    display.setCursor(67, 25); display.println("BRG:---");
    display.setCursor(67, 35); display.println("SPD:---");
  }
  // 4th row: DST to saved location if saved, else SAT count
  if (locSaved) {
    float dst = haversineDistance(cachedLat, cachedLon, savedLat, savedLon);
    if (dst >= 1000)
      snprintf(buf, sizeof(buf), "WPT:%.1fk", dst/1000.0);
    else
      snprintf(buf, sizeof(buf), "WPT:%.0fm", dst);
    display.setCursor(67, 45); display.println(buf);
  } else {
    snprintf(buf, sizeof(buf), "SAT:%u", cachedSats);
    display.setCursor(67, 45); display.println(buf);
  }

  // Nav dots — small, right-aligned in right panel
  {
    int total = pageCount();
    const int dotW = 3, gap = 2;
    int rowW = total*(dotW+gap)-gap;
    int startX = 127 - rowW;
    int y = 59;
    for (int i = 0; i < total; i++) {
      int x = startX + i*(dotW+gap);
      if (x < 64) continue;  // stay in right panel only
      if (i == currentPage) display.fillRect(x, y, dotW, dotW, SSD1306_WHITE);
      else                  display.drawRect(x, y, dotW, dotW, SSD1306_WHITE);
    }
  }}

void drawPageNavigate() {
  // Compass on RIGHT side — cx=97, cy=32, cr=24, divider at x=62
  const int cx=97, cy=32, cr=24;

  // Determine target — waypoints take priority over saved location
  bool hasTarget = false;
  float targetLat = 0, targetLon = 0;
  char  targetName[WP_NAME_LEN + 1] = {0};
  bool  isWaypoint = false;

  if (waypointCount > 0) {
    targetLat  = waypoints[waypointCurrent].lat;
    targetLon  = waypoints[waypointCurrent].lon;
    strncpy(targetName, waypoints[waypointCurrent].name, WP_NAME_LEN);
    hasTarget  = true;
    isWaypoint = true;
  } else if (locSaved) {
    targetLat  = (float)savedLat;
    targetLon  = (float)savedLon;
    strncpy(targetName, "Waypoint", WP_NAME_LEN);
    hasTarget  = true;
  }

  float dist    = hasTarget ? haversineDistance(cachedLat,cachedLon,targetLat,targetLon) : 0;
  float bearing = hasTarget ? bearingTo(cachedLat,cachedLon,targetLat,targetLon) : 0;

  display.drawCircle(cx,cy,cr,SSD1306_WHITE);
  display.drawLine(cx,cy-cr,   cx,cy-cr+4,   SSD1306_WHITE);
  display.drawLine(cx,cy+cr,   cx,cy+cr-4,   SSD1306_WHITE);
  display.drawLine(cx-cr,cy,   cx-cr+4,cy,   SSD1306_WHITE);
  display.drawLine(cx+cr,cy,   cx+cr-4,cy,   SSD1306_WHITE);
  display.setCursor(cx-2,cy-cr-8); display.println("N");
  display.setCursor(cx-2,cy+cr+1); display.println("S");
  display.setCursor(cx-cr-7,cy-3); display.println("W");
  display.setCursor(cx+cr+2,cy-3); display.println("E");

  if (!hasTarget) {
    display.setCursor(cx-10,cy-3); display.println("No WP");
  } else if (cachedFix) {
    drawNeedle(cx,cy,cr-4,bearing);
  } else {
    display.setCursor(cx-4,cy-3); display.println("--");
  }

  // Vertical divider
  display.drawLine(62,0,62,63,SSD1306_WHITE);

  // Left panel
  char buf[20];
  if (!hasTarget) {
    display.setCursor(1,12); display.println("No waypoints");
    display.setCursor(1,24); display.println("Set via app");
  } else {
    // Waypoint name — y=4
    char shortName[10];
    strncpy(shortName, targetName, 9); shortName[9] = 0;
    display.setCursor(1,4); display.println(shortName);

    if (dist >= 1000.0) snprintf(buf,sizeof(buf),"%.2f km",dist/1000.0);
    else                snprintf(buf,sizeof(buf),"%.0f m",dist);

    bool near = (dist < 20.0);
    if (near) {
      display.fillRect(0,10,61,9,SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    display.setCursor(1,12); display.println(buf);
    if (near) display.setTextColor(SSD1306_WHITE);

    display.setCursor(1,24); display.println(getCardinal(bearing));
    snprintf(buf,sizeof(buf),"%.0f deg",bearing);
    display.setCursor(1,33); display.println(buf);
  }

  // Bottom rule + hint
  display.drawLine(0,50,61,50,SSD1306_WHITE);
  if (isWaypoint) {
    snprintf(buf,sizeof(buf),"%d/%d Hold:next",waypointCurrent+1,waypointCount);
    display.setCursor(1,54); display.println(buf);
  } else {
    display.setCursor(1,54); display.println("Hold:clear");
  }
}

#if ENABLE_LORA
void drawPageGroups() {
  display.setTextSize(1);
  
  // State 1: Pending key share request (recipient)
  if (pendingKeyShare.active) {
    display.setCursor(0, 0);
    display.println("Join group?");
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    // Truncate group name to fit
    char truncName[20];
    strncpy(truncName, pendingKeyShare.groupName, 19);
    truncName[19] = 0;
    display.setCursor(0, 14);
    display.println(truncName);
    display.setCursor(0, 28);
    display.print("Via: ");
    display.println(pendingKeyShare.leaderName);
    display.setCursor(0, OLED_HINT_Y);
    display.println("Hld:Join Dbl:No");
    return;
  }
  
  // State 2: Key share active (leader broadcasting)
  if (keyShareActive) {
    display.setCursor(0, 0);
    display.println("Sharing Key");
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    char truncName[16];
    strncpy(truncName, keyShareGroupName, 15);
    truncName[15] = 0;
    display.setCursor(0, 14);
    display.println(truncName);
    display.setCursor(0, 26);
    display.print("Joined: ");
    display.println(keyShareAckCount);
    uint32_t elapsed = (millis() - keyShareSentMs) / 1000;
    display.print("Elapsed: ");
    display.print(elapsed);
    display.println("s");
    display.setCursor(0, OLED_HINT_Y);
    display.println("Lng:Done Dbl:Cancel");
    return;
  }
  
  // State 3: Sub-menu open for selected group
  if (groupMenuOpen) {
    // Header = group name (or "Personal")
    display.setCursor(0, 0);
    if (storedGroupSel == 0) {
      display.println("Personal");
    } else {
      char truncName[18];
      strncpy(truncName, storedGroups[storedGroupSel - 1].name, 17);
      truncName[17] = 0;
      display.println(truncName);
    }
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    
    // Menu items depend on selection
    bool isPersonal = (storedGroupSel == 0);
    bool isActive = isPersonal ? (activeStoredGroup == -1) : (activeStoredGroup == storedGroupSel - 1);
    bool isLeader = !isPersonal && storedGroups[storedGroupSel - 1].isLeader;
    bool canLeave = isPersonal || storedGroups[storedGroupSel - 1].canLeave;
    
    // Build menu items dynamically
    const char* menuItems[4];
    int menuCount = 0;
    
    // Only show Activate/Deactivate if there's something to switch to
    if (isPersonal && isActive && storedGroupCount == 0) {
      // Personal is active and only option - no activate/deactivate
    } else if (isActive) {
      menuItems[menuCount++] = "Deactivate";
    } else {
      menuItems[menuCount++] = "Activate";
    }
    
    if (!isPersonal && isLeader) {
      menuItems[menuCount++] = "Share Key";
    }
    
    if (!isPersonal && canLeave) {
      menuItems[menuCount++] = "Leave Group";
    }
    
    menuItems[menuCount++] = "Back";
    
    // Clamp selection
    if (groupMenuSel >= menuCount) groupMenuSel = menuCount - 1;
    
    // Draw menu items (max 3 fit above hint line)
    int y = 14;
    for (int i = 0; i < menuCount && i < 3; i++) {
      bool sel = (i == groupMenuSel);
      if (sel) {
        display.fillRect(0, y - 1, 128, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
      }
      display.setCursor(2, y);
      display.print(sel ? "> " : "  ");
      display.println(menuItems[i]);
      if (sel) display.setTextColor(SSD1306_WHITE);
      y += 10;
    }
    
    display.setCursor(0, OLED_HINT_Y);
    display.println("Sht:Cycle Lng:Select");
    return;
  }
  
  // State 4: Group list (with Personal at top)
  display.setCursor(0, 0);
  display.println("Groups");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Count active peers (those seen in last 2 mins)
  int activePeerCount = 0;
  uint32_t now = millis();
  for (int p = 0; p < LORA_MAX_PEERS; p++) {
    if (loraPeers[p].active && (now - loraPeers[p].lastSeenMs < 120000)) {
      activePeerCount++;
    }
  }
  
  // Total items = 1 (Personal) + storedGroupCount
  int totalItems = 1 + storedGroupCount;
  
  // Calculate visible window (max 3 items fit above hint)
  int startIdx = 0;
  if (storedGroupSel > 1) startIdx = storedGroupSel - 1;
  if (startIdx + 3 > totalItems) startIdx = max(0, totalItems - 3);
  
  int y = 14;
  for (int i = startIdx; i < startIdx + 3 && i < totalItems; i++) {
    bool selected = (i == storedGroupSel);
    bool active;
    const char* name;
    
    if (i == 0) {
      // Personal option
      name = "Personal";
      active = (activeStoredGroup == -1);
    } else {
      // Stored group (index i-1)
      name = storedGroups[i - 1].name;
      active = (activeStoredGroup == i - 1);
    }
    
    if (selected) {
      display.fillRect(0, y - 1, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    
    display.setCursor(0, y);
    display.print(selected ? ">" : " ");
    
    // Truncate name (shorter if active to fit count)
    char truncName[14];
    int maxLen = active ? 10 : 15;
    strncpy(truncName, name, maxLen);
    truncName[maxLen] = 0;
    display.print(truncName);
    
    // Show peer count for active group
    if (active) {
      char countBuf[8];
      snprintf(countBuf, sizeof(countBuf), " (%d)", activePeerCount);
      display.print(countBuf);
    }
    
    if (selected) display.setTextColor(SSD1306_WHITE);
    y += 10;
  }
  
  display.setCursor(0, OLED_HINT_Y);
  display.println("Dbl:Cycle Lng:Menu");
}
#endif

void drawPageClock() {
  char buf[24];
  int hour,minute,second,day,month,year;
  getLocalTime(hour,minute,second,day,month,year);
  bool valid = gps.time.isValid() && gps.date.isValid();

  display.drawBitmap(2, 1, ICON_TIME,8,8,SSD1306_WHITE);
  display.setCursor(13, 1);
  display.println(tzAdjustMode ? "SET TIMEZONE" : "Clock");
  display.drawLine(0,10,127,10,SSD1306_WHITE);

  if (tzAdjustMode) {
    display.setTextSize(2);
    snprintf(buf,sizeof(buf),"UTC%+d",tzOffset);
    display.setCursor(20,20); display.println(buf);
    display.setTextSize(1);
    display.setCursor(2, 42); display.println("Short: +1hr  wrap");
    display.setCursor(2, 52); display.println("Hold: save & exit");
  } else {
#ifdef SIMULATE_GPS
    // Fake ticking time for testing
    uint32_t fakeSec = (millis() / 1000) % 86400;
    int fakeH = ((14 + tzOffset) + fakeSec/3600) % 24;
    int fakeM = (fakeSec % 3600) / 60;
    int fakeS = fakeSec % 60;
    display.setTextSize(2);
    snprintf(buf,sizeof(buf),"%02d:%02d:%02d",fakeH,fakeM,fakeS);
    display.setCursor(16,15); display.println(buf);
    display.setTextSize(1);
    display.drawLine(0,34,127,34,SSD1306_WHITE);
    display.setCursor(2, 39); display.println("19/03/2026");
    snprintf(buf,sizeof(buf),"UTC%+d",tzOffset);
    display.setCursor(92,39); display.println(buf);
#else
    if (valid) {
      display.setTextSize(2);
      snprintf(buf,sizeof(buf),"%02d:%02d:%02d",hour,minute,second);
      display.setCursor(16,15); display.println(buf);
      display.setTextSize(1);
      display.drawLine(0,34,127,34,SSD1306_WHITE);
      snprintf(buf,sizeof(buf),"%02d/%02d/%04d",day,month,year);
      display.setCursor(2, 39); display.println(buf);
      snprintf(buf,sizeof(buf),"UTC%+d",tzOffset);
      display.setCursor(92,39); display.println(buf);
    } else {
      display.setTextSize(2);
      display.setCursor(16,15); display.println("--:--:--");
      display.setTextSize(1);
      display.setCursor(2, 39); display.println("No GPS time fix");
    }
#endif
    display.setCursor(2, 48); display.println("Hold: set timezone");
  }
  display.setTextSize(1);
}

void drawPageCountdown() {
  char buf[24];

  // Header — title changes in set mode
  display.drawBitmap(2, 1, ICON_CDT, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1);
  if (cdtSetMode)       display.println("Set countdown");
  else if (cdtRunning)  display.println("Countdown");
  else if (cdtDone)     display.println("Time's up!");
  else                  display.println("Countdown");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Calculate display time
  uint32_t dispMs;
  if (cdtRunning) {
    uint32_t elapsed = millis() - cdtStartMs;
    dispMs = (elapsed < cdtRemMs) ? cdtRemMs - elapsed : 0;
  } else {
    dispMs = cdtDone ? 0 : cdtRemMs;
  }

  uint32_t dispSec = dispMs / 1000;
  uint32_t mm = dispSec / 60;
  uint32_t ss = dispSec % 60;

  // Large centred time — size 2, y=15
  display.setTextSize(2);
  snprintf(buf, sizeof(buf), "%02lu:%02lu", mm, ss);
  display.setCursor(16, 15); display.println(buf);
  display.setTextSize(1);

  display.drawLine(0, 34, 127, 34, SSD1306_WHITE);

  // Set mode — show preset label and cycling hint
  if (cdtSetMode) {
    uint32_t setMin = cdtSetSecs / 60;
    uint32_t setSec = cdtSetSecs % 60;
    if (setSec > 0)
      snprintf(buf, sizeof(buf), "%lum %lus", setMin, setSec);
    else
      snprintf(buf, sizeof(buf), "%lu min", setMin);
    display.setCursor(2, 39); display.println(buf);
    display.setCursor(2, 49); display.println("Press:preset Hold:done");
  } else if (cdtDone) {
    display.setCursor(2, 39); display.println("Timer complete");
    display.setCursor(2, 49); display.println("Hold: reset");
  } else if (cdtRunning) {
    display.setCursor(2, 39); display.println("Running...");
    display.setCursor(2, 49); display.println("Dbl: stop");
  } else {
    uint32_t setMin = cdtSetSecs / 60;
    snprintf(buf, sizeof(buf), "Set: %lu min", setMin);
    display.setCursor(2, 39); display.println(buf);
    display.setCursor(2, 49); display.println("Dbl:start Hold:set");
  }
}

void drawPageStopwatch() {
  char buf[24];
  uint32_t ms = swCurrentMs();

  // Header — same layout as clock page
  display.drawBitmap(2, 1, ICON_TIMER, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1);
  display.println(swRunning ? "Stopwatch" : (swElapsedMs > 0 ? "Stopped" : "Stopwatch"));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Time — setTextSize(2), y=15, centred — matches clock page
  formatTime(ms, buf, sizeof(buf));
  display.setTextSize(2);
  int len = strlen(buf);
  display.setCursor((128 - len * 12) / 2, 15);
  display.println(buf);
  display.setTextSize(1);

  // Bottom line at y=34 — matches clock page (4px below time bottom y=31)
  display.drawLine(0, 34, 127, 34, SSD1306_WHITE);

  // Instructions
  if (!swRunning && swElapsedMs == 0) {
    display.setCursor(2, 39); display.println("Dbl: start");
  } else if (swRunning) {
    display.setCursor(2, 39); display.println("Dbl: stop");
  } else {
    display.setCursor(2, 39); display.println("Dbl: resume");
  }
  display.setCursor(2, 48); display.println("Hold: clear");
}


void drawPageRoute() {
  char buf[32];
  display.drawBitmap(2, 1, ICON_ROUTE,8,8,SSD1306_WHITE);
  display.setCursor(13, 1);
  display.println(routeRecording ? "Recording..." : "Track");
  display.drawLine(0,10,127,10,SSD1306_WHITE);

  snprintf(buf,sizeof(buf),"Points: %d",routeCount);
  display.setCursor(2, 14); display.println(buf);

  if (routeTotalDist>=1000.0)
    snprintf(buf,sizeof(buf),"Dist: %.2f km",routeTotalDist/1000.0);
  else
    snprintf(buf,sizeof(buf),"Dist: %.0f m",routeTotalDist);
  display.setCursor(2, 24); display.println(buf);

  if (routeRecording) {
    uint32_t el = millis()-routeStartMs;
    snprintf(buf,sizeof(buf),"Time:  %02lu:%02lu",el/60000,(el%60000)/1000);
    display.setCursor(2, 34); display.println(buf);
  }
  if (!cachedFix) { display.setCursor(2, 34); display.println("Waiting for fix..."); }

  display.drawLine(0,45,127,45,SSD1306_WHITE);
  display.setCursor(2, 49);
  if (routeRecording)        display.println("Dbl:stop  Hold:clear");
  else if (routeCount>0)     display.println("Dbl:start Hold:clear");
  else                       display.println("Dbl:start");
}

// Route log icon
static const uint8_t PROGMEM ICON_LOG[] = {
  0b11111110,
  0b10000010,
  0b10111010,
  0b10100010,
  0b10111010,
  0b10000010,
  0b11111110,
  0b00000000,
};

void drawPageRouteLog() {
  char buf[32];

  // Header
  display.drawBitmap(2, 1, ICON_LOG, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1);
  display.print("Track Log");
  if (routeRecording) {
    display.setCursor(80, 1); display.print("*REC");
  }
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (logClearWarn) {
    // Warning — track recording in progress
    display.setCursor(2, 14); display.println("! Track in progress");
    display.setCursor(2, 24); display.println("It won't be saved.");
    display.drawLine(0, 34, 127, 34, SSD1306_WHITE);
    display.setCursor(2, 38); display.println("Press: cancel");
    display.setCursor(2, 48); display.println("Hold: clear anyway");
  } else {
    // Count and total summary
    snprintf(buf, sizeof(buf), "Tracks: %d", routeLogCount);
    display.setCursor(2, 14); display.println(buf);

    int totalPts = 0;
    float totalDist = 0;
    for (int i = 0; i < routeLogCount; i++) {
      totalPts  += routeLog[i].count;
      totalDist += routeLog[i].distM;
    }
    if (totalDist >= 1000)
      snprintf(buf, sizeof(buf), "%.1fkm  %dpts", totalDist/1000.0, totalPts);
    else
      snprintf(buf, sizeof(buf), "%.0fm  %dpts", totalDist, totalPts);
    display.setCursor(2, 23); display.println(buf);

    display.drawLine(0, 32, 127, 32, SSD1306_WHITE);

    // Show last 2 tracks
    int start = max(0, routeLogCount - 2);
    for (int i = start; i < routeLogCount; i++) {
      float d = routeLog[i].distM;
      if (d >= 1000)
        snprintf(buf, sizeof(buf), "#%d: %.1fkm %dpts", i+1, d/1000.0, routeLog[i].count);
      else
        snprintf(buf, sizeof(buf), "#%d: %.0fm %dpts",  i+1, d, routeLog[i].count);
      display.setCursor(2, 35 + (i - start) * 10);
      display.println(buf);
    }

    // Hold hint — no divider, sits clear of nav dots
    display.setCursor(2, 50); display.println("Hold: clear all");
  }
}

void drawPageElevation() {
  char buf[32];

  // Header
  display.drawBitmap(2, 1, ICON_ELEV, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1); display.println("Elevation");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (!cachedFix) {
    display.setCursor(2, 28); display.println("Acquiring signal...");
    return;
  }

  // Choose data source:
  // 1. Active track recording
  // 2. Last completed track log entry
  // 3. Live rolling buffer
  float*  pts  = nullptr;
  int     npts = 0;
  bool    usingBuf = false;

  if (routeRecording && routeCount > 1) {
    // Build temp float array from track alt values
    // Use circular elev buf as proxy — it mirrors track
    pts  = elevBuf;
    npts = constrain(elevBufCount, 2, ELEV_BUF_SIZE);
    usingBuf = true;
  } else if (routeLogCount > 0) {
    // Last completed track — build temp altitude array
    RoutePoint* rp   = routeLog[routeLogCount-1].points;
    int         rCnt = routeLog[routeLogCount-1].count;
    int         step = max(1, rCnt / 128);
    npts = 0;
    for (int i = 0; i < rCnt && npts < 128; i += step)
      trackAltBuf[npts++] = rp[i].alt;
    pts = trackAltBuf;
  } else {
    // Live rolling buffer
    pts  = elevBuf;
    npts = constrain(elevBufCount, 2, ELEV_BUF_SIZE);
    usingBuf = true;
  }

  if (npts < 2) {
    display.setCursor(2, 28); display.println("Building history...");
    snprintf(buf, sizeof(buf), "Alt: %.0fm", cachedAlt);
    display.setCursor(2, 38); display.println(buf);
    return;
  }

  // Find min/max
  float altMin = 99999.0, altMax = -99999.0;
  // For circular buffer, read in order
  for (int i = 0; i < npts; i++) {
    float v;
    if (usingBuf) {
      // Read from oldest to newest
      int idx = (elevBufHead - npts + i + ELEV_BUF_SIZE) % ELEV_BUF_SIZE;
      v = elevBuf[idx];
    } else {
      v = pts[i];
    }
    if (v < altMin) altMin = v;
    if (v > altMax) altMax = v;
  }
  float altRange = altMax - altMin;
  if (altRange < 1.0) altRange = 1.0;

  // Stats row
  snprintf(buf, sizeof(buf), "Lo:%.0fm Hi:%.0fm", altMin, altMax);
  display.setCursor(2, 14); display.print(buf);
  snprintf(buf, sizeof(buf), "%.0f%s", displayAlt(cachedAlt), altUnit());
  display.setCursor(103, 14); display.println(buf);

  // Graph area: x=0-127, y=22-63 (42px tall)
  const int gX=0, gY=22, gW=128, gH=42;

  if (!elevLineMode) {
    // ── Bar graph ────────────────────────────────────────────
    for (int col = 0; col < gW; col++) {
      int ptIdx = (int)((float)col / (float)(gW-1) * (float)(npts-1));
      float v;
      if (usingBuf) {
        int bufIdx = (elevBufHead - npts + ptIdx + ELEV_BUF_SIZE) % ELEV_BUF_SIZE;
        v = elevBuf[bufIdx];
      } else {
        v = pts[ptIdx];
      }
      int barH = (int)((v - altMin) / altRange * (float)(gH-1));
      barH = constrain(barH, 1, gH);
      display.drawFastVLine(gX+col, gY+gH-barH, barH, SSD1306_WHITE);
    }
  } else {
    // ── Line graph ───────────────────────────────────────────
    int prevX = -1, prevY = -1;
    for (int col = 0; col < gW; col++) {
      int ptIdx = (int)((float)col / (float)(gW-1) * (float)(npts-1));
      float v;
      if (usingBuf) {
        int bufIdx = (elevBufHead - npts + ptIdx + ELEV_BUF_SIZE) % ELEV_BUF_SIZE;
        v = elevBuf[bufIdx];
      } else {
        v = pts[ptIdx];
      }
      int y = gY + gH - 1 - (int)((v - altMin) / altRange * (float)(gH-1));
      y = constrain(y, gY, gY+gH-1);
      if (prevX >= 0)
        display.drawLine(prevX, prevY, gX+col, y, SSD1306_WHITE);
      prevX = gX+col; prevY = y;
    }
  }
}

void drawPageSaveLoc() {
  char buf[24];

  // Header — Pin icon + Waypoint title
  display.drawBitmap(2, 1, ICON_PIN, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1); display.println("Waypoint");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Current position
  display.setCursor(2, 14);
  display.println(cachedFix ? "Current position:" : "No GPS fix");
  if (cachedFix) {
    snprintf(buf, sizeof(buf), "%.6f", cachedLat);
    display.setCursor(2, 23); display.println(buf);
    snprintf(buf, sizeof(buf), "%.6f", cachedLon);
    display.setCursor(2, 32); display.println(buf);
  }

  display.drawLine(0, 42, 127, 42, SSD1306_WHITE);

  // Hint
  display.setCursor(2, 46);
  display.println(cachedFix ? "Hold: mark waypoint" : "Waiting for fix...");
}

void drawPairingScreen() {
  display.clearDisplay();
  // Header
  display.setCursor(2, 1); display.println("Pair Device");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Instruction
  display.setCursor(2, 14); display.println("Enter this PIN");
  display.setCursor(2, 24); display.println("in your browser:");

  // PIN — large, centred
  char pinStr[8];
  snprintf(pinStr, sizeof(pinStr), "%06lu", pairingPin);
  // Draw each digit larger using size 2
  display.setTextSize(2);
  int pinW = 6 * 6 * 2;  // 6 digits * 6px * scale2
  display.setCursor((128 - pinW) / 2, 36);
  display.println(pinStr);
  display.setTextSize(1);
  display.display();
}

// Weather condition code to short description
const char* weatherIcon(int id) {
  if (id >= 200 && id < 300) return "Storm";
  if (id >= 300 && id < 400) return "Drizzle";
  if (id >= 500 && id < 600) return "Rain";
  if (id >= 600 && id < 700) return "Snow";
  if (id >= 700 && id < 800) return "Mist";
  if (id == 800)              return "Clear";
  if (id == 801)              return "Few clouds";
  if (id == 802)              return "Scattered";
  if (id >= 803)              return "Cloudy";
  return "?";
}

// Simple weather page — large symbol + temperature
void drawPageWeatherSimple() {
  char buf[32];

  // Header — cloud icon + location name + timestamp top right
  display.drawBitmap(2, 1, ICON_CLOUD, 8, 8, SSD1306_WHITE);
  const char* title = (weather.valid && strlen(weather.locationName) > 0) ? weather.locationName : "Weather";
  display.setCursor(13, 1); display.println(title);
  // Last updated time — top right
  if (wifiLastFetchHour >= 0) {
    char timeBuf[6];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", wifiLastFetchHour, wifiLastFetchMin);
    int tw = strlen(timeBuf) * 6;
    display.setCursor(127 - tw, 1); display.println(timeBuf);
  }
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (!weather.valid && !moonData.valid) {
    display.setCursor(2, 22); display.println("No weather data");
    display.setCursor(2, 34); display.println(wifiEnabled ? wifiSSID : "No WiFi set");
    return;
  }

  float temp = setElevFt ? weather.tempC * 9/5 + 32 : weather.tempC;
  const char* unit = setElevFt ? "F" : "C";

  // Large weather icon — 24x24 bitmap, left side
  const uint8_t* wxIcon = ICON_WX_CLOUD;  // default
  if (weather.valid) {
    int id = weather.weatherId;
    if      (id >= 200 && id < 300) wxIcon = ICON_WX_STORM;
    else if (id >= 300 && id < 400) wxIcon = ICON_WX_DRIZZLE;
    else if (id >= 500 && id < 600) wxIcon = ICON_WX_RAIN;
    else if (id >= 600 && id < 700) wxIcon = ICON_WX_SNOW;
    else if (id >= 700 && id < 800) wxIcon = ICON_WX_MIST;
    else if (id == 800)             wxIcon = ICON_WX_CLEAR;
    else if (id <= 802)             wxIcon = ICON_WX_FEW;
    else                            wxIcon = ICON_WX_CLOUD;
  }
  display.drawBitmap(2, 17, wxIcon, 24, 24, SSD1306_WHITE);

  // Temperature — large font, vertically centred with icon
  display.setFont(&FreeSansBold12pt7b);
  snprintf(buf, sizeof(buf), "%.0f%s", temp, unit);
  display.setCursor(34, 36);
  display.println(buf);
  display.setFont(nullptr);  // reset font

  // Condition text
  if (weather.valid) {
    display.setCursor(2, 43); display.println(weather.desc);
  }
}


void drawPageWifi() {
  char buf[48];

  // Header
  display.drawBitmap(2, 1, ICON_CLOUD, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1); display.println("Sky & Sun");
  if (magDecValid) {
    snprintf(buf, sizeof(buf), "%+.1fd", magDeclination);
    int w = strlen(buf) * 6;
    display.setCursor(127 - w, 1); display.println(buf);
  }
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (weather.valid) {
    float temp  = setElevFt ? weather.tempC * 9/5 + 32 : weather.tempC;
    float feels = setElevFt ? weather.feelsLikeC * 9/5 + 32 : weather.feelsLikeC;
    const char* u = setElevFt ? "F" : "C";
    // Row 1 — temp + feels like
    snprintf(buf, sizeof(buf), "%.0f%s (feels %.0f)", temp, u, feels);
    display.setCursor(2, 13); display.println(buf);
    // Row 2 — wind
    float wind = setSpeedMph ? weather.windKmh * 0.621371f : weather.windKmh;
    float gust = setSpeedMph ? weather.windGustKmh * 0.621371f : weather.windGustKmh;
    const char* wu = setSpeedMph ? "mph" : "km/h";
    snprintf(buf, sizeof(buf), "%s %.0f%s gst%.0f H:%d%%",
      weather.windDir, wind, wu, gust, weather.humidity);
    display.setCursor(2, 23); display.println(buf);
  }

  display.drawLine(0, 33, 127, 33, SSD1306_WHITE);

  if (sunData.valid) {
    // Row 3 — Sunrise/sunset
    snprintf(buf, sizeof(buf), "Rise %s  Set %s", sunData.sunrise, sunData.sunset);
    display.setCursor(2, 36); display.println(buf);
    // Row 4 — Golden hour
    snprintf(buf, sizeof(buf), "Gold %s/%s", sunData.golden, sunData.goldenEve);
    display.setCursor(2, 46); display.println(buf);
    // Row 5 — Day length + moon (abbreviated)
    char moonBuf[16] = "";
    if (moonData.valid) {
      // Abbreviate phase name to fit
      const char* ph = moonData.phaseName;
      if      (strstr(ph, "New"))      snprintf(moonBuf, sizeof(moonBuf), " New");
      else if (strstr(ph, "Waxing C")) snprintf(moonBuf, sizeof(moonBuf), " WxC%.0f%%", moonData.illumination);
      else if (strstr(ph, "First"))    snprintf(moonBuf, sizeof(moonBuf), " 1Q%.0f%%",  moonData.illumination);
      else if (strstr(ph, "Waxing G")) snprintf(moonBuf, sizeof(moonBuf), " WxG%.0f%%", moonData.illumination);
      else if (strstr(ph, "Full"))     snprintf(moonBuf, sizeof(moonBuf), " Full");
      else if (strstr(ph, "Waning G")) snprintf(moonBuf, sizeof(moonBuf), " WnG%.0f%%", moonData.illumination);
      else if (strstr(ph, "Last"))     snprintf(moonBuf, sizeof(moonBuf), " 3Q%.0f%%",  moonData.illumination);
      else if (strstr(ph, "Waning C")) snprintf(moonBuf, sizeof(moonBuf), " WnC%.0f%%", moonData.illumination);
    }
    snprintf(buf, sizeof(buf), "Day %s%s", sunData.dayLength, moonBuf);
    display.setCursor(2, 56); display.println(buf);
  } else if (moonData.valid) {
    snprintf(buf, sizeof(buf), "%s %.0f%%", moonData.phaseName, moonData.illumination);
    display.setCursor(2, 56); display.println(buf);
  }
}

// ── QR Code Generator (embedded, no external library) ─────────────────────────
// Minimal QR code generator for WiFi credentials display
// Based on Project Nayuki's QR code generator, simplified for embedded use

#define QR_SIZE 29  // Version 3 QR code is 29x29 modules

static uint8_t qrModules[QR_SIZE][QR_SIZE];
static uint8_t qrFuncMask[QR_SIZE][QR_SIZE];

static void qrSetModule(int x, int y, bool dark, bool isFunc) {
  if (x >= 0 && x < QR_SIZE && y >= 0 && y < QR_SIZE) {
    qrModules[y][x] = dark ? 1 : 0;
    if (isFunc) qrFuncMask[y][x] = 1;
  }
}

static void qrDrawFinderPattern(int cx, int cy) {
  for (int dy = -4; dy <= 4; dy++) {
    for (int dx = -4; dx <= 4; dx++) {
      int dist = max(abs(dx), abs(dy));
      if (dist <= 3) {
        bool dark = (dist != 2) && (dist != 4);
        qrSetModule(cx + dx, cy + dy, dark, true);
      }
    }
  }
}

static void qrDrawAlignmentPattern(int cx, int cy) {
  for (int dy = -2; dy <= 2; dy++) {
    for (int dx = -2; dx <= 2; dx++) {
      bool dark = (abs(dx) == 2 || abs(dy) == 2 || (dx == 0 && dy == 0));
      qrSetModule(cx + dx, cy + dy, dark, true);
    }
  }
}

static void qrDrawFormatBits(uint32_t bits) {
  // Format info around finders
  for (int i = 0; i < 6; i++) qrSetModule(8, i, (bits >> i) & 1, true);
  qrSetModule(8, 7, (bits >> 6) & 1, true);
  qrSetModule(8, 8, (bits >> 7) & 1, true);
  qrSetModule(7, 8, (bits >> 8) & 1, true);
  for (int i = 9; i < 15; i++) qrSetModule(14 - i, 8, (bits >> i) & 1, true);
  
  for (int i = 0; i < 8; i++) qrSetModule(QR_SIZE - 1 - i, 8, (bits >> i) & 1, true);
  for (int i = 8; i < 15; i++) qrSetModule(8, QR_SIZE - 15 + i, (bits >> i) & 1, true);
  qrSetModule(8, QR_SIZE - 8, 1, true);  // Always dark
}

static void qrDrawFunctionPatterns() {
  memset(qrModules, 0, sizeof(qrModules));
  memset(qrFuncMask, 0, sizeof(qrFuncMask));
  
  // Finder patterns
  qrDrawFinderPattern(3, 3);
  qrDrawFinderPattern(QR_SIZE - 4, 3);
  qrDrawFinderPattern(3, QR_SIZE - 4);
  
  // Alignment pattern (version 3 has one at 22,22)
  qrDrawAlignmentPattern(22, 22);
  
  // Timing patterns
  for (int i = 0; i < QR_SIZE; i++) {
    qrSetModule(6, i, (i % 2) == 0, true);
    qrSetModule(i, 6, (i % 2) == 0, true);
  }
  
  // Format bits placeholder (mask 0, ECC L = 0x77C4)
  qrDrawFormatBits(0x77C4);
}

static uint8_t qrDataBits[128];
static int qrDataLen = 0;

static void qrAddBits(uint32_t val, int len) {
  for (int i = len - 1; i >= 0; i--) {
    int byteIdx = qrDataLen / 8;
    int bitIdx = 7 - (qrDataLen % 8);
    if (byteIdx < 128) {
      if ((val >> i) & 1) qrDataBits[byteIdx] |= (1 << bitIdx);
    }
    qrDataLen++;
  }
}

static void qrEncodeData(const char* text) {
  memset(qrDataBits, 0, sizeof(qrDataBits));
  qrDataLen = 0;
  
  int len = strlen(text);
  // Byte mode indicator
  qrAddBits(0b0100, 4);
  // Character count (8 bits for version 3 byte mode)
  qrAddBits(len, 8);
  // Data
  for (int i = 0; i < len; i++) {
    qrAddBits((uint8_t)text[i], 8);
  }
  // Terminator
  qrAddBits(0, min(4, 440 - qrDataLen));  // Version 3-L has 440 data bits
  // Pad to byte boundary
  while (qrDataLen % 8 != 0) qrAddBits(0, 1);
  // Pad codewords
  uint8_t padBytes[] = {0xEC, 0x11};
  int padIdx = 0;
  while (qrDataLen < 440) {
    qrAddBits(padBytes[padIdx], 8);
    padIdx = 1 - padIdx;
  }
}

// Reed-Solomon error correction (simplified for Version 3-L: 15 EC codewords)
static uint8_t qrGfMul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0) return 0;
  // GF(256) multiplication using log/exp tables
  static const uint8_t exp[512] = {
    1,2,4,8,16,32,64,128,29,58,116,232,205,135,19,38,76,152,45,90,180,117,234,201,143,3,6,12,24,48,96,192,
    157,39,78,156,37,74,148,53,106,212,181,119,238,193,159,35,70,140,5,10,20,40,80,160,93,186,105,210,185,111,
    222,161,95,190,97,194,153,47,94,188,101,202,137,15,30,60,120,240,253,231,211,187,107,214,177,127,254,225,
    223,163,91,182,113,226,217,175,67,134,17,34,68,136,13,26,52,104,208,189,103,206,129,31,62,124,248,237,199,
    147,59,118,236,197,151,51,102,204,133,23,46,92,184,109,218,169,79,158,33,66,132,21,42,84,168,77,154,41,82,
    164,85,170,73,146,57,114,228,213,183,115,230,209,191,99,198,145,63,126,252,229,215,179,123,246,241,255,227,
    219,171,75,150,49,98,196,149,55,110,220,165,87,174,65,130,25,50,100,200,141,7,14,28,56,112,224,221,167,83,
    166,81,162,89,178,121,242,249,239,195,155,43,86,172,69,138,9,18,36,72,144,61,122,244,245,247,243,251,235,
    203,139,11,22,44,88,176,125,250,233,207,131,27,54,108,216,173,71,142,1
  };
  static const uint8_t log[256] = {
    0,0,1,25,2,50,26,198,3,223,51,238,27,104,199,75,4,100,224,14,52,141,239,129,28,193,105,248,200,8,76,113,
    5,138,101,47,225,36,15,33,53,147,142,218,240,18,130,69,29,181,194,125,106,39,249,185,201,154,9,120,77,228,
    114,166,6,191,139,98,102,221,48,253,226,152,37,179,16,145,34,136,54,208,148,206,143,150,219,189,241,210,19,
    92,131,56,70,64,30,66,182,163,195,72,126,110,107,58,40,84,250,133,186,61,202,94,155,159,10,21,121,43,78,212,
    229,172,115,243,167,87,7,112,192,247,140,128,99,13,103,74,222,237,49,197,254,24,227,165,153,119,38,184,180,
    124,17,68,146,217,35,32,137,46,55,63,209,91,149,188,207,205,144,135,151,178,220,252,190,97,242,86,211,171,
    20,42,93,158,132,60,57,83,71,109,65,162,31,45,67,216,183,123,164,118,196,23,73,236,127,12,111,246,108,161,
    59,82,41,157,85,170,251,96,134,177,187,204,62,90,203,89,95,176,156,169,160,81,11,245,22,235,122,117,44,215,
    79,174,213,233,230,231,173,232,116,214,244,234,168,80,88,175
  };
  return exp[log[a] + log[b]];
}

static void qrAddECC() {
  // Version 3-L: 55 data codewords, 15 EC codewords, 1 block
  uint8_t data[55];
  for (int i = 0; i < 55; i++) data[i] = qrDataBits[i];
  
  // Generator polynomial for 15 EC codewords (precomputed)
  static const uint8_t gen[15] = {8,183,61,91,202,37,51,58,58,237,140,124,5,99,105};
  
  uint8_t ecc[15] = {0};
  for (int i = 0; i < 55; i++) {
    uint8_t factor = data[i] ^ ecc[0];
    memmove(ecc, ecc + 1, 14);
    ecc[14] = 0;
    for (int j = 0; j < 15; j++) {
      ecc[j] ^= qrGfMul(gen[j], factor);
    }
  }
  
  // Append ECC to data
  for (int i = 0; i < 15; i++) qrDataBits[55 + i] = ecc[i];
}

static void qrPlaceData() {
  int bitIdx = 0;
  int totalBits = 70 * 8;  // 55 data + 15 ECC = 70 codewords
  
  // Place bits in zigzag pattern
  for (int right = QR_SIZE - 1; right >= 1; right -= 2) {
    if (right == 6) right = 5;  // Skip timing column
    for (int vert = 0; vert < QR_SIZE; vert++) {
      for (int j = 0; j < 2; j++) {
        int x = right - j;
        bool upward = ((right + 1) / 2) % 2 == 0;
        int y = upward ? (QR_SIZE - 1 - vert) : vert;
        if (!qrFuncMask[y][x] && bitIdx < totalBits) {
          int byteIdx = bitIdx / 8;
          int bitPos = 7 - (bitIdx % 8);
          bool dark = (qrDataBits[byteIdx] >> bitPos) & 1;
          // Apply mask pattern 0: (x + y) % 2 == 0
          if ((x + y) % 2 == 0) dark = !dark;
          qrModules[y][x] = dark ? 1 : 0;
          bitIdx++;
        }
      }
    }
  }
}

static void qrGenerate(const char* text) {
  qrDrawFunctionPatterns();
  qrEncodeData(text);
  qrAddECC();
  qrPlaceData();
}

// Shows WiFi QR code that phones can scan to auto-join the device AP
void drawQRCode() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Build WiFi credential string: WIFI:T:WPA;S:ssid;P:password;;
  char wifiStr[80];
  snprintf(wifiStr, sizeof(wifiStr), "WIFI:T:WPA;S:%s;P:%s;;", apSSID, apPass);
  
  // Generate QR code
  qrGenerate(wifiStr);
  
  // Draw QR code — 2px per module = 58x58, centered
  const int scale = 2;
  const int qrX = (128 - QR_SIZE * scale) / 2;
  const int qrY = 2;
  
  for (int y = 0; y < QR_SIZE; y++) {
    for (int x = 0; x < QR_SIZE; x++) {
      if (qrModules[y][x]) {
        display.fillRect(qrX + x * scale, qrY + y * scale, scale, scale, SSD1306_WHITE);
      }
    }
  }
  
  // Hint text at bottom
  display.setCursor(20, 56);
  display.print("Scan to connect");
  
  display.display();
}


void drawPageSettings() {
  // Show brief saved confirmation
  if (settingsSaved) {
    display.clearDisplay();  // ensure clean slate
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.drawBitmap(2, 1, ICON_COG, 8, 8, SSD1306_WHITE);
    display.setCursor(13, 1); display.println("Settings");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    // Circle with tick — centred at cx=64, cy=36, r=12
    const int cx = 64, cy = 36, r = 12;
    display.drawCircle(cx, cy, r, SSD1306_WHITE);
    // Tick: left leg from (58,36) to (62,41), right leg from (62,41) to (70,29)
    display.drawLine(cx-6, cy,   cx-2, cy+5, SSD1306_WHITE);
    display.drawLine(cx-2, cy+5, cx+6, cy-7, SSD1306_WHITE);
    // "Saved!" text below circle
    display.setCursor(40, 54); display.println("Saved!");
    return;
  }
  
  // Settings: 7 rows total with LoRa, 3 visible per page
  // Page 0: rows 0-2, Page 1: rows 3-5, Page 2: row 6
  const int ROWS_PER_PAGE = 3;
#if ENABLE_LORA
  const int TOTAL_ROWS = 7;
#else
  const int TOTAL_ROWS = 6;
#endif
  int settingsPage = settingsRow / ROWS_PER_PAGE;
  int startRow = settingsPage * ROWS_PER_PAGE;
  int endRow = min(startRow + ROWS_PER_PAGE, TOTAL_ROWS);
  
  // Header — shows current edit row name when in edit mode
  display.drawBitmap(2, 1, ICON_COG, 8, 8, SSD1306_WHITE);
  if (settingsSel) {
#if ENABLE_LORA
    const char* rowNames[7] = {"Speed", "Elevation", "Timeout", "Theme", "Hotspot", "Relay", "Lock"};
#else
    const char* rowNames[6] = {"Speed", "Elevation", "Timeout", "Theme", "Hotspot", "Lock"};
#endif
    char hdr[20];
    snprintf(hdr, sizeof(hdr), "Set: %s", rowNames[settingsRow]);
    display.setCursor(13, 1); display.println(hdr);
  } else {
    display.setCursor(13, 1); display.println("Settings");
  }
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Row labels and values
#if ENABLE_LORA
  const char* rows[7] = {"Speed", "Elevation", "Timeout", "Theme", "Hotspot", "Relay", "Lock"};
  char vals[7][10];
#else
  const char* rows[6] = {"Speed", "Elevation", "Timeout", "Theme", "Hotspot", "Lock"};
  char vals[6][10];
#endif
  snprintf(vals[0], 10, "%s", setSpeedMph   ? "mph"   : "km/h");
  snprintf(vals[1], 10, "%s", setElevFt     ? "ft"    : "m");
  uint16_t ts = TIMEOUT_PRESETS[setTimeoutIdx];
  if (ts == 0) snprintf(vals[2], 10, "Never");
  else         snprintf(vals[2], 10, "%ds", ts);
  snprintf(vals[3], 10, "%s", setBrightFull ? "Dark"  : "Light");
  snprintf(vals[4], 10, "%s", apEnabled     ? "On"    : "Off");
#if ENABLE_LORA
  snprintf(vals[5], 10, "%s", relayEnabled  ? "On"    : "Off");
  snprintf(vals[6], 10, "%s", deviceLocked  ? "On"    : "Off");
#else
  snprintf(vals[5], 10, "%s", deviceLocked  ? "On"    : "Off");
#endif

  for (int i = startRow; i < endRow; i++) {
    int y = 13 + (i - startRow) * 12;
    bool active = (i == settingsRow);

    if (active && settingsSel) {
      // Invert active row while editing
      display.fillRect(0, y-1, 127, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }

    // Label
    display.setCursor(6, y);
    display.print(rows[i]);

    // Right-align value
    int vw = strlen(vals[i]) * 6;
    display.setCursor(127 - vw, y);
    display.print(vals[i]);

    display.setTextColor(SSD1306_WHITE);

    // Row cursor indicator — small filled rect on left edge
    if (active) {
      if (settingsSel) {
        // Left arrow while editing (inverted so draws black on white)
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(2, y); display.print("*");
        display.setTextColor(SSD1306_WHITE);
      } else {
        display.fillRect(0, y, 3, 7, SSD1306_WHITE);
      }
    }
  }
  
  // Scroll indicators
  if (settingsPage == 0 && TOTAL_ROWS > ROWS_PER_PAGE) {
    // Down arrow at bottom right to show more items below
    int ax = 120, ay = 57;
    display.drawLine(ax, ay, ax-4, ay-4, SSD1306_WHITE);
    display.drawLine(ax, ay, ax+4, ay-4, SSD1306_WHITE);
    display.drawLine(ax, ay-1, ax-3, ay-4, SSD1306_WHITE);
    display.drawLine(ax, ay-1, ax+3, ay-4, SSD1306_WHITE);
  } else if (settingsPage > 0) {
    // Up arrow at top right to show items above
    int ax = 120, ay = 14;
    display.drawLine(ax, ay, ax-4, ay+4, SSD1306_WHITE);
    display.drawLine(ax, ay, ax+4, ay+4, SSD1306_WHITE);
    display.drawLine(ax, ay+1, ax-3, ay+4, SSD1306_WHITE);
    display.drawLine(ax, ay+1, ax+3, ay+4, SSD1306_WHITE);
  }
}

void drawPagePower() {
  if (sosActive) {
    // ── SOS active screen ──
    display.clearDisplay();
    // Flashing header
    display.setCursor(2, 1); display.println("** SOS ACTIVE **");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    display.setCursor(2, 14);
    display.println("Broadcasting SOS.");
    display.setCursor(2, 24);
    uint8_t remaining = SOS_REPEAT_COUNT - sosBroadcastCount;
    char buf[24];
    snprintf(buf, sizeof(buf), "%d tx remaining.", remaining);
    display.println(buf);
    display.setCursor(2, 36);
    display.println("Press to cancel.");
    display.drawLine(0, 54, 127, 54, SSD1306_WHITE);
    display.display();
    return;
  }
  // Header
  display.drawBitmap(2, 1, ICON_PWRBTN, 8, 8, SSD1306_WHITE);
  display.setCursor(13, 1); display.println("Power off");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Power symbol centred on left
  const int cx = 21, cy = 32, r = 14;
  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  display.fillRect(cx-2, cy-r-1, 5, 5, SSD1306_BLACK);
  display.drawLine(cx, cy-r-1, cx, cy-3, SSD1306_WHITE);

  // Right side — power off + SOS hint
  display.setCursor(45, 14); display.println("Hold 3s:");
  display.setCursor(45, 22); display.println("power off");
  display.setCursor(45, 34); display.println("Double tap:");
  display.setCursor(45, 43); display.println("Send SOS");

  display.drawLine(0, 54, 127, 54, SSD1306_WHITE);
}

#if ENABLE_LORA
void drawPageMessages() {
  // Header
  display.setCursor(2, 1); display.print("Messages");
  if (loraMsgUnread > 0) {
    display.print(" (");
    display.print(loraMsgUnread);
    display.print(" new)");
  }
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (loraMsgCount == 0) {
    display.setCursor(10, 30); display.println("No messages");
    return;
  }

  // Show messages with scroll support
  // Available height: y=13 to y=OLED_HINT_Y (46) = 33px, fits 2 messages × ~16px each
  // Each message: name row (8px) + text row (8px) = 16px
  const int maxVisible = 2;
  int shown = 0;
  int y = 13;
  
  // Calculate starting message index based on scroll
  int startIdx = loraMsgCount - 1 - loraMsgScroll;
  
  for (int m = startIdx; m >= 0 && shown < maxVisible && y < OLED_HINT_Y; m--) {
    int slot = m % LORA_MAX_MSGS;
    if (!loraMsgs[slot].active) continue;
    if (!loraMsgs[slot].read) {
      loraMsgs[slot].read = true;
      if (loraMsgUnread > 0) loraMsgUnread--;
    }
    // From name — max 10 chars
    char fromShort[11] = {0};
    const char* src = loraMsgs[slot].fromName[0] ? loraMsgs[slot].fromName : loraMsgs[slot].fromId;
    strncpy(fromShort, src, 10);
    display.setCursor(0, y);
    display.print(fromShort);
    display.print(":");
    y += 8;
    // Message text — truncate to 21 chars
    char line[22] = {0};
    strncpy(line, loraMsgs[slot].text, 21);
    display.setCursor(4, y);
    display.println(line);
    y += 9;
    shown++;
  }
  
  // Show scroll indicators and count in hint area
  display.drawLine(0, OLED_HINT_Y, 127, OLED_HINT_Y, SSD1306_WHITE);
  char hint[22];
  int current = loraMsgScroll + 1;
  int total = loraMsgCount;
  
  // Up arrow if can scroll up (to newer)
  if (loraMsgScroll > 0) {
    display.setCursor(2, OLED_HINT_Y + 2);
    display.print("\x18");  // up arrow
  }
  // Down arrow if can scroll down (to older)
  if (loraMsgScroll + maxVisible < loraMsgCount) {
    display.setCursor(120, OLED_HINT_Y + 2);
    display.print("\x19");  // down arrow
  }
  // Center: message position
  snprintf(hint, sizeof(hint), "%d/%d", current, total);
  int hintW = strlen(hint) * 6;
  display.setCursor((128 - hintW) / 2, OLED_HINT_Y + 2);
  display.print(hint);
}
#endif

// ============================================================
// OLED DRAW DISPATCHER
// ============================================================
void wakeScreen() {
  if (!screenOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    screenOn = true;
  }
  lastActivityMs = millis();
}

// Draw page content only (no display.display() call) — used by transitions
void oledDrawPage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Pairing screen takes full control
  if (pairingActive) { drawPairingScreen(); return; }
  // SOS active — always show SOS screen
  if (sosActive) { drawPagePower(); return; }

  if      (currentPage==0)                    drawPageGPS();
#if ENABLE_LORA
  else if (loraMsgCount>0 && currentPage==pageMessages())  drawPageMessages();
  else if (currentPage==pageGroups())         drawPageGroups();
#endif
  else if (currentPage==pageCompass())        drawPageCompass();
  else if (currentPage==pageTrack())          drawPageRoute();
  else if (currentPage==pageWaypoint()) {
    if (waypointCount > 0 || locSaved)   drawPageNavigate();
    else                                  drawPageSaveLoc();
  }
  else if (currentPage==pageElevation())      drawPageElevation();
  else if (routeLogCount>0 && currentPage==pageRouteLog()) drawPageRouteLog();
  else if (weather.valid && currentPage==pageWeather()) {
    if (weatherDetailActive) drawPageWifi();
    else                     drawPageWeatherSimple();
  }
  else if (currentPage==pageClock())          drawPageClock();
  else if (currentPage==pageCountdown())      drawPageCountdown();
  else if (currentPage==pageStopwatch())      drawPageStopwatch();
  else if (currentPage==pageInfo())           drawPageInfo();
  else if (currentPage==pageSettings())       drawPageSettings();
  else if (currentPage==pagePower())          drawPagePower();
  else                                        drawPageGPS();

  // Nav dots — suppressed on compass and settings saved
  bool isCompass = (currentPage == pageCompass());
  if (!isCompass && !settingsSaved && !weatherDetailActive)
    drawNavDots();
}

void oledDraw() {
  if (!screenOn) return;
  if (qrCodeActive) return;  // Don't overwrite QR code display
  
#if ENABLE_LORA
  // Pending key share request — jump to Groups page automatically
  if (pendingKeyShare.active && currentPage != pageGroups()) {
    currentPage = pageGroups();
  }
#endif

  oledDrawPage();
  display.display();
}

// ============================================================
// BUTTON HANDLER
// ============================================================
void handleButton() {
  bool state = digitalRead(BTN_PIN);
  uint32_t now = millis();

  if (state==LOW && btnLastState==HIGH) {
    btnPressedMs   = now;
    btnLongFired   = false;
    lastActivityMs = now;  // reset timeout on every press
  }

  if (state==LOW && btnLastState==LOW && !btnLongFired) {
    uint32_t held = now - btnPressedMs;
    uint32_t threshold = (currentPage==pagePower()) ? BTN_POWER_MS : BTN_LONG_MS;

#if ENABLE_LORA
    // Groups page long press handling
    if (currentPage == pageGroups() && held >= BTN_LONG_MS) {
      btnLongFired = true;
      wakeScreen();
      
      if (pendingKeyShare.active) {
        // Accept pending key share
        keyShareAccept();
      } else if (keyShareActive) {
        // Done sharing
        keyShareActive = false;
      } else if (groupMenuOpen) {
        // Execute selected menu action
        bool isPersonal = (storedGroupSel == 0);
        bool isActive = isPersonal ? (activeStoredGroup == -1) : (activeStoredGroup == storedGroupSel - 1);
        bool isLeader = !isPersonal && storedGroups[storedGroupSel - 1].isLeader;
        bool canLeave = isPersonal || storedGroups[storedGroupSel - 1].canLeave;
        
        // Determine which action based on menu position
        int actionIdx = 0;
        const char* actions[4] = {NULL, NULL, NULL, NULL};
        actions[actionIdx++] = isActive ? "deactivate" : "activate";
        if (!isPersonal && isLeader) actions[actionIdx++] = "share";
        if (!isPersonal && canLeave) actions[actionIdx++] = "leave";
        actions[actionIdx++] = "back";
        
        const char* action = actions[groupMenuSel];
        
        if (strcmp(action, "activate") == 0) {
          if (isPersonal) {
            // Switch to personal (default key)
            activeStoredGroup = -1;
            memcpy(loraKey, LORA_DEFAULT_KEY, LORA_KEY_LEN);
            loraSaveKey(loraKey);
            storedGroupsSave();
          } else {
            storedGroupActivate(storedGroupSel - 1);
          }
          groupMenuOpen = false;
        } else if (strcmp(action, "deactivate") == 0) {
          // Switch to personal - also restore original name if saved
          activeStoredGroup = -1;
          memcpy(loraKey, LORA_DEFAULT_KEY, LORA_KEY_LEN);
          loraSaveKey(loraKey);
          // Restore original device name if we have one saved
          if (strlen(originalDeviceName) > 0) {
            strncpy(bleDeviceName, originalDeviceName, sizeof(bleDeviceName) - 1);
            bleDeviceName[sizeof(bleDeviceName) - 1] = 0;
            prefs.begin("azimuth", false);
            prefs.putString("devName", bleDeviceName);
            prefs.remove("origName");
            prefs.end();
            originalDeviceName[0] = 0;
            Serial.printf("[GROUPS] Deactivate: Restored original name: %s\n", bleDeviceName);
          }
          storedGroupsSave();
          // Notify BLE web app
          if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
            char json[32];
            snprintf(json, sizeof(json), "{\"groupsChanged\":1}");
            pPeerChar->setValue((uint8_t*)json, strlen(json));
            pPeerChar->notify();
          }
          groupMenuOpen = false;
        } else if (strcmp(action, "share") == 0) {
          strncpy(keyShareGroupName, storedGroups[storedGroupSel - 1].name, 31);
          loraKeyShare();
          groupMenuOpen = false;
        } else if (strcmp(action, "leave") == 0) {
          storedGroupDelete(storedGroupSel - 1);
          storedGroupSel = 0; // Back to Personal
          groupMenuOpen = false;
        } else if (strcmp(action, "back") == 0) {
          groupMenuOpen = false;
        }
        groupMenuSel = 0;
      } else {
        // Open sub-menu for selected item
        groupMenuOpen = true;
        groupMenuSel = 0;
      }
      
      oledDraw();
      btnLastState = state;
      return;
    }
#endif

    if (held >= threshold) {
      btnLongFired = true;
      wakeScreen();

      // ── Long press actions ──────────────────────────────
      if (!screenOn) {
        // Already woken above
      } else if (currentPage == 0) {
        // GPS / Acquiring — show PIN overlay while held, revert on release
        pinOverlayActive = true;
        drawPairingScreen();
        // Block here until button released — same pattern as drawAboutScreen
        while (digitalRead(BTN_PIN) == LOW) delay(10);
        pinOverlayActive = false;
        // Always revert to page 0 — if we were acquiring and now have a fix
        // the next oledDraw() will show the position page automatically
        currentPage = 0;
        oledDraw();
#if ENABLE_LORA
      } else if (loraMsgCount > 0 && currentPage == pageMessages()) {
        // Long press — scroll to older messages
        const int maxVisible = 2;
        if (loraMsgScroll + maxVisible < loraMsgCount) {
          loraMsgScroll++;
          oledDraw();
        }
#endif
      } else if (weather.valid && currentPage==pageWeather()) {
        weatherDetailActive = true;
        oledDraw();
      } else if (currentPage==pageCompass()) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        screenOn = false;
      } else if (currentPage==pageTrack()) {
        clearRoute();
        oledDraw();
      } else if (routeLogCount>0 && currentPage==pageRouteLog()) {
        if (routeRecording) {
          logClearWarn = true;
        } else if (logClearWarn) {
          routeLogClear();
        } else {
          routeLogClear();
        }
        oledDraw();
      } else if (currentPage==pageInfo()) {
        drawAboutScreen();
        while (digitalRead(BTN_PIN)==HIGH) delay(10);
        while (digitalRead(BTN_PIN)==LOW)  delay(10);
        oledDraw();
      } else if (currentPage==pageCountdown()) {
        if (cdtDone) {
          // Reset to last set time
          cdtDone    = false;
          cdtRunning = false;
          cdtRemMs   = cdtSetSecs * 1000UL;
        } else if (cdtSetMode) {
          // Exit set mode
          cdtSetMode = false;
          cdtRemMs   = cdtSetSecs * 1000UL;
        } else if (!cdtRunning) {
          // Enter set mode
          cdtSetMode = true;
        }
        oledDraw();
      } else if (currentPage==pageElevation()) {
        // Toggle bar/line graph — saved to NVS
        elevLineMode = !elevLineMode;
        saveElevMode();
        oledDraw();
      } else if (currentPage==pageStopwatch()) {
        // Long press — clear stopwatch
        swClear(); oledDraw();
      } else if (currentPage==pageClock()) {
        if (tzAdjustMode) { saveTimezone(); tzAdjustMode=false; }
        else              { tzAdjustMode=true; }
        oledDraw();
      } else if (locSaved && currentPage==pageSaveLoc()) {
        if (waypointCount > 0) {
          waypointCurrent = (waypointCurrent + 1) % waypointCount;
        } else {
          locSaved=false; savedLat=0; savedLon=0;
          prefs.begin("azimuth", false);
          prefs.putBool("pinSaved", false);
          prefs.end();
        }
        oledDraw();
      } else if (currentPage==pageSaveLoc()) {
        if (waypointCount > 0) {
          waypointCurrent = (waypointCurrent + 1) % waypointCount;
        } else if (cachedFix) {
          savedLat=cachedLat; savedLon=cachedLon; locSaved=true;
          prefs.begin("azimuth", false);
          prefs.putDouble("pinLat", savedLat);
          prefs.putDouble("pinLon", savedLon);
          prefs.putBool("pinSaved", true);
          prefs.end();
          display.clearDisplay();
          display.drawBitmap(56, 18, ICON_PIN, 8, 8, SSD1306_WHITE);
          display.setCursor(8, 36); display.println("Waypoint marked!");
          display.display(); delay(1200);
        }
        oledDraw();
      } else if (currentPage==pageSettings()) {
        if (!settingsSel) {
          // Enter edit mode on current row
          settingsSel    = true;
          settingsIdleMs = millis();
        } else {
          // Confirm row — advance to next, or save+exit on last row
#if ENABLE_LORA
          const int lastRow = 5;
#else
          const int lastRow = 4;
#endif
          if (settingsRow < lastRow) {
            settingsRow++;
            settingsIdleMs = millis();
          } else {
            // Last row — save and exit
            settingsSel = false;
            settingsRow = 0;
            saveSettings();
            settingsSaved   = true;
            settingsSavedMs = millis();
          }
        }
        oledDraw();
      } else if (currentPage==pagePower()) {
        display.clearDisplay();
        display.setCursor(20,28); display.println("Powered off");
        display.display(); delay(800);
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        esp_deep_sleep_start();
      }
    }
  }

  if (state==HIGH && btnLastState==LOW) {
    // Clear weather detail overlay on release
    if (weatherDetailActive) {
      weatherDetailActive = false;
      oledDraw();
    }
    // Dismiss QR code on any button press
    if (qrCodeActive) {
      qrCodeActive = false;
      oledDraw();
      btnLastState = state;
      return;
    }
    // ── Short press release ──────────────────────────────
    if (!btnLongFired) {
      bool wasScreenOff = !screenOn;
      wakeScreen();
      
#if ENABLE_LORA
      // ── Groups page press handling (with double press) ──────
      if (currentPage == pageGroups()) {
        // If screen was off, just wake it - don't process press
        if (wasScreenOff) {
          Serial.println("[BTN] Screen was off, just waking");
          btnPendingSingle = false;
          oledDraw();
          btnLastState = state;
          return;
        }
        
        Serial.printf("[BTN] Groups release: pending=%d, elapsed=%lu, btnPendingMs=%lu, now=%lu\n", 
                      btnPendingSingle, (now - btnPendingMs), btnPendingMs, now);
        if (btnPendingSingle && (now - btnPendingMs <= BTN_DOUBLE_MS)) {
          // Double press confirmed
          Serial.println("[BTN] Double press confirmed!");
          btnPendingSingle = false;
          if (pendingKeyShare.active) {
            // Decline key share
            keyShareDecline();
          } else if (keyShareActive) {
            // Cancel key share broadcast
            loraKeyShareCancel();
          } else if (groupMenuOpen) {
            // Close menu without action
            groupMenuOpen = false;
            groupMenuSel = 0;
          } else {
            // Cycle through list (Personal + stored groups)
            int totalItems = 1 + storedGroupCount;
            storedGroupSel = (storedGroupSel + 1) % totalItems;
            Serial.printf("[BTN] Cycled to %d of %d\n", storedGroupSel, totalItems);
          }
          oledDraw();
          btnLastState = state;
          return;
        } else {
          // First press — wait for possible double
          Serial.printf("[BTN] First press, setting pending=1, btnPendingMs=%lu\n", now);
          btnPendingSingle = true;
          btnPendingMs = now;
          lastActivityMs = now;
          btnLastState = state;
          return;
        }
      }
#endif
      
      if (wasScreenOff) {
        // Screen was off — just wake, stay on current page
        btnPendingSingle = false;
        oledDraw();
      } else if (currentPage==pageSettings() && settingsSel) {
        // Short press = cycle value for active row
        settingsIdleMs = millis();
        switch (settingsRow) {
          case 0: setSpeedMph   = !setSpeedMph;   break;
          case 1: setElevFt     = !setElevFt;     break;
          case 2: setTimeoutIdx = (setTimeoutIdx+1) % TIMEOUT_PRESET_COUNT; break;
          case 3: setBrightFull = !setBrightFull;
                  display.invertDisplay(!setBrightFull); break;
          case 4: // Hotspot — instant toggle
                  if (apEnabled) {
                    apStop();
                  } else {
                    apStart();
                  }
                  break;
#if ENABLE_LORA
          case 5: // Relay mode toggle
                  relayEnabled = !relayEnabled;
                  Serial.printf("[RELAY] %s\n", relayEnabled ? "Enabled" : "Disabled");
                  break;
          case 6: // Lock mode toggle
                  deviceLocked = !deviceLocked;
                  Serial.printf("[LOCK] %s\n", deviceLocked ? "Locked" : "Unlocked");
                  break;
#else
          case 5: // Lock mode toggle (non-LoRa build)
                  deviceLocked = !deviceLocked;
                  Serial.printf("[LOCK] %s\n", deviceLocked ? "Locked" : "Unlocked");
                  break;
#endif
        }
        oledDraw();
      } else if (tzAdjustMode) {
        tzOffset++; if (tzOffset>14) tzOffset=-12;
        oledDraw();
#if ENABLE_LORA
      } else if (loraMsgCount > 0 && currentPage == pageMessages()) {
        if (btnPendingSingle && (now - btnPendingMs <= BTN_DOUBLE_MS)) {
          // Double press — navigate to next page
          btnPendingSingle = false;
          loraMsgScroll = 0;  // reset scroll when leaving
          nextPage();
        } else {
          // First/single press — scroll up (newer messages) or wait for double
          btnPendingSingle = true;
          btnPendingMs = now;
          lastActivityMs = now;
          if (loraMsgScroll > 0) {
            loraMsgScroll--;
            oledDraw();
          }
        }
#endif
      } else if (routeLogCount>0 && currentPage==pageRouteLog()) {
        if (logClearWarn) {
          logClearWarn = false;
          oledDraw();
        } else {
          nextPage();
        }
      } else if (currentPage==pageTrack()) {
        if (btnPendingSingle && (now - btnPendingMs <= BTN_DOUBLE_MS)) {
          // Second press arrived within window — double press confirmed
          btnPendingSingle = false;
          if (routeRecording) {
            routeRecording = false;
            routeLogSave();  // save completed route to log
            Serial.printf("[ROUTE] Stopped. %d pts\n", routeCount);
          } else {
            clearRoute();
            routeStartMs=millis(); lastRouteMs=0;
            routeRecording = true;
            Serial.println("[ROUTE] Started");
          }
          oledDraw();
        } else {
          // First press — start waiting for possible double press
          btnPendingSingle = true;
          btnPendingMs = now;
          lastActivityMs = now;  // prevent screen timeout during wait window
          // Don't act yet — loop() will handle timeout
        }
      } else if (currentPage==pageCountdown()) {
        if (btnPendingSingle && (now - btnPendingMs <= BTN_DOUBLE_MS)) {
          // Double press — start/stop (always works, exits set mode first)
          btnPendingSingle = false;
          cdtSetMode = false;
          if (!cdtDone) {
            if (cdtRunning) {
              cdtRemMs   = cdtRemMs - (millis() - cdtStartMs);
              cdtRunning = false;
            } else {
              cdtRemMs   = cdtSetSecs * 1000UL;
              cdtStartMs = millis();
              cdtRunning = true;
            }
          }
          oledDraw();
        } else {
          // Single press — cycle preset in set mode, navigate otherwise
          btnPendingSingle = false;
          if (cdtSetMode) {
            cdtPresetIdx = (cdtPresetIdx + 1) % CDT_PRESET_COUNT;
            cdtSetSecs   = CDT_PRESETS[cdtPresetIdx];
            oledDraw();
          } else {
            nextPage();
          }
        }
      } else if (currentPage==pageStopwatch()) {
        if (btnPendingSingle && (now - btnPendingMs <= BTN_DOUBLE_MS)) {
          // Second press arrived within window — double press confirmed
          btnPendingSingle = false;
          if (swRunning) swStop();
          else           swStart();
          oledDraw();
        } else {
          // First press — start waiting for possible double press
          btnPendingSingle = true;
          btnPendingMs = now;
          lastActivityMs = now;  // prevent screen timeout during wait window
          // Don't act yet — loop() will handle timeout
        }
      } else if (currentPage==pagePower()) {
        if (sosActive) {
          // Any press while SOS active — cancel SOS
          loraSOSCancel();
        } else if (btnPendingSingle && (now - btnPendingMs <= BTN_DOUBLE_MS)) {
          // Double press on Power page — activate SOS
          btnPendingSingle = false;
          loraSOSStart();
        } else {
          // First press — start waiting for possible double press
          btnPendingSingle = true;
          btnPendingMs = now;
          lastActivityMs = now;
        }
      } else {
        nextPage();
      }
    }
  }

  btnLastState = state;
}

// ============================================================
// GPS SIMULATION
// ============================================================
#ifdef SIMULATE_GPS
void simulateGPS() {
  if (millis() - lastGpsUpdate < 1000) return;
  lastGpsUpdate = millis();

  // Simulate 5 second search before fix acquired
  // simSearchStart is set the first time simulateGPS runs
  static uint32_t simSearchStart = 0;
  if (simSearchStart == 0) simSearchStart = millis();
  uint32_t elapsed = millis() - simSearchStart;

  if (elapsed < 10000) {
    cachedFix  = false;
    cachedSats = elapsed / 1200;  // 0..7 sats building up
    return;
  }

  const float R = 0.0003;
  simAngle += 2.0;
  if (simAngle >= 360.0) simAngle = 0.0;
  cachedLat = 53.259171 + R*cos(radians(simAngle));
  cachedLon = -2.518049 + R*sin(radians(simAngle));
  cachedSpd = 5.2;
  cachedHdg = fmod(simAngle + 90.0, 360.0);
  cachedAlt = 120.0 + 40.0 * sin(radians(simAngle * 2));  // undulating 80-160m
  cachedSats = 8;
  cachedFix  = true;
}
#endif

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] Azimuth v3.15");
  Serial.printf("[BOOT] Device: %s\n", bleDeviceName);

  pinMode(VEXT_CTRL_PIN,  OUTPUT); digitalWrite(VEXT_CTRL_PIN,  LOW);
  pinMode(VGNSS_CTRL_PIN, OUTPUT); digitalWrite(VGNSS_CTRL_PIN, LOW);
  delay(200);

  Wire.begin(OLED_SDA, OLED_SCL);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  delay(20);
  digitalWrite(OLED_RST, HIGH); delay(20);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("[OLED] Init FAILED");
  } else {
    Serial.println("[OLED] Ready");
  display.invertDisplay(!setBrightFull);  // apply saved brightness mode
    showSplash();
  }

  // Load persisted route logs, waypoints and groups from FFat
  routeLogLoad();
  waypointLoad();
  groupsLoad();
  pendingKeyShareLoad();  // restore any pending key share request
  storedGroupsLoad();     // restore on-device stored groups

  pinMode(BTN_PIN, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_PIN, 0);
  lastActivityMs = millis();
  Serial.println("[BTN] Ready");
  lastActivityMs = millis();  // start timeout from boot complete

  analogReadResolution(12);
  readBattery();

  // LED init
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  loadTimezone();

  // GPS UART init — VGNSS already LOW (powered on) from setup() start
  delay(1000);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  while (gpsSerial.available()) gpsSerial.read();  // flush startup chars
  Serial.printf("[GPS] UART RX=%d TX=%d Baud=%d\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

  // Send hot start if we have a last known position, otherwise warm start
  if (lastKnownLat != 0.0) {
    // Provide last known position hint for faster acquisition
    // Format: $PCAS11,lat,N/S,lon,E/W,alt,DDMMYY,HHMMSS.SS*checksum
    // Simpler: just send hot start command — module uses its own NV memory
    gpsSerial.print("$PCAS10,0*1C\r\n");  // hot start
    Serial.println("[GPS] Hot start sent");
  } else {
    gpsSerial.print("$PCAS10,1*1D\r\n");  // warm start
    Serial.println("[GPS] Warm start sent");
  }
  delay(100);
  // Set update rate to 5Hz for faster fix detection
  gpsSerial.print("$PCAS02,200*1D\r\n");  // 200ms = 5Hz
  Serial.println("[GPS] Update rate set to 5Hz");

  // Generate random 6-digit pairing PIN before BLE init
  pairingPin = (esp_random() % 900000) + 100000;  // 100000-999999
  Serial.printf("[BLE] Pairing PIN: %06lu\n", pairingPin);

  NimBLEDevice::init(bleDeviceName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);       // max TX power
  NimBLEDevice::setMTU(185);                    // larger MTU for route data

  // Security — accept connections without pairing/bonding
  // This prevents Windows/macOS from dropping the connection
  // when they try to initiate pairing and get rejected
  // Just Works bonding — reliable with Web Bluetooth on Windows/Mac/Android
  // PIN security is handled at application level via the web app
  NimBLEDevice::setSecurityAuth(true, false, false);  // bond=true, MITM=false, SC=false
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityCallbacks(new SecurityCB());

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new BLEServerCB());
  pServer->advertiseOnDisconnect(true);

  // Generic Access Service (0x1800) — required for proper Windows recognition
  NimBLEService* pGASvc = pServer->createService((uint16_t)0x1800);
  // Device Name characteristic (0x2A00)
  NimBLECharacteristic* pNameChar = pGASvc->createCharacteristic(
    (uint16_t)0x2A00, NIMBLE_PROPERTY::READ);
  pNameChar->setValue(bleDeviceName);
  // Appearance characteristic (0x2A01) — 0x0180 = Generic Location and Navigation
  NimBLECharacteristic* pAppearChar = pGASvc->createCharacteristic(
    (uint16_t)0x2A01, NIMBLE_PROPERTY::READ);
  // Appearance: 0x0180 = Generic Location and Navigation, little-endian
  uint8_t appearance[2] = {0x80, 0x01};
  pAppearChar->setValue(appearance, 2);
  pGASvc->start();

  // Device Information Service (0x180A)
  NimBLEService* pDISvc = pServer->createService((uint16_t)0x180A);
  NimBLECharacteristic* pMfgChar = pDISvc->createCharacteristic(
    (uint16_t)0x2A29, NIMBLE_PROPERTY::READ);
  pMfgChar->setValue("OEO Technologies");
  NimBLECharacteristic* pModelChar = pDISvc->createCharacteristic(
    (uint16_t)0x2A24, NIMBLE_PROPERTY::READ);
  pModelChar->setValue("Azimuth GPS Tracker");
  NimBLECharacteristic* pFWChar = pDISvc->createCharacteristic(
    (uint16_t)0x2A26, NIMBLE_PROPERTY::READ);
  pFWChar->setValue(FW_VERSION);
  pDISvc->start();

  NimBLEService* pSvc = pServer->createService(BLE_SERVICE_UUID);

  pGpsChar = pSvc->createCharacteristic(
    BLE_GPS_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pGpsChar->setValue("{\"fix\":0}");
  pGpsChar->setCallbacks(new GpsCharCB());

  pRouteChar = pSvc->createCharacteristic(
    BLE_ROUTE_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pRouteChar->setValue("{\"ready\":true}");
  pRouteChar->setCallbacks(new RouteCharCB());

  pScrnChar = pSvc->createCharacteristic(
    BLE_SCRN_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pScrnChar->setValue("idle");

  pSetChar = pSvc->createCharacteristic(
    BLE_SET_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pScanChar = pSvc->createCharacteristic(
    BLE_SCAN_CHAR_UUID,
    NIMBLE_PROPERTY::READ);  // web app polls this after scan
  const char* initScan = "{\"nets\":[]}";
  pScanChar->setValue((uint8_t*)initScan, strlen(initScan));
  pResChar = pSvc->createCharacteristic(
    BLE_RES_CHAR_UUID,
    NIMBLE_PROPERTY::READ);
  const char* initRes = "{}";
  pResChar->setValue((uint8_t*)initRes, strlen(initRes));

  pPeerChar = pSvc->createCharacteristic(
    BLE_PEER_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pPeerChar->setValue("{}");

  pSetChar->setValue("{}");
  pSetChar->setCallbacks(new SetCharCB());

  pSvc->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  // Only advertise standard 16-bit UUIDs — custom 128-bit UUIDs trigger
  // Windows "Base System Device" driver installation
  pAdv->addServiceUUID((uint16_t)0x180A);  // Device Information Service
  pAdv->addServiceUUID((uint16_t)0x1800);  // Generic Access Service
  pAdv->setMinPreferred(0x06);
  pAdv->setMaxPreferred(0x12);
  pAdv->start();
  Serial.println("[BLE] Advertising started");

  // LoRa init — must be before WiFi to avoid SPI conflicts
#if ENABLE_LORA
  Serial.flush();
  delay(100);
  loraInitDeviceId();
  loraLoadKey();
  Serial.printf("[LORA] Device ID: %s\n", loraDeviceId);
  memset(loraPeers, 0, sizeof(loraPeers));
  Serial.println("[LORA] Initialising SX1262...");
  Serial.flush();
  if (loraBegin()) {
    Serial.println("[LORA] Peer sharing active");
  } else {
    Serial.println("[LORA] LoRa unavailable — GPS/BLE still operational");
  }
#endif

  // WiFi connected mode — fetch data at boot if credentials stored
  if (wifiEnabled) {
    wifiConnectAndFetch();
  }
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  uint32_t now = millis();

  // GPS
#ifdef SIMULATE_GPS
  simulateGPS();
#else
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (now - lastGpsUpdate >= 1000) {
    lastGpsUpdate = now;
    bool newFix = gps.location.isValid();
    if (newFix && !cachedFix) {
      cachedFixIsApprox = false;
      cachedApproxAccM  = 5000.0f;
    }
    cachedFix  = newFix;
    cachedLat  = cachedFix ? gps.location.lat() : cachedLat;
    cachedLon  = cachedFix ? gps.location.lng() : cachedLon;
    cachedSpd  = gps.speed.isValid()   ? gps.speed.kmph()    : 0.0;
    cachedHdg  = gps.course.isValid()  ? gps.course.deg()    : 0.0;
    cachedSats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  }
#endif

  // Route recording
  if (routeRecording && cachedFix && (now - lastRouteMs >= ROUTE_INTERVAL_MS)) {
    lastRouteMs = now;
    routeAddPoint();
  }

  // LED flash every 10s when route recording — reminder indicator
  static uint32_t lastLedFlash = 0;
  static bool     ledOn        = false;
  if (routeRecording) {
    if (!ledOn && (now - lastLedFlash >= LED_INTERVAL_MS)) {
      digitalWrite(LED_PIN, HIGH);
      ledOn        = true;
      lastLedFlash = now;
    } else if (ledOn && (now - lastLedFlash >= LED_FLASH_MS)) {
      digitalWrite(LED_PIN, LOW);
      ledOn = false;
    }
  } else {
    // Ensure LED is off when not recording
    if (ledOn) { digitalWrite(LED_PIN, LOW); ledOn = false; }
  }

  // Countdown timer — check for zero and handle alert
  if (cdtRunning) {
    uint32_t elapsed = millis() - cdtStartMs;
    if (elapsed >= cdtRemMs) {
      // Hit zero
      cdtRunning    = false;
      cdtDone       = true;
      cdtAlerting   = true;
      cdtAlertStart = now;
      cdtAlertFlash = 0;
      cdtRemMs      = 0;
      Serial.println("[CDT] Reached zero");
    }
  }

  // Alert — flash screen and LED for 5 seconds
  if (cdtAlerting) {
    uint32_t alertElapsed = now - cdtAlertStart;
    if (alertElapsed < 5000) {
      // Flash every 250ms
      uint8_t flashStep = alertElapsed / 250;
      if (flashStep != cdtAlertFlash) {
        cdtAlertFlash = flashStep;
        if (flashStep % 2 == 0) {
          display.invertDisplay(true);
          digitalWrite(LED_PIN, HIGH);
        } else {
          display.invertDisplay(false);
          digitalWrite(LED_PIN, LOW);
        }
      }
    } else {
      // Alert done
      cdtAlerting = false;
      display.invertDisplay(false);
      digitalWrite(LED_PIN, LOW);
      oledDraw();
    }
  }

  // Save last known position to NVS every 30s when we have a fix
  if (cachedFix && (now - lastPosSaveMs >= POS_SAVE_INTERVAL_MS)) {
    lastPosSaveMs = now;
    saveLastPosition();
  }

  // Elevation buffer — sample cachedAlt every second when we have a fix
  if (cachedFix && (now - lastElevMs >= 1000)) {
    lastElevMs = now;
    elevBuf[elevBufHead] = cachedAlt;
    elevBufHead = (elevBufHead + 1) % ELEV_BUF_SIZE;
    if (elevBufCount < ELEV_BUF_SIZE) elevBufCount++;
  }

  // Auto-push route log when BLE connects
  if (blePushLogPending && bleConnected && clientSubscribed) {
    blePushLogPending = false;
    blePushRouteLog();
  }

  // Battery
  if (now - lastBatRead >= BAT_READ_MS) {
    lastBatRead = now;
    readBattery();
  }

  // OLED update
  uint32_t oledInterval = (currentPage==pageCountdown() && cdtRunning) ? 500
                        : (currentPage==pageStopwatch() && swRunning) ? 100
                        : (currentPage==0 && !cachedFix) ? 300
                        : bleConnected ? 2000 : 1000;
  // Compass and navigate refresh faster
  if (currentPage==pageCompass() || (currentPage==pageWaypoint()) || (weather.valid && currentPage==pageWeather())) oledInterval = 500;

  if (now - lastOledUpdate >= oledInterval) {
    lastOledUpdate = now;
    oledDraw();
  }

  handleButton();  // process button first so lastActivityMs is current

  // Recalculate now after button handler (btnPendingMs may have been updated)
  now = millis();

  // Pending single press timeout — if no second press arrived, act
  if (btnPendingSingle && (now - btnPendingMs > BTN_DOUBLE_MS)) {
    Serial.printf("[LOOP] Single press timeout, pending was 1, elapsed=%lu\n", now - btnPendingMs);
    btnPendingSingle = false;
    btnLongFired     = true;  // prevent long press on new page
    btnPressedMs     = now;
    lastActivityMs   = now;  // critical — reset timeout before navigating
    
#if ENABLE_LORA
    // Groups page single press handling
    if (currentPage == pageGroups()) {
      if (pendingKeyShare.active || keyShareActive) {
        // In broadcast state — ignore single press
        wakeScreen(); oledDraw();
        return;
      } else if (groupMenuOpen) {
        // Cycle through menu items
        bool isPersonal = (storedGroupSel == 0);
        bool isLeader = !isPersonal && storedGroups[storedGroupSel - 1].isLeader;
        bool canLeave = isPersonal || storedGroups[storedGroupSel - 1].canLeave;
        int menuCount = 1; // Activate/Deactivate
        if (!isPersonal && isLeader) menuCount++;
        if (!isPersonal && canLeave) menuCount++;
        menuCount++; // Back
        groupMenuSel = (groupMenuSel + 1) % menuCount;
        Serial.println("[LOOP] Groups menu cycle");
        wakeScreen(); oledDraw();
        return;
      }
      // Normal list view — navigate to next page
      Serial.println("[LOOP] Groups navigate to next page");
    }
#endif
    
    if (!settingsSel) {      // block pagination while editing settings
      if (currentPage == pageSettings()) { settingsRow = 0; }
      wakeScreen();
      nextPage();
    } else {
      wakeScreen(); oledDraw();
    }
  }

  // WiFi hourly refresh
  if (wifiEnabled && wifiLastFetchMs > 0 &&
      millis() - wifiLastFetchMs >= WIFI_FETCH_INTERVAL_MS) {
    wifiConnectAndFetch();
    oledDraw();
  }

  // Settings saved message — clear after 1s, then show QR if hotspot is on
  if (settingsSaved && (millis() - settingsSavedMs >= 1000)) {
    settingsSaved = false;
    if (apEnabled) {
      // Show QR code for WiFi connection
      qrCodeActive = true;
      drawQRCode();
    } else {
      oledDraw();
    }
  }

  // Settings idle abandon — exit edit mode after 5s inactivity
  if (settingsSel && (millis() - settingsIdleMs >= SETTINGS_IDLE_MS)) {
    settingsSel = false;
    settingsRow = 0;
    oledDraw();
  }

  // Screen timeout (skip if QR code is active — user needs time to scan)
  if (screenOn && !qrCodeActive && currentPage!=pageCompass() && !(locSaved && currentPage==pageWaypoint()) && !(weather.valid && currentPage==pageWeather())) {
    if (millis() - lastActivityMs >= timeoutMs()) {
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      screenOn = false;
    }
  }

  // BLE notify
  // ── Async OLED redraw ────────────────────────────────────────────────────
  if (oledRedrawPending && !qrCodeActive) {
    oledRedrawPending = false;
    oledDraw();
  }

  // ── Async WiFi refresh ──────────────────────────────────────────────────
  if (wifiRefreshPending) {
    wifiRefreshPending = false;
    wifiConnectAndFetch();
    if (!qrCodeActive) oledDraw();
  }

  // ── Async waypoint clear broadcast ──────────────────────────────────────
  if (wpClearPending) {
    wpClearPending = false;
    loraWaypointClearBroadcast();
  }

  // ── Async waypoint parse + broadcast ────────────────────────────────────
  if (wpBroadcastPending) {
    wpBroadcastPending = false;
    // Parse JSON on main loop stack — safe from BLE stack overflow
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, wpPendingJson);
    if (!err && doc["wps"].is<JsonArray>()) {
      JsonArray arr = doc["wps"].as<JsonArray>();
      int n = 0;
      for (JsonObject wp : arr) {
        if (n >= WP_MAX) break;
        waypoints[n].lat = wp["lat"] | 0.0f;
        waypoints[n].lon = wp["lon"] | 0.0f;
        const char* nm = wp["name"] | "";
        strncpy(waypoints[n].name, nm, WP_NAME_LEN);
        waypoints[n].name[WP_NAME_LEN] = 0;
        n++;
      }
      waypointCount   = n;
      waypointCurrent = 0;
      waypointSave();
      Serial.printf("[WP] Set %d waypoint(s) from BLE\n", waypointCount);
      // Now safe to broadcast — we're on the main loop, not BLE stack
      loraWaypointBroadcast();
      // Notify web app with real count
      if (bleConnected && clientSubscribed && sessionAuthorised && pSetChar) {
        char resp[32];
        snprintf(resp, sizeof(resp), "{\"wps\":%d}", waypointCount);
        pSetChar->setValue(resp);
        pSetChar->notify();
      }
    }
    memset(wpPendingJson, 0, sizeof(wpPendingJson));
  }

  // ── Async weather push from BLE ─────────────────────────────────────────
  if (wxPushPending) {
    wxPushPending = false;
    StaticJsonDocument<320> doc;
    if (deserializeJson(doc, wxPendingJson) == DeserializationError::Ok) {
      // Always apply to own weather struct
      weather.tempC       = doc["t"]  | 0.0f;
      weather.feelsLikeC  = doc["f"]  | 0.0f;
      weather.tempMinC    = doc["mn"] | 0.0f;
      weather.tempMaxC    = doc["mx"] | 0.0f;
      weather.humidity    = doc["h"]  | 0;
      weather.windKmh     = doc["w"]  | 0.0f;
      weather.windGustKmh = doc["g"]  | 0.0f;
      weather.weatherId   = doc["id"] | 0;
      strlcpy(weather.windDir,     doc["wd"] | "?", sizeof(weather.windDir));
      strlcpy(weather.desc,        doc["d"]  | "",  sizeof(weather.desc));
      strlcpy(weather.locationName,doc["loc"]| "",  sizeof(weather.locationName));
      weather.valid = true; weatherReceived = true;
      if (gps.time.isValid()) {
        int fh, fm, fs, fd, fmo, fy;
        getLocalTime(fh, fm, fs, fd, fmo, fy);
        wifiLastFetchHour = fh;
        wifiLastFetchMin  = fm;
      }
      Serial.printf("[WX] Weather set via BLE: %.1fC %s\n", weather.tempC, weather.desc);
      oledDraw();
      // If a peer ID is specified, forward via LoRa
      if (strlen(wxPendingPeerId) == 6) {
        loraSendWeather(wxPendingPeerId);
        yield();
      }
    }
    memset(wxPendingJson,  0, sizeof(wxPendingJson));
    memset(wxPendingPeerId,0, sizeof(wxPendingPeerId));
  }
  if (wifiScanPending) {
    wifiScanPending = false;
    Serial.println("[WIFI] Scanning networks...");
    WiFi.mode(WIFI_STA);
    int n = WiFi.scanNetworks();
    String json = "{\"nets\":[";
    int sent = 0;
    for (int i = 0; i < n && sent < 6; i++) {
      String ssid = WiFi.SSID(i);
      if (ssid.length() == 0) continue;
      if (ssid.length() > 20) ssid = ssid.substring(0, 20);
      for (int ch2 = ssid.length()-1; ch2 >= 0; ch2--) {
        if (ssid.charAt(ch2) == '"') ssid.remove(ch2, 1);
      }
      if (sent > 0) json += ",";
      json += "{\"s\":\"" + ssid + "\",\"r\":" + String(WiFi.RSSI(i)) + "}";
      sent++;
    }
    json += "]}";
    WiFi.scanDelete();
    WiFi.mode(WIFI_OFF);
      wifiScanResult = json;
    if (pScanChar) pScanChar->setValue((uint8_t*)json.c_str(), json.length());
    }

  if (bleConnected && clientSubscribed && sessionAuthorised && (now - lastBleNotify >= 1000)) {
    lastBleNotify = now;
    String json = buildJSON();
    pGpsChar->setValue((uint8_t*)json.c_str(), json.length());
    pGpsChar->notify();
  }

  // ── LoRa TX/RX ───────────────────────────────────────────────────────────
#if ENABLE_LORA
  if (loraReady && radio &&
      (now - lastLoraTxMs >= (uint32_t)loraIntervalSec * 1000UL)) {
    lastLoraTxMs = now;
    uint8_t pkt[48];
    int pktLen = loraBuildPacket(pkt, sizeof(pkt));
    radio->clearDio1Action();
    int state = radio->transmit(pkt, pktLen);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.printf("[LORA] TX %d bytes (id=%s)\n", pktLen, loraDeviceId);
    } else {
      Serial.printf("[LORA] TX failed: %d\n", state);
    }
    radio->setDio1Action(loraRxISR);
    radio->startReceive();
    loraRxFlag = false;
  }

  // ── Message repeat TX ────────────────────────────────────────────────────
  if (loraReady && radio && msgRepeatCount > 0 && now >= msgRepeatNextMs) {
    radio->clearDio1Action();
    int state = radio->transmit(msgRepeatBuf, msgRepeatLen);
    radio->setDio1Action(loraRxISR);
    radio->startReceive();
    loraRxFlag = false;
    msgRepeatCount--;
    msgRepeatNextMs = now + 5000;
    Serial.printf("[LORA] MSG repeat, %d remaining\n", msgRepeatCount);
  }

  // ── SOS TX ───────────────────────────────────────────────────────────────
  if (loraReady && radio && sosActive && now >= sosNextTxMs) {
    uint8_t sosPkt[48]; int sosLen = 0;
    loraSOSBuild(sosPkt, sosLen);
    radio->clearDio1Action();
    radio->transmit(sosPkt, sosLen);
    radio->setDio1Action(loraRxISR);
    radio->startReceive();
    loraRxFlag = false;
    sosBroadcastCount++;
    sosNextTxMs = now + SOS_REPEAT_GAP_MS;
    Serial.printf("[LORA] SOS TX %d/%d\n", sosBroadcastCount, SOS_REPEAT_COUNT);
    if (sosBroadcastCount >= SOS_REPEAT_COUNT) loraSOSCancel();
    else oledDraw(); // update remaining count on screen
  }

  // ── SOS LED rapid flash while active ────────────────────────────────────
  if (sosActive) {
    static uint32_t sosLedMs = 0;
    static bool sosLedState = false;
    if (now - sosLedMs >= SOS_LED_FLASH_MS) {
      sosLedMs = now;
      sosLedState = !sosLedState;
      digitalWrite(LED_PIN, sosLedState ? HIGH : LOW);
    }
  }

  // ── Stale peer alert ─────────────────────────────────────────────────────
  // Flash OLED and notify BLE when a previously active peer goes silent >5 min
  static uint32_t lastStaleCheckMs = 0;
  if (now - lastStaleCheckMs >= 30000UL) {
    lastStaleCheckMs = now;
    for (int p = 0; p < LORA_MAX_PEERS; p++) {
      if (!loraPeers[p].active) continue;
      uint32_t silentMs = now - loraPeers[p].lastSeenMs;
      // Alert once at the 5-minute mark (within the 30s check window)
      if (silentMs >= 300000UL && silentMs < 330000UL) {
        Serial.printf("[LORA] Stale peer alert: %s\n", loraPeers[p].name);
        // Flash OLED: brief invert flash to draw attention
        display.invertDisplay(true);  delay(200);
        display.invertDisplay(false); delay(100);
        display.invertDisplay(true);  delay(200);
        display.invertDisplay(false);
        // Notify BLE with stale peer JSON
        if (bleConnected && clientSubscribed && sessionAuthorised && pPeerChar) {
          char json[80];
          snprintf(json, sizeof(json),
            "{\"stale\":1,\"peer\":\"%s\",\"name\":\"%s\",\"silentMin\":%lu}",
            loraPeers[p].id, loraPeers[p].name, silentMs / 60000UL);
          pPeerChar->setValue((uint8_t*)json, strlen(json));
          pPeerChar->notify();
        }
      }
    }
  }

  if (loraReady && radio && loraRxFlag) {
    loraRxFlag = false;
    uint8_t buf[128];
    int state = radio->readData(buf, sizeof(buf));
    if (state == RADIOLIB_ERR_NONE) {
      int len = radio->getPacketLength();
      // Capture signal quality for this packet
      loraLastRssi = (int16_t)radio->getRSSI();
      loraLastSnr  = (int8_t)radio->getSNR();
      loraHandlePacket(buf, len);
    }
    radio->startReceive();
    yield();
  }
  // Handle scheduled registration response (random backoff)
  if (regResponsePending && millis() >= regResponseScheduledMs) {
    loraRegResponse();
  }
  static uint32_t lastPeerExpiry = 0;
  if (now - lastPeerExpiry >= 30000UL) {
    lastPeerExpiry = now;
    loraExpirePeers();
  }
#endif // ENABLE_LORA

  // ── AP web server ────────────────────────────────────────────────────────
  if (apEnabled) {
    dnsServer.processNextRequest();
    webServer.handleClient();
  }

  delay(10);
}
