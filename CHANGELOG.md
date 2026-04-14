# Azimuth GPS Tracker — v3.19 Release Notes

**Release Date:** April 2026

## Overview

Version 3.19 introduces the **Multi-Protocol Scanner** — a passive listener that receives and displays position broadcasts from Meshtastic and MeshCore devices on the map. This release also includes numerous UI improvements, BLE pairing enhancements, and bug fixes.

---

## Major New Features

### Multi-Protocol Scanner (Meshtastic & MeshCore)

Azimuth can now passively listen for position broadcasts from other mesh networks and display them as external peers on the map.

**Supported Networks:**
- **Meshtastic** — EU868 LongFast (869.525 MHz, SF11, BW250)
- **MeshCore** — EU868 (869.4 MHz, SF11, BW250)

**Key Features:**
- Automatic channel hopping between Meshtastic and MeshCore frequencies
- AES-128-CTR decryption for Meshtastic packets (supports custom channel keys)
- Default public channel (AQ==) support
- Protobuf parsing for position data extraction
- External peers shown with "M" badge on map and in peers list
- Signal quality display (RSSI/SNR)
- Configurable scanner modes: Off, Meshtastic only, MeshCore only, Auto (both)

**Web App — Channel Management:**
- Add custom Meshtastic channels via share URL import
- Manual channel entry (name + PSK)
- Per-channel enable/disable toggles
- Packet count statistics per channel
- Default channel toggle
- Persistent storage via NVS

### WiFi Radar — Live Heading Indicator

The WiFi hotspot radar view now displays a live compass heading indicator showing the direction the device is facing.

- Blue arrow pointing from center in direction of travel
- "N" label at top of radar indicating north
- Updates in real-time based on GPS heading
- Only displays when GPS has a valid fix

---

## Device UI Improvements

### Pairing Screen Enhancements
- **Auto-popup**: PIN screen now appears automatically when an unbound device connects
- **Larger PIN**: Increased from text size 2 to size 3 for better readability
- **Simplified text**: Changed from "Enter this PIN in your browser:" to just "Enter this PIN:"
- **Auto-dismiss**: Screen returns to previous page after successful PIN verification

### Waypoint Page Layout
- Moved horizontal rule up to give more space for hint text
- Navigation dots now displayed in left panel only (not duplicated at center)
- Shortened hint text: "Hld:nxt" and "Hld:clr"

### Track Page Layout
- Moved hint text up slightly (y=47) to center between rule and nav dots

### Splash Screen
- Reduced mountain hold time: 900ms → 500ms
- Reduced final frame hold time: 2500ms → 1200ms
- Faster boot experience overall

---

## Web App Improvements

### LoRa Settings Panel Redesign
- Clear section headers: "RADIO SETTINGS", "NETWORK KEY (AES-128)", "MESHTASTIC SCANNER"
- Divider lines between sections
- Consistent styling with WiFi & Weather panel

### Meshtastic Channels Panel
- Toggle alignment fixed (removed scale transform)
- Channel enable/disable now works correctly (sends MESHCHTOG command)
- Visual consistency improvements

### Exit Wayfinding Dialog
- Max-width capped at 420px for desktop browsers
- Previously stretched full width on large screens

### Device Info
- Firmware version now displayed in About section
- Board type shown (Heltec V3 / Wireless Tracker)
- Format: "v3.19 (Heltec V3)"

---

## API Changes

### New BLE/WiFi Commands

| Command | Description |
|---------|-------------|
| `MESHCHTOG:idx:0/1` | Toggle channel enabled state |
| `MESHCHDEF:0/1` | Toggle default channel (AQ==) |
| `DELMESHCH:idx` | Delete a custom channel |
| `MESHCHURL:url` | Import channel from Meshtastic share URL |
| `ADDMESHCH:name\|psk` | Add channel manually |
| `GETMESHCH` | Get list of configured channels |

### Settings JSON Extended

`buildSettingsJSON()` now includes:
```json
{
  "fw": "v3.19",
  "board": "v3"
}
```

Board values: `"v3"` (Heltec WiFi LoRa 32 V3) or `"tracker"` (Wireless Tracker)

---

## Bug Fixes

- **NVS Stack Overflow**: BLE handlers no longer call `meshChannelsSave()` directly — uses deferred flag checked in `loop()`
- **Meshtastic Decryption**: Fixed nonce byte order (packetId in bytes 0-7, fromNode in bytes 8-15)
- **Key Expansion**: Correct handling of AQ== key (simple zero-padded, not SHA256 hashed)
- **Packet Receive Loop**: Fixed stale `getPacketLength()` by using interrupt-driven receive pattern
- **Channel Toggle**: `meshToggleChannel()` now sends actual command instead of just reloading
- **PIN Screen Dismiss**: VERPIN success now clears `pairingActive` and returns to previous page

---

## File Changes

| File | Description |
|------|-------------|
| `gps_tracker_v319.ino` | Main firmware with scanner and all fixes |
| `meshtastic_scanner.cpp` | Scanner implementation (config switching, decryption, parsing) |
| `meshtastic_scanner.h` | Scanner header with configuration constants |
| `index.html` | Web app with channel management UI |

---

## Scanner Configuration

```cpp
// Scan timing
#define SCANNER_SCAN_INTERVAL_MS  30000   // Scan every 30 seconds
#define SCANNER_SCAN_DURATION_MS  8000    // Listen for 8 seconds per protocol

// Meshtastic EU868 LongFast
#define MESHTASTIC_FREQ_EU868     869.525
#define MESHTASTIC_BW             250.0
#define MESHTASTIC_SF             11
#define MESHTASTIC_CR             5

// MeshCore EU868
#define MESHCORE_FREQ_EU868       869.4
#define MESHCORE_BW               250.0
#define MESHCORE_SF               11
#define MESHCORE_CR               5
```

---

## Upgrade Notes

1. Flash new firmware to device
2. Run `make_webapp.ps1` to regenerate `webapp_gz.h` from updated `index.html`
3. Copy `meshtastic_scanner.cpp` and `meshtastic_scanner.h` to firmware folder
4. Scanner is disabled by default — enable via LoRa Settings → Scanner
5. Existing settings and routes are preserved

---

## Known Limitations

- Scanner only supports EU868 frequencies (US915/AU915 frequencies defined but not tested)
- Meshtastic decryption requires matching channel key
- GPS heading only accurate when moving (stationary heading may drift)
- Scanner temporarily switches radio config — may miss Azimuth packets during scan window

---

## What's Next (v3.20)

- APRS-IS integration (Phase 3)
- US915/AU915 scanner frequency support
- Meshtastic message display (text messages)
- MeshCore full packet parsing

---

*Built by [OEO Technologies](https://oeo.dev) in the UK.*
