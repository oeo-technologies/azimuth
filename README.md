# Azimuth

**Open-source GPS tracker with LoRa mesh networking — no cloud, no subscriptions.**

<p align="center">
  <img src="docs/hero.png" alt="Azimuth device and web app" width="600">
</p>

Azimuth lets hikers, runners, and outdoor groups share real-time positions over LoRa radio — even without mobile signal. Connect via Bluetooth or the device's WiFi hotspot to see everyone on a map.

## Features

- 📍 **Real-time GPS** — position, speed, altitude, heading
- 📡 **LoRa mesh** — share location with nearby devices (1-10km range)
- 🔁 **Relay mode** — extend range by relaying packets through intermediate devices
- 🔐 **AES-128 encryption** — your position data stays private
- 📴 **Fully offline** — no cloud, no SIM, no subscriptions
- 🗺️ **Route management** — record tracks, name them, view stats, export GPX
- 🌐 **Web app** — connect via BLE or WiFi hotspot, works offline with cached maps
- 🖥️ **OLED display** — 14+ pages: position, compass, track recording, waypoints, weather, and more
- ⚡ **Web flasher** — update firmware from your browser, no IDE needed
- 📻 **Multi-protocol scanner** — see Meshtastic and MeshCore devices on your map (NEW in v3.19)

## Hardware

| Component | Spec |
|-----------|------|
| Board | [Heltec WiFi LoRa 32 V3](https://heltec.org/project/wifi-lora-32-v3/) + GPS module |
| MCU | ESP32-S3 dual-core @ 240MHz |
| LoRa | SX1262 (EU868 / US915 / AU915) |
| Display | 128×64 OLED |
| GPS | AT6558R |
| Battery | 3.7V LiPo (JST connector) |

**Alternative:** [Heltec Wireless Tracker](https://heltec.org/project/wireless-tracker/) with built-in GPS and TFT display.

**Total cost:** ~£30

## Quick Start

### Option 1: Web Flasher (easiest)
1. Go to [oeo.dev/flash](https://oeo.dev/flash)
2. Connect your device via USB
3. Click "Connect" and select the serial port
4. Flash — done!

### Option 2: Arduino IDE
1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add Heltec ESP32 board package: `https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.9/package_heltec_esp32_index.json`
3. Install libraries: `RadioLib`, `NimBLE-Arduino`, `Adafruit SSD1306`, `TinyGPSPlus`, `mbedtls`
4. Open `firmware/gps_tracker_v319.ino`
5. Select board: **WiFi LoRa 32(V3)**, USB CDC On Boot: **Enabled**
6. Upload

## Web App

The companion web app runs entirely in your browser:

- **Online:** [azimuth.oeo.dev](https://azimuth.oeo.dev) — connect via Bluetooth
- **Offline:** Connect to device WiFi hotspot (`Azimuth-{name}`, password: `azimuth1`)

Features:
- Live map with your position and all peers
- Route recording with elevation, speed, and pace charts
- Route viewer — browse saved routes, rename, delete, export GPX
- Waypoint management
- Offline map caching (download tiles for use without internet)
- Direct and broadcast messaging
- **Multi-protocol scanner** — see Meshtastic/MeshCore peers (v3.19+)
- **Live radar heading** — compass direction on WiFi radar view (v3.19+)

## Multi-Protocol Scanner (v3.19+)

Azimuth can passively listen for position broadcasts from other mesh networks:

| Network | Frequency (EU) | Encryption |
|---------|----------------|------------|
| Meshtastic | 869.525 MHz | AES-128-CTR (channel key) |
| MeshCore | 869.4 MHz | Unencrypted |

**Features:**
- Automatic channel hopping between protocols
- Decrypts Meshtastic packets if you have the channel key
- Default public channel (AQ==) supported out of the box
- Import custom channels from Meshtastic share URLs
- External peers shown with "M" badge on map

**Enable via:** LoRa Settings → Scanner → Meshtastic / MeshCore / Auto

## Architecture

```
┌─────────────────┐         LoRa 868/915MHz        ┌─────────────────┐
│   Azimuth #1    │◄──────────────────────────────►│   Azimuth #2    │
│   (ESP32-S3)    │         AES-128 encrypted      │   (ESP32-S3)    │
└────────┬────────┘                                └────────┬────────┘
         │ BLE / WiFi                                       │ BLE / WiFi
         ▼                                                  ▼
┌─────────────────┐                                ┌─────────────────┐
│    Web App      │                                │    Web App      │
│   (Browser)     │                                │   (Browser)     │
└─────────────────┘                                └─────────────────┘
         │
         │ Scanner (passive receive)
         ▼
┌─────────────────┐        ┌─────────────────┐
│   Meshtastic    │        │    MeshCore     │
│    Devices      │        │    Devices      │
└─────────────────┘        └─────────────────┘
```

**No server required.** Devices communicate directly over LoRa. The web app connects to your device via Bluetooth or WiFi — all data stays local.

## LoRa Protocol

Azimuth uses a custom encrypted packet format:

| Magic Byte | Purpose |
|------------|---------|
| `0xA9` | Position broadcast |
| `0xAB` | Broadcast message |
| `0xAC` | Direct message |
| `0xAE` | SOS alert |
| `0xB7` | Group key share |

All position and message packets are AES-128 CTR encrypted. Devices sharing the same key can see each other; others cannot.

### Relay Mode
Enable relay mode to extend mesh range. Relaying devices will rebroadcast packets they receive (with deduplication to prevent loops).

## Project Structure

```
azimuth/
├── firmware/
│   ├── gps_tracker_v319.ino    # Main firmware
│   ├── meshtastic_scanner.cpp  # Multi-protocol scanner
│   ├── meshtastic_scanner.h    # Scanner header
│   └── webapp_gz.h             # Embedded web app (gzip)
├── webapp/
│   ├── index.html              # Web app source
│   ├── sw.js                   # Service worker (offline maps)
│   └── manifest.json           # PWA manifest
└── docs/
    ├── features.html           # Feature documentation
    ├── guide.html              # User guide
    └── brand.html              # Brand assets
```

## Building the Embedded Web App

The web app is gzip-compressed and embedded in the firmware. After editing `webapp/index.html`:

```powershell
# Windows (PowerShell)
cd firmware
.\make_webapp.ps1
# Then recompile in Arduino IDE
```

## Regional Settings

| Region | Frequency | Power | Notes |
|--------|-----------|-------|-------|
| EU868 | 868.1 MHz | 14 dBm | EU regulatory limit |
| US915 | 910.5 MHz | 22 dBm | FCC compliant |
| AU915 | 916.8 MHz | 22 dBm | ACMA compliant |

## Version History

| Version | Date | Highlights |
|---------|------|------------|
| v3.19 | Apr 2026 | Multi-protocol scanner (Meshtastic/MeshCore), radar heading, UI improvements |
| v3.18 | Apr 2026 | Route viewer, Track Log redesign, BLE route transfer, GPX export |
| v3.17 | Mar 2026 | Weather integration, countdown timer, stopwatch |
| v3.16 | Feb 2026 | Groups system, key sharing, relay mode |

## Contributing

Contributions welcome! Areas where help is needed:
- Native iOS app (Web Bluetooth not supported on iOS)
- Additional LoRa regions (scanner frequencies for US915/AU915)
- Battery optimization
- UI/UX improvements
- Additional mesh protocol support

## License

MIT — use it, modify it, sell it, whatever. Just don't blame me if you get lost.

## Links

- 🌐 **Website:** [oeo.dev](https://oeo.dev)
- 🚀 **Web App:** [azimuth.oeo.dev](https://azimuth.oeo.dev)
- ⚡ **Web Flasher:** [oeo.dev/flash](https://oeo.dev/flash)
- 📖 **User Guide:** [oeo.dev/guide](https://oeo.dev/guide)
- 📋 **Features:** [oeo.dev/features](https://oeo.dev/features)

---

*Built by [OEO Technologies](https://oeo.dev) in the UK.*
