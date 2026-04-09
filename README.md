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
- 🌐 **Web app** — connect via BLE or WiFi hotspot, works offline with cached maps
- 🖥️ **OLED display** — 14 pages: position, compass, track recording, waypoints, weather, and more
- ⚡ **Web flasher** — update firmware from your browser, no IDE needed

## Hardware

| Component | Spec |
|-----------|------|
| Board | [Heltec WiFi LoRa 32 V3](https://heltec.org/project/wifi-lora-32-v3/) + GPS module |
| MCU | ESP32-S3 dual-core @ 240MHz |
| LoRa | SX1262 (EU868 / US915 / AU915) |
| Display | 128×64 OLED |
| GPS | AT6558R |
| Battery | 3.7V LiPo (JST connector) |

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
3. Install libraries: `RadioLib`, `NimBLE-Arduino`, `Adafruit SSD1306`, `TinyGPSPlus`
4. Open `firmware/gps_tracker_v316.ino`
5. Select board: **WiFi LoRa 32(V3)**, USB CDC On Boot: **Enabled**
6. Upload

## Web App

The companion web app runs entirely in your browser:

- **Online:** [azimuth.oeo.dev](https://azimuth.oeo.dev) — connect via Bluetooth
- **Offline:** Connect to device WiFi hotspot (`Azimuth-{name}`, password: `azimuth1`)

Features:
- Live map with your position and all peers
- Route recording and GPX export
- Waypoint management
- Offline map caching (download tiles for use without internet)
- Direct and broadcast messaging

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
│   ├── gps_tracker_v316.ino    # Main firmware
│   └── webapp_gz.h             # Embedded web app (gzip)
└── webapp/
    ├── index.html              # Web app source
    ├── sw.js                   # Service worker (offline maps)
    └── manifest.json           # PWA manifest
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

## Contributing

Contributions welcome! Areas where help is needed:
- Native iOS app (Web Bluetooth not supported on iOS)
- Additional LoRa regions
- Battery optimization
- UI/UX improvements

## License

MIT — use it, modify it, sell it, whatever. Just don't blame me if you get lost.

## Links

- 🌐 **Website:** [oeo.dev](https://oeo.dev)
- 🚀 **Web App:** [azimuth.oeo.dev](https://azimuth.oeo.dev)
- ⚡ **Web Flasher:** [oeo.dev/flash](https://oeo.dev/flash)
- 📖 **User Guide:** [oeo.dev/guide](https://oeo.dev/guide)

---

*Built by [OEO Technologies](https://oeo.dev) in the UK.*
