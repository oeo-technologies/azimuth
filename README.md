# Azimuth GPS Tracker

Open-source GPS tracker for outdoor adventures. Built on the Heltec WiFi LoRa 32 V3 (ESP32-S3).

Real-time peer position sharing over LoRa, AES-128 encrypted. Browser-based companion app via Web Bluetooth. No cloud, no subscriptions, no compromises.

→ [azimuth.oeo.dev](https://azimuth.oeo.dev) — Web app (Chrome + Android)  
→ [oeo.dev](https://oeo.dev) — Project website

## Features

**Device**
- GPS tracking with speed, heading, altitude
- LoRa mesh networking (868MHz EU / 915MHz US)
- AES-128 encryption for all communications
- OLED display with 14 pages (position, compass, messages, weather, timers...)
- Route recording with GPX export
- Waypoint navigation with bearing/distance
- Weather from OpenWeatherMap
- WiFi hotspot mode with config portal

**Groups & Messaging**
- Private groups with auto-generated encryption keys
- Key sharing via LoRa tap-to-accept
- Broadcast and direct messaging
- SOS alerts

**Relay Mode**
- Turn any device into a mesh range extender
- Store-and-forward for offline peers
- Automatic deduplication

**Web App**
- Real-time map with peer positions
- Offline map tile caching
- Full device configuration
- Route management and GPX export
- Works on device hotspot (no internet required)

## Repository Structure

```
firmware/         Arduino sketch + webapp header
webapp/           Web Bluetooth companion app + PWA files
```

## Hardware

| Component | Spec |
|-----------|------|
| Board | Heltec WiFi LoRa 32 V3 + GPS |
| MCU | ESP32-S3 dual-core @ 240MHz |
| LoRa | SX1262 — 868MHz (EU) / 915MHz (US/AU) |
| GPS | AT6558R |
| Display | SSD1306 0.96″ OLED 128×64 |
| Charging | USB-C + LiPo connector |

## Firmware

**Current version: v3.16**

### Flash Pre-built Binary (Recommended)

Visit [oeo.dev/flash](https://oeo.dev/flash) to flash the latest firmware directly from your browser — no Arduino IDE required.

### Arduino IDE Setup

1. Install Arduino IDE 2.x
2. Add board package URL in **File → Preferences → Additional Board Manager URLs**:
   ```
   https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.9/package_heltec_esp32_index.json
   ```
3. Install **Heltec ESP32 Dev-Boards** (3.0.3) via Board Manager
4. Install libraries via Library Manager:
   - RadioLib 6.6.0
   - NimBLE-Arduino 1.4.2
   - Adafruit SSD1306
   - Adafruit GFX
   - TinyGPSPlus
5. Board settings:
   - Board: WiFi LoRa 32(V3)
   - USB CDC On Boot: Enabled
   - Flash Size: 8MB

## Web App

The companion app runs in Chrome (desktop + Android). No installation required.

Open [azimuth.oeo.dev](https://azimuth.oeo.dev), tap Connect, enter the 6-digit PIN shown on your device.

**Browser support:** Chrome desktop, Chrome Android, Edge.  
**Not supported:** Safari, Firefox, iOS (Web Bluetooth limitation).

## License

MIT — see [LICENSE](LICENSE)

© 2026 OEO Technologies
