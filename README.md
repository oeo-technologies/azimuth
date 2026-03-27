# Azimuth GPS Tracker

Open-source GPS tracker for outdoor adventures. Built on the Heltec WiFi LoRa 32 V4 (ESP32-S3).

Real-time peer position sharing over LoRa, AES-128 encrypted. Browser-based companion app via Web Bluetooth. No cloud, no subscriptions, no compromises.

**→ [azimuth.oeo.dev](https://azimuth.oeo.dev)** — Web app (Chrome + Android)  
**→ [oeo.dev](https://oeo.dev)** — Project website

---

## Repository Structure

```
firmware/         Arduino sketch (gps_tracker_v307.ino)
webapp/           Web Bluetooth companion app + PWA files
website/          oeo.dev marketing/documentation site
docs/             Additional documentation
```

## Hardware

- **Board:** Heltec WiFi LoRa 32 V4
- **MCU:** ESP32-S3 dual-core @ 240MHz
- **LoRa:** SX1262 — 868MHz (EU) / 915MHz (US/AU)
- **GPS:** AT6558R
- **Display:** SSD1306 0.96″ OLED 128×64
- **Charging:** USB-C

## Firmware

Current version: **v3.0.7**

### Arduino IDE Setup

1. Install Arduino IDE 2.x
2. Add board package URL in File → Preferences → Additional Board Manager URLs:
   ```
   https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.9/package_heltec_esp32_index.json
   ```
3. Install **Heltec ESP32 Dev-Boards** (3.0.3) via Board Manager
4. Install libraries via Library Manager:
   - RadioLib 6.6.0
   - NimBLE-Arduino 1.4.2
   - Adafruit SSD1306
   - TinyGPSPlus
5. Board settings: WiFi LoRa 32(V4), USB CDC On Boot: Enabled, Flash Size: 16MB

### Flash Pre-built Binary

Visit [oeo.dev/flash](https://oeo.dev/flash.html) to flash the latest firmware directly from your browser — no Arduino IDE required.

## Web App

The companion app runs in Chrome (desktop + Android). No installation required.

Open [azimuth.oeo.dev](https://azimuth.oeo.dev), tap Connect, enter the on-screen PIN from your device.

**Browser support:** Chrome desktop, Chrome Android. Not supported on Safari, Firefox or iOS Chrome (Web Bluetooth limitation).

## License

MIT — see [LICENSE](LICENSE)

© 2026 OEO Technologies
