# Changelog

All notable changes to the Azimuth GPS Tracker firmware.

## v3.16 — 2026-04-09

### New Features

**Groups**
- Create private networks with custom names and auto-generated 16-character encryption keys
- Switch between Personal mode and group modes instantly
- Key sharing via LoRa — request sends to all peers, they tap to accept
- Up to 5 stored groups per device
- Leader/member roles with key rotation on group leave

**Relay Mode**
- Turn any device into a mesh range extender
- Relays position broadcasts, SOS alerts, and group messages
- Deduplication via packet hashing (30s window)
- Store-and-forward queues messages for offline peers (max 8, 1hr expiry)
- Enable via Settings → Relay on device or web app

**Mesh Status Page**
- New `/status` endpoint — standalone HTML page showing device info, peers, signal strength
- Auto-refreshes every 5 seconds
- Works offline, no external dependencies

**Config Portal**
- New `/config` endpoint — full device configuration via browser
- Settings: name, units, timeout, theme, LoRa region, interval, relay mode
- No BLE required — useful for initial setup

**WiFi QR Code**
- After saving settings with hotspot enabled, device displays QR code
- Scan with phone camera to auto-join device WiFi
- Screen stays on until button press dismisses

**Lockdown Mode**
- Settings → Lock blocks all remote configuration (BLE/WiFi)
- SOS and messages still work when locked
- Unlock via device screen only

### Fixes
- Radar now displays correctly in WiFi AP mode (Leaflet fallback)
- About screen updated to oeo.dev/azimuth

### Web App
- Groups panel with create/edit/delete/share
- Relay toggle and stats in settings
- Radar fix for device hotspot mode

---

## v3.09 — 2026-04-04

### Features
- WiFi AP UI for device configuration
- Offline map tile caching
- Peers JSON fix for iOS limitation workaround

---

## v3.07 — 2026-03-29

### Features
- Initial public release
- GPS tracking with OLED display
- LoRa peer position sharing
- AES-128 encryption
- Web Bluetooth companion app
- Route recording
- Waypoint navigation
- Weather integration
- Compass, timers, device info pages
