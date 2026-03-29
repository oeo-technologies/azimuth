# Changelog

All notable changes to Azimuth firmware and web app are documented here.

---

## [v3.0.9] — 2026-03-29

### Firmware
- **Wayfinding mode** — shared waypoint system for group navigation
- Web app pushes up to 10 named waypoints (12 char names) via BLE (`SETWP` JSON command)
- Leader device broadcasts waypoints to all peers over LoRa (`0xAF` packets, 2 waypoints per packet)
- Broadcast repeats 3 times with 2 second gaps to maximise range reach
- Receiving devices send a `0xB0` ACK packet back to confirm receipt (random delay to avoid collisions)
- Leader device relays ACKs to web app via BLE notify `{"wpAck":"deviceId","wps":N}`
- Waypoints persist across reboots via FFat (`/waypoints.bin`)
- Navigate page extended — long press cycles through waypoints, shows name/distance/bearing and "N/M Hold:next"
- Navigate page falls back to saved pin behaviour when no waypoints are set
- `CLEARWP` BLE command clears all waypoints from device and flash storage
- Navigate page refresh rate increased to 500ms when waypoints are active

### Web app
- Waypoints panel in drawer — tap map to place or enter coordinates manually, up to 10 waypoints
- Numbered circle markers on map for each waypoint
- Send to group button — writes `SETWP` JSON over BLE
- ACK tracking — status area shows which peers have confirmed receipt e.g. `Received by: Runner1 ✓ · Runner2 ✓`
- Toast notification fires for each ACK received from a peer
- Clear waypoints button — clears local list, map markers, and sends `CLEARWP` to device
- Per-peer waypoint distances shown on peer cards e.g. `To WPs: 1 450m · 2 1.2km`

---

## [v3.0.8] — 2026-03-28

### Firmware
- **Route log persistence** — completed routes saved to FFat (`/route_N.bin`), survive reboots
- **Weather fix** — weather fetch now triggers on approximate GPS fix, no longer waits for full fix
- **Acquiring screen** — new map pin graphic with signal arcs shown while waiting for GPS fix
- **Splash screen** — Option B bearing mark design (Azimuth logo)
- Navigate page — retuned compass layout (cx=97, cy=32, cr=24), near-waypoint highlight box
- Power page — larger power symbol, tighter text layout
- Countdown page — all positions retuned
- Device Info page — BLE icon in header, battery row left-aligned, USB power text
- GPS page — BLE icon removed
- Double press window raised to 500ms

### Web app
- CyclOSM tile layer removed (403 referer errors) — default tile changed to CartoDB Voyager
- Zoom control moved to bottom-left, clear of stats overlay
- Map type button repositioned with hidden label (flashes 2s on tap)
- Locate button repositioned clear of map type button

---

## [v3.0.7] — 2026-03-27

### Firmware
- **SOS system** — long press on any page triggers SOS broadcast (`0xAE` packet) with GPS position
- **Message repeat** — unacknowledged messages retry up to 3 times
- **Peer limit increased** — supports up to 20 simultaneous LoRa peers (was 10)
- SOS cancel via short press during active SOS

### Web app
- SOS alert modal — persistent overlay with peer name and map pin on SOS receipt
- Stale peer warning — toast notification when a peer goes silent for 5 minutes

---

## [v3.0.6] — 2026-03-26

### Firmware
- **AES-128 encryption** — all LoRa packets encrypted with shared network key
- `SETLORAKEY` BLE command to set 16-char network key
- Key persisted to NVS

### Web app
- Network key management in settings drawer
- Peer sharing panel — distance lines, follow mode, locate button per peer
- Map auto-follow toggle

---

## [v3.0.5] — 2026-03-25

### Firmware
- **LoRa peer sharing** — devices broadcast position, name, battery, altitude, recording status
- Peer packets (`0xA9`) include encrypted payload with device name and status flags
- Peers expire after 5 minutes of silence

### Web app
- Peers panel — live list of nearby Azimuth devices with distance, bearing, battery
- Peer markers on map with orange dot indicators
- Distance line overlay between own position and selected peer

---

## [v3.0.0] — 2026-03-24

### Firmware
- Initial public release
- ESP32-S3 / Heltec WiFi LoRa 32 V4 support
- SSD1306 128×64 OLED display with 14-page UI
- AT6558R GPS with NMEA parsing
- BLE companion app connectivity with PIN authentication
- WiFi geolocation (BeaconDB) for approximate position without GPS fix
- LoRa 868MHz regional support (EU, US, AU, AS, IN)
- FFat filesystem for route recording
- NVS settings persistence
- RadioLib SX1262 driver

---

## Hardware

| Component | Part |
|-----------|------|
| MCU | ESP32-S3FN8 |
| LoRa | SX1262 |
| Display | SSD1306 128×64 OLED |
| GPS | AT6558R (L1) |
| Board | Heltec WiFi LoRa 32 V4 |

## Licence

MIT — see [LICENSE](LICENSE) for details.
Commercial use requires a separate licence agreement. See [oeo.dev](https://oeo.dev) for details.
