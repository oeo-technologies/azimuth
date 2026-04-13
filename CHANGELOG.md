# Azimuth GPS Tracker — v3.18 Release Notes

**Release Date:** April 2026

## Overview

Version 3.18 brings comprehensive route management features to both the device and web app, including a redesigned Track Log interface, BLE route transfer, and a full-featured route viewer with charts and GPX export.

---

## New Features

### Device — Track Log Page Redesign

- **Scrollable route list** with summary row showing total tracks and combined distance
- **Individual route display** showing route number, name, and distance
- **Navigation**: Double-press to cycle through routes, single-press to change pages
- **Individual route deletion** with confirmation dialog showing route name and stats
- **Clear all** option from summary row with confirmation
- **Scroll indicator** (down arrow) appears when more routes exist below current view
- **Route naming** — names stored in flash and persist across reboots

### Device — Compass Page Fix

- Fixed compass rose positioning so the "N" label no longer overlaps the header

### Web App — Route Viewer Panel

- **Route list** with icons showing date, time, duration, elevation gain, and point count
- **Distance display** handles large values (km with appropriate decimal places)
- **Elevation formatting** handles high altitudes (10,000m+ displayed as km)
- **Route detail view** with:
  - Editable route name (click pencil icon)
  - Stats grid: Distance, Elevation, Avg Speed, Max Speed
  - Interactive map showing route polyline with start/end markers
  - Chart tabs: Elevation / Speed / Pace profiles
  - X-axis toggle: Distance or Time
  - Chart style toggle: Line or Bar
- **GPX Export** — download individual routes as GPX files
- **Delete route** with double confirmation dialog
- **Theme-aware icons** — colors adapt to Forest/Canyon/Mountain themes

### BLE Route Transfer

- **Polling protocol** for reliable transfer (replaces notification-based approach)
- Handles Chrome's BLE notification queue limits
- Progress indicator in menu during import
- Route metadata (name, distance, start time) transferred with points
- Commands: `GETLOG`, `GETROUTE:N`, `NEXT`, `RENAME:idx:name`

### Web App — Route Panel Features

- **Bottom elevation panel** syncs with route chart (title and color match selected tab)
- **Map interaction** — map remains interactive while viewing route details
- Panel overlay is transparent; routes can be panned/zoomed
- **Back navigation** clears route from map and returns to list

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/routes` | GET | List all saved routes with metadata |
| `/api/route?idx=N` | GET | Get full route data (points array) |
| `/api/route/rename` | POST | Rename a route (body: `idx=N&name=NewName`) |
| `/api/route/delete` | POST | Delete a route (body: `idx=N`) |

---

## Device Button Controls — Track Log Page

| Action | Function |
|--------|----------|
| Single press | Go to next page |
| Double press | Cycle through routes |
| Long press (summary selected) | Clear all routes |
| Long press (route selected) | Delete selected route |

---

## File Changes

- `gps_tracker_v318.ino` — Firmware with all new features
- `index.html` — Updated web app with route viewer

---

## Upgrade Notes

1. Flash new firmware to device
2. Run `make_webapp.ps1` to regenerate `webapp_gz.h` from updated `index.html`
3. Existing saved routes will be preserved (stored in FFat)
4. Route names default to empty; can be set via web app or device

---

## Known Limitations

- Maximum 10 routes stored on device (ROUTE_LOG_MAX)
- Route names limited to 32 characters
- BLE transfer speed limited by polling protocol (~1-2 seconds per route depending on size)

---

## Bug Fixes

- Fixed compass "N" label being cut off at top of screen
- Fixed BLE notification drops on large route transfers
- Fixed elevation panel showing when no route selected
- Fixed `bleConnected` undefined error in route name save
