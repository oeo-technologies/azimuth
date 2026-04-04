const CACHE = 'azimuth-v3';
const TILE_CACHE = 'azimuth-tiles-v1';

const PRECACHE = [
  './gps_tracker_responsive.html',
  'https://fonts.googleapis.com/css2?family=DM+Sans:wght@400;500;600&family=DM+Serif+Display&display=swap',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
];

// Tile hosts we want to cache
const TILE_HOSTS = [
  'tile.openstreetmap.org',
  'tile.cyclosm.org',
  'basemaps.cartocdn.com',
  'tiles.stadiamaps.com',
];

function isTileRequest(url) {
  return TILE_HOSTS.some(h => url.hostname.includes(h));
}

// Install — cache core assets
self.addEventListener('install', e => {
  e.waitUntil(
    caches.open(CACHE).then(cache => cache.addAll(PRECACHE)).then(() => self.skipWaiting())
  );
});

// Activate — clean old caches
self.addEventListener('activate', e => {
  e.waitUntil(
    caches.keys().then(keys =>
      Promise.all(keys.filter(k => k !== CACHE && k !== TILE_CACHE).map(k => caches.delete(k)))
    ).then(() => self.clients.claim())
  );
});

// Fetch
self.addEventListener('fetch', e => {
  const url = new URL(e.request.url);

  // Map tiles — cache-first (serve offline if cached, otherwise network)
  if (isTileRequest(url)) {
    e.respondWith(
      caches.open(TILE_CACHE).then(async cache => {
        const cached = await cache.match(e.request);
        if (cached) return cached;

        // Over-zoom fallback — if z>14 and offline, try serving z14 tile
        const parts = url.pathname.match(/\/(\d+)\/(\d+)\/(\d+)\.png$/);
        if (parts) {
          const z = parseInt(parts[1]), x = parseInt(parts[2]), y = parseInt(parts[3]);
          if (z > 14) {
            const scale = 2 ** (z - 14);
            const z14x = Math.floor(x / scale);
            const z14y = Math.floor(y / scale);
            const z14url = url.origin + url.pathname.replace(`/${z}/${x}/${y}`, `/14/${z14x}/${z14y}`);
            const fallback = await cache.match(z14url);
            if (fallback) return fallback;
          }
        }

        return fetch(e.request).then(response => {
          if (response && response.status === 200) {
            cache.put(e.request, response.clone());
          }
          return response;
        }).catch(() => null);
      })
    );
    return;
  }

  // Skip non-cacheable requests
  if (
    url.protocol === 'blob:' ||
    url.hostname.includes('openweathermap') ||
    url.hostname.includes('sunrise-sunset') ||
    url.hostname.includes('ngdc.noaa') ||
    url.hostname.includes('ip-api') ||
    url.hostname.includes('beacondb')
  ) {
    return;
  }

  // Cache-first for app shell (fonts, leaflet, html)
  e.respondWith(
    caches.match(e.request).then(cached => {
      if (cached) return cached;
      return fetch(e.request).then(response => {
        if (!response || response.status !== 200 || response.type === 'opaque') return response;
        const clone = response.clone();
        caches.open(CACHE).then(cache => cache.put(e.request, clone));
        return response;
      }).catch(() => caches.match('./gps_tracker_responsive.html'));
    })
  );
});

// Message from app — download tiles for a region
self.addEventListener('message', e => {
  if (e.data && e.data.type === 'DOWNLOAD_TILES') {
    const { tiles, tileUrlTemplate } = e.data;
    downloadTiles(tiles, tileUrlTemplate, e.source);
  }
  if (e.data && e.data.type === 'CLEAR_TILES') {
    caches.delete(TILE_CACHE).then(() => {
      e.source.postMessage({ type: 'TILES_CLEARED' });
    });
  }
  if (e.data && e.data.type === 'TILE_CACHE_SIZE') {
    getTileCacheSize().then(size => {
      e.source.postMessage({ type: 'TILE_CACHE_SIZE_RESULT', size });
    });
  }
});

async function downloadTiles(tiles, tileUrlTemplate, client) {
  const cache = await caches.open(TILE_CACHE);
  let done = 0;
  let failed = 0;
  const total = tiles.length;

  // Process in batches of 6 to avoid overwhelming the network
  const BATCH = 6;
  for (let i = 0; i < tiles.length; i += BATCH) {
    const batch = tiles.slice(i, i + BATCH);
    await Promise.all(batch.map(async ({ z, x, y }) => {
      const url = tileUrlTemplate
        .replace('{z}', z).replace('{x}', x).replace('{y}', y)
        .replace('{s}', ['a','b','c'][Math.floor(Math.random()*3)]);
      try {
        const existing = await cache.match(url);
        if (!existing) {
          const resp = await fetch(url, { mode: 'cors' });
          if (resp && resp.status === 200) {
            await cache.put(url, resp);
          } else { failed++; }
        }
        done++;
      } catch { failed++; done++; }
      // Progress update every 10 tiles
      if (done % 10 === 0 || done === total) {
        client.postMessage({ type: 'TILE_PROGRESS', done, total, failed });
      }
    }));
  }
  client.postMessage({ type: 'TILE_DOWNLOAD_COMPLETE', done, total, failed });
}

async function getTileCacheSize() {
  const cache = await caches.open(TILE_CACHE);
  const keys = await cache.keys();
  let bytes = 0;
  for (const req of keys) {
    try {
      const resp = await cache.match(req);
      const buf = await resp.arrayBuffer();
      bytes += buf.byteLength;
    } catch {}
  }
  return { bytes, count: keys.length };
}
