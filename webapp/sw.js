const CACHE = 'azimuth-v1';

const PRECACHE = [
  './gps_tracker_responsive.html',
  'https://fonts.googleapis.com/css2?family=DM+Sans:wght@400;500;600&family=DM+Serif+Display&display=swap',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
];

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
      Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k)))
    ).then(() => self.clients.claim())
  );
});

// Fetch — cache-first for assets, network-first for map tiles
self.addEventListener('fetch', e => {
  const url = new URL(e.request.url);

  // Always network for BLE, API calls, map tiles
  if (
    url.protocol === 'blob:' ||
    url.hostname.includes('openstreetmap') ||
    url.hostname.includes('cartocdn') ||
    url.hostname.includes('cyclosm') ||
    url.hostname.includes('openweathermap') ||
    url.hostname.includes('sunrise-sunset') ||
    url.hostname.includes('ngdc.noaa') ||
    url.hostname.includes('ip-api')
  ) {
    return;
  }

  // Cache-first for everything else (fonts, leaflet, app shell)
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
