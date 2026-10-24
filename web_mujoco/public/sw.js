const CACHE_NAME = 'rebot-mujoco-runtime-v3';

self.addEventListener('install', () => {
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    Promise.all([
      self.clients.claim(),
      caches.keys().then((names) =>
        Promise.all(names.filter((name) => name.startsWith('rebot-mujoco-') && name !== CACHE_NAME).map((name) => caches.delete(name)))
      )
    ])
  );
});

self.addEventListener('message', (event) => {
  if (event.data?.type !== 'CACHE_APP_SHELL' || !Array.isArray(event.data.urls)) return;
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) =>
      Promise.allSettled(
        [...new Set(event.data.urls)].map(async (value) => {
          const url = new URL(value);
          const scope = new URL(self.registration.scope);
          if (url.origin !== scope.origin || !url.pathname.startsWith(scope.pathname)) return;
          const request = new Request(url.href, { credentials: 'same-origin' });
          if (await cache.match(request)) return;
          const response = await fetch(request, { cache: 'force-cache' });
          if (response.ok) await cache.put(request, response);
        })
      )
    )
  );
});

function isCacheable(requestUrl) {
  const scope = new URL(self.registration.scope);
  if (requestUrl.origin !== scope.origin || !requestUrl.pathname.startsWith(scope.pathname)) return false;
  return requestUrl.pathname.includes('/assets/') || requestUrl.pathname.includes('/models/') || requestUrl.pathname.includes('/environments/');
}

async function pruneOldModelFile(cache, url) {
  // Each file has its own content hash. Updating XML must not evict meshes.
  if ((!url.pathname.includes('/models/') && !url.pathname.includes('/environments/')) || !url.searchParams.has('v')) return;
  const sourcePath=value=>value.replace(/(?:\.stlp1)?\.gzbin$/,'');
  const requests = await cache.keys();
  await Promise.all(requests.map((request) => {
    const cached = new URL(request.url);
    if (cached.origin !== url.origin) return false;
    const outdated=sourcePath(cached.pathname)===sourcePath(url.pathname)&&cached.href!==url.href;
    return outdated ? cache.delete(request) : false;
  }));
}

async function cacheFirst(request, event) {
  const cache = await caches.open(CACHE_NAME);
  const url = new URL(request.url);
  const cached = await cache.match(request);
  if (cached) return cached;
  const response = await fetch(request);
  if (response.ok) {
    event.waitUntil(
      cache.put(request, response.clone())
        .then(() => pruneOldModelFile(cache, url))
        .catch(() => false)
    );
  }
  return response;
}

async function networkFirstPage(request, event) {
  const cache = await caches.open(CACHE_NAME);
  const fallbackKey = new Request(self.registration.scope);
  try {
    const response = await fetch(request);
    if (response.ok) {
      event.waitUntil(cache.put(fallbackKey, response.clone()).catch(() => false));
    }
    return response;
  } catch (error) {
    return (await cache.match(fallbackKey)) || Promise.reject(error);
  }
}

self.addEventListener('fetch', (event) => {
  if (event.request.method !== 'GET') return;
  const url = new URL(event.request.url);
  if (event.request.mode === 'navigate' && url.origin === self.location.origin) {
    event.respondWith(networkFirstPage(event.request, event));
    return;
  }
  if (isCacheable(url)) event.respondWith(cacheFirst(event.request, event));
});
