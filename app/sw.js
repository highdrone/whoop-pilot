// Minimal service worker so Chrome offers "Install Whoop Pilot" (runs as its own app window).
// Always goes to the network: the app is served from this Mac, so there is nothing to cache. Page loads are left
// alone, so Chrome sends its own navigation headers (tools/whoop.mjs lets only those open the app from elsewhere).
self.addEventListener("install", () => self.skipWaiting());
self.addEventListener("activate", (e) => e.waitUntil(self.clients.claim()));
self.addEventListener("fetch", (e) => {
  if (e.request.mode !== "navigate") e.respondWith(fetch(e.request).catch(() => fetch(e.request)));
});
