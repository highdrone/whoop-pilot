#!/usr/bin/env python3
"""Simulator-only fallback for when Node.js is missing: serves Whoop Pilot on http://localhost:8790/app/
(localhost only) with the Python that ships with macOS. No goggles video, recorder or capture import:
those need tools/whoop.mjs. Port: first argument or $PORT. GET /whoop-pilot.json identifies us, so
start.command and whoop.mjs can tell us from other servers."""
import http.server
import os
import sys

PORT = int(sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PORT", "8790"))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".mjs": "text/javascript",
        ".json": "application/json",
        ".svg": "image/svg+xml",
        ".lua": "text/plain; charset=utf-8",
        ".md": "text/markdown; charset=utf-8",
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def do_GET(self):
        if self.path == "/whoop-pilot.json":
            body = b'{"app":"whoop-pilot","server":"python"}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", "/app/")
            self.end_headers()
            return
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


class Server(http.server.ThreadingHTTPServer):
    request_queue_size = 128  # the app loads ~30 modules at once; the default backlog of 5 drops some
    daemon_threads = True


if __name__ == "__main__":
    try:
        server = Server(("127.0.0.1", PORT), Handler)
    except OSError:
        sys.exit(f"Port {PORT} is already in use by another program.")
    print(f"Whoop Pilot running at http://localhost:{PORT}/app/  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        sys.exit(0)
