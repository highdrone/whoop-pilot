#!/bin/bash
# Double-click to start the goggles helper without opening the app. The helper is part of Whoop Pilot's
# server now (whoop.mjs, the same one start.command runs): it reads the DJI Goggles 3 live view over
# USB-C (or the goggles' Wi-Fi with --wifi) and the app on http://localhost:8790 connects by itself.
# Keep this window open. If the server is already running there is nothing to do. Needs Node.js.
# Diagnostics: node goggles.mjs probe | capture 10 (with the server stopped).
cd "$(dirname "$0")"
if ! command -v node >/dev/null; then
  echo "Node.js is missing. Install Homebrew (https://brew.sh), then: brew install node"
  read -n 1 -s -r -p "Press any key to close."
  exit 1
fi
if [ ! -d node_modules/usb ] && ! npm install --silent --no-audit --no-fund; then
  echo "npm install failed. Run it here by hand to see why:   cd \"$PWD\" && npm install"
  read -n 1 -s -r -p "Press any key to close."
  exit 1
fi
exec node whoop.mjs --no-open "$@"
