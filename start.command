#!/bin/bash
# Double-click to start Whoop Pilot: one local server (tools/whoop.mjs: the app, the goggles video, the
# flight recorder, your captures) on http://localhost:8790, then Chrome opens the app. Closing this window
# stops it. From a terminal, "./start.command --wifi" reads the goggles over their Wi-Fi instead of USB-C.
# Without Node.js it falls back to a simulator-only server (tools/serve.py, the Python that ships with macOS).
cd "$(dirname "$0")"
PORT=8790 # the app keeps its settings, API key and houses per address, so always this one
URL="http://localhost:$PORT/app/"

if command -v node >/dev/null; then
  if [ ! -d tools/node_modules/usb ] && ! (cd tools && npm install --silent --no-audit --no-fund); then
    echo "The goggles package didn't install, so there will be no goggles video. Try by hand, then run this again:"
    echo "   cd \"$PWD/tools\" && npm install"
  fi
  exec node tools/whoop.mjs "$@"
fi

echo "Node.js is missing, so Whoop Pilot runs as the simulator only: no goggles video, flight recorder or capture import."
echo "For the real drone: install Homebrew (https://brew.sh), then  brew install node  and run this again."
ours() { curl -s -m 1 "http://127.0.0.1:$PORT/whoop-pilot.json" | grep -q '"app":"whoop-pilot"'; }
python3 -c '' 2>/dev/null || { echo "Python 3 is missing too: run  xcode-select --install  (or brew install node), then run this again."; exit 1; }
open_app() { [ -n "$WHOOP_NO_OPEN" ] || open -a "Google Chrome" "$URL" 2>/dev/null || open -a "Microsoft Edge" "$URL" 2>/dev/null || open "$URL"; }
if ours; then
  echo "Whoop Pilot is already running: $URL"
  open_app
  exit 0
fi
if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "Port $PORT is used by another program. Whoop Pilot keeps your settings under $URL, so it can't use another port. Quit that program and run this again."
  exit 1
fi
python3 tools/serve.py "$PORT" &
PID=$!
trap 'kill $PID 2>/dev/null; exit' INT TERM HUP EXIT
for _ in 1 2 3 4 5 6 7 8 9 10; do ours && break; sleep 0.3; done
echo "Whoop Pilot (simulator only): $URL"
open_app
echo "Keep this window open. Close it (or press Ctrl+C) to stop the app."
wait
