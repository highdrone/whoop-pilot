# Whoop Pilot

Experimental, supervised indoor FPV research software: a local Chrome interface, simulator, scan-based planning, optional Claude assistance, and an EdgeTX radio bridge.

**Not a flight-certified product. Start in the simulator. Bench-test with propellers removed. Keep a competent pilot on the sticks.** A passing software test or checklist does not establish that autonomous flight is safe. This publication did not test real flight, radio hardware, goggles hardware, browser GPU localization, or cloud API calls.

## Guide

- **[Download/read the 18-page Field Guide](docs/Whoop-Pilot-Guide.pdf)**
- [Accessible HTML guide](docs/guide/guide.html)
- [Hardware setup instructions](app/setup.html) — best opened from the local app.

The public guide retains the workflow and page layout, and retains the original screenshots and house maps with their owner’s explicit publication permission. Timings, historical cost estimates and example localization results are not independently validated performance claims. Verify current API pricing yourself; a software budget is not a billing guarantee.

## Quick start (macOS + Google Chrome)

Prerequisites: Node.js 22 or newer and npm. Python 3 is optional for simulator-only static serving. Download this repository or clone it, then:

```sh
cd whoop-pilot/tools
npm ci
cd ..
./start.command
```

The app opens at `http://localhost:8790/app/`. Keep its terminal window open. The normal Node server includes goggles support and access to your own capture folders; use the Python fallback below when you only want static simulator access, without any hardware bridge.

```sh
python3 tools/serve.py
# Open http://localhost:8790/app/ in Chrome; choose Simulator.
```

The built-in procedural world runs without your house or a Claude key. WebGPU-dependent features need a compatible Chrome/GPU. The small XFeat dependency and vendored browser runtimes are included under their own licenses; larger vision models download from pinned upstream URLs and are cached locally. First-time model preparation requires internet and substantial disk/memory.

## Your own scan

The guide and derived `tools/fixtures/house` scan geometry are included with the owner’s explicit permission. No raw private flight recording, personal radio backup or API key is distributed. Supply your own SiteSpec/Spacial capture using **Settings → House** and the folder/import controls. The Node server can list captures under `~/SiteSpec Projects` and `~/Spacial Projects`. Only use captures and images you have permission to process. Keep those folders and all recordings outside this Git checkout. The app's house-import, planning and simulator code is retained.

## Hardware scope

The source and guide target a BetaFPV Meteor65 Pro II / DJI O4, DJI Goggles 3, and RadioMaster TX15 using EdgeTX, ELRS and Betaflight ANGLE mode. This is a development target, not a verified compatibility certification. See the local setup guide before configuring hardware. `radio/SCRIPTS/MIXES/aibrg.lua` is the bridge source; **no personal radio/model settings are included**. Third-party device firmware and proprietary DJI/Betaflight configuration tools are not redistributed.

The bridge's manual axis takeover, AI switch, heartbeat timeout and ANGLE behavior are retained. The app does not arm a real drone. **Stop/Escape** ends the mission; **Land/L** requests landing. These are software controls, not substitutes for a pilot or an independent disarm procedure. Hardware must be checked props-off before any actual flight.

## Optional Claude and privacy

Without a key, the offline parser and simulator remain available. A user may enter their own Anthropic key in **Settings → Brain**. The key is stored in the browser origin's local storage and sent to Anthropic for authentication; do not use this on a shared/untrusted browser profile.

- AI voice/typed commands can send command text, drone state, memory context **and a camera frame**. Browser speech recognition may also use the browser vendor's speech service.
- Picture checks have a consent setting; house surveys send rendered views of your scan after their approval flow. Images may include people, pets or private interiors. Enabling AI is not local-only processing.
- Local history/house data use browser storage; the Node recorder stores video/telemetry/commands in `~/Library/Application Support/WhoopPilot`. Local storage is not encryption or backup.
- Vision model downloads contact upstream hosting. No cloud key is needed for procedural simulation.
- Never upload `.env`, browser storage, keys, recordings or scan exports in an issue/PR. Review recordings, metadata and screenshots before sharing.

## Tests (no drone / no paid API)

```sh
cd tools
npm ci --ignore-scripts
npm test
```

The default public suite uses mock Claude, mock EdgeTX/Lua, fake goggles and generated geometry. `npm run test:extended` also runs the larger scan-derived fixture suites and may take substantially longer. It covers agent behavior, change detection, radio manual takeover/failsafe, local server origin/token/path protections, procedural simulation, twin geometry and vision/cache mechanics. No test should use a real radio or cloud key. The scan-derived fixture exercises import/planning/localization logic offline. Tests requiring a withheld real H.264 camera recording are not distributed; real flight and live camera performance remain unverified.

`python3 tools/check-publication.py` checks the tracked/public source for common secret/PII patterns and rejects private dataset types. CI runs that check and the offline suite. Automated scans cannot guarantee absence of every possible secret or identifying detail.

## Project layout

- `app/`: browser application, safety/controller, planning, vision, import, UI, simulator.
- `tools/whoop.mjs`: loopback Node server, goggles adapter, recorder and capture import.
- `tools/serve.py`: simulator-only static fallback.
- `radio/SCRIPTS/MIXES/aibrg.lua`: EdgeTX mixer script.
- `docs/`: sanitized field guide; authorized original guide imagery and privacy-corrected text.

## License and contributions

Owner-authorized guide screenshots and scan-derived fixtures are intentionally public and must not be described as anonymous synthetic data.

Original project source and the public guide: [MIT](LICENSE). Vendored dependencies remain under their respective licenses; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and adjacent notices. No affiliation with or endorsement by DJI, BetaFPV, RadioMaster, Anthropic or other vendors is implied. See [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md).
