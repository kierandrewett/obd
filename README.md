# OBD Dashboard

A real-time OBD-II diagnostic dashboard for ELM327 adapters, built in Rust with [egui](https://github.com/emilk/egui). Runs as a native desktop app and as a web app (WASM) accessible from any browser.

## Features

- **Auto-detection** - Automatically finds your ELM327 adapter (USB, Bluetooth, serial) and negotiates baud rate and OBD protocol
- **Live gauges** - Radial gauges for RPM, speed, coolant temp, oil temp, throttle, engine load with color-coded warning/danger thresholds
- **60+ PIDs** - Supports all standard Mode 01 PIDs: temperatures, pressures, fuel trims, O2 sensors, catalyst temps, and more
- **Bar gauges + sparklines** - Secondary sensors shown as bar gauges, with trend sparklines for key values
- **DTC reading** - Read stored and pending diagnostic trouble codes. Codes appear instantly; descriptions are looked up in the background from a database of 21,000+ manufacturer-specific codes across 37 makes
- **Smart DTC descriptions** - Descriptions are sourced in priority order: manufacturer-specific → corporate family alias (e.g. Opel → GM) → SAE J2012 generic. Source attribution is shown in the table so you know where each description came from
- **DTC clearing** - Clear trouble codes and reset the MIL (Check Engine Light)
- **Freeze frame** - Read Mode 02 freeze frame data captured when a DTC was triggered
- **VIN decoding** - Reads and decodes your Vehicle Identification Number on connect, showing make, country, and model year in the header bar
- **Configurable polling** - Three poll modes (Minimal/Fast/Full) with adjustable cycle delay for tuning refresh rate vs. bus load
- **Web Serial support** - Browser-based version uses the Web Serial API (Chrome/Edge) to connect directly to an ELM327 over USB
- **Structured debug log** - All OBD messages, value changes, and events logged to both a bottom panel and `obd-debug.log` with timestamps
- **Screen wake lock** - Prevents screen sleep while polling is active (Linux, via `systemd-inhibit`)
- **Dark/Light theme** - Toggle between dark and light mode from the tab bar

## Supported Hardware

Any **ELM327**-compatible OBD-II adapter connected via:

- USB (e.g. `/dev/ttyUSB0`)
- Bluetooth serial (e.g. `/dev/rfcomm0`)
- Native serial (e.g. `/dev/ttyS0`, `COM3`)

Tested protocols: ISO 15765-4 CAN (11-bit and 29-bit, 500/250 kbaud), ISO 9141-2, ISO 14230-4 KWP, SAE J1850.

## Installation

### Pre-built binaries

Download the latest release for your platform from the [Releases](https://github.com/kierandrewett/obd/releases) page.

### Build from source

Requires [Rust](https://rustup.rs/) 1.85+.

```bash
git clone https://github.com/kierandrewett/obd.git
cd obd
cargo build --release
```

The binary will be at `target/release/obd-dashboard`.

#### Linux dependencies

On Debian/Ubuntu:

```bash
sudo apt install -y pkg-config libudev-dev libgtk-3-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libxkbcommon-dev
```

On Fedora:

```bash
sudo dnf install -y pkg-config systemd-devel gtk3-devel libxcb-devel libxkbcommon-devel
```

#### Serial port permissions

You may need permission to access the serial port:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for the group change to take effect
```

Or as a one-off:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

### Web app (WASM)

Requires [trunk](https://trunkrs.dev/) and the `wasm32-unknown-unknown` target:

```bash
rustup target add wasm32-unknown-unknown
cargo install trunk
trunk serve
```

Then open `http://localhost:8080` in Chrome or Edge and connect via Web Serial.

#### Docker (self-hosted web server)

```bash
docker build -t obd-dashboard .
docker run -p 8080:80 obd-dashboard
```

## Usage

```bash
# Run with auto-detection
cargo run --release

# Enable debug logging
RUST_LOG=debug cargo run --release

# Set a specific port via environment variable (used as default in the UI)
OBD_PORT=/dev/ttyUSB0 cargo run --release
```

### Tabs

| Tab | Description |
|-----|-------------|
| **Dashboard** | Live radial gauges, bar gauges, and sparkline trends |
| **Sensors** | Table view of all live PID values with raw hex data |
| **DTCs** | Read and clear stored/pending diagnostic trouble codes |
| **Freeze Frame** | Snapshot of sensor data from when a DTC was triggered |
| **Vehicle Info** | VIN, ELM327 version, protocol, supported PIDs |

### DTC descriptions

When you read DTCs, codes appear in the table immediately. Descriptions are then resolved in the background using a three-tier lookup:

1. **Manufacturer-specific** — exact match from the vehicle's make database
2. **Corporate family** — if no exact match, checks related manufacturers that share DTC code tables (e.g. an Opel code checks the GM family: Chevrolet → Oldsmobile → Saturn). The source column shows which make was used and the corporate relationship.
3. **SAE J2012** — generic standard description that applies to all makes

If a description comes from a related manufacturer rather than your vehicle's own make, the source column is explicit about this: it shows the make name, the corporate family, and a tooltip explaining why the codes may be shared.

Manufacturer-specific codes are fetched from [dot.report](https://dot.report/dtc/) using the script in `scripts/`. The database covers 37 manufacturers and 21,000+ codes.

### Log panel

The resizable bottom panel shows a structured debug log. Every OBD exchange is tagged:

```
2026-03-28 14:30:05.123 [CONNECTED] port=/dev/ttyUSB0 baud=38400 protocol=ISO 15765-4 CAN (11-bit, 500 kbaud)
2026-03-28 14:30:06.456 [VALUE_INIT] pid=010C name=Engine RPM value=750.00 unit=RPM raw=410C0BB8
2026-03-28 14:30:07.789 [VALUE_CHANGE] pid=010C name=Engine RPM prev=750.00 new=2100.00 unit=RPM raw=410C2100
2026-03-28 14:30:08.012 [DTC_STORED] code=P0300
```

This is also written to `obd-debug.log`, so you can pipe it to an LLM for analysis.

## Supported PIDs

<details>
<summary>Mode 01 - Live Data (click to expand)</summary>

| PID | Name | Unit |
|-----|------|------|
| 0104 | Engine Load | % |
| 0105 | Coolant Temperature | °C |
| 0106 | Short Term Fuel Trim Bank 1 | % |
| 0107 | Long Term Fuel Trim Bank 1 | % |
| 0108 | Short Term Fuel Trim Bank 2 | % |
| 0109 | Long Term Fuel Trim Bank 2 | % |
| 010A | Fuel Pressure | kPa |
| 010B | Intake Manifold Pressure | kPa |
| 010C | Engine RPM | RPM |
| 010D | Vehicle Speed | km/h |
| 010E | Timing Advance | ° |
| 010F | Intake Air Temperature | °C |
| 0110 | MAF Air Flow Rate | g/s |
| 0111 | Throttle Position | % |
| 0114–011B | O2 Sensor Voltages | V |
| 011F | Engine Run Time | s |
| 0121 | Distance with MIL On | km |
| 012C | Commanded EGR | % |
| 012D | EGR Error | % |
| 012E | Evaporative Purge | % |
| 012F | Fuel Level | % |
| 0131 | Distance Since DTC Clear | km |
| 0133 | Barometric Pressure | kPa |
| 013C–013F | Catalyst Temperatures | °C |
| 0142 | Control Module Voltage | V |
| 0144 | Commanded Equiv Ratio | λ |
| 0145–014B | Throttle/Accelerator Positions | % |
| 0146 | Ambient Air Temperature | °C |
| 0151 | Fuel Type | — |
| 0152 | Ethanol Fuel Percent | % |
| 015C | Engine Oil Temperature | °C |
| 015D | Fuel Injection Timing | ° |
| 015E | Engine Fuel Rate | L/h |

</details>

<details>
<summary>Other Modes</summary>

| Mode | Description |
|------|-------------|
| Mode 02 | Freeze Frame Data |
| Mode 03 | Stored Diagnostic Trouble Codes |
| Mode 04 | Clear DTCs and MIL |
| Mode 07 | Pending Diagnostic Trouble Codes |
| Mode 09 | Vehicle Information (VIN, Calibration ID) |

</details>

## Architecture

```
src/
  main.rs              Entry point, logging, OBD worker thread, GUI launch
  app.rs               egui application: tabs, gauges, controls, log panel
  elm327.rs            ELM327 serial driver: auto-detect, init, send/receive
  obd.rs               OBD-II PID definitions, decoders, DTC parsing
  obd_ops.rs           Shared async OBD operations (used by native and WASM)
  gauges.rs            Custom egui widgets: radial gauges, bar gauges, sparklines
  vin_decoder.rs       VIN WMI lookup: 200+ manufacturers
  dtc_database.rs      Manufacturer DTC database loader with corporate alias groups
  dtc_descriptions.rs  Built-in SAE J2012 DTC descriptions
  web_serial.rs        WASM worker: Web Serial API and WebSocket emulator adapter
  lib.rs               WASM entry point

dtc_codes/             Per-make DTC description JSON files (37 makes, 21,000+ codes)
scripts/               fetch_dtc_codes.js — scraper for dot.report
```

The app runs two threads (native) or a single-threaded async loop (WASM):

1. **GUI thread** — egui rendering, user interaction
2. **OBD worker thread** — serial communication, PID polling, DTC reading
3. **Description thread** — spawned per DTC scan to enrich codes in the background without blocking the UI

Communication is via `mpsc` channels: commands flow GUI → worker, events flow worker → GUI.

## Updating the DTC database

```bash
cd scripts
npm install
node fetch_dtc_codes.js --concurrency 20
```

This incrementally updates `dtc_codes/` from dot.report. Already-scraped codes are skipped. Use `--headed` if Cloudflare blocks the headless browser.

## License

[MIT](LICENSE)
