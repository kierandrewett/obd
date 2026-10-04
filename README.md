# OBD Dashboard

A real-time OBD-II diagnostic dashboard for ELM-compatible and J2534 adapters, built in Rust with [egui](https://github.com/emilk/egui). Runs as a native desktop app and as a web app (WASM) accessible from any browser.

## Features

- **Auto-detection** - Automatically finds your ELM327 adapter (USB, Bluetooth, serial) and negotiates baud rate and OBD protocol
- **Live gauges** - Radial gauges for RPM, speed, coolant temp, oil temp, throttle, engine load with color-coded warning/danger thresholds
- **90 Mode 01 PID definitions** - Temperatures, pressures, fuel trims, O2 voltage/current, torque, pedal, evaporative-system data, odometer, and more; the ECU's support bitmap determines which readings are available
- **Freematics USB telemetry** - Passively displays the firmware's current acquisition stream in the same sensor view, preserving per-reading age and support status without starting a second ECU polling loop
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

## Supported connections

| Connection | Desktop | Browser | Current diagnostic scope |
|---|---|---|---|
| ELM-compatible USB / serial | Yes | Web Serial | Standard OBD using the adapter's implemented protocols |
| Corsa D MS-CAN ELM profile | Yes, experimental | No | Standard OBD requests over User Protocol B; no body-module scan |
| ELM-compatible Bluetooth serial | OS serial port required | Depends on browser/OS exposure | Same ELM diagnostic path |
| ELM-compatible Wi-Fi / TCP | Yes, explicit host and port | Not directly | Same ELM diagnostic path |
| J2534 04.04 vendor driver | Yes, matching native library required | Not directly | Standard ISO 15765 CAN at 250/500 kbit/s |
| Legacy PSA VCI / Lexia / DiagBox-specific library | Not implemented | No | Requires a separate backend unless the device supplies a compatible J2534 driver |

The desktop connection selector is available above the dashboard and on the disconnected screen.
Serial connections retain automatic port/baud detection. TCP connections need the adapter's documented
host and port. Branded ELM-compatible identities such as OBDLink, STN, ELS and vLinker are accepted;
initialisation also requires successful commands and a valid vehicle response.

The optional **Corsa D MS-CAN** serial profile is separate from automatic HS-CAN OBD. It configures
ELM User Protocol B at the Corsa D's reported approximately 95.2 kbit/s, then tries the standard
`0100` supported-PID request. Some MS-CAN segments may have no generic OBD responder; in that case
the UI explicitly reports that no generic responder was found and does not fall back to HS-CAN.
This profile does not passively monitor raw frames or discover/read Opel body modules. Select the
adapter's MS position yourself; the app cannot switch the adapter's physical pins.

### J2534 setup

1. Install the interface manufacturer's J2534 **04.04** driver.
2. Select **J2534 pass-through**, then choose an installed driver or enter its absolute library path.
3. Select the vehicle's CAN identifier format and rate. This backend does not automatically search protocols.
4. For 29-bit CAN, set the ECU source address in hexadecimal. The default is `10`; check the vehicle
   documentation. This connects to one selected ECU. The 11-bit connection accepts the standard
   OBD response IDs `7E8` through `7EF`.
5. Connect with the ignition on. The app requires a valid supported-PID response before reporting a connection.

Windows driver discovery reads the registry view matching the app's architecture. A 32-bit driver needs
an i686 app build; a 64-bit driver needs an x86_64 app build. A 05.00-only driver is not supported.
The loader does not convert between architectures. No vendor drivers or OEM subscriptions are bundled.
Only load a trusted manufacturer library: loading a native library executes its code in the application.

The backend performs ISO-TP through the vendor driver, sets flow-control filters, retains response CAN IDs,
ignores transmit/start indications and waits for pending ECU responses within the request deadline.
The existing gauges, VIN, stored/pending DTCs and freeze-frame operations use this shared diagnostic path.
The current screens still present standard OBD data rather than an inventory of individual modules.

### Coverage limits

Connection support does **not** establish compatibility with every vehicle or module. Manufacturer fault-code
JSON files provide descriptions, not module access. Manufacturer-specific module discovery, manual/automatic
bus switching, custom pin routing, legacy J2534 protocols, CAN FD, DoIP and security-gateway authentication
are not implemented by the new J2534 backend. A capable interface does not add those application features.
The existing corporate-family description fallback also remains; it is not a verified ECU-specific definition.

Serial and TCP exchanges are tested against local simulators. J2534 is tested through a compiled native
C library exercising the real 04.04 loader, addressing, filters, VIN, DTCs and error cleanup. No physical
Ford/PSA/Opel/FCA adapter or vehicle has been validated for these new connections yet.

See [adapter implementation notes](docs/adapters.md) and [remaining work](TODO.md).

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
  adapter.rs           Shared diagnostic payload interface and ELM response normalisation
  elm327.rs            ELM327 serial driver: auto-detect, send/receive
  elm_tcp.rs           Native TCP transport for ELM-compatible adapters
  j2534.rs             Native J2534 04.04 loader and ISO 15765 diagnostics
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
