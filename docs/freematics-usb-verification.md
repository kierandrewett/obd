# Freematics USB dashboard verification

The dashboard's Freematics USB connection is a passive reader for the Model B
TeleLogger `@FT1` stream. Auto-detect opens only USB serial ports at 115200 baud
and accepts a device only after parsing a checksummed telemetry record. It sends
no ELM or diagnostic commands. USB serial control lines are preserved on open;
the app does not assert DTR or RTS. A serial monitor must not share the selected
port with the dashboard.

The repeatable parser-level serial traffic scenario is
`tests/fixtures/freematics_usb_scenario.txt`, exercised by
`freematics_usb::tests::simulated_serial_shudder_and_reconnect_scenario_is_repeatable`.
The test fragments input into 11-byte reads, includes debug output, one
checksum-corrupt frame, a warm-idle RPM dip (820 to 540 RPM), a Model B supply
dip (13.75 to 11.80 V), an RPM measurement age of 1.5 s (stale against its
250 ms target), per-frame voltage and motion waveform fields, a cumulative USB
drop count, and a changed boot ID. Expected result: four valid frames retained,
the corrupt frame rejected, the ECU capture UTC and per-measurement ages
preserved, voltage kept distinct from ECU control-module voltage, and restart
visible by boot ID.

Run with:

```sh
cargo test simulated_serial_shudder_and_reconnect_scenario_is_repeatable --lib
```

This validates parser behavior against simulated traffic only. Hardware
auto-detection, live sampling cadence, upload continuity, actual acquisition
ages, and record-after-disconnect behavior still require a flashed Model B
connected to the car and laptop.

## Laptop flash and serial smoke test (2026-10-03)

- The connected adapter identified as ESP32-D0WDQ6 revision 1.1 with 16 MB
  flash. After boot, firmware reported hardware type 14 (Model B) and build ID
  `d00cd760c115-dirty`.
- PlatformIO upload to `/dev/ttyUSB0` completed; esptool verified hashes for
  bootloader, partition table, boot-app table, and application image.
- Firmware source base commit: `d00cd760c115fea810b836ab290a4a795bce655f`.
  Firmware image SHA-256:
  `da6c3f5bae73f923a5a6b9ea46c5428f222884450737cec1a8f61223ed8c7afc`.
- A passive 115200-baud read received and checksum-validated an `@FT1` frame:
  device capture monotonic time 4751 ms; device UTC-valid flag 1; capture UTC
  `2026-10-02T12:32:35.744Z`; 30 serialized fields. The laptop clock at the
  time was `2026-10-03T16:25:59Z`, so the device UTC was about 27 h 53 min
  behind and is not suitable for aligning symptom times until its clock is
  synchronized. That sampled frame had no RPM/age fields and reported Model B
  supply field 0x24 as 383 (3.83 V under the dashboard's scale); this is not
  evidence of valid vehicle supply voltage or a running ECU.
- The dashboard subsequently logged `CONNECTED` on `/dev/ttyUSB0` using
  `Freematics Telemetry v1` / passive mode. Its debug log reported one
  corrupt record during startup; `obd-dashboard` then held the serial port.
  Do not run a serial monitor concurrently. The connection was established,
  but no live RPM or valid in-car voltage was confirmed through the UI.
- The user's initial connection log shows auto-detect timing out after about
  1.7 s. Linux's serialport documentation warns that opening a port can pulse
  DTR and reset ESP32/CH340 devices even when DTR is preserved. Auto-detect now
  waits up to 12 s for a valid checksummed frame to allow that boot cycle.

Still not verified on the vehicle: live RPM and supply voltage/ages, 250 ms
sampling cadence, upload continuity, SD journalling while connected, and
recording after closing/disconnecting the dashboard. Do not treat the smoke
test as a vehicle test or a mechanical diagnosis.

On the live dashboard, the operator can write `SHUDDER`, `AC_ON`, `AC_OFF`, and
`ELECTRICAL_LOAD_CHANGE` markers into the laptop's `obd-debug.log`. Each marker
contains laptop UTC, the newest device capture UTC/monotonic timestamp, and
device-frame receive age so it can be aligned with the SD journal. These are
manual observations, not automated A/C state readings or a diagnosis.
