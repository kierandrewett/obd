# Freematics USB dashboard verification

The dashboard's Freematics USB connection is a passive reader for the Model B
TeleLogger `@FT1` stream. Auto-detect considers only the Model B's CP210x USB
bridge (VID:PID `10c4:ea60`), then accepts it only after parsing a checksummed
telemetry record. Generic CH340 OBD adapters and other serial devices are
rejected before the app opens a port. It sends no ELM or diagnostic commands.
The app preserves DTR and never asserts DTR or RTS itself. Linux's tty layer can
still pulse DTR during open despite that setting, which may reset the ESP32;
the app waits up to 12 seconds for the device's first frame after opening. Keep
one reader on the selected port; a serial monitor must not share it with the
dashboard.

For a source-of-gaps measurement, stop the dashboard and run the firmware
repository's `tools/measure_usb_stream.py` against the verified Model B port.
The sanitized JSON reports capture cadence, transport arrivals, device USB
drops, per-PID acquisition-age resets, the OBD timeout-counter delta, and the
latest OBD read latency observed on frames with acquisition activity. A PID
age reset indicates a successful ECU response even when its value did not
change; the timeout counter records failed reads. More than one ECU request
can occur between 250 ms telemetry frames, so latency percentiles are sampled
observations, not a lossless per-request trace. Never run this measurement at
the same time as the dashboard or serial monitor.

The repeatable parser-level serial traffic scenario is
`tests/fixtures/freematics_usb_scenario.txt`, exercised by
`freematics_usb::tests::simulated_serial_shudder_and_reconnect_scenario_is_repeatable`.
The test fragments input into 11-byte reads, includes debug output, one
checksum-valid frame with a colonless field (matching the malformed shape
observed on hardware), a warm-idle RPM dip (820 to 540 RPM), a Model B supply
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
- A 12 s passive serial smoke read observed 28 `@FT1` lines: 23 passed the
  framing/checksum/field checks and 5 had a comma-delimited payload item with
  no `:` separator. The dashboard independently logged a rising corrupt-record
  count. Valid records are arriving, but the reason for malformed lines still
  needs investigation before relying on loss-sensitive captures.
- The dashboard subsequently logged `CONNECTED` on `/dev/ttyUSB0` using
  `Freematics Telemetry v1` / passive mode. Its debug log reported one
  corrupt record during startup; `obd-dashboard` then held the serial port.
  Do not run a serial monitor concurrently. The connection was established,
  but no live RPM or valid in-car voltage was confirmed through the UI.
- A repaint-policy regression test reproduced the idle-refresh bug (red when a
  connected passive stream did not request another repaint) and now passes
  with a 100 ms repaint interval while Freematics is connected, without
  enabling diagnostic polling.
- The user's initial connection log shows auto-detect timing out after about
  1.7 s. Linux's serialport documentation warns that opening a port can pulse
  DTR and reset ESP32/CH340 devices even when DTR is preserved. Auto-detect now
  waits up to 12 s for a valid checksummed frame to allow that boot cycle.

## Current USB and diagnostics changes

The firmware now publishes each FT1 telemetry record as one contiguous serial
write. The Model B USB queue is bounded and coalesces queued old snapshots to
the newest waiting sample while preserving any record already in flight. Every
discard increments the cumulative USB drop counter in the next frame. This is
a live-view policy only; the cloud uploader and SD journal use their existing
independent acquisition path.

`tools/check-usb-telemetry-queue.py` compiles the production queue header with a
mutex-backed FreeRTOS critical-section shim. It checks preserved in-flight
records, stale-backlog dropping, sequence/checksum integrity, and concurrent
single-producer/single-consumer stress. It is a host simulation, not a
measurement of vehicle sampling or upload latency under a saturated UART.

FT1's supported-PID header can now append a validated optional `;vin=` value.
The dashboard shows that device-reported VIN and supported Mode 01 inventory.
The firmware already has an 87-entry generic Mode 01 catalogue; at runtime it
reports and polls only PIDs the connected ECU advertises. It does not invent
support or refresh a failed reading's timestamp. The firmware's periodic
stored/pending/permanent DTC scans (two-minute interval) are shown with each
scan's status and age. The passive USB connection still cannot trigger a scan
or clear codes. Freeze-frame and manufacturer-specific module scans are not
provided by this stream.

Parser and app tests cover VIN validation, DTC status/count/code/age semantics,
unscanned versus successful-empty scans, and passive UI behavior. The repeatable
serial fixture still covers RPM/voltage dips, actual per-PID ages, partial and
corrupt input, debug lines, and a device restart. A separate app regression test
rejects a delayed duplicate/older capture from the same boot (so stale queued
frames cannot overwrite newer values) while accepting the 32-bit capture clock
wrap. The Corsa D MS-CAN profile is separate: it configures an ELM-compatible
adapter for User Protocol B at about 95.2 kbit/s and does not add Opel
body-module addressing or decoding.

The latest firmware build has not yet been installed in the connected car.
Live acquisition ages, actual RPM/supply voltage, upload continuity, SD
journalling through the reset, and recording after closing the dashboard still
need post-flash vehicle verification. Do not treat simulated tests as a
vehicle test or a mechanical diagnosis.
