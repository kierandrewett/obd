# Adapter implementation and validation

## Boundary

`DiagnosticAdapter` exchanges diagnostic payload bytes and returns separate responses with optional
source identifiers. CAN identifiers and ISO-TP segmentation belong to the backend. `ElmAdapter`
remains the interface for ELM commands; its blanket diagnostic implementation normalises headers-off
responses. J2534 does not emulate an ELM device or accept AT commands.

`obd_ops` uses the diagnostic interface for standard requests. `request_hex` currently adapts these
responses to the existing standard OBD decoders. Enhanced diagnostics should consume the typed
responses directly so module identities are not discarded. The existing DTC model and make-based
description lookup are not sufficient for ECU-specific enhanced diagnostics.

Desktop serial, TCP and J2534 connections share the same worker and operations. The browser retains
Web Serial and the development WebSocket emulator. A native bridge would be required for browser
access to J2534 DLLs. This change does not introduce such a bridge.

## Corsa D medium-speed CAN (experimental)

The desktop serial ELM profile sends `AT PB 91 06` followed by `ATSPB`: protocol B options `91`
select 11-bit ISO-TP with the ELM 8/7 baud multiplier, and divisor `06` gives approximately
95.238 kbit/s. It then tries the standard OBD-II `0100` request. A selected MS-CAN bus can be
reachable while having no generic OBD responder; that is represented as an explicit no-responder
state, with no silent retry on HS-CAN. The profile is not a raw CAN logger and does not implement
Opel body-module addressing, discovery, or decoding. The adapter's HS/MS switch remains manual.

The ELM parameter interpretation follows the [ELM327 datasheet](https://www.elmelectronics.com/wp-content/uploads/2017/01/ELM327DS.pdf),
which specifies that `PB` sets User Protocol B CAN options/baud rate. A [Corsa D CAN bus field report](https://blog.ajwilson.me/posts/2021-05-06-reading-corsa-d-canbus/)
reports approximately 95 kbit/s for the medium-speed network. This profile has simulator coverage
only; it has not been verified with the user's reader or vehicle.

## J2534 contract

- Version 04.04 exports, Windows calling convention and 32-bit integer fields.
- Driver lifetime exceeds device/channel lifetime. Failed setup releases opened handles.
- One ISO 15765 channel; 11-bit or 29-bit, 250 or 500 kbit/s.
- 11-bit functional requests use `7DF`; eight flow-control filters pair `7E8..7EF` with `7E0..7E7`.
- 29-bit functional requests use `18DB33F1`; one selected ECU address pairs `18DAF1xx` with `18DAxxF1`.
  Source address `10` is only the UI default, not a claim that every ECU uses that address.
- No address-extension byte, alternate connector pins or enhanced module scan.
- Maximum application payload is 4095 bytes. The driver performs ISO-TP segmentation and reassembly.
- Receive processing checks lengths, protocol, source and indication flags. Pending responses must
  complete within the caller's deadline. Ordinary functional replies are collected until 150 ms of
  quiet after the last matching response; this is a collection window, not an ECU timing guarantee.
- Native driver discovery uses `PassThruSupport.04.04` in the Windows registry view matching the process.
  Manual absolute paths are supported. An installed driver is not proof that an adapter is connected.
- Non-Windows builds can load libraries implementing this exact ABI. This is exercised by the Linux
  fixture; it does not imply that a Windows DLL or arbitrary Linux J2534 library works on Linux.

No third-party implementation code was copied. Reference material:

- [Opus programming documentation](https://opusivs-uk.com/support/oem-customer-support/how-to-program-with-opus-ivss-passthru-sae-j2534-dll/)
- [Opus driver discovery](https://opusivs.com/support/oem-customer-support/program-sae-j2534/discovery-of-the-dll/)
- [Quantex flow-control filters](https://quantexlab.de/en/develop/j2534/pt_start_msgfilt.html)
- [Quantex receive messages and indications](https://quantexlab.de/en/develop/j2534/pt_readmsg.html)

## PSA-specific interfaces

An interface sold for Lexia/PP2000/DiagBox must not be treated as ELM-compatible merely because it uses USB.
[PyPSADiag](https://github.com/Barracuda09/PyPSADiag) demonstrates a separate VCI path using installed PSA
communication files under `AWRoot`, and also documents an ELM route. Its implementation is GPL-2.0;
its code and the vendor libraries have not been imported into this MIT project.

Before implementing that backend, establish the relevant library ABI, driver architecture, device-opening
and channel-selection behaviour, and permitted distribution of any dependencies. Validate with a physical
interface. Do not advertise PSA VCI support until that path works. A device with a compatible J2534 04.04
driver can use the J2534 backend within the CAN limits above, regardless of its marketing label.

## Checks

```sh
cargo test --all-targets
cargo check --target wasm32-unknown-unknown --lib
cargo check --target x86_64-pc-windows-gnu --all-targets
cargo check --target i686-pc-windows-gnu --all-targets
cargo fmt --check
similarity-rs src --threshold 0.85
```

The Linux integration tests need a C compiler (`cc`) and PTY access. They use local sockets and a test-only
shared library, not a vehicle. Cross-target checks require the corresponding Rust targets and toolchain.
Windows compilation does not test Windows driver loading or registry discovery against a real installation.
The egui test lays out each adapter selector and checks that selection reaches the worker command.

For a 32-bit Windows build on a configured toolchain:

```sh
cargo build --release --target i686-pc-windows-gnu --bin obd-dashboard
```

## Physical validation still required

For each adapter, record its model, driver/firmware version, OS and process architecture, vehicle generation,
engine, selected network and operations tested. Compare VIN, live data and fault reads against a reference
tool. Keep raw responses and explicit failures. The first hardware pass should use read operations;
clearing codes needs a separate deliberate action because it changes vehicle diagnostic state.

Vehicle profiles and definitions should be developed by platform and ECU identity: Ford, GM-derived Opel,
PSA-derived Opel/Peugeot/Citroen/DS and FCA-derived vehicles. Corporate ownership is not a diagnostic
protocol or a reliable proprietary DTC namespace.
