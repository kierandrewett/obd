//! OBD operations shared across desktop and WASM targets.
//!
//! Diagnostic functions use [`DiagnosticAdapter`]; initialisation remains adapter-specific.
//! - Desktop calls them via `elm327::block_on(obd_ops::foo(...))`
//! - WASM calls them with `.await`

use crate::adapter::{DiagnosticAdapter, request_hex};
use crate::app::{ObdEvent, PollConfig, PollMode};
use crate::elm327::{Elm327Error, ElmAdapter, decode_protocol};
use crate::obd;
use std::sync::mpsc;

const SUPPORTED_PID_RANGES: &[&str] = &["0100", "0120", "0140", "0160", "0180", "01A0", "01C0"];

/// Run the standard ELM327 initialisation sequence.
/// `status` receives human-readable progress strings.
pub async fn init_elm<A, F>(elm: &mut A, status: F) -> Result<(), Elm327Error>
where
    A: ElmAdapter,
    F: Fn(&str),
{
    init_elm_with_mode(elm, status, crate::elm327::ElmCanMode::Auto).await
}

/// Initialise an ELM adapter in automatic OBD mode or an explicitly selected
/// Corsa D medium-speed CAN mode. The latter is opt-in and must fail closed if
/// the adapter rejects either user-protocol command.
pub async fn init_elm_with_mode<A, F>(
    elm: &mut A,
    status: F,
    mode: crate::elm327::ElmCanMode,
) -> Result<(), Elm327Error>
where
    A: ElmAdapter,
    F: Fn(&str),
{
    // Reset — ignore errors; the device may not respond immediately.
    let _ = elm.send("ATZ", 2000).await;
    elm.sleep_ms(500).await;

    for command in ["ATE0", "ATL0", "ATS0", "ATH0"] {
        let lines = elm.send(command, 2000).await?;
        if !lines.iter().any(|line| line.trim() == "OK") {
            return Err(Elm327Error::InitFailed(format!(
                "{command} rejected: {}",
                lines.join(" | ")
            )));
        }
    }

    match mode {
        crate::elm327::ElmCanMode::Auto => {
            require_elm_ok(elm, "ATSP0").await?;
        }
        crate::elm327::ElmCanMode::CorsaDMediumSpeed => {
            // ELM327 PP 2C=0x91 selects 11-bit ISO-TP with the 8/7 bitrate
            // multiplier; PP 2D=0x06 gives (500/6)*(8/7) ~= 95.2 kbit/s.
            // AT PB applies temporary parameters and is supported only by
            // genuine/compatible ELM implementations. Never fall back to HS.
            require_elm_ok(elm, "AT PB 91 06").await?;
            require_elm_ok(elm, "ATSPB").await?;
        }
    }

    // Branded compatible adapters need not report the literal ELM327 name.
    if let Ok(lines) = elm.send("ATI", 1000).await {
        if let Some(version) = lines.iter().find(|line| {
            let upper = line.to_ascii_uppercase();
            ["ELM", "OBDLINK", "STN", "ELS", "VLINKER"]
                .iter()
                .any(|name| upper.contains(name))
        }) {
            elm.info_mut().elm_version = version.clone();
        }
    }

    status("Detecting OBD protocol...");
    let has_obd_pid_response = match request_hex(elm, "0100", 8000).await {
        Ok(lines) => lines
            .iter()
            .any(|line| line.starts_with("4100") && line.len() >= 12),
        Err(error) if mode == crate::elm327::ElmCanMode::CorsaDMediumSpeed => {
            tracing::info!(error = %error, "No generic OBD Mode 01 responder on selected MS-CAN bus");
            false
        }
        Err(error) => return Err(error),
    };
    if !has_obd_pid_response && mode == crate::elm327::ElmCanMode::Auto {
        return Err(Elm327Error::InitFailed(
            "No supported-PID response; check ignition and adapter connection".into(),
        ));
    }

    let protocol_lines = match mode {
        crate::elm327::ElmCanMode::Auto => elm.send("ATDPN", 1000).await.ok(),
        crate::elm327::ElmCanMode::CorsaDMediumSpeed => Some(elm.send("ATDPN", 1000).await?),
    };
    if let Some(p) = protocol_lines.as_ref().and_then(|lines| lines.first()) {
        if mode == crate::elm327::ElmCanMode::CorsaDMediumSpeed
            && p.trim().trim_start_matches('A') != "B"
        {
            return Err(Elm327Error::InitFailed(format!(
                "Requested Corsa D MS-CAN User Protocol B, adapter reports {}",
                p.trim()
            )));
        }
        elm.info_mut().protocol = decode_protocol(p.trim()).to_string();
    } else if mode == crate::elm327::ElmCanMode::CorsaDMediumSpeed {
        return Err(Elm327Error::InitFailed(
            "Adapter did not report the selected MS-CAN protocol".into(),
        ));
    }

    if mode == crate::elm327::ElmCanMode::CorsaDMediumSpeed {
        elm.info_mut().protocol = if has_obd_pid_response {
            "Corsa D MS-CAN · ISO-TP 11-bit · 95.2 kbit/s".into()
        } else {
            "Corsa D MS-CAN · 95.2 kbit/s · no generic OBD responder".into()
        };
    }

    if let Ok(lines) = elm.send("ATRV", 1000).await {
        elm.info_mut().voltage = lines.into_iter().next();
    }

    Ok(())
}

async fn require_elm_ok<A: ElmAdapter>(elm: &mut A, command: &str) -> Result<(), Elm327Error> {
    let lines = elm.send(command, 2000).await?;
    if !lines.iter().any(|line| line.trim() == "OK") {
        return Err(Elm327Error::InitFailed(format!(
            "{command} rejected: {}",
            lines.join(" | ")
        )));
    }
    Ok(())
}

/// Read stored (Mode 03) and pending (Mode 07) DTCs.
///
/// Sends `DtcResult` immediately with codes and `DescSource::Pending` descriptions
/// so the UI can display codes right away.  Returns the raw lists so the caller
/// can enrich descriptions in a background task.
pub async fn read_dtcs<A: DiagnosticAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
) -> (Vec<obd::Dtc>, Vec<obd::Dtc>) {
    let stored = match request_hex(elm, "03", 5000).await {
        Ok(lines) => obd::parse_dtc_response_lines(&lines, "43"),
        Err(error) => {
            let _ = event_tx.send(ObdEvent::Error(format!("Stored DTC read failed: {error}")));
            return (Vec::new(), Vec::new());
        }
    };
    let pending = match request_hex(elm, "07", 5000).await {
        Ok(lines) => obd::parse_dtc_response_lines(&lines, "47"),
        Err(error) => {
            let _ = event_tx.send(ObdEvent::Error(format!("Pending DTC read failed: {error}")));
            return (Vec::new(), Vec::new());
        }
    };
    let _ = event_tx.send(ObdEvent::DtcResult {
        stored: stored.clone(),
        pending: pending.clone(),
    });
    (stored, pending)
}

/// Clear all DTCs (Mode 04) then re-read to confirm.
/// Same immediate-send behaviour as `read_dtcs`.
pub async fn clear_dtcs<A: DiagnosticAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
) -> (Vec<obd::Dtc>, Vec<obd::Dtc>) {
    match request_hex(elm, "04", 5000).await {
        Ok(_) => {
            let _ = event_tx.send(ObdEvent::LogMessage("[DTC_CLEAR] DTCs cleared".into()));
            read_dtcs(elm, event_tx).await
        }
        Err(e) => {
            let _ = event_tx.send(ObdEvent::Error(format!("Clear DTCs failed: {e}")));
            (Vec::new(), Vec::new())
        }
    }
}

/// Read the VIN via Mode 09 PID 02.
pub async fn read_vin<A: DiagnosticAdapter>(elm: &mut A, event_tx: &mpsc::Sender<ObdEvent>) {
    match request_hex(elm, "0902", 5000).await {
        Ok(lines) => {
            let vin = obd::parse_encoded_string_response(&lines, "4902")
                .unwrap_or_else(|| "Not available".into());
            let _ = event_tx.send(ObdEvent::Vin(vin));
        }
        Err(e) => {
            let _ = event_tx.send(ObdEvent::Error(format!("VIN read failed: {e}")));
        }
    }
}

/// Poll a set of Mode 01 PIDs determined by `poll_config.mode`.
pub async fn poll_live_data<A: DiagnosticAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
    pid_defs: &[obd::PidDef],
    poll_config: &PollConfig,
) {
    let cmds: &[&str] = match poll_config.mode {
        PollMode::Minimal => &["010C", "010D", "0111", "0104"],
        PollMode::Fast => &["010C", "010D", "0111", "0104", "0105", "010F", "0110"],
        PollMode::Full => &[
            "010C", "010D", "0105", "0104", "0111", "010F", "0110", "012F", "0106", "0107", "010B",
            "010E", "015C", "0142", "0146", "012C", "012E", "0133", "0149", "0144", "01A6",
        ],
    };

    for cmd in cmds {
        let pid_def = match pid_defs.iter().find(|p| p.cmd == *cmd) {
            Some(p) => p,
            None => continue,
        };
        if let Ok(lines) = request_hex(elm, cmd, 2000).await {
            let raw = lines.join("|");
            if let Some(data_bytes) = obd::parse_elm_response(cmd, &lines) {
                let _ = event_tx.send(ObdEvent::LiveData {
                    pid_cmd: cmd.to_string(),
                    name: pid_def.description.to_string(),
                    value: obd::decode_pid(pid_def, &data_bytes),
                    unit: pid_def.unit.to_string(),
                    raw,
                });
            }
        }
        if poll_config.inter_pid_delay_ms > 0 {
            elm.delay(poll_config.inter_pid_delay_ms).await;
        }
    }
}

/// Read freeze frame data (Mode 02) for a standard set of PIDs.
pub async fn read_freeze_frame<A: DiagnosticAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
    pid_defs: &[obd::PidDef],
) {
    let freeze_pids = [
        "0104", "0105", "0106", "0107", "010B", "010C", "010D", "010E", "010F", "0110", "0111",
        "012F", "0142",
    ];

    for pid01 in &freeze_pids {
        let cmd = format!("02{}00", &pid01[2..]);
        let pid_def = match pid_defs.iter().find(|p| p.cmd == *pid01) {
            Some(p) => p,
            None => continue,
        };
        if let Ok(lines) = request_hex(elm, &cmd, 3000).await {
            let prefix_42 = format!("42{}", &pid01[2..4]);
            for line in &lines {
                let clean = line.replace(' ', "").to_uppercase();
                if let Some(pos) = clean.find(&prefix_42) {
                    let after = &clean[pos + prefix_42.len()..];
                    let data_str = if after.len() >= 2 { &after[2..] } else { after };
                    let mut bytes = Vec::new();
                    let mut i = 0;
                    while i + 1 < data_str.len() {
                        if let Ok(b) = u8::from_str_radix(&data_str[i..i + 2], 16) {
                            bytes.push(b);
                        }
                        i += 2;
                    }
                    if !bytes.is_empty() {
                        let _ = event_tx.send(ObdEvent::FreezeFrameData {
                            pid_cmd: pid01.to_string(),
                            name: pid_def.description.to_string(),
                            value: obd::decode_pid(pid_def, &bytes),
                            unit: pid_def.unit.to_string(),
                        });
                    }
                }
            }
        }
    }
}

/// Query supported Mode 01 PIDs, following continuation bits through PID C0.
/// Keep these ranges in sync with the 01A6 odometer entry in the PID catalogue.
fn append_supported_pid_page(base: u8, data: &[u8], supported: &mut Vec<u8>) -> Option<bool> {
    if data.len() < 4 {
        return None;
    }
    let bits = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
    // Bitmap bit 0 announces the next 32-PID page; it is not itself a PID.
    // Bits 31 through 1 map to base+1 through base+31.
    for bit_position in 1..=31u32 {
        if bits & (1u32 << bit_position) != 0 {
            let pid = u16::from(base) + (32 - bit_position as u16);
            if pid <= u16::from(u8::MAX) {
                supported.push(pid as u8);
            }
        }
    }
    Some(base < 0xC0 && bits & 1 != 0)
}

pub async fn query_supported_pids<A: DiagnosticAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
) {
    let mut all_supported = Vec::new();
    for range in SUPPORTED_PID_RANGES {
        match request_hex(elm, range, 2000).await {
            Ok(lines) => {
                let Some(data) = obd::parse_elm_response(range, &lines) else {
                    break;
                };
                let base = u8::from_str_radix(&range[2..4], 16).unwrap_or(0);
                match append_supported_pid_page(base, &data, &mut all_supported) {
                    Some(true) => {}
                    Some(false) | None => break,
                }
            }
            Err(_) => break,
        }
    }
    let _ = event_tx.send(ObdEvent::SupportedPids(all_supported));
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod elm_profile_tests {
    use super::*;
    use crate::elm327::{ConnectionInfo, ElmCanMode};

    struct FakeElm {
        commands: Vec<String>,
        info: ConnectionInfo,
        reject_profile: bool,
    }

    impl FakeElm {
        fn new(reject_profile: bool) -> Self {
            Self {
                commands: Vec::new(),
                info: ConnectionInfo {
                    port: "fake".into(),
                    baud: 38_400,
                    protocol: String::new(),
                    elm_version: String::new(),
                    voltage: None,
                },
                reject_profile,
            }
        }
    }

    impl ElmAdapter for FakeElm {
        async fn send(
            &mut self,
            command: &str,
            _timeout_ms: u64,
        ) -> Result<Vec<String>, Elm327Error> {
            self.commands.push(command.to_string());
            Ok(match command {
                "AT PB 91 06" if self.reject_profile => vec!["?".into()],
                "ATDPN" => vec!["B".into()],
                "ATI" => vec!["ELM327 test".into()],
                "ATRV" => vec!["12.4V".into()],
                "0100" => vec!["NO DATA".into()],
                _ => vec!["OK".into()],
            })
        }

        fn info(&self) -> &ConnectionInfo {
            &self.info
        }

        fn info_mut(&mut self) -> &mut ConnectionInfo {
            &mut self.info
        }
    }

    #[test]
    fn corsa_d_mscan_selects_user1_952_kbit_and_never_falls_back() {
        let mut elm = FakeElm::new(false);
        crate::elm327::block_on(init_elm_with_mode(
            &mut elm,
            |_| {},
            ElmCanMode::CorsaDMediumSpeed,
        ))
        .unwrap();

        assert_eq!(elm.commands[0], "ATZ");
        assert_eq!(elm.commands[5], "AT PB 91 06");
        assert_eq!(elm.commands[6], "ATSPB");
        assert!(!elm.commands.iter().any(|command| command == "ATSP0"));
        assert_eq!(elm.commands.last().map(String::as_str), Some("ATRV"));
        assert!(elm.info.protocol.contains("95.2 kbit/s"));
        assert!(elm.info.protocol.contains("no generic OBD responder"));
    }

    #[test]
    fn unsupported_user_protocol_fails_closed_without_hs_fallback() {
        let mut elm = FakeElm::new(true);
        let result = crate::elm327::block_on(init_elm_with_mode(
            &mut elm,
            |_| {},
            ElmCanMode::CorsaDMediumSpeed,
        ));

        assert!(result.is_err());
        assert!(elm.commands.iter().any(|command| command == "AT PB 91 06"));
        assert!(!elm.commands.iter().any(|command| command == "ATSP0"));
        assert!(!elm.commands.iter().any(|command| command == "0100"));
    }

    #[test]
    fn supported_pid_continuation_only_pages_do_not_add_pids() {
        let mut supported = Vec::new();
        assert_eq!(
            append_supported_pid_page(0x00, &[0, 0, 0, 1], &mut supported),
            Some(true)
        );
        assert!(
            supported.is_empty(),
            "continuation bit is not a supported PID"
        );
        assert_eq!(
            append_supported_pid_page(0x20, &[0, 0, 0, 1], &mut supported),
            Some(true)
        );
        assert!(supported.is_empty());
    }

    #[test]
    fn supported_pid_pages_decode_map_across_continuations() {
        let mut supported = Vec::new();

        // 0100 reports PID 01 and announces 0120. Bit 0 must not become PID 20.
        assert_eq!(
            append_supported_pid_page(0x00, &[0x80, 0, 0, 1], &mut supported),
            Some(true)
        );
        assert_eq!(supported, vec![0x01]);

        // 0120 reports PID 21 and announces 0140. Bit 0 must not become PID 40.
        assert_eq!(
            append_supported_pid_page(0x20, &[0x80, 0, 0, 1], &mut supported),
            Some(true)
        );
        assert_eq!(supported, vec![0x01, 0x21]);

        // 0140 reports PID 5F and stops; the lowest PID bit is still reserved.
        assert_eq!(
            append_supported_pid_page(0x40, &[0, 0, 0, 2], &mut supported),
            Some(false)
        );
        assert_eq!(supported, vec![0x01, 0x21, 0x5F]);

        // PID A6 is bit 6 of the 01A0 page; bit 0 announces a 01C0 page.
        assert_eq!(
            append_supported_pid_page(0xA0, &[0x04, 0, 0, 1], &mut supported),
            Some(true)
        );
        assert_eq!(supported, vec![0x01, 0x21, 0x5F, 0xA6]);

        // The terminal 01C0 page has no continuation page in the standard map.
        assert_eq!(
            append_supported_pid_page(0xC0, &[0, 0, 0, 1], &mut supported),
            Some(false)
        );
        assert_eq!(supported, vec![0x01, 0x21, 0x5F, 0xA6]);
        assert_eq!(
            append_supported_pid_page(0x00, &[1, 2, 3], &mut supported),
            None
        );
        assert_eq!(SUPPORTED_PID_RANGES.last(), Some(&"01C0"));
        assert!(!SUPPORTED_PID_RANGES.contains(&"01E0"));
    }
}

/// Full DTC enrichment pipeline using the compile-time embedded DTC database.
/// Checks the manufacturer DB first (direct match then alias/family group),
/// falls back to SAE J2012, then marks as `NotFound`.
/// Used on WASM where the filesystem is unavailable at runtime.
#[cfg(target_arch = "wasm32")]
pub fn enrich_with_db(dtcs: Vec<crate::obd::Dtc>, make: Option<&str>) -> Vec<crate::obd::Dtc> {
    dtcs.into_iter()
        .map(|mut dtc| {
            if let Some(m) = make {
                let db = &*crate::dtc_database::EMBEDDED_DB;
                if let Some((desc, alias_src)) = db.lookup_with_source(m, &dtc.code) {
                    dtc.description = desc.to_string();
                    dtc.desc_source = match alias_src {
                        None => crate::obd::DescSource::Own,
                        Some(a) => crate::obd::DescSource::Family(title_case(a)),
                    };
                    return dtc;
                }
            }
            // SAE J2012 generic fallback
            let sae = crate::dtc_descriptions::describe(&dtc.code);
            if !sae.is_empty() {
                dtc.description = sae.to_string();
                dtc.desc_source = crate::obd::DescSource::Sae;
            } else {
                dtc.desc_source = crate::obd::DescSource::NotFound;
            }
            dtc
        })
        .collect()
}

#[cfg(target_arch = "wasm32")]
fn title_case(s: &str) -> String {
    let mut t = s.to_string();
    if let Some(c) = t.get_mut(0..1) {
        c.make_ascii_uppercase();
    }
    t
}
