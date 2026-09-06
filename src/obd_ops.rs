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

/// Run the standard ELM327 initialisation sequence.
/// `status` receives human-readable progress strings.
pub async fn init_elm<A, F>(elm: &mut A, status: F) -> Result<(), Elm327Error>
where
    A: ElmAdapter,
    F: Fn(&str),
{
    // Reset — ignore errors; the device may not respond immediately.
    let _ = elm.send("ATZ", 2000).await;
    elm.sleep_ms(500).await;

    for command in ["ATE0", "ATL0", "ATS0", "ATH0", "ATSP0"] {
        let lines = elm.send(command, 2000).await?;
        if !lines.iter().any(|line| line.trim() == "OK") {
            return Err(Elm327Error::InitFailed(format!(
                "{command} rejected: {}",
                lines.join(" | ")
            )));
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
    let lines = request_hex(elm, "0100", 8000).await?;
    if !lines
        .iter()
        .any(|line| line.starts_with("4100") && line.len() >= 12)
    {
        return Err(Elm327Error::InitFailed(
            "No supported-PID response; check ignition and adapter connection".into(),
        ));
    }

    if let Ok(lines) = elm.send("ATDPN", 1000).await {
        if let Some(p) = lines.first() {
            elm.info_mut().protocol = decode_protocol(p.trim()).to_string();
        }
    }

    if let Ok(lines) = elm.send("ATRV", 1000).await {
        elm.info_mut().voltage = lines.into_iter().next();
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
            "010E", "015C", "0142", "0146", "012C", "012E", "0133", "0149", "0144",
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

/// Query supported PIDs across the four standard Mode 01 ranges.
pub async fn query_supported_pids<A: DiagnosticAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
) {
    let mut all_supported = Vec::new();
    for range in &["0100", "0120", "0140", "0160"] {
        match request_hex(elm, range, 2000).await {
            Ok(lines) => {
                if let Some(data) = obd::parse_elm_response(range, &lines) {
                    let base = u8::from_str_radix(&range[2..4], 16).unwrap_or(0);
                    if data.len() >= 4 {
                        let bits = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                        for i in 0..32u8 {
                            if bits & (1 << (31 - i)) != 0 {
                                all_supported.push(base + i + 1);
                            }
                        }
                    }
                }
            }
            Err(_) => break,
        }
    }
    let _ = event_tx.send(ObdEvent::SupportedPids(all_supported));
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
