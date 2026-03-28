//! OBD operations shared across desktop and WASM targets.
//!
//! All functions are `async` and generic over [`ElmAdapter`].
//! - Desktop calls them via `elm327::block_on(obd_ops::foo(...))`
//! - WASM calls them with `.await`

use crate::app::{ObdEvent, PollConfig, PollMode};
use crate::elm327::{decode_protocol, Elm327Error, ElmAdapter};
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

    elm.send("ATE0", 1000).await?; // echo off
    elm.send("ATL0", 1000).await?; // linefeeds off
    elm.send("ATS0", 1000).await?; // spaces off
    elm.send("ATH0", 1000).await?; // headers off
    elm.send("ATSP0", 2000).await?; // auto-detect protocol

    // Read firmware version string.
    if let Ok(lines) = elm.send("ATI", 1000).await {
        if let Some(ver) = lines.iter().find(|l| l.contains("ELM")) {
            elm.info_mut().elm_version = ver.clone();
        }
    }

    // Trigger protocol detection with a real OBD request.
    status("Detecting OBD protocol…");
    match elm.send("0100", 8000).await {
        Ok(lines) => {
            if lines
                .iter()
                .any(|l| l.contains("UNABLE") || l.contains("NO DATA") || l.contains("BUS INIT"))
            {
                return Err(Elm327Error::ProtocolError(
                    "Vehicle not responding — is the ignition on?".into(),
                ));
            }
        }
        Err(e) => return Err(Elm327Error::InitFailed(format!("Protocol detection: {e}"))),
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
/// `enrich` receives the raw code list and may replace descriptions with
/// manufacturer-specific ones.  Pass `|d| d` on platforms without a DTC database.
pub async fn read_dtcs<A, F>(elm: &mut A, event_tx: &mpsc::Sender<ObdEvent>, enrich: F)
where
    A: ElmAdapter,
    F: Fn(Vec<obd::Dtc>) -> Vec<obd::Dtc>,
{
    let stored = match elm.send("03", 5000).await {
        Ok(lines) => enrich(obd::parse_dtc_response_lines(&lines, "43")),
        Err(_) => Vec::new(),
    };
    let pending = match elm.send("07", 5000).await {
        Ok(lines) => enrich(obd::parse_dtc_response_lines(&lines, "47")),
        Err(_) => Vec::new(),
    };
    let _ = event_tx.send(ObdEvent::DtcResult { stored, pending });
}

/// Clear all DTCs (Mode 04) then re-read to confirm.
pub async fn clear_dtcs<A, F>(elm: &mut A, event_tx: &mpsc::Sender<ObdEvent>, enrich: F)
where
    A: ElmAdapter,
    F: Fn(Vec<obd::Dtc>) -> Vec<obd::Dtc>,
{
    match elm.send("04", 5000).await {
        Ok(_) => {
            let _ = event_tx.send(ObdEvent::LogMessage("[DTC_CLEAR] DTCs cleared".into()));
            read_dtcs(elm, event_tx, enrich).await;
        }
        Err(e) => {
            let _ = event_tx.send(ObdEvent::Error(format!("Clear DTCs failed: {e}")));
        }
    }
}

/// Read the VIN via Mode 09 PID 02.
pub async fn read_vin<A: ElmAdapter>(elm: &mut A, event_tx: &mpsc::Sender<ObdEvent>) {
    match elm.send("0902", 5000).await {
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
pub async fn poll_live_data<A: ElmAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
    pid_defs: &[obd::PidDef],
    poll_config: &PollConfig,
) {
    let cmds: &[&str] = match poll_config.mode {
        PollMode::Minimal => &["010C", "010D", "0111", "0104"],
        PollMode::Fast => &["010C", "010D", "0111", "0104", "0105", "010F", "0110"],
        PollMode::Full => &[
            "010C", "010D", "0105", "0104", "0111", "010F", "0110", "012F", "0106", "0107",
            "010B", "010E", "015C", "0142", "0146", "012C", "012E", "0133", "0149", "0144",
        ],
    };

    for cmd in cmds {
        let pid_def = match pid_defs.iter().find(|p| p.cmd == *cmd) {
            Some(p) => p,
            None => continue,
        };
        if let Ok(lines) = elm.send(cmd, 2000).await {
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
            elm.sleep_ms(poll_config.inter_pid_delay_ms).await;
        }
    }
}

/// Read freeze frame data (Mode 02) for a standard set of PIDs.
pub async fn read_freeze_frame<A: ElmAdapter>(
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
        if let Ok(lines) = elm.send(&cmd, 3000).await {
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
pub async fn query_supported_pids<A: ElmAdapter>(
    elm: &mut A,
    event_tx: &mpsc::Sender<ObdEvent>,
) {
    let mut all_supported = Vec::new();
    for range in &["0100", "0120", "0140", "0160"] {
        match elm.send(range, 2000).await {
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
