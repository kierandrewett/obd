mod app;
mod dtc_descriptions;
mod elm327;
mod gauges;
mod obd;
mod vin_decoder;

use app::{ObdApp, ObdEvent, OdbCmd};
use obd::PidDef;
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::Duration;
use tracing::{info, warn};
use tracing_subscriber::{fmt, layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

fn main() {
    // ── Tracing setup (internal/driver logging to stderr) ───────────────────
    tracing_subscriber::registry()
        .with(EnvFilter::new(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "info".to_string()),
        ))
        .with(
            fmt::layer()
                .with_target(false)
                .with_thread_ids(true)
                .with_ansi(true)
                .with_writer(std::io::stderr),
        )
        .init();

    // ── OBD debug log file (same content as the Log panel) ──────────────────
    let log_path = "obd-debug.log";
    let log_file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(log_path)
        .expect("Failed to open obd-debug.log");
    let log_file = Arc::new(Mutex::new(log_file));

    info!("OBD Dashboard starting, debug log: {log_path}");

    // ── Channels ────────────────────────────────────────────────────────────
    let (cmd_tx, cmd_rx) = mpsc::channel::<OdbCmd>();
    let (event_tx, event_rx) = mpsc::channel::<ObdEvent>();

    // ── OBD background thread ───────────────────────────────────────────────
    let obd_thread = thread::spawn(move || {
        obd_worker(cmd_rx, event_tx);
    });

    // ── GUI ─────────────────────────────────────────────────────────────────
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 750.0])
            .with_min_inner_size([800.0, 500.0])
            .with_title("OBD-II Dashboard"),
        ..Default::default()
    };

    let cmd_tx_clone = cmd_tx.clone();
    let log_file_clone = log_file.clone();
    eframe::run_native(
        "OBD-II Dashboard",
        native_options,
        Box::new(move |cc| Ok(Box::new(ObdApp::new(cc, cmd_tx_clone, event_rx, log_file_clone)))),
    )
    .unwrap();

    // Shutdown
    info!("GUI closed, shutting down");
    let _ = cmd_tx.send(OdbCmd::Shutdown);
    let _ = obd_thread.join();
}

// ── OBD worker thread ───────────────────────────────────────────────────────

fn obd_worker(cmd_rx: mpsc::Receiver<OdbCmd>, event_tx: mpsc::Sender<ObdEvent>) {
    use app::PollConfig;

    let mut elm: Option<elm327::Elm327> = None;
    let mut live_running = false;
    let mut poll_config = PollConfig::default();

    let pid_defs = obd::mode01_pids();

    loop {
        // Check for commands (non-blocking when live data is running)
        let cmd = if live_running {
            cmd_rx.try_recv().ok()
        } else {
            match cmd_rx.recv_timeout(Duration::from_millis(100)) {
                Ok(cmd) => Some(cmd),
                Err(mpsc::RecvTimeoutError::Timeout) => None,
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        };

        if let Some(cmd) = cmd {
            match cmd {
                OdbCmd::Connect { port, baud } => {
                    let _ = event_tx.send(ObdEvent::Connecting("Scanning for OBD adapter...".into()));

                    let progress_tx = event_tx.clone();
                    let progress = move |msg: &str| {
                        let _ = progress_tx.send(ObdEvent::Connecting(msg.to_string()));
                    };

                    let result = if let Some(port_name) = port {
                        elm327::connect(&port_name, baud, Some(&progress))
                    } else {
                        elm327::auto_connect(Some(&progress))
                    };

                    match result {
                        Ok(device) => {
                            let info = device.info.clone();
                            elm = Some(device);
                            let _ = event_tx.send(ObdEvent::Connected(info));

                            // Read voltage + VIN on connect
                            if let Some(ref mut e) = elm {
                                if let Ok(v) = e.read_voltage() {
                                    let _ = event_tx.send(ObdEvent::Voltage(v));
                                }
                                match e.send_logged("0902", Duration::from_secs(5)) {
                                    Ok(lines) => {
                                        if let Some(vin) = obd::parse_encoded_string_response(&lines, "4902") {
                                            let _ = event_tx.send(ObdEvent::Vin(vin));
                                        }
                                    }
                                    Err(err) => {
                                        info!("VIN not available: {err}");
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            let _ = event_tx.send(ObdEvent::ConnectionFailed(e.to_string()));
                        }
                    }
                }

                OdbCmd::Disconnect => {
                    elm = None;
                    live_running = false;
                    let _ = event_tx.send(ObdEvent::Disconnected);
                }

                OdbCmd::StartLiveData => {
                    live_running = true;
                    info!("Live data polling started");
                }

                OdbCmd::StopLiveData => {
                    live_running = false;
                    info!("Live data polling stopped");
                }

                OdbCmd::ReadDtcs => {
                    if let Some(ref mut e) = elm {
                        read_dtcs(e, &event_tx);
                    }
                }

                OdbCmd::ClearDtcs => {
                    if let Some(ref mut e) = elm {
                        info!("[DTC_CLEAR] Clearing DTCs");
                        match e.send_logged("04", Duration::from_secs(5)) {
                            Ok(_) => {
                                let _ = event_tx.send(ObdEvent::LogMessage(
                                    "[DTC_CLEAR] DTCs cleared successfully".into(),
                                ));
                                // Re-read to confirm
                                read_dtcs(e, &event_tx);
                            }
                            Err(err) => {
                                let _ = event_tx.send(ObdEvent::Error(format!("Clear DTCs failed: {err}")));
                            }
                        }
                    }
                }

                OdbCmd::ReadFreezeFrame => {
                    if let Some(ref mut e) = elm {
                        read_freeze_frame(e, &event_tx, &pid_defs);
                    }
                }

                OdbCmd::ReadVin => {
                    if let Some(ref mut e) = elm {
                        match e.send_logged("0902", Duration::from_secs(5)) {
                            Ok(lines) => {
                                if let Some(vin) = obd::parse_encoded_string_response(&lines, "4902") {
                                    let _ = event_tx.send(ObdEvent::Vin(vin));
                                } else {
                                    let _ = event_tx.send(ObdEvent::Vin("Not available".into()));
                                }
                            }
                            Err(err) => {
                                let _ = event_tx.send(ObdEvent::Error(format!("VIN read failed: {err}")));
                            }
                        }
                    }
                }

                OdbCmd::SetPollConfig(config) => {
                    info!(mode = ?config.mode, cycle_delay = config.cycle_delay_ms, inter_pid_delay = config.inter_pid_delay_ms, "Poll config updated");
                    poll_config = config;
                }

                OdbCmd::QuerySupportedPids => {
                    if let Some(ref mut e) = elm {
                        let mut all_supported = Vec::new();
                        for range in &["0100", "0120", "0140", "0160"] {
                            match e.query_supported_pids(range) {
                                Ok(pids) => all_supported.extend(pids),
                                Err(_) => break,
                            }
                        }
                        let _ = event_tx.send(ObdEvent::SupportedPids(all_supported));
                    }
                }

                OdbCmd::Shutdown => {
                    info!("OBD worker shutting down");
                    break;
                }
            }
        }

        // Live data polling
        if live_running {
            if let Some(ref mut e) = elm {
                poll_live_data(e, &event_tx, &pid_defs, &poll_config);

                // Also poll voltage periodically (every poll cycle includes it)
                if let Ok(v) = e.read_voltage() {
                    let _ = event_tx.send(ObdEvent::Voltage(v));
                }

                if poll_config.cycle_delay_ms > 0 {
                    std::thread::sleep(Duration::from_millis(poll_config.cycle_delay_ms));
                }
            }
        }
    }
}

fn poll_live_data(
    elm: &mut elm327::Elm327,
    event_tx: &mpsc::Sender<ObdEvent>,
    pid_defs: &[PidDef],
    poll_config: &app::PollConfig,
) {
    use app::PollMode;

    let poll_cmds: &[&str] = match poll_config.mode {
        PollMode::Minimal => &[
            "010C", // RPM
            "010D", // Speed
            "0111", // Throttle
            "0104", // Engine load
        ],
        PollMode::Fast => &[
            "010C", // RPM
            "010D", // Speed
            "0111", // Throttle
            "0104", // Engine load
            "0105", // Coolant temp
            "010F", // Intake temp
            "0110", // MAF
        ],
        PollMode::Full => &[
            "010C", // RPM
            "010D", // Speed
            "0105", // Coolant temp
            "0104", // Engine load
            "0111", // Throttle
            "010F", // Intake temp
            "0110", // MAF
            "012F", // Fuel level
            "0106", // Short fuel trim B1
            "0107", // Long fuel trim B1
            "010B", // Intake pressure
            "010E", // Timing advance
            "015C", // Oil temp
            "0142", // Control module voltage
            "0146", // Ambient temp
            "012C", // Commanded EGR
            "012E", // Evap purge
            "0133", // Barometric pressure
            "0149", // Accelerator pos D
            "0144", // Commanded equiv ratio
        ],
    };

    for cmd in poll_cmds {
        let pid_def = match pid_defs.iter().find(|p| p.cmd == *cmd) {
            Some(p) => p,
            None => continue,
        };

        match elm.send_logged(cmd, Duration::from_secs(2)) {
            Ok(lines) => {
                let raw = lines.join("|");
                if let Some(data_bytes) = obd::parse_elm_response(cmd, &lines) {
                    let value = obd::decode_pid(pid_def, &data_bytes);
                    let _ = event_tx.send(ObdEvent::LiveData {
                        pid_cmd: cmd.to_string(),
                        name: pid_def.description.to_string(),
                        value,
                        unit: pid_def.unit.to_string(),
                        raw,
                    });
                }
            }
            Err(e) => {
                // Don't spam errors for unsupported PIDs
                if !e.to_string().contains("Timeout") {
                    warn!(cmd, error = %e, "PID query failed");
                }
            }
        }

        if poll_config.inter_pid_delay_ms > 0 {
            std::thread::sleep(Duration::from_millis(poll_config.inter_pid_delay_ms));
        }
    }
}

fn enrich_dtcs(dtcs: Vec<obd::Dtc>) -> Vec<obd::Dtc> {
    dtcs.into_iter()
        .map(|mut dtc| {
            let desc = dtc_descriptions::describe(&dtc.code);
            if !desc.is_empty() {
                dtc.description = desc.to_string();
            }
            dtc
        })
        .collect()
}

fn read_dtcs(elm: &mut elm327::Elm327, event_tx: &mpsc::Sender<ObdEvent>) {
    info!("[DTC_READ] Reading stored DTCs (Mode 03)");
    let stored = match elm.send_logged("03", Duration::from_secs(5)) {
        Ok(lines) => enrich_dtcs(obd::parse_dtc_response_lines(&lines, "43")),
        Err(e) => {
            let _ = event_tx.send(ObdEvent::Error(format!("Read stored DTCs failed: {e}")));
            Vec::new()
        }
    };

    info!("[DTC_READ] Reading pending DTCs (Mode 07)");
    let pending = match elm.send_logged("07", Duration::from_secs(5)) {
        Ok(lines) => enrich_dtcs(obd::parse_dtc_response_lines(&lines, "47")),
        Err(e) => {
            let _ = event_tx.send(ObdEvent::Error(format!("Read pending DTCs failed: {e}")));
            Vec::new()
        }
    };

    let _ = event_tx.send(ObdEvent::DtcResult { stored, pending });
}

fn read_freeze_frame(
    elm: &mut elm327::Elm327,
    event_tx: &mpsc::Sender<ObdEvent>,
    pid_defs: &[PidDef],
) {
    info!("[FREEZE_FRAME] Reading freeze frame data (Mode 02)");

    // Freeze frame uses Mode 02 with same PIDs as Mode 01, plus frame number suffix
    let freeze_pids = [
        "0104", "0105", "0106", "0107", "010B", "010C", "010D", "010E", "010F",
        "0110", "0111", "012F", "0142",
    ];

    for pid01 in &freeze_pids {
        // Mode 02 command: replace first '01' with '02', append '00' for frame 0
        let cmd = format!("02{}00", &pid01[2..]);
        let pid_def = match pid_defs.iter().find(|p| p.cmd == *pid01) {
            Some(p) => p,
            None => continue,
        };

        match elm.send_logged(&cmd, Duration::from_secs(3)) {
            Ok(lines) => {
                // Response prefix is 42XX instead of 41XX
                let prefix_42 = format!("42{}", &pid01[2..4]);
                for line in &lines {
                    let clean = line.replace(' ', "").to_uppercase();
                    if let Some(pos) = clean.find(&prefix_42) {
                        let data_str = &clean[pos + prefix_42.len()..];
                        let mut bytes = Vec::new();
                        let mut i = 0;
                        while i + 1 < data_str.len() {
                            if let Ok(byte) = u8::from_str_radix(&data_str[i..i + 2], 16) {
                                bytes.push(byte);
                            }
                            i += 2;
                        }
                        if !bytes.is_empty() {
                            let value = obd::decode_pid(pid_def, &bytes);
                            let _ = event_tx.send(ObdEvent::FreezeFrameData {
                                pid_cmd: pid01.to_string(),
                                name: pid_def.description.to_string(),
                                value,
                                unit: pid_def.unit.to_string(),
                            });
                        }
                    }
                }
            }
            Err(_) => {
                // Freeze frame may not be available
            }
        }
    }
}