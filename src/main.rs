mod app;
mod dtc_database;
mod dtc_descriptions;
mod elm327;
mod gauges;
mod obd;
mod obd_ops;
mod vin_decoder;

// On WASM, only lib.rs (and its web_serial module) is used.
// The binary target still compiles for WASM but is empty.
#[cfg(target_arch = "wasm32")]
fn main() {}

#[cfg(not(target_arch = "wasm32"))]
use app::{ObdApp, ObdEvent, OdbCmd};
#[cfg(not(target_arch = "wasm32"))]
use dtc_database::DtcDatabase;
#[cfg(not(target_arch = "wasm32"))]
use elm327::ElmAdapter as _;
#[cfg(not(target_arch = "wasm32"))]
use std::sync::{Arc, Mutex, mpsc};
#[cfg(not(target_arch = "wasm32"))]
use std::thread;
#[cfg(not(target_arch = "wasm32"))]
use std::time::Duration;
#[cfg(not(target_arch = "wasm32"))]
use tracing::info;
#[cfg(not(target_arch = "wasm32"))]
use tracing_subscriber::{EnvFilter, fmt, layer::SubscriberExt, util::SubscriberInitExt};

#[cfg(not(target_arch = "wasm32"))]
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

    // ── DTC database ─────────────────────────────────────────────────────────
    let dtc_db = Arc::new(match dtc_database::find_database_path() {
        Some(path) => {
            let db = DtcDatabase::load(&path);
            if db.is_loaded() {
                info!(path, makes = db.make_count(), codes = db.code_count(), "Loaded DTC database");
            } else {
                info!(path, "dtc_codes.json found but empty or unreadable");
            }
            db
        }
        None => {
            info!("No dtc_codes.json found — run scripts/fetch_dtc_codes.py to enable manufacturer-specific descriptions");
            DtcDatabase::default()
        }
    });

    // ── Channels ────────────────────────────────────────────────────────────
    let (cmd_tx, cmd_rx) = mpsc::channel::<OdbCmd>();
    let (event_tx, event_rx) = mpsc::channel::<ObdEvent>();

    // ── OBD background thread ───────────────────────────────────────────────
    let dtc_db_worker = dtc_db.clone();
    let obd_thread = thread::spawn(move || {
        obd_worker(cmd_rx, event_tx, dtc_db_worker);
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
        Box::new(move |cc| {
            Ok(Box::new(ObdApp::new(
                cc,
                cmd_tx_clone,
                event_rx,
                Some(log_file_clone),
            )))
        }),
    )
    .unwrap();

    // Shutdown
    info!("GUI closed, shutting down");
    let _ = cmd_tx.send(OdbCmd::Shutdown);
    let _ = obd_thread.join();
}

// ── OBD worker thread ───────────────────────────────────────────────────────

/// Holds either a real serial ELM327 or (in debug builds) a WebSocket emulator connection.
#[cfg(not(target_arch = "wasm32"))]
enum AnyElm {
    Serial(elm327::Elm327),
    #[cfg(debug_assertions)]
    Ws(elm327::WsElm327),
}

#[cfg(not(target_arch = "wasm32"))]
impl elm327::ElmAdapter for AnyElm {
    async fn send(&mut self, cmd: &str, timeout_ms: u64) -> Result<Vec<String>, elm327::Elm327Error> {
        match self {
            Self::Serial(e) => e.send(cmd, timeout_ms).await,
            #[cfg(debug_assertions)]
            Self::Ws(e) => e.send(cmd, timeout_ms).await,
        }
    }
    async fn sleep_ms(&mut self, ms: u64) {
        std::thread::sleep(std::time::Duration::from_millis(ms));
    }
    fn info(&self) -> &elm327::ConnectionInfo {
        match self {
            Self::Serial(e) => e.info(),
            #[cfg(debug_assertions)]
            Self::Ws(e) => e.info(),
        }
    }
    fn info_mut(&mut self) -> &mut elm327::ConnectionInfo {
        match self {
            Self::Serial(e) => e.info_mut(),
            #[cfg(debug_assertions)]
            Self::Ws(e) => e.info_mut(),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn obd_worker(cmd_rx: mpsc::Receiver<OdbCmd>, event_tx: mpsc::Sender<ObdEvent>, dtc_db: Arc<DtcDatabase>) {
    use app::PollConfig;

    let mut elm: Option<AnyElm> = None;
    let mut live_running = false;
    let mut poll_config = PollConfig::default();
    let mut current_make: Option<String> = None;

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
                    let _ =
                        event_tx.send(ObdEvent::Connecting("Scanning for OBD adapter...".into()));

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
                            elm = Some(AnyElm::Serial(device));
                            let _ = event_tx.send(ObdEvent::Connected(info));

                            // Read voltage + VIN on connect
                            if let Some(ref mut e) = elm {
                                if let Ok(v) = elm327::block_on(e.read_voltage()) {
                                    let _ = event_tx.send(ObdEvent::Voltage(v));
                                }
                                elm327::block_on(obd_ops::read_vin(e, &event_tx));
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

                OdbCmd::ReadDtcs { make } => {
                    if make.is_some() {
                        current_make = make;
                    }
                    if let Some(ref mut e) = elm {
                        let (stored, pending) = elm327::block_on(obd_ops::read_dtcs(e, &event_tx));
                        let tx2    = event_tx.clone();
                        let make2  = current_make.clone();
                        let db2    = dtc_db.clone();
                        thread::spawn(move || {
                            let _ = tx2.send(ObdEvent::DtcDescriptionsReady {
                                stored:  enrich_dtcs(stored,  make2.as_deref(), &db2),
                                pending: enrich_dtcs(pending, make2.as_deref(), &db2),
                            });
                        });
                    }
                }

                OdbCmd::ClearDtcs => {
                    if let Some(ref mut e) = elm {
                        info!("[DTC_CLEAR] Clearing DTCs");
                        let (stored, pending) = elm327::block_on(obd_ops::clear_dtcs(e, &event_tx));
                        let tx2    = event_tx.clone();
                        let make2  = current_make.clone();
                        let db2    = dtc_db.clone();
                        thread::spawn(move || {
                            let _ = tx2.send(ObdEvent::DtcDescriptionsReady {
                                stored:  enrich_dtcs(stored,  make2.as_deref(), &db2),
                                pending: enrich_dtcs(pending, make2.as_deref(), &db2),
                            });
                        });
                    }
                }

                OdbCmd::ReadFreezeFrame => {
                    if let Some(ref mut e) = elm {
                        elm327::block_on(obd_ops::read_freeze_frame(e, &event_tx, &pid_defs));
                    }
                }

                OdbCmd::ReadVin => {
                    if let Some(ref mut e) = elm {
                        elm327::block_on(obd_ops::read_vin(e, &event_tx));
                    }
                }

                OdbCmd::SetPollConfig(config) => {
                    info!(mode = ?config.mode, cycle_delay = config.cycle_delay_ms, inter_pid_delay = config.inter_pid_delay_ms, "Poll config updated");
                    poll_config = config;
                }

                OdbCmd::QuerySupportedPids => {
                    if let Some(ref mut e) = elm {
                        elm327::block_on(obd_ops::query_supported_pids(e, &event_tx));
                    }
                }

                OdbCmd::ConnectLocal { ws_port } => {
                    #[cfg(debug_assertions)]
                    {
                        let addr = format!("127.0.0.1:{ws_port}");
                        let _ = event_tx.send(ObdEvent::Connecting(
                            format!("Connecting to ws://{addr}…"),
                        ));
                        match elm327::WsElm327::connect(&addr) {
                            Ok(mut ws_elm) => {
                                let init_tx = event_tx.clone();
                                match elm327::block_on(obd_ops::init_elm(&mut ws_elm, move |msg| {
                                    let _ = init_tx.send(ObdEvent::Connecting(msg.to_string()));
                                })) {
                                    Ok(()) => {
                                        let info = ws_elm.info.clone();
                                        elm = Some(AnyElm::Ws(ws_elm));
                                        let _ = event_tx.send(ObdEvent::Connected(info));
                                        if let Some(ref mut e) = elm {
                                            if let Ok(v) = elm327::block_on(e.read_voltage()) {
                                                let _ = event_tx.send(ObdEvent::Voltage(v));
                                            }
                                            elm327::block_on(obd_ops::read_vin(e, &event_tx));
                                        }
                                    }
                                    Err(e) => {
                                        let _ = event_tx
                                            .send(ObdEvent::ConnectionFailed(e.to_string()));
                                    }
                                }
                            }
                            Err(e) => {
                                let _ =
                                    event_tx.send(ObdEvent::ConnectionFailed(e.to_string()));
                            }
                        }
                    }
                    #[cfg(not(debug_assertions))]
                    drop(ws_port);
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
                elm327::block_on(obd_ops::poll_live_data(e, &event_tx, &pid_defs, &poll_config));

                // Also poll voltage periodically (every poll cycle includes it)
                if let Ok(v) = elm327::block_on(e.read_voltage()) {
                    let _ = event_tx.send(ObdEvent::Voltage(v));
                }

                if poll_config.cycle_delay_ms > 0 {
                    std::thread::sleep(Duration::from_millis(poll_config.cycle_delay_ms));
                }
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn enrich_dtcs(dtcs: Vec<obd::Dtc>, make: Option<&str>, db: &DtcDatabase) -> Vec<obd::Dtc> {
    dtcs.into_iter()
        .map(|mut dtc| {
            // Try manufacturer DB (direct match, then same-family alias group).
            if let Some(m) = make {
                if let Some((desc, alias_src)) = db.lookup_with_source(m, &dtc.code) {
                    dtc.description = desc.to_string();
                    dtc.desc_source = match alias_src {
                        None    => obd::DescSource::Own,
                        Some(a) => obd::DescSource::Family(title_case(a)),
                    };
                    return dtc;
                }
            }
            // SAE J2012 generic fallback.
            let sae = dtc_descriptions::describe(&dtc.code);
            if !sae.is_empty() {
                dtc.description = sae.to_string();
                dtc.desc_source = obd::DescSource::Sae;
            } else {
                dtc.desc_source = obd::DescSource::NotFound;
            }
            dtc
        })
        .collect()
}

#[cfg(not(target_arch = "wasm32"))]
fn title_case(s: &str) -> String {
    let mut t = s.to_string();
    if let Some(c) = t.get_mut(0..1) {
        c.make_ascii_uppercase();
    }
    t
}

