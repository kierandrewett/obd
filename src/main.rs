mod adapter;
mod app;
mod dtc_database;
mod dtc_descriptions;
mod elm327;
#[cfg(not(target_arch = "wasm32"))]
#[allow(dead_code)] // The binary uses a subset; the library copy exposes parser metadata to tests.
mod freematics_usb;
mod gauges;
mod obd;
mod obd_ops;
mod vin_decoder;

// On WASM, only lib.rs (and its web_serial module) is used.
// The binary target still compiles for WASM but is empty.
#[cfg(target_arch = "wasm32")]
fn main() {}

#[cfg(not(target_arch = "wasm32"))]
use adapter::DiagnosticAdapter as _;
#[cfg(not(target_arch = "wasm32"))]
use app::{ObdApp, ObdEvent, OdbCmd};
#[cfg(not(target_arch = "wasm32"))]
use dtc_database::DtcDatabase;
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
                info!(
                    path,
                    makes = db.make_count(),
                    codes = db.code_count(),
                    "Loaded DTC database"
                );
            } else {
                info!(path, "dtc_codes.json found but empty or unreadable");
            }
            db
        }
        None => {
            info!(
                "No dtc_codes.json found — run scripts/fetch_dtc_codes.py to enable manufacturer-specific descriptions"
            );
            DtcDatabase::default()
        }
    });

    // ── Channels ────────────────────────────────────────────────────────────
    let (cmd_tx, cmd_rx) = mpsc::channel::<OdbCmd>();
    let (event_tx, event_rx) = mpsc::channel::<ObdEvent>();
    let (telemetry_tx, telemetry_rx) = mpsc::sync_channel::<freematics_usb::FreematicsFrame>(8);

    // ── OBD background thread ───────────────────────────────────────────────
    let dtc_db_worker = dtc_db.clone();
    let obd_thread = thread::spawn(move || {
        obd_worker(cmd_rx, event_tx, telemetry_tx, dtc_db_worker);
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
                telemetry_rx,
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

/// Native adapters share diagnostic operations without emulating ELM commands.
#[cfg(not(target_arch = "wasm32"))]
enum AnyAdapter {
    Serial(elm327::Elm327),
    Tcp(elm_tcp::TcpElm),
    J2534(j2534::J2534),
    #[cfg(debug_assertions)]
    Ws(elm327::WsElm327),
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Default)]
struct TelemetryReaderDropCounter {
    cumulative: u64,
}

#[cfg(not(target_arch = "wasm32"))]
impl TelemetryReaderDropCounter {
    fn total(&self) -> u64 {
        self.cumulative
    }

    fn dropped(&mut self) {
        self.cumulative = self.cumulative.saturating_add(1);
    }

    fn reset(&mut self) {
        self.cumulative = 0;
    }
}

#[cfg(not(target_arch = "wasm32"))]
enum TelemetryEnqueueResult {
    Enqueued,
    Dropped,
    Disconnected,
}

#[cfg(not(target_arch = "wasm32"))]
fn enqueue_telemetry_frame(
    mut frame: freematics_usb::FreematicsFrame,
    telemetry_tx: &mpsc::SyncSender<freematics_usb::FreematicsFrame>,
    reader_drops: &mut TelemetryReaderDropCounter,
) -> TelemetryEnqueueResult {
    frame.reader_drops = reader_drops.total();
    match telemetry_tx.try_send(frame) {
        Err(mpsc::TrySendError::Full(_)) => {
            reader_drops.dropped();
            TelemetryEnqueueResult::Dropped
        }
        Ok(()) => TelemetryEnqueueResult::Enqueued,
        Err(mpsc::TrySendError::Disconnected(_)) => TelemetryEnqueueResult::Disconnected,
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl adapter::DiagnosticAdapter for AnyAdapter {
    async fn request(
        &mut self,
        payload: &[u8],
        timeout_ms: u64,
    ) -> Result<Vec<adapter::DiagnosticResponse>, elm327::Elm327Error> {
        match self {
            Self::Serial(e) => e.request(payload, timeout_ms).await,
            Self::Tcp(e) => e.request(payload, timeout_ms).await,
            Self::J2534(e) => e.request(payload, timeout_ms).await,
            #[cfg(debug_assertions)]
            Self::Ws(e) => e.request(payload, timeout_ms).await,
        }
    }
    fn connection_info(&self) -> &elm327::ConnectionInfo {
        match self {
            Self::Serial(e) => e.connection_info(),
            Self::Tcp(e) => e.connection_info(),
            Self::J2534(e) => e.connection_info(),
            #[cfg(debug_assertions)]
            Self::Ws(e) => e.connection_info(),
        }
    }
    async fn voltage(&mut self) -> Result<String, elm327::Elm327Error> {
        match self {
            Self::Serial(e) => e.voltage().await,
            Self::Tcp(e) => e.voltage().await,
            Self::J2534(e) => e.voltage().await,
            #[cfg(debug_assertions)]
            Self::Ws(e) => e.voltage().await,
        }
    }
    async fn delay(&mut self, ms: u64) {
        std::thread::sleep(Duration::from_millis(ms));
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn obd_worker(
    cmd_rx: mpsc::Receiver<OdbCmd>,
    event_tx: mpsc::Sender<ObdEvent>,
    telemetry_tx: mpsc::SyncSender<freematics_usb::FreematicsFrame>,
    dtc_db: Arc<DtcDatabase>,
) {
    use app::PollConfig;

    let mut elm: Option<AnyAdapter> = None;
    let mut freematics: Option<freematics_usb::FreematicsUsb> = None;
    let mut freematics_pending_connection: Option<elm327::ConnectionInfo> = None;
    let mut reader_drops = TelemetryReaderDropCounter::default();
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
                OdbCmd::Connect {
                    port,
                    baud,
                    elm_can_mode,
                } => {
                    elm = None;
                    freematics = None;
                    freematics_pending_connection = None;
                    live_running = false;
                    current_make = None;
                    let _ =
                        event_tx.send(ObdEvent::Connecting("Scanning for OBD adapter...".into()));

                    let progress_tx = event_tx.clone();
                    let progress = move |msg: &str| {
                        let _ = progress_tx.send(ObdEvent::Connecting(msg.to_string()));
                    };

                    let result = if let Some(port_name) = port {
                        elm327::connect_with_mode(&port_name, baud, elm_can_mode, Some(&progress))
                    } else {
                        if elm_can_mode == elm327::ElmCanMode::Auto {
                            elm327::auto_connect(Some(&progress))
                        } else {
                            Err(elm327::Elm327Error::InitFailed(
                                "The Corsa D MS-CAN profile requires a selected serial port and baud rate".into(),
                            ))
                        }
                    };

                    finish_connection(result.map(AnyAdapter::Serial), &mut elm, &event_tx);
                }

                OdbCmd::ConnectAdapter(config) => {
                    elm = None;
                    freematics = None;
                    freematics_pending_connection = None;
                    live_running = false;
                    current_make = None;
                    let _ = event_tx.send(ObdEvent::Connecting("Connecting to adapter...".into()));
                    let result = match config {
                        app::NativeConnection::Tcp(address) => {
                            elm_tcp::TcpElm::connect(&address, |message| {
                                let _ = event_tx.send(ObdEvent::Connecting(message.into()));
                            })
                            .map(AnyAdapter::Tcp)
                        }
                        app::NativeConnection::J2534 {
                            library,
                            protocol,
                            ecu_address,
                        } => j2534::J2534::connect_to(
                            std::path::Path::new(&library),
                            protocol,
                            ecu_address,
                        )
                        .map(AnyAdapter::J2534),
                    };
                    finish_connection(result, &mut elm, &event_tx);
                }

                OdbCmd::ConnectFreematicsUsb(port) => {
                    elm = None;
                    freematics = None;
                    freematics_pending_connection = None;
                    live_running = false;
                    current_make = None;
                    let _ = event_tx.send(ObdEvent::Connecting(
                        "Opening passive Freematics USB telemetry...".into(),
                    ));
                    let result = if let Some(port) = port {
                        freematics_usb::FreematicsUsb::connect(&port)
                            .map(|device| (device, Vec::new()))
                    } else {
                        freematics_usb::FreematicsUsb::auto_connect()
                    };
                    match result {
                        Ok((device, initial_frames)) => {
                            let info = elm327::ConnectionInfo {
                                port: device.port_name(),
                                baud: device.baud_rate(),
                                protocol: "Freematics Telemetry v2".into(),
                                elm_version: "Passive TeleLogger USB stream".into(),
                                voltage: None,
                            };
                            let _ = event_tx.send(ObdEvent::Connecting(format!(
                                "Listening on {} for a checksummed Freematics telemetry frame; port stays open",
                                info.port
                            )));
                            freematics_pending_connection = Some(info);
                            freematics = Some(device);
                            reader_drops.reset();
                            for frame in initial_frames {
                                if let Some(info) = freematics_pending_connection.take() {
                                    let _ = event_tx.send(ObdEvent::Connected(info));
                                }
                                let _ = enqueue_telemetry_frame(
                                    frame,
                                    &telemetry_tx,
                                    &mut reader_drops,
                                );
                            }
                        }
                        Err(error) => {
                            let _ = event_tx.send(ObdEvent::ConnectionFailed(error));
                        }
                    }
                }

                OdbCmd::Disconnect => {
                    elm = None;
                    freematics = None;
                    freematics_pending_connection = None;
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
                        let tx2 = event_tx.clone();
                        let make2 = current_make.clone();
                        let db2 = dtc_db.clone();
                        thread::spawn(move || {
                            let _ = tx2.send(ObdEvent::DtcDescriptionsReady {
                                stored: enrich_dtcs(stored, make2.as_deref(), &db2),
                                pending: enrich_dtcs(pending, make2.as_deref(), &db2),
                            });
                        });
                    }
                }

                OdbCmd::ClearDtcs => {
                    if let Some(ref mut e) = elm {
                        info!("[DTC_CLEAR] Clearing DTCs");
                        let (stored, pending) = elm327::block_on(obd_ops::clear_dtcs(e, &event_tx));
                        let tx2 = event_tx.clone();
                        let make2 = current_make.clone();
                        let db2 = dtc_db.clone();
                        thread::spawn(move || {
                            let _ = tx2.send(ObdEvent::DtcDescriptionsReady {
                                stored: enrich_dtcs(stored, make2.as_deref(), &db2),
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
                    elm = None;
                    freematics = None;
                    freematics_pending_connection = None;
                    live_running = false;
                    current_make = None;
                    #[cfg(debug_assertions)]
                    {
                        let addr = format!("127.0.0.1:{ws_port}");
                        let _ = event_tx
                            .send(ObdEvent::Connecting(format!("Connecting to ws://{addr}…")));
                        match elm327::WsElm327::connect(&addr) {
                            Ok(mut ws_elm) => {
                                let init_tx = event_tx.clone();
                                match elm327::block_on(obd_ops::init_elm(&mut ws_elm, move |msg| {
                                    let _ = init_tx.send(ObdEvent::Connecting(msg.to_string()));
                                })) {
                                    Ok(()) => finish_connection(
                                        Ok(AnyAdapter::Ws(ws_elm)),
                                        &mut elm,
                                        &event_tx,
                                    ),
                                    Err(e) => {
                                        let _ = event_tx
                                            .send(ObdEvent::ConnectionFailed(e.to_string()));
                                    }
                                }
                            }
                            Err(e) => {
                                let _ = event_tx.send(ObdEvent::ConnectionFailed(e.to_string()));
                            }
                        }
                    }
                    #[cfg(not(debug_assertions))]
                    let _ = ws_port;
                }

                OdbCmd::Shutdown => {
                    info!("OBD worker shutting down");
                    break;
                }
            }
        }

        let mut telemetry_receiver_closed = false;
        if let Some(device) = &mut freematics {
            match device.read_frames() {
                Ok(frames) => {
                    for frame in frames {
                        if let Some(info) = freematics_pending_connection.take() {
                            let _ = event_tx.send(ObdEvent::Connected(info));
                        }
                        match enqueue_telemetry_frame(frame, &telemetry_tx, &mut reader_drops) {
                            TelemetryEnqueueResult::Enqueued | TelemetryEnqueueResult::Dropped => {}
                            TelemetryEnqueueResult::Disconnected => {
                                telemetry_receiver_closed = true;
                                break;
                            }
                        }
                    }
                }
                Err(error) => {
                    if freematics_pending_connection.take().is_some() {
                        let _ = event_tx.send(ObdEvent::ConnectionFailed(format!(
                            "Freematics port opened but telemetry read failed: {error}"
                        )));
                    } else {
                        let _ = event_tx.send(ObdEvent::Error(error));
                        let _ = event_tx.send(ObdEvent::Disconnected);
                    }
                    telemetry_receiver_closed = true;
                }
            }
        }
        if telemetry_receiver_closed {
            freematics = None;
        }

        // Live data polling
        if live_running {
            if let Some(ref mut e) = elm {
                elm327::block_on(obd_ops::poll_live_data(
                    e,
                    &event_tx,
                    &pid_defs,
                    &poll_config,
                ));

                // Also poll voltage periodically (every poll cycle includes it)
                if let Ok(v) = elm327::block_on(e.voltage()) {
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
                        None => obd::DescSource::Own,
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

#[cfg(not(target_arch = "wasm32"))]
mod elm_tcp;
#[cfg(not(target_arch = "wasm32"))]
mod j2534;

#[cfg(not(target_arch = "wasm32"))]
fn finish_connection(
    result: Result<AnyAdapter, elm327::Elm327Error>,
    adapter: &mut Option<AnyAdapter>,
    events: &mpsc::Sender<ObdEvent>,
) {
    match result {
        Ok(mut device) => {
            let _ = events.send(ObdEvent::Connected(device.connection_info().clone()));
            if let Ok(voltage) = elm327::block_on(device.voltage()) {
                let _ = events.send(ObdEvent::Voltage(voltage));
            }
            elm327::block_on(obd_ops::read_vin(&mut device, events));
            *adapter = Some(device);
        }
        Err(error) => {
            let _ = events.send(ObdEvent::ConnectionFailed(error.to_string()));
        }
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod telemetry_reader_drop_tests {
    use super::{TelemetryEnqueueResult, TelemetryReaderDropCounter, enqueue_telemetry_frame};
    use crate::freematics_usb::FreematicsFrame;
    use std::collections::{HashMap, HashSet};
    use std::sync::mpsc;

    fn frame() -> FreematicsFrame {
        FreematicsFrame {
            boot_id: 7,
            capture_ms: 0,
            reader_received_at: std::time::Instant::now(),
            capture_utc_ms: None,
            dropped_records: 0,
            supported_pids: Some(HashSet::new()),
            raw_mode01: HashMap::new(),
            vin: None,
            calibration_id: None,
            ecu_name: None,
            fields: Vec::new(),
            corrupt_records: 0,
            corrupt_sample_hex: None,
            reader_drops: 0,
        }
    }

    #[test]
    fn successful_delivery_does_not_erase_the_cumulative_drop_total() {
        let (sender, receiver) = mpsc::sync_channel(1);
        sender.try_send(frame()).unwrap();
        let mut drops = TelemetryReaderDropCounter::default();
        assert!(matches!(
            enqueue_telemetry_frame(frame(), &sender, &mut drops),
            TelemetryEnqueueResult::Dropped
        ));
        assert_eq!(drops.total(), 1);
        receiver.try_recv().unwrap();
        assert!(matches!(
            enqueue_telemetry_frame(frame(), &sender, &mut drops),
            TelemetryEnqueueResult::Enqueued
        ));
        assert_eq!(receiver.try_recv().unwrap().reader_drops, 1);
        assert_eq!(
            drops.total(),
            1,
            "the count is cumulative for the connection"
        );
    }
}
