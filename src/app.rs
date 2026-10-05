use crate::elm327::ConnectionInfo;
use crate::gauges::RadialGauge;
use crate::obd::{self, DescSource, Dtc, ObdValue, PidDef};
use egui::{self, Color32, RichText};
use std::collections::HashMap;
#[cfg(not(target_arch = "wasm32"))]
use std::io::Write;
use std::sync::mpsc;
#[cfg(not(target_arch = "wasm32"))]
use std::sync::{Arc, Mutex};

// ── Cross-platform clipboard ──────────────────────────────────────────────────

/// Write `text` to the system clipboard.
///
/// On native this delegates to egui's arboard-backed clipboard.
/// On web, `navigator.clipboard` requires HTTPS; we also provide an
/// `execCommand` fallback so it works on plain HTTP (dev servers etc.).
fn platform_copy(ctx: &egui::Context, text: &str) {
    #[cfg(not(target_arch = "wasm32"))]
    ctx.copy_text(text.to_string());

    #[cfg(target_arch = "wasm32")]
    {
        let _ = ctx;
        web_clipboard_write(text);
    }
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen::prelude::wasm_bindgen(inline_js = r#"
export function web_clipboard_write(text) {
    if (navigator.clipboard && window.isSecureContext) {
        navigator.clipboard.writeText(text).catch(function() { fallback(text); });
    } else {
        fallback(text);
    }
    function fallback(t) {
        var el = document.createElement('textarea');
        el.value = t;
        el.style.cssText = 'position:fixed;top:0;left:0;width:1px;height:1px;opacity:0';
        document.body.appendChild(el);
        el.focus();
        el.select();
        try { document.execCommand('copy'); } catch (_) {}
        document.body.removeChild(el);
    }
}
"#)]
extern "C" {
    fn web_clipboard_write(text: &str);
}
use std::time::Duration;
use std::time::Instant;

// ── Messages between OBD thread and GUI ─────────────────────────────────────

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug)]
pub enum NativeConnection {
    Tcp(String),
    J2534 {
        library: String,
        protocol: crate::j2534::CanProtocol,
        ecu_address: u8,
    },
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Default, PartialEq, Eq)]
enum ConnectionKind {
    #[default]
    Serial,
    FreematicsUsb,
    Tcp,
    J2534,
}

#[derive(Debug)]
pub enum OdbCmd {
    Connect {
        port: Option<String>,
        baud: Option<u32>,
        #[cfg(not(target_arch = "wasm32"))]
        elm_can_mode: crate::elm327::ElmCanMode,
    },
    #[cfg(not(target_arch = "wasm32"))]
    ConnectFreematicsUsb(Option<String>),
    #[cfg(not(target_arch = "wasm32"))]
    ConnectAdapter(NativeConnection),
    /// Connect to a local OBD emulator via WebSocket (web only).
    #[cfg(any(target_arch = "wasm32", debug_assertions))]
    ConnectLocal {
        ws_port: u16,
    },
    Disconnect,
    StartLiveData,
    StopLiveData,
    ReadDtcs {
        make: Option<String>,
    },
    ClearDtcs,
    ReadFreezeFrame,
    ReadVin,
    QuerySupportedPids,
    SetPollConfig(PollConfig),
    Shutdown,
}

#[derive(Debug, Clone)]
pub struct PollConfig {
    /// Which PID set to poll
    pub mode: PollMode,
    /// Delay between individual PID requests in ms (0 = as fast as possible)
    pub inter_pid_delay_ms: u64,
    /// Delay between full poll cycles in ms
    pub cycle_delay_ms: u64,
}

impl Default for PollConfig {
    fn default() -> Self {
        Self {
            mode: PollMode::Fast,
            inter_pid_delay_ms: 0,
            cycle_delay_ms: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PollMode {
    /// Only RPM, Speed, Throttle, Load (highest refresh rate)
    Minimal,
    /// Core driving PIDs: RPM, Speed, Throttle, Load, Coolant, Intake, MAF
    Fast,
    /// All commonly useful PIDs
    Full,
}

#[derive(Debug, Clone)]
pub enum ObdEvent {
    Connecting(String),
    Connected(ConnectionInfo),
    ConnectionFailed(String),
    Disconnected,
    LiveData {
        pid_cmd: String,
        name: String,
        value: ObdValue,
        unit: String,
        raw: String,
    },
    DtcResult {
        stored: Vec<Dtc>,
        pending: Vec<Dtc>,
    },
    /// Descriptions enriched in background — replaces DtcResult lists silently.
    DtcDescriptionsReady {
        stored: Vec<Dtc>,
        pending: Vec<Dtc>,
    },
    FreezeFrameData {
        pid_cmd: String,
        name: String,
        value: ObdValue,
        unit: String,
    },
    Vin(String),
    SupportedPids(Vec<u8>),
    Voltage(String),
    Error(String),
    LogMessage(String),
}

// ── App Tab ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    Dashboard,
    Sensors,
    DtcCodes,
    FreezeFrame,
    VehicleInfo,
}

// ── App State ───────────────────────────────────────────────────────────────

pub struct ObdApp {
    // Communication
    cmd_tx: mpsc::Sender<OdbCmd>,
    event_rx: mpsc::Receiver<ObdEvent>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_rx: mpsc::Receiver<crate::freematics_usb::FreematicsFrame>,

    // Connection state
    connected: bool,
    connecting: bool,
    connection_info: Option<ConnectionInfo>,
    connection_status: String,

    #[cfg(not(target_arch = "wasm32"))]
    connection_kind: ConnectionKind,
    #[cfg(not(target_arch = "wasm32"))]
    tcp_address: String,
    #[cfg(not(target_arch = "wasm32"))]
    j2534_library: String,
    #[cfg(not(target_arch = "wasm32"))]
    j2534_protocol: crate::j2534::CanProtocol,
    #[cfg(not(target_arch = "wasm32"))]
    j2534_ecu_address: u8,
    #[cfg(not(target_arch = "wasm32"))]
    j2534_drivers: Vec<crate::j2534::DriverInfo>,
    #[cfg(not(target_arch = "wasm32"))]
    elm_can_mode: crate::elm327::ElmCanMode,

    // Port selection
    available_ports: Vec<String>,
    selected_port: Option<String>,
    selected_baud: Option<u32>,
    #[allow(dead_code)]
    auto_connect: bool,
    #[cfg(any(target_arch = "wasm32", debug_assertions))]
    emulator_port: u16,

    // Live data
    live_data: HashMap<String, LivePidState>,
    live_running: bool,
    supported_pids: Vec<u8>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_support_reported: bool,

    // DTCs
    stored_dtcs: Vec<Dtc>,
    pending_dtcs: Vec<Dtc>,
    dtc_status: String,
    clear_dtc_confirm: bool,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_dtcs: [FreematicsDtcScan; 3],
    #[cfg(not(target_arch = "wasm32"))]
    freematics_freeze_data: Vec<crate::freematics_usb::FreematicsMeasurement>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_freeze_status: Option<u32>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_freeze_read_age_ms: Option<u32>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_freeze_trigger_dtc: Option<u32>,

    // Freeze frame
    freeze_data: Vec<(String, ObdValue, String)>,
    freeze_frame_read: bool,

    // Vehicle info
    vin: Option<String>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_calibration_id: Option<String>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_ecu_name: Option<String>,
    voltage: Option<String>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_boot_id: Option<u64>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_capture_ms: Option<u32>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_capture_utc_ms: Option<i64>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_clock_anchor: Option<FreematicsClockAnchor>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_last_frame_received_at: Option<Instant>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_supply: Option<(f64, Option<u32>, Instant)>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_supply_history: Vec<HistoryPoint>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_motion_history: Vec<HistoryPoint>,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_acquisition_health: crate::freematics_usb::FreematicsAcquisitionHealth,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_dropped_records: u32,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_reader_drops: u64,
    #[cfg(not(target_arch = "wasm32"))]
    freematics_corrupt_records: u64,

    // UI state
    active_tab: Tab,
    log_messages: Vec<String>,
    log_auto_scroll: bool,
    log_panel_open: bool,
    log_panel_height: f32,
    log_last_count: usize,
    poll_config: PollConfig,
    dark_mode: bool,

    // PID definitions
    pid_defs: Vec<PidDef>,

    // Log file writer (desktop only)
    #[cfg(not(target_arch = "wasm32"))]
    log_file: Option<Arc<Mutex<std::fs::File>>>,

    // Screen wake lock — desktop only (std::process not available on wasm32)
    #[cfg(not(target_arch = "wasm32"))]
    wake_lock: Option<std::process::Child>,
}

#[cfg(not(target_arch = "wasm32"))]
struct FreematicsDtcScan {
    availability: crate::freematics_usb::FreematicsDtcAvailability,
    status: Option<u8>,
    count: Option<u8>,
    age_ms: Option<u32>,
    received_at: Option<Instant>,
    codes: Vec<Dtc>,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FreematicsDtcState {
    NotReported,
    NeverScanned,
    Unknown,
    NoResponse,
    RespondedNoCodes,
    Codes,
    InvalidCodes,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FreematicsDtcPresentation {
    state: FreematicsDtcState,
    count: Option<u8>,
    age_ms: Option<u64>,
    stale: bool,
}

#[cfg(not(target_arch = "wasm32"))]
fn freematics_sample_utc_ms(
    frame_capture_ms: u32,
    frame_capture_utc_ms: Option<i64>,
    sample_capture_ms: u32,
) -> Option<i64> {
    let frame_utc_ms = frame_capture_utc_ms?;
    let offset_ms = sample_capture_ms.wrapping_sub(frame_capture_ms) as i32 as i64;
    frame_utc_ms.checked_add(offset_ms)
}

#[cfg(not(target_arch = "wasm32"))]
fn freematics_dtc_presentation(
    scan: &FreematicsDtcScan,
    now: Instant,
) -> FreematicsDtcPresentation {
    const STALE_AFTER_MS: u64 = 120_000;
    let age_ms = scan.age_ms(now);
    let state = match scan.availability {
        crate::freematics_usb::FreematicsDtcAvailability::Unsupported => {
            FreematicsDtcState::NotReported
        }
        crate::freematics_usb::FreematicsDtcAvailability::NoScan => {
            FreematicsDtcState::NeverScanned
        }
        crate::freematics_usb::FreematicsDtcAvailability::UnknownStatus => {
            FreematicsDtcState::Unknown
        }
        crate::freematics_usb::FreematicsDtcAvailability::Fresh
        | crate::freematics_usb::FreematicsDtcAvailability::Stale => match scan.status {
            Some(0) => FreematicsDtcState::NoResponse,
            Some(1) => FreematicsDtcState::RespondedNoCodes,
            Some(2) if scan.count == Some(0) => FreematicsDtcState::RespondedNoCodes,
            Some(2) if scan.codes.is_empty() => FreematicsDtcState::InvalidCodes,
            Some(2) => FreematicsDtcState::Codes,
            _ => FreematicsDtcState::Unknown,
        },
    };
    FreematicsDtcPresentation {
        state,
        count: scan.count,
        age_ms,
        stale: age_ms.is_some_and(|age| age > STALE_AFTER_MS),
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Clone, Copy)]
struct FreematicsClockAnchor {
    capture_ms: u32,
    received_at: Instant,
}

#[cfg(not(target_arch = "wasm32"))]
fn freematics_sample_instant(anchor: FreematicsClockAnchor, capture_ms: u32) -> Option<Instant> {
    let delta_ms = capture_ms.wrapping_sub(anchor.capture_ms) as i32;
    if delta_ms >= 0 {
        anchor
            .received_at
            .checked_add(Duration::from_millis(delta_ms as u64))
    } else {
        anchor
            .received_at
            .checked_sub(Duration::from_millis(delta_ms.unsigned_abs() as u64))
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn push_waveform_point(history: &mut Vec<HistoryPoint>, point: HistoryPoint) {
    history.push(point);
    const MAX_WAVEFORM_POINTS: usize = 3000;
    const TRIM_WAVEFORM_POINTS: usize = 512;
    if history.len() > MAX_WAVEFORM_POINTS {
        history.drain(..TRIM_WAVEFORM_POINTS);
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Default for FreematicsDtcScan {
    fn default() -> Self {
        Self {
            availability: crate::freematics_usb::FreematicsDtcAvailability::Unsupported,
            status: None,
            count: None,
            age_ms: None,
            received_at: None,
            codes: Vec::new(),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl FreematicsDtcScan {
    fn age_ms(&self, now: Instant) -> Option<u64> {
        Some(
            self.age_ms? as u64
                + now.saturating_duration_since(self.received_at?).as_millis() as u64,
        )
    }
}

struct LivePidState {
    name: String,
    value: ObdValue,
    unit: String,
    numeric_value: f64,
    history: Vec<HistoryPoint>,
    raw: String,
    age_ms: Option<u32>,
    supported: Option<bool>,
    received_at: Instant,
    capture_ms: Option<u32>,
}

#[derive(Debug, Clone, Copy)]
struct HistoryPoint {
    captured_at: Instant,
    /// Device capture wall-clock time when the firmware reports valid UTC.
    /// `captured_at` remains the monotonic ordering/freshness clock.
    #[allow(dead_code)] // Preserved for export/inspection; graph geometry uses monotonic time.
    capture_utc_ms: Option<i64>,
    value: f64,
}

#[derive(Clone, Copy)]
struct TimeSeriesConfig<'a> {
    label: &'a str,
    color: Color32,
    unit: &'a str,
    minimum_range: f64,
    decimals: usize,
    freshness: Duration,
    max_gap: Duration,
}

struct GaugeSpec {
    column: usize,
    pid: &'static str,
    label: &'static str,
    min: f64,
    max: f64,
    unit: &'static str,
    warning: Option<f64>,
    danger: Option<f64>,
    decimals: usize,
}

fn show_time_series(ui: &mut egui::Ui, state: &LivePidState, config: TimeSeriesConfig<'_>) {
    let age = state
        .age_ms
        .map(|age| Duration::from_millis(age as u64))
        .map(|age| age.saturating_add(state.received_at.elapsed()));
    show_time_series_values(ui, &state.history, state.numeric_value, age, config);
}

fn show_time_series_values(
    ui: &mut egui::Ui,
    history: &[HistoryPoint],
    numeric_value: f64,
    age: Option<Duration>,
    config: TimeSeriesConfig<'_>,
) {
    let now = Instant::now();
    let window = Duration::from_secs(60);
    let start = now - window;
    let samples: Vec<_> = history
        .iter()
        .copied()
        .filter(|point| point.captured_at >= start && point.captured_at <= now)
        .collect();

    ui.horizontal(|ui| {
        ui.label(RichText::new(config.label).strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!(
                    "{numeric_value:.precision$} {}",
                    config.unit,
                    precision = config.decimals
                ))
                .monospace()
                .color(config.color),
            );
            ui.label(
                RichText::new(match age {
                    Some(age) if age <= config.freshness => format!("{} ms", age.as_millis()),
                    Some(age) => format!("Stale · {} ms", age.as_millis()),
                    None => "Age unavailable".to_string(),
                })
                .small()
                .color(if age.is_none_or(|age| age > config.freshness) {
                    Color32::from_rgb(220, 170, 80)
                } else {
                    ui.visuals().weak_text_color()
                }),
            );
        });
    });

    let width = ui.available_width().max(180.0);
    let (rect, response) = ui.allocate_exact_size(egui::vec2(width, 156.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let bg = ui.visuals().extreme_bg_color;
    let grid = ui.visuals().widgets.noninteractive.bg_stroke.color;
    let muted = ui.visuals().weak_text_color();
    painter.rect_filled(rect, 2.0, bg);

    let plot = rect.shrink2(egui::vec2(42.0, 18.0));
    let values: Vec<_> = samples.iter().map(|point| point.value).collect();
    let (mut min, mut max) = if values.is_empty() {
        (0.0, config.minimum_range)
    } else {
        (
            values.iter().copied().fold(f64::INFINITY, f64::min),
            values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        )
    };
    let observed_range = max - min;
    if observed_range < config.minimum_range {
        let center = (min + max) / 2.0;
        min = center - config.minimum_range / 2.0;
        max = center + config.minimum_range / 2.0;
    }
    let padding = (max - min) * 0.08;
    min -= padding;
    max += padding;

    for fraction in [0.0_f32, 0.5, 1.0] {
        let y = egui::lerp(plot.bottom()..=plot.top(), fraction);
        painter.line_segment(
            [egui::pos2(plot.left(), y), egui::pos2(plot.right(), y)],
            egui::Stroke::new(0.5_f32, grid),
        );
        let value = min + (max - min) * fraction as f64;
        painter.text(
            egui::pos2(rect.left() + 2.0, y),
            egui::Align2::LEFT_CENTER,
            format!("{value:.precision$}", precision = config.decimals),
            egui::FontId::monospace(9.0),
            muted,
        );
    }
    painter.text(
        egui::pos2(plot.left(), rect.bottom() - 1.0),
        egui::Align2::LEFT_BOTTOM,
        "60s",
        egui::FontId::monospace(9.0),
        muted,
    );
    painter.text(
        egui::pos2(plot.right(), rect.bottom() - 1.0),
        egui::Align2::RIGHT_BOTTOM,
        "now",
        egui::FontId::monospace(9.0),
        muted,
    );

    let position = |point: HistoryPoint| {
        let elapsed = now.duration_since(point.captured_at).as_secs_f64();
        let x_fraction = (1.0 - elapsed / window.as_secs_f64()).clamp(0.0, 1.0) as f32;
        let y_fraction = ((point.value - min) / (max - min)).clamp(0.0, 1.0) as f32;
        egui::pos2(
            egui::lerp(plot.left()..=plot.right(), x_fraction),
            egui::lerp(plot.bottom()..=plot.top(), y_fraction),
        )
    };
    for pair in samples.windows(2) {
        if pair[1].captured_at.duration_since(pair[0].captured_at) <= config.max_gap {
            painter.line_segment(
                [position(pair[0]), position(pair[1])],
                egui::Stroke::new(2.0_f32, config.color),
            );
        }
    }

    if samples.len() < 2 {
        painter.text(
            plot.center(),
            egui::Align2::CENTER_CENTER,
            "Waiting for fresh samples",
            egui::FontId::proportional(11.0),
            muted,
        );
    }

    if let Some(pointer) = response.hover_pos()
        && plot.contains(pointer)
        && !samples.is_empty()
    {
        let nearest = samples
            .iter()
            .min_by(|a, b| {
                (position(**a).x - pointer.x)
                    .abs()
                    .total_cmp(&(position(**b).x - pointer.x).abs())
            })
            .expect("samples was checked as non-empty");
        response.on_hover_text(format!(
            "{:.1} {} · captured {:.2}s ago",
            nearest.value,
            config.unit,
            now.duration_since(nearest.captured_at).as_secs_f32()
        ));
    }
}

impl ObdApp {
    pub fn new(
        _cc: &eframe::CreationContext<'_>,
        cmd_tx: mpsc::Sender<OdbCmd>,
        event_rx: mpsc::Receiver<ObdEvent>,
        #[cfg(not(target_arch = "wasm32"))] freematics_rx: mpsc::Receiver<
            crate::freematics_usb::FreematicsFrame,
        >,
        #[cfg(not(target_arch = "wasm32"))] log_file: Option<Arc<Mutex<std::fs::File>>>,
    ) -> Self {
        Self::new_state(
            cmd_tx,
            event_rx,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_rx,
            #[cfg(not(target_arch = "wasm32"))]
            log_file,
        )
    }

    fn new_state(
        cmd_tx: mpsc::Sender<OdbCmd>,
        event_rx: mpsc::Receiver<ObdEvent>,
        #[cfg(not(target_arch = "wasm32"))] freematics_rx: mpsc::Receiver<
            crate::freematics_usb::FreematicsFrame,
        >,
        #[cfg(not(target_arch = "wasm32"))] log_file: Option<Arc<Mutex<std::fs::File>>>,
    ) -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        let available_ports = crate::elm327::scan_ports();
        #[cfg(target_arch = "wasm32")]
        let available_ports: Vec<String> = Vec::new();

        let pid_defs = obd::mode01_pids();
        #[cfg(not(target_arch = "wasm32"))]
        let initial_connection_kind = if available_ports
            .iter()
            .any(|port| is_usb_serial_port_name(port))
        {
            ConnectionKind::FreematicsUsb
        } else {
            ConnectionKind::default()
        };

        Self {
            cmd_tx,
            event_rx,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_rx,
            connected: false,
            connecting: false,
            connection_info: None,
            connection_status: "Disconnected".to_string(),
            #[cfg(not(target_arch = "wasm32"))]
            connection_kind: initial_connection_kind,
            #[cfg(not(target_arch = "wasm32"))]
            tcp_address: String::new(),
            #[cfg(not(target_arch = "wasm32"))]
            j2534_library: String::new(),
            #[cfg(not(target_arch = "wasm32"))]
            j2534_protocol: crate::j2534::CanProtocol::default(),
            #[cfg(not(target_arch = "wasm32"))]
            j2534_ecu_address: 0x10,
            #[cfg(not(target_arch = "wasm32"))]
            j2534_drivers: crate::j2534::discover_drivers(),
            #[cfg(not(target_arch = "wasm32"))]
            elm_can_mode: crate::elm327::ElmCanMode::Auto,
            available_ports,
            selected_port: None,
            selected_baud: None,
            auto_connect: true,
            #[cfg(any(target_arch = "wasm32", debug_assertions))]
            emulator_port: 35000,
            live_data: HashMap::new(),
            live_running: false,
            supported_pids: Vec::new(),
            #[cfg(not(target_arch = "wasm32"))]
            freematics_support_reported: false,
            stored_dtcs: Vec::new(),
            pending_dtcs: Vec::new(),
            dtc_status: String::new(),
            clear_dtc_confirm: false,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_dtcs: std::array::from_fn(|_| FreematicsDtcScan::default()),
            #[cfg(not(target_arch = "wasm32"))]
            freematics_freeze_data: Vec::new(),
            #[cfg(not(target_arch = "wasm32"))]
            freematics_freeze_status: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_freeze_read_age_ms: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_freeze_trigger_dtc: None,
            freeze_data: Vec::new(),
            freeze_frame_read: false,
            vin: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_calibration_id: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_ecu_name: None,
            voltage: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_boot_id: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_capture_ms: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_capture_utc_ms: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_clock_anchor: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_last_frame_received_at: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_supply: None,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_supply_history: Vec::new(),
            #[cfg(not(target_arch = "wasm32"))]
            freematics_motion_history: Vec::new(),
            #[cfg(not(target_arch = "wasm32"))]
            freematics_acquisition_health:
                crate::freematics_usb::FreematicsAcquisitionHealth::default(),
            #[cfg(not(target_arch = "wasm32"))]
            freematics_dropped_records: 0,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_reader_drops: 0,
            #[cfg(not(target_arch = "wasm32"))]
            freematics_corrupt_records: 0,
            active_tab: Tab::Dashboard,
            log_messages: Vec::new(),
            log_auto_scroll: true,
            log_panel_open: true,
            log_panel_height: 180.0,
            log_last_count: 0,
            poll_config: PollConfig::default(),
            dark_mode: true,
            pid_defs,
            #[cfg(not(target_arch = "wasm32"))]
            log_file,
            #[cfg(not(target_arch = "wasm32"))]
            wake_lock: None,
        }
    }

    fn process_events(&mut self) {
        while let Ok(event) = self.event_rx.try_recv() {
            match event {
                ObdEvent::Connecting(msg) => {
                    if !self.connecting {
                        self.connected = false;
                        self.live_running = false;
                        self.connection_info = None;
                        self.vin = None;
                        self.voltage = None;
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            self.freematics_calibration_id = None;
                            self.freematics_ecu_name = None;
                        }
                        self.live_data.clear();
                        self.supported_pids.clear();
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            self.freematics_support_reported = false;
                        }
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            self.freematics_boot_id = None;
                            self.freematics_capture_ms = None;
                            self.freematics_capture_utc_ms = None;
                            self.freematics_clock_anchor = None;
                            self.freematics_last_frame_received_at = None;
                            self.freematics_supply = None;
                            self.freematics_supply_history.clear();
                            self.freematics_motion_history.clear();
                            self.freematics_acquisition_health =
                                crate::freematics_usb::FreematicsAcquisitionHealth::default();
                            self.freematics_dropped_records = 0;
                            self.freematics_reader_drops = 0;
                            self.freematics_corrupt_records = 0;
                            self.freematics_dtcs =
                                std::array::from_fn(|_| FreematicsDtcScan::default());
                            self.freematics_freeze_data.clear();
                            self.freematics_freeze_status = None;
                            self.freematics_freeze_read_age_ms = None;
                            self.freematics_freeze_trigger_dtc = None;
                        }
                        self.stored_dtcs.clear();
                        self.pending_dtcs.clear();
                        self.dtc_status.clear();
                        self.freeze_data.clear();
                        self.freeze_frame_read = false;
                        self.clear_dtc_confirm = false;
                        self.release_wake_lock();
                    }
                    self.connecting = true;
                    self.connection_status = msg.clone();
                    self.add_log(&format!("[CONNECT] {msg}"));
                }
                ObdEvent::Connected(info) => {
                    self.connected = true;
                    self.connecting = false;
                    self.connection_status = if info.baud == 0 {
                        format!("Connected: {} | {}", info.port, info.protocol)
                    } else {
                        format!(
                            "Connected: {} @ {} baud | {}",
                            info.port, info.baud, info.protocol
                        )
                    };
                    self.add_log(&format!(
                        "[CONNECTED] port={} baud={} protocol={} elm={}",
                        info.port, info.baud, info.protocol, info.elm_version
                    ));
                    self.connection_info = Some(info);
                }
                ObdEvent::ConnectionFailed(msg) => {
                    self.connected = false;
                    self.connecting = false;
                    self.connection_status = format!("Failed: {msg}");
                    self.add_log(&format!("[CONNECT_FAILED] {msg}"));
                }
                ObdEvent::Disconnected => {
                    self.connected = false;
                    self.connecting = false;
                    self.live_running = false;
                    self.connection_info = None;
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        self.freematics_calibration_id = None;
                        self.freematics_ecu_name = None;
                        self.freematics_boot_id = None;
                        self.freematics_capture_ms = None;
                        self.freematics_capture_utc_ms = None;
                        self.freematics_clock_anchor = None;
                        self.freematics_last_frame_received_at = None;
                        self.freematics_supply = None;
                        self.freematics_supply_history.clear();
                        self.freematics_motion_history.clear();
                        self.freematics_acquisition_health =
                            crate::freematics_usb::FreematicsAcquisitionHealth::default();
                        self.freematics_dropped_records = 0;
                        self.freematics_reader_drops = 0;
                        self.freematics_corrupt_records = 0;
                        self.freematics_dtcs =
                            std::array::from_fn(|_| FreematicsDtcScan::default());
                    }
                    self.vin = None;
                    self.voltage = None;
                    self.live_data.clear();
                    self.supported_pids.clear();
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        self.freematics_support_reported = false;
                    }
                    self.connection_status = "Disconnected".to_string();
                    self.release_wake_lock();
                    self.add_log("[DISCONNECTED]");
                }
                ObdEvent::LiveData {
                    pid_cmd,
                    name,
                    value,
                    unit,
                    raw,
                } => {
                    let received_at = Instant::now();
                    let numeric = match &value {
                        ObdValue::Numeric(v) => *v,
                        _ => 0.0,
                    };

                    // Log value changes
                    let prev = self.live_data.get(&pid_cmd).map(|s| s.numeric_value);
                    if let Some(prev_val) = prev {
                        let delta = (numeric - prev_val).abs();
                        let threshold = (prev_val.abs() * 0.01).max(0.1);
                        if delta > threshold {
                            self.add_log(&format!(
                                "[VALUE_CHANGE] pid={pid_cmd} name={name} prev={prev_val:.2} new={numeric:.2} unit={unit} raw={raw}"
                            ));
                        }
                    } else {
                        self.add_log(&format!(
                            "[VALUE_INIT] pid={pid_cmd} name={name} value={numeric:.2} unit={unit} raw={raw}"
                        ));
                    }

                    let state = self
                        .live_data
                        .entry(pid_cmd)
                        .or_insert_with(|| LivePidState {
                            name: name.clone(),
                            value: value.clone(),
                            unit: unit.clone(),
                            numeric_value: numeric,
                            history: Vec::new(),
                            raw: raw.clone(),
                            age_ms: None,
                            supported: None,
                            received_at: Instant::now(),
                            capture_ms: None,
                        });
                    state.value = value;
                    state.unit = unit;
                    state.numeric_value = numeric;
                    state.raw = raw;
                    state.history.push(HistoryPoint {
                        captured_at: received_at,
                        capture_utc_ms: None,
                        value: numeric,
                    });
                    state.age_ms = None;
                    state.supported = None;
                    state.received_at = received_at;
                    state.capture_ms = None;
                    if state.history.len() > 300 {
                        state.history.remove(0);
                    }
                }
                ObdEvent::DtcResult { stored, pending } => {
                    if stored.is_empty() && pending.is_empty() {
                        self.dtc_status = "No trouble codes found".to_string();
                        self.add_log("[DTC_SCAN] No DTCs found");
                    } else {
                        self.dtc_status =
                            format!("{} stored, {} pending", stored.len(), pending.len());
                        for dtc in &stored {
                            self.add_log(&format!("[DTC_STORED] code={}", dtc.code));
                        }
                        for dtc in &pending {
                            self.add_log(&format!("[DTC_PENDING] code={}", dtc.code));
                        }
                    }
                    self.stored_dtcs = stored;
                    self.pending_dtcs = pending;
                }
                ObdEvent::DtcDescriptionsReady { stored, pending } => {
                    self.stored_dtcs = stored;
                    self.pending_dtcs = pending;
                }
                ObdEvent::FreezeFrameData {
                    pid_cmd,
                    name,
                    value,
                    unit,
                } => {
                    self.add_log(&format!(
                        "[FREEZE_FRAME] pid={pid_cmd} name={name} value={value} unit={unit}"
                    ));
                    self.freeze_data.push((name, value, unit));
                }
                ObdEvent::Vin(vin) => {
                    self.add_log(&format!("[VIN] {vin}"));
                    self.vin = Some(vin);
                }
                ObdEvent::SupportedPids(pids) => {
                    self.add_log(&format!(
                        "[SUPPORTED_PIDS] count={} pids={:02X?}",
                        pids.len(),
                        pids
                    ));
                    self.supported_pids = pids;
                }
                ObdEvent::Voltage(v) => {
                    self.add_log(&format!("[VOLTAGE] {v}"));
                    self.voltage = Some(v);
                }
                ObdEvent::Error(msg) => {
                    self.add_log(&format!("[ERROR] {msg}"));
                }
                ObdEvent::LogMessage(msg) => {
                    self.add_log(&msg);
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let frames: Vec<_> = self.freematics_rx.try_iter().collect();
            for frame in frames {
                self.apply_freematics_frame(frame);
            }
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn apply_freematics_frame(&mut self, frame: crate::freematics_usb::FreematicsFrame) {
        let received_at = frame.reader_received_at;
        if self.freematics_boot_id == Some(frame.boot_id)
            && self.freematics_capture_ms.is_some_and(|previous| {
                let elapsed = frame.capture_ms.wrapping_sub(previous);
                elapsed == 0 || elapsed >= (1 << 31)
            })
        {
            return;
        }
        if let Some(previous) = self.freematics_boot_id
            && previous != frame.boot_id
        {
            self.add_log(&format!(
                "[FREEMATICS_RESTART] boot_id={:016X} previous={previous:016X}",
                frame.boot_id
            ));
            self.live_data.clear();
            self.freematics_capture_utc_ms = None;
            self.freematics_clock_anchor = None;
            self.freematics_supply = None;
            self.freematics_supply_history.clear();
            self.freematics_motion_history.clear();
            self.freematics_acquisition_health =
                crate::freematics_usb::FreematicsAcquisitionHealth::default();
            // The device's USB-drop counter is scoped to a firmware boot.
            // Compare its new value against a fresh baseline below.
            self.freematics_dropped_records = 0;
            self.freematics_dtcs = std::array::from_fn(|_| FreematicsDtcScan::default());
        }
        let anchor = *self
            .freematics_clock_anchor
            .get_or_insert(FreematicsClockAnchor {
                capture_ms: frame.capture_ms,
                received_at,
            });
        let frame_capture_at =
            freematics_sample_instant(anchor, frame.capture_ms).unwrap_or(received_at);
        self.freematics_last_frame_received_at = Some(received_at);
        let previous_device_drops = self.freematics_dropped_records;
        let previous_reader_drops = self.freematics_reader_drops;
        let previous_corrupt_records = self.freematics_corrupt_records;
        self.freematics_boot_id = Some(frame.boot_id);
        self.freematics_capture_ms = Some(frame.capture_ms);
        self.freematics_capture_utc_ms = frame.capture_utc_ms;
        self.freematics_dropped_records = frame.dropped_records;
        self.freematics_reader_drops = frame.reader_drops;
        self.freematics_acquisition_health = frame.acquisition_health();
        let next_freeze_status = frame.freeze_frame_status();
        let next_freeze_trigger = frame.freeze_frame_trigger_dtc();
        let new_freeze_capture = next_freeze_status == Some(3)
            && (self.freematics_freeze_status != Some(3)
                || self.freematics_freeze_trigger_dtc != next_freeze_trigger);
        self.freematics_freeze_status = next_freeze_status;
        self.freematics_freeze_read_age_ms = frame.freeze_frame_read_age_ms();
        self.freematics_freeze_trigger_dtc = next_freeze_trigger;
        let freeze_data = frame.freeze_frame_measurements();
        if matches!(self.freematics_freeze_status, Some(0 | 2)) || new_freeze_capture {
            self.freematics_freeze_data.clear();
        }
        if !freeze_data.is_empty() {
            self.freematics_freeze_data = freeze_data;
        }
        self.freematics_support_reported = frame.supported_pids.is_some();
        self.supported_pids = frame
            .supported_pids
            .as_ref()
            .map(|pids| pids.iter().copied().collect())
            .unwrap_or_default();

        if let Some(vin) = frame.vin.as_ref() {
            self.vin = Some(vin.clone());
        }
        self.freematics_calibration_id = frame.calibration_id.clone();
        self.freematics_ecu_name = frame.ecu_name.clone();
        if let Some((volts, age_ms)) = frame.model_b_supply_voltage() {
            self.freematics_supply = Some((volts, age_ms, received_at));
            let voltage_waveform = frame.voltage_waveform();
            if voltage_waveform.is_empty() {
                if let Some(captured_at) = age_ms
                    .and_then(|age| frame_capture_at.checked_sub(Duration::from_millis(age as u64)))
                {
                    push_waveform_point(
                        &mut self.freematics_supply_history,
                        HistoryPoint {
                            captured_at,
                            capture_utc_ms: age_ms.and_then(|age| {
                                freematics_sample_utc_ms(
                                    frame.capture_ms,
                                    frame.capture_utc_ms,
                                    frame.capture_ms.wrapping_sub(age),
                                )
                            }),
                            value: volts,
                        },
                    );
                }
            } else {
                for (capture_ms, value) in voltage_waveform {
                    if let Some(captured_at) = freematics_sample_instant(anchor, capture_ms) {
                        push_waveform_point(
                            &mut self.freematics_supply_history,
                            HistoryPoint {
                                captured_at,
                                capture_utc_ms: freematics_sample_utc_ms(
                                    frame.capture_ms,
                                    frame.capture_utc_ms,
                                    capture_ms,
                                ),
                                value,
                            },
                        );
                    }
                }
            }
        }
        for (capture_ms, value) in frame.acceleration_waveform() {
            if let Some(captured_at) = freematics_sample_instant(anchor, capture_ms) {
                push_waveform_point(
                    &mut self.freematics_motion_history,
                    HistoryPoint {
                        captured_at,
                        capture_utc_ms: freematics_sample_utc_ms(
                            frame.capture_ms,
                            frame.capture_utc_ms,
                            capture_ms,
                        ),
                        value,
                    },
                );
            }
        }
        for (index, scan) in frame.dtc_scans().into_iter().enumerate() {
            let status = match scan.status {
                Some(crate::freematics_usb::FreematicsDtcStatus::NoResponse) => Some(0),
                Some(crate::freematics_usb::FreematicsDtcStatus::Response) => Some(1),
                Some(crate::freematics_usb::FreematicsDtcStatus::Codes) => Some(2),
                None => None,
            };
            let codes = if scan.status == Some(crate::freematics_usb::FreematicsDtcStatus::Codes) {
                scan.code_slots
                    .iter()
                    .take(scan.count.unwrap_or(0) as usize)
                    .flatten()
                    .copied()
                    .filter(|raw| *raw != 0)
                    .map(|raw| Dtc {
                        code: obd::decode_dtc_bytes((raw >> 8) as u8, raw as u8),
                        description: String::new(),
                        desc_source: DescSource::NotFound,
                    })
                    .collect()
            } else {
                Vec::new()
            };
            let incoming = FreematicsDtcScan {
                availability: scan.availability,
                status,
                count: scan.count,
                age_ms: scan.age_ms,
                received_at: scan.age_ms.map(|_| received_at),
                codes,
            };
            // Some firmware frames omit a scan mode until it has produced a
            // result. Keep the last reported result (and its original age
            // anchor) instead of replacing it with an unreported placeholder.
            if incoming.availability
                != crate::freematics_usb::FreematicsDtcAvailability::Unsupported
                || self.freematics_dtcs[index].availability
                    == crate::freematics_usb::FreematicsDtcAvailability::Unsupported
            {
                self.freematics_dtcs[index] = incoming;
            }
        }

        for measurement in frame.measurements() {
            let key = measurement.cmd;
            let numeric = measurement.value;
            let age_ms = measurement.age_ms;
            let display_value = measurement
                .display_value
                .clone()
                .unwrap_or(ObdValue::Numeric(numeric));
            let raw_value = measurement.raw_bytes.as_ref().map(|bytes| {
                bytes
                    .iter()
                    .map(|byte| format!("{byte:02X}"))
                    .collect::<String>()
            });
            let raw_text = raw_value.map_or_else(
                || format!("capture={} age_ms={age_ms:?}", frame.capture_ms),
                |hex| format!("capture={} age_ms={age_ms:?} raw={hex}", frame.capture_ms),
            );
            let age_limit = if key == "010C" || key == "010D" {
                250
            } else {
                1000
            };
            let state = self
                .live_data
                .entry(key.clone())
                .or_insert_with(|| LivePidState {
                    name: measurement.name.clone(),
                    value: ObdValue::Numeric(numeric),
                    unit: measurement.unit.clone(),
                    numeric_value: numeric,
                    history: Vec::new(),
                    raw: raw_text.clone(),
                    age_ms,
                    supported: measurement.supported,
                    received_at,
                    capture_ms: None,
                });
            state.name = measurement.name;
            state.value = display_value;
            state.unit = measurement.unit;
            state.numeric_value = numeric;
            state.raw = raw_text;
            state.age_ms = age_ms;
            state.supported = measurement.supported;
            state.received_at = received_at;

            // Only add a point when the source says the ECU measurement is
            // within its freshness budget; repeated cached values remain
            // visible with their real age but do not look like new samples.
            let new_capture = state.capture_ms != Some(frame.capture_ms);
            state.capture_ms = Some(frame.capture_ms);
            if new_capture && age_ms.is_some_and(|age| age <= age_limit) {
                if let Some(captured_at) = age_ms
                    .and_then(|age| received_at.checked_sub(Duration::from_millis(age as u64)))
                {
                    state.history.push(HistoryPoint {
                        captured_at,
                        capture_utc_ms: age_ms.and_then(|age| {
                            freematics_sample_utc_ms(
                                frame.capture_ms,
                                frame.capture_utc_ms,
                                frame.capture_ms.wrapping_sub(age),
                            )
                        }),
                        value: numeric,
                    });
                }
                if state.history.len() > 300 {
                    state.history.remove(0);
                }
            }
        }
        if frame.dropped_records > previous_device_drops {
            self.add_log(&format!(
                "[FREEMATICS_USB_DROPS] cumulative={}",
                frame.dropped_records
            ));
        }
        if frame.corrupt_records > previous_corrupt_records {
            self.freematics_corrupt_records = frame.corrupt_records;
            self.add_log(&format!(
                "[FREEMATICS_USB_CORRUPT] cumulative={} sample={}",
                frame.corrupt_records,
                frame.corrupt_sample_hex.as_deref().unwrap_or("unavailable")
            ));
        }
        if frame.reader_drops > previous_reader_drops {
            self.add_log(&format!(
                "[FREEMATICS_DASHBOARD_DROPS] cumulative={}",
                frame.reader_drops
            ));
        }
    }

    fn add_log(&mut self, msg: &str) {
        let timestamp = chrono::Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
        let line = format!("{timestamp} {msg}");

        // Write to stdout
        println!("{line}");

        // Write to log file (desktop only)
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(log_file) = &self.log_file {
            if let Ok(mut f) = log_file.lock() {
                let _ = writeln!(f, "{line}");
            }
        }

        self.log_messages.push(line);
        // Keep in-memory log bounded
        if self.log_messages.len() > 10000 {
            self.log_messages.drain(..5000);
        }
    }

    fn send_cmd(&self, cmd: OdbCmd) {
        let _ = self.cmd_tx.send(cmd);
    }

    fn is_freematics_usb(&self) -> bool {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.connection_info.as_ref().is_some_and(|info| {
                matches!(
                    info.protocol.as_str(),
                    "Freematics Telemetry v1" | "Freematics Telemetry v2"
                )
            }) || self.connection_kind == ConnectionKind::FreematicsUsb
        }
        #[cfg(target_arch = "wasm32")]
        {
            false
        }
    }

    fn vehicle_make(&self) -> Option<String> {
        let make = crate::vin_decoder::decode(self.vin.as_deref()?).make;
        if make == "Unknown" { None } else { Some(make) }
    }

    /// Check if engine appears to be running based on RPM > 0
    fn engine_running(&self) -> bool {
        self.live_data
            .get("010C")
            .is_some_and(|s| s.numeric_value > 0.0)
    }

    fn displayed_age_ms(state: &LivePidState) -> Option<u128> {
        if state.capture_ms.is_some() {
            state
                .age_ms
                .map(|age| age as u128 + state.received_at.elapsed().as_millis())
        } else {
            Some(state.received_at.elapsed().as_millis())
        }
    }

    fn pid_is_stale(&self, pid: &str, state: &LivePidState) -> bool {
        if !self.is_freematics_usb() {
            return false;
        }
        let limit = if pid == "010C" || pid == "010D" {
            250
        } else {
            1000
        };
        Self::displayed_age_ms(state).is_none_or(|age| age > limit)
    }

    fn show_pid_age(&self, ui: &mut egui::Ui, pid: &str, state: &LivePidState) {
        if self.is_freematics_usb() {
            let age = Self::displayed_age_ms(state);
            let text = match age {
                Some(age) if !self.pid_is_stale(pid, state) => format!("Freshness age: {age} ms"),
                Some(age) => format!("Out-of-date reading · acquisition age {age} ms"),
                None => "Out-of-date reading · acquisition age unavailable".to_string(),
            };
            ui.colored_label(
                if self.pid_is_stale(pid, state) {
                    Color32::from_rgb(220, 170, 80)
                } else {
                    Color32::from_gray(130)
                },
                RichText::new(text).small(),
            );
        }
    }

    fn show_engine_warning(&self, ui: &mut egui::Ui) {
        if self.live_running && !self.engine_running() && !self.live_data.is_empty() {
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("Engine not running - RPM is 0. Sensor data may be unavailable or inaccurate.")
                        .color(Color32::from_rgb(220, 180, 50)),
                );
            });
            ui.add_space(2.0);
        }
    }

    // ── UI Sections ─────────────────────────────────────────────────────────

    fn connect_selected(&self) {
        #[cfg(not(target_arch = "wasm32"))]
        match self.connection_kind {
            ConnectionKind::Tcp => {
                self.send_cmd(OdbCmd::ConnectAdapter(NativeConnection::Tcp(
                    self.tcp_address.trim().into(),
                )));
                return;
            }
            ConnectionKind::J2534 => {
                self.send_cmd(OdbCmd::ConnectAdapter(NativeConnection::J2534 {
                    library: self.j2534_library.trim().into(),
                    protocol: self.j2534_protocol,
                    ecu_address: self.j2534_ecu_address,
                }));
                return;
            }
            ConnectionKind::FreematicsUsb => {
                self.send_cmd(OdbCmd::ConnectFreematicsUsb(self.selected_port.clone()));
                return;
            }
            ConnectionKind::Serial => {}
        }
        self.send_cmd(OdbCmd::Connect {
            port: self.selected_port.clone(),
            baud: self.selected_baud,
            #[cfg(not(target_arch = "wasm32"))]
            elm_can_mode: self.elm_can_mode,
        });
    }

    fn show_adapter_selector(&mut self, ui: &mut egui::Ui) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            egui::ComboBox::from_id_salt(ui.id().with("adapter_kind"))
                .selected_text(match self.connection_kind {
                    ConnectionKind::Serial => "USB / serial",
                    ConnectionKind::FreematicsUsb => "Freematics USB",
                    ConnectionKind::Tcp => "Wi-Fi / TCP",
                    ConnectionKind::J2534 => "J2534 pass-through",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.connection_kind,
                        ConnectionKind::Serial,
                        "USB / serial (ELM-compatible)",
                    );
                    ui.selectable_value(
                        &mut self.connection_kind,
                        ConnectionKind::FreematicsUsb,
                        "Freematics USB (passive telemetry)",
                    );
                    ui.selectable_value(
                        &mut self.connection_kind,
                        ConnectionKind::Tcp,
                        "Wi-Fi / TCP (ELM-compatible)",
                    );
                    ui.selectable_value(
                        &mut self.connection_kind,
                        ConnectionKind::J2534,
                        "J2534 pass-through",
                    );
                });
            if self.connection_kind == ConnectionKind::Serial {
                egui::ComboBox::from_id_salt(ui.id().with("can_mode"))
                    .selected_text(match self.elm_can_mode {
                        crate::elm327::ElmCanMode::Auto => "Automatic OBD / HS-CAN",
                        crate::elm327::ElmCanMode::CorsaDMediumSpeed => {
                            "Corsa D MS-CAN · experimental"
                        }
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.elm_can_mode,
                            crate::elm327::ElmCanMode::Auto,
                            "Automatic OBD / HS-CAN (default)",
                        );
                        ui.selectable_value(
                            &mut self.elm_can_mode,
                            crate::elm327::ElmCanMode::CorsaDMediumSpeed,
                            "Corsa D MS-CAN · 95.2 kbit/s",
                        );
                    });
            }
            match self.connection_kind {
                ConnectionKind::Tcp => {
                    ui.add(
                        egui::TextEdit::singleline(&mut self.tcp_address)
                            .hint_text("Adapter host:port")
                            .desired_width(190.0),
                    );
                    return;
                }
                ConnectionKind::J2534 => {
                    egui::ComboBox::from_id_salt(ui.id().with("j2534_driver"))
                        .selected_text("Installed drivers")
                        .show_ui(ui, |ui| {
                            for driver in &self.j2534_drivers {
                                if ui
                                    .selectable_label(
                                        self.j2534_library == driver.path.to_string_lossy(),
                                        &driver.name,
                                    )
                                    .clicked()
                                {
                                    self.j2534_library = driver.path.to_string_lossy().into_owned();
                                }
                            }
                            if self.j2534_drivers.is_empty() {
                                ui.label("No matching drivers found");
                            }
                        });
                    if ui.button("Refresh drivers").clicked() {
                        self.j2534_drivers = crate::j2534::discover_drivers();
                    }
                    ui.add(
                        egui::TextEdit::singleline(&mut self.j2534_library)
                            .hint_text("Absolute driver library path")
                            .desired_width(240.0),
                    );
                    egui::ComboBox::from_id_salt(ui.id().with("j2534_protocol"))
                        .selected_text(self.j2534_protocol.label())
                        .show_ui(ui, |ui| {
                            for protocol in crate::j2534::CanProtocol::ALL {
                                ui.selectable_value(
                                    &mut self.j2534_protocol,
                                    protocol,
                                    protocol.label(),
                                );
                            }
                        });
                    if self.j2534_protocol.extended() {
                        ui.label("ECU address:");
                        ui.add(egui::DragValue::new(&mut self.j2534_ecu_address).hexadecimal(2, false, true))
                            .on_hover_text("29-bit response source address in hex. Default 10; set from vehicle documentation. This connection queries one ECU.");
                    }
                    ui.label("04.04 driver; standard CAN diagnostics only")
                        .on_hover_text("Driver must match the app's 32/64-bit architecture. Legacy protocols, manufacturer modules and PSA-specific drivers are not yet supported by this backend.");
                    return;
                }
                ConnectionKind::Serial => {}
                ConnectionKind::FreematicsUsb => {
                    ui.label("Passive telemetry; no diagnostic commands");
                }
            }
        }
        egui::ComboBox::from_id_salt(ui.id().with("serial_port"))
            .selected_text(self.selected_port.as_deref().unwrap_or("Auto-detect"))
            .show_ui(ui, |ui| {
                let detecting_freematics = self.is_freematics_usb();
                ui.selectable_value(
                    &mut self.selected_port,
                    None,
                    if detecting_freematics {
                        "Auto-detect Freematics USB"
                    } else {
                        "Auto-detect ELM adapter"
                    },
                );
                for port in &self.available_ports {
                    if detecting_freematics && !is_usb_serial_port_name(port) {
                        continue;
                    }
                    ui.selectable_value(&mut self.selected_port, Some(port.clone()), port);
                }
            });
        #[cfg(not(target_arch = "wasm32"))]
        if ui.button("Refresh ports").clicked() {
            self.available_ports = crate::elm327::scan_ports();
        }
    }

    fn show_connection_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            // Status indicator
            let (status_color, status_text) = if self.connected {
                (Color32::from_rgb(50, 200, 80), "Connected")
            } else if self.connecting {
                (Color32::from_rgb(220, 180, 50), "Connecting...")
            } else {
                (Color32::from_rgb(180, 50, 50), "Disconnected")
            };
            ui.colored_label(status_color, "●");
            ui.label(
                RichText::new(status_text)
                    .strong()
                    .color(Color32::from_gray(190)),
            );
            ui.separator();

            if self.connected {
                // Vehicle info from VIN
                if let Some(vin) = &self.vin {
                    let summary = crate::vin_decoder::summary(vin);
                    ui.label(RichText::new(summary).strong());
                    ui.separator();
                }

                if let Some(info) = &self.connection_info {
                    ui.label(
                        RichText::new(format!("{} | {}", info.port, info.protocol))
                            .color(Color32::from_gray(140))
                            .small(),
                    );
                }
                if let Some(v) = &self.voltage {
                    ui.separator();
                    ui.label(RichText::new(v.to_string()).color(Color32::from_rgb(80, 160, 220)));
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Disconnect").clicked() {
                        self.send_cmd(OdbCmd::Disconnect);
                    }
                });
            } else if !self.connecting {
                self.show_adapter_selector(ui);

                if ui.button(RichText::new("Connect").strong()).clicked() {
                    self.connect_selected();
                }

                #[cfg(any(target_arch = "wasm32", debug_assertions))]
                {
                    ui.separator();
                    ui.add(
                        egui::DragValue::new(&mut self.emulator_port)
                            .range(1024..=65535)
                            .prefix("localhost:"),
                    );
                    if ui
                        .button(RichText::new("Connect to emulator").strong())
                        .on_hover_text("Connect to a local obd-emulator instance via WebSocket")
                        .clicked()
                    {
                        self.send_cmd(OdbCmd::ConnectLocal {
                            ws_port: self.emulator_port,
                        });
                    }
                }
            }
        });

        if !self.connection_status.is_empty() {
            ui.label(
                RichText::new(&self.connection_status)
                    .color(Color32::from_gray(120))
                    .small(),
            );
        }
    }

    fn show_tab_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let tabs = [
                (Tab::Dashboard, "Dashboard"),
                (Tab::Sensors, "Sensors"),
                (Tab::DtcCodes, "DTCs"),
                (Tab::FreezeFrame, "Freeze Frame"),
                (Tab::VehicleInfo, "Vehicle Info"),
            ];
            for (tab, label) in &tabs {
                let selected = self.active_tab == *tab;
                let text = if selected {
                    RichText::new(*label)
                        .strong()
                        .color(Color32::from_rgb(80, 160, 220))
                } else {
                    RichText::new(*label).color(Color32::from_gray(160))
                };
                if ui.selectable_label(selected, text).clicked() {
                    self.active_tab = *tab;
                }
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Theme toggle
                let theme_label = if self.dark_mode {
                    RichText::new("Light").color(Color32::from_gray(160))
                } else {
                    RichText::new("Dark").color(Color32::from_gray(160))
                };
                if ui.selectable_label(false, theme_label).clicked() {
                    self.dark_mode = !self.dark_mode;
                }

                ui.separator();

                // Log toggle
                let log_label = if self.log_panel_open {
                    RichText::new("Log ▼").color(Color32::from_rgb(80, 160, 220))
                } else {
                    RichText::new("Log ▲").color(Color32::from_gray(160))
                };
                if ui
                    .selectable_label(self.log_panel_open, log_label)
                    .clicked()
                {
                    self.log_panel_open = !self.log_panel_open;
                }
            });
        });
    }

    fn start_polling(&mut self) {
        self.live_data.clear();
        self.send_cmd(OdbCmd::SetPollConfig(self.poll_config.clone()));
        self.send_cmd(OdbCmd::StartLiveData);
        self.live_running = true;
        self.acquire_wake_lock();
    }

    fn stop_polling(&mut self) {
        self.send_cmd(OdbCmd::StopLiveData);
        self.live_running = false;
        self.release_wake_lock();
    }

    fn acquire_wake_lock(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            if self.wake_lock.is_some() {
                return;
            }

            #[cfg(target_os = "windows")]
            {
                // ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED
                const ES_CONTINUOUS: u32 = 0x80000000;
                const ES_SYSTEM_REQUIRED: u32 = 0x00000001;
                const ES_DISPLAY_REQUIRED: u32 = 0x00000002;
                #[link(name = "kernel32")]
                unsafe extern "system" {
                    fn SetThreadExecutionState(flags: u32) -> u32;
                }
                unsafe {
                    SetThreadExecutionState(
                        ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED,
                    );
                }
                // Use a dummy child sentinel so release_wake_lock knows to clear it.
                // On Windows we don't have a child process, so we use a no-op `cmd /c exit`.
                if let Ok(child) = std::process::Command::new("cmd")
                    .args(["/c", "exit"])
                    .stdin(std::process::Stdio::null())
                    .stdout(std::process::Stdio::null())
                    .stderr(std::process::Stdio::null())
                    .spawn()
                {
                    self.wake_lock = Some(child);
                }
                self.add_log("[WAKE_LOCK] Screen sleep inhibited");
                return;
            }

            #[cfg(target_os = "macos")]
            {
                // caffeinate -i: prevent idle sleep; lives until killed
                match std::process::Command::new("caffeinate")
                    .arg("-i")
                    .stdin(std::process::Stdio::null())
                    .stdout(std::process::Stdio::null())
                    .stderr(std::process::Stdio::null())
                    .spawn()
                {
                    Ok(child) => {
                        self.wake_lock = Some(child);
                        self.add_log("[WAKE_LOCK] Screen sleep inhibited");
                        return;
                    }
                    Err(e) => {
                        self.add_log(&format!("[WAKE_LOCK] caffeinate unavailable: {e}"));
                        return;
                    }
                }
            }

            #[cfg(target_os = "linux")]
            {
                // systemd-inhibit keeps the inhibit as long as the child process lives.
                match std::process::Command::new("systemd-inhibit")
                    .args([
                        "--what=idle",
                        "--who=OBD Dashboard",
                        "--why=Live OBD polling active",
                        "--mode=block",
                        "sleep",
                        "infinity",
                    ])
                    .stdin(std::process::Stdio::null())
                    .stdout(std::process::Stdio::null())
                    .stderr(std::process::Stdio::null())
                    .spawn()
                {
                    Ok(child) => {
                        self.wake_lock = Some(child);
                        self.add_log("[WAKE_LOCK] Screen sleep inhibited");
                    }
                    Err(e) => {
                        self.add_log(&format!("[WAKE_LOCK] systemd-inhibit unavailable: {e}"));
                    }
                }
            }
        } // end #[cfg(not(target_arch = "wasm32"))]
    }

    fn release_wake_lock(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            #[cfg(target_os = "windows")]
            {
                if self.wake_lock.is_some() {
                    const ES_CONTINUOUS: u32 = 0x80000000;
                    #[link(name = "kernel32")]
                    unsafe extern "system" {
                        fn SetThreadExecutionState(flags: u32) -> u32;
                    }
                    unsafe {
                        SetThreadExecutionState(ES_CONTINUOUS);
                    }
                }
            }

            if let Some(mut child) = self.wake_lock.take() {
                let _ = child.kill();
                let _ = child.wait();
                self.add_log("[WAKE_LOCK] Screen sleep re-enabled");
            }
        } // end #[cfg(not(target_arch = "wasm32"))]
    }

    fn show_dashboard(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.vertical_centered(|ui| {
                ui.add_space(ui.available_height() * 0.25);

                // Port selector
                ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                    ui.horizontal_wrapped(|ui| {
                        self.show_adapter_selector(ui);
                    });
                });

                ui.add_space(12.0);

                let button = egui::Button::new(
                    RichText::new("Connect")
                        .size(32.0)
                        .strong()
                        .color(Color32::WHITE),
                )
                .min_size(egui::vec2(280.0, 80.0))
                .fill(Color32::from_rgb(40, 120, 200))
                .corner_radius(12.0);

                if ui.add_enabled(!self.connecting, button).clicked() {
                    self.connect_selected();
                }

                ui.add_space(12.0);

                if self.connecting {
                    ui.spinner();
                    ui.label(
                        RichText::new(&self.connection_status)
                            .color(Color32::from_rgb(220, 180, 50)),
                    );
                    #[cfg(not(target_arch = "wasm32"))]
                    if self.connection_kind == ConnectionKind::FreematicsUsb
                        && ui.button("Cancel listening").clicked()
                    {
                        self.send_cmd(OdbCmd::Disconnect);
                    }
                } else {
                    ui.label(
                        RichText::new("Select an adapter connection and connect")
                            .color(Color32::from_gray(100)),
                    );
                }
            });
            return;
        }

        let freematics_usb = {
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.is_freematics_usb()
            }
            #[cfg(target_arch = "wasm32")]
            {
                false
            }
        };

        // Show big start button only for command/response adapters.
        if !freematics_usb && !self.live_running {
            ui.vertical_centered(|ui| {
                ui.add_space(ui.available_height() * 0.25);

                let button = egui::Button::new(
                    RichText::new("Start Polling")
                        .size(32.0)
                        .strong()
                        .color(Color32::WHITE),
                )
                .min_size(egui::vec2(280.0, 80.0))
                .fill(Color32::from_rgb(40, 120, 200))
                .corner_radius(12.0);

                if ui.add(button).clicked() {
                    self.start_polling();
                }

                ui.add_space(16.0);

                ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Mode:").color(Color32::from_gray(140)));
                        let mode = &mut self.poll_config.mode;
                        if ui
                            .selectable_label(*mode == PollMode::Minimal, "Minimal")
                            .clicked()
                        {
                            *mode = PollMode::Minimal;
                        }
                        if ui
                            .selectable_label(*mode == PollMode::Fast, "Fast")
                            .clicked()
                        {
                            *mode = PollMode::Fast;
                        }
                        if ui
                            .selectable_label(*mode == PollMode::Full, "Full")
                            .clicked()
                        {
                            *mode = PollMode::Full;
                        }
                    });

                    ui.add_space(8.0);
                    ui.label(
                        RichText::new("Select polling mode and press Start")
                            .color(Color32::from_gray(100)),
                    );
                });
            });
            return;
        }

        if !freematics_usb {
            // Controls bar when running
            ui.horizontal(|ui| {
                if ui
                    .button(RichText::new("Stop").color(Color32::from_rgb(220, 50, 50)))
                    .clicked()
                {
                    self.stop_polling();
                }

                ui.separator();

                ui.label(RichText::new("Poll:").color(Color32::from_gray(140)));
                let mut changed = false;
                let mode = &mut self.poll_config.mode;
                if ui
                    .selectable_label(*mode == PollMode::Minimal, "Minimal")
                    .clicked()
                {
                    *mode = PollMode::Minimal;
                    changed = true;
                }
                if ui
                    .selectable_label(*mode == PollMode::Fast, "Fast")
                    .clicked()
                {
                    *mode = PollMode::Fast;
                    changed = true;
                }
                if ui
                    .selectable_label(*mode == PollMode::Full, "Full")
                    .clicked()
                {
                    *mode = PollMode::Full;
                    changed = true;
                }

                ui.separator();

                ui.label(RichText::new("Delay:").color(Color32::from_gray(140)));
                let mut cycle_ms = self.poll_config.cycle_delay_ms as u32;
                let slider = egui::Slider::new(&mut cycle_ms, 0..=1000).suffix("ms");
                if ui.add(slider).changed() {
                    self.poll_config.cycle_delay_ms = cycle_ms as u64;
                    changed = true;
                }

                if changed {
                    self.live_data.clear();
                    self.send_cmd(OdbCmd::SetPollConfig(self.poll_config.clone()));
                }

                ui.separator();
                ui.label(
                    RichText::new(format!("{} sensors", self.live_data.len()))
                        .color(Color32::from_gray(140)),
                );
            });
        }

        self.show_engine_warning(ui);
        ui.add_space(4.0);

        let avail = ui.available_size();

        egui::ScrollArea::vertical().show(ui, |ui| {
            // ── Top row: primary gauges (RPM + Speed large, 4 smaller) ──
            let gauge_size = ((avail.x - 40.0) / 4.0).clamp(120.0, 200.0);
            let small_gauge = (gauge_size * 0.78).clamp(100.0, 150.0);

            ui.columns(2, |cols| {
                // Left column: RPM
                cols[0].vertical_centered(|ui| {
                    if let Some(s) = self.live_data.get("010C") {
                        RadialGauge::new("RPM", s.numeric_value, 0.0, 8000.0, "RPM")
                            .size(gauge_size)
                            .warning(5500.0)
                            .danger(7000.0)
                            .show(ui);
                        self.show_pid_age(ui, "010C", s);
                    }
                });
                // Right column: Speed
                cols[1].vertical_centered(|ui| {
                    if let Some(s) = self.live_data.get("010D") {
                        RadialGauge::new("Speed", s.numeric_value, 0.0, 260.0, "km/h")
                            .size(gauge_size)
                            .warning(130.0)
                            .danger(180.0)
                            .show(ui);
                        self.show_pid_age(ui, "010D", s);
                    }
                });
            });

            ui.add_space(4.0);

            // ── Second row: 4 smaller gauges ────────────────────────────
            ui.columns(4, |cols| {
                let gauges = [
                    GaugeSpec {
                        column: 0,
                        pid: "0105",
                        label: "Coolant",
                        min: -40.0,
                        max: 215.0,
                        unit: "\u{00B0}C",
                        warning: Some(100.0),
                        danger: Some(115.0),
                        decimals: 0,
                    },
                    GaugeSpec {
                        column: 1,
                        pid: "015C",
                        label: "Oil Temp",
                        min: -40.0,
                        max: 215.0,
                        unit: "\u{00B0}C",
                        warning: Some(120.0),
                        danger: Some(140.0),
                        decimals: 0,
                    },
                    GaugeSpec {
                        column: 2,
                        pid: "0111",
                        label: "Throttle",
                        min: 0.0,
                        max: 100.0,
                        unit: "%",
                        warning: None,
                        danger: None,
                        decimals: 1,
                    },
                    GaugeSpec {
                        column: 3,
                        pid: "0104",
                        label: "Load",
                        min: 0.0,
                        max: 100.0,
                        unit: "%",
                        warning: Some(80.0),
                        danger: Some(95.0),
                        decimals: 1,
                    },
                ];
                for gauge in gauges {
                    cols[gauge.column].vertical_centered(|ui| {
                        if let Some(s) = self.live_data.get(gauge.pid) {
                            let mut g = RadialGauge::new(
                                gauge.label,
                                s.numeric_value,
                                gauge.min,
                                gauge.max,
                                gauge.unit,
                            )
                            .size(small_gauge)
                            .decimals(gauge.decimals);
                            if let Some(w) = gauge.warning {
                                g = g.warning(w);
                            }
                            if let Some(d) = gauge.danger {
                                g = g.danger(d);
                            }
                            g.show(ui);
                            self.show_pid_age(ui, gauge.pid, s);
                        }
                    });
                }
            });

            ui.add_space(6.0);
            ui.separator();
            ui.add_space(4.0);

            ui.separator();
            ui.label(RichText::new("Trends · last 60 seconds").strong());
            if let Some(state) = self.live_data.get("010C") {
                show_time_series(
                    ui,
                    state,
                    TimeSeriesConfig {
                        label: "Engine RPM",
                        color: Color32::from_rgb(220, 115, 95),
                        unit: "rpm",
                        minimum_range: 180.0,
                        decimals: 0,
                        freshness: Duration::from_millis(250),
                        max_gap: Duration::from_secs(1),
                    },
                );
            }
            ui.columns(2, |cols| {
                let charts = [
                    (
                        "010D",
                        TimeSeriesConfig {
                            label: "Road speed",
                            color: Color32::from_rgb(85, 165, 225),
                            unit: "km/h",
                            minimum_range: 8.0,
                            decimals: 0,
                            freshness: Duration::from_secs(1),
                            max_gap: Duration::from_secs(3),
                        },
                    ),
                    (
                        "0105",
                        TimeSeriesConfig {
                            label: "Coolant temperature",
                            color: Color32::from_rgb(225, 180, 65),
                            unit: "°C",
                            minimum_range: 15.0,
                            decimals: 0,
                            freshness: Duration::from_secs(1),
                            max_gap: Duration::from_secs(4),
                        },
                    ),
                    (
                        "0111",
                        TimeSeriesConfig {
                            label: "Throttle position",
                            color: Color32::from_rgb(75, 190, 125),
                            unit: "%",
                            minimum_range: 10.0,
                            decimals: 0,
                            freshness: Duration::from_secs(1),
                            max_gap: Duration::from_secs(4),
                        },
                    ),
                    (
                        "0104",
                        TimeSeriesConfig {
                            label: "Calculated engine load",
                            color: Color32::from_rgb(175, 125, 220),
                            unit: "%",
                            minimum_range: 10.0,
                            decimals: 0,
                            freshness: Duration::from_secs(1),
                            max_gap: Duration::from_secs(4),
                        },
                    ),
                    (
                        "0142",
                        TimeSeriesConfig {
                            label: "ECU control-module voltage",
                            color: Color32::from_rgb(80, 195, 195),
                            unit: "V",
                            minimum_range: 1.0,
                            decimals: 2,
                            freshness: Duration::from_secs(1),
                            max_gap: Duration::from_secs(4),
                        },
                    ),
                    (
                        "012F",
                        TimeSeriesConfig {
                            label: "Fuel level",
                            color: Color32::from_rgb(205, 135, 185),
                            unit: "%",
                            minimum_range: 10.0,
                            decimals: 0,
                            freshness: Duration::from_secs(1),
                            max_gap: Duration::from_secs(4),
                        },
                    ),
                ];
                for (index, (pid, config)) in charts.into_iter().enumerate() {
                    if let Some(state) = self.live_data.get(pid) {
                        cols[index % 2].vertical(|ui| {
                            show_time_series(ui, state, config);
                            ui.add_space(8.0);
                        });
                    }
                }
            });

            #[cfg(not(target_arch = "wasm32"))]
            if freematics_usb {
                ui.columns(2, |cols| {
                    if let Some((value, age_ms, received_at)) = self.freematics_supply {
                        let age = age_ms.map(|age| {
                            Duration::from_millis(age as u64)
                                .saturating_add(received_at.elapsed())
                        });
                        show_time_series_values(
                            &mut cols[0],
                            &self.freematics_supply_history,
                            value,
                            age,
                            TimeSeriesConfig {
                                label: "Vehicle battery voltage",
                                color: Color32::from_rgb(235, 180, 70),
                                unit: "V",
                                minimum_range: 0.6,
                                decimals: 2,
                                freshness: Duration::from_secs(1),
                                max_gap: Duration::from_millis(500),
                            },
                        );
                    }
                    if let Some(latest) = self.freematics_motion_history.last() {
                        show_time_series_values(
                            &mut cols[1],
                            &self.freematics_motion_history,
                            latest.value,
                            Some(Instant::now().saturating_duration_since(latest.captured_at)),
                            TimeSeriesConfig {
                                label: "Acceleration magnitude",
                                color: Color32::from_rgb(105, 175, 215),
                                unit: "g",
                                minimum_range: 0.15,
                                decimals: 3,
                                freshness: Duration::from_millis(500),
                                max_gap: Duration::from_millis(500),
                            },
                        );
                    }
                });

                match self.freematics_last_frame_received_at {
                    Some(last) if last.elapsed() > Duration::from_millis(1500) => {
                        ui.label(
                            RichText::new(format!(
                                "Telemetry delayed · last frame {:.1}s ago",
                                last.elapsed().as_secs_f32()
                            ))
                            .small()
                            .color(Color32::from_rgb(215, 170, 85)),
                        );
                    }
                    None => {
                        ui.label(
                            RichText::new("Waiting for telemetry frame")
                                .small()
                                .color(ui.visuals().weak_text_color()),
                        );
                    }
                    _ => {}
                }

                let health = self.freematics_acquisition_health;
                let obd_state = match health.obd_state {
                    Some(0) => "disconnected".to_string(),
                    Some(1) => "initializing".to_string(),
                    Some(2) => "connected".to_string(),
                    Some(3) => "failed".to_string(),
                    Some(value) => format!("unknown ({value})"),
                    None => "not reported".to_string(),
                };
                let timeouts = health
                    .cumulative_timeouts
                    .map_or_else(|| "—".to_string(), |value| value.to_string());
                let latency = health
                    .last_request_latency_ms
                    .map_or_else(|| "—".to_string(), |value| format!("{value} ms"));
                let failures = health
                    .consecutive_fast_failures
                    .map_or_else(|| "—".to_string(), |value| value.to_string());
                ui.label(
                    RichText::new(format!(
                        "Acquisition · ECU {obd_state} · last request {latency} · timeouts {timeouts} · fast failures {failures}"
                    ))
                    .small()
                    .color(if health.consecutive_fast_failures.unwrap_or(0) > 0
                        || health.obd_state == Some(3)
                    {
                        Color32::from_rgb(215, 170, 85)
                    } else {
                        ui.visuals().weak_text_color()
                    }),
                );
                ui.label(
                    RichText::new(format!(
                        "Delivery · device drops {} · laptop drops {} · corrupt frames {}",
                        self.freematics_dropped_records,
                        self.freematics_reader_drops,
                        self.freematics_corrupt_records
                    ))
                    .small()
                    .color(ui.visuals().weak_text_color()),
                );
            }
        });
    }

    fn show_sensors(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new("Not connected").color(Color32::from_gray(120)));
            });
            return;
        }

        if self.is_freematics_usb() {
            ui.label("This connection is read-only; active diagnostic requests are unavailable.");
            let captured = self
                .freematics_capture_utc_ms
                .and_then(chrono::DateTime::<chrono::Utc>::from_timestamp_millis)
                .map(|timestamp| timestamp.to_rfc3339_opts(chrono::SecondsFormat::Millis, true))
                .unwrap_or_else(|| "UTC unavailable on device".to_string());
            ui.label(
                RichText::new(format!("Latest device frame captured: {captured}"))
                    .color(Color32::from_gray(145)),
            );
        } else {
            ui.horizontal(|ui| {
                if self.live_running {
                    if ui.button("Stop").clicked() {
                        self.stop_polling();
                    }
                } else if ui.button("Start").clicked() {
                    self.start_polling();
                    self.live_running = true;
                }
                if ui.button("Query Supported PIDs").clicked() {
                    self.send_cmd(OdbCmd::QuerySupportedPids);
                }
            });
        }
        ui.add_space(4.0);

        egui::ScrollArea::vertical().show(ui, |ui| {
            egui_extras::TableBuilder::new(ui)
                .striped(true)
                .column(egui_extras::Column::exact(70.0)) // PID
                .column(egui_extras::Column::remainder().at_least(200.0)) // Name
                .column(egui_extras::Column::exact(120.0)) // Value
                .column(egui_extras::Column::exact(60.0)) // Unit
                .column(egui_extras::Column::exact(100.0)) // Raw
                .column(egui_extras::Column::exact(115.0)) // Age / status
                .header(20.0, |mut header| {
                    header.col(|ui| {
                        ui.strong("PID");
                    });
                    header.col(|ui| {
                        ui.strong("Sensor");
                    });
                    header.col(|ui| {
                        ui.strong("Value");
                    });
                    header.col(|ui| {
                        ui.strong("Unit");
                    });
                    header.col(|ui| {
                        ui.strong("Raw");
                    });
                    header.col(|ui| {
                        ui.strong("Age / status");
                    });
                })
                .body(|mut body| {
                    let mut entries: Vec<_> = self.live_data.iter().collect();
                    entries.sort_by(|a, b| a.0.cmp(b.0));

                    for (cmd, state) in entries {
                        body.row(18.0, |mut row| {
                            row.col(|ui| {
                                ui.label(
                                    RichText::new(cmd.as_str())
                                        .color(Color32::from_rgb(80, 160, 220))
                                        .monospace(),
                                );
                            });
                            row.col(|ui| {
                                ui.label(&state.name);
                            });
                            row.col(|ui| {
                                ui.label(RichText::new(format!("{}", state.value)).strong());
                            });
                            row.col(|ui| {
                                ui.label(RichText::new(&state.unit).color(Color32::from_gray(140)));
                            });
                            row.col(|ui| {
                                ui.label(
                                    RichText::new(&state.raw)
                                        .monospace()
                                        .color(Color32::from_gray(100))
                                        .small(),
                                );
                            });
                            row.col(|ui| {
                                let age = Self::displayed_age_ms(state);
                                let stale = self.pid_is_stale(cmd, state);
                                let support = match state.supported {
                                    Some(false) => "unsupported".to_string(),
                                    Some(true) if stale => "stale".to_string(),
                                    Some(true) => age
                                        .map(|age| format!("{age} ms"))
                                        .unwrap_or_else(|| "age unavailable".into()),
                                    None if stale => "stale".to_string(),
                                    None => age
                                        .map(|age| format!("{age} ms"))
                                        .unwrap_or_else(|| "age unavailable".into()),
                                };
                                ui.colored_label(
                                    if stale {
                                        Color32::from_rgb(220, 170, 80)
                                    } else {
                                        Color32::from_gray(150)
                                    },
                                    support,
                                );
                            });
                        });
                    }
                });
        });
    }

    fn show_dtcs(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new("Not connected").color(Color32::from_gray(120)));
            });
            return;
        }

        if self.is_freematics_usb() {
            ui.label(
                RichText::new("Device-reported scan results · read-only")
                    .color(Color32::from_gray(150)),
            );
            ui.label(
                RichText::new(
                    "The firmware scans DTCs periodically. This USB adapter cannot trigger a scan or clear codes.",
                )
                .color(Color32::from_gray(125)),
            );
            ui.add_space(10.0);
            let now = Instant::now();
            let sections = [
                ("Stored", &self.freematics_dtcs[0]),
                ("Pending", &self.freematics_dtcs[1]),
                ("Permanent", &self.freematics_dtcs[2]),
            ];
            egui::ScrollArea::vertical().show(ui, |ui| {
                for (label, scan) in sections {
                    ui.heading(label);
                    let presentation = freematics_dtc_presentation(scan, now);
                    let (status_text, status_color) = match presentation.state {
                        FreematicsDtcState::NotReported => (
                            "Unsupported or not reported by this telemetry stream",
                            Color32::from_gray(125),
                        ),
                        FreematicsDtcState::NeverScanned => {
                            ("Never scanned by the device", Color32::from_gray(125))
                        }
                        FreematicsDtcState::Unknown => (
                            "Scan status is unknown; no conclusion about codes",
                            Color32::from_rgb(235, 165, 55),
                        ),
                        FreematicsDtcState::NoResponse => (
                            "Scan attempted; ECU did not respond",
                            Color32::from_rgb(235, 165, 55),
                        ),
                        FreematicsDtcState::RespondedNoCodes => (
                            "ECU responded · no codes reported",
                            Color32::from_rgb(70, 190, 100),
                        ),
                        FreematicsDtcState::Codes => (
                            "ECU reported diagnostic codes",
                            Color32::from_rgb(220, 100, 85),
                        ),
                        FreematicsDtcState::InvalidCodes => (
                            "ECU reported codes, but no valid code slots were present",
                            Color32::from_rgb(235, 165, 55),
                        ),
                    };
                    ui.colored_label(status_color, status_text);
                    let stale = presentation.stale;
                    let Some(age_ms) = presentation.age_ms else {
                        if !matches!(
                            presentation.state,
                            FreematicsDtcState::NotReported | FreematicsDtcState::NeverScanned
                        ) {
                            ui.label(
                                RichText::new(
                                    "Scan age unavailable; results cannot be freshness-checked",
                                )
                                .color(Color32::from_gray(125)),
                            );
                        }
                        ui.add_space(8.0);
                        continue;
                    };
                    let age_text = if age_ms < 1_000 {
                        format!("{} ms ago", age_ms)
                    } else {
                        format!("{:.1} s ago", age_ms as f64 / 1_000.0)
                    };
                    ui.horizontal(|ui| {
                        if let Some(count) = presentation.count {
                            ui.label(
                                RichText::new(format!("Reported count: {count}"))
                                    .color(Color32::from_gray(145)),
                            );
                        }
                        ui.label(
                            RichText::new(format!("Last scan {age_text}")).color(if stale {
                                Color32::from_rgb(235, 165, 55)
                            } else {
                                Color32::from_gray(145)
                            }),
                        );
                    });
                    if stale {
                        ui.label(
                            RichText::new(
                                "Scan is old; results may no longer reflect current ECU state",
                            )
                            .color(Color32::from_rgb(235, 165, 55)),
                        );
                    }
                    for dtc in &scan.codes {
                        ui.monospace(&dtc.code);
                    }
                    ui.add_space(12.0);
                }
            });
            return;
        }

        ui.horizontal(|ui| {
            if ui.button("Read DTCs").clicked() {
                self.send_cmd(OdbCmd::ReadDtcs {
                    make: self.vehicle_make(),
                });
            }
            #[cfg(not(target_arch = "wasm32"))]
            if self.elm_can_mode == crate::elm327::ElmCanMode::CorsaDMediumSpeed {
                ui.label(
                    RichText::new("Clear disabled for experimental MS-CAN")
                        .color(Color32::from_gray(130)),
                );
            } else if ui
                .button(RichText::new("Clear DTCs").color(Color32::from_rgb(220, 50, 50)))
                .clicked()
            {
                self.clear_dtc_confirm = true;
            }
            #[cfg(target_arch = "wasm32")]
            if ui
                .button(RichText::new("Clear DTCs").color(Color32::from_rgb(220, 50, 50)))
                .clicked()
            {
                self.clear_dtc_confirm = true;
            }
            if !self.dtc_status.is_empty() {
                ui.label(RichText::new(&self.dtc_status).color(Color32::from_gray(140)));
            }
        });
        ui.add_space(8.0);

        egui::ScrollArea::vertical().show(ui, |ui| {
            let all_empty = self.stored_dtcs.is_empty() && self.pending_dtcs.is_empty();

            if !all_empty {
                let sections: &[(&str, &[obd::Dtc], Color32)] = &[
                    ("Stored", &self.stored_dtcs, Color32::from_rgb(220, 50, 50)),
                    ("Pending", &self.pending_dtcs, Color32::from_rgb(220, 180, 50)),
                ];

                for (label, dtcs, color) in sections {
                    if dtcs.is_empty() {
                        continue;
                    }
                    ui.heading(RichText::new(format!("{} DTCs ({})", label, dtcs.len())).color(*color));
                    ui.add_space(4.0);

                    egui_extras::TableBuilder::new(ui)
                        .striped(true)
                        .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                        .column(egui_extras::Column::exact(90.0))   // Code
                        .column(egui_extras::Column::remainder())    // Description
                        .column(egui_extras::Column::initial(300.0).at_least(300.0))  // Source
                        .header(20.0, |mut row| {
                            row.col(|ui| { ui.strong("Code"); });
                            row.col(|ui| { ui.strong("Description"); });
                            row.col(|ui| { ui.strong("Source"); });
                        })
                        .body(|mut body| {
                            for dtc in *dtcs {
                                body.row(22.0, |mut row| {
                                    row.col(|ui| {
                                        ui.label(
                                            RichText::new(&dtc.code)
                                                .strong()
                                                .color(*color)
                                                .monospace(),
                                        );
                                    });
                                    row.col(|ui| {
                                        match &dtc.desc_source {
                                            DescSource::Pending => {
                                                ui.label(
                                                    RichText::new("Looking up description…")
                                                        .color(Color32::from_gray(100))
                                                        .italics(),
                                                );
                                            }
                                            DescSource::NotFound => {
                                                ui.label(
                                                    RichText::new("No description found")
                                                        .color(Color32::from_gray(110))
                                                        .italics(),
                                                );
                                            }
                                            _ => { ui.label(&dtc.description); }
                                        }
                                    });
                                    row.col(|ui| {
                                        match &dtc.desc_source {
                                            DescSource::Pending => {
                                                ui.label(
                                                    RichText::new("loading…")
                                                        .color(Color32::from_gray(80))
                                                        .italics(),
                                                );
                                            }
                                            DescSource::Family(canonical) => {
                                                let family = crate::dtc_database::family_label(canonical);
                                                let vehicle = self.vehicle_make()
                                                    .unwrap_or_else(|| "this vehicle".to_string());
                                                ui.label(
                                                    RichText::new(format!("via {canonical} · {family}"))
                                                        .color(Color32::from_rgb(180, 140, 60)),
                                                ).on_hover_text(format!(
                                                    "No {vehicle}-specific description was found.\n\
                                                     {canonical} is in the same corporate family ({family})\
                                                     \nand shares DTC codes with {vehicle}."
                                                ));
                                            }
                                            DescSource::Sae => {
                                                ui.label(
                                                    RichText::new("SAE J2012")
                                                        .color(Color32::from_gray(120)),
                                                ).on_hover_text(
                                                    "Generic SAE J2012 standard description — \
                                                     no manufacturer-specific entry found."
                                                );
                                            }
                                            _ => {}
                                        }
                                    });
                                });
                            }
                        });

                    ui.add_space(12.0);
                }
            } else if !self.dtc_status.is_empty() {
                ui.label(
                    RichText::new("No trouble codes found")
                        .color(Color32::from_rgb(50, 200, 80))
                        .size(16.0),
                );
            }
        });
    }

    fn show_freeze_frame(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new("Not connected").color(Color32::from_gray(120)));
            });
            return;
        }

        if self.is_freematics_usb() {
            self.show_freematics_freeze_frame(ui);
            return;
        }

        if ui.button("Read Freeze Frame").clicked() {
            self.freeze_data.clear();
            self.freeze_frame_read = true;
            self.send_cmd(OdbCmd::ReadFreezeFrame);
        }
        ui.add_space(8.0);

        if self.freeze_data.is_empty() {
            if self.freeze_frame_read {
                ui.label(
                    RichText::new("No freeze frame data available.").color(Color32::from_gray(140)),
                );
                ui.add_space(4.0);
                ui.label(
                    RichText::new(
                        "Freeze frame data is only captured when a DTC (trouble code) is stored. \
                        If your car has no active DTCs, the freeze frame buffer will be empty.",
                    )
                    .color(Color32::from_gray(100)),
                );
            } else {
                ui.label(
                    RichText::new("Click 'Read Freeze Frame' to fetch snapshot data.")
                        .color(Color32::from_gray(120)),
                );
            }
        } else {
            egui::ScrollArea::vertical().show(ui, |ui| {
                egui_extras::TableBuilder::new(ui)
                    .striped(true)
                    .column(egui_extras::Column::remainder().at_least(200.0))
                    .column(egui_extras::Column::exact(120.0))
                    .column(egui_extras::Column::exact(60.0))
                    .header(20.0, |mut header| {
                        header.col(|ui| {
                            ui.strong("Sensor");
                        });
                        header.col(|ui| {
                            ui.strong("Value");
                        });
                        header.col(|ui| {
                            ui.strong("Unit");
                        });
                    })
                    .body(|mut body| {
                        for (name, value, unit) in &self.freeze_data {
                            body.row(18.0, |mut row| {
                                row.col(|ui| {
                                    ui.label(name);
                                });
                                row.col(|ui| {
                                    ui.label(RichText::new(format!("{value}")).strong());
                                });
                                row.col(|ui| {
                                    ui.label(unit);
                                });
                            });
                        }
                    });
            });
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn show_freematics_freeze_frame(&self, ui: &mut egui::Ui) {
        if self.freematics_freeze_data.is_empty() {
            let message = match self.freematics_freeze_status {
                Some(0) => "No stored DTC freeze frame has been captured.",
                Some(2) => "The ECU did not provide supported Mode 02 frame-0 values.",
                Some(3) => "Reading the ECU's stored Mode 02 frame-0 values…",
                Some(_) => "Waiting for passive freeze-frame telemetry from Freematics.",
                None => "Waiting for passive freeze-frame telemetry from Freematics.",
            };
            ui.label(RichText::new(message).color(Color32::from_gray(140)));
            return;
        }

        ui.horizontal(|ui| {
            ui.strong("ECU Mode 02 frame 0");
            if self.freematics_freeze_status == Some(3) {
                ui.label(
                    RichText::new("Capture in progress · partial values")
                        .color(Color32::from_rgb(220, 180, 80)),
                );
            }
            ui.label("Original fault-time timestamp is not available from this ECU response.");
            match self.freematics_freeze_read_age_ms {
                Some(age) => {
                    ui.label(format!("Device read age: {age} ms"));
                }
                None => {
                    ui.label(
                        RichText::new("Device read age unavailable")
                            .color(Color32::from_rgb(220, 180, 80)),
                    );
                }
            }
            if let Some(dtc) = self.freematics_freeze_trigger_dtc {
                ui.label(format!("Trigger DTC raw: 0x{dtc:04X}"));
            }
        });
        ui.add_space(8.0);
        egui::ScrollArea::vertical().show(ui, |ui| {
            egui_extras::TableBuilder::new(ui)
                .striped(true)
                .column(egui_extras::Column::exact(64.0))
                .column(egui_extras::Column::remainder().at_least(180.0))
                .column(egui_extras::Column::exact(120.0))
                .column(egui_extras::Column::exact(60.0))
                .header(20.0, |mut header| {
                    header.col(|ui| {
                        ui.strong("PID");
                    });
                    header.col(|ui| {
                        ui.strong("Sensor");
                    });
                    header.col(|ui| {
                        ui.strong("Value");
                    });
                    header.col(|ui| {
                        ui.strong("Unit");
                    });
                })
                .body(|mut body| {
                    for measurement in &self.freematics_freeze_data {
                        body.row(20.0, |mut row| {
                            row.col(|ui| {
                                ui.label(&measurement.cmd[2..]);
                            });
                            row.col(|ui| {
                                ui.label(&measurement.name);
                            });
                            row.col(|ui| {
                                ui.label(RichText::new(format!("{}", measurement.value)).strong());
                            });
                            row.col(|ui| {
                                ui.label(&measurement.unit);
                            });
                        });
                    }
                });
        });
    }

    fn show_vehicle_info(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new("Not connected").color(Color32::from_gray(120)));
            });
            return;
        }

        if self.is_freematics_usb() {
            ui.label("VIN, calibration ID, ECU name, and supported Mode 01 PIDs appear when available in device telemetry; active queries are unavailable.");
        } else {
            #[cfg(not(target_arch = "wasm32"))]
            if self.elm_can_mode == crate::elm327::ElmCanMode::CorsaDMediumSpeed {
                ui.label(
                    RichText::new("Experimental bus setup only: standard OBD requests remain read-only; Opel module-specific addressing and decoding are not implemented.")
                        .color(Color32::from_rgb(220, 180, 80)),
                );
            }
            ui.horizontal(|ui| {
                if ui.button("Read VIN").clicked() {
                    self.send_cmd(OdbCmd::ReadVin);
                }
                if ui.button("Query Supported PIDs").clicked() {
                    self.send_cmd(OdbCmd::QuerySupportedPids);
                }
                if ui.button("Read DTCs").clicked() {
                    self.send_cmd(OdbCmd::ReadDtcs {
                        make: self.vehicle_make(),
                    });
                }
            });
        }

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.add_space(12.0);

            // ── Vehicle section ─────────────────────────────────────
            ui.heading("Vehicle");
            ui.add_space(4.0);
            egui::Grid::new("vehicle_grid")
                .num_columns(2)
                .spacing([20.0, 6.0])
                .show(ui, |ui| {
                    ui.label(RichText::new("VIN:").strong());
                    if let Some(vin) = &self.vin {
                        ui.label(RichText::new(vin).monospace());
                    } else {
                        ui.label(RichText::new("Not read").color(Color32::from_gray(140)));
                    }
                    ui.end_row();

                    #[cfg(not(target_arch = "wasm32"))]
                    if self.is_freematics_usb() {
                        ui.label(RichText::new("Calibration ID:").strong());
                        ui.label(
                            self.freematics_calibration_id
                                .as_deref()
                                .unwrap_or("Not available in telemetry"),
                        );
                        ui.end_row();
                        ui.label(RichText::new("ECU name:").strong());
                        ui.label(
                            self.freematics_ecu_name
                                .as_deref()
                                .unwrap_or("Not available in telemetry"),
                        );
                        ui.end_row();
                    }

                    if let Some(vin) = &self.vin {
                        let info = crate::vin_decoder::decode(vin);
                        if info.make != "Unknown" {
                            ui.label(RichText::new("Make:").strong());
                            ui.label(&info.make);
                            ui.end_row();
                        }
                        if info.country != "Unknown" {
                            ui.label(RichText::new("Country:").strong());
                            ui.label(&info.country);
                            ui.end_row();
                        }
                        if let Some(year) = &info.year {
                            ui.label(RichText::new("Model Year:").strong());
                            ui.label(year);
                            ui.end_row();
                        }
                        ui.label(RichText::new("WMI:").strong());
                        ui.label(RichText::new(&info.wmi).monospace());
                        ui.end_row();
                    }

                    if let Some(v) = &self.voltage {
                        ui.label(RichText::new("Adapter-reported Voltage:").strong());
                        ui.label(v);
                        ui.end_row();
                    }

                    if self.is_freematics_usb() {
                        ui.label(RichText::new("Device-reported supported Mode 01 PIDs:").strong());
                        ui.label(if !self.freematics_support_reported {
                            "Not reported yet".to_string()
                        } else if self.supported_pids.is_empty() {
                            "Reported: none supported".to_string()
                        } else {
                            format!("{}", self.supported_pids.len())
                        });
                        ui.end_row();
                    }
                });

            ui.add_space(16.0);

            // ── Adapter section ─────────────────────────────────────
            ui.heading("Adapter");
            ui.add_space(4.0);
            if let Some(info) = &self.connection_info {
                egui::Grid::new("adapter_grid")
                    .num_columns(2)
                    .spacing([20.0, 6.0])
                    .show(ui, |ui| {
                        ui.label(RichText::new("Adapter interface:").strong());
                        ui.label(&info.elm_version);
                        ui.end_row();

                        ui.label(RichText::new("Protocol:").strong());
                        ui.label(&info.protocol);
                        ui.end_row();

                        ui.label(RichText::new("Port:").strong());
                        ui.label(RichText::new(&info.port).monospace());
                        ui.end_row();

                        if info.baud != 0 {
                            ui.label(RichText::new("Serial baud rate:").strong());
                            ui.label(format!("{} baud", info.baud));
                            ui.end_row();
                        }
                    });
            }

            ui.add_space(16.0);

            // ── DTC summary section ─────────────────────────────────
            ui.heading("Diagnostics");
            ui.add_space(4.0);
            egui::Grid::new("diag_grid")
                .num_columns(2)
                .spacing([20.0, 6.0])
                .show(ui, |ui| {
                    ui.label(RichText::new("Stored DTCs:").strong());
                    if self.stored_dtcs.is_empty() {
                        ui.label(RichText::new("None").color(Color32::from_rgb(50, 200, 80)));
                    } else {
                        ui.label(
                            RichText::new(format!("{}", self.stored_dtcs.len()))
                                .color(Color32::from_rgb(220, 50, 50))
                                .strong(),
                        );
                    }
                    ui.end_row();

                    ui.label(RichText::new("Pending DTCs:").strong());
                    if self.pending_dtcs.is_empty() {
                        ui.label(RichText::new("None").color(Color32::from_rgb(50, 200, 80)));
                    } else {
                        ui.label(
                            RichText::new(format!("{}", self.pending_dtcs.len()))
                                .color(Color32::from_rgb(220, 180, 50))
                                .strong(),
                        );
                    }
                    ui.end_row();

                    // Status from Mode 01 PID 01 if available
                    if let Some(state) = self.live_data.get("0101") {
                        ui.label(RichText::new("MIL Status:").strong());
                        ui.label(format!("{}", state.value));
                        ui.end_row();
                    }

                    if let Some(state) = self.live_data.get("011C") {
                        ui.label(RichText::new("OBD Standard:").strong());
                        ui.label(format!("{}", state.value));
                        ui.end_row();
                    }

                    if let Some(state) = self.live_data.get("0151") {
                        ui.label(RichText::new("Fuel Type:").strong());
                        ui.label(format!("{}", state.value));
                        ui.end_row();
                    }

                    if let Some(state) = self.live_data.get("011F") {
                        let secs = state.numeric_value;
                        let hours = (secs / 3600.0) as u32;
                        let mins = ((secs % 3600.0) / 60.0) as u32;
                        ui.label(RichText::new("Run Time:").strong());
                        ui.label(format!("{}h {}m", hours, mins));
                        ui.end_row();
                    }

                    if let Some(state) = self.live_data.get("0131") {
                        ui.label(RichText::new("Distance Since Clear:").strong());
                        ui.label(format!("{:.0} km", state.numeric_value));
                        ui.end_row();
                    }

                    if let Some(state) = self.live_data.get("0130") {
                        ui.label(RichText::new("Warm-ups Since Clear:").strong());
                        ui.label(format!("{:.0}", state.numeric_value));
                        ui.end_row();
                    }
                });

            ui.add_space(16.0);

            // ── Supported PIDs section ────────────────────────────
            if !self.supported_pids.is_empty()
                || (self.is_freematics_usb() && self.freematics_support_reported)
            {
                ui.heading("Supported PIDs");
                ui.add_space(4.0);

                if self.supported_pids.is_empty() {
                    ui.label("Completed scan: no supported Mode 01 PIDs reported");
                } else {
                    ui.label(format!(
                        "{} PIDs supported by this vehicle",
                        self.supported_pids.len()
                    ));
                }
                ui.add_space(4.0);

                let pid_names: HashMap<u8, &str> = self
                    .pid_defs
                    .iter()
                    .filter_map(|p| {
                        u8::from_str_radix(&p.cmd[2..4], 16)
                            .ok()
                            .map(|pid| (pid, p.description))
                    })
                    .collect();

                ui.horizontal_wrapped(|ui| {
                    for &pid in &self.supported_pids {
                        let name = pid_names.get(&pid).unwrap_or(&"");
                        let label = format!("{pid:02X}");
                        ui.label(
                            RichText::new(label)
                                .monospace()
                                .color(Color32::from_rgb(80, 160, 220)),
                        )
                        .on_hover_text(*name);
                    }
                });
            }
        });
    }

    fn show_log(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.log_auto_scroll, "Auto-scroll");
            if ui.button("Clear").clicked() {
                self.log_messages.clear();
                self.log_last_count = 0;
            }
            if ui.button("Copy").clicked() {
                let text = self.log_messages.join("\n");
                platform_copy(ui.ctx(), &text);
            }
            ui.label(
                RichText::new(format!("{} lines", self.log_messages.len()))
                    .color(Color32::from_gray(80))
                    .small(),
            );
        });

        let num_messages = self.log_messages.len();

        let scroll = egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .max_height(ui.available_height())
            .stick_to_bottom(self.log_auto_scroll);

        let response = scroll.show_rows(
            ui,
            14.0, // row height
            num_messages,
            |ui, row_range| {
                for i in row_range {
                    if let Some(line) = self.log_messages.get(i) {
                        let color = log_line_color(line);
                        ui.label(
                            RichText::new(line.as_str())
                                .monospace()
                                .color(color)
                                .size(10.5),
                        );
                    }
                }
            },
        );

        // Force scroll to bottom when new messages arrive
        if self.log_auto_scroll && num_messages != self.log_last_count && num_messages > 0 {
            ui.scroll_to_rect(
                response.inner_rect.translate(egui::vec2(0.0, f32::MAX)),
                Some(egui::Align::BOTTOM),
            );
        }
        self.log_last_count = num_messages;
    }
}

fn log_line_color(line: &str) -> Color32 {
    if line.contains("[ERROR]") {
        Color32::from_rgb(220, 50, 50)
    } else if line.contains("[DTC_STORED]") || line.contains("[DTC_PENDING]") {
        Color32::from_rgb(220, 180, 50)
    } else if line.contains("[CONNECTED]") || line.contains("[VIN]") {
        Color32::from_rgb(50, 200, 80)
    } else if line.contains("[VALUE_CHANGE]") {
        Color32::from_rgb(80, 160, 220)
    } else if line.contains("[CONNECT]") {
        Color32::from_rgb(100, 180, 220)
    } else {
        Color32::from_gray(130)
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn is_usb_serial_port_name(port: &str) -> bool {
    port.starts_with("/dev/ttyUSB")
        || port.starts_with("/dev/ttyACM")
        || port.starts_with("/dev/serial/by-id/")
        || port.starts_with("/dev/cu.usb")
        || port.to_ascii_uppercase().starts_with("COM")
}

fn should_request_live_repaint(
    live_running: bool,
    connecting: bool,
    passive_telemetry_connected: bool,
) -> Option<Duration> {
    (live_running || connecting || passive_telemetry_connected)
        .then_some(Duration::from_millis(100))
}

impl eframe::App for ObdApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.process_events();

        // Freematics is a passive stream, so live_running stays false; keep
        // repainting while connected so queued telemetry is applied without
        // requiring mouse/keyboard input.
        #[cfg(not(target_arch = "wasm32"))]
        let passive_telemetry_connected = self.is_freematics_usb() && self.connected;
        #[cfg(target_arch = "wasm32")]
        let passive_telemetry_connected = false;
        if let Some(interval) = should_request_live_repaint(
            self.live_running,
            self.connecting,
            passive_telemetry_connected,
        ) {
            ctx.request_repaint_after(interval);
        }

        // Dark theme
        if self.dark_mode {
            ctx.set_visuals(egui::Visuals::dark());
        } else {
            ctx.set_visuals(egui::Visuals::light());
        }

        egui::TopBottomPanel::top("connection_bar").show(ctx, |ui| {
            ui.add_space(4.0);
            self.show_connection_bar(ui);
            ui.add_space(2.0);
            ui.separator();
            self.show_tab_bar(ui);
            ui.add_space(2.0);
        });

        // Log panel as resizable bottom pane
        if self.log_panel_open {
            egui::TopBottomPanel::bottom("log_panel")
                .resizable(true)
                .min_height(60.0)
                .default_height(self.log_panel_height)
                .show(ctx, |ui| {
                    self.log_panel_height = ui.available_height();
                    self.show_log(ui);
                });
        }

        egui::CentralPanel::default().show(ctx, |ui| match self.active_tab {
            Tab::Dashboard => self.show_dashboard(ui),
            Tab::Sensors => self.show_sensors(ui),
            Tab::DtcCodes => self.show_dtcs(ui),
            Tab::FreezeFrame => self.show_freeze_frame(ui),
            Tab::VehicleInfo => self.show_vehicle_info(ui),
        });

        // Clear DTCs confirmation modal
        if self.clear_dtc_confirm {
            egui::Window::new("Clear Trouble Codes")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                .show(ctx, |ui| {
                    ui.add_space(8.0);
                    ui.label(
                        RichText::new("Are you sure you want to clear all DTCs?")
                            .strong()
                            .size(15.0),
                    );
                    ui.add_space(8.0);
                    ui.label("This will:");
                    ui.label("  - Clear all stored diagnostic trouble codes");
                    ui.label("  - Clear all pending trouble codes");
                    ui.label("  - Reset the MIL (Check Engine Light)");
                    ui.label("  - Erase freeze frame data");
                    ui.label("  - Reset I/M readiness monitors");
                    ui.add_space(12.0);
                    ui.horizontal(|ui| {
                        if ui
                            .button(
                                RichText::new("Yes, Clear All")
                                    .color(Color32::from_rgb(220, 50, 50)),
                            )
                            .clicked()
                        {
                            self.send_cmd(OdbCmd::ClearDtcs);
                            self.clear_dtc_confirm = false;
                        }
                        if ui.button("Cancel").clicked() {
                            self.clear_dtc_confirm = false;
                        }
                    });
                    ui.add_space(4.0);
                });
        }
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod adapter_ui_tests {
    use super::*;
    use std::collections::HashSet;

    fn rendered_freematics_dtcs(app: &mut ObdApp, context: &egui::Context) -> String {
        let output = context.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(900.0, 600.0),
                )),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_dtcs(ui));
            },
        );
        output
            .shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::Shape::Text(text) => Some(text.galley.job.text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join(" ")
    }

    #[test]
    fn connected_freematics_stream_schedules_ui_repaints_without_diagnostic_polling() {
        assert_eq!(
            should_request_live_repaint(false, false, true),
            Some(Duration::from_millis(100))
        );
        assert_eq!(should_request_live_repaint(false, false, false), None);
    }

    #[test]
    fn freematics_dtc_view_model_distinguishes_unreported_scan_outcomes_and_staleness() {
        use crate::freematics_usb::FreematicsDtcAvailability as Availability;

        let now = Instant::now();
        let mut scan = FreematicsDtcScan::default();
        assert_eq!(
            freematics_dtc_presentation(&scan, now).state,
            FreematicsDtcState::NotReported
        );

        scan.availability = Availability::NoScan;
        assert_eq!(
            freematics_dtc_presentation(&scan, now).state,
            FreematicsDtcState::NeverScanned
        );

        scan.availability = Availability::Fresh;
        scan.status = Some(1);
        scan.count = Some(0);
        scan.age_ms = Some(250);
        scan.received_at = Some(now);
        let responded = freematics_dtc_presentation(&scan, now);
        assert_eq!(responded.state, FreematicsDtcState::RespondedNoCodes);
        assert_eq!(responded.count, Some(0));
        assert_eq!(responded.age_ms, Some(250));
        assert!(!responded.stale);

        scan.status = Some(2);
        scan.count = Some(3);
        scan.codes.push(Dtc {
            code: "P0134".into(),
            description: String::new(),
            desc_source: DescSource::NotFound,
        });
        assert_eq!(
            freematics_dtc_presentation(&scan, now).state,
            FreematicsDtcState::Codes
        );

        scan.received_at = Some(now - Duration::from_secs(121));
        assert!(freematics_dtc_presentation(&scan, now).stale);
        scan.status = None;
        scan.availability = Availability::UnknownStatus;
        assert_eq!(
            freematics_dtc_presentation(&scan, now).state,
            FreematicsDtcState::Unknown
        );
    }

    #[test]
    fn freematics_dtc_ui_labels_unreported_never_scanned_and_ecu_no_codes() {
        let (commands, command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;
        app.connected = true;
        app.freematics_dtcs[1].availability =
            crate::freematics_usb::FreematicsDtcAvailability::NoScan;
        app.freematics_dtcs[2] = FreematicsDtcScan {
            availability: crate::freematics_usb::FreematicsDtcAvailability::Fresh,
            status: Some(1),
            count: Some(0),
            age_ms: Some(10),
            received_at: Some(Instant::now()),
            codes: Vec::new(),
        };

        let context = egui::Context::default();
        let text = rendered_freematics_dtcs(&mut app, &context);
        assert!(text.contains("Unsupported or not reported"));
        assert!(text.contains("Never scanned by the device"));
        assert!(text.contains("ECU responded · no codes reported"));
        assert!(text.contains("Reported count: 0"));
        app.freematics_dtcs[0] = FreematicsDtcScan {
            availability: crate::freematics_usb::FreematicsDtcAvailability::Fresh,
            status: Some(1),
            count: Some(0),
            age_ms: Some(120_001),
            received_at: Some(Instant::now()),
            codes: Vec::new(),
        };
        assert!(rendered_freematics_dtcs(&mut app, &context).contains("Scan is old"));
        assert!(
            command_rx.try_recv().is_err(),
            "Freematics USB DTC view stays passive"
        );
    }

    #[test]
    fn freematics_history_keeps_device_utc_and_monotonic_capture_time() {
        assert_eq!(
            freematics_sample_utc_ms(1_250, Some(1_790_966_400_000), 1_150),
            Some(1_790_966_399_900)
        );
        assert_eq!(freematics_sample_utc_ms(1_250, None, 1_150), None);
    }

    #[test]
    fn freematics_cached_dtc_vin_and_supported_pids_are_passive() {
        let (commands, command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;
        app.connected = true;
        let fields = [
            (0x300, 1.0),
            (0x301, 0x0134 as f64),
            (0x310, 2.0),
            (0x360, 500.0),
            (0x320, 0.0),
            (0x330, 1.0),
            (0x361, 500.0),
            (0x340, 0.0),
            (0x350, 1.0),
            (0x362, 500.0),
            (0x20C, 812.5),
            (0x363, 2_400.0),
            (0x364, 1.0),
            (0x365, 1281.0),
            (0x10C, 812.5),
            (0x40C, 100.0),
        ]
        .into_iter()
        .map(|(pid, value)| crate::freematics_usb::TelemetryField {
            pid,
            values: vec![value],
        })
        .collect();
        let frame = crate::freematics_usb::FreematicsFrame {
            boot_id: 7,
            capture_ms: 1_250,
            reader_received_at: Instant::now(),
            capture_utc_ms: Some(1_790_966_400_000),
            capture_sequence: None,
            dropped_records: 0,
            supported_pids: Some(HashSet::from([0x0C, 0x0D])),
            raw_mode01: HashMap::new(),
            vin: Some("1HGCM82633A004352".into()),
            calibration_id: Some("CAL-123".into()),
            ecu_name: Some("ENGINE".into()),
            fields,
            corrupt_records: 0,
            corrupt_sample_hex: None,
            reader_drops: 0,
        };

        app.apply_freematics_frame(frame.clone());
        assert_eq!(app.freematics_capture_utc_ms, Some(1_790_966_400_000));
        let rpm_history = &app.live_data["010C"].history;
        assert_eq!(rpm_history.len(), 1);
        assert_eq!(rpm_history[0].capture_utc_ms, Some(1_790_966_399_900));
        assert_eq!(
            rpm_history[0].captured_at,
            frame.reader_received_at - Duration::from_millis(100)
        );
        assert_eq!(app.vin.as_deref(), Some("1HGCM82633A004352"));
        assert_eq!(app.freematics_calibration_id.as_deref(), Some("CAL-123"));
        assert_eq!(app.freematics_ecu_name.as_deref(), Some("ENGINE"));
        assert!(app.freematics_support_reported);
        assert_eq!(
            app.supported_pids.iter().copied().collect::<HashSet<_>>(),
            HashSet::from([0x0C, 0x0D])
        );
        assert_eq!(app.freematics_dtcs[0].codes[0].code, "P0134");
        assert_eq!(app.freematics_dtcs[0].count, Some(1));
        assert_eq!(
            app.freematics_dtcs[0].availability,
            crate::freematics_usb::FreematicsDtcAvailability::Fresh
        );
        assert_eq!(app.freematics_dtcs[1].count, Some(0));
        assert_eq!(app.freematics_dtcs[1].status, Some(1));
        assert_eq!(app.freematics_freeze_data.len(), 1);
        assert_eq!(app.freematics_freeze_data[0].cmd, "020C");
        assert_eq!(app.freematics_freeze_data[0].value, 812.5);
        assert_eq!(app.freematics_freeze_data[0].age_ms, Some(2_400));
        assert_eq!(app.freematics_freeze_status, Some(1));
        assert_eq!(app.freematics_freeze_trigger_dtc, Some(1281));
        assert!(
            command_rx.try_recv().is_err(),
            "passive telemetry must not send diagnostic commands"
        );

        let context = egui::Context::default();
        let _ = context.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(900.0, 600.0),
                )),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_dtcs(ui));
            },
        );
        assert!(
            command_rx.try_recv().is_err(),
            "drawing cached DTCs must remain passive"
        );
        let output = context.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(900.0, 600.0),
                )),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_freeze_frame(ui));
            },
        );
        let rendered = output
            .shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::Shape::Text(text) => Some(text.galley.job.text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        assert!(rendered.contains("ECU Mode 02 frame 0"));
        assert!(rendered.contains("Device read age: 2400 ms"));
        assert!(rendered.contains("Original fault-time timestamp is not available"));
        assert!(rendered.contains("Engine RPM"));
        assert!(rendered.contains("812.5"));
        assert!(!rendered.contains("requests are unavailable"));
        assert!(
            command_rx.try_recv().is_err(),
            "rendering passive freeze-frame telemetry must not send diagnostics"
        );

        let mut next_capture = frame.clone();
        next_capture.capture_ms += 1;
        next_capture.fields.retain(|field| {
            (field.pid < 0x200 || field.pid > 0x2ff) && !(0x363..=0x365).contains(&field.pid)
        });
        next_capture.fields.extend([
            crate::freematics_usb::TelemetryField {
                pid: 0x364,
                values: vec![3.0],
            },
            crate::freematics_usb::TelemetryField {
                pid: 0x365,
                values: vec![1282.0],
            },
        ]);
        app.apply_freematics_frame(next_capture.clone());
        assert!(app.freematics_freeze_data.is_empty());
        assert_eq!(app.freematics_freeze_status, Some(3));

        next_capture.capture_ms += 1;
        next_capture
            .fields
            .push(crate::freematics_usb::TelemetryField {
                pid: 0x20d,
                values: vec![7.0],
            });
        app.apply_freematics_frame(next_capture);
        assert_eq!(app.freematics_freeze_data.len(), 1);
        assert_eq!(app.freematics_freeze_data[0].cmd, "020D");
        let partial_output = context.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(900.0, 600.0),
                )),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_freeze_frame(ui));
            },
        );
        let partial_text = partial_output
            .shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::Shape::Text(text) => Some(text.galley.job.text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        assert!(partial_text.contains("Capture in progress · partial values"));
        assert!(partial_text.contains("Vehicle Speed"));
        let _ = context.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(900.0, 600.0),
                )),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_vehicle_info(ui));
            },
        );
        assert!(
            command_rx.try_recv().is_err(),
            "drawing cached vehicle identity must remain passive"
        );

        let mut omitted_dtc_frame = frame.clone();
        omitted_dtc_frame.capture_ms += 250;
        omitted_dtc_frame.reader_received_at += Duration::from_millis(250);
        omitted_dtc_frame
            .fields
            .retain(|field| !(0x300..=0x362).contains(&field.pid));
        app.apply_freematics_frame(omitted_dtc_frame);
        assert_eq!(app.freematics_dtcs[0].count, Some(1));
        assert_eq!(app.freematics_dtcs[0].codes[0].code, "P0134");
    }

    #[test]
    fn freematics_support_ui_state_distinguishes_unknown_and_empty() {
        let (commands, _command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;

        let mut frame = crate::freematics_usb::FreematicsFrame {
            boot_id: 7,
            capture_ms: 1_250,
            reader_received_at: Instant::now(),
            capture_utc_ms: None,
            capture_sequence: None,
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
        };
        app.apply_freematics_frame(frame.clone());
        assert!(app.freematics_support_reported);
        assert!(app.supported_pids.is_empty());

        frame.capture_ms += 1;
        frame.reader_received_at = Instant::now();
        frame.supported_pids = None;
        app.apply_freematics_frame(frame);
        assert!(!app.freematics_support_reported);
        assert!(app.supported_pids.is_empty());
    }

    #[test]
    fn freematics_raw_status_and_compound_o2_values_reach_sensor_rows_passively() {
        let (commands, command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;
        app.connected = true;
        let fields = [(0x101, 133.0), (0x104, 50.0), (0x114, 0.5)]
            .into_iter()
            .map(|(pid, value)| crate::freematics_usb::TelemetryField {
                pid,
                values: vec![value],
            })
            .collect();
        let frame = crate::freematics_usb::FreematicsFrame {
            boot_id: 42,
            capture_ms: 1000,
            reader_received_at: Instant::now(),
            capture_utc_ms: None,
            capture_sequence: None,
            dropped_records: 0,
            supported_pids: Some(HashSet::from([0x01, 0x04, 0x14])),
            raw_mode01: HashMap::from([
                (0x01, vec![0x85, 0x08, 0x01, 0x00]),
                (0x04, vec![0x80]),
                (0x14, vec![0x64, 0x90]),
            ]),
            vin: None,
            calibration_id: None,
            ecu_name: None,
            fields,
            corrupt_records: 0,
            corrupt_sample_hex: None,
            reader_drops: 0,
        };

        app.apply_freematics_frame(frame);
        assert!(matches!(
            app.live_data["0101"].value,
            ObdValue::StatusResult(ref status) if status.mil_on && status.dtc_count == 5
        ));
        assert!(app.live_data["0101"].raw.contains("raw=85080100"));
        assert!(app.live_data["0104"].raw.contains("raw=80"));
        assert!((app.live_data["0114-TRIM"].numeric_value - 12.5).abs() < 0.001);
        assert!(command_rx.try_recv().is_err());
    }

    #[test]
    fn adapter_selection_routes_commands_and_reconnection_clears_vehicle_state() {
        let (commands, command_rx) = mpsc::channel();
        let (events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        let context = egui::Context::default();
        for kind in [
            ConnectionKind::Serial,
            ConnectionKind::FreematicsUsb,
            ConnectionKind::Tcp,
            ConnectionKind::J2534,
        ] {
            app.connection_kind = kind;
            app.tcp_address = "127.0.0.1:35000".into();
            app.j2534_library = "C:\\Vendor\\driver.dll".into();
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1100.0, 750.0),
                )),
                ..Default::default()
            };
            let output = context.run(input, |ctx| {
                egui::TopBottomPanel::top("connection").show(ctx, |ui| app.show_connection_bar(ui));
                egui::CentralPanel::default().show(ctx, |ui| app.show_dashboard(ui));
            });
            assert!(!output.shapes.is_empty());
            app.connect_selected();
            match (command_rx.recv().unwrap(), &app.connection_kind) {
                (OdbCmd::Connect { .. }, ConnectionKind::Serial) => {}
                (OdbCmd::ConnectFreematicsUsb(_), ConnectionKind::FreematicsUsb) => {}
                (OdbCmd::ConnectAdapter(NativeConnection::Tcp(address)), ConnectionKind::Tcp) => {
                    assert_eq!(address, "127.0.0.1:35000");
                }
                (
                    OdbCmd::ConnectAdapter(NativeConnection::J2534 { library, .. }),
                    ConnectionKind::J2534,
                ) => {
                    assert_eq!(library, app.j2534_library);
                }
                _ => panic!("Selected adapter did not reach worker command"),
            }
        }
        app.connected = true;
        app.vin = Some("OLD VEHICLE".into());
        app.voltage = Some("12.6V".into());
        events
            .send(ObdEvent::Connecting("New adapter".into()))
            .unwrap();
        app.process_events();
        assert!(!app.connected);
        assert!(app.vin.is_none());
        assert!(app.voltage.is_none());
    }

    #[test]
    fn freematics_port_can_stay_pending_until_protocol_is_confirmed() {
        let (commands, _command_rx) = mpsc::channel();
        let (events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;

        events
            .send(ObdEvent::Connecting(
                "Listening for a checksummed Freematics frame; port stays open".into(),
            ))
            .unwrap();
        app.process_events();
        assert!(app.connecting);
        assert!(!app.connected);
        assert!(app.connection_status.contains("port stays open"));

        events
            .send(ObdEvent::Connected(ConnectionInfo {
                port: "/dev/ttyUSB0".into(),
                baud: 460_800,
                protocol: "Freematics Telemetry v1".into(),
                elm_version: "Passive TeleLogger USB stream".into(),
                voltage: None,
            }))
            .unwrap();
        app.process_events();
        assert!(!app.connecting);
        assert!(app.connected);
    }

    #[test]
    fn freematics_frames_preserve_ages_and_clear_gauge_history_on_restart() {
        let (commands, _command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;

        let frame_for = |boot: u64, capture: u32, rpm: u32, age: u32| {
            let payload =
                format!("ABCDEF#0:{capture},10C:{rpm},40C:{age},1A6:123456.7,4A6:15,24:1375,94:25");
            let checksum = payload
                .bytes()
                .fold(0u8, |sum, byte| sum.wrapping_add(byte));
            let wire =
                format!("@FT1,{boot},{capture},1,1790966401000,0,0C,A6|{payload}*{checksum:02X}");
            crate::freematics_usb::parse_line(wire.as_bytes()).unwrap()
        };

        app.apply_freematics_frame(frame_for(42, 1000, 820, 10));
        let displayed_age = ObdApp::displayed_age_ms(app.live_data.get("010C").unwrap()).unwrap();
        assert!((10..=100).contains(&displayed_age));
        assert!(!app.pid_is_stale("010C", app.live_data.get("010C").unwrap()));
        assert_eq!(
            app.live_data
                .get("010C")
                .unwrap()
                .history
                .iter()
                .map(|point| point.value)
                .collect::<Vec<_>>(),
            [820.0]
        );
        let odometer = app.live_data.get("01A6").unwrap();
        assert_eq!(odometer.numeric_value, 123456.7);
        assert_eq!(odometer.unit, "km");
        assert_eq!(odometer.age_ms, Some(15));
        assert_eq!(odometer.supported, Some(true));
        assert_eq!(
            odometer
                .history
                .iter()
                .map(|point| point.value)
                .collect::<Vec<_>>(),
            [123456.7]
        );

        app.apply_freematics_frame(frame_for(42, 1250, 540, 1500));
        assert!(app.pid_is_stale("010C", app.live_data.get("010C").unwrap()));
        assert_eq!(
            app.live_data
                .get("010C")
                .unwrap()
                .history
                .iter()
                .map(|point| point.value)
                .collect::<Vec<_>>(),
            [820.0]
        );

        app.apply_freematics_frame(frame_for(99, 250, 810, 8));
        assert_eq!(app.freematics_boot_id, Some(99));
        assert_eq!(
            app.live_data
                .get("010C")
                .unwrap()
                .history
                .iter()
                .map(|point| point.value)
                .collect::<Vec<_>>(),
            [810.0]
        );
    }

    #[test]
    fn freematics_voltage_and_motion_waveforms_use_device_capture_time() {
        let (commands, _command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;

        let payload = "ABCDEF#0:100,24:1375,94:0,A0:50;1370,A0:75;1372,A1:60,A2:0.1;0.2;0.3";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let wire = format!("@FT1,7,100,0,0,0,|{payload}*{checksum:02X}");
        app.apply_freematics_frame(crate::freematics_usb::parse_line(wire.as_bytes()).unwrap());

        assert_eq!(
            app.freematics_supply_history
                .iter()
                .map(|point| point.value)
                .collect::<Vec<_>>(),
            [13.7, 13.72]
        );
        assert_eq!(app.freematics_motion_history.len(), 1);
        assert!((app.freematics_motion_history[0].value - 0.374_165_738_677_394_17).abs() < 1e-12);
        assert_eq!(
            app.freematics_supply_history[1]
                .captured_at
                .duration_since(app.freematics_supply_history[0].captured_at),
            Duration::from_millis(25)
        );
        let frame_received = app.freematics_last_frame_received_at.unwrap();
        assert_eq!(
            frame_received.duration_since(app.freematics_supply_history[1].captured_at),
            Duration::from_millis(25)
        );
        assert_eq!(app.freematics_capture_utc_ms, None);
    }

    #[test]
    fn queued_freematics_frames_keep_their_serial_receive_age() {
        let (commands, _command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;

        let payload = "ABCDEF#0:100,10C:800,40C:50,24:1375";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let wire = format!("@FT1,42,100,0,0,0,0C,0D|{payload}*{checksum:02X}");
        let mut frame = crate::freematics_usb::parse_line(wire.as_bytes()).unwrap();
        frame.reader_received_at = Instant::now() - Duration::from_secs(2);
        let serial_received_at = frame.reader_received_at;
        app.apply_freematics_frame(frame);

        let rpm = app.live_data.get("010C").unwrap();
        assert!(app.pid_is_stale("010C", rpm));
        assert!(ObdApp::displayed_age_ms(rpm).unwrap() >= 2_050);
        assert_eq!(
            app.freematics_last_frame_received_at,
            Some(serial_received_at)
        );
        assert!(
            Instant::now().duration_since(app.freematics_last_frame_received_at.unwrap())
                >= Duration::from_secs(2)
        );
    }

    #[test]
    fn freematics_dashboard_renders_waveforms_and_gap_diagnostics() {
        let (commands, _command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;
        app.connected = true;

        let payload = "ABCDEF#0:100,87:3,88:42,89:2,8A:1,24:1375,94:0,A0:50;1370,A0:75;1372,A1:60,A2:0.1;0.2;0.3";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let wire = format!("@FT1,7,100,0,0,0,|{payload}*{checksum:02X}");
        app.apply_freematics_frame(crate::freematics_usb::parse_line(wire.as_bytes()).unwrap());

        let context = egui::Context::default();
        let output = context.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1100.0, 900.0),
                )),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_dashboard(ui));
            },
        );
        let rendered_text: Vec<_> = output
            .shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(text) => Some(text.galley.job.text.as_str()),
                _ => None,
            })
            .collect();
        for expected in [
            "Vehicle battery voltage",
            "Acceleration magnitude",
            "Acquisition · ECU connected",
            "Delivery · device drops",
        ] {
            assert!(
                rendered_text.iter().any(|text| text.contains(expected)),
                "dashboard omitted {expected:?}"
            );
        }

        app.freematics_last_frame_received_at = Some(Instant::now() - Duration::from_secs(2));
        let output = context.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1100.0, 900.0),
                )),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_dashboard(ui));
            },
        );
        assert!(output.shapes.iter().any(|shape| match &shape.shape {
            egui::epaint::Shape::Text(text) => {
                text.galley.job.text.contains("Telemetry delayed")
            }
            _ => false,
        }));
    }

    #[test]
    fn device_drop_counter_restart_is_reported_from_the_new_boot_baseline() {
        let (commands, _command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;

        let frame_for = |boot: u64, capture: u32, dropped: u32| {
            let payload = format!("ABCDEF#0:{capture},10C:800,40C:5");
            let checksum = payload
                .bytes()
                .fold(0u8, |sum, byte| sum.wrapping_add(byte));
            let wire = format!("@FT1,{boot},{capture},0,0,{dropped},0C|{payload}*{checksum:02X}");
            crate::freematics_usb::parse_line(wire.as_bytes()).unwrap()
        };

        app.apply_freematics_frame(frame_for(10, 100, 7));
        app.log_messages.clear();
        app.apply_freematics_frame(frame_for(11, 20, 1));

        assert!(
            app.log_messages
                .iter()
                .any(|line| { line.contains("[FREEMATICS_USB_DROPS] cumulative=1") })
        );
    }

    #[test]
    fn freematics_rejects_replayed_capture_but_accepts_counter_wrap() {
        let (commands, _command_rx) = mpsc::channel();
        let (_events, event_rx) = mpsc::channel();
        let (_telemetry_tx, telemetry_rx) = mpsc::channel();
        let mut app = ObdApp::new_state(commands, event_rx, telemetry_rx, None);
        app.connection_kind = ConnectionKind::FreematicsUsb;

        let frame_for = |capture: u32, rpm: u32| {
            let payload = format!("ABCDEF#0:{capture},10C:{rpm},40C:5");
            let checksum = payload
                .bytes()
                .fold(0u8, |sum, byte| sum.wrapping_add(byte));
            let wire = format!("@FT1,42,{capture},0,0,0,0C|{payload}*{checksum:02X}");
            crate::freematics_usb::parse_line(wire.as_bytes()).unwrap()
        };

        app.apply_freematics_frame(frame_for(u32::MAX - 10, 820));
        app.apply_freematics_frame(frame_for(u32::MAX - 20, 100));
        assert_eq!(
            app.live_data.get("010C").unwrap().numeric_value,
            820.0,
            "a delayed capture from this boot must not overwrite newer telemetry"
        );

        app.apply_freematics_frame(frame_for(20, 540));
        assert_eq!(
            app.live_data.get("010C").unwrap().numeric_value,
            540.0,
            "the device capture counter may wrap from u32::MAX to zero"
        );
    }
}
