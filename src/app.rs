use crate::elm327::{self, ConnectionInfo};
use crate::gauges::{BarGauge, RadialGauge, sparkline};
use crate::obd::{self, Dtc, ObdValue, PidDef};
use egui::{self, Color32, RichText};
use std::collections::HashMap;
use std::io::Write;
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

// ── Messages between OBD thread and GUI ─────────────────────────────────────

#[derive(Debug)]
pub enum OdbCmd {
    Connect {
        port: Option<String>,
        baud: Option<u32>,
    },
    Disconnect,
    StartLiveData,
    StopLiveData,
    ReadDtcs,
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

    // Connection state
    connected: bool,
    connecting: bool,
    connection_info: Option<ConnectionInfo>,
    connection_status: String,

    // Port selection
    available_ports: Vec<String>,
    selected_port: Option<String>,
    selected_baud: Option<u32>,
    #[allow(dead_code)]
    auto_connect: bool,

    // Live data
    live_data: HashMap<String, LivePidState>,
    live_running: bool,
    supported_pids: Vec<u8>,

    // DTCs
    stored_dtcs: Vec<Dtc>,
    pending_dtcs: Vec<Dtc>,
    dtc_status: String,
    clear_dtc_confirm: bool,

    // Freeze frame
    freeze_data: Vec<(String, ObdValue, String)>,
    freeze_frame_read: bool,

    // Vehicle info
    vin: Option<String>,
    voltage: Option<String>,

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

    // Log file writer
    log_file: Arc<Mutex<std::fs::File>>,

    // Screen wake lock (child process handle)
    wake_lock: Option<std::process::Child>,
}

struct LivePidState {
    name: String,
    value: ObdValue,
    unit: String,
    numeric_value: f64,
    history: Vec<f64>,
    last_update: Instant,
    raw: String,
}

impl ObdApp {
    pub fn new(
        _cc: &eframe::CreationContext<'_>,
        cmd_tx: mpsc::Sender<OdbCmd>,
        event_rx: mpsc::Receiver<ObdEvent>,
        log_file: Arc<Mutex<std::fs::File>>,
    ) -> Self {
        let available_ports = elm327::scan_ports();
        let pid_defs = obd::mode01_pids();

        Self {
            cmd_tx,
            event_rx,
            connected: false,
            connecting: false,
            connection_info: None,
            connection_status: "Disconnected".to_string(),
            available_ports,
            selected_port: None,
            selected_baud: None,
            auto_connect: true,
            live_data: HashMap::new(),
            live_running: false,
            supported_pids: Vec::new(),
            stored_dtcs: Vec::new(),
            pending_dtcs: Vec::new(),
            dtc_status: String::new(),
            clear_dtc_confirm: false,
            freeze_data: Vec::new(),
            freeze_frame_read: false,
            vin: None,
            voltage: None,
            active_tab: Tab::Dashboard,
            log_messages: Vec::new(),
            log_auto_scroll: true,
            log_panel_open: true,
            log_panel_height: 180.0,
            log_last_count: 0,
            poll_config: PollConfig::default(),
            dark_mode: true,
            pid_defs,
            log_file,
            wake_lock: None,
        }
    }

    fn process_events(&mut self) {
        while let Ok(event) = self.event_rx.try_recv() {
            match event {
                ObdEvent::Connecting(msg) => {
                    self.connecting = true;
                    self.connection_status = msg.clone();
                    self.add_log(&format!("[CONNECT] {msg}"));
                }
                ObdEvent::Connected(info) => {
                    self.connected = true;
                    self.connecting = false;
                    self.connection_status = format!(
                        "Connected: {} @ {} baud | {}",
                        info.port, info.baud, info.protocol
                    );
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
                            last_update: Instant::now(),
                            raw: raw.clone(),
                        });
                    state.value = value;
                    state.unit = unit;
                    state.numeric_value = numeric;
                    state.raw = raw;
                    state.last_update = Instant::now();
                    state.history.push(numeric);
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
    }

    fn add_log(&mut self, msg: &str) {
        let timestamp = chrono::Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
        let line = format!("{timestamp} {msg}");

        // Write to stdout
        println!("{line}");

        // Write to log file
        if let Ok(mut f) = self.log_file.lock() {
            let _ = writeln!(f, "{line}");
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

    /// Check if engine appears to be running based on RPM > 0
    fn engine_running(&self) -> bool {
        self.live_data
            .get("010C")
            .is_some_and(|s| s.numeric_value > 0.0)
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

    fn show_connection_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            // Status indicator
            let (status_color, status_text) = if self.connected {
                (Color32::from_rgb(50, 200, 80), "Connected")
            } else if self.connecting {
                (Color32::from_rgb(220, 180, 50), "Connecting...")
            } else {
                (Color32::from_rgb(180, 50, 50), "Disconnected")
            };
            ui.colored_label(status_color, RichText::new(status_text).strong());
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
                // Port selector
                egui::ComboBox::from_label("")
                    .selected_text(self.selected_port.as_deref().unwrap_or("Auto-detect"))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.selected_port, None, "Auto-detect");
                        for port in &self.available_ports {
                            ui.selectable_value(&mut self.selected_port, Some(port.clone()), port);
                        }
                    });

                if ui.button("Refresh ports").clicked() {
                    self.available_ports = elm327::scan_ports();
                }

                if ui.button(RichText::new("Connect").strong()).clicked() {
                    self.send_cmd(OdbCmd::Connect {
                        port: self.selected_port.clone(),
                        baud: self.selected_baud,
                    });
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
            extern "system" {
                fn SetThreadExecutionState(flags: u32) -> u32;
            }
            unsafe {
                SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED);
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
    }

    fn release_wake_lock(&mut self) {
        #[cfg(target_os = "windows")]
        {
            if self.wake_lock.is_some() {
                const ES_CONTINUOUS: u32 = 0x80000000;
                #[link(name = "kernel32")]
                extern "system" {
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
    }

    fn show_dashboard(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.vertical_centered(|ui| {
                ui.add_space(ui.available_height() * 0.25);

                // Port selector
                ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Port:").color(Color32::from_gray(140)));
                        egui::ComboBox::from_id_salt("port_select_main")
                            .width(200.0)
                            .selected_text(self.selected_port.as_deref().unwrap_or("Auto-detect"))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.selected_port, None, "Auto-detect");
                                for port in &self.available_ports {
                                    ui.selectable_value(
                                        &mut self.selected_port,
                                        Some(port.clone()),
                                        port,
                                    );
                                }
                            });
                        if ui.small_button("Refresh").clicked() {
                            self.available_ports = crate::elm327::scan_ports();
                        }
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

                if ui.add(button).clicked() {
                    self.send_cmd(OdbCmd::Connect {
                        port: self.selected_port.clone(),
                        baud: self.selected_baud,
                    });
                }

                ui.add_space(12.0);

                if self.connecting {
                    ui.spinner();
                    ui.label(
                        RichText::new(&self.connection_status)
                            .color(Color32::from_rgb(220, 180, 50)),
                    );
                } else {
                    ui.label(
                        RichText::new("Select a port or auto-detect and connect")
                            .color(Color32::from_gray(100)),
                    );
                }
            });
            return;
        }

        // Show big start button when not polling
        if !self.live_running {
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
                    }
                });
            });

            ui.add_space(4.0);

            // ── Second row: 4 smaller gauges ────────────────────────────
            ui.columns(4, |cols| {
                let gauges: [(
                    usize,
                    &str,
                    &str,
                    f64,
                    f64,
                    &str,
                    Option<f64>,
                    Option<f64>,
                    usize,
                ); 4] = [
                    (
                        0,
                        "0105",
                        "Coolant",
                        -40.0,
                        215.0,
                        "\u{00B0}C",
                        Some(100.0),
                        Some(115.0),
                        0,
                    ),
                    (
                        1,
                        "015C",
                        "Oil Temp",
                        -40.0,
                        215.0,
                        "\u{00B0}C",
                        Some(120.0),
                        Some(140.0),
                        0,
                    ),
                    (2, "0111", "Throttle", 0.0, 100.0, "%", None, None, 1),
                    (
                        3,
                        "0104",
                        "Load",
                        0.0,
                        100.0,
                        "%",
                        Some(80.0),
                        Some(95.0),
                        1,
                    ),
                ];
                for (i, cmd, label, min, max, unit, warn, danger, dec) in gauges {
                    cols[i].vertical_centered(|ui| {
                        if let Some(s) = self.live_data.get(cmd) {
                            let mut g = RadialGauge::new(label, s.numeric_value, min, max, unit)
                                .size(small_gauge)
                                .decimals(dec);
                            if let Some(w) = warn {
                                g = g.warning(w);
                            }
                            if let Some(d) = danger {
                                g = g.danger(d);
                            }
                            g.show(ui);
                        }
                    });
                }
            });

            ui.add_space(6.0);
            ui.separator();
            ui.add_space(4.0);

            // ── Bottom section: two columns ─────────────────────────────
            // Left: bar gauges   Right: sparklines
            ui.columns(2, |cols| {
                // ── Left: bar gauges ────────────────────────────────────
                cols[0].vertical(|ui| {
                    ui.label(
                        RichText::new("Sensors")
                            .size(13.0)
                            .color(Color32::from_gray(160)),
                    );
                    ui.add_space(4.0);

                    let bar_w = (ui.available_width() - 130.0).max(80.0);

                    let bar_pids: &[(&str, &str, f64, f64, &str)] = &[
                        ("012F", "Fuel Level", 0.0, 100.0, "%"),
                        ("0142", "Battery", 0.0, 18.0, "V"),
                        ("010F", "Intake Temp", -40.0, 215.0, "\u{00B0}C"),
                        ("0110", "MAF", 0.0, 655.35, "g/s"),
                        ("010B", "Intake kPa", 0.0, 255.0, "kPa"),
                        ("010E", "Timing", -64.0, 63.5, "\u{00B0}"),
                        ("0106", "STFT B1", -100.0, 99.2, "%"),
                        ("0107", "LTFT B1", -100.0, 99.2, "%"),
                        ("0133", "Baro", 0.0, 255.0, "kPa"),
                        ("0146", "Ambient", -40.0, 215.0, "\u{00B0}C"),
                        ("012C", "EGR", 0.0, 100.0, "%"),
                        ("012E", "Evap", 0.0, 100.0, "%"),
                        ("0149", "Accel Pos", 0.0, 100.0, "%"),
                        ("0144", "Equiv \u{03BB}", 0.0, 2.0, "\u{03BB}"),
                    ];

                    for &(cmd, label, min, max, unit) in bar_pids {
                        if let Some(state) = self.live_data.get(cmd) {
                            BarGauge::new(label, state.numeric_value, min, max, unit)
                                .width(bar_w)
                                .decimals(1)
                                .show(ui);
                        }
                    }
                });

                // ── Right: sparkline trends ─────────────────────────────
                cols[1].vertical(|ui| {
                    ui.label(
                        RichText::new("Trends")
                            .size(13.0)
                            .color(Color32::from_gray(160)),
                    );
                    ui.add_space(4.0);

                    let spark_w = (ui.available_width() - 60.0).max(100.0);

                    let sparkline_pids: &[(&str, &str, Color32)] = &[
                        ("010C", "RPM", Color32::from_rgb(220, 100, 100)),
                        ("010D", "Speed", Color32::from_rgb(80, 160, 220)),
                        ("0105", "Coolant", Color32::from_rgb(220, 180, 50)),
                        ("0111", "Throttle", Color32::from_rgb(50, 200, 80)),
                        ("0104", "Load", Color32::from_rgb(180, 120, 220)),
                        ("012F", "Fuel", Color32::from_rgb(100, 200, 200)),
                    ];

                    for &(cmd, label, color) in sparkline_pids {
                        if let Some(state) = self.live_data.get(cmd) {
                            if state.history.len() >= 2 {
                                ui.horizontal(|ui| {
                                    ui.label(
                                        RichText::new(format!("{label:>8}"))
                                            .monospace()
                                            .color(color)
                                            .size(11.0),
                                    );
                                    sparkline(ui, &state.history, spark_w, 28.0, color);
                                    // Current value next to sparkline
                                    ui.label(
                                        RichText::new(format!("{:.0}", state.numeric_value))
                                            .monospace()
                                            .color(Color32::from_gray(180))
                                            .size(11.0),
                                    );
                                });
                                ui.add_space(2.0);
                            }
                        }
                    }
                });
            });
        });
    }

    fn show_sensors(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new("Not connected").color(Color32::from_gray(120)));
            });
            return;
        }

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
        ui.add_space(4.0);

        egui::ScrollArea::vertical().show(ui, |ui| {
            egui_extras::TableBuilder::new(ui)
                .striped(true)
                .column(egui_extras::Column::exact(70.0)) // PID
                .column(egui_extras::Column::remainder().at_least(200.0)) // Name
                .column(egui_extras::Column::exact(120.0)) // Value
                .column(egui_extras::Column::exact(60.0)) // Unit
                .column(egui_extras::Column::exact(100.0)) // Raw
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

        ui.horizontal(|ui| {
            if ui.button("Read DTCs").clicked() {
                self.send_cmd(OdbCmd::ReadDtcs);
            }
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
            if !self.stored_dtcs.is_empty() {
                ui.heading(
                    RichText::new(format!("Stored DTCs ({})", self.stored_dtcs.len()))
                        .color(Color32::from_rgb(220, 50, 50)),
                );
                for dtc in &self.stored_dtcs {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(&dtc.code)
                                .strong()
                                .color(Color32::from_rgb(220, 50, 50))
                                .monospace(),
                        );
                        if !dtc.description.is_empty() {
                            ui.label(&dtc.description);
                        }
                    });
                }
                ui.add_space(8.0);
            }

            if !self.pending_dtcs.is_empty() {
                ui.heading(
                    RichText::new(format!("Pending DTCs ({})", self.pending_dtcs.len()))
                        .color(Color32::from_rgb(220, 180, 50)),
                );
                for dtc in &self.pending_dtcs {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(&dtc.code)
                                .strong()
                                .color(Color32::from_rgb(220, 180, 50))
                                .monospace(),
                        );
                        if !dtc.description.is_empty() {
                            ui.label(&dtc.description);
                        }
                    });
                }
            }

            if self.stored_dtcs.is_empty()
                && self.pending_dtcs.is_empty()
                && !self.dtc_status.is_empty()
            {
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

    fn show_vehicle_info(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new("Not connected").color(Color32::from_gray(120)));
            });
            return;
        }

        ui.horizontal(|ui| {
            if ui.button("Read VIN").clicked() {
                self.send_cmd(OdbCmd::ReadVin);
            }
            if ui.button("Query Supported PIDs").clicked() {
                self.send_cmd(OdbCmd::QuerySupportedPids);
            }
            if ui.button("Read DTCs").clicked() {
                self.send_cmd(OdbCmd::ReadDtcs);
            }
        });

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
                        ui.label(RichText::new("Battery Voltage:").strong());
                        ui.label(v);
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
                        ui.label(RichText::new("ELM Version:").strong());
                        ui.label(&info.elm_version);
                        ui.end_row();

                        ui.label(RichText::new("Protocol:").strong());
                        ui.label(&info.protocol);
                        ui.end_row();

                        ui.label(RichText::new("Port:").strong());
                        ui.label(RichText::new(&info.port).monospace());
                        ui.end_row();

                        ui.label(RichText::new("Baud Rate:").strong());
                        ui.label(format!("{} baud", info.baud));
                        ui.end_row();
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
            if !self.supported_pids.is_empty() {
                ui.heading("Supported PIDs");
                ui.add_space(4.0);

                ui.label(format!(
                    "{} PIDs supported by this vehicle",
                    self.supported_pids.len()
                ));
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
                ui.ctx().copy_text(text);
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

impl eframe::App for ObdApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.process_events();

        // Request repaint while live data is running
        if self.live_running || self.connecting {
            ctx.request_repaint_after(Duration::from_millis(100));
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
