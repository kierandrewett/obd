//! OBD-II Vehicle Emulator
//!
//! Creates a virtual serial port (PTY) that behaves exactly like an ELM327 adapter
//! connected to a real vehicle. Connect any OBD tool (including obd-dashboard) to
//! the displayed `/dev/pts/N` path at any standard baud rate.
//!
//! The control panel lets you drive the simulated engine: start/stop, accelerate,
//! brake, set voltage, fuel level, inject DTCs, etc.

#[cfg(target_arch = "wasm32")]
fn main() {}

#[cfg(not(target_arch = "wasm32"))]
mod emulator {
    use eframe::egui::{self, Color32, Key, RichText, Slider};
    use std::ffi::CStr;
    use std::net::TcpListener;
    use std::os::unix::io::RawFd;
    use std::sync::{Arc, Mutex};
    use std::thread;
    use std::time::{Duration, Instant};
    use tungstenite::accept;

    // ── Simulator state ──────────────────────────────────────────────────────

    pub struct SimState {
        // Engine
        pub engine_on: bool,
        pub rpm: f32,
        pub throttle: f32,    // 0-100 %
        pub engine_load: f32, // 0-100 %

        // Motion
        pub speed: f32, // km/h

        // Temperatures (°C)
        pub coolant_temp: f32,
        pub intake_temp: f32,
        pub oil_temp: f32,
        pub ambient_temp: f32,

        // Fuel & air
        pub maf: f32,             // g/s
        pub fuel_level: f32,      // 0-100 %
        pub short_fuel_trim: f32, // -25..+25 %
        pub long_fuel_trim: f32,
        pub intake_pressure: f32, // kPa
        pub baro_pressure: f32,   // kPa
        pub timing_advance: f32,  // degrees

        // Electrical
        pub voltage: f32,      // V
        pub base_voltage: f32, // Slider baseline

        // Runtime (seconds since engine start)
        pub runtime_secs: u32,

        // DTCs
        pub stored_dtcs: Vec<String>,
        pub pending_dtcs: Vec<String>,
        pub mil_on: bool,

        // Vehicle info
        pub vin: String,

        // Physics inputs (written by UI, read by physics)
        pub accel: bool,
        pub brake: bool,
        pub idle_rpm: f32,
        pub max_rpm: f32,
        pub max_speed: f32,

        // Transmission
        pub gear: u8, // 1–6

        // Physics internals
        warmup: f32, // 0-1 warmup progress
        last_update: Option<Instant>,
        engine_start_time: Option<Instant>,

        // Diagnostics
        pub connected: bool,
        pub last_cmd: String,
        pub cmd_count: u64,
    }

    impl Default for SimState {
        fn default() -> Self {
            Self {
                engine_on: false,
                rpm: 0.0,
                throttle: 0.0,
                engine_load: 0.0,
                speed: 0.0,
                coolant_temp: 22.0,
                intake_temp: 22.0,
                oil_temp: 22.0,
                ambient_temp: 22.0,
                maf: 0.0,
                fuel_level: 78.0,
                short_fuel_trim: 0.0,
                long_fuel_trim: 0.0,
                intake_pressure: 101.0,
                baro_pressure: 101.0,
                timing_advance: 12.0,
                voltage: 12.4,
                base_voltage: 14.2,
                runtime_secs: 0,
                stored_dtcs: Vec::new(),
                pending_dtcs: Vec::new(),
                mil_on: false,
                vin: "1HGCM82633A004352".to_string(),
                accel: false,
                brake: false,
                idle_rpm: 800.0,
                max_rpm: 6500.0,
                max_speed: 220.0,
                gear: 0,
                warmup: 0.0,
                last_update: None,
                engine_start_time: None,
                connected: false,
                last_cmd: String::new(),
                cmd_count: 0,
            }
        }
    }

    impl SimState {
        /// Step the physics simulation by `dt` seconds.
        pub fn physics_tick(&mut self) {
            let now = Instant::now();
            let dt = self
                .last_update
                .map(|t| now.duration_since(t).as_secs_f32().min(0.1))
                .unwrap_or(0.016);
            self.last_update = Some(now);

            if self.engine_on {
                // Runtime counter
                if let Some(start) = self.engine_start_time {
                    self.runtime_secs = now.duration_since(start).as_secs() as u32;
                }

                // Warmup: 0→1 over 3 minutes
                self.warmup = (self.warmup + dt / 180.0).min(1.0);

                // Throttle follows accel key; bleeds off otherwise
                if self.accel {
                    self.throttle = (self.throttle + 150.0 * dt).min(100.0);
                } else {
                    self.throttle = (self.throttle - 120.0 * dt).max(0.0);
                }
                if self.brake {
                    self.throttle = 0.0;
                }

                // Gear ratios expressed as RPM-per-km/h at the wheels
                // (final_drive × gear_ratio / tyre_circumference × unit_conversion)
                // gear == 0 means neutral
                const GEAR_FACTORS: [f32; 6] = [124.2, 66.4, 43.9, 33.2, 26.9, 22.0];
                const UPSHIFT_RPM: f32 = 2700.0;
                const DOWNSHIFT_RPM: f32 = 1300.0;

                let in_gear = self.gear > 0;
                let g = self.gear.saturating_sub(1).min(5) as usize;
                let wheel_rpm = if in_gear {
                    self.speed * GEAR_FACTORS[g]
                } else {
                    0.0
                };

                // RPM: engine braking (off-throttle in gear) vs throttle-driven
                if in_gear && self.throttle < 5.0 && self.speed > 4.0 {
                    // Wheels drive the engine — RPM follows wheel speed
                    let target = wheel_rpm.max(self.idle_rpm * 0.85);
                    self.rpm += (target - self.rpm) * (dt * 10.0).min(1.0);
                } else {
                    // Throttle drives RPM; in gear also can't drop below wheel demand
                    let throttle_rpm = self.idle_rpm
                        + (self.max_rpm - self.idle_rpm) * (self.throttle / 100.0).powf(0.65);
                    let rpm_target = if in_gear {
                        throttle_rpm.max(wheel_rpm)
                    } else {
                        throttle_rpm
                    };
                    let rpm_rate = if rpm_target > self.rpm {
                        3500.0
                    } else {
                        2000.0
                    };
                    let delta = (rpm_target - self.rpm).abs().min(rpm_rate * dt);
                    self.rpm += (rpm_target - self.rpm).signum() * delta;
                    self.rpm = self.rpm.clamp(self.idle_rpm * 0.95, self.max_rpm);
                }

                // Neutral below 4 km/h; auto-shift above
                if self.speed < 4.0 {
                    self.gear = 0; // neutral
                } else {
                    if self.gear == 0 {
                        self.gear = 1; // engage 1st when rolling
                    }
                    // Upshift
                    if self.gear < 6 {
                        let next_wheel_rpm = self.speed * GEAR_FACTORS[g + 1];
                        if self.rpm > UPSHIFT_RPM && next_wheel_rpm > DOWNSHIFT_RPM * 0.9 {
                            self.gear += 1;
                        }
                    }
                    // Downshift
                    if self.gear > 1 && wheel_rpm < DOWNSHIFT_RPM * 0.85 {
                        self.gear -= 1;
                    }
                }

                // Speed
                let drive_force = (self.throttle / 100.0) * 8.0; // km/h per second at full throttle
                let drag = self.speed * 0.03;
                // Engine braking: only when in gear and off throttle
                let engine_brake = if in_gear && self.throttle < 5.0 && self.speed > 4.0 {
                    ((self.rpm - self.idle_rpm).max(0.0) / 1000.0) * 6.0
                } else {
                    0.0
                };
                if self.brake {
                    self.speed = (self.speed - 45.0 * dt).max(0.0);
                } else {
                    self.speed = (self.speed + (drive_force - drag - engine_brake) * dt)
                        .clamp(0.0, self.max_speed);
                }

                // Derived quantities
                self.engine_load =
                    (self.throttle * 0.75 + (self.rpm / self.max_rpm) * 25.0).clamp(0.0, 100.0);
                self.maf = self.engine_load / 100.0 * 28.0 + 1.2;
                self.intake_pressure =
                    (self.baro_pressure - self.engine_load / 100.0 * 45.0).clamp(20.0, 110.0);
                self.timing_advance = (18.0 - self.engine_load / 100.0 * 10.0).clamp(2.0, 25.0);

                // Warmup of coolant / oil
                let target_coolant = self.ambient_temp + self.warmup * (92.0 - self.ambient_temp);
                self.coolant_temp += (target_coolant - self.coolant_temp) * (dt * 0.4).min(1.0);

                let target_oil = self.ambient_temp + self.warmup * (96.0 - self.ambient_temp);
                self.oil_temp += (target_oil - self.oil_temp) * (dt * 0.25).min(1.0);

                // Voltage: alternator boosts above 12V, sags under heavy load
                let alt_voltage = self.base_voltage - (self.engine_load / 100.0) * 1.2;
                self.voltage += (alt_voltage - self.voltage) * (dt * 2.0).min(1.0);
            } else {
                // Engine off: coast / cool down
                self.rpm = (self.rpm - 5000.0 * dt).max(0.0);
                self.throttle = 0.0;
                self.engine_load = 0.0;
                self.maf = 0.0;
                self.gear = 0;

                self.speed = if self.brake {
                    (self.speed - 45.0 * dt).max(0.0)
                } else {
                    (self.speed - 4.0 * dt).max(0.0)
                };

                self.coolant_temp += (self.ambient_temp - self.coolant_temp) * (dt * 0.04).min(1.0);
                self.oil_temp += (self.ambient_temp - self.oil_temp) * (dt * 0.03).min(1.0);
                self.voltage += (12.3 - self.voltage) * (dt * 0.8).min(1.0);
            }

            // Intake air temp loosely follows ambient + engine heat
            let heat = if self.engine_on {
                self.engine_load / 100.0 * 12.0
            } else {
                0.0
            };
            self.intake_temp += (self.ambient_temp + heat - self.intake_temp) * (dt * 0.3).min(1.0);
        }

        /// Build the ELM327 response string for an incoming command (no prompt, no \r).
        pub fn respond(&mut self, raw: &str) -> String {
            let cmd = raw.trim().to_uppercase();
            self.last_cmd = cmd.clone();
            self.cmd_count += 1;
            self.connected = true;

            // Handle "ATST XX" style with argument
            let base = cmd.split_whitespace().next().unwrap_or(&cmd);

            match base {
                // ── AT init commands ─────────────────────────────────────────
                "ATZ" => return "ELM327 v2.1".into(),
                "ATI" => return "ELM327 v2.1".into(),
                "ATRV" => return format!("{:.1}V", self.voltage),
                c if c.starts_with("AT") => return "OK".into(),

                // ── Supported PIDs ────────────────────────────────────────────
                "0100" => return "4100BE3F8003".into(),
                "0120" => return "412000022001".into(),
                "0140" => return "414044000011".into(),
                "0160" => return "416000000000".into(),

                // ── VIN (Mode 09 PID 02) ──────────────────────────────────────
                "0902" => {
                    let hex: String = self.vin.bytes().map(|b| format!("{b:02X}")).collect();
                    return format!("490201{hex}");
                }

                // ── Mode 01 live data ─────────────────────────────────────────
                "010C" => {
                    let v = (self.rpm * 4.0) as u32;
                    return format!("410C{v:04X}");
                }
                "010D" => return format!("410D{:02X}", self.speed as u32),
                "0104" => return format!("4104{:02X}", (self.engine_load / 100.0 * 255.0) as u32),
                "0105" => return format!("4105{:02X}", (self.coolant_temp as i32 + 40) as u32),
                "0111" => return format!("4111{:02X}", (self.throttle / 100.0 * 255.0) as u32),
                "010F" => return format!("410F{:02X}", (self.intake_temp as i32 + 40) as u32),
                "0110" => {
                    let v = (self.maf * 100.0) as u32;
                    return format!("4110{v:04X}");
                }
                "012F" => return format!("412F{:02X}", (self.fuel_level / 100.0 * 255.0) as u32),
                "0106" => {
                    let v = ((self.short_fuel_trim / 100.0 + 1.0) * 128.0) as u32;
                    return format!("4106{v:02X}");
                }
                "0107" => {
                    let v = ((self.long_fuel_trim / 100.0 + 1.0) * 128.0) as u32;
                    return format!("4107{v:02X}");
                }
                "010B" => return format!("410B{:02X}", self.intake_pressure as u32),
                "010E" => {
                    let v = ((self.timing_advance + 64.0) * 2.0) as u32;
                    return format!("410E{v:02X}");
                }
                "015C" => return format!("415C{:02X}", (self.oil_temp as i32 + 40) as u32),
                "0142" => {
                    let v = (self.voltage * 1000.0) as u32;
                    return format!("4142{v:04X}");
                }
                "0146" => return format!("4146{:02X}", (self.ambient_temp as i32 + 40) as u32),
                "0133" => return format!("4133{:02X}", self.baro_pressure as u32),
                "011F" => return format!("411F{:04X}", self.runtime_secs.min(0xFFFF)),

                // ── Mode 03 / 07: stored and pending DTCs ─────────────────────
                "03" => {
                    if self.stored_dtcs.is_empty() {
                        return "430000000000".into();
                    }
                    return format!("43{}", encode_dtcs(&self.stored_dtcs));
                }
                "07" => {
                    if self.pending_dtcs.is_empty() {
                        return "470000000000".into();
                    }
                    return format!("47{}", encode_dtcs(&self.pending_dtcs));
                }

                // ── Mode 04: clear DTCs ───────────────────────────────────────
                "04" => {
                    self.stored_dtcs.clear();
                    self.pending_dtcs.clear();
                    self.mil_on = false;
                    return "44".into();
                }

                // ── Mode 02: freeze frame — mirror Mode 01 with Mode 02 prefix
                c if c.starts_with("02") && c.len() == 6 => {
                    let pid = &c[2..4];
                    let pid01 = format!("01{pid}");
                    let r01 = self.respond(&pid01);
                    // r01 = "41XX..." → rewrite as "42XX00..."
                    if r01.len() >= 4 && r01.starts_with("41") {
                        let data = &r01[4..];
                        return format!("42{pid}00{data}");
                    }
                    return "NO DATA".into();
                }

                _ => return "NO DATA".into(),
            }
        }
    }

    // ── DTC encoding ─────────────────────────────────────────────────────────

    fn encode_dtcs(dtcs: &[String]) -> String {
        let mut out = String::new();
        for dtc in dtcs {
            if let Some((b1, b2)) = encode_dtc(dtc) {
                out.push_str(&format!("{b1:02X}{b2:02X}"));
            }
        }
        // Pad to even DTC pairs (minimum two zero-pairs so the dashboard parser
        // doesn't confuse a short response as "no data")
        while out.len() < 8 {
            out.push_str("0000");
        }
        out
    }

    fn encode_dtc(dtc: &str) -> Option<(u8, u8)> {
        let s = dtc.trim().to_uppercase();
        if s.len() < 5 {
            return None;
        }
        let type_bits: u8 = match s.chars().next()? {
            'P' => 0,
            'C' => 1,
            'B' => 2,
            'U' => 3,
            _ => return None,
        };
        let d1 = u8::from_str_radix(&s[1..2], 16).ok()?;
        let d2 = u8::from_str_radix(&s[2..3], 16).ok()?;
        let d3 = u8::from_str_radix(&s[3..4], 16).ok()?;
        let d4 = u8::from_str_radix(&s[4..5], 16).ok()?;
        // Encoding is reverse of decode_dtc_bytes in obd.rs
        let b1 = (type_bits << 6) | (d1 << 4) | d2;
        let b2 = (d3 << 4) | d4;
        Some((b1, b2))
    }

    // ── PTY creation ─────────────────────────────────────────────────────────

    pub fn create_pty() -> Result<(RawFd, String), String> {
        unsafe {
            let master = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
            if master < 0 {
                return Err(format!("posix_openpt: {}", std::io::Error::last_os_error()));
            }
            if libc::grantpt(master) < 0 {
                libc::close(master);
                return Err("grantpt failed".into());
            }
            if libc::unlockpt(master) < 0 {
                libc::close(master);
                return Err("unlockpt failed".into());
            }
            let name_ptr = libc::ptsname(master);
            if name_ptr.is_null() {
                libc::close(master);
                return Err("ptsname failed".into());
            }
            let slave_path = CStr::from_ptr(name_ptr).to_string_lossy().into_owned();

            // Set master side to raw mode so the terminal line discipline
            // doesn't mangle our protocol bytes.
            let mut tios: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(master, &mut tios) == 0 {
                libc::cfmakeraw(&mut tios);
                libc::tcsetattr(master, libc::TCSANOW, &tios);
            }

            Ok((master, slave_path))
        }
    }

    // ── Protocol thread ───────────────────────────────────────────────────────

    /// Spawns a background thread that reads commands from `master_fd` and writes
    /// ELM327-formatted responses.  Each response is terminated with `\r>`.
    pub fn spawn_protocol(master_fd: RawFd, state: Arc<Mutex<SimState>>) {
        thread::spawn(move || {
            let mut buf: Vec<u8> = Vec::with_capacity(64);
            let mut byte = [0u8; 1];

            loop {
                let n = unsafe { libc::read(master_fd, byte.as_mut_ptr() as *mut libc::c_void, 1) };
                if n <= 0 {
                    // EOF / error — port was closed
                    if let Ok(mut st) = state.lock() {
                        st.connected = false;
                    }
                    thread::sleep(Duration::from_millis(100));
                    buf.clear();
                    continue;
                }

                match byte[0] {
                    b'\r' | b'\n' => {
                        if buf.is_empty() {
                            // Empty line — just re-send prompt
                            let prompt = b">";
                            unsafe {
                                libc::write(
                                    master_fd,
                                    prompt.as_ptr() as *const libc::c_void,
                                    prompt.len(),
                                );
                            }
                            continue;
                        }

                        let cmd = String::from_utf8_lossy(&buf).into_owned();
                        buf.clear();

                        let response = {
                            let mut st = state.lock().unwrap();
                            st.respond(&cmd)
                        };

                        // Format: response\r\n>
                        let full = format!("{response}\r\n>");
                        unsafe {
                            libc::write(
                                master_fd,
                                full.as_ptr() as *const libc::c_void,
                                full.len(),
                            );
                        }
                    }
                    // Ignore stray linefeeds
                    b'\x00' => {}
                    b => buf.push(b),
                }
            }
        });
    }

    // ── WebSocket server ──────────────────────────────────────────────────────

    /// Spawns a background thread that accepts WebSocket connections on `ws_port`
    /// and handles OBD/AT commands identically to the PTY protocol thread.
    /// Each WS text message = one command; each response = one text message.
    pub fn spawn_websocket_server(ws_port: u16, state: Arc<Mutex<SimState>>) {
        thread::spawn(move || {
            let addr = format!("127.0.0.1:{ws_port}");
            let listener = match TcpListener::bind(&addr) {
                Ok(l) => l,
                Err(e) => {
                    eprintln!("WebSocket server failed to bind {addr}: {e}");
                    return;
                }
            };
            eprintln!("WebSocket OBD server listening on ws://{addr}");

            for stream in listener.incoming() {
                let stream = match stream {
                    Ok(s) => s,
                    Err(_) => continue,
                };
                let state = state.clone();
                thread::spawn(move || {
                    let mut ws = match accept(stream) {
                        Ok(w) => w,
                        Err(_) => return,
                    };
                    {
                        if let Ok(mut st) = state.lock() {
                            st.connected = true;
                        }
                    }
                    loop {
                        let msg = match ws.read() {
                            Ok(m) => m,
                            Err(_) => break,
                        };
                        let text = match msg {
                            tungstenite::Message::Text(t) => t,
                            tungstenite::Message::Close(_) => break,
                            _ => continue,
                        };
                        let response = {
                            let mut st = state.lock().unwrap();
                            st.respond(text.trim())
                        };
                        if ws
                            .send(tungstenite::Message::Text(response.into()))
                            .is_err()
                        {
                            break;
                        }
                    }
                    if let Ok(mut st) = state.lock() {
                        st.connected = false;
                    }
                });
            }
        });
    }

    // ── egui application ─────────────────────────────────────────────────────

    pub struct EmulatorApp {
        state: Arc<Mutex<SimState>>,
        slave_path: String,
        pty_error: Option<String>,
        ws_port: u16,
        new_dtc: String,
        copy_flash: u8,    // frames remaining for "Copied!" flash
        ws_copy_flash: u8, // frames remaining for WS URL "Copied!" flash
    }

    impl EmulatorApp {
        pub fn new(
            cc: &eframe::CreationContext<'_>,
            state: Arc<Mutex<SimState>>,
            slave_path: String,
            pty_error: Option<String>,
            ws_port: u16,
        ) -> Self {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            Self {
                state,
                slave_path,
                pty_error,
                ws_port,
                new_dtc: String::new(),
                copy_flash: 0,
                ws_copy_flash: 0,
            }
        }
    }

    impl eframe::App for EmulatorApp {
        fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
            // ── Physics tick ─────────────────────────────────────────────────
            let (key_accel, key_brake) = ctx.input(|i| {
                (
                    i.key_down(Key::ArrowUp) || i.key_down(Key::W),
                    i.key_down(Key::ArrowDown) || i.key_down(Key::S),
                )
            });

            {
                let mut st = self.state.lock().unwrap();
                // Keyboard input supplements button state (set each frame by UI below)
                st.accel |= key_accel;
                st.brake |= key_brake;
                st.physics_tick();
                // Reset for next frame; UI buttons will re-set if held
                st.accel = key_accel;
                st.brake = key_brake;
            }

            ctx.request_repaint_after(Duration::from_millis(33));
            if self.copy_flash > 0 {
                self.copy_flash -= 1;
            }
            if self.ws_copy_flash > 0 {
                self.ws_copy_flash -= 1;
            }

            // ── Top bar ──────────────────────────────────────────────────────
            egui::TopBottomPanel::top("top").show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("OBD-II Emulator");
                    ui.separator();

                    if let Some(ref err) = self.pty_error {
                        ui.colored_label(Color32::RED, format!("PTY error: {err}"));
                    } else {
                        ui.label("Port:");
                        ui.code(&self.slave_path);
                        let copy_label = if self.copy_flash > 0 {
                            "✓ Copied"
                        } else {
                            "Copy"
                        };
                        if ui.small_button(copy_label).clicked() {
                            ctx.copy_text(self.slave_path.clone());
                            self.copy_flash = 60;
                        }
                    }

                    ui.separator();

                    let ws_url = format!("ws://localhost:{}", self.ws_port);
                    ui.label("WS:");
                    ui.code(&ws_url);
                    let ws_copy_label = if self.ws_copy_flash > 0 {
                        "✓ Copied"
                    } else {
                        "Copy"
                    };
                    if ui.small_button(ws_copy_label).clicked() {
                        ctx.copy_text(ws_url);
                        self.ws_copy_flash = 60;
                    }

                    ui.separator();

                    let st = self.state.lock().unwrap();
                    let (col, label) = if st.connected {
                        (Color32::from_rgb(50, 200, 80), "● Connected")
                    } else {
                        (Color32::from_rgb(140, 140, 140), "○ Waiting")
                    };
                    ui.colored_label(col, label);
                    if st.connected {
                        ui.separator();
                        ui.small(format!("{} cmds | last: {}", st.cmd_count, st.last_cmd));
                    }
                });
            });

            // ── Main content ─────────────────────────────────────────────────
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.columns(2, |cols| {
                    // ── Left column: controls ─────────────────────────────────
                    let ui = &mut cols[0];

                    // Engine on/off
                    {
                        let mut st = self.state.lock().unwrap();
                        let (eng_label, eng_col) = if st.engine_on {
                            ("■ Stop Engine", Color32::from_rgb(200, 60, 60))
                        } else {
                            ("▶ Start Engine", Color32::from_rgb(50, 180, 80))
                        };
                        let btn = egui::Button::new(
                            RichText::new(eng_label).strong().color(Color32::WHITE),
                        )
                        .fill(eng_col)
                        .min_size(egui::vec2(160.0, 36.0));
                        if ui.add(btn).clicked() {
                            st.engine_on = !st.engine_on;
                            if st.engine_on {
                                st.engine_start_time = Some(Instant::now());
                                st.warmup = 0.0;
                                st.rpm = st.idle_rpm;
                            } else {
                                st.engine_start_time = None;
                                st.runtime_secs = 0;
                            }
                        }
                    }

                    ui.add_space(8.0);

                    // Accelerate / Brake (hold buttons)
                    ui.horizontal(|ui| {
                        let accel_btn = ui.add_sized(
                            [120.0, 56.0],
                            egui::Button::new(RichText::new("⬆  Accelerate").strong().size(14.0))
                                .fill(Color32::from_rgb(30, 100, 200)),
                        );
                        ui.add_space(8.0);
                        let brake_btn = ui.add_sized(
                            [120.0, 56.0],
                            egui::Button::new(RichText::new("⬇  Brake").strong().size(14.0))
                                .fill(Color32::from_rgb(180, 60, 30)),
                        );

                        let mut st = self.state.lock().unwrap();
                        st.accel |= accel_btn.is_pointer_button_down_on();
                        st.brake |= brake_btn.is_pointer_button_down_on();
                    });
                    ui.small("(hold, or use ↑ ↓ / W S keys)");

                    ui.add_space(12.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // Sliders
                    {
                        let mut st = self.state.lock().unwrap();

                        ui.label(RichText::new("Engine Settings").strong());
                        ui.add(Slider::new(&mut st.idle_rpm, 600.0..=1200.0).text("Idle RPM"));
                        ui.add(Slider::new(&mut st.max_rpm, 4000.0..=8000.0).text("Rev Limit"));
                        ui.add(
                            Slider::new(&mut st.max_speed, 80.0..=300.0)
                                .suffix(" km/h")
                                .text("Top Speed"),
                        );

                        ui.add_space(8.0);
                        ui.label(RichText::new("Electrical").strong());
                        ui.add(
                            Slider::new(&mut st.base_voltage, 11.0..=15.5)
                                .suffix(" V")
                                .text("Base Voltage"),
                        );

                        ui.add_space(8.0);
                        ui.label(RichText::new("Fuel & Environment").strong());
                        ui.add(
                            Slider::new(&mut st.fuel_level, 0.0..=100.0)
                                .suffix(" %")
                                .text("Fuel Level"),
                        );
                        ui.add(
                            Slider::new(&mut st.baro_pressure, 85.0..=110.0)
                                .suffix(" kPa")
                                .text("Baro Pressure"),
                        );
                        ui.add(
                            Slider::new(&mut st.ambient_temp, -20.0..=50.0)
                                .suffix(" °C")
                                .text("Ambient Temp"),
                        );

                        ui.add_space(8.0);
                        ui.label(RichText::new("Fuel Trims").strong());
                        ui.add(
                            Slider::new(&mut st.short_fuel_trim, -25.0..=25.0)
                                .suffix(" %")
                                .text("Short Trim B1"),
                        );
                        ui.add(
                            Slider::new(&mut st.long_fuel_trim, -25.0..=25.0)
                                .suffix(" %")
                                .text("Long Trim B1"),
                        );
                    }

                    ui.add_space(12.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // VIN editor
                    {
                        const VIN_PRESETS: &[(&str, &str)] = &[
                            ("Honda Accord 2003", "1HGCM82633A004352"),
                            ("Ford Mustang GT 2018", "1FA6P8CF5J5100001"),
                            ("Ford F-150 2014", "1FTFW1ET5EFC00001"),
                            ("Ford Focus 2011 (EU)", "WF0XXXGBBXBR00001"),
                            ("Opel Astra H 2007", "W0L0AHL3574000001"),
                            ("Opel Corsa D 2010", "W0L0XCE75A4000001"),
                            ("VW Golf VI 2010", "WVWZZZ1KZAM000001"),
                            ("VW Passat B7 2013", "WVWZZZ3CZDE000001"),
                            ("Audi A4 B8 2012", "WAUZZZ8K5CA000001"),
                            ("BMW 3 Series 2014", "WBA3A5C55EF000001"),
                            ("Mercedes C-Class 2008", "WDBRF52H08F000001"),
                            ("Toyota Corolla 2005", "JTDBR32E050000001"),
                            ("Toyota Camry 2015", "4T1BF1FK5FU000001"),
                            ("Nissan Altima 2012", "1N4AL3AP5CC000001"),
                            ("Renault Megane III 2011", "VF1BZ0J0H50000001"),
                            ("Peugeot 308 2014", "VF3LBHNZHES000001"),
                            ("Hyundai i30 2013", "KMHD35LE8DU000001"),
                            ("Subaru Impreza 2011", "JF1GR7E62BG000001"),
                            ("Chevrolet Silverado 2018", "1GCVKNEC6JZ000001"),
                            ("Dodge Charger 2015", "2C3CDXHG9FH000001"),
                        ];

                        let mut st = self.state.lock().unwrap();
                        ui.label(RichText::new("VIN").strong());
                        egui::ComboBox::from_id_salt("vin_preset")
                            .selected_text("Load preset…")
                            .show_ui(ui, |ui| {
                                for (label, vin) in VIN_PRESETS {
                                    if ui.selectable_label(st.vin == *vin, *label).clicked() {
                                        st.vin = vin.to_string();
                                    }
                                }
                            });
                        ui.text_edit_singleline(&mut st.vin);
                    }

                    // ── Right column: live values + DTCs ──────────────────────
                    let ui = &mut cols[1];
                    let st = self.state.lock().unwrap();

                    ui.label(RichText::new("Live Values").strong());
                    ui.add_space(4.0);

                    egui::Grid::new("live")
                        .num_columns(2)
                        .striped(true)
                        .spacing([12.0, 4.0])
                        .show(ui, |ui| {
                            let val = |v: f32, decimals: usize, unit: &str| -> RichText {
                                RichText::new(format!("{:.prec$} {unit}", v, prec = decimals))
                                    .color(Color32::from_rgb(120, 200, 255))
                                    .monospace()
                            };

                            ui.label("RPM");
                            ui.add(egui::Label::new(val(st.rpm, 0, "")));
                            ui.end_row();

                            ui.label("Gear");
                            ui.monospace(if st.gear == 0 {
                                "N".to_string()
                            } else {
                                st.gear.to_string()
                            });
                            ui.end_row();

                            ui.label("Speed");
                            ui.add(egui::Label::new(val(st.speed, 1, "km/h")));
                            ui.end_row();

                            ui.label("Throttle");
                            ui.add(egui::Label::new(val(st.throttle, 1, "%")));
                            ui.end_row();

                            ui.label("Engine Load");
                            ui.add(egui::Label::new(val(st.engine_load, 1, "%")));
                            ui.end_row();

                            ui.label("Coolant");
                            ui.add(egui::Label::new(val(st.coolant_temp, 1, "°C")));
                            ui.end_row();

                            ui.label("Oil Temp");
                            ui.add(egui::Label::new(val(st.oil_temp, 1, "°C")));
                            ui.end_row();

                            ui.label("Intake Temp");
                            ui.add(egui::Label::new(val(st.intake_temp, 1, "°C")));
                            ui.end_row();

                            ui.label("MAF");
                            ui.add(egui::Label::new(val(st.maf, 2, "g/s")));
                            ui.end_row();

                            ui.label("Intake Press");
                            ui.add(egui::Label::new(val(st.intake_pressure, 1, "kPa")));
                            ui.end_row();

                            ui.label("Timing Adv");
                            ui.add(egui::Label::new(val(st.timing_advance, 1, "°")));
                            ui.end_row();

                            ui.label("Voltage");
                            ui.add(egui::Label::new(val(st.voltage, 2, "V")));
                            ui.end_row();

                            ui.label("Fuel Level");
                            ui.add(egui::Label::new(val(st.fuel_level, 1, "%")));
                            ui.end_row();

                            ui.label("Runtime");
                            let mins = st.runtime_secs / 60;
                            let secs = st.runtime_secs % 60;
                            ui.monospace(format!("{mins:02}:{secs:02}"));
                            ui.end_row();
                        });

                    drop(st); // release lock before mutable borrow below

                    ui.add_space(12.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // ── DTC management ────────────────────────────────────────
                    ui.label(RichText::new("Fault Codes (DTCs)").strong());
                    ui.add_space(4.0);

                    // MIL toggle
                    {
                        let mut st = self.state.lock().unwrap();
                        ui.horizontal(|ui| {
                            ui.label("MIL:");
                            let (mil_col, mil_lbl) = if st.mil_on {
                                (Color32::from_rgb(240, 200, 0), "ON (Check Engine)")
                            } else {
                                (Color32::from_rgb(100, 100, 100), "OFF")
                            };
                            ui.colored_label(mil_col, mil_lbl);
                            if ui.small_button("Toggle").clicked() {
                                st.mil_on = !st.mil_on;
                            }
                            if ui.small_button("Clear All").clicked() {
                                st.stored_dtcs.clear();
                                st.pending_dtcs.clear();
                                st.mil_on = false;
                            }
                        });
                    }

                    ui.add_space(6.0);

                    // Stored DTCs list
                    {
                        let mut st = self.state.lock().unwrap();
                        ui.label("Stored:");
                        ui.horizontal_wrapped(|ui| {
                            let mut to_remove: Option<usize> = None;
                            for (i, dtc) in st.stored_dtcs.iter().enumerate() {
                                ui.group(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            RichText::new(dtc)
                                                .color(Color32::from_rgb(240, 150, 60))
                                                .monospace(),
                                        );
                                        if ui.small_button("×").clicked() {
                                            to_remove = Some(i);
                                        }
                                    });
                                });
                            }
                            if let Some(i) = to_remove {
                                st.stored_dtcs.remove(i);
                            }
                            if st.stored_dtcs.is_empty() {
                                ui.label(RichText::new("(none)").color(Color32::from_gray(140)));
                            }
                        });
                    }

                    ui.add_space(4.0);

                    // Pending DTCs list
                    {
                        let mut st = self.state.lock().unwrap();
                        ui.label("Pending:");
                        ui.horizontal_wrapped(|ui| {
                            let mut to_remove: Option<usize> = None;
                            for (i, dtc) in st.pending_dtcs.iter().enumerate() {
                                ui.group(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            RichText::new(dtc)
                                                .color(Color32::from_rgb(200, 120, 60))
                                                .monospace(),
                                        );
                                        if ui.small_button("×").clicked() {
                                            to_remove = Some(i);
                                        }
                                    });
                                });
                            }
                            if let Some(i) = to_remove {
                                st.pending_dtcs.remove(i);
                            }
                            if st.pending_dtcs.is_empty() {
                                ui.label(RichText::new("(none)").color(Color32::from_gray(140)));
                            }
                        });
                    }

                    ui.add_space(8.0);

                    // Add DTC
                    ui.horizontal(|ui| {
                        ui.label("Add DTC:");
                        let resp = ui.add(
                            egui::TextEdit::singleline(&mut self.new_dtc)
                                .hint_text("P0420")
                                .desired_width(70.0)
                                .char_limit(5),
                        );
                        let submit = resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));

                        if ui.button("+ Stored").clicked() || submit {
                            let code = self.new_dtc.trim().to_uppercase();
                            if is_valid_dtc(&code) {
                                let mut st = self.state.lock().unwrap();
                                if !st.stored_dtcs.contains(&code) {
                                    st.stored_dtcs.push(code.clone());
                                }
                                self.new_dtc.clear();
                            }
                        }
                        if ui.button("+ Pending").clicked() {
                            let code = self.new_dtc.trim().to_uppercase();
                            if is_valid_dtc(&code) {
                                let mut st = self.state.lock().unwrap();
                                if !st.pending_dtcs.contains(&code) {
                                    st.pending_dtcs.push(code.clone());
                                }
                                self.new_dtc.clear();
                            }
                        }
                    });

                    // Quick-add common codes
                    ui.add_space(4.0);
                    ui.horizontal_wrapped(|ui| {
                        for code in &[
                            "P0420", "P0171", "P0300", "P0128", "P0442", "P0507", "P0101", "P0301",
                            "P0172", "U0100",
                        ] {
                            if ui.small_button(*code).clicked() {
                                let mut st = self.state.lock().unwrap();
                                let code = code.to_string();
                                if !st.stored_dtcs.contains(&code) {
                                    st.stored_dtcs.push(code);
                                    st.mil_on = true;
                                }
                            }
                        }
                    });
                });
            });
        }
    }

    fn is_valid_dtc(s: &str) -> bool {
        if s.len() != 5 {
            return false;
        }
        matches!(s.chars().next(), Some('P' | 'C' | 'B' | 'U'))
            && s[1..].chars().all(|c| c.is_ascii_hexdigit())
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    use emulator::*;
    use std::sync::{Arc, Mutex};

    const WS_PORT: u16 = 35000;

    let state = Arc::new(Mutex::new(SimState::default()));

    // Create PTY
    let (slave_path, pty_error) = match create_pty() {
        Ok((master_fd, path)) => {
            println!("OBD Emulator PTY port: {path}");
            println!("Connect obd-dashboard (desktop) to: {path}");
            spawn_protocol(master_fd, state.clone());
            (path, None)
        }
        Err(e) => {
            eprintln!("Failed to create PTY: {e}");
            ("/dev/null (PTY failed)".into(), Some(e))
        }
    };

    // Start WebSocket server for web app connections
    spawn_websocket_server(WS_PORT, state.clone());
    println!("WebSocket OBD server: ws://localhost:{WS_PORT}");
    println!("Connect web app to emulator using localhost:{WS_PORT}");

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([900.0, 680.0])
            .with_min_inner_size([700.0, 500.0])
            .with_title("OBD-II Emulator"),
        ..Default::default()
    };

    eframe::run_native(
        "OBD-II Emulator",
        native_options,
        Box::new(move |cc| {
            Ok(Box::new(EmulatorApp::new(
                cc, state, slave_path, pty_error, WS_PORT,
            )))
        }),
    )
    .unwrap();
}
