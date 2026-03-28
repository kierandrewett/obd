use std::io::{Read, Write};
use std::time::{Duration, Instant};
use tracing::{debug, info, warn};

const COMMON_BAUDS: &[u32] = &[38400, 9600, 115200, 57600, 19200, 230400, 500000];
const PORT_PATTERNS: &[&str] = &[
    "/dev/ttyUSB",
    "/dev/ttyACM",
    "/dev/ttyS",
    "/dev/rfcomm",
    "/dev/tty.usbserial",
    "/dev/tty.OBD",
    "/dev/tty.OBDII",
    "COM",
];

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct ConnectionInfo {
    pub port: String,
    pub baud: u32,
    pub protocol: String,
    pub elm_version: String,
    pub voltage: Option<String>,
}

pub struct Elm327 {
    port: Box<dyn serialport::SerialPort>,
    pub info: ConnectionInfo,
}

#[derive(Debug)]
pub enum Elm327Error {
    NoPortFound,
    NoBaudFound(String),
    InitFailed(String),
    Timeout(String),
    Serial(String),
    ProtocolError(String),
}

impl std::fmt::Display for Elm327Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Elm327Error::NoPortFound => write!(f, "No OBD adapter found on any serial port"),
            Elm327Error::NoBaudFound(p) => write!(f, "No working baud rate found for {p}"),
            Elm327Error::InitFailed(s) => write!(f, "ELM327 init failed: {s}"),
            Elm327Error::Timeout(s) => write!(f, "Timeout: {s}"),
            Elm327Error::Serial(s) => write!(f, "Serial error: {s}"),
            Elm327Error::ProtocolError(s) => write!(f, "Protocol error: {s}"),
        }
    }
}

impl std::error::Error for Elm327Error {}

/// Scan system for available serial ports matching OBD adapter patterns
pub fn scan_ports() -> Vec<String> {
    let mut found = Vec::new();
    match serialport::available_ports() {
        Ok(ports) => {
            for port in ports {
                let name = port.port_name.clone();
                let dominated_by_pattern = PORT_PATTERNS.iter().any(|pat| name.starts_with(pat));
                if dominated_by_pattern {
                    info!(port = %name, port_type = ?port.port_type, "Found serial port");
                    found.push(name);
                }
            }
        }
        Err(e) => {
            warn!("Failed to enumerate serial ports: {e}");
        }
    }

    // Also add any from serialport enumeration that we didn't pattern-match
    if let Ok(ports) = serialport::available_ports() {
        for port in ports {
            if !found.contains(&port.port_name) {
                match &port.port_type {
                    serialport::SerialPortType::UsbPort(usb) => {
                        info!(port = %port.port_name, vid = usb.vid, pid = usb.pid, "Found USB serial port");
                        found.push(port.port_name);
                    }
                    _ => {}
                }
            }
        }
    }

    found.sort();
    found
}

/// Try to connect with auto port and baud detection
pub fn auto_connect(
    progress: Option<&dyn Fn(&str)>,
) -> Result<Elm327, Elm327Error> {
    let ports = scan_ports();
    if ports.is_empty() {
        return Err(Elm327Error::NoPortFound);
    }

    let report = |msg: &str| {
        info!("{msg}");
        if let Some(f) = progress {
            f(msg);
        }
    };

    for port_name in &ports {
        report(&format!("Trying port {port_name}..."));
        match try_port(port_name, &report) {
            Ok(elm) => return Ok(elm),
            Err(e) => {
                warn!(port = %port_name, error = %e, "Port failed");
            }
        }
    }

    Err(Elm327Error::NoPortFound)
}

/// Connect to a specific port with optional baud override
pub fn connect(
    port_name: &str,
    baud: Option<u32>,
    progress: Option<&dyn Fn(&str)>,
) -> Result<Elm327, Elm327Error> {
    let report = |msg: &str| {
        info!("{msg}");
        if let Some(f) = progress {
            f(msg);
        }
    };

    if let Some(baud_rate) = baud {
        report(&format!("Connecting to {port_name} at {baud_rate} baud..."));
        try_port_baud(port_name, baud_rate, &report)
    } else {
        report(&format!("Auto-detecting baud rate for {port_name}..."));
        try_port(port_name, &report)
    }
}

fn try_port(
    port_name: &str,
    report: &dyn Fn(&str),
) -> Result<Elm327, Elm327Error> {
    for &baud in COMMON_BAUDS {
        report(&format!("  Trying {baud} baud..."));
        match try_port_baud(port_name, baud, report) {
            Ok(elm) => return Ok(elm),
            Err(e) => {
                debug!(baud, error = %e, "Baud rate failed");
            }
        }
    }
    Err(Elm327Error::NoBaudFound(port_name.to_string()))
}

fn try_port_baud(
    port_name: &str,
    baud: u32,
    report: &dyn Fn(&str),
) -> Result<Elm327, Elm327Error> {
    let mut port = serialport::new(port_name, baud)
        .timeout(Duration::from_secs(3))
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .open()
        .map_err(|e| Elm327Error::Serial(format!("{port_name}: {e}")))?;

    // Flush
    let _ = port.clear(serialport::ClearBuffer::All);
    std::thread::sleep(Duration::from_millis(200));

    // Send ATZ (reset)
    write_cmd(&mut port, "ATZ")?;
    let response = read_response(&mut port, Duration::from_secs(3))?;
    debug!(response = ?response, "ATZ response");

    let has_elm = response.iter().any(|l| l.contains("ELM") || l.contains("elm"));
    if !has_elm {
        return Err(Elm327Error::InitFailed("No ELM response to ATZ".into()));
    }

    let elm_version = response.iter()
        .find(|l| l.contains("ELM"))
        .cloned()
        .unwrap_or_else(|| "ELM327 (unknown version)".to_string());

    report(&format!("  Found: {elm_version}"));
    info!(version = %elm_version, port = %port_name, baud, "ELM327 detected");

    // Echo off
    write_cmd(&mut port, "ATE0")?;
    let _ = read_response(&mut port, Duration::from_secs(2))?;

    // Linefeeds off
    write_cmd(&mut port, "ATL0")?;
    let _ = read_response(&mut port, Duration::from_secs(2))?;

    // Spaces off (cleaner parsing)
    write_cmd(&mut port, "ATS0")?;
    let _ = read_response(&mut port, Duration::from_secs(2))?;

    // Headers off
    write_cmd(&mut port, "ATH0")?;
    let _ = read_response(&mut port, Duration::from_secs(2))?;

    // Auto protocol
    write_cmd(&mut port, "ATSP0")?;
    let _ = read_response(&mut port, Duration::from_secs(2))?;

    // Trigger protocol detection with 0100
    report("  Detecting OBD protocol...");
    write_cmd(&mut port, "0100")?;
    let resp_0100 = read_response(&mut port, Duration::from_secs(10))?;
    debug!(response = ?resp_0100, "0100 response");

    let has_data = resp_0100.iter().any(|l| l.starts_with("41"));
    if !has_data {
        let has_error = resp_0100.iter().any(|l| {
            l.contains("UNABLE") || l.contains("NO DATA") || l.contains("ERROR") || l.contains("BUS INIT")
        });
        if has_error {
            return Err(Elm327Error::ProtocolError(
                "Vehicle not responding. Is ignition on?".into(),
            ));
        }
    }

    // Get detected protocol
    write_cmd(&mut port, "ATDPN")?;
    let proto_resp = read_response(&mut port, Duration::from_secs(2))?;
    let protocol = proto_resp.first()
        .map(|s| decode_protocol(s.trim()))
        .unwrap_or_else(|| "Unknown".to_string());

    report(&format!("  Protocol: {protocol}"));
    info!(protocol = %protocol, "OBD protocol detected");

    // Read voltage
    write_cmd(&mut port, "ATRV")?;
    let volt_resp = read_response(&mut port, Duration::from_secs(2))?;
    let voltage = volt_resp.first().cloned();
    if let Some(v) = &voltage {
        info!(voltage = %v, "Battery voltage");
    }

    let info = ConnectionInfo {
        port: port_name.to_string(),
        baud,
        protocol,
        elm_version,
        voltage,
    };

    Ok(Elm327 { port, info })
}

impl Elm327 {
    /// Send an OBD/AT command and return response lines
    pub fn send(&mut self, cmd: &str, timeout: Duration) -> Result<Vec<String>, Elm327Error> {
        debug!(cmd, "TX");
        write_cmd(&mut self.port, cmd)?;
        let lines = read_response(&mut self.port, timeout)?;
        debug!(cmd, response = ?lines, "RX");
        Ok(lines)
    }

    /// Send command, log the raw exchange
    pub fn send_logged(&mut self, cmd: &str, timeout: Duration) -> Result<Vec<String>, Elm327Error> {
        let start = Instant::now();
        let lines = self.send(cmd, timeout)?;
        let elapsed = start.elapsed();
        info!(
            cmd = cmd,
            response_lines = lines.len(),
            elapsed_ms = elapsed.as_millis(),
            raw_response = %lines.join(" | "),
            "OBD exchange"
        );
        Ok(lines)
    }

    /// Read battery voltage
    pub fn read_voltage(&mut self) -> Result<String, Elm327Error> {
        let lines = self.send("ATRV", Duration::from_secs(2))?;
        Ok(lines.into_iter().next().unwrap_or_else(|| "N/A".to_string()))
    }

    /// Check which PIDs are supported in a range
    pub fn query_supported_pids(&mut self, range_cmd: &str) -> Result<Vec<u8>, Elm327Error> {
        let lines = self.send_logged(range_cmd, Duration::from_secs(3))?;
        let base_pid = u8::from_str_radix(&range_cmd[2..4], 16).unwrap_or(0);

        for line in &lines {
            let clean = line.replace(' ', "").to_uppercase();
            let prefix = format!("41{}", &range_cmd[2..4].to_uppercase());
            if let Some(pos) = clean.find(&prefix) {
                let hex = &clean[pos + prefix.len()..];
                if hex.len() >= 8 {
                    if let Ok(bits) = u32::from_str_radix(&hex[..8], 16) {
                        let mut supported = Vec::new();
                        for i in 0..32 {
                            if bits & (1 << (31 - i)) != 0 {
                                supported.push(base_pid + i as u8 + 1);
                            }
                        }
                        return Ok(supported);
                    }
                }
            }
        }
        Ok(Vec::new())
    }
}

fn write_cmd(port: &mut Box<dyn serialport::SerialPort>, cmd: &str) -> Result<(), Elm327Error> {
    let data = format!("{cmd}\r");
    port.write_all(data.as_bytes())
        .map_err(|e| Elm327Error::Serial(e.to_string()))?;
    port.flush()
        .map_err(|e| Elm327Error::Serial(e.to_string()))?;
    Ok(())
}

fn read_response(
    port: &mut Box<dyn serialport::SerialPort>,
    timeout: Duration,
) -> Result<Vec<String>, Elm327Error> {
    let start = Instant::now();
    let mut buf = Vec::with_capacity(512);
    let mut tmp = [0u8; 256];

    loop {
        if start.elapsed() > timeout {
            // Return what we have if anything
            if !buf.is_empty() {
                break;
            }
            return Err(Elm327Error::Timeout(format!(
                "No response within {}ms",
                timeout.as_millis()
            )));
        }

        match port.read(&mut tmp) {
            Ok(n) => {
                buf.extend_from_slice(&tmp[..n]);
                // Check for prompt character '>'
                if buf.contains(&b'>') {
                    break;
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                // Keep waiting
                continue;
            }
            Err(e) => {
                if !buf.is_empty() {
                    break;
                }
                return Err(Elm327Error::Serial(e.to_string()));
            }
        }
    }

    let raw = String::from_utf8_lossy(&buf);
    let lines: Vec<String> = raw
        .split('\r')
        .map(|s| s.replace('\n', "").replace('>', "").trim().to_string())
        .filter(|s| !s.is_empty())
        .collect();

    Ok(lines)
}

fn decode_protocol(s: &str) -> String {
    // ATDPN returns something like "A6" or "6"
    let num = s.trim_start_matches('A').trim();
    match num {
        "0" => "Auto".to_string(),
        "1" => "SAE J1850 PWM (41.6 kbaud)".to_string(),
        "2" => "SAE J1850 VPW (10.4 kbaud)".to_string(),
        "3" => "ISO 9141-2 (5 baud init)".to_string(),
        "4" => "ISO 14230-4 KWP (5 baud init)".to_string(),
        "5" => "ISO 14230-4 KWP (fast init)".to_string(),
        "6" => "ISO 15765-4 CAN (11-bit, 500 kbaud)".to_string(),
        "7" => "ISO 15765-4 CAN (29-bit, 500 kbaud)".to_string(),
        "8" => "ISO 15765-4 CAN (11-bit, 250 kbaud)".to_string(),
        "9" => "ISO 15765-4 CAN (29-bit, 250 kbaud)".to_string(),
        "A" => "SAE J1939 CAN (29-bit, 250 kbaud)".to_string(),
        "B" => "USER1 CAN (11-bit, 125 kbaud)".to_string(),
        "C" => "USER2 CAN (11-bit, 50 kbaud)".to_string(),
        _ => format!("Protocol {s}"),
    }
}