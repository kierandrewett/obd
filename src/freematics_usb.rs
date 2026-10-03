use crate::obd;
use std::collections::HashSet;
use std::io::{self, Read};
use std::thread;
use std::time::{Duration, Instant};

const FRAME_PREFIX: &[u8] = b"@FT1,";
const MAX_LINE_BYTES: usize = 16 * 1024;
const USB_BAUD: u32 = 115_200;
const AUTO_CONNECT_STARTUP_TIMEOUT: Duration = Duration::from_secs(12);

#[derive(Debug, Clone, PartialEq)]
pub struct TelemetryField {
    pub pid: u16,
    pub values: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FreematicsFrame {
    pub boot_id: u64,
    pub capture_ms: u32,
    pub capture_utc_ms: Option<i64>,
    pub dropped_records: u32,
    pub supported_pids: Option<HashSet<u8>>,
    pub fields: Vec<TelemetryField>,
    pub corrupt_records: u64,
    pub reader_drops: u64,
}

impl FreematicsFrame {
    pub fn measurements(&self) -> Vec<FreematicsMeasurement> {
        let definitions = obd::mode01_pids();
        let mut output = Vec::new();
        for field in &self.fields {
            if field.pid & 0xF00 != 0x100 || field.values.is_empty() {
                continue;
            }
            let pid = (field.pid & 0xFF) as u8;
            let cmd = format!("01{pid:02X}");
            let Some(definition) = definitions.iter().find(|definition| definition.cmd == cmd)
            else {
                continue;
            };
            let age_pid = 0x400 | pid as u16;
            let age_ms = self
                .fields
                .iter()
                .find(|candidate| candidate.pid == age_pid)
                .and_then(|candidate| candidate.values.first())
                .map(|age| (*age).clamp(0.0, u32::MAX as f64) as u32);
            output.push(FreematicsMeasurement {
                cmd,
                name: definition.description.to_string(),
                unit: definition.unit.to_string(),
                value: field.values[0],
                age_ms,
                supported: self
                    .supported_pids
                    .as_ref()
                    .map(|supported| supported.contains(&pid)),
            });
        }
        output
    }

    pub fn model_b_supply_voltage(&self) -> Option<(f64, Option<u32>)> {
        let value = self
            .fields
            .iter()
            .find(|field| field.pid == 0x24)?
            .values
            .first()?;
        let age = self
            .fields
            .iter()
            .find(|field| field.pid == 0x94)
            .and_then(|field| field.values.first())
            .map(|age| (*age).clamp(0.0, u32::MAX as f64) as u32);
        Some((value / 100.0, age))
    }

    pub fn ecu_control_module_voltage(&self) -> Option<(f64, Option<u32>)> {
        let value = self
            .fields
            .iter()
            .find(|field| field.pid == 0x142)?
            .values
            .first()?;
        let age = self
            .fields
            .iter()
            .find(|field| field.pid == 0x442)
            .and_then(|field| field.values.first())
            .map(|age| (*age).clamp(0.0, u32::MAX as f64) as u32);
        Some((*value, age))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct FreematicsMeasurement {
    pub cmd: String,
    pub name: String,
    pub unit: String,
    pub value: f64,
    pub age_ms: Option<u32>,
    pub supported: Option<bool>,
}

#[derive(Default)]
pub struct FreematicsParser {
    partial: Vec<u8>,
    corrupt_records: u64,
}

impl FreematicsParser {
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<FreematicsFrame> {
        let mut frames = Vec::new();
        for byte in bytes {
            if *byte == b'\n' {
                let line = std::mem::take(&mut self.partial);
                if let Some(mut frame) = parse_line(&line) {
                    frame.corrupt_records = self.corrupt_records;
                    frames.push(frame);
                } else if line.starts_with(FRAME_PREFIX) {
                    self.corrupt_records = self.corrupt_records.saturating_add(1);
                }
                continue;
            }
            self.partial.push(*byte);
            if self.partial.len() > MAX_LINE_BYTES {
                self.partial.clear();
                self.corrupt_records = self.corrupt_records.saturating_add(1);
            }
        }
        frames
    }

    pub fn corrupt_records(&self) -> u64 {
        self.corrupt_records
    }
}

pub fn parse_line(line: &[u8]) -> Option<FreematicsFrame> {
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    if !line.starts_with(FRAME_PREFIX) {
        return None;
    }
    let line = std::str::from_utf8(line).ok()?;
    let (header, payload) = line.split_once('|')?;
    let mut metadata = header.strip_prefix("@FT1,")?.splitn(6, ',');
    let boot_id = metadata.next()?.parse().ok()?;
    let capture_ms = metadata.next()?.parse().ok()?;
    let utc_valid = match metadata.next()? {
        "0" => false,
        "1" => true,
        _ => return None,
    };
    let capture_utc_ms: i64 = metadata.next()?.parse().ok()?;
    let dropped_records = metadata.next()?.parse().ok()?;
    let supported_text = metadata.next()?;
    let supported_pids = if supported_text.is_empty() {
        None
    } else {
        Some(
            supported_text
                .split(',')
                .map(|pid| u8::from_str_radix(pid, 16).ok())
                .collect::<Option<HashSet<_>>>()?,
        )
    };
    let capture_utc_ms = if utc_valid && capture_utc_ms >= 1_704_067_200_000 {
        Some(capture_utc_ms)
    } else if !utc_valid && capture_utc_ms == 0 {
        None
    } else {
        return None;
    };

    let (device_id, body) = payload.split_once('#')?;
    if device_id.is_empty() {
        return None;
    }
    let (data, checksum) = body.rsplit_once('*')?;
    let checksum = u8::from_str_radix(checksum, 16).ok()?;
    let checksum_input = payload.split_once('*')?.0.as_bytes();
    if checksum_input
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte))
        != checksum
    {
        return None;
    }

    let fields = data
        .split(',')
        .map(|field| {
            let (pid, values) = field.split_once(':')?;
            let pid = u16::from_str_radix(pid, 16).ok()?;
            let values = values
                .split(';')
                .map(|value| {
                    let parsed: f64 = value.parse().ok()?;
                    parsed.is_finite().then_some(parsed)
                })
                .collect::<Option<Vec<_>>>()?;
            (!values.is_empty()).then_some(TelemetryField { pid, values })
        })
        .collect::<Option<Vec<_>>>()?;

    Some(FreematicsFrame {
        boot_id,
        capture_ms,
        capture_utc_ms,
        dropped_records,
        supported_pids,
        fields,
        corrupt_records: 0,
        reader_drops: 0,
    })
}

pub struct FreematicsUsb {
    port: Box<dyn serialport::SerialPort>,
    parser: FreematicsParser,
}

impl FreematicsUsb {
    pub fn connect(port_name: &str) -> Result<Self, String> {
        let port = serialport::new(port_name, USB_BAUD)
            .timeout(Duration::from_millis(75))
            .preserve_dtr_on_open()
            .open()
            .map_err(|error| format!("{port_name}: {error}"))?;
        Ok(Self {
            port,
            parser: FreematicsParser::default(),
        })
    }

    pub fn port_name(&self) -> String {
        self.port
            .name()
            .unwrap_or_else(|| "Freematics USB".to_string())
    }

    pub fn auto_connect() -> Result<(Self, Vec<FreematicsFrame>), String> {
        let ports = serialport::available_ports()
            .map_err(|error| format!("Cannot enumerate serial ports: {error}"))?;
        let candidates: Vec<_> = ports
            .into_iter()
            .filter(|port| matches!(port.port_type, serialport::SerialPortType::UsbPort(_)))
            .map(|port| port.port_name)
            .collect();
        if candidates.is_empty() {
            return Err("No USB serial ports found for Freematics auto-detect".into());
        }

        let mut failures = Vec::new();
        for candidate in candidates {
            let mut device = match Self::connect(&candidate) {
                Ok(device) => device,
                Err(error) => {
                    failures.push(error);
                    continue;
                }
            };
            // Listen only: do not send ELM or diagnostic commands. A valid
            // TeleLogger frame is the protocol identity proof.
            // Linux may pulse DTR while opening a USB serial device, which
            // resets some ESP32/CH340 combinations despite preserving DTR in
            // the builder. Allow the firmware to finish booting and emit its
            // first frame instead of treating that expected restart as a
            // failed protocol probe.
            let startup_deadline = Instant::now() + AUTO_CONNECT_STARTUP_TIMEOUT;
            while Instant::now() < startup_deadline {
                match device.read_frames() {
                    Ok(frames) if !frames.is_empty() => return Ok((device, frames)),
                    Ok(_) => thread::sleep(Duration::from_millis(10)),
                    Err(error) => {
                        failures.push(error);
                        break;
                    }
                }
            }
        }
        let detail = if failures.is_empty() {
            "USB serial ports produced no valid @FT1 telemetry frame".to_string()
        } else {
            format!(
                "No USB serial port produced valid @FT1 telemetry; {} open/read error(s)",
                failures.len()
            )
        };
        Err(detail)
    }

    pub fn read_frames(&mut self) -> Result<Vec<FreematicsFrame>, String> {
        let mut bytes = [0u8; 1024];
        match self.port.read(&mut bytes) {
            Ok(0) => Ok(Vec::new()),
            Ok(count) => Ok(self.parser.feed(&bytes[..count])),
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
            {
                Ok(Vec::new())
            }
            Err(error) => Err(format!("Freematics USB read failed: {error}")),
        }
    }

    pub fn corrupt_records(&self) -> u64 {
        self.parser.corrupt_records()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wire_record(capture_ms: u32, rpm: f64, age: u32, supported: &str) -> String {
        let payload = format!(
            "ABCDEF#0:{capture_ms},10C:{rpm},40C:{age},24:1375,A0:100;1280,A1:125,A2:0.1;0.2;0.3"
        );
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        format!("@FT1,42,{capture_ms},1,1790966400000,0,{supported}|{payload}*{checksum:02X}\n")
    }

    #[test]
    fn parses_partial_records_and_preserves_capture_metadata_and_waveforms() {
        let line = wire_record(1200, 720.0, 125, "0C,0D");
        let split = line.len() / 2;
        let mut parser = FreematicsParser::default();
        assert!(parser.feed(&line.as_bytes()[..split]).is_empty());
        let frame = parser.feed(&line.as_bytes()[split..]).remove(0);
        assert_eq!(frame.boot_id, 42);
        assert_eq!(frame.capture_ms, 1200);
        assert_eq!(frame.capture_utc_ms, Some(1_790_966_400_000));
        assert_eq!(frame.measurements()[0].cmd, "010C");
        assert_eq!(frame.measurements()[0].age_ms, Some(125));
        assert_eq!(frame.measurements()[0].supported, Some(true));
        assert_eq!(
            frame
                .fields
                .iter()
                .find(|field| field.pid == 0xA0)
                .unwrap()
                .values,
            [100.0, 1280.0]
        );
    }

    #[test]
    fn rejects_corruption_and_ignores_debug_and_partial_log_lines() {
        let mut line = wire_record(1200, 720.0, 125, "0C");
        line = line.replace("720", "999");
        let mut parser = FreematicsParser::default();
        assert!(parser.feed(b"[OBD] debug line\n").is_empty());
        assert!(parser.feed(line.as_bytes()).is_empty());
        assert_eq!(parser.corrupt_records(), 1);
    }

    #[test]
    fn records_without_valid_utc_keep_monotonic_capture_time_only() {
        let payload = "ABCDEF#0:55,10C:600,40C:20";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,9,55,0,0,3,|{payload}*{checksum:02X}\n");
        let frame = parse_line(line.trim_end().as_bytes()).unwrap();
        assert_eq!(frame.capture_utc_ms, None);
        assert_eq!(frame.capture_ms, 55);
        assert_eq!(frame.dropped_records, 3);
    }

    #[test]
    fn keeps_model_supply_and_ecu_control_module_voltage_distinct() {
        let payload = "ABCDEF#24:1375,142:13.82,94:12,442:15";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,9,55,0,0,0,|{payload}*{checksum:02X}");
        let frame = parse_line(line.as_bytes()).unwrap();
        assert_eq!(frame.model_b_supply_voltage(), Some((13.75, Some(12))));
        assert_eq!(frame.ecu_control_module_voltage(), Some((13.82, Some(15))));
    }

    #[test]
    fn simulated_serial_shudder_and_reconnect_scenario_is_repeatable() {
        let traffic = include_bytes!("../tests/fixtures/freematics_usb_scenario.txt");
        let mut parser = FreematicsParser::default();
        let mut frames = Vec::new();
        for fragment in traffic.chunks(11) {
            frames.extend(parser.feed(fragment));
        }

        assert_eq!(frames.len(), 4, "one corrupt frame must be rejected");
        assert_eq!(parser.corrupt_records(), 1);
        assert_eq!(
            frames
                .iter()
                .map(|frame| frame.capture_ms)
                .collect::<Vec<_>>(),
            [1000, 1250, 1500, 250]
        );
        assert_eq!(frames[0].capture_utc_ms, Some(1_790_966_401_000));
        assert_eq!(frames[0].measurements()[0].value, 820.0);
        assert_eq!(frames[1].measurements()[0].value, 540.0);
        assert_eq!(frames[2].measurements()[0].age_ms, Some(1500));
        assert_eq!(frames[2].model_b_supply_voltage(), Some((11.8, Some(20))));
        assert_eq!(
            frames[2]
                .fields
                .iter()
                .find(|field| field.pid == 0xA0)
                .unwrap()
                .values,
            [1350.0, 1330.0, 1180.0]
        );
        assert_eq!(
            frames[3].boot_id, 99,
            "changed boot ID marks a device restart"
        );
        assert_eq!(frames[3].dropped_records, 2);
    }
}
