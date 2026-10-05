use crate::obd;
use std::collections::{HashMap, HashSet};
use std::io::{self, Read};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

const FRAME_V1_PREFIX: &[u8] = b"@FT1,";
const FRAME_V2_PREFIX: &[u8] = b"@FT2,";
const FRAME_FAMILY_PREFIX: &[u8] = b"@FT";
const MAX_LINE_BYTES: usize = 16 * 1024;
const USB_BAUD: u32 = 460_800;
const LEGACY_USB_BAUD: u32 = 115_200;
const LEGACY_BAUD_FALLBACK_DELAY: Duration = Duration::from_secs(3);
const FREEMATICS_USB_VID: u16 = 0x10c4;
const FREEMATICS_USB_PID: u16 = 0xea60;
const DTC_SCAN_INTERVAL_MS: u32 = 120_000;
const DTC_CODE_SLOTS: usize = 15;
const MAX_IDENTITY_METADATA_BYTES: usize = 32;

fn legacy_baud_fallback_due(elapsed: Duration, valid_frame_seen: bool, attempted: bool) -> bool {
    !valid_frame_seen && !attempted && elapsed >= LEGACY_BAUD_FALLBACK_DELAY
}

fn is_supported_freematics_port(port: &serialport::SerialPortInfo) -> bool {
    matches!(
        &port.port_type,
        serialport::SerialPortType::UsbPort(info)
            if info.vid == FREEMATICS_USB_VID && info.pid == FREEMATICS_USB_PID
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreematicsDtcMode {
    Stored,
    Pending,
    Permanent,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreematicsDtcStatus {
    NoResponse,
    Response,
    Codes,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreematicsDtcAvailability {
    /// This frame has no status field for the mode, so it is not advertised.
    Unsupported,
    /// The firmware's initial no-response status has no scan age or count.
    NoScan,
    /// A scan age is present and is within the configured scan interval.
    Fresh,
    /// A scan age is present but exceeds the configured scan interval.
    Stale,
    /// A status field was present but did not contain a defined status value.
    UnknownStatus,
}

/// One mode-specific DTC scan decoded from a Freematics telemetry frame.
///
/// `count` and `code_slots` are optional because the firmware only emits them
/// after the first scan. Slots retain the ECU's raw 16-bit DTC values; unused
/// slots are represented by `Some(0)` when emitted by the firmware.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FreematicsDtcScan {
    pub mode: FreematicsDtcMode,
    pub availability: FreematicsDtcAvailability,
    pub status: Option<FreematicsDtcStatus>,
    pub count: Option<u8>,
    pub code_slots: [Option<u16>; DTC_CODE_SLOTS],
    pub age_ms: Option<u32>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TelemetryField {
    pub pid: u16,
    pub values: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FreematicsFrame {
    pub boot_id: u64,
    pub capture_ms: u32,
    /// Host monotonic instant when the complete telemetry record reached the
    /// serial parser. Preserved across the reader queue so dashboard age
    /// includes time spent waiting for UI processing.
    pub reader_received_at: Instant,
    pub capture_utc_ms: Option<i64>,
    /// Optional durable sample sequence supplied by newer FT2 firmware.
    /// Older FT1/FT2 records do not carry this metadata.
    pub capture_sequence: Option<u64>,
    pub dropped_records: u32,
    /// `None` means firmware has not reported a support result. An explicit
    /// `-` metadata marker means the scan completed and found no supported PIDs.
    pub supported_pids: Option<HashSet<u8>>,
    /// Exact ECU response data bytes for the passive USB-only structured PID
    /// extension. The cloud telemetry schema remains unchanged.
    pub raw_mode01: HashMap<u8, Vec<u8>>,
    pub vin: Option<String>,
    pub calibration_id: Option<String>,
    pub ecu_name: Option<String>,
    pub fields: Vec<TelemetryField>,
    pub corrupt_records: u64,
    /// Bounded hex-only sample from the first corrupt telemetry record since the
    /// previous valid frame. Never contains raw serial text in application logs.
    pub corrupt_sample_hex: Option<String>,
    pub reader_drops: u64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FreematicsAcquisitionHealth {
    pub obd_state: Option<u32>,
    pub cumulative_timeouts: Option<u32>,
    pub last_request_latency_ms: Option<u32>,
    pub consecutive_fast_failures: Option<u32>,
}

impl FreematicsFrame {
    pub fn acquisition_health(&self) -> FreematicsAcquisitionHealth {
        FreematicsAcquisitionHealth {
            obd_state: self.field_u32(0x089),
            cumulative_timeouts: self.field_u32(0x087),
            last_request_latency_ms: self.field_u32(0x088),
            consecutive_fast_failures: self.field_u32(0x08A),
        }
    }

    /// Return stored, pending, and permanent DTC scan records in that order.
    pub fn dtc_scans(&self) -> [FreematicsDtcScan; 3] {
        [
            self.dtc_scan(FreematicsDtcMode::Stored, 0x300, 0x310, 0x301, 0x360),
            self.dtc_scan(FreematicsDtcMode::Pending, 0x320, 0x330, 0x321, 0x361),
            self.dtc_scan(FreematicsDtcMode::Permanent, 0x340, 0x350, 0x341, 0x362),
        ]
    }

    fn dtc_scan(
        &self,
        mode: FreematicsDtcMode,
        count_pid: u16,
        status_pid: u16,
        base_pid: u16,
        age_pid: u16,
    ) -> FreematicsDtcScan {
        let status_value = self.field_u32(status_pid);
        let status = status_value.and_then(|value| match value {
            0 => Some(FreematicsDtcStatus::NoResponse),
            1 => Some(FreematicsDtcStatus::Response),
            2 => Some(FreematicsDtcStatus::Codes),
            _ => None,
        });
        let count = self
            .field_u32(count_pid)
            .and_then(|value| u8::try_from(value).ok());
        let age_ms = self.field_u32(age_pid);
        let mut code_slots = [None; DTC_CODE_SLOTS];
        for (index, slot) in code_slots.iter_mut().enumerate() {
            *slot = self
                .field_u32(base_pid + index as u16)
                .and_then(|value| u16::try_from(value).ok());
        }

        let availability = match (status_value, status, age_ms, count) {
            (None, _, _, _) => FreematicsDtcAvailability::Unsupported,
            (Some(_), None, _, _) => FreematicsDtcAvailability::UnknownStatus,
            (Some(0), Some(FreematicsDtcStatus::NoResponse), None, None) => {
                FreematicsDtcAvailability::NoScan
            }
            (_, _, Some(age), _) if age > DTC_SCAN_INTERVAL_MS => FreematicsDtcAvailability::Stale,
            _ => FreematicsDtcAvailability::Fresh,
        };

        FreematicsDtcScan {
            mode,
            availability,
            status,
            count,
            code_slots,
            age_ms,
        }
    }

    fn field_u32(&self, pid: u16) -> Option<u32> {
        let value = self
            .fields
            .iter()
            .find(|field| field.pid == pid)?
            .values
            .first()?;
        if *value < 0.0 || *value > u32::MAX as f64 || value.fract() != 0.0 {
            return None;
        }
        Some(*value as u32)
    }

    fn field_age_ms(&self, pid: u16) -> Option<u32> {
        self.field_u32(pid)
    }

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
            let age_ms = self.field_age_ms(age_pid);
            output.push(FreematicsMeasurement {
                cmd: cmd.clone(),
                name: definition.description.to_string(),
                unit: definition.unit.to_string(),
                value: field.values[0],
                display_value: self
                    .raw_mode01
                    .get(&pid)
                    .map(|bytes| obd::decode_pid(definition, bytes)),
                raw_bytes: self.raw_mode01.get(&pid).cloned(),
                age_ms,
                supported: self
                    .supported_pids
                    .as_ref()
                    .map(|supported| supported.contains(&pid)),
            });

            let Some(raw) = self.raw_mode01.get(&pid) else {
                continue;
            };
            if pid == 0x03 && raw.len() == 2 {
                output.push(FreematicsMeasurement {
                    cmd: format!("{cmd}-B2"),
                    name: "Fuel system status bank 2".to_string(),
                    unit: String::new(),
                    value: field.values[0],
                    display_value: Some(obd::decode_pid(definition, &raw[1..])),
                    raw_bytes: Some(vec![raw[1]]),
                    age_ms,
                    supported: Some(true),
                });
            } else if (0x14..=0x1B).contains(&pid) && raw.len() == 2 {
                output.push(FreematicsMeasurement {
                    cmd: format!("{cmd}-TRIM"),
                    name: format!("{} short-term fuel trim", definition.description),
                    unit: "%".to_string(),
                    value: (raw[1] as f64 - 128.0) * 100.0 / 128.0,
                    display_value: None,
                    raw_bytes: Some(vec![raw[1]]),
                    age_ms,
                    supported: Some(true),
                });
            } else if ((0x24..=0x2B).contains(&pid) || (0x34..=0x3B).contains(&pid))
                && raw.len() == 4
            {
                let lambda = (u16::from_be_bytes([raw[0], raw[1]]) as f64) * 2.0 / 32768.0;
                output.push(FreematicsMeasurement {
                    cmd: format!("{cmd}-LAMBDA"),
                    name: format!("{} lambda", definition.description),
                    unit: "λ".to_string(),
                    value: lambda,
                    display_value: None,
                    raw_bytes: Some(raw[..2].to_vec()),
                    age_ms,
                    supported: Some(true),
                });
            }
        }
        output
    }

    /// Return passive Mode 02 frame-0 values carried alongside live telemetry.
    /// Freematics encodes a Mode 01 PID `p` as field `0x200 + p`; `0x363`
    /// carries elapsed time since the logger first read this ECU snapshot, not
    /// the ECU's unavailable fault-time capture age.
    /// These values are already decoded by firmware, so this method never
    /// issues a diagnostic request.
    pub fn freeze_frame_measurements(&self) -> Vec<FreematicsMeasurement> {
        let definitions = obd::mode01_pids();
        let age_ms = self.freeze_frame_read_age_ms();
        self.fields
            .iter()
            .filter_map(|field| {
                let pid = field.pid.checked_sub(0x200)?;
                if pid > u8::MAX as u16 || field.values.is_empty() {
                    return None;
                }
                let pid = pid as u8;
                let cmd = format!("01{pid:02X}");
                let definition = definitions.iter().find(|item| item.cmd == cmd)?;
                Some(FreematicsMeasurement {
                    cmd: format!("02{pid:02X}"),
                    name: definition.description.to_string(),
                    unit: definition.unit.to_string(),
                    value: field.values[0],
                    display_value: None,
                    raw_bytes: None,
                    age_ms,
                    // The Mode 01 support bitmap says nothing about whether
                    // this ECU exposes the PID in Mode 02 frame 0.
                    supported: None,
                })
            })
            .collect()
    }

    pub fn freeze_frame_status(&self) -> Option<u32> {
        self.field_u32(0x364)
    }

    pub fn freeze_frame_read_age_ms(&self) -> Option<u32> {
        self.field_u32(0x363)
    }

    pub fn freeze_frame_trigger_dtc(&self) -> Option<u32> {
        self.field_u32(0x365)
    }

    pub fn model_b_supply_voltage(&self) -> Option<(f64, Option<u32>)> {
        let value = self
            .fields
            .iter()
            .find(|field| field.pid == 0x24)?
            .values
            .first()?;
        let age = self.field_age_ms(0x94);
        Some((value / 100.0, age))
    }

    pub fn ecu_control_module_voltage(&self) -> Option<(f64, Option<u32>)> {
        let value = self
            .fields
            .iter()
            .find(|field| field.pid == 0x142)?
            .values
            .first()?;
        let age = self.field_age_ms(0x442);
        Some((*value, age))
    }

    /// Decode high-rate Model B supply-voltage samples as device-monotonic
    /// `(capture_ms, volts)` points. Malformed or impossible entries are
    /// omitted without discarding the containing telemetry frame.
    pub fn voltage_waveform(&self) -> Vec<(u32, f64)> {
        self.fields
            .iter()
            .filter(|field| field.pid == 0x0A0 && field.values.len() == 2)
            .filter_map(|field| {
                let timestamp = exact_u32(field.values[0])?;
                let centivolts = exact_u32(field.values[1])?;
                (centivolts <= 65_535).then_some((timestamp, centivolts as f64 / 100.0))
            })
            .collect()
    }

    /// Decode the timestamped raw accelerometer vectors into magnitude
    /// samples (in g, matching the firmware sensor's native units).
    pub fn acceleration_waveform(&self) -> Vec<(u32, f64)> {
        let timestamps: Vec<_> = self
            .fields
            .iter()
            .filter(|field| field.pid == 0x0A1 && field.values.len() == 1)
            .filter_map(|field| exact_u32(field.values[0]))
            .collect();
        let acceleration: Vec<_> = self
            .fields
            .iter()
            .filter(|field| field.pid == 0x0A2 && field.values.len() == 3)
            .map(|field| &field.values)
            .collect();
        timestamps
            .into_iter()
            .zip(acceleration)
            .filter_map(|(timestamp, vector)| {
                let magnitude = vector.iter().map(|axis| axis * axis).sum::<f64>().sqrt();
                magnitude.is_finite().then_some((timestamp, magnitude))
            })
            .collect()
    }
}

fn exact_u32(value: f64) -> Option<u32> {
    (value >= 0.0 && value <= u32::MAX as f64 && value.fract() == 0.0).then_some(value as u32)
}

#[derive(Debug, Clone)]
pub struct FreematicsMeasurement {
    pub cmd: String,
    pub name: String,
    pub unit: String,
    pub value: f64,
    /// Structured SAE rendering for status PIDs; numeric `value` remains the
    /// firmware's existing normalized scalar for charts and compatibility.
    pub display_value: Option<obd::ObdValue>,
    pub raw_bytes: Option<Vec<u8>>,
    pub age_ms: Option<u32>,
    pub supported: Option<bool>,
}

#[derive(Default)]
pub struct FreematicsParser {
    partial: Vec<u8>,
    corrupt_records: u64,
    corrupt_sample_hex: Option<String>,
    discarding_oversize: bool,
}

impl FreematicsParser {
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<FreematicsFrame> {
        let mut frames = Vec::new();
        for byte in bytes {
            if *byte == b'\n' {
                if self.discarding_oversize {
                    self.partial.clear();
                    self.discarding_oversize = false;
                    continue;
                }
                let line = std::mem::take(&mut self.partial);
                if let Some(mut frame) = parse_line(&line) {
                    frame.corrupt_records = self.corrupt_records;
                    frame.corrupt_sample_hex = self.corrupt_sample_hex.take();
                    frames.push(frame);
                } else if is_telemetry_prefix(&line) {
                    self.corrupt_records = self.corrupt_records.saturating_add(1);
                    if self.corrupt_sample_hex.is_none() {
                        self.corrupt_sample_hex = Some(corrupt_line_sample(&line));
                    }
                }
                continue;
            }
            let mut byte_appended = false;
            if self.discarding_oversize {
                self.partial.push(*byte);
                byte_appended = true;
                if let Some(offset) = frame_prefix_offset(&self.partial, 0) {
                    self.partial.drain(..offset);
                    self.discarding_oversize = false;
                } else {
                    // Keep only enough trailing bytes to recognize a prefix
                    // split across serial reads without retaining an unbounded
                    // corrupt line.
                    const PREFIX_TAIL: usize = FRAME_V2_PREFIX.len() - 1;
                    if self.partial.len() > PREFIX_TAIL {
                        self.partial.drain(..self.partial.len() - PREFIX_TAIL);
                    }
                    continue;
                }
            }
            if !byte_appended {
                self.partial.push(*byte);
            }
            if is_telemetry_prefix(&self.partial) {
                if let Some(offset) = next_frame_offset(&self.partial) {
                    self.corrupt_records = self.corrupt_records.saturating_add(1);
                    if self.corrupt_sample_hex.is_none() {
                        self.corrupt_sample_hex = Some(corrupt_line_sample(&self.partial));
                    }
                    self.partial.drain(..offset);
                }
            }
            if self.partial.len() > MAX_LINE_BYTES {
                if is_telemetry_prefix(&self.partial) && self.corrupt_sample_hex.is_none() {
                    self.corrupt_sample_hex = Some(corrupt_line_sample(&self.partial));
                }
                if is_telemetry_prefix(&self.partial) {
                    self.corrupt_records = self.corrupt_records.saturating_add(1);
                }
                const PREFIX_TAIL: usize = FRAME_V2_PREFIX.len() - 1;
                self.partial.drain(..self.partial.len() - PREFIX_TAIL);
                self.discarding_oversize = true;
            }
        }
        frames
    }

    pub fn corrupt_records(&self) -> u64 {
        self.corrupt_records
    }
}

fn is_telemetry_prefix(bytes: &[u8]) -> bool {
    bytes.starts_with(FRAME_FAMILY_PREFIX)
}

fn next_frame_offset(bytes: &[u8]) -> Option<usize> {
    frame_prefix_offset(bytes, 1)
}

fn frame_prefix_offset(bytes: &[u8], start: usize) -> Option<usize> {
    (start..bytes.len()).find(|&offset| {
        let remaining = &bytes[offset..];
        remaining.starts_with(FRAME_V1_PREFIX) || remaining.starts_with(FRAME_V2_PREFIX)
    })
}

fn corrupt_line_sample(line: &[u8]) -> String {
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    let fields = std::str::from_utf8(line)
        .ok()
        .and_then(|text| text.split_once('|').map(|(_, payload)| payload))
        .and_then(|payload| payload.split_once('#').map(|(_, data)| data))
        .and_then(|data| {
            data.split_once('*')
                .map_or(Some(data), |(data, _)| Some(data))
        });

    if let Some(fields) = fields {
        for field in fields.split(',') {
            let valid = field.split_once(':').is_some_and(|(pid, values)| {
                u16::from_str_radix(pid, 16).is_ok()
                    && !values.is_empty()
                    && values
                        .split(';')
                        .all(|value| value.parse::<f64>().is_ok_and(|number| number.is_finite()))
            });
            if !valid {
                return format!(
                    "field_len={} field_hex={}",
                    field.len(),
                    bounded_hex(field.as_bytes())
                );
            }
        }
    }

    let edge = 12;
    let prefix = &line[..line.len().min(edge)];
    let suffix = &line[line.len().saturating_sub(edge)..];
    format!(
        "record_len={} edge_hex={}..{}",
        line.len(),
        bounded_hex(prefix),
        bounded_hex(suffix)
    )
}

fn bounded_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut output = String::with_capacity(bytes.len().min(32) * 2);
    for byte in bytes.iter().take(32) {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0F) as usize] as char);
    }
    output
}

pub fn parse_line(line: &[u8]) -> Option<FreematicsFrame> {
    let reader_received_at = Instant::now();
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    let version2 = line.starts_with(FRAME_V2_PREFIX);
    if !version2 && !line.starts_with(FRAME_V1_PREFIX) {
        return None;
    }
    let line = std::str::from_utf8(line).ok()?;
    let (header, payload) = line.split_once('|')?;
    let header = if version2 {
        header.strip_prefix("@FT2,")?
    } else {
        header.strip_prefix("@FT1,")?
    };
    let mut metadata = header.splitn(6, ',');
    let boot_id = metadata.next()?.parse().ok()?;
    let capture_ms = metadata.next()?.parse().ok()?;
    let utc_valid = match metadata.next()? {
        "0" => false,
        "1" => true,
        _ => return None,
    };
    let capture_utc_ms: i64 = metadata.next()?.parse().ok()?;
    let dropped_records = metadata.next()?.parse().ok()?;
    let metadata_text = metadata.next()?;
    let mut suffix = metadata_text.split(';');
    let supported_text = suffix.next()?;
    let mut vin = None;
    let mut calibration_id = None;
    let mut ecu_name = None;
    let mut raw_mode01 = None;
    let mut raw_mode01_seen = false;
    let mut raw_mode01_invalid = false;
    let mut capture_sequence = None;
    let mut capture_sequence_seen = false;
    for item in suffix {
        if let Some(value) = item.strip_prefix("vin=") {
            if vin.is_some()
                || value.len() != 17
                || !value.bytes().all(|b| b.is_ascii_alphanumeric())
            {
                return None;
            }
            vin = Some(value.to_string());
        } else if let Some(value) = item.strip_prefix("cal=") {
            if calibration_id.is_none() {
                calibration_id = decode_identity_metadata(value);
            }
        } else if let Some(value) = item.strip_prefix("ecu=") {
            if ecu_name.is_none() {
                ecu_name = decode_identity_metadata(value);
            }
        } else if let Some(value) = item.strip_prefix("raw=") {
            if raw_mode01_seen {
                // Raw data is optional metadata outside the serialized sample
                // checksum. A duplicate makes the extension ambiguous, but
                // must not discard otherwise valid timestamped measurements.
                raw_mode01_invalid = true;
                raw_mode01 = None;
            } else {
                raw_mode01_seen = true;
                match decode_raw_mode01(value) {
                    Some(decoded) => raw_mode01 = Some(decoded),
                    None => raw_mode01_invalid = true,
                }
            }
        } else if let Some(value) = item.strip_prefix("seq=") {
            if !version2 || capture_sequence_seen {
                return None;
            }
            capture_sequence_seen = true;
            capture_sequence = Some(value.parse::<u64>().ok()?);
        }
    }
    let supported_pids = if supported_text.is_empty() {
        None
    } else if supported_text == "-" {
        Some(HashSet::new())
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
    if version2 {
        if checksum.len() != 8 || !checksum.bytes().all(|byte| byte.is_ascii_hexdigit()) {
            return None;
        }
        let checksum = u32::from_str_radix(checksum, 16).ok()?;
        let checksum_end = line.rfind('*')?;
        if checksum_end <= line.find('|')?
            || crc32_iso_hdlc(&line.as_bytes()[..checksum_end]) != checksum
        {
            return None;
        }
    } else {
        let checksum = u8::from_str_radix(checksum, 16).ok()?;
        let checksum_input = payload.split_once('*')?.0.as_bytes();
        if checksum_input
            .iter()
            .fold(0u8, |sum, byte| sum.wrapping_add(*byte))
            != checksum
        {
            return None;
        }
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
        reader_received_at,
        capture_utc_ms,
        capture_sequence,
        dropped_records,
        supported_pids,
        raw_mode01: if raw_mode01_invalid {
            HashMap::new()
        } else {
            raw_mode01.unwrap_or_default()
        },
        vin,
        calibration_id,
        ecu_name,
        fields,
        corrupt_records: 0,
        corrupt_sample_hex: None,
        reader_drops: 0,
    })
}

fn crc32_iso_hdlc(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}

fn raw_mode01_widths() -> &'static HashMap<u8, usize> {
    static WIDTHS: OnceLock<HashMap<u8, usize>> = OnceLock::new();
    WIDTHS.get_or_init(|| {
        obd::mode01_pids()
            .into_iter()
            .filter_map(|definition| {
                let pid = definition.cmd.strip_prefix("01")?;
                let pid = u8::from_str_radix(pid, 16).ok()?;
                // PidDef.bytes includes the positive-service and PID bytes.
                let width = definition.bytes.checked_sub(2)? as usize;
                (width > 0 && width <= 4).then_some((pid, width))
            })
            .collect()
    })
}

fn decode_raw_mode01(text: &str) -> Option<HashMap<u8, Vec<u8>>> {
    if text.is_empty() {
        return None;
    }
    let mut output = HashMap::new();
    for entry in text.split(',') {
        let (pid, hex) = entry.split_once(':')?;
        if pid.len() != 2 || hex.len() % 2 != 0 {
            return None;
        }
        let pid = u8::from_str_radix(pid, 16).ok()?;
        let width = *raw_mode01_widths().get(&pid)?;
        if hex.len() != width * 2 || output.contains_key(&pid) {
            return None;
        }
        let bytes = hex
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                let pair = std::str::from_utf8(pair).ok()?;
                u8::from_str_radix(pair, 16).ok()
            })
            .collect::<Option<Vec<_>>>()?;
        output.insert(pid, bytes);
    }
    Some(output)
}

fn decode_identity_metadata(encoded: &str) -> Option<String> {
    let encoded = encoded.as_bytes();
    if encoded.is_empty()
        || encoded.len() > MAX_IDENTITY_METADATA_BYTES * 2
        || encoded.len() % 2 != 0
    {
        return None;
    }
    let bytes = (0..encoded.len())
        .step_by(2)
        .map(|index| {
            let pair = std::str::from_utf8(&encoded[index..index + 2]).ok()?;
            u8::from_str_radix(pair, 16).ok()
        })
        .collect::<Option<Vec<_>>>()?;
    if !bytes.iter().all(|byte| (0x20..=0x7e).contains(byte)) {
        return None;
    }
    let decoded = std::str::from_utf8(&bytes)
        .ok()?
        .trim_matches(char::is_whitespace);
    (!decoded.is_empty()).then(|| decoded.to_string())
}

pub struct FreematicsUsb {
    port: Box<dyn serialport::SerialPort>,
    parser: FreematicsParser,
    baud_rate: u32,
    opened_at: Instant,
    valid_frame_seen: bool,
    legacy_baud_attempted: bool,
}

impl FreematicsUsb {
    pub fn connect(port_name: &str) -> Result<Self, String> {
        let ports = serialport::available_ports()
            .map_err(|error| format!("Cannot enumerate serial ports: {error}"))?;
        if !ports
            .iter()
            .any(|port| port.port_name == port_name && is_supported_freematics_port(port))
        {
            return Err(format!(
                "{port_name} is not the supported Freematics Model B CP210x bridge (10c4:ea60); refusing to open an unidentified serial device"
            ));
        }

        let port = serialport::new(port_name, USB_BAUD)
            .timeout(Duration::from_millis(75))
            .preserve_dtr_on_open()
            .open()
            .map_err(|error| format!("{port_name}: {error}"))?;
        Ok(Self::from_port(port))
    }

    fn from_port(port: Box<dyn serialport::SerialPort>) -> Self {
        Self {
            port,
            parser: FreematicsParser::default(),
            baud_rate: USB_BAUD,
            opened_at: Instant::now(),
            valid_frame_seen: false,
            legacy_baud_attempted: false,
        }
    }

    pub fn port_name(&self) -> String {
        self.port
            .name()
            .unwrap_or_else(|| "Freematics USB".to_string())
    }

    pub fn baud_rate(&self) -> u32 {
        self.baud_rate
    }

    pub fn auto_connect() -> Result<(Self, Vec<FreematicsFrame>), String> {
        let ports = serialport::available_ports()
            .map_err(|error| format!("Cannot enumerate serial ports: {error}"))?;
        let candidates: Vec<_> = ports
            .into_iter()
            .filter(is_supported_freematics_port)
            .map(|port| port.port_name)
            .collect();
        if candidates.is_empty() {
            return Err(
                "No Freematics Model B CP210x USB bridge found (10c4:ea60); refusing to probe generic serial adapters".into(),
            );
        }

        let mut failures = Vec::new();
        for candidate in candidates {
            let device = match Self::connect(&candidate) {
                Ok(device) => device,
                Err(error) => {
                    failures.push(error);
                    continue;
                }
            };
            // Keep the single identified port open while telemetry is pending.
            // Reopening after a short protocol timeout can repeat Linux tty
            // modem-control transitions. The worker promotes this to Connected
            // only after receiving a valid checksummed FT1 frame.
            return Ok((device, Vec::new()));
        }
        let detail = if failures.is_empty() {
            "No supported Freematics USB serial port could be opened".to_string()
        } else {
            format!(
                "No supported Freematics USB serial port could be opened; {} open error(s)",
                failures.len()
            )
        };
        Err(detail)
    }

    pub fn read_frames(&mut self) -> Result<Vec<FreematicsFrame>, String> {
        let mut bytes = [0u8; 1024];
        let frames = match self.port.read(&mut bytes) {
            Ok(0) => Vec::new(),
            Ok(count) => self.parser.feed(&bytes[..count]),
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
            {
                Vec::new()
            }
            Err(error) => return Err(format!("Freematics USB read failed: {error}")),
        };
        if !frames.is_empty() {
            self.valid_frame_seen = true;
            return Ok(frames);
        }
        if legacy_baud_fallback_due(
            self.opened_at.elapsed(),
            self.valid_frame_seen,
            self.legacy_baud_attempted,
        ) {
            self.port
                .set_baud_rate(LEGACY_USB_BAUD)
                .map_err(|error| format!("Cannot try legacy Freematics USB baud: {error}"))?;
            self.baud_rate = LEGACY_USB_BAUD;
            self.parser = FreematicsParser::default();
            self.legacy_baud_attempted = true;
        }
        Ok(frames)
    }

    pub fn corrupt_records(&self) -> u64 {
        self.parser.corrupt_records()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prefers_high_speed_and_falls_back_once_only_before_telemetry() {
        assert_eq!(USB_BAUD, 460_800);
        assert_eq!(LEGACY_USB_BAUD, 115_200);
        assert!(!legacy_baud_fallback_due(
            LEGACY_BAUD_FALLBACK_DELAY - Duration::from_millis(1),
            false,
            false,
        ));
        assert!(legacy_baud_fallback_due(
            LEGACY_BAUD_FALLBACK_DELAY,
            false,
            false,
        ));
        assert!(!legacy_baud_fallback_due(
            LEGACY_BAUD_FALLBACK_DELAY * 2,
            true,
            false,
        ));
        assert!(!legacy_baud_fallback_due(
            LEGACY_BAUD_FALLBACK_DELAY * 2,
            false,
            true,
        ));
    }

    #[cfg(target_os = "linux")]
    fn pty_device() -> (FreematicsUsb, std::fs::File) {
        use std::os::fd::FromRawFd;

        let mut master = -1;
        let mut slave = -1;
        // SAFETY: openpty initializes both output descriptors or returns an
        // error. Ownership is transferred below to File and serialport.
        let result = unsafe {
            libc::openpty(
                &mut master,
                &mut slave,
                std::ptr::null_mut(),
                std::ptr::null(),
                std::ptr::null(),
            )
        };
        assert_eq!(
            result,
            0,
            "openpty failed: {}",
            std::io::Error::last_os_error()
        );
        let path =
            std::fs::read_link(format!("/proc/self/fd/{slave}")).expect("resolve PTY slave path");
        // SAFETY: close the original slave descriptor before serialport opens
        // its own descriptor for the same PTY.
        assert_eq!(unsafe { libc::close(slave) }, 0);
        let port = serialport::new(path.to_str().expect("PTY path is UTF-8"), USB_BAUD)
            .timeout(Duration::from_millis(20))
            .preserve_dtr_on_open()
            .open()
            .expect("open PTY slave through serialport");
        // SAFETY: master is a newly allocated descriptor from openpty and is
        // now owned by this File.
        let master = unsafe { std::fs::File::from_raw_fd(master) };
        (FreematicsUsb::from_port(port), master)
    }

    #[cfg(target_os = "linux")]
    fn read_pty_frame(device: &mut FreematicsUsb) -> FreematicsFrame {
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            let frames = device.read_frames().expect("read PTY telemetry");
            if let Some(frame) = frames.into_iter().next() {
                return frame;
            }
        }
        panic!("timed out waiting for a complete FT1/FT2 frame over PTY");
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn reads_fragmented_ft1_frames_at_preferred_baud_over_pty() {
        use std::io::Write;

        let (mut device, mut master) = pty_device();
        let wire = wire_record(1250, 790.0, 2, "0C");
        let split = wire.len() / 2;
        master.write_all(&wire.as_bytes()[..split]).unwrap();
        assert!(device.read_frames().unwrap().is_empty());
        master.write_all(&wire.as_bytes()[split..]).unwrap();

        let frame = read_pty_frame(&mut device);
        assert_eq!(frame.capture_ms, 1250);
        assert_eq!(frame.corrupt_records, 0);
        assert_eq!(device.baud_rate(), 460_800);
        assert_eq!(device.port.baud_rate().unwrap(), 460_800);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn switches_same_pty_to_legacy_baud_and_discards_wrong_rate_partial() {
        use std::io::Write;

        let (mut device, mut master) = pty_device();
        master.write_all(b"@FT1,partial-at-wrong-baud").unwrap();
        assert!(device.read_frames().unwrap().is_empty());
        device.opened_at = Instant::now() - LEGACY_BAUD_FALLBACK_DELAY;
        assert!(device.read_frames().unwrap().is_empty());
        assert_eq!(device.baud_rate(), LEGACY_USB_BAUD);
        assert_eq!(device.port.baud_rate().unwrap(), LEGACY_USB_BAUD);
        assert!(device.legacy_baud_attempted);

        master
            .write_all(wire_record(1500, 775.0, 3, "0C").as_bytes())
            .unwrap();
        let frame = read_pty_frame(&mut device);
        assert_eq!(frame.capture_ms, 1500);
        assert_eq!(frame.corrupt_records, 0);
        assert_eq!(device.baud_rate(), LEGACY_USB_BAUD);
    }

    #[test]
    fn auto_detect_accepts_only_the_model_b_cp210x_bridge() {
        let model_b = serialport::SerialPortInfo {
            port_name: "/dev/ttyUSB1".into(),
            port_type: serialport::SerialPortType::UsbPort(serialport::UsbPortInfo {
                vid: 0x10c4,
                pid: 0xea60,
                serial_number: None,
                manufacturer: None,
                product: None,
            }),
        };
        let generic_ch340 = serialport::SerialPortInfo {
            port_name: "/dev/ttyUSB0".into(),
            port_type: serialport::SerialPortType::UsbPort(serialport::UsbPortInfo {
                vid: 0x1a86,
                pid: 0x7523,
                serial_number: None,
                manufacturer: None,
                product: None,
            }),
        };
        let other_usb_uart = serialport::SerialPortInfo {
            port_name: "/dev/ttyUSB2".into(),
            port_type: serialport::SerialPortType::UsbPort(serialport::UsbPortInfo {
                vid: 0x0403,
                pid: 0x6001,
                serial_number: None,
                manufacturer: None,
                product: None,
            }),
        };

        assert!(is_supported_freematics_port(&model_b));
        assert!(!is_supported_freematics_port(&generic_ch340));
        assert!(!is_supported_freematics_port(&other_usb_uart));
    }

    fn wire_record(capture_ms: u32, rpm: f64, age: u32, supported: &str) -> String {
        let payload = format!(
            "ABCDEF#0:{capture_ms},10C:{rpm},40C:{age},24:1375,A0:100;1280,A1:125,A2:0.1;0.2;0.3"
        );
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        format!("@FT1,42,{capture_ms},1,1790966400000,0,{supported}|{payload}*{checksum:02X}\n")
    }

    fn wire_record_v2(header: &str, payload: &str) -> String {
        let body = format!("@FT2,{header}|{payload}");
        format!("{body}*{:08X}", crc32_iso_hdlc(body.as_bytes()))
    }

    #[test]
    fn crc32_matches_iso_hdlc_check_vector() {
        assert_eq!(crc32_iso_hdlc(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn ft2_crc_covers_the_envelope_and_serialized_sample() {
        let payload = "ABCDEF#0:1200,10C:718.5,40C:125,24:1375";
        let record = wire_record_v2(
            "42,1200,1,1790966400000,7,0C,0D;vin=W0L0SDL68D4050841;raw=0C:02CE",
            payload,
        );
        let frame = parse_line(record.as_bytes()).expect("valid FT2 record");
        assert_eq!(frame.boot_id, 42);
        assert_eq!(frame.capture_ms, 1200);
        assert_eq!(frame.dropped_records, 7);
        assert_eq!(frame.vin.as_deref(), Some("W0L0SDL68D4050841"));

        // Every metadata region is covered, not just the serialized fields.
        for (marker, relative_offset, replacement) in [
            ("42,", 0, b'5'),
            ("1200,", 0, b'2'),
            ("1,179", 2, b'2'),
            ("1790966400000", 0, b'2'),
            ("7,", 0, b'8'),
            ("0C,0D", 0, b'1'),
            ("vin=W", 4, b'X'),
            ("02CE", 0, b'1'),
            ("718.5", 0, b'8'),
        ] {
            let mut corrupted = record.clone();
            let offset = corrupted.find(marker).expect("test mutation marker") + relative_offset;
            unsafe {
                corrupted.as_bytes_mut()[offset] = replacement;
            }
            assert!(
                parse_line(corrupted.as_bytes()).is_none(),
                "accepted mutation at {marker}"
            );
        }
        let mut malformed_crc = record.clone();
        let crc_offset = malformed_crc.rfind('*').unwrap() + 1;
        malformed_crc.replace_range(crc_offset..crc_offset + 1, "G");
        assert!(parse_line(malformed_crc.as_bytes()).is_none());
        assert!(parse_line(&record.as_bytes()[..crc_offset - 1]).is_none());
        let mut extra_crc = record.clone();
        extra_crc.push('0');
        assert!(parse_line(extra_crc.as_bytes()).is_none());

        let mut crlf_parser = FreematicsParser::default();
        let frames = crlf_parser.feed(format!("{record}\r\n").as_bytes());
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn parses_optional_ft2_capture_sequence_and_preserves_legacy_records() {
        let payload = "ABCDEF#0:1200,10C:718.5,40C:125,24:1375";
        let with_sequence = wire_record_v2(
            "42,1200,1,1790966400000,7,0C,0D;seq=18446744073709551615",
            payload,
        );
        assert_eq!(
            parse_line(with_sequence.as_bytes())
                .expect("FT2 with sequence")
                .capture_sequence,
            Some(u64::MAX)
        );

        let older_ft2 = wire_record_v2("42,1200,1,1790966400000,7,0C,0D", payload);
        assert_eq!(
            parse_line(older_ft2.as_bytes())
                .expect("legacy FT2 without sequence")
                .capture_sequence,
            None
        );
        let legacy_ft1 = wire_record(1200, 718.5, 125, "0C,0D");
        assert_eq!(
            parse_line(legacy_ft1.trim_end().as_bytes())
                .expect("legacy FT1")
                .capture_sequence,
            None
        );
    }

    #[test]
    fn rejects_malformed_or_duplicate_ft2_capture_sequence() {
        let payload = "ABCDEF#0:1200,10C:718.5,40C:125,24:1375";
        for metadata in [
            "42,1200,1,1790966400000,7,0C;seq=",
            "42,1200,1,1790966400000,7,0C;seq=-1",
            "42,1200,1,1790966400000,7,0C;seq=18446744073709551616",
            "42,1200,1,1790966400000,7,0C;seq=1;seq=2",
        ] {
            assert!(
                parse_line(wire_record_v2(metadata, payload).as_bytes()).is_none(),
                "accepted invalid sequence metadata: {metadata}"
            );
        }
    }

    #[test]
    fn parser_counts_a_corrupt_ft2_record_and_recovers_at_next_line() {
        let mut corrupt = wire_record_v2("42,1200,0,0,0,|", "ABCDEF#0:1200,10C:700");
        let offset = corrupt.find("42,").unwrap();
        unsafe {
            corrupt.as_bytes_mut()[offset] = b'9';
        }
        let valid = wire_record_v2("42,1450,0,0,0,|", "ABCDEF#0:1450,10C:715");
        let mut parser = FreematicsParser::default();
        let mut received = Vec::new();
        for byte in format!("{corrupt}{valid}\n").as_bytes() {
            received.extend(parser.feed(&[*byte]));
        }
        assert_eq!(received.len(), 1);
        assert_eq!(received[0].capture_ms, 1450);
        assert_eq!(received[0].corrupt_records, 1);
    }

    #[test]
    fn parser_resynchronizes_after_oversized_frame_without_newline() {
        let valid = wire_record_v2("42,1450,0,0,0,|", "ABCDEF#0:1450,10C:715");
        let input = format!("@FT2,damaged{}{}\n", "x".repeat(MAX_LINE_BYTES + 8), valid);
        let mut parser = FreematicsParser::default();
        let mut frames = Vec::new();
        for chunk in input.as_bytes().chunks(17) {
            frames.extend(parser.feed(chunk));
        }
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].capture_ms, 1450);
        assert_eq!(frames[0].corrupt_records, 1);
    }

    #[test]
    fn unsupported_future_telemetry_version_is_counted_as_corrupt() {
        let valid = wire_record_v2("42,1450,0,0,0,|", "ABCDEF#0:1450,10C:715");
        let mut parser = FreematicsParser::default();
        let frames = parser.feed(format!("@FT3,unsupported\n{valid}\n").as_bytes());
        assert_eq!(frames.len(), 1);
        assert_eq!(parser.corrupt_records(), 1);
        assert_eq!(frames[0].corrupt_records, 1);
    }

    fn dtc_wire_frame(fields: &str) -> FreematicsFrame {
        let payload = format!("ABCDEF#0:100,{fields}");
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,42,100,0,0,0,|{payload}*{checksum:02X}");
        parse_line(line.as_bytes()).unwrap()
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
        assert_eq!(frame.vin, None);
        assert_eq!(frame.calibration_id, None);
        assert_eq!(frame.ecu_name, None);
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
        assert_eq!(frame.voltage_waveform(), [(100, 12.8)]);
        let acceleration = frame.acceleration_waveform();
        assert_eq!(acceleration.len(), 1);
        assert_eq!(acceleration[0].0, 125);
        assert!((acceleration[0].1 - 0.374_165_738_677_394_17).abs() < 1e-12);
    }

    #[test]
    fn exposes_acquisition_health_without_mixing_it_into_ecu_pids() {
        let payload = "ABCDEF#0:100,87:3,88:42,89:2,8A:1,10C:800,40C:10";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,42,100,0,0,0,0C|{payload}*{checksum:02X}");
        let frame = parse_line(line.as_bytes()).unwrap();

        assert_eq!(
            frame.acquisition_health(),
            FreematicsAcquisitionHealth {
                obd_state: Some(2),
                cumulative_timeouts: Some(3),
                last_request_latency_ms: Some(42),
                consecutive_fast_failures: Some(1),
            }
        );
        assert_eq!(frame.measurements().len(), 1);
    }

    #[test]
    fn exposes_firmware_odometer_pid_with_capture_age_and_support_status() {
        let payload = "ABCDEF#0:100,1A6:123456.7,4A6:500";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,42,100,1,1790966400000,0,A6|{payload}*{checksum:02X}");
        let frame = parse_line(line.as_bytes()).unwrap();
        let measurements = frame.measurements();

        assert_eq!(measurements.len(), 1);
        assert_eq!(measurements[0].cmd, "01A6");
        assert_eq!(measurements[0].name, "Vehicle odometer");
        assert_eq!(measurements[0].unit, "km");
        assert_eq!(measurements[0].value, 123456.7);
        assert_eq!(measurements[0].age_ms, Some(500));
        assert_eq!(measurements[0].supported, Some(true));
    }

    #[test]
    fn decodes_passive_mode02_frame_zero_values_and_snapshot_age() {
        let payload = "ABCDEF#0:100,20C:812.5,20D:0,363:2400,10C:790,260:1";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,42,100,1,1790966400000,0,0C,0D|{payload}*{checksum:02X}");
        let frame = parse_line(line.as_bytes()).unwrap();
        let measurements = frame.freeze_frame_measurements();

        assert_eq!(measurements.len(), 2, "only known Mode 01 PIDs are mapped");
        assert_eq!(measurements[0].cmd, "020C");
        assert_eq!(measurements[0].name, "Engine RPM");
        assert_eq!(measurements[0].value, 812.5);
        assert_eq!(measurements[0].unit, "RPM");
        assert_eq!(measurements[0].age_ms, Some(2400));
        assert_eq!(measurements[0].supported, None);
        assert_eq!(measurements[1].cmd, "020D");
        assert_eq!(measurements[1].value, 0.0);
        assert_eq!(measurements[1].age_ms, Some(2400));
        assert_eq!(measurements[1].supported, None);
    }

    #[test]
    fn passive_freeze_frame_age_is_optional_and_never_falls_back_to_live_values() {
        let payload = "ABCDEF#0:100,10C:790,363:1";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,42,100,0,0,0,0C|{payload}*{checksum:02X}");
        let frame = parse_line(line.as_bytes()).unwrap();

        assert!(frame.freeze_frame_measurements().is_empty());
    }

    #[test]
    fn maps_all_firmware_extended_mode01_pids_into_live_measurements() {
        let pids = [
            0x1A, 0x1B, 0x1E, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x35, 0x36, 0x37, 0x38, 0x39,
            0x3A, 0x3B, 0x4B, 0x53, 0x54, 0x59, 0x5A, 0x61, 0x62, 0x63, 0xA6,
        ];
        let fields = pids
            .iter()
            .flat_map(|pid| [format!("1{pid:02X}:1"), format!("4{pid:02X}:25")])
            .collect::<Vec<_>>()
            .join(",");
        let payload = format!("ABCDEF#0:100,{fields}");
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let supported = pids
            .iter()
            .map(|pid| format!("{pid:02X}"))
            .collect::<Vec<_>>()
            .join(",");
        let line = format!("@FT1,42,100,1,1790966400000,0,{supported}|{payload}*{checksum:02X}");

        let frame = parse_line(line.as_bytes()).unwrap();
        let measurements = frame.measurements();
        let commands = measurements
            .iter()
            .map(|measurement| measurement.cmd.as_str())
            .collect::<std::collections::HashSet<_>>();

        assert_eq!(measurements.len(), pids.len());
        for pid in pids {
            let command = format!("01{pid:02X}");
            assert!(commands.contains(command.as_str()), "missing {command}");
            let measurement = measurements
                .iter()
                .find(|measurement| measurement.cmd == command)
                .unwrap();
            assert_eq!(measurement.age_ms, Some(25), "age for {command}");
            assert_eq!(measurement.supported, Some(true), "support for {command}");
        }
    }

    #[test]
    fn parses_valid_optional_vin_suffix_without_changing_supported_pids() {
        let line = wire_record(1200, 720.0, 125, "0C,0D;vin=1HGCM82633A004352");
        let frame = parse_line(line.trim_end().as_bytes()).unwrap();

        assert_eq!(frame.vin.as_deref(), Some("1HGCM82633A004352"));
        assert_eq!(frame.supported_pids.unwrap(), HashSet::from([0x0C, 0x0D]));
    }

    #[test]
    fn distinguishes_unknown_pid_support_from_a_completed_empty_scan() {
        let unknown = parse_line(wire_record(1200, 720.0, 125, "").trim_end().as_bytes()).unwrap();
        let completed_empty =
            parse_line(wire_record(1200, 720.0, 125, "-").trim_end().as_bytes()).unwrap();

        assert_eq!(unknown.supported_pids, None);
        assert_eq!(completed_empty.supported_pids, Some(HashSet::new()));
        assert!(
            completed_empty
                .measurements()
                .iter()
                .all(|measurement| measurement.supported == Some(false))
        );
    }

    #[test]
    fn parses_hex_encoded_identity_metadata_after_supported_pids_and_vin() {
        let line = wire_record(
            1200,
            720.0,
            125,
            "0C,0D;vin=1HGCM82633A004352;cal=2043414C2D31323320;ecu=20454E47494E452020",
        );
        let frame = parse_line(line.trim_end().as_bytes()).unwrap();

        assert_eq!(frame.vin.as_deref(), Some("1HGCM82633A004352"));
        assert_eq!(frame.calibration_id.as_deref(), Some("CAL-123"));
        assert_eq!(frame.ecu_name.as_deref(), Some("ENGINE"));
        assert_eq!(frame.supported_pids.unwrap(), HashSet::from([0x0C, 0x0D]));
        assert_eq!(
            frame
                .fields
                .iter()
                .find(|field| field.pid == 0x10C)
                .unwrap()
                .values,
            [720.0]
        );
    }

    #[test]
    fn parses_structured_raw_mode01_bytes_and_decodes_compound_components() {
        let payload = "CAR#101:133,103:2,114:0.5,124:2,134:0";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!(
            "@FT1,42,100,0,0,0,01,03,04,0C,14,24,34,42,A6;raw=01:85080100,03:0201,04:80,0C:0320,14:6490,24:80004000,34:80008000,42:3854,A6:00000010|{payload}*{checksum:02X}"
        );
        let frame = parse_line(line.as_bytes()).unwrap();
        assert_eq!(frame.raw_mode01[&0x01], [0x85, 0x08, 0x01, 0x00]);
        assert_eq!(frame.raw_mode01[&0x04], [0x80]);
        assert_eq!(frame.raw_mode01[&0x0C], [0x03, 0x20]);
        assert_eq!(frame.raw_mode01[&0x14], [0x64, 0x90]);
        assert_eq!(frame.raw_mode01[&0x42], [0x38, 0x54]);
        assert_eq!(frame.raw_mode01[&0xA6], [0, 0, 0, 0x10]);

        let measurements = frame.measurements();
        let status = measurements.iter().find(|item| item.cmd == "0101").unwrap();
        assert!(matches!(
            status.display_value,
            Some(obd::ObdValue::StatusResult(ref value)) if value.mil_on && value.dtc_count == 5
        ));
        let trim = measurements
            .iter()
            .find(|item| item.cmd == "0114-TRIM")
            .unwrap();
        assert!((trim.value - 12.5).abs() < 0.001);
        let lambda_voltage = measurements
            .iter()
            .find(|item| item.cmd == "0124-LAMBDA")
            .unwrap();
        assert!((lambda_voltage.value - 2.0).abs() < 0.001);
        let lambda_current = measurements
            .iter()
            .find(|item| item.cmd == "0134-LAMBDA")
            .unwrap();
        assert!((lambda_current.value - 2.0).abs() < 0.001);
    }

    #[test]
    fn raw_mode01_widths_follow_the_dashboard_pid_catalogue() {
        let widths = raw_mode01_widths();
        assert_eq!(widths.get(&0x04), Some(&1));
        assert_eq!(widths.get(&0x0C), Some(&2));
        assert_eq!(widths.get(&0x14), Some(&2));
        assert_eq!(widths.get(&0x42), Some(&2));
        assert_eq!(widths.get(&0xA6), Some(&4));
        assert!(!widths.contains_key(&0xFF));
    }

    #[test]
    fn ignores_invalid_optional_raw_mode01_without_dropping_sample_data() {
        for suffix in [
            "raw=14:6400,14:6480",
            "raw=14:64",
            "raw=24:800040",
            "raw=55:01020304",
        ] {
            let line = wire_record(1200, 800.0, 0, &format!("0C,0D;{suffix}"));
            let frame = parse_line(line.trim_end().as_bytes()).expect("base sample should survive");
            assert!(
                frame.raw_mode01.is_empty(),
                "accepted raw metadata {suffix}"
            );
            let rpm = frame
                .measurements()
                .into_iter()
                .find(|measurement| measurement.cmd == "010C")
                .unwrap();
            assert_eq!(rpm.value, 800.0);
            assert_eq!(rpm.age_ms, Some(0));
            assert_eq!(rpm.supported, Some(true));
        }
    }

    #[test]
    fn raw_bytes_only_decode_the_mode01_field_with_the_same_pid() {
        let payload = "CAR#101:133,10C:800";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,42,100,0,0,0,01,0C;raw=01:85080100|{payload}*{checksum:02X}");
        let frame = parse_line(line.as_bytes()).unwrap();
        let rpm = frame
            .measurements()
            .into_iter()
            .find(|measurement| measurement.cmd == "010C")
            .unwrap();
        assert_eq!(rpm.value, 800.0);
        assert!(rpm.raw_bytes.is_none());
        assert!(rpm.display_value.is_none());
    }

    #[test]
    fn malformed_identity_metadata_is_ignored_without_rejecting_numeric_telemetry() {
        let cases = vec![
            ("cal=414".to_string(), None, None),             // partial byte
            ("cal=GG".to_string(), None, None),              // non-hex
            ("cal=410042".to_string(), None, None),          // non-printable NUL
            ("cal=20".to_string(), None, None),              // trims to empty
            ("cal=4142;ecu=".to_string(), Some("AB"), None), // partial second field
            (
                format!("cal={}", "41".repeat(MAX_IDENTITY_METADATA_BYTES + 1)),
                None,
                None,
            ), // over limit
            ("cal=4142;ecu=414243".to_string(), Some("AB"), Some("ABC")), // valid fields
        ];

        for (suffix, expected_cal, expected_ecu) in cases {
            let line = wire_record(1200, 720.0, 125, &format!("0C;{suffix}"));
            let frame = parse_line(line.trim_end().as_bytes())
                .unwrap_or_else(|| panic!("metadata rejected numeric frame: {suffix}"));
            assert_eq!(frame.calibration_id.as_deref(), expected_cal, "{suffix}");
            assert_eq!(frame.ecu_name.as_deref(), expected_ecu, "{suffix}");
            assert_eq!(
                frame
                    .fields
                    .iter()
                    .find(|field| field.pid == 0x10C)
                    .unwrap()
                    .values,
                [720.0]
            );
        }
    }

    #[test]
    fn rejects_vin_suffixes_that_are_not_exactly_17_ascii_alphanumeric_bytes() {
        for vin in ["1HGCM82633A00435", "1HGCM82633A00435-"] {
            let line = wire_record(1200, 720.0, 125, &format!("0C;vin={vin}"));
            assert!(parse_line(line.trim_end().as_bytes()).is_none(), "{vin}");
        }
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
    fn rejects_checksum_correct_record_with_colonless_field_and_counts_it() {
        let payload = "ABCDEF#0:1250,10C:999,40C:5,BADFIELD";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,42,1250,1,1790966401250,0,0C|{payload}*{checksum:02X}\n");

        assert!(parse_line(line.trim_end().as_bytes()).is_none());
        let mut parser = FreematicsParser::default();
        assert!(parser.feed(line.as_bytes()).is_empty());
        assert_eq!(parser.corrupt_records(), 1);
    }

    #[test]
    fn carries_one_bounded_corrupt_field_sample_to_the_next_valid_frame() {
        let payload = "ABCDEF#0:1250,10C:999,40C:5,BADFIELD";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let malformed = format!("@FT1,42,1250,1,1790966401250,0,0C|{payload}*{checksum:02X}\n");
        let mut parser = FreematicsParser::default();
        assert!(parser.feed(malformed.as_bytes()).is_empty());

        let frame = parser
            .feed(wire_record(1500, 650.0, 20, "0C").as_bytes())
            .remove(0);
        assert_eq!(frame.corrupt_records, 1);
        assert_eq!(
            frame.corrupt_sample_hex.as_deref(),
            Some("field_len=8 field_hex=4241444649454C44")
        );

        let next = parser
            .feed(wire_record(1750, 640.0, 20, "0C").as_bytes())
            .remove(0);
        assert_eq!(next.corrupt_sample_hex, None);
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
    fn malformed_device_ages_are_unavailable_not_falsely_fresh() {
        let payload = "ABCDEF#10C:600,40C:-1,24:1375,94:1.5,142:13.8,442:4294967296";
        let checksum = payload
            .bytes()
            .fold(0u8, |sum, byte| sum.wrapping_add(byte));
        let line = format!("@FT1,9,55,0,0,0,|{payload}*{checksum:02X}");
        let frame = parse_line(line.as_bytes()).unwrap();

        let rpm = frame
            .measurements()
            .into_iter()
            .find(|measurement| measurement.cmd == "010C")
            .unwrap();
        assert_eq!(rpm.value, 600.0);
        assert_eq!(rpm.age_ms, None, "negative age must not become zero");
        assert_eq!(frame.model_b_supply_voltage(), Some((13.75, None)));
        assert_eq!(
            frame.ecu_control_module_voltage(),
            Some((13.8, None)),
            "out-of-range age must not saturate to a plausible age"
        );
    }

    #[test]
    fn decodes_distinct_dtc_modes_status_counts_slots_and_scan_ages() {
        let fields = "300:2,301:264,302:265,303:0,304:0,305:0,306:0,307:0,308:0,309:0,30A:0,30B:0,30C:0,30D:0,30E:0,30F:0,310:2,360:1000,320:0,330:1,361:120000,340:1,341:8721,350:2,362:120001";
        let scans = dtc_wire_frame(fields).dtc_scans();

        assert_eq!(scans[0].mode, FreematicsDtcMode::Stored);
        assert_eq!(scans[0].availability, FreematicsDtcAvailability::Fresh);
        assert_eq!(scans[0].status, Some(FreematicsDtcStatus::Codes));
        assert_eq!(scans[0].count, Some(2));
        assert_eq!(scans[0].code_slots[0], Some(264));
        assert_eq!(scans[0].code_slots[1], Some(265));
        assert_eq!(scans[0].code_slots[2], Some(0));
        assert_eq!(scans[0].age_ms, Some(1000));

        assert_eq!(scans[1].mode, FreematicsDtcMode::Pending);
        assert_eq!(scans[1].availability, FreematicsDtcAvailability::Fresh);
        assert_eq!(scans[1].status, Some(FreematicsDtcStatus::Response));
        assert_eq!(scans[1].count, Some(0));
        assert_eq!(scans[1].age_ms, Some(DTC_SCAN_INTERVAL_MS));

        assert_eq!(scans[2].mode, FreematicsDtcMode::Permanent);
        assert_eq!(scans[2].availability, FreematicsDtcAvailability::Stale);
        assert_eq!(scans[2].status, Some(FreematicsDtcStatus::Codes));
        assert_eq!(scans[2].count, Some(1));
        assert_eq!(scans[2].code_slots[0], Some(8721));
        assert_eq!(scans[2].age_ms, Some(DTC_SCAN_INTERVAL_MS + 1));
    }

    #[test]
    fn distinguishes_unscanned_and_unreported_dtc_modes_from_empty_scan() {
        let scans = dtc_wire_frame("310:0,350:1,340:0,362:1").dtc_scans();

        assert_eq!(scans[0].availability, FreematicsDtcAvailability::NoScan);
        assert_eq!(scans[0].status, Some(FreematicsDtcStatus::NoResponse));
        assert_eq!(scans[0].count, None);
        assert_eq!(scans[0].age_ms, None);

        assert_eq!(
            scans[1].availability,
            FreematicsDtcAvailability::Unsupported
        );
        assert_eq!(scans[1].status, None);

        assert_eq!(scans[2].availability, FreematicsDtcAvailability::Fresh);
        assert_eq!(scans[2].status, Some(FreematicsDtcStatus::Response));
        assert_eq!(scans[2].count, Some(0));
        assert_eq!(scans[2].age_ms, Some(1));
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

    #[test]
    fn simulated_ft2_shudder_scenario_preserves_time_voltage_and_restart() {
        let traffic = include_bytes!("../tests/fixtures/freematics_ft2_shudder_scenario.txt");
        let mut parser = FreematicsParser::default();
        let mut frames = Vec::new();
        for fragment in traffic.chunks(13) {
            frames.extend(parser.feed(fragment));
        }

        assert_eq!(frames.len(), 4);
        assert_eq!(parser.corrupt_records(), 1);
        assert_eq!(frames[0].capture_utc_ms, Some(1_790_966_401_000));
        assert_eq!(frames[0].measurements()[0].value, 820.0);
        assert_eq!(frames[1].measurements()[0].value, 540.0);
        assert_eq!(frames[1].model_b_supply_voltage(), Some((11.8, Some(20))));
        assert_eq!(frames[2].capture_utc_ms, Some(1_790_966_401_500));
        assert_eq!(frames[2].model_b_supply_voltage(), Some((12.9, Some(20))));
        assert_eq!(frames[3].boot_id, 99);
        assert_eq!(frames[3].dropped_records, 2);
    }
}
