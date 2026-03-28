use std::fmt;

// ── OBD PID Definition ──────────────────────────────────────────────────────

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ObdMode {
    Mode01,     // Live data
    Mode02,     // Freeze frame
    Mode03,     // Stored DTCs
    Mode04,     // Clear DTCs
    Mode07,     // Pending DTCs
    Mode09,     // Vehicle info
    Mode0A,     // Permanent DTCs
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Unit {
    Percent,
    Rpm,
    Kmh,
    Celsius,
    Kpa,
    Degrees,
    GramsPerSec,
    Volts,
    Seconds,
    Km,
    Milliamps,
    Pa,
    LitersPerHour,
    Ratio,
    Count,
    None,
}

impl fmt::Display for Unit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Unit::Percent => write!(f, "%"),
            Unit::Rpm => write!(f, "RPM"),
            Unit::Kmh => write!(f, "km/h"),
            Unit::Celsius => write!(f, "°C"),
            Unit::Kpa => write!(f, "kPa"),
            Unit::Degrees => write!(f, "°"),
            Unit::GramsPerSec => write!(f, "g/s"),
            Unit::Volts => write!(f, "V"),
            Unit::Seconds => write!(f, "s"),
            Unit::Km => write!(f, "km"),
            Unit::Milliamps => write!(f, "mA"),
            Unit::Pa => write!(f, "Pa"),
            Unit::LitersPerHour => write!(f, "L/h"),
            Unit::Ratio => write!(f, "λ"),
            Unit::Count => write!(f, ""),
            Unit::None => write!(f, ""),
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Decoder {
    Percent,            // A * 100 / 255
    PercentCentered,    // (A - 128) * 100 / 128
    Temp,               // A - 40
    Rpm,                // (A*256 + B) / 4
    Speed,              // A
    TimingAdvance,      // A / 2 - 64
    Maf,                // (A*256 + B) / 100
    FuelPressure,       // A * 3
    Pressure,           // A
    SensorVoltage,      // A / 200, (B - 128) * 100/128
    ControlModuleVolt,  // (A*256 + B) / 1000
    AbsoluteLoad,       // (A*256 + B) * 100 / 255
    EquivRatio,         // (A*256 + B) / 32768
    EvapPressure,       // ((A*256) + B) / 4  (signed)
    AbsEvapPressure,    // (A*256 + B) / 200
    EvapPressureAlt,    // A*256 + B - 32767
    InjectTiming,       // ((A*256 + B) / 128) - 210
    FuelRate,           // (A*256 + B) / 20
    RunTime,            // A*256 + B
    DistanceU16,        // A*256 + B
    MaxMaf,             // A * 10
    O2WrVoltage,        // ((A*256+B)/32768)*2, ((C*256+D)/256)-128  -> voltage part
    O2WrCurrent,        // ((A*256+B)/32768)*2, ((C*256+D)/256)-128  -> current part
    CatalystTemp,       // (A*256 + B) / 10 - 40
    Count,              // A
    Pid,                // bitmask
    Status,             // special
    FuelStatus,         // special
    AirStatus,          // special
    ObdCompliance,      // lookup
    FuelType,           // lookup
    SingleDtc,          // special
    Dtc,                // special
    EncodedString,      // ASCII
    Drop,               // ignore
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct PidDef {
    pub cmd: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub bytes: u8,
    pub decoder: Decoder,
    pub unit: Unit,
    pub min: f64,
    pub max: f64,
}

/// Decoded OBD value
#[derive(Debug, Clone)]
pub enum ObdValue {
    Numeric(f64),
    Text(String),
    Supported(Vec<u8>),     // supported PID bitmask
    Dtcs(Vec<Dtc>),
    StatusResult(StatusData),
    NoData,
}

impl fmt::Display for ObdValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ObdValue::Numeric(v) => write!(f, "{v:.2}"),
            ObdValue::Text(s) => write!(f, "{s}"),
            ObdValue::Supported(pids) => write!(f, "{} PIDs supported", pids.len()),
            ObdValue::Dtcs(codes) => {
                if codes.is_empty() {
                    write!(f, "No DTCs")
                } else {
                    let strs: Vec<String> = codes.iter().map(|d| d.code.clone()).collect();
                    write!(f, "{}", strs.join(", "))
                }
            }
            ObdValue::StatusResult(s) => write!(f, "MIL: {}, DTCs: {}", s.mil_on, s.dtc_count),
            ObdValue::NoData => write!(f, "N/A"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Dtc {
    pub code: String,
    pub description: String,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct StatusData {
    pub mil_on: bool,
    pub dtc_count: u8,
    pub ignition_type: String,
}

// ── All Mode 01 PIDs ────────────────────────────────────────────────────────

pub fn mode01_pids() -> Vec<PidDef> {
    vec![
        PidDef { cmd: "0100", name: "PIDS_A", description: "Supported PIDs [01-20]", bytes: 6, decoder: Decoder::Pid, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0101", name: "STATUS", description: "Status since DTCs cleared", bytes: 6, decoder: Decoder::Status, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0102", name: "FREEZE_DTC", description: "DTC that triggered freeze frame", bytes: 4, decoder: Decoder::SingleDtc, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0103", name: "FUEL_STATUS", description: "Fuel System Status", bytes: 4, decoder: Decoder::FuelStatus, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0104", name: "ENGINE_LOAD", description: "Calculated Engine Load", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "0105", name: "COOLANT_TEMP", description: "Engine Coolant Temperature", bytes: 3, decoder: Decoder::Temp, unit: Unit::Celsius, min: -40.0, max: 215.0 },
        PidDef { cmd: "0106", name: "SHORT_FUEL_TRIM_1", description: "Short Term Fuel Trim - Bank 1", bytes: 3, decoder: Decoder::PercentCentered, unit: Unit::Percent, min: -100.0, max: 99.2 },
        PidDef { cmd: "0107", name: "LONG_FUEL_TRIM_1", description: "Long Term Fuel Trim - Bank 1", bytes: 3, decoder: Decoder::PercentCentered, unit: Unit::Percent, min: -100.0, max: 99.2 },
        PidDef { cmd: "0108", name: "SHORT_FUEL_TRIM_2", description: "Short Term Fuel Trim - Bank 2", bytes: 3, decoder: Decoder::PercentCentered, unit: Unit::Percent, min: -100.0, max: 99.2 },
        PidDef { cmd: "0109", name: "LONG_FUEL_TRIM_2", description: "Long Term Fuel Trim - Bank 2", bytes: 3, decoder: Decoder::PercentCentered, unit: Unit::Percent, min: -100.0, max: 99.2 },
        PidDef { cmd: "010A", name: "FUEL_PRESSURE", description: "Fuel Pressure", bytes: 3, decoder: Decoder::FuelPressure, unit: Unit::Kpa, min: 0.0, max: 765.0 },
        PidDef { cmd: "010B", name: "INTAKE_PRESSURE", description: "Intake Manifold Pressure", bytes: 3, decoder: Decoder::Pressure, unit: Unit::Kpa, min: 0.0, max: 255.0 },
        PidDef { cmd: "010C", name: "RPM", description: "Engine RPM", bytes: 4, decoder: Decoder::Rpm, unit: Unit::Rpm, min: 0.0, max: 16383.75 },
        PidDef { cmd: "010D", name: "SPEED", description: "Vehicle Speed", bytes: 3, decoder: Decoder::Speed, unit: Unit::Kmh, min: 0.0, max: 255.0 },
        PidDef { cmd: "010E", name: "TIMING_ADVANCE", description: "Timing Advance", bytes: 3, decoder: Decoder::TimingAdvance, unit: Unit::Degrees, min: -64.0, max: 63.5 },
        PidDef { cmd: "010F", name: "INTAKE_TEMP", description: "Intake Air Temperature", bytes: 3, decoder: Decoder::Temp, unit: Unit::Celsius, min: -40.0, max: 215.0 },
        PidDef { cmd: "0110", name: "MAF", description: "Air Flow Rate (MAF)", bytes: 4, decoder: Decoder::Maf, unit: Unit::GramsPerSec, min: 0.0, max: 655.35 },
        PidDef { cmd: "0111", name: "THROTTLE_POS", description: "Throttle Position", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "0112", name: "AIR_STATUS", description: "Secondary Air Status", bytes: 3, decoder: Decoder::AirStatus, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0114", name: "O2_B1S1", description: "O2: Bank 1 - Sensor 1 Voltage", bytes: 4, decoder: Decoder::SensorVoltage, unit: Unit::Volts, min: 0.0, max: 1.275 },
        PidDef { cmd: "0115", name: "O2_B1S2", description: "O2: Bank 1 - Sensor 2 Voltage", bytes: 4, decoder: Decoder::SensorVoltage, unit: Unit::Volts, min: 0.0, max: 1.275 },
        PidDef { cmd: "0116", name: "O2_B1S3", description: "O2: Bank 1 - Sensor 3 Voltage", bytes: 4, decoder: Decoder::SensorVoltage, unit: Unit::Volts, min: 0.0, max: 1.275 },
        PidDef { cmd: "0117", name: "O2_B1S4", description: "O2: Bank 1 - Sensor 4 Voltage", bytes: 4, decoder: Decoder::SensorVoltage, unit: Unit::Volts, min: 0.0, max: 1.275 },
        PidDef { cmd: "0118", name: "O2_B2S1", description: "O2: Bank 2 - Sensor 1 Voltage", bytes: 4, decoder: Decoder::SensorVoltage, unit: Unit::Volts, min: 0.0, max: 1.275 },
        PidDef { cmd: "0119", name: "O2_B2S2", description: "O2: Bank 2 - Sensor 2 Voltage", bytes: 4, decoder: Decoder::SensorVoltage, unit: Unit::Volts, min: 0.0, max: 1.275 },
        PidDef { cmd: "011C", name: "OBD_COMPLIANCE", description: "OBD Standards Compliance", bytes: 3, decoder: Decoder::ObdCompliance, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "011F", name: "RUN_TIME", description: "Engine Run Time", bytes: 4, decoder: Decoder::RunTime, unit: Unit::Seconds, min: 0.0, max: 65535.0 },
        PidDef { cmd: "0120", name: "PIDS_B", description: "Supported PIDs [21-40]", bytes: 6, decoder: Decoder::Pid, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0121", name: "DISTANCE_W_MIL", description: "Distance Traveled with MIL on", bytes: 4, decoder: Decoder::DistanceU16, unit: Unit::Km, min: 0.0, max: 65535.0 },
        PidDef { cmd: "0122", name: "FUEL_RAIL_PRESSURE_VAC", description: "Fuel Rail Pressure (relative to vacuum)", bytes: 4, decoder: Decoder::EvapPressure, unit: Unit::Kpa, min: 0.0, max: 5177.265 },
        PidDef { cmd: "0123", name: "FUEL_RAIL_PRESSURE_DIRECT", description: "Fuel Rail Pressure (direct inject)", bytes: 4, decoder: Decoder::AbsEvapPressure, unit: Unit::Kpa, min: 0.0, max: 655350.0 },
        PidDef { cmd: "0124", name: "O2_S1_WR_VOLTAGE", description: "O2 Sensor 1 WR Lambda Voltage", bytes: 6, decoder: Decoder::O2WrVoltage, unit: Unit::Volts, min: 0.0, max: 8.0 },
        PidDef { cmd: "0125", name: "O2_S2_WR_VOLTAGE", description: "O2 Sensor 2 WR Lambda Voltage", bytes: 6, decoder: Decoder::O2WrVoltage, unit: Unit::Volts, min: 0.0, max: 8.0 },
        PidDef { cmd: "012C", name: "COMMANDED_EGR", description: "Commanded EGR", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "012D", name: "EGR_ERROR", description: "EGR Error", bytes: 3, decoder: Decoder::PercentCentered, unit: Unit::Percent, min: -100.0, max: 99.2 },
        PidDef { cmd: "012E", name: "EVAPORATIVE_PURGE", description: "Commanded Evaporative Purge", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "012F", name: "FUEL_LEVEL", description: "Fuel Level Input", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "0130", name: "WARMUPS_SINCE_DTC_CLEAR", description: "Warm-ups since codes cleared", bytes: 3, decoder: Decoder::Count, unit: Unit::Count, min: 0.0, max: 255.0 },
        PidDef { cmd: "0131", name: "DISTANCE_SINCE_DTC_CLEAR", description: "Distance since codes cleared", bytes: 4, decoder: Decoder::DistanceU16, unit: Unit::Km, min: 0.0, max: 65535.0 },
        PidDef { cmd: "0132", name: "EVAP_VAPOR_PRESSURE", description: "Evap system vapor pressure", bytes: 4, decoder: Decoder::EvapPressure, unit: Unit::Pa, min: -8192.0, max: 8191.75 },
        PidDef { cmd: "0133", name: "BAROMETRIC_PRESSURE", description: "Barometric Pressure", bytes: 3, decoder: Decoder::Pressure, unit: Unit::Kpa, min: 0.0, max: 255.0 },
        PidDef { cmd: "0134", name: "O2_S1_WR_CURRENT", description: "O2 Sensor 1 WR Lambda Current", bytes: 6, decoder: Decoder::O2WrCurrent, unit: Unit::Milliamps, min: -128.0, max: 128.0 },
        PidDef { cmd: "013C", name: "CATALYST_TEMP_B1S1", description: "Catalyst Temp: Bank 1 - Sensor 1", bytes: 4, decoder: Decoder::CatalystTemp, unit: Unit::Celsius, min: -40.0, max: 6513.5 },
        PidDef { cmd: "013D", name: "CATALYST_TEMP_B2S1", description: "Catalyst Temp: Bank 2 - Sensor 1", bytes: 4, decoder: Decoder::CatalystTemp, unit: Unit::Celsius, min: -40.0, max: 6513.5 },
        PidDef { cmd: "013E", name: "CATALYST_TEMP_B1S2", description: "Catalyst Temp: Bank 1 - Sensor 2", bytes: 4, decoder: Decoder::CatalystTemp, unit: Unit::Celsius, min: -40.0, max: 6513.5 },
        PidDef { cmd: "013F", name: "CATALYST_TEMP_B2S2", description: "Catalyst Temp: Bank 2 - Sensor 2", bytes: 4, decoder: Decoder::CatalystTemp, unit: Unit::Celsius, min: -40.0, max: 6513.5 },
        PidDef { cmd: "0140", name: "PIDS_C", description: "Supported PIDs [41-60]", bytes: 6, decoder: Decoder::Pid, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0142", name: "CONTROL_MODULE_VOLTAGE", description: "Control Module Voltage", bytes: 4, decoder: Decoder::ControlModuleVolt, unit: Unit::Volts, min: 0.0, max: 65.535 },
        PidDef { cmd: "0143", name: "ABSOLUTE_LOAD", description: "Absolute Load Value", bytes: 4, decoder: Decoder::AbsoluteLoad, unit: Unit::Percent, min: 0.0, max: 25700.0 },
        PidDef { cmd: "0144", name: "COMMANDED_EQUIV_RATIO", description: "Commanded Equivalence Ratio", bytes: 4, decoder: Decoder::EquivRatio, unit: Unit::Ratio, min: 0.0, max: 2.0 },
        PidDef { cmd: "0145", name: "RELATIVE_THROTTLE_POS", description: "Relative Throttle Position", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "0146", name: "AMBIENT_AIR_TEMP", description: "Ambient Air Temperature", bytes: 3, decoder: Decoder::Temp, unit: Unit::Celsius, min: -40.0, max: 215.0 },
        PidDef { cmd: "0147", name: "THROTTLE_POS_B", description: "Absolute Throttle Position B", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "0148", name: "THROTTLE_POS_C", description: "Absolute Throttle Position C", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "0149", name: "ACCELERATOR_POS_D", description: "Accelerator Pedal Position D", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "014A", name: "ACCELERATOR_POS_E", description: "Accelerator Pedal Position E", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "014C", name: "THROTTLE_ACTUATOR", description: "Commanded Throttle Actuator", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "014D", name: "RUN_TIME_MIL", description: "Time Run with MIL on", bytes: 4, decoder: Decoder::RunTime, unit: Unit::Seconds, min: 0.0, max: 65535.0 },
        PidDef { cmd: "014E", name: "TIME_SINCE_DTC_CLEARED", description: "Time since DTCs cleared", bytes: 4, decoder: Decoder::RunTime, unit: Unit::Seconds, min: 0.0, max: 65535.0 },
        PidDef { cmd: "0151", name: "FUEL_TYPE", description: "Fuel Type", bytes: 3, decoder: Decoder::FuelType, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0152", name: "ETHANOL_PERCENT", description: "Ethanol Fuel Percent", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "015B", name: "HYBRID_BATTERY_REMAINING", description: "Hybrid Battery Pack Remaining Life", bytes: 3, decoder: Decoder::Percent, unit: Unit::Percent, min: 0.0, max: 100.0 },
        PidDef { cmd: "015C", name: "OIL_TEMP", description: "Engine Oil Temperature", bytes: 3, decoder: Decoder::Temp, unit: Unit::Celsius, min: -40.0, max: 215.0 },
        PidDef { cmd: "015D", name: "FUEL_INJECT_TIMING", description: "Fuel Injection Timing", bytes: 4, decoder: Decoder::InjectTiming, unit: Unit::Degrees, min: -210.0, max: 301.992 },
        PidDef { cmd: "015E", name: "FUEL_RATE", description: "Engine Fuel Rate", bytes: 4, decoder: Decoder::FuelRate, unit: Unit::LitersPerHour, min: 0.0, max: 3276.75 },
    ]
}

#[allow(dead_code)]
pub fn mode09_pids() -> Vec<PidDef> {
    vec![
        PidDef { cmd: "0900", name: "PIDS_9A", description: "Supported PIDs [01-20]", bytes: 7, decoder: Decoder::Pid, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0902", name: "VIN", description: "Vehicle Identification Number", bytes: 22, decoder: Decoder::EncodedString, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "0904", name: "CALIBRATION_ID", description: "Calibration ID", bytes: 18, decoder: Decoder::EncodedString, unit: Unit::None, min: 0.0, max: 0.0 },
        PidDef { cmd: "090A", name: "ECU_NAME", description: "ECU Name", bytes: 22, decoder: Decoder::EncodedString, unit: Unit::None, min: 0.0, max: 0.0 },
    ]
}

/// PIDs that are good for dashboard gauges
#[allow(dead_code)]
pub fn gauge_pids() -> Vec<&'static str> {
    vec!["010C", "010D", "0105", "0111", "0104", "015C", "012F", "0142"]
}

// ── Decoding functions ──────────────────────────────────────────────────────

pub fn decode_pid(pid: &PidDef, data: &[u8]) -> ObdValue {
    if data.is_empty() {
        return ObdValue::NoData;
    }
    match pid.decoder {
        Decoder::Percent => {
            let a = data[0] as f64;
            ObdValue::Numeric(a * 100.0 / 255.0)
        }
        Decoder::PercentCentered => {
            let a = data[0] as f64;
            ObdValue::Numeric((a - 128.0) * 100.0 / 128.0)
        }
        Decoder::Temp => {
            let a = data[0] as f64;
            ObdValue::Numeric(a - 40.0)
        }
        Decoder::Rpm => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 4.0)
        }
        Decoder::Speed => {
            ObdValue::Numeric(data[0] as f64)
        }
        Decoder::TimingAdvance => {
            let a = data[0] as f64;
            ObdValue::Numeric(a / 2.0 - 64.0)
        }
        Decoder::Maf => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 100.0)
        }
        Decoder::FuelPressure => {
            ObdValue::Numeric(data[0] as f64 * 3.0)
        }
        Decoder::Pressure => {
            ObdValue::Numeric(data[0] as f64)
        }
        Decoder::SensorVoltage => {
            let a = data[0] as f64;
            ObdValue::Numeric(a / 200.0)
        }
        Decoder::ControlModuleVolt => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 1000.0)
        }
        Decoder::AbsoluteLoad => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) * 100.0 / 255.0)
        }
        Decoder::EquivRatio => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 32768.0)
        }
        Decoder::EvapPressure => {
            if data.len() < 2 { return ObdValue::NoData; }
            let raw = (data[0] as i16) * 256 + data[1] as i16;
            ObdValue::Numeric(raw as f64 / 4.0)
        }
        Decoder::AbsEvapPressure => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 200.0)
        }
        Decoder::EvapPressureAlt => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric(a * 256.0 + b - 32767.0)
        }
        Decoder::InjectTiming => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 128.0 - 210.0)
        }
        Decoder::FuelRate => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 20.0)
        }
        Decoder::RunTime | Decoder::DistanceU16 => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric(a * 256.0 + b)
        }
        Decoder::MaxMaf => {
            ObdValue::Numeric(data[0] as f64 * 10.0)
        }
        Decoder::O2WrVoltage => {
            if data.len() < 4 { return ObdValue::NoData; }
            let c = data[2] as f64;
            let d = data[3] as f64;
            ObdValue::Numeric((c * 256.0 + d) * 8.0 / 65536.0)
        }
        Decoder::O2WrCurrent => {
            if data.len() < 4 { return ObdValue::NoData; }
            let c = data[2] as f64;
            let d = data[3] as f64;
            ObdValue::Numeric((c * 256.0 + d) / 256.0 - 128.0)
        }
        Decoder::CatalystTemp => {
            if data.len() < 2 { return ObdValue::NoData; }
            let a = data[0] as f64;
            let b = data[1] as f64;
            ObdValue::Numeric((a * 256.0 + b) / 10.0 - 40.0)
        }
        Decoder::Count => {
            ObdValue::Numeric(data[0] as f64)
        }
        Decoder::Pid => {
            if data.len() < 4 { return ObdValue::NoData; }
            let bits = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
            let base_pid = u8::from_str_radix(&pid.cmd[2..4], 16).unwrap_or(0);
            let mut supported = Vec::new();
            for i in 0..32 {
                if bits & (1 << (31 - i)) != 0 {
                    supported.push(base_pid + i as u8 + 1);
                }
            }
            ObdValue::Supported(supported)
        }
        Decoder::Status => {
            if data.len() < 4 { return ObdValue::NoData; }
            let mil_on = (data[0] & 0x80) != 0;
            let dtc_count = data[0] & 0x7F;
            let ignition_type = if (data[1] & 0x08) != 0 {
                "Compression (Diesel)".to_string()
            } else {
                "Spark (Gasoline)".to_string()
            };
            ObdValue::StatusResult(StatusData { mil_on, dtc_count, ignition_type })
        }
        Decoder::FuelStatus => {
            let status = match data[0] {
                1 => "Open loop - insufficient engine temp",
                2 => "Closed loop - using O2 sensor",
                4 => "Open loop - engine load/decel",
                8 => "Open loop - system failure",
                16 => "Closed loop - feedback fault",
                _ => "Unknown",
            };
            ObdValue::Text(status.to_string())
        }
        Decoder::AirStatus => {
            let status = match data[0] {
                1 => "Upstream",
                2 => "Downstream of catalytic converter",
                4 => "From outside atmosphere or off",
                8 => "Pump commanded on for diagnostics",
                _ => "Unknown",
            };
            ObdValue::Text(status.to_string())
        }
        Decoder::ObdCompliance => {
            let s = match data[0] {
                1 => "OBD-II (CARB)",
                2 => "OBD (EPA)",
                3 => "OBD + OBD-II",
                4 => "OBD-I",
                5 => "Not OBD compliant",
                6 => "EOBD (Europe)",
                7 => "EOBD + OBD-II",
                8 => "EOBD + OBD",
                9 => "EOBD + OBD + OBD-II",
                10 => "JOBD (Japan)",
                11 => "JOBD + OBD-II",
                12 => "JOBD + EOBD",
                13 => "JOBD + EOBD + OBD-II",
                _ => "Unknown",
            };
            ObdValue::Text(s.to_string())
        }
        Decoder::FuelType => {
            let s = match data[0] {
                0 => "Not available",
                1 => "Gasoline",
                2 => "Methanol",
                3 => "Ethanol",
                4 => "Diesel",
                5 => "LPG",
                6 => "CNG",
                7 => "Propane",
                8 => "Electric",
                9 => "Bifuel (Gasoline)",
                10 => "Bifuel (Methanol)",
                11 => "Bifuel (Ethanol)",
                12 => "Bifuel (LPG)",
                13 => "Bifuel (CNG)",
                14 => "Bifuel (Propane)",
                15 => "Bifuel (Electric)",
                16 => "Bifuel (Gasoline/Electric)",
                17 => "Hybrid (Gasoline)",
                18 => "Hybrid (Ethanol)",
                19 => "Hybrid (Diesel)",
                20 => "Hybrid (Electric)",
                21 => "Hybrid (Mixed)",
                22 => "Hybrid (Regenerative)",
                23 => "Bifuel (Diesel)",
                _ => "Unknown",
            };
            ObdValue::Text(s.to_string())
        }
        Decoder::SingleDtc => {
            if data.len() < 2 { return ObdValue::NoData; }
            if data[0] == 0 && data[1] == 0 {
                return ObdValue::Text("No freeze frame DTC".to_string());
            }
            let code = decode_dtc_bytes(data[0], data[1]);
            ObdValue::Text(code)
        }
        Decoder::Dtc => {
            ObdValue::Dtcs(decode_dtc_response(data))
        }
        Decoder::EncodedString => {
            let s: String = data.iter()
                .filter(|&&b| b >= 0x20 && b < 0x7F)
                .map(|&b| b as char)
                .collect();
            ObdValue::Text(s.trim().to_string())
        }
        Decoder::Drop => ObdValue::NoData,
    }
}

// ── DTC decoding ────────────────────────────────────────────────────────────

pub fn decode_dtc_bytes(b1: u8, b2: u8) -> String {
    let prefix = match (b1 >> 6) & 0x03 {
        0 => 'P',
        1 => 'C',
        2 => 'B',
        3 => 'U',
        _ => '?',
    };
    let d1 = (b1 >> 4) & 0x03;
    let d2 = b1 & 0x0F;
    let d3 = (b2 >> 4) & 0x0F;
    let d4 = b2 & 0x0F;
    format!("{}{}{:X}{:X}{:X}", prefix, d1, d2, d3, d4)
}

pub fn decode_dtc_response(data: &[u8]) -> Vec<Dtc> {
    let mut codes = Vec::new();
    let mut i = 0;
    while i + 1 < data.len() {
        if data[i] == 0 && data[i + 1] == 0 {
            i += 2;
            continue;
        }
        let code = decode_dtc_bytes(data[i], data[i + 1]);
        codes.push(Dtc {
            code,
            description: String::new(),
        });
        i += 2;
    }
    codes
}

/// Parse raw response lines from ELM327 for a given PID command
pub fn parse_elm_response(cmd: &str, lines: &[String]) -> Option<Vec<u8>> {
    let mode_response = format!("4{}", &cmd[1..2]);
    let pid_hex = &cmd[2..4];
    let prefix = format!("{}{}", mode_response, pid_hex).to_uppercase();

    for line in lines {
        let clean = line.replace(' ', "").to_uppercase();
        if clean.starts_with("NODATA") || clean.starts_with("ERROR") || clean.starts_with("?") || clean.starts_with("UNABLE") {
            return None;
        }
        if let Some(pos) = clean.find(&prefix) {
            let data_str = &clean[pos + prefix.len()..];
            let mut bytes = Vec::new();
            let mut i = 0;
            while i + 1 < data_str.len() {
                if let Ok(byte) = u8::from_str_radix(&data_str[i..i + 2], 16) {
                    bytes.push(byte);
                }
                i += 2;
            }
            if !bytes.is_empty() {
                return Some(bytes);
            }
        }
    }
    None
}

/// Parse DTC response lines (mode 03/07/0A)
pub fn parse_dtc_response_lines(lines: &[String], response_prefix: &str) -> Vec<Dtc> {
    let mut all_bytes = Vec::new();
    for line in lines {
        let clean = line.replace(' ', "").to_uppercase();
        if !clean.starts_with(response_prefix) { continue; }
        let data_part = &clean[response_prefix.len()..];
        let mut i = 0;
        while i + 1 < data_part.len() {
            if let Ok(byte) = u8::from_str_radix(&data_part[i..i + 2], 16) {
                all_bytes.push(byte);
            }
            i += 2;
        }
    }
    decode_dtc_response(&all_bytes)
}

/// Parse multi-line encoded string response (VIN, Calibration ID, etc.)
///
/// ELM327 multi-frame responses (ISO-TP) come in several formats:
///
/// Format A (with headers off, spaces off, CAN multi-frame):
///   014              <- number of lines to follow
///   0:490201574F4C   <- first frame: seq 0, then 4902 01 + data
///   1:4A4533303030   <- continuation: seq 1 + data
///   2:3030303030300  <- continuation: seq 2 + data
///
/// Format B (single-line responses per message):
///   49020157...
///   49020200...
///   49020300...
///
/// Format C (spaces on):
///   49 02 01 57 4F 4C ...
pub fn parse_encoded_string_response(lines: &[String], response_prefix: &str) -> Option<String> {
    let mut all_hex = String::new();
    let mut in_multiframe = false;

    for line in lines {
        let clean = line.replace(' ', "").to_uppercase();

        // Skip empty, NO DATA, prompts
        if clean.is_empty() || clean.starts_with("NODATA") || clean.starts_with("?") {
            continue;
        }

        // Multi-frame: line starts with digit + colon (e.g. "0:", "1:", "2:")
        if clean.len() >= 2 && clean.as_bytes()[1] == b':' && clean.as_bytes()[0].is_ascii_digit() {
            in_multiframe = true;
            let seq = clean.as_bytes()[0] - b'0';
            let data = &clean[2..];

            if seq == 0 {
                // First frame: contains response prefix + count byte, then data
                // e.g. "490201574F4C..." -> skip "490201" (prefix=4902, count=01)
                if let Some(pos) = data.find(response_prefix) {
                    let after_prefix = &data[pos + response_prefix.len()..];
                    // Skip the count/sequence byte (2 hex chars)
                    let payload = if after_prefix.len() >= 2 { &after_prefix[2..] } else { after_prefix };
                    all_hex.push_str(payload);
                } else {
                    // No prefix found in frame 0 - just take data after prefix len
                    all_hex.push_str(data);
                }
            } else {
                // Continuation frames: all data
                all_hex.push_str(data);
            }
            continue;
        }

        // Single-frame format: line contains the response prefix directly
        if clean.contains(response_prefix) {
            if let Some(pos) = clean.find(response_prefix) {
                let after_prefix = &clean[pos + response_prefix.len()..];
                // Skip count/sequence byte (2 hex chars)
                let payload = if after_prefix.len() >= 2 { &after_prefix[2..] } else { after_prefix };
                all_hex.push_str(payload);
            }
            continue;
        }

        // If we were in a multi-frame and this line is just hex data (no prefix, no colon)
        // it might be a continuation without sequence numbers (rare but possible)
        if in_multiframe && clean.chars().all(|c| c.is_ascii_hexdigit()) {
            all_hex.push_str(&clean);
        }
    }

    if all_hex.is_empty() {
        return None;
    }

    // Decode hex to ASCII
    let mut result = String::new();
    let mut i = 0;
    let bytes = all_hex.as_bytes();
    while i + 1 < bytes.len() {
        if let Ok(byte) = u8::from_str_radix(
            std::str::from_utf8(&bytes[i..i + 2]).unwrap_or(""),
            16,
        ) {
            if byte >= 0x20 && byte < 0x7F {
                result.push(byte as char);
            }
        }
        i += 2;
    }

    let trimmed = result.trim().to_string();
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed)
    }
}