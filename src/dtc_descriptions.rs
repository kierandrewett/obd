use std::collections::HashMap;
use std::sync::LazyLock;

/// Common OBD-II DTC descriptions
/// Source: SAE J2012 / ISO 15031-6 standard codes
static DTC_MAP: LazyLock<HashMap<&'static str, &'static str>> = LazyLock::new(|| {
    let mut m = HashMap::new();

    // ── Fuel and Air Metering ───────────────────────────────────────────
    m.insert("P0001", "Fuel Volume Regulator Control Circuit/Open");
    m.insert(
        "P0002",
        "Fuel Volume Regulator Control Circuit Range/Performance",
    );
    m.insert("P0003", "Fuel Volume Regulator Control Circuit Low");
    m.insert("P0004", "Fuel Volume Regulator Control Circuit High");
    m.insert(
        "P0010",
        "Intake Camshaft Position Actuator Circuit (Bank 1)",
    );
    m.insert(
        "P0011",
        "Intake Camshaft Position Timing Over-Advanced (Bank 1)",
    );
    m.insert(
        "P0012",
        "Intake Camshaft Position Timing Over-Retarded (Bank 1)",
    );
    m.insert(
        "P0013",
        "Exhaust Camshaft Position Actuator Circuit (Bank 1)",
    );
    m.insert(
        "P0014",
        "Exhaust Camshaft Position Timing Over-Advanced (Bank 1)",
    );
    m.insert(
        "P0015",
        "Exhaust Camshaft Position Timing Over-Retarded (Bank 1)",
    );
    m.insert(
        "P0016",
        "Crankshaft Position - Camshaft Position Correlation (Bank 1 Sensor A)",
    );
    m.insert(
        "P0017",
        "Crankshaft Position - Camshaft Position Correlation (Bank 1 Sensor B)",
    );
    m.insert(
        "P0018",
        "Crankshaft Position - Camshaft Position Correlation (Bank 2 Sensor A)",
    );
    m.insert(
        "P0019",
        "Crankshaft Position - Camshaft Position Correlation (Bank 2 Sensor B)",
    );
    m.insert(
        "P0020",
        "Intake Camshaft Position Actuator Circuit (Bank 2)",
    );
    m.insert(
        "P0021",
        "Intake Camshaft Position Timing Over-Advanced (Bank 2)",
    );
    m.insert(
        "P0022",
        "Intake Camshaft Position Timing Over-Retarded (Bank 2)",
    );
    m.insert(
        "P0023",
        "Exhaust Camshaft Position Actuator Circuit (Bank 2)",
    );
    m.insert(
        "P0024",
        "Exhaust Camshaft Position Timing Over-Advanced (Bank 2)",
    );
    m.insert(
        "P0025",
        "Exhaust Camshaft Position Timing Over-Retarded (Bank 2)",
    );
    m.insert("P0030", "HO2S Heater Control Circuit (Bank 1, Sensor 1)");
    m.insert(
        "P0031",
        "HO2S Heater Control Circuit Low (Bank 1, Sensor 1)",
    );
    m.insert(
        "P0032",
        "HO2S Heater Control Circuit High (Bank 1, Sensor 1)",
    );
    m.insert("P0036", "HO2S Heater Control Circuit (Bank 1, Sensor 2)");
    m.insert(
        "P0037",
        "HO2S Heater Control Circuit Low (Bank 1, Sensor 2)",
    );
    m.insert(
        "P0038",
        "HO2S Heater Control Circuit High (Bank 1, Sensor 2)",
    );

    // ── Fuel System ─────────────────────────────────────────────────────
    m.insert("P0100", "Mass or Volume Air Flow Circuit");
    m.insert("P0101", "Mass or Volume Air Flow Circuit Range/Performance");
    m.insert("P0102", "Mass or Volume Air Flow Circuit Low Input");
    m.insert("P0103", "Mass or Volume Air Flow Circuit High Input");
    m.insert("P0104", "Mass or Volume Air Flow Circuit Intermittent");
    m.insert(
        "P0105",
        "Manifold Absolute Pressure/Barometric Pressure Circuit",
    );
    m.insert("P0106", "MAP/Barometric Pressure Circuit Range/Performance");
    m.insert("P0107", "MAP/Barometric Pressure Circuit Low Input");
    m.insert("P0108", "MAP/Barometric Pressure Circuit High Input");
    m.insert("P0109", "MAP/Barometric Pressure Circuit Intermittent");
    m.insert("P0110", "Intake Air Temperature Circuit");
    m.insert("P0111", "Intake Air Temperature Circuit Range/Performance");
    m.insert("P0112", "Intake Air Temperature Circuit Low Input");
    m.insert("P0113", "Intake Air Temperature Circuit High Input");
    m.insert("P0114", "Intake Air Temperature Circuit Intermittent");
    m.insert("P0115", "Engine Coolant Temperature Circuit");
    m.insert(
        "P0116",
        "Engine Coolant Temperature Circuit Range/Performance",
    );
    m.insert("P0117", "Engine Coolant Temperature Circuit Low Input");
    m.insert("P0118", "Engine Coolant Temperature Circuit High Input");
    m.insert("P0119", "Engine Coolant Temperature Circuit Intermittent");
    m.insert("P0120", "Throttle/Pedal Position Sensor/Switch A Circuit");
    m.insert(
        "P0121",
        "Throttle/Pedal Position Sensor/Switch A Range/Performance",
    );
    m.insert(
        "P0122",
        "Throttle/Pedal Position Sensor/Switch A Circuit Low Input",
    );
    m.insert(
        "P0123",
        "Throttle/Pedal Position Sensor/Switch A Circuit High Input",
    );
    m.insert(
        "P0125",
        "Insufficient Coolant Temperature for Closed Loop Fuel Control",
    );
    m.insert(
        "P0128",
        "Coolant Thermostat (Coolant Temperature Below Thermostat Regulating Temperature)",
    );
    m.insert("P0130", "O2 Sensor Circuit (Bank 1, Sensor 1)");
    m.insert("P0131", "O2 Sensor Circuit Low Voltage (Bank 1, Sensor 1)");
    m.insert("P0132", "O2 Sensor Circuit High Voltage (Bank 1, Sensor 1)");
    m.insert(
        "P0133",
        "O2 Sensor Circuit Slow Response (Bank 1, Sensor 1)",
    );
    m.insert(
        "P0134",
        "O2 Sensor Circuit No Activity Detected (Bank 1, Sensor 1)",
    );
    m.insert("P0135", "O2 Sensor Heater Circuit (Bank 1, Sensor 1)");
    m.insert("P0136", "O2 Sensor Circuit (Bank 1, Sensor 2)");
    m.insert("P0137", "O2 Sensor Circuit Low Voltage (Bank 1, Sensor 2)");
    m.insert("P0138", "O2 Sensor Circuit High Voltage (Bank 1, Sensor 2)");
    m.insert(
        "P0139",
        "O2 Sensor Circuit Slow Response (Bank 1, Sensor 2)",
    );
    m.insert(
        "P0140",
        "O2 Sensor Circuit No Activity Detected (Bank 1, Sensor 2)",
    );
    m.insert("P0141", "O2 Sensor Heater Circuit (Bank 1, Sensor 2)");
    m.insert("P0150", "O2 Sensor Circuit (Bank 2, Sensor 1)");
    m.insert("P0151", "O2 Sensor Circuit Low Voltage (Bank 2, Sensor 1)");
    m.insert("P0152", "O2 Sensor Circuit High Voltage (Bank 2, Sensor 1)");
    m.insert(
        "P0153",
        "O2 Sensor Circuit Slow Response (Bank 2, Sensor 1)",
    );
    m.insert(
        "P0154",
        "O2 Sensor Circuit No Activity Detected (Bank 2, Sensor 1)",
    );
    m.insert("P0155", "O2 Sensor Heater Circuit (Bank 2, Sensor 1)");
    m.insert("P0156", "O2 Sensor Circuit (Bank 2, Sensor 2)");
    m.insert("P0157", "O2 Sensor Circuit Low Voltage (Bank 2, Sensor 2)");
    m.insert("P0158", "O2 Sensor Circuit High Voltage (Bank 2, Sensor 2)");
    m.insert(
        "P0159",
        "O2 Sensor Circuit Slow Response (Bank 2, Sensor 2)",
    );
    m.insert(
        "P0160",
        "O2 Sensor Circuit No Activity Detected (Bank 2, Sensor 2)",
    );
    m.insert("P0161", "O2 Sensor Heater Circuit (Bank 2, Sensor 2)");

    // ── Ignition / Misfire ──────────────────────────────────────────────
    m.insert("P0171", "System Too Lean (Bank 1)");
    m.insert("P0172", "System Too Rich (Bank 1)");
    m.insert("P0174", "System Too Lean (Bank 2)");
    m.insert("P0175", "System Too Rich (Bank 2)");
    m.insert("P0200", "Injector Circuit");
    m.insert("P0201", "Injector Circuit - Cylinder 1");
    m.insert("P0202", "Injector Circuit - Cylinder 2");
    m.insert("P0203", "Injector Circuit - Cylinder 3");
    m.insert("P0204", "Injector Circuit - Cylinder 4");
    m.insert("P0205", "Injector Circuit - Cylinder 5");
    m.insert("P0206", "Injector Circuit - Cylinder 6");
    m.insert("P0207", "Injector Circuit - Cylinder 7");
    m.insert("P0208", "Injector Circuit - Cylinder 8");
    m.insert("P0217", "Engine Overtemperature Condition");
    m.insert("P0218", "Transmission Over Temperature Condition");
    m.insert("P0219", "Engine Overspeed Condition");
    m.insert("P0220", "Throttle/Pedal Position Sensor/Switch B Circuit");
    m.insert(
        "P0221",
        "Throttle/Pedal Position Sensor/Switch B Range/Performance",
    );
    m.insert(
        "P0222",
        "Throttle/Pedal Position Sensor/Switch B Circuit Low Input",
    );
    m.insert(
        "P0223",
        "Throttle/Pedal Position Sensor/Switch B Circuit High Input",
    );
    m.insert("P0230", "Fuel Pump Primary Circuit");
    m.insert("P0261", "Cylinder 1 Injector Circuit Low");
    m.insert("P0262", "Cylinder 1 Injector Circuit High");
    m.insert("P0263", "Cylinder 1 Contribution/Balance");
    m.insert("P0264", "Cylinder 2 Injector Circuit Low");
    m.insert("P0265", "Cylinder 2 Injector Circuit High");
    m.insert("P0267", "Cylinder 3 Injector Circuit Low");
    m.insert("P0268", "Cylinder 3 Injector Circuit High");
    m.insert("P0270", "Cylinder 4 Injector Circuit Low");
    m.insert("P0271", "Cylinder 4 Injector Circuit High");
    m.insert("P0300", "Random/Multiple Cylinder Misfire Detected");
    m.insert("P0301", "Cylinder 1 Misfire Detected");
    m.insert("P0302", "Cylinder 2 Misfire Detected");
    m.insert("P0303", "Cylinder 3 Misfire Detected");
    m.insert("P0304", "Cylinder 4 Misfire Detected");
    m.insert("P0305", "Cylinder 5 Misfire Detected");
    m.insert("P0306", "Cylinder 6 Misfire Detected");
    m.insert("P0307", "Cylinder 7 Misfire Detected");
    m.insert("P0308", "Cylinder 8 Misfire Detected");
    m.insert("P0325", "Knock Sensor 1 Circuit (Bank 1)");
    m.insert("P0326", "Knock Sensor 1 Circuit Range/Performance (Bank 1)");
    m.insert("P0327", "Knock Sensor 1 Circuit Low Input (Bank 1)");
    m.insert("P0328", "Knock Sensor 1 Circuit High Input (Bank 1)");
    m.insert("P0330", "Knock Sensor 2 Circuit (Bank 2)");
    m.insert("P0335", "Crankshaft Position Sensor A Circuit");
    m.insert(
        "P0336",
        "Crankshaft Position Sensor A Circuit Range/Performance",
    );
    m.insert("P0340", "Camshaft Position Sensor A Circuit (Bank 1)");
    m.insert(
        "P0341",
        "Camshaft Position Sensor A Circuit Range/Performance (Bank 1)",
    );
    m.insert("P0345", "Camshaft Position Sensor A Circuit (Bank 2)");

    // ── Emission Controls ───────────────────────────────────────────────
    m.insert("P0400", "Exhaust Gas Recirculation Flow");
    m.insert(
        "P0401",
        "Exhaust Gas Recirculation Flow Insufficient Detected",
    );
    m.insert("P0402", "Exhaust Gas Recirculation Flow Excessive Detected");
    m.insert("P0403", "Exhaust Gas Recirculation Circuit");
    m.insert(
        "P0404",
        "Exhaust Gas Recirculation Circuit Range/Performance",
    );
    m.insert("P0405", "Exhaust Gas Recirculation Sensor A Circuit Low");
    m.insert("P0406", "Exhaust Gas Recirculation Sensor A Circuit High");
    m.insert("P0410", "Secondary Air Injection System");
    m.insert(
        "P0411",
        "Secondary Air Injection System Incorrect Flow Detected",
    );
    m.insert(
        "P0420",
        "Catalyst System Efficiency Below Threshold (Bank 1)",
    );
    m.insert(
        "P0421",
        "Warm Up Catalyst Efficiency Below Threshold (Bank 1)",
    );
    m.insert(
        "P0430",
        "Catalyst System Efficiency Below Threshold (Bank 2)",
    );
    m.insert("P0440", "Evaporative Emission Control System");
    m.insert(
        "P0441",
        "Evaporative Emission Control System Incorrect Purge Flow",
    );
    m.insert(
        "P0442",
        "Evaporative Emission Control System Leak Detected (Small Leak)",
    );
    m.insert(
        "P0443",
        "Evaporative Emission Control System Purge Control Valve Circuit",
    );
    m.insert(
        "P0446",
        "Evaporative Emission Control System Vent Control Circuit",
    );
    m.insert(
        "P0449",
        "Evaporative Emission Control System Vent Valve/Solenoid Circuit",
    );
    m.insert(
        "P0450",
        "Evaporative Emission Control System Pressure Sensor",
    );
    m.insert(
        "P0451",
        "Evaporative Emission Control System Pressure Sensor Range/Performance",
    );
    m.insert(
        "P0452",
        "Evaporative Emission Control System Pressure Sensor Low Input",
    );
    m.insert(
        "P0453",
        "Evaporative Emission Control System Pressure Sensor High Input",
    );
    m.insert(
        "P0455",
        "Evaporative Emission Control System Leak Detected (Gross Leak)",
    );
    m.insert(
        "P0456",
        "Evaporative Emission Control System Leak Detected (Very Small Leak)",
    );

    // ── Vehicle Speed / Idle Control ────────────────────────────────────
    m.insert("P0500", "Vehicle Speed Sensor");
    m.insert("P0501", "Vehicle Speed Sensor Range/Performance");
    m.insert("P0503", "Vehicle Speed Sensor Intermittent/Erratic/High");
    m.insert("P0505", "Idle Control System");
    m.insert("P0506", "Idle Control System RPM Lower Than Expected");
    m.insert("P0507", "Idle Control System RPM Higher Than Expected");
    m.insert("P0510", "Closed Throttle Position Switch");
    m.insert("P0520", "Engine Oil Pressure Sensor/Switch Circuit");
    m.insert(
        "P0521",
        "Engine Oil Pressure Sensor/Switch Range/Performance",
    );
    m.insert("P0522", "Engine Oil Pressure Sensor/Switch Low Voltage");
    m.insert("P0523", "Engine Oil Pressure Sensor/Switch High Voltage");
    m.insert("P0530", "A/C Refrigerant Pressure Sensor Circuit");
    m.insert("P0562", "System Voltage Low");
    m.insert("P0563", "System Voltage High");

    // ── Computer / Auxiliary ────────────────────────────────────────────
    m.insert("P0600", "Serial Communication Link");
    m.insert("P0601", "Internal Control Module Memory Check Sum Error");
    m.insert("P0602", "Control Module Programming Error");
    m.insert("P0603", "Internal Control Module KAM Error");
    m.insert("P0604", "Internal Control Module RAM Error");
    m.insert("P0606", "PCM Processor Fault");
    m.insert("P0607", "Control Module Performance");
    m.insert("P0610", "Control Module Vehicle Options Error");
    m.insert("P0620", "Generator Control Circuit");
    m.insert("P0627", "Fuel Pump A Control Circuit/Open");
    m.insert("P0628", "Fuel Pump A Control Circuit Low");
    m.insert("P0629", "Fuel Pump A Control Circuit High");

    // ── Transmission ────────────────────────────────────────────────────
    m.insert("P0700", "Transmission Control System (MIL Request)");
    m.insert("P0701", "Transmission Control System Range/Performance");
    m.insert("P0705", "Transmission Range Sensor Circuit (PRNDL Input)");
    m.insert(
        "P0706",
        "Transmission Range Sensor Circuit Range/Performance",
    );
    m.insert("P0710", "Transmission Fluid Temperature Sensor Circuit");
    m.insert("P0715", "Input/Turbine Speed Sensor Circuit");
    m.insert("P0720", "Output Speed Sensor Circuit");
    m.insert("P0725", "Engine Speed Input Circuit");
    m.insert("P0730", "Incorrect Gear Ratio");
    m.insert("P0731", "Gear 1 Incorrect Ratio");
    m.insert("P0732", "Gear 2 Incorrect Ratio");
    m.insert("P0733", "Gear 3 Incorrect Ratio");
    m.insert("P0734", "Gear 4 Incorrect Ratio");
    m.insert("P0735", "Gear 5 Incorrect Ratio");
    m.insert("P0740", "Torque Converter Clutch Circuit");
    m.insert(
        "P0741",
        "Torque Converter Clutch Circuit Performance or Stuck Off",
    );
    m.insert("P0742", "Torque Converter Clutch Circuit Stuck On");
    m.insert("P0743", "Torque Converter Clutch Circuit Electrical");
    m.insert("P0744", "Torque Converter Clutch Circuit Intermittent");
    m.insert("P0748", "Pressure Control Solenoid A Electrical");
    m.insert("P0750", "Shift Solenoid A");
    m.insert("P0751", "Shift Solenoid A Performance or Stuck Off");
    m.insert("P0752", "Shift Solenoid A Stuck On");
    m.insert("P0755", "Shift Solenoid B");
    m.insert("P0756", "Shift Solenoid B Performance or Stuck Off");
    m.insert("P0757", "Shift Solenoid B Stuck On");
    m.insert("P0760", "Shift Solenoid C");
    m.insert("P0765", "Shift Solenoid D");
    m.insert("P0770", "Shift Solenoid E");

    // ── Common Body Codes ───────────────────────────────────────────────
    m.insert("B0001", "Driver Frontal Stage 1 Deployment Control");
    m.insert("B0002", "Driver Frontal Stage 2 Deployment Control");
    m.insert("B0100", "Head Lamp - Loss of Communication");

    // ── Common Chassis Codes ────────────────────────────────────────────
    m.insert("C0035", "Left Front Wheel Speed Circuit");
    m.insert("C0040", "Right Front Wheel Speed Circuit");
    m.insert("C0045", "Left Rear Wheel Speed Circuit");
    m.insert("C0050", "Right Rear Wheel Speed Circuit");
    m.insert("C0060", "Left Front ABS Solenoid 1 Circuit");
    m.insert("C0065", "Left Front ABS Solenoid 2 Circuit");
    m.insert("C0070", "Right Front ABS Solenoid 1 Circuit");
    m.insert("C0110", "Pump Motor Circuit");
    m.insert("C0241", "PCM Indicated Traction Control Malfunction");
    m.insert("C0242", "PCM Indicated VDC/ESC Malfunction");

    // ── Common Network Codes ────────────────────────────────────────────
    m.insert("U0001", "High Speed CAN Communication Bus");
    m.insert("U0073", "Control Module Communication Bus A Off");
    m.insert("U0100", "Lost Communication with ECM/PCM A");
    m.insert("U0101", "Lost Communication with TCM");
    m.insert("U0121", "Lost Communication with ABS Control Module");
    m.insert("U0140", "Lost Communication with Body Control Module");
    m.insert("U0155", "Lost Communication with Instrument Panel Cluster");
    m.insert("U0164", "Lost Communication with HVAC Control Module");

    m
});

/// Look up description for a DTC code
pub fn describe(code: &str) -> &'static str {
    DTC_MAP
        .get(code.to_uppercase().as_str())
        .copied()
        .unwrap_or("")
}
