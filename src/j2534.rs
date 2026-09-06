//! J2534 04.04 ISO 15765 backend. Uses the Windows ABI (32-bit ULONG).
//! Vendor drivers must match the process architecture. No OEM DLLs are bundled.

use crate::adapter::{DiagnosticAdapter, DiagnosticResponse};
use crate::elm327::{ConnectionInfo, Elm327Error};
use libloading::Library;
use std::ffi::{c_char, c_void};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const ISO15765: u32 = 6;
const CAN_29BIT_ID: u32 = 0x100;
const FRAME_PAD: u32 = 0x40;
const ERR_TIMEOUT: u32 = 9;
const ERR_BUFFER_EMPTY: u32 = 0x10;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CanProtocol {
    #[default]
    Can11Bit500K,
    Can29Bit500K,
    Can11Bit250K,
    Can29Bit250K,
}

impl CanProtocol {
    pub const ALL: [Self; 4] = [
        Self::Can11Bit500K,
        Self::Can29Bit500K,
        Self::Can11Bit250K,
        Self::Can29Bit250K,
    ];
    pub fn label(self) -> &'static str {
        match self {
            Self::Can11Bit500K => "CAN 11-bit / 500 kbit/s",
            Self::Can29Bit500K => "CAN 29-bit / 500 kbit/s",
            Self::Can11Bit250K => "CAN 11-bit / 250 kbit/s",
            Self::Can29Bit250K => "CAN 29-bit / 250 kbit/s",
        }
    }
    pub fn extended(self) -> bool {
        matches!(self, Self::Can29Bit500K | Self::Can29Bit250K)
    }
    fn baud(self) -> u32 {
        if matches!(self, Self::Can11Bit250K | Self::Can29Bit250K) {
            250000
        } else {
            500000
        }
    }
    fn flags(self) -> u32 {
        if self.extended() { CAN_29BIT_ID } else { 0 }
    }
    fn request_id(self) -> u32 {
        if self.extended() { 0x18DB33F1 } else { 0x7DF }
    }
    fn addresses(self, ecu: u32) -> (u32, u32) {
        if self.extended() {
            (0x18DAF100 | ecu, 0x18DA00F1 | (ecu << 8))
        } else {
            (0x7E8 + ecu, 0x7E0 + ecu)
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DriverInfo {
    pub name: String,
    pub path: PathBuf,
}

/// Enumerate the matching Windows registry view. Does not load driver code.
pub fn discover_drivers() -> Vec<DriverInfo> {
    #[cfg(windows)]
    {
        let view = if cfg!(target_pointer_width = "64") {
            "/reg:64"
        } else {
            "/reg:32"
        };
        let output = std::process::Command::new("reg.exe")
            .args(["query", r"HKLM\SOFTWARE\PassThruSupport.04.04", "/s", view])
            .output();
        if let Ok(output) = output {
            if output.status.success() {
                return parse_registry(&String::from_utf8_lossy(&output.stdout));
            }
        }
    }
    Vec::new()
}

#[cfg(any(windows, test))]
fn parse_registry(text: &str) -> Vec<DriverInfo> {
    let mut drivers = Vec::new();
    let mut name = String::new();
    let mut path = None;
    let flush = |drivers: &mut Vec<DriverInfo>, name: &str, path: &mut Option<PathBuf>| {
        if let Some(path) = path.take() {
            drivers.push(DriverInfo {
                name: name.to_string(),
                path,
            });
        }
    };
    for line in text.lines() {
        let line = line.trim();
        if line.starts_with("HKEY_") {
            flush(&mut drivers, &name, &mut path);
            name = line.rsplit('\\').next().unwrap_or(line).to_string();
        } else if let Some((key, value)) = line.split_once("REG_SZ") {
            match key.trim() {
                "Name" => name = value.trim().to_string(),
                "FunctionLibrary" => path = Some(PathBuf::from(value.trim())),
                _ => {}
            }
        }
    }
    flush(&mut drivers, &name, &mut path);
    drivers.sort_by(|a, b| a.name.cmp(&b.name));
    drivers.dedup_by(|a, b| a.path == b.path);
    drivers
}

#[repr(C)]
struct Message {
    protocol: u32,
    rx_status: u32,
    tx_flags: u32,
    timestamp: u32,
    data_size: u32,
    extra_data_index: u32,
    data: [u8; 4128],
}

impl Message {
    fn new(id: u32, payload: &[u8], flags: u32) -> Result<Self, Elm327Error> {
        if payload.len() > 4095 {
            return Err(error("ISO-TP payload exceeds 4095 bytes"));
        }
        let mut message = Self {
            protocol: ISO15765,
            rx_status: 0,
            tx_flags: flags,
            timestamp: 0,
            data_size: (4 + payload.len()) as u32,
            extra_data_index: 0,
            data: [0; 4128],
        };
        message.data[..4].copy_from_slice(&id.to_be_bytes());
        message.data[4..4 + payload.len()].copy_from_slice(payload);
        Ok(message)
    }

    fn response(&self, protocol: CanProtocol) -> Result<Option<DiagnosticResponse>, Elm327Error> {
        // Ignore transmit echoes, start indications and transmit-complete indications.
        if self.rx_status & (0x01 | 0x02 | 0x08) != 0 {
            return Ok(None);
        }
        if self.rx_status & 0x10 != 0 {
            return Err(error("Driver reported an ISO-TP padding error"));
        }
        if self.protocol != ISO15765 {
            return Err(error("Driver returned the wrong protocol"));
        }
        if !(5..=4128).contains(&self.data_size) {
            return Err(error("Driver returned an invalid message length"));
        }
        let id = u32::from_be_bytes(self.data[..4].try_into().unwrap());
        let valid = if protocol.extended() {
            id & 0x1FFFFF00 == 0x18DAF100
        } else {
            (0x7E8..=0x7EF).contains(&id)
        };
        if !valid {
            return Ok(None);
        }
        let end = if self.extra_data_index == 0 {
            self.data_size
        } else {
            self.extra_data_index
        };
        if end < 5 || end > self.data_size {
            return Err(error("Driver returned invalid extra-data index"));
        }
        Ok(Some(DiagnosticResponse {
            source: Some(id),
            payload: self.data[4..end as usize].to_vec(),
        }))
    }
}

type Open = unsafe extern "system" fn(*const c_void, *mut u32) -> u32;
type Close = unsafe extern "system" fn(u32) -> u32;
type Connect = unsafe extern "system" fn(u32, u32, u32, u32, *mut u32) -> u32;
type Messages = unsafe extern "system" fn(u32, *mut Message, *mut u32, u32) -> u32;
type Filter =
    unsafe extern "system" fn(u32, u32, *mut Message, *mut Message, *mut Message, *mut u32) -> u32;
type Ioctl = unsafe extern "system" fn(u32, u32, *mut c_void, *mut c_void) -> u32;
type LastError = unsafe extern "system" fn(*mut c_char) -> u32;

struct Api {
    open: Open,
    close: Close,
    connect: Connect,
    disconnect: Close,
    read: Messages,
    write: Messages,
    filter: Filter,
    ioctl: Ioctl,
    last_error: LastError,
    // Kept alive until all device/channel cleanup has finished.
    _library: Library,
}

impl Api {
    fn load(path: &Path) -> Result<Self, Elm327Error> {
        if !path.is_absolute() {
            return Err(error(
                "Select an absolute path to the vendor driver library",
            ));
        }
        // SAFETY: the user selects an installed native J2534 04.04 library. All
        // function pointers use its documented Windows ABI and remain owned here.
        unsafe {
            let library = Library::new(path).map_err(|e| {
                error(&format!(
                    "Cannot load {}: {e}. Driver and app must both be {}-bit",
                    path.display(),
                    usize::BITS
                ))
            })?;
            macro_rules! symbol {
                ($name:literal) => {
                    *library
                        .get(concat!($name, "\0").as_bytes())
                        .map_err(|e| error(&format!("Missing J2534 04.04 export: {e}")))?
                };
            }
            Ok(Self {
                open: symbol!("PassThruOpen"),
                close: symbol!("PassThruClose"),
                connect: symbol!("PassThruConnect"),
                disconnect: symbol!("PassThruDisconnect"),
                read: symbol!("PassThruReadMsgs"),
                write: symbol!("PassThruWriteMsgs"),
                filter: symbol!("PassThruStartMsgFilter"),
                ioctl: symbol!("PassThruIoctl"),
                last_error: symbol!("PassThruGetLastError"),
                _library: library,
            })
        }
    }

    fn check(&self, status: u32, operation: &str) -> Result<(), Elm327Error> {
        if status == 0 {
            return Ok(());
        }
        let mut text = [0u8; 256];
        // SAFETY: J2534 requires a 256-byte error buffer.
        unsafe {
            (self.last_error)(text.as_mut_ptr().cast());
        }
        let end = text.iter().position(|b| *b == 0).unwrap_or(text.len());
        Err(error(&format!(
            "{operation} failed (0x{status:02X}): {}",
            String::from_utf8_lossy(&text[..end])
        )))
    }
}

fn error(message: &str) -> Elm327Error {
    Elm327Error::ProtocolError(format!("J2534: {message}"))
}

pub struct J2534 {
    api: Api,
    device: u32,
    channel: Option<u32>,
    protocol: CanProtocol,
    ecu_address: u8,
    info: ConnectionInfo,
}

impl J2534 {
    /// Open one explicitly selected OBD CAN protocol. A successful 0100 response
    /// is required before the GUI reports a vehicle connection.
    pub fn connect(path: &Path, protocol: CanProtocol) -> Result<Self, Elm327Error> {
        Self::connect_to(path, protocol, 0x10)
    }

    /// For 29-bit OBD, select the ECU source address explicitly. Unlike 11-bit
    /// OBD, responders are not confined to eight consecutive CAN identifiers.
    pub fn connect_to(
        path: &Path,
        protocol: CanProtocol,
        ecu_address: u8,
    ) -> Result<Self, Elm327Error> {
        let api = Api::load(path)?;
        let mut device = 0;
        // SAFETY: output pointers are valid and calls are serialised on the worker.
        api.check(
            unsafe { (api.open)(std::ptr::null(), &mut device) },
            "PassThruOpen",
        )?;
        let mut adapter = Self {
            api,
            device,
            channel: None,
            protocol,
            ecu_address,
            info: ConnectionInfo {
                port: path
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into_owned(),
                baud: 0,
                protocol: protocol.label().into(),
                elm_version: "J2534 04.04".into(),
                voltage: None,
            },
        };
        let mut channel = 0;
        adapter.api.check(
            unsafe {
                (adapter.api.connect)(
                    device,
                    ISO15765,
                    protocol.flags(),
                    protocol.baud(),
                    &mut channel,
                )
            },
            "PassThruConnect",
        )?;
        adapter.channel = Some(channel);
        // Standard OBD ECU address range. This is not an enhanced module scan.
        let ecu_addresses: Vec<u32> = if protocol.extended() {
            vec![u32::from(ecu_address)]
        } else {
            (0..8).collect()
        };
        for ecu in ecu_addresses {
            let (response_id, flow_id) = protocol.addresses(ecu);
            let mut mask = Message::new(
                if protocol.extended() {
                    0x1FFFFFFF
                } else {
                    0x7FF
                },
                &[],
                protocol.flags(),
            )?;
            let mut pattern = Message::new(response_id, &[], protocol.flags())?;
            let mut flow = Message::new(flow_id, &[], protocol.flags() | FRAME_PAD)?;
            let mut filter_id = 0;
            adapter.api.check(
                unsafe {
                    (adapter.api.filter)(
                        channel,
                        3,
                        &mut mask,
                        &mut pattern,
                        &mut flow,
                        &mut filter_id,
                    )
                },
                "PassThruStartMsgFilter",
            )?;
        }
        let replies = crate::elm327::block_on(adapter.request(&[1, 0], 3000))?;
        if !replies
            .iter()
            .any(|reply| reply.payload.starts_with(&[0x41, 0]) && reply.payload.len() >= 6)
        {
            return Err(error(
                "Vehicle did not answer supported-PID request; check ignition and CAN selection",
            ));
        }
        adapter.info.voltage = crate::elm327::block_on(adapter.voltage()).ok();
        Ok(adapter)
    }
}

impl DiagnosticAdapter for J2534 {
    async fn request(
        &mut self,
        payload: &[u8],
        timeout_ms: u64,
    ) -> Result<Vec<DiagnosticResponse>, Elm327Error> {
        if payload.is_empty() || timeout_ms == 0 {
            return Err(error("Request and timeout must be non-empty"));
        }
        let channel = self.channel.ok_or_else(|| error("No open channel"))?;
        let mut message = Message::new(
            self.protocol.request_id(),
            payload,
            self.protocol.flags() | FRAME_PAD,
        )?;
        // SAFETY: all buffers are owned, sized to the 04.04 ABI, and no other
        // thread accesses this driver. Clear stale replies before each request.
        self.api.check(
            unsafe { (self.api.ioctl)(channel, 8, std::ptr::null_mut(), std::ptr::null_mut()) },
            "CLEAR_RX_BUFFER",
        )?;
        let mut count = 1;
        self.api.check(
            unsafe { (self.api.write)(channel, &mut message, &mut count, 1000) },
            "PassThruWriteMsgs",
        )?;
        if count != 1 {
            return Err(error("Driver did not transmit the request"));
        }
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        let mut replies = Vec::new();
        let mut last_reply = None;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            let mut message = Message::new(0, &[], 0)?;
            let mut count = 1;
            let wait = remaining.as_millis().clamp(1, 50) as u32;
            let status = unsafe { (self.api.read)(channel, &mut message, &mut count, wait) };
            if status != ERR_TIMEOUT && status != ERR_BUFFER_EMPTY {
                self.api.check(status, "PassThruReadMsgs")?;
            }
            if count > 1 {
                return Err(error("Driver returned an invalid message count"));
            }
            if count == 1 {
                if let Some(reply) = message.response(self.protocol)? {
                    if self.protocol.extended()
                        && reply.source
                            != Some(self.protocol.addresses(u32::from(self.ecu_address)).0)
                    {
                        continue;
                    }
                    if reply.payload.first() == Some(&(payload[0].wrapping_add(0x40)))
                        || reply.payload.starts_with(&[0x7F, payload[0]])
                    {
                        replies.push(reply);
                        last_reply = Some(Instant::now());
                    }
                }
            }
            if last_reply.is_some_and(|time| time.elapsed() >= Duration::from_millis(150)) {
                break;
            }
            if count == 0 {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        if replies.is_empty() {
            return Err(Elm327Error::Timeout("J2534: no vehicle response".into()));
        }
        Ok(replies)
    }

    fn connection_info(&self) -> &ConnectionInfo {
        &self.info
    }
    async fn voltage(&mut self) -> Result<String, Elm327Error> {
        let mut millivolts = 0u32;
        self.api.check(
            unsafe {
                (self.api.ioctl)(
                    self.device,
                    3,
                    std::ptr::null_mut(),
                    (&mut millivolts as *mut u32).cast(),
                )
            },
            "READ_VBATT",
        )?;
        Ok(format!("{:.1}V", millivolts as f64 / 1000.0))
    }
    async fn delay(&mut self, ms: u64) {
        std::thread::sleep(Duration::from_millis(ms));
    }
}

impl Drop for J2534 {
    fn drop(&mut self) {
        // Disconnect releases filters too. Library remains loaded through cleanup.
        unsafe {
            if let Some(channel) = self.channel.take() {
                (self.api.disconnect)(channel);
            }
            (self.api.close)(self.device);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_driver_paths_with_spaces() {
        let registry = "HKEY_LOCAL_MACHINE\\SOFTWARE\\PassThruSupport.04.04\\Example\n    FunctionLibrary    REG_SZ    C:\\Program Files\\Vendor\\driver.dll\n    Name    REG_SZ    Example VCI\n";
        let drivers = parse_registry(registry);
        assert_eq!(drivers[0].name, "Example VCI");
        assert_eq!(
            drivers[0].path,
            PathBuf::from(r"C:\Program Files\Vendor\driver.dll")
        );
    }

    #[test]
    fn validates_driver_message_lengths_and_indications() {
        assert_eq!(std::mem::size_of::<Message>(), 4152);
        let mut message = Message::new(0x7E8, &[0x41, 0x0c, 0x1a, 0xf8], 0).unwrap();
        assert_eq!(
            message
                .response(CanProtocol::default())
                .unwrap()
                .unwrap()
                .source,
            Some(0x7E8)
        );
        message.rx_status = 2;
        assert!(message.response(CanProtocol::default()).unwrap().is_none());
        message.rx_status = 0;
        message.data_size = 5000;
        assert!(message.response(CanProtocol::default()).is_err());
    }
}
