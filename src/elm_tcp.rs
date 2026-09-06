//! Native TCP transport for Wi-Fi/Ethernet ELM-compatible adapters.

use crate::elm327::{ConnectionInfo, Elm327Error, ElmAdapter};
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

pub struct TcpElm {
    stream: TcpStream,
    info: ConnectionInfo,
}

impl TcpElm {
    pub fn connect(address: &str, progress: impl Fn(&str)) -> Result<Self, Elm327Error> {
        let addresses = address
            .to_socket_addrs()
            .map_err(|e| Elm327Error::Serial(e.to_string()))?;
        let mut last_error = "Address did not resolve".to_string();
        for address in addresses {
            match TcpStream::connect_timeout(&address, Duration::from_secs(3)) {
                Ok(stream) => {
                    stream
                        .set_write_timeout(Some(Duration::from_secs(2)))
                        .map_err(|e| Elm327Error::Serial(e.to_string()))?;
                    let mut adapter = Self {
                        stream,
                        info: ConnectionInfo {
                            port: format!("tcp://{address}"),
                            baud: 0,
                            protocol: String::new(),
                            elm_version: String::new(),
                            voltage: None,
                        },
                    };
                    crate::elm327::block_on(crate::obd_ops::init_elm(&mut adapter, progress))?;
                    return Ok(adapter);
                }
                Err(error) => last_error = error.to_string(),
            }
        }
        Err(Elm327Error::Serial(format!("TCP {address}: {last_error}")))
    }
}

impl ElmAdapter for TcpElm {
    async fn send(&mut self, cmd: &str, timeout_ms: u64) -> Result<Vec<String>, Elm327Error> {
        if cmd.contains(['\r', '\n']) {
            return Err(Elm327Error::ProtocolError(
                "One ELM command required".into(),
            ));
        }
        self.stream
            .write_all(format!("{cmd}\r").as_bytes())
            .map_err(|e| Elm327Error::Serial(e.to_string()))?;
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        let mut data = Vec::new();
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(Elm327Error::Timeout(format!("TCP response to {cmd}")));
            }
            self.stream
                .set_read_timeout(Some(remaining))
                .map_err(|e| Elm327Error::Serial(e.to_string()))?;
            // Read through exactly one prompt, without consuming the next response.
            let mut byte = [0];
            match self.stream.read(&mut byte) {
                Ok(0) => return Err(Elm327Error::Serial("TCP adapter disconnected".into())),
                Ok(_) if byte[0] == b'>' => break,
                Ok(_) => data.push(byte[0]),
                Err(error)
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                    ) =>
                {
                    return Err(Elm327Error::Timeout(format!("TCP response to {cmd}")));
                }
                Err(error) => return Err(Elm327Error::Serial(error.to_string())),
            }
            if data.len() > 65536 {
                return Err(Elm327Error::ProtocolError(
                    "ELM response exceeds 64 KiB".into(),
                ));
            }
        }
        Ok(String::from_utf8_lossy(&data)
            .split(['\r', '\n'])
            .map(str::trim)
            .filter(|line| !line.is_empty() && !line.eq_ignore_ascii_case(cmd))
            .map(str::to_owned)
            .collect())
    }

    fn info(&self) -> &ConnectionInfo {
        &self.info
    }
    fn info_mut(&mut self) -> &mut ConnectionInfo {
        &mut self.info
    }
    async fn sleep_ms(&mut self, ms: u64) {
        std::thread::sleep(Duration::from_millis(ms));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::DiagnosticAdapter;

    #[test]
    fn connects_and_polls_over_a_real_tcp_socket() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let server = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut command = Vec::new();
            let mut byte = [0];
            while stream.read(&mut byte).unwrap_or(0) > 0 {
                if byte[0] != b'\r' {
                    command.push(byte[0]);
                    continue;
                }
                let cmd = String::from_utf8(std::mem::take(&mut command)).unwrap();
                let reply = match cmd.as_str() {
                    "ATZ" | "ATI" => "OBDLink MX+",
                    "0100" => "4100BE3FA813",
                    "ATDPN" => "A6",
                    "ATRV" => "12.6V",
                    "010C" => "410C1AF8",
                    _ => "OK",
                };
                stream.write_all(format!("{reply}\r>").as_bytes()).unwrap();
            }
        });
        let mut adapter = TcpElm::connect(&address.to_string(), |_| {}).unwrap();
        let replies = crate::elm327::block_on(adapter.request(&[1, 12], 1000)).unwrap();
        assert_eq!(replies[0].payload, [0x41, 0x0c, 0x1a, 0xf8]);
        drop(adapter);
        server.join().unwrap();
    }
}
