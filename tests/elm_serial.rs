#![cfg(target_os = "linux")]

use obd_dashboard::adapter::DiagnosticAdapter;
use obd_dashboard::elm327::{block_on, connect};
use std::io::{Read, Write};
use std::os::fd::FromRawFd;

#[test]
fn branded_elm_adapter_uses_shared_initialisation_over_a_serial_pty() {
    let mut master = 0;
    let mut slave = 0;
    // SAFETY: openpty writes two owned descriptors to valid pointers.
    assert_eq!(unsafe { libc::openpty(&mut master, &mut slave, std::ptr::null_mut(), std::ptr::null(), std::ptr::null()) }, 0);
    let mut name = [0i8; 256];
    assert_eq!(unsafe { libc::ttyname_r(slave, name.as_mut_ptr(), name.len()) }, 0);
    let path = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }.to_str().unwrap().to_owned();
    let mut master = unsafe { std::fs::File::from_raw_fd(master) };
    let slave = unsafe { std::fs::File::from_raw_fd(slave) };
    let server = std::thread::spawn(move || {
        let mut command = Vec::new();
        let mut byte = [0];
        while master.read(&mut byte).unwrap_or(0) > 0 {
            if byte[0] != b'\r' { command.push(byte[0]); continue; }
            let cmd = String::from_utf8(std::mem::take(&mut command)).unwrap();
            let response = match cmd.as_str() {
                "ATZ" | "ATI" => "OBDLink EX",
                "0100" => "4100BE3FA813",
                "ATDPN" => "A6",
                "ATRV" => "12.5V",
                "010C" => "410C0BB8",
                _ => "OK",
            };
            master.write_all(format!("{response}\r>").as_bytes()).unwrap();
            if cmd == "010C" {
                // Wait for the adapter to consume the response before closing
                // the PTY master, which would discard the unread slave buffer.
                std::thread::sleep(std::time::Duration::from_millis(100));
                break;
            }
        }
    });
    let mut adapter = connect(&path, Some(38400), None).unwrap();
    assert_eq!(adapter.info.elm_version, "OBDLink EX");
    assert_eq!(block_on(adapter.request(&[1, 12], 1000)).unwrap()[0].payload, [0x41, 0x0c, 0x0b, 0xb8]);
    drop(adapter);
    drop(slave);
    server.join().unwrap();
}
