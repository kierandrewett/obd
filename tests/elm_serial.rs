#![cfg(target_os = "linux")]

use obd_dashboard::adapter::DiagnosticAdapter;
use obd_dashboard::elm327::{ElmCanMode, block_on, connect, connect_with_mode};
use std::io::{Read, Write};
use std::os::fd::FromRawFd;

fn open_pty() -> (std::fs::File, std::fs::File, String) {
    let mut master = -1;
    let mut slave = -1;
    // SAFETY: openpty writes two owned descriptors to valid pointers.
    assert_eq!(
        unsafe {
            libc::openpty(
                &mut master,
                &mut slave,
                std::ptr::null_mut(),
                std::ptr::null(),
                std::ptr::null(),
            )
        },
        0
    );
    let mut name = [0i8; 256];
    assert_eq!(
        unsafe { libc::ttyname_r(slave, name.as_mut_ptr(), name.len()) },
        0
    );
    let path = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }
        .to_str()
        .unwrap()
        .to_owned();
    // SAFETY: openpty returned distinct, owned descriptors.
    let master = unsafe { std::fs::File::from_raw_fd(master) };
    // SAFETY: openpty returned distinct, owned descriptors.
    let slave = unsafe { std::fs::File::from_raw_fd(slave) };
    (master, slave, path)
}

fn start_elm_server<F>(
    mut master: std::fs::File,
    mut response_for: F,
    stop_at: &'static str,
) -> std::thread::JoinHandle<Vec<String>>
where
    F: FnMut(&str) -> &'static str + Send + 'static,
{
    std::thread::spawn(move || {
        let mut commands = Vec::new();
        let mut command = Vec::new();
        let mut byte = [0];
        while master.read(&mut byte).unwrap_or(0) > 0 {
            if byte[0] != b'\r' {
                command.push(byte[0]);
                continue;
            }
            let cmd = String::from_utf8(std::mem::take(&mut command)).unwrap();
            commands.push(cmd.clone());
            master
                .write_all(format!("{}\r>", response_for(&cmd)).as_bytes())
                .unwrap();
            if cmd == stop_at {
                // Let the adapter consume the response before closing the PTY.
                std::thread::sleep(std::time::Duration::from_millis(100));
                break;
            }
        }
        commands
    })
}

#[test]
fn branded_elm_adapter_uses_shared_initialisation_over_a_serial_pty() {
    let (master, slave, path) = open_pty();
    let server = start_elm_server(
        master,
        |cmd| match cmd {
            "ATZ" | "ATI" => "OBDLink EX",
            "0100" => "4100BE3FA813",
            "ATDPN" => "A6",
            "ATRV" => "12.5V",
            "010C" => "410C0BB8",
            _ => "OK",
        },
        "010C",
    );
    let mut adapter = connect(&path, Some(38400), None).unwrap();
    assert_eq!(adapter.info.elm_version, "OBDLink EX");
    assert_eq!(
        block_on(adapter.request(&[1, 12], 1000)).unwrap()[0].payload,
        [0x41, 0x0c, 0x0b, 0xb8]
    );
    drop(adapter);
    drop(slave);
    let commands = server.join().unwrap();
    assert!(commands.contains(&"ATSP0".to_string()));
}

#[test]
fn corsa_d_mscan_profile_uses_user_protocol_b_without_hs_fallback_over_serial() {
    let (master, slave, path) = open_pty();
    let server = start_elm_server(
        master,
        |cmd| match cmd {
            "ATZ" | "ATI" => "ELM327 v1.5",
            "0100" => "NO DATA",
            "ATDPN" => "B",
            "ATRV" => "12.4V",
            _ => "OK",
        },
        "ATRV",
    );
    let adapter =
        connect_with_mode(&path, Some(38400), ElmCanMode::CorsaDMediumSpeed, None).unwrap();

    assert!(adapter.info.protocol.contains("95.2 kbit/s"));
    assert!(adapter.info.protocol.contains("no generic OBD responder"));
    drop(adapter);
    drop(slave);
    let commands = server.join().unwrap();
    assert_eq!(commands[5], "AT PB 91 06");
    assert_eq!(commands[6], "ATSPB");
    assert!(commands.contains(&"0100".to_string()));
    assert!(!commands.contains(&"ATSP0".to_string()));
}
