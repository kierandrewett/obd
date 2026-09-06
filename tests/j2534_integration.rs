//! Exercise the real loader and C ABI on Linux without requiring a vendor DLL.
#![cfg(target_os = "linux")]

use obd_dashboard::adapter::{DiagnosticAdapter, request_hex};
use obd_dashboard::app::ObdEvent;
use obd_dashboard::elm327::block_on;
use obd_dashboard::j2534::{CanProtocol, J2534};

#[test]
fn native_driver_connects_reads_faults_and_vin_and_cleans_up_errors() {
    let folder = std::env::temp_dir().join(format!("obd-j2534-test-{}", std::process::id()));
    std::fs::create_dir_all(&folder).unwrap();
    let driver = folder.join("mock.so");
    assert!(
        std::process::Command::new("cc")
            .args([
                "-shared",
                "-fPIC",
                "-Wall",
                "-Wextra",
                "-Werror",
                "tests/fixtures/j2534_driver.c",
                "-o"
            ])
            .arg(&driver)
            .status()
            .unwrap()
            .success()
    );

    // The second handle keeps mock counters available after adapter cleanup.
    let control = unsafe { libloading::Library::new(&driver).unwrap() };
    let scenario = unsafe {
        control
            .get::<unsafe extern "C" fn(u32)>(b"MockScenario\0")
            .unwrap()
    };
    let closes = unsafe {
        control
            .get::<unsafe extern "C" fn() -> u32>(b"MockCloses\0")
            .unwrap()
    };
    let disconnects = unsafe {
        control
            .get::<unsafe extern "C" fn() -> u32>(b"MockDisconnects\0")
            .unwrap()
    };
    for protocol in CanProtocol::ALL {
        let mut adapter = J2534::connect(&driver, protocol).unwrap();
        assert_eq!(block_on(adapter.voltage()).unwrap(), "12.6V");
        let replies = block_on(adapter.request(&[1, 0], 1000)).unwrap();
        assert_eq!(
            replies.len(),
            1,
            "must discard transmit and start indications"
        );
        assert!(replies[0].source.is_some());

        let (tx, rx) = std::sync::mpsc::channel();
        block_on(obd_dashboard::obd_ops::read_vin(&mut adapter, &tx));
        assert!(matches!(rx.recv().unwrap(), ObdEvent::Vin(vin) if vin == "WF0A1234567890123"));
        let (stored, pending) = block_on(obd_dashboard::obd_ops::read_dtcs(&mut adapter, &tx));
        assert_eq!(
            stored
                .iter()
                .map(|dtc| dtc.code.as_str())
                .collect::<Vec<_>>(),
            ["P0133", "U0100"]
        );
        assert!(pending.is_empty());
        assert!(matches!(rx.recv().unwrap(), ObdEvent::DtcResult { .. }));

        unsafe {
            scenario(2);
        }
        assert!(block_on(adapter.request(&[1, 0], 20)).is_err());
        unsafe {
            scenario(3);
        }
        assert!(block_on(adapter.request(&[1, 0], 100)).is_err());
        unsafe {
            scenario(4);
        }
        assert!(block_on(request_hex(&mut adapter, "04", 1000)).is_err());
        block_on(obd_dashboard::obd_ops::clear_dtcs(&mut adapter, &tx));
        assert!(
            matches!(rx.recv().unwrap(), ObdEvent::Error(_)),
            "a rejected clear must never be reported as success"
        );
        unsafe {
            scenario(5);
        }
        assert!(block_on(request_hex(&mut adapter, "0100", 1000)).is_ok());
        unsafe {
            scenario(6);
        }
        assert!(block_on(request_hex(&mut adapter, "0100", 100)).is_err());
        unsafe {
            scenario(0);
        }
    }
    assert_eq!(unsafe { closes() }, 4);
    assert_eq!(unsafe { disconnects() }, 4);
    unsafe {
        scenario(1);
    }
    assert!(J2534::connect(&driver, CanProtocol::default()).is_err());
    assert_eq!(unsafe { closes() }, 5, "filter failure closes device");
    assert_eq!(unsafe { disconnects() }, 5, "filter failure closes channel");
    drop(control);
    std::fs::remove_dir_all(folder).unwrap();
}
