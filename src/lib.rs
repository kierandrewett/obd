pub mod app;
pub mod dtc_database;
pub mod dtc_descriptions;
pub mod elm327;
pub mod gauges;
pub mod obd;
pub mod obd_ops;
pub mod vin_decoder;

#[cfg(target_arch = "wasm32")]
mod web_serial;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

/// WASM entry point — called automatically by the browser via wasm-bindgen.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub fn start() {
    use std::sync::mpsc;

    // Better panic messages in the browser console.
    console_error_panic_hook::set_once();

    // Forward tracing events to the browser console.
    tracing_wasm::set_as_global_default();

    let (cmd_tx, cmd_rx) = mpsc::channel::<app::OdbCmd>();
    let (event_tx, event_rx) = mpsc::channel::<app::ObdEvent>();

    // Spawn the async Web Serial OBD worker.
    wasm_bindgen_futures::spawn_local(web_serial::run_worker(cmd_rx, event_tx));

    // Run the eframe web app.
    let cmd_tx_clone = cmd_tx.clone();
    wasm_bindgen_futures::spawn_local(async move {
        use eframe::wasm_bindgen::JsCast as _;

        let window = web_sys::window().expect("no window");
        let document = window.document().expect("no document");
        let canvas = document
            .get_element_by_id("canvas")
            .expect("element #canvas not found")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("#canvas is not a <canvas>");

        // Size the canvas buffer to physical pixels so the WebGL framebuffer
        // is full-resolution on HiDPI/Retina displays.  eframe reads
        // canvas.width / canvas.height for the framebuffer size, so we must
        // set these *before* WebRunner::start().
        let ppp = window.device_pixel_ratio() as f32;
        let css_w = window
            .inner_width()
            .ok()
            .and_then(|v| v.as_f64())
            .unwrap_or(800.0) as f32;
        let css_h = window
            .inner_height()
            .ok()
            .and_then(|v| v.as_f64())
            .unwrap_or(600.0) as f32;
        canvas.set_width((css_w * ppp).round() as u32);
        canvas.set_height((css_h * ppp).round() as u32);

        eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(move |cc| {
                    // Match egui's logical pixel density to the display's device pixel ratio
                    // so rendering is sharp on HiDPI / Retina screens.
                    let ppp = web_sys::window()
                        .map(|w| w.device_pixel_ratio() as f32)
                        .unwrap_or(1.0);
                    cc.egui_ctx.set_pixels_per_point(ppp);

                    Ok(Box::new(app::ObdApp::new(cc, cmd_tx_clone, event_rx)))
                }),
            )
            .await
            .expect("failed to start eframe");
    });
}
