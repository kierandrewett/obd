//! Async OBD-II worker for WASM using the Web Serial API or a local WebSocket
//! emulator connection.

use crate::app::{ObdEvent, OdbCmd, PollConfig};
use crate::elm327::{ConnectionInfo, Elm327Error, ElmAdapter};
use crate::obd;
use crate::obd_ops;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::mpsc;
use wasm_bindgen::closure::Closure;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;

// ── JS helpers ────────────────────────────────────────────────────────────────

fn js_call(obj: &JsValue, method: &str, args: &[JsValue]) -> Result<JsValue, String> {
    let f: js_sys::Function = js_sys::Reflect::get(obj, &JsValue::from_str(method))
        .map_err(|_| format!("no method `{method}`"))?
        .dyn_into()
        .map_err(|_| format!("`{method}` is not a function"))?;
    let result = match args.len() {
        0 => f.call0(obj),
        1 => f.call1(obj, &args[0]),
        2 => f.call2(obj, &args[0], &args[1]),
        _ => return Err(format!("too many args for `{method}`")),
    };
    result.map_err(|e| format!("`{method}()` threw: {e:?}"))
}

fn js_get(obj: &JsValue, key: &str) -> Result<JsValue, String> {
    js_sys::Reflect::get(obj, &JsValue::from_str(key))
        .map_err(|_| format!("no property `{key}`"))
}

fn js_set(obj: &JsValue, key: &str, val: &JsValue) -> Result<(), String> {
    js_sys::Reflect::set(obj, &JsValue::from_str(key), val)
        .map(|_| ())
        .map_err(|_| format!("set `{key}` failed"))
}

async fn js_await(promise_like: JsValue) -> Result<JsValue, String> {
    JsFuture::from(js_sys::Promise::from(promise_like))
        .await
        .map_err(|e| format!("promise rejected: {e:?}"))
}

// ── Web Serial adapter ────────────────────────────────────────────────────────

struct WebElm327 {
    port: JsValue,
    writer: JsValue,
    read_buffer: Rc<RefCell<VecDeque<u8>>>,
    pub info: ConnectionInfo,
}

impl WebElm327 {
    async fn connect(baud: u32, event_tx: &mpsc::Sender<ObdEvent>) -> Result<Self, String> {
        let _ = event_tx.send(ObdEvent::Connecting("Requesting serial port…".into()));

        let window = web_sys::window().ok_or("no window")?;
        let navigator = window.navigator();
        let serial = js_get(&navigator.into(), "serial")
            .map_err(|_| "Web Serial API not available in this browser".to_string())?;

        if serial.is_undefined() || serial.is_null() {
            return Err(
                "Web Serial API is not supported in this browser. \
                 Please use Chrome or Edge."
                    .into(),
            );
        }

        let options = js_sys::Object::new();
        let port = js_await(js_call(&serial, "requestPort", &[options.into()])?)
            .await
            .map_err(|e| format!("Port request cancelled or failed: {e}"))?;

        let _ = event_tx.send(ObdEvent::Connecting("Opening port…".into()));

        let baud_rate = if baud == 0 { 38400 } else { baud };
        let open_opts = js_sys::Object::new();
        js_set(&open_opts.clone().into(), "baudRate", &JsValue::from(baud_rate))?;
        js_await(js_call(&port, "open", &[open_opts.into()])?)
            .await
            .map_err(|e| format!("Could not open port at {baud_rate} baud: {e}"))?;

        let writable = js_get(&port, "writable")?;
        let writer = js_call(&writable, "getWriter", &[])
            .map_err(|e| format!("getWriter failed: {e}"))?;

        let readable = js_get(&port, "readable")?;
        let reader = js_call(&readable, "getReader", &[])
            .map_err(|e| format!("getReader failed: {e}"))?;

        let read_buffer: Rc<RefCell<VecDeque<u8>>> = Rc::new(RefCell::new(VecDeque::new()));
        let buf_clone = read_buffer.clone();
        wasm_bindgen_futures::spawn_local(serial_read_loop(reader, buf_clone));

        let _ = event_tx.send(ObdEvent::Connecting("Initialising ELM327…".into()));

        let mut elm = Self {
            port,
            writer,
            read_buffer,
            info: ConnectionInfo {
                port: "Web Serial".into(),
                baud: baud_rate,
                protocol: "Unknown".into(),
                elm_version: "Unknown".into(),
                voltage: None,
            },
        };

        obd_ops::init_elm(&mut elm, |msg| {
            let _ = event_tx.send(ObdEvent::Connecting(msg.to_string()));
        })
        .await
        .map_err(|e| e.to_string())?;

        Ok(elm)
    }

    async fn send_raw(&mut self, cmd: &str, timeout_ms: u64) -> Result<Vec<String>, String> {
        self.read_buffer.borrow_mut().clear();
        let data = format!("{cmd}\r");
        let array = js_sys::Uint8Array::from(data.as_bytes());
        js_await(js_call(&self.writer, "write", &[array.into()])?)
            .await
            .map_err(|e| format!("write failed: {e}"))?;

        let start = js_sys::Date::now();
        loop {
            if js_sys::Date::now() - start > timeout_ms as f64 {
                return Err(format!("timeout waiting for response to `{cmd}`"));
            }
            let has_prompt = self.read_buffer.borrow().contains(&b'>');
            if has_prompt {
                let bytes: Vec<u8> = self.read_buffer.borrow().iter().copied().collect();
                let text = String::from_utf8_lossy(&bytes).into_owned();
                return Ok(parse_serial_response(cmd, &text));
            }
            gloo_timers::future::TimeoutFuture::new(5).await;
        }
    }

    async fn close(self) {
        let _ = js_call(&self.writer, "releaseLock", &[]);
        if let Ok(readable) = js_get(&self.port, "readable") {
            if let Ok(cancel_promise) = js_call(&readable, "cancel", &[]) {
                let _ = js_await(cancel_promise).await;
            }
        }
        if let Ok(close_promise) = js_call(&self.port, "close", &[]) {
            let _ = js_await(close_promise).await;
        }
    }
}

impl ElmAdapter for WebElm327 {
    async fn send(&mut self, cmd: &str, timeout_ms: u64) -> Result<Vec<String>, Elm327Error> {
        self.send_raw(cmd, timeout_ms).await.map_err(Elm327Error::Serial)
    }
    async fn sleep_ms(&mut self, ms: u64) {
        gloo_timers::future::TimeoutFuture::new(ms as u32).await;
    }
    fn info(&self) -> &ConnectionInfo { &self.info }
    fn info_mut(&mut self) -> &mut ConnectionInfo { &mut self.info }
}

async fn serial_read_loop(reader: JsValue, buffer: Rc<RefCell<VecDeque<u8>>>) {
    loop {
        let result = match JsFuture::from(js_sys::Promise::from(
            match js_call(&reader, "read", &[]) {
                Ok(p) => p,
                Err(_) => break,
            },
        ))
        .await
        {
            Ok(v) => v,
            Err(_) => break,
        };
        if js_get(&result, "done").ok().and_then(|v| v.as_bool()).unwrap_or(false) {
            break;
        }
        if let Ok(value) = js_get(&result, "value") {
            buffer.borrow_mut().extend(js_sys::Uint8Array::from(value).to_vec());
        }
    }
}

fn parse_serial_response(cmd: &str, raw: &str) -> Vec<String> {
    let cmd_upper = cmd.to_uppercase();
    raw.split(['\r', '\n'])
        .map(|s| s.replace('>', "").trim().to_uppercase())
        .filter(|s| !s.is_empty() && s != ">" && s != &cmd_upper)
        .collect()
}

// ── WebSocket adapter (for local emulator) ────────────────────────────────────

struct WsElm327 {
    ws: web_sys::WebSocket,
    responses: Rc<RefCell<VecDeque<String>>>,
    // Keep the closure alive for the lifetime of the connection
    _onmessage: Closure<dyn FnMut(web_sys::MessageEvent)>,
    pub info: ConnectionInfo,
}

impl WsElm327 {
    async fn connect(ws_url: &str, event_tx: &mpsc::Sender<ObdEvent>) -> Result<Self, String> {
        let _ = event_tx.send(ObdEvent::Connecting(format!("Connecting to {ws_url}…")));

        let ws = web_sys::WebSocket::new(ws_url)
            .map_err(|e| format!("WebSocket error: {e:?}"))?;

        let responses: Rc<RefCell<VecDeque<String>>> = Rc::new(RefCell::new(VecDeque::new()));
        let responses_clone = responses.clone();

        let onmessage: Closure<dyn FnMut(web_sys::MessageEvent)> =
            Closure::wrap(Box::new(move |e: web_sys::MessageEvent| {
                if let Some(s) = e.data().as_string() {
                    responses_clone.borrow_mut().push_back(s);
                }
            }));
        ws.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));

        // Wait for WebSocket to open (up to 5 s)
        let deadline = js_sys::Date::now() + 5000.0;
        loop {
            match ws.ready_state() {
                1 => break, // OPEN
                2 | 3 => {
                    // CLOSING / CLOSED
                    return Err(format!("Could not connect to {ws_url} — is obd-emulator running?"));
                }
                _ => {} // CONNECTING (0)
            }
            if js_sys::Date::now() > deadline {
                ws.close().ok();
                return Err(format!("Timeout connecting to {ws_url}"));
            }
            gloo_timers::future::TimeoutFuture::new(50).await;
        }

        let _ = event_tx.send(ObdEvent::Connecting("Initialising ELM327…".into()));

        let mut elm = Self {
            ws,
            responses,
            _onmessage: onmessage,
            info: ConnectionInfo {
                port: ws_url.to_string(),
                baud: 0,
                protocol: "Unknown".into(),
                elm_version: "Unknown".into(),
                voltage: None,
            },
        };

        obd_ops::init_elm(&mut elm, |msg| {
            let _ = event_tx.send(ObdEvent::Connecting(msg.to_string()));
        })
        .await
        .map_err(|e| e.to_string())?;

        Ok(elm)
    }

    async fn send_raw(&mut self, cmd: &str, timeout_ms: u64) -> Result<Vec<String>, String> {
        self.responses.borrow_mut().clear();
        self.ws
            .send_with_str(cmd)
            .map_err(|e| format!("WS send: {e:?}"))?;

        let deadline = js_sys::Date::now() + timeout_ms as f64;
        loop {
            if js_sys::Date::now() > deadline {
                return Err(format!("timeout waiting for response to `{cmd}`"));
            }
            if let Some(resp) = self.responses.borrow_mut().pop_front() {
                // Emulator sends one response per message; parse into lines
                let lines: Vec<String> = resp
                    .split(['\r', '\n'])
                    .map(|s| s.trim().to_uppercase())
                    .filter(|s| !s.is_empty() && s != ">")
                    .collect();
                return Ok(lines);
            }
            gloo_timers::future::TimeoutFuture::new(10).await;
        }
    }

    fn close(self) {
        self.ws.close().ok();
    }
}

impl ElmAdapter for WsElm327 {
    async fn send(&mut self, cmd: &str, timeout_ms: u64) -> Result<Vec<String>, Elm327Error> {
        self.send_raw(cmd, timeout_ms).await.map_err(Elm327Error::Serial)
    }
    async fn sleep_ms(&mut self, ms: u64) {
        gloo_timers::future::TimeoutFuture::new(ms as u32).await;
    }
    fn info(&self) -> &ConnectionInfo { &self.info }
    fn info_mut(&mut self) -> &mut ConnectionInfo { &mut self.info }
}

// ── Unified connection type ───────────────────────────────────────────────────

enum ElmConn {
    Serial(WebElm327),
    Ws(WsElm327),
}

impl ElmConn {
    async fn close(self) {
        match self {
            ElmConn::Serial(e) => e.close().await,
            ElmConn::Ws(e) => e.close(),
        }
    }
}

impl ElmAdapter for ElmConn {
    async fn send(&mut self, cmd: &str, timeout_ms: u64) -> Result<Vec<String>, Elm327Error> {
        match self {
            ElmConn::Serial(e) => e.send(cmd, timeout_ms).await,
            ElmConn::Ws(e) => e.send(cmd, timeout_ms).await,
        }
    }
    async fn sleep_ms(&mut self, ms: u64) {
        match self {
            ElmConn::Serial(e) => e.sleep_ms(ms).await,
            ElmConn::Ws(e) => e.sleep_ms(ms).await,
        }
    }
    fn info(&self) -> &ConnectionInfo {
        match self {
            ElmConn::Serial(e) => e.info(),
            ElmConn::Ws(e) => e.info(),
        }
    }
    fn info_mut(&mut self) -> &mut ConnectionInfo {
        match self {
            ElmConn::Serial(e) => e.info_mut(),
            ElmConn::Ws(e) => e.info_mut(),
        }
    }
}

// ── Main worker loop ──────────────────────────────────────────────────────────

pub async fn run_worker(cmd_rx: mpsc::Receiver<OdbCmd>, event_tx: mpsc::Sender<ObdEvent>) {
    let mut elm: Option<ElmConn> = None;
    let mut live_running = false;
    let mut poll_config = PollConfig::default();
    let pid_defs = obd::mode01_pids();

    loop {
        loop {
            match cmd_rx.try_recv() {
                Ok(cmd) => match cmd {
                    OdbCmd::Connect { baud, .. } => {
                        match WebElm327::connect(baud.unwrap_or(0), &event_tx).await {
                            Ok(e) => {
                                let info = e.info.clone();
                                let voltage = e.info.voltage.clone();
                                elm = Some(ElmConn::Serial(e));
                                let _ = event_tx.send(ObdEvent::Connected(info));
                                if let Some(v) = voltage {
                                    let _ = event_tx.send(ObdEvent::Voltage(v));
                                }
                                if let Some(ref mut e) = elm {
                                    obd_ops::read_vin(e, &event_tx).await;
                                }
                            }
                            Err(e) => {
                                let _ = event_tx.send(ObdEvent::ConnectionFailed(e));
                            }
                        }
                    }
                    OdbCmd::ConnectLocal { ws_port } => {
                        let url = format!("ws://localhost:{ws_port}");
                        match WsElm327::connect(&url, &event_tx).await {
                            Ok(e) => {
                                let info = e.info.clone();
                                let voltage = e.info.voltage.clone();
                                elm = Some(ElmConn::Ws(e));
                                let _ = event_tx.send(ObdEvent::Connected(info));
                                if let Some(v) = voltage {
                                    let _ = event_tx.send(ObdEvent::Voltage(v));
                                }
                                if let Some(ref mut e) = elm {
                                    obd_ops::read_vin(e, &event_tx).await;
                                }
                            }
                            Err(e) => {
                                let _ = event_tx.send(ObdEvent::ConnectionFailed(e));
                            }
                        }
                    }
                    OdbCmd::Disconnect => {
                        if let Some(e) = elm.take() {
                            e.close().await;
                        }
                        live_running = false;
                        let _ = event_tx.send(ObdEvent::Disconnected);
                    }
                    OdbCmd::StartLiveData => live_running = true,
                    OdbCmd::StopLiveData => live_running = false,
                    OdbCmd::ReadDtcs { .. } => {
                        if let Some(ref mut e) = elm {
                            obd_ops::read_dtcs(e, &event_tx, |d| d).await;
                        }
                    }
                    OdbCmd::ClearDtcs => {
                        if let Some(ref mut e) = elm {
                            obd_ops::clear_dtcs(e, &event_tx, |d| d).await;
                        }
                    }
                    OdbCmd::ReadFreezeFrame => {
                        if let Some(ref mut e) = elm {
                            obd_ops::read_freeze_frame(e, &event_tx, &pid_defs).await;
                        }
                    }
                    OdbCmd::ReadVin => {
                        if let Some(ref mut e) = elm {
                            obd_ops::read_vin(e, &event_tx).await;
                        }
                    }
                    OdbCmd::QuerySupportedPids => {
                        if let Some(ref mut e) = elm {
                            obd_ops::query_supported_pids(e, &event_tx).await;
                        }
                    }
                    OdbCmd::SetPollConfig(config) => poll_config = config,
                    OdbCmd::Shutdown => return,
                },
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => return,
            }
        }

        if live_running {
            if let Some(ref mut e) = elm {
                obd_ops::poll_live_data(e, &event_tx, &pid_defs, &poll_config).await;
                if let Ok(v) = e.read_voltage().await {
                    let _ = event_tx.send(ObdEvent::Voltage(v));
                }
                if poll_config.cycle_delay_ms > 0 {
                    gloo_timers::future::TimeoutFuture::new(poll_config.cycle_delay_ms as u32)
                        .await;
                }
            }
        }

        gloo_timers::future::TimeoutFuture::new(10).await;
    }
}
