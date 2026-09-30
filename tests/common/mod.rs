//! Shared deterministic test harness: a controllable [`Backend`] mock.
//!
//! Nothing here is compiled into the production binary; it lives under `tests/`
//! only.

#![allow(dead_code)]

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use axum::body::Body;
use axum::http::Request;
use axum::Router;
use ts_stream::http;
use ts_stream::session::{
    Backend, BackendError, DeviceInfo, OpenRequest, RxEvent, SessionConfig, SessionManager,
    StreamState, TsReceiver,
};

/// Simple one-shot gate used to park `open` or `stop` deterministically.
pub struct Gate {
    open: Mutex<bool>,
    cv: Condvar,
}

impl Gate {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            open: Mutex::new(false),
            cv: Condvar::new(),
        })
    }

    pub fn release(&self) {
        *self.open.lock().unwrap() = true;
        self.cv.notify_all();
    }

    pub fn wait(&self) {
        let mut guard = self.open.lock().unwrap();
        while !*guard {
            guard = self.cv.wait(guard).unwrap();
        }
    }
}

/// One scripted receiver event.
pub enum Step {
    Data(Vec<u8>),
    Pending,
    NoSignal,
    Fatal(String),
    DecoderUnsupported(String),
    Stopped,
    Panic,
}

impl Step {
    fn into_event(self) -> Result<RxEvent, BackendError> {
        match self {
            Step::Data(bytes) => Ok(RxEvent::Data(bytes)),
            Step::Pending => Ok(RxEvent::Pending),
            Step::NoSignal => Ok(RxEvent::NoSignal),
            Step::Fatal(message) => Err(BackendError::Fatal(message)),
            Step::DecoderUnsupported(message) => Err(BackendError::DecoderUnsupported(message)),
            Step::Stopped => Err(BackendError::Stopped),
            Step::Panic => panic!("mock receiver: scripted panic"),
        }
    }
}

/// Shared per-open receiver state, also held by the test for live control.
pub struct MockReceiverState {
    steps: Mutex<VecDeque<Step>>,
    cv: Condvar,
    recv_count: AtomicU64,
    stop_count: AtomicU64,
    stopped: AtomicBool,
    stop_gate: Mutex<Option<Arc<Gate>>>,
    drop_gate: Mutex<Option<Arc<Gate>>>,
    drop_count: AtomicU64,
    dropped: AtomicBool,
}

impl MockReceiverState {
    fn new(steps: Vec<Step>) -> Self {
        Self {
            steps: Mutex::new(steps.into()),
            cv: Condvar::new(),
            recv_count: AtomicU64::new(0),
            stop_count: AtomicU64::new(0),
            stopped: AtomicBool::new(false),
            stop_gate: Mutex::new(None),
            drop_gate: Mutex::new(None),
            drop_count: AtomicU64::new(0),
            dropped: AtomicBool::new(false),
        }
    }

    /// Append a step and wake a parked `recv_timeout`.
    pub fn push(&self, step: Step) {
        self.steps.lock().unwrap().push_back(step);
        self.cv.notify_all();
    }

    /// Make `stop()` block until `gate` is released.
    pub fn set_stop_gate(&self, gate: Arc<Gate>) {
        *self.stop_gate.lock().unwrap() = Some(gate);
    }

    /// Make the receiver's destructor block until `gate` is released.
    pub fn set_drop_gate(&self, gate: Arc<Gate>) {
        *self.drop_gate.lock().unwrap() = Some(gate);
    }

    pub fn recv_calls(&self) -> u64 {
        self.recv_count.load(Ordering::SeqCst)
    }

    pub fn stop_calls(&self) -> u64 {
        self.stop_count.load(Ordering::SeqCst)
    }

    pub fn is_stopped(&self) -> bool {
        self.stopped.load(Ordering::SeqCst)
    }

    /// Number of times the receiver destructor has been entered.
    pub fn drop_calls(&self) -> u64 {
        self.drop_count.load(Ordering::SeqCst)
    }

    /// Whether the receiver destructor has completed.
    pub fn is_dropped(&self) -> bool {
        self.dropped.load(Ordering::SeqCst)
    }
}

pub struct MockReceiver {
    state: Arc<MockReceiverState>,
}

impl TsReceiver for MockReceiver {
    fn recv_timeout(&mut self, timeout: Duration) -> Result<RxEvent, BackendError> {
        self.state.recv_count.fetch_add(1, Ordering::SeqCst);
        let deadline = Instant::now() + timeout;
        let mut steps = self.state.steps.lock().unwrap();
        loop {
            if let Some(step) = steps.pop_front() {
                return step.into_event();
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(RxEvent::NoSignal);
            }
            let (guard, _) = self
                .state
                .cv
                .wait_timeout(steps, deadline - now)
                .expect("mock condvar not poisoned");
            steps = guard;
        }
    }

    fn stop(&mut self) -> Result<(), BackendError> {
        self.state.stop_count.fetch_add(1, Ordering::SeqCst);
        if let Some(gate) = self.state.stop_gate.lock().unwrap().clone() {
            gate.wait();
        }
        self.state.stopped.store(true, Ordering::SeqCst);
        Ok(())
    }
}

impl Drop for MockReceiver {
    fn drop(&mut self) {
        self.state.drop_count.fetch_add(1, Ordering::SeqCst);
        if let Some(gate) = self.state.drop_gate.lock().unwrap().clone() {
            gate.wait();
        }
        self.state.dropped.store(true, Ordering::SeqCst);
    }
}

/// How the mock should react to the next `open`.
pub enum OpenBehavior {
    /// Succeed with a receiver pre-loaded with `steps`.
    Receiver(Vec<Step>),
    Unavailable(String),
    Unsupported(String),
}

struct MockBackendInner {
    behaviors: Mutex<VecDeque<OpenBehavior>>,
    opened: Mutex<VecDeque<Arc<MockReceiverState>>>,
    open_gate: Mutex<Option<Arc<Gate>>>,
    stop_gate_on_open: Mutex<Option<Arc<Gate>>>,
    drop_gate_on_open: Mutex<Option<Arc<Gate>>>,
    devices: Mutex<Result<Vec<DeviceInfo>, BackendError>>,
}

/// Deterministic [`Backend`] implementation.
#[derive(Clone)]
pub struct MockBackend {
    inner: Arc<MockBackendInner>,
}

impl Default for MockBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl MockBackend {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(MockBackendInner {
                behaviors: Mutex::new(VecDeque::new()),
                opened: Mutex::new(VecDeque::new()),
                open_gate: Mutex::new(None),
                stop_gate_on_open: Mutex::new(None),
                drop_gate_on_open: Mutex::new(None),
                devices: Mutex::new(Ok(vec![DeviceInfo {
                    index: 0,
                    name: "Mock RTL-SDR".to_string(),
                    serial: Some("00000001".to_string()),
                }])),
            }),
        }
    }

    pub fn push_behavior(&self, behavior: OpenBehavior) {
        self.inner.behaviors.lock().unwrap().push_back(behavior);
    }

    pub fn park_open(&self, gate: Arc<Gate>) {
        *self.inner.open_gate.lock().unwrap() = Some(gate);
    }

    pub fn park_stop_on_open(&self, gate: Arc<Gate>) {
        *self.inner.stop_gate_on_open.lock().unwrap() = Some(gate);
    }

    /// Park the receiver destructor of every newly opened receiver on `gate`.
    pub fn park_drop_on_open(&self, gate: Arc<Gate>) {
        *self.inner.drop_gate_on_open.lock().unwrap() = Some(gate);
    }

    pub fn set_devices(&self, devices: Result<Vec<DeviceInfo>, BackendError>) {
        *self.inner.devices.lock().unwrap() = devices;
    }

    /// Wait until a receiver has been handed out and return its control handle.
    pub async fn wait_for_receiver(&self, timeout: Duration) -> Arc<MockReceiverState> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(state) = self.inner.opened.lock().unwrap().front().cloned() {
                return state;
            }
            assert!(
                Instant::now() < deadline,
                "timed out waiting for the mock backend to open a receiver"
            );
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }
}

impl Backend for MockBackend {
    type Receiver = MockReceiver;

    fn list_devices(&self) -> Result<Vec<DeviceInfo>, BackendError> {
        self.inner.devices.lock().unwrap().clone()
    }

    fn open(&self, _request: OpenRequest) -> Result<MockReceiver, BackendError> {
        if let Some(gate) = self.inner.open_gate.lock().unwrap().clone() {
            gate.wait();
        }
        let behavior = self
            .inner
            .behaviors
            .lock()
            .unwrap()
            .pop_front()
            .unwrap_or(OpenBehavior::Receiver(Vec::new()));
        match behavior {
            OpenBehavior::Unavailable(message) => Err(BackendError::Unavailable(message)),
            OpenBehavior::Unsupported(message) => Err(BackendError::DecoderUnsupported(message)),
            OpenBehavior::Receiver(steps) => {
                let state = Arc::new(MockReceiverState::new(steps));
                if let Some(gate) = self.inner.stop_gate_on_open.lock().unwrap().clone() {
                    state.set_stop_gate(gate);
                }
                if let Some(gate) = self.inner.drop_gate_on_open.lock().unwrap().clone() {
                    state.set_drop_gate(gate);
                }
                self.inner
                    .opened
                    .lock()
                    .unwrap()
                    .push_back(Arc::clone(&state));
                Ok(MockReceiver { state })
            }
        }
    }
}

/// Fast, deterministic session configuration for tests.
pub fn test_config() -> SessionConfig {
    SessionConfig {
        device_index: 0,
        gain: None,
        ppm: 0,
        sample_rate: 1_200_000,
        initial_lock_timeout: Duration::from_millis(300),
        reacquire_timeout: Duration::from_millis(300),
        queue_capacity: 8,
        poll_interval: Duration::from_millis(10),
    }
}

/// Build a router over an existing mock backend.
pub fn router_for(
    backend: MockBackend,
    config: SessionConfig,
) -> (Router, SessionManager<MockBackend>) {
    let manager = SessionManager::new(Arc::new(backend), config);
    (http::build_router(manager.clone()), manager)
}

pub fn stream_request(channel: u32) -> Request<Body> {
    Request::builder()
        .uri(format!("/stream?ch={channel}"))
        .body(Body::empty())
        .unwrap()
}

pub fn version_request() -> Request<Body> {
    Request::builder()
        .uri("/version")
        .body(Body::empty())
        .unwrap()
}

/// Await a lifecycle state, panicking on timeout.
pub async fn wait_for_state(
    manager: &SessionManager<MockBackend>,
    wanted: StreamState,
    timeout: Duration,
) {
    let deadline = Instant::now() + timeout;
    while manager.state() != wanted {
        assert!(
            Instant::now() < deadline,
            "timed out waiting for state {:?} (currently {:?})",
            wanted,
            manager.state()
        );
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}
