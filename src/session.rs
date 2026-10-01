//! Single-slot stream session: atomic reservation, state machine, blocking
//! receiver worker and bounded TS queue.
//!
//! # Exclusivity contract
//!
//! * Exactly one stream slot exists per process.
//! * The slot is reserved atomically when the state moves out of `idle`, i.e.
//!   *before* the device is opened (`tuning`). It stays reserved through
//!   `locking`, `streaming`, `reacquiring` and `stopping`.
//! * A second `GET /stream` while reserved fails with [`StreamStartError::Busy`]
//!   regardless of the requested channel; the HTTP layer maps that to `409`.
//!   Once shutdown has begun the slot can never be reserved again.
//! * The reservation is released by a supervisor task only after the receiver
//!   worker thread has been joined, which also means the receiver's destructor
//!   has run. A worker panic unwinds through [`Worker`]'s `Drop`, which stops
//!   the receiver before the thread exits, so the supervisor still releases the
//!   slot only after that cleanup has fully completed.
//!
//! # Threading
//!
//! The receiver is blocking and is driven on a dedicated OS thread, never on the
//! Tokio runtime. The worker communicates lock/stream progress through a
//! `oneshot` and delivers TS chunks over a bounded `tokio::sync::mpsc` channel.
//! A full channel means the HTTP client is too slow: the worker counts the
//! overflow and closes the connection instead of growing without bound.
//! Dropping the response body closes the channel, which the worker observes and
//! treats as client disconnect.

use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio::sync::{mpsc, oneshot, watch};

// ---------------------------------------------------------------------------
// Public constants / channel plan
// ---------------------------------------------------------------------------

/// First Japanese UHF physical channel that carries ISDB-T.
pub const UHF_MIN_CHANNEL: u32 = 13;
/// Last Japanese UHF physical channel that carries ISDB-T.
pub const UHF_MAX_CHANNEL: u32 = 62;
/// Centre frequency of physical channel 13 in Hz (473 + 1/7 MHz).
pub const UHF_BASE_FREQUENCY_HZ: u32 = 473_142_857;
/// Channel spacing in Hz.
pub const UHF_CHANNEL_SPACING_HZ: u32 = 6_000_000;

/// Default initial-lock deadline, independent from the reacquire deadline.
pub const DEFAULT_INITIAL_LOCK_TIMEOUT: Duration = Duration::from_secs(10);
/// Default deadline to reacquire a lost lock before ending the connection.
pub const DEFAULT_REACQUIRE_TIMEOUT: Duration = Duration::from_secs(10);
/// Default bounded TS queue depth, in chunks.
pub const DEFAULT_QUEUE_CAPACITY: usize = 64;
/// Default receiver poll interval (also the worker cancellation granularity).
pub const DEFAULT_POLL_INTERVAL: Duration = Duration::from_millis(100);

/// Map a Japanese UHF physical channel to its centre frequency in Hz.
///
/// Only `13..=62` are accepted; anything else is rejected so the HTTP layer can
/// answer `400` instead of silently tuning somewhere else.
pub fn channel_frequency_hz(channel: u32) -> Option<u32> {
    if !(UHF_MIN_CHANNEL..=UHF_MAX_CHANNEL).contains(&channel) {
        return None;
    }
    Some(UHF_BASE_FREQUENCY_HZ + (channel - UHF_MIN_CHANNEL) * UHF_CHANNEL_SPACING_HZ)
}

/// Server stream lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamState {
    /// No reservation; the slot is free.
    Idle,
    /// Slot reserved, device open/tuning in progress.
    Tuning,
    /// Device configured; waiting for a valid TS lock.
    Locking,
    /// Valid TS data is flowing to the client.
    Streaming,
    /// Lock was lost; trying to reacquire (bounded by the reacquire deadline).
    Reacquiring,
    /// Teardown in progress; the reservation is still held.
    Stopping,
    /// An unrecoverable receiver error occurred; teardown follows.
    Error,
}

impl StreamState {
    /// Stable lowercase label (used in diagnostics).
    pub fn as_str(self) -> &'static str {
        match self {
            StreamState::Idle => "idle",
            StreamState::Tuning => "tuning",
            StreamState::Locking => "locking",
            StreamState::Streaming => "streaming",
            StreamState::Reacquiring => "reacquiring",
            StreamState::Stopping => "stopping",
            StreamState::Error => "error",
        }
    }
}

/// Emit one non-panicking lifecycle timing diagnostic on stderr.
///
/// Hardware bring-up needs to separate *tuning* (device open / configure) from
/// *locking* (waiting for the first valid TS) and *stopping* (cancel, join,
/// close). This writes one stable, generic line per begin/end marker so a
/// hardware test can parse elapsed milliseconds per phase:
///
/// ```text
/// ts-stream: phase=<phase> state=<state> event=<begin|end> [elapsed_ms=<n>]
/// ```
///
/// stdout is reserved for CLI output, so diagnostics always go to stderr. The
/// write result is ignored: a broken stderr must never panic a backend call or
/// the worker thread while a receiver is live.
fn log_phase(state: StreamState, phase: &str, event: &str, elapsed: Option<Duration>) {
    match elapsed {
        Some(elapsed) => {
            let _ = writeln!(
                std::io::stderr(),
                "ts-stream: phase={phase} state={} event={event} elapsed_ms={}",
                state.as_str(),
                elapsed.as_millis(),
            );
        }
        None => {
            let _ = writeln!(
                std::io::stderr(),
                "ts-stream: phase={phase} state={} event={event}",
                state.as_str(),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Backend abstraction
// ---------------------------------------------------------------------------

/// One enumerated device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    pub index: u32,
    pub name: String,
    pub serial: Option<String>,
}

/// Everything needed to open one receiver.
#[derive(Debug, Clone)]
pub struct OpenRequest {
    pub device_index: u32,
    pub frequency_hz: u32,
    pub gain: Option<f64>,
    pub ppm: i32,
    pub sample_rate: u32,
}

/// One event from a receiver.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RxEvent {
    /// Valid transport-stream bytes (one or more TS packets).
    Data(Vec<u8>),
    /// No new data yet, but no lock loss either: a poll timeout or a
    /// still-acquiring demodulator. Poll again without widening any deadline.
    Pending,
    /// The lock/signal was lost, or a valid IQ discontinuity was detected. The
    /// session enters `reacquiring`.
    NoSignal,
}

/// Receiver failure surface.
#[derive(Debug, Clone, thiserror::Error)]
pub enum BackendError {
    /// The device/library could not be used.
    #[error("device unavailable: {0}")]
    Unavailable(String),
    /// The requested receiver configuration is not supported (for example a
    /// rejected sample rate, resampling ratio, or modulation).
    #[error("decoder not implemented: {0}")]
    DecoderUnsupported(String),
    /// Unrecoverable receiver error.
    #[error("fatal receiver error: {0}")]
    Fatal(String),
    /// The receiver was stopped/cancelled.
    #[error("receiver stopped")]
    Stopped,
}

/// Blocking receiver driven from the worker thread.
pub trait TsReceiver: Send + 'static {
    /// Block up to `timeout` for the next event.
    fn recv_timeout(&mut self, timeout: Duration) -> Result<RxEvent, BackendError>;

    /// Cancel and join any receiver-owned worker. Must be idempotent.
    fn stop(&mut self) -> Result<(), BackendError>;
}

/// Injectable backend. Production wires [`crate::session::rtl_backend::RtlBackend`];
/// tests provide deterministic mocks.
pub trait Backend: Send + Sync + 'static {
    type Receiver: TsReceiver;

    /// Enumerate devices.
    fn list_devices(&self) -> Result<Vec<DeviceInfo>, BackendError>;

    /// Open and configure a receiver for `request`. Blocking.
    ///
    /// This call runs on the worker thread and is **not preemptible**: a
    /// shutdown/cancel that arrives while `open` is blocked inside `librtlsdr`
    /// (device open, tuning, sample-rate selection) can only be observed *after*
    /// it returns. The worker then checks cancellation and tears the receiver
    /// down; it never abandons an in-flight call or force-closes it on a timer
    /// (see [`SessionManager::shutdown`]). The initial-lock deadline is likewise
    /// measured from after this call returns, so slow configuration is not
    /// misreported as a lock timeout.
    fn open(&self, request: OpenRequest) -> Result<Self::Receiver, BackendError>;
}

/// Backend used when no hardware backend is appropriate (e.g. tests that only
/// exercise HTTP semantics). Every operation reports `Unavailable`.
#[derive(Debug, Default, Clone, Copy)]
pub struct UnavailableBackend;

impl Backend for UnavailableBackend {
    type Receiver = NullReceiver;

    fn list_devices(&self) -> Result<Vec<DeviceInfo>, BackendError> {
        Err(BackendError::Unavailable(
            "no RTL-SDR backend is active in this build".to_string(),
        ))
    }

    fn open(&self, _request: OpenRequest) -> Result<NullReceiver, BackendError> {
        Err(BackendError::Unavailable(
            "no RTL-SDR backend is active in this build".to_string(),
        ))
    }
}

/// Receiver that immediately reports that it is stopped.
#[derive(Debug, Default, Clone, Copy)]
pub struct NullReceiver;

impl TsReceiver for NullReceiver {
    fn recv_timeout(&mut self, _timeout: Duration) -> Result<RxEvent, BackendError> {
        Err(BackendError::Stopped)
    }

    fn stop(&mut self) -> Result<(), BackendError> {
        Ok(())
    }
}

/// Backend that fails every operation with a fixed diagnostic.
///
/// Used by the server when `librtlsdr` cannot be loaded: the HTTP server still
/// starts and serves `/version`, while `/stream` honestly answers `503`.
#[derive(Debug, Clone)]
pub struct UnavailableBackendWithReason {
    reason: String,
}

impl UnavailableBackendWithReason {
    /// Create the backend with the diagnostic reported for every request.
    pub fn new(reason: impl Into<String>) -> Self {
        Self {
            reason: reason.into(),
        }
    }
}

impl Backend for UnavailableBackendWithReason {
    type Receiver = NullReceiver;

    fn list_devices(&self) -> Result<Vec<DeviceInfo>, BackendError> {
        Err(BackendError::Unavailable(self.reason.clone()))
    }

    fn open(&self, _request: OpenRequest) -> Result<NullReceiver, BackendError> {
        Err(BackendError::Unavailable(self.reason.clone()))
    }
}

// ---------------------------------------------------------------------------
// Session configuration / errors
// ---------------------------------------------------------------------------

/// Tuning and lifecycle configuration for one server.
#[derive(Debug, Clone)]
pub struct SessionConfig {
    pub device_index: u32,
    pub gain: Option<f64>,
    pub ppm: i32,
    pub sample_rate: u32,
    /// Maximum time to wait for a valid TS lock **after** [`Backend::open`]
    /// returns. Device open and configuration are synchronous and not
    /// preemptible, so their duration is not counted against this deadline.
    pub initial_lock_timeout: Duration,
    pub reacquire_timeout: Duration,
    pub queue_capacity: usize,
    pub poll_interval: Duration,
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self {
            device_index: 0,
            gain: None,
            ppm: 0,
            sample_rate: crate::cli::DEFAULT_SAMPLE_RATE,
            initial_lock_timeout: DEFAULT_INITIAL_LOCK_TIMEOUT,
            reacquire_timeout: DEFAULT_REACQUIRE_TIMEOUT,
            queue_capacity: DEFAULT_QUEUE_CAPACITY,
            poll_interval: DEFAULT_POLL_INTERVAL,
        }
    }
}

/// Why [`SessionManager::start_stream`] could not hand back a streaming body.
#[derive(Debug, thiserror::Error)]
pub enum StreamStartError {
    #[error("invalid channel {0}")]
    BadChannel(u32),
    #[error("stream slot already in use")]
    Busy,
    #[error("initial lock timeout")]
    InitialLockTimeout,
    #[error("device unavailable: {0}")]
    Unavailable(String),
    #[error("decoder not implemented: {0}")]
    Unsupported(String),
    #[error("internal error: {0}")]
    Internal(String),
}

/// A successfully started stream: the bounded TS chunk receiver for the client.
///
/// The reservation is held by a detached supervisor task, not by this value, so
/// dropping the body does not by itself release the slot.
pub struct StreamResponse {
    pub(crate) body: mpsc::Receiver<Vec<u8>>,
}

impl StreamResponse {
    /// Take the TS chunk receiver. Dropping it signals client disconnect.
    pub fn into_body_receiver(self) -> mpsc::Receiver<Vec<u8>> {
        self.body
    }
}

// ---------------------------------------------------------------------------
// Shared slot
// ---------------------------------------------------------------------------

struct Inner {
    reserved: bool,
    state: StreamState,
    generation: u64,
    active_channel: Option<u32>,
    /// Set permanently by [`SessionManager::begin_shutdown`]. While set, no new
    /// reservation may be taken, so a shutdown cannot race a fresh `start_stream`.
    shutting_down: bool,
}

struct Shared {
    inner: Mutex<Inner>,
    state_tx: watch::Sender<StreamState>,
}

impl Shared {
    /// Publish `state` to both the mutex-protected metadata and the watch
    /// channel, atomically with respect to reservation transitions.
    ///
    /// The lock is held across the watch update so the reservation flags and the
    /// observed state can never get out of order. `send_replace` is used because
    /// `send` is a no-op when no receiver is subscribed, which would leave a
    /// later subscriber observing stale state.
    ///
    /// While [`Inner::shutting_down`] is set, the in-flight worker may still be
    /// unwinding out of a blocking backend call. Any progress state it publishes
    /// after shutdown (`tuning` / `locking` / `streaming` / `reacquiring`) is
    /// suppressed: once shutdown has published `stopping`, the state must not
    /// move backwards while the receiver is torn down. Terminal states
    /// (`stopping`, `error`) and the final `idle` release are still allowed.
    fn set_state(&self, state: StreamState) {
        let mut guard = self.inner.lock().expect("session mutex poisoned");
        if guard.shutting_down
            && matches!(
                state,
                StreamState::Tuning
                    | StreamState::Locking
                    | StreamState::Streaming
                    | StreamState::Reacquiring
            )
        {
            return;
        }
        guard.state = state;
        let _ = self.state_tx.send_replace(state);
    }
}

/// RAII owner of the single stream slot.
///
/// Moved into the supervisor task, which drops it only after the receiver worker
/// thread has been joined. It is *not* released on a plain handle drop on the
/// request side.
struct Reservation {
    shared: Arc<Shared>,
    generation: u64,
    active: bool,
}

impl Reservation {
    fn release(&mut self) {
        if !self.active {
            return;
        }
        self.active = false;
        let mut guard = self.shared.inner.lock().expect("session mutex poisoned");
        // Only the reservation that still owns the current generation may return
        // the slot to `Idle`; otherwise a released-but-late worker could clobber
        // the state of the next reservation.
        if guard.generation == self.generation {
            guard.reserved = false;
            guard.state = StreamState::Idle;
            guard.active_channel = None;
            let _ = self.shared.state_tx.send_replace(StreamState::Idle);
        }
    }
}

impl Drop for Reservation {
    fn drop(&mut self) {
        self.release();
    }
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

enum LockOutcome {
    Locked,
    Timeout,
    Cancelled,
    Overflow,
    Unavailable(String),
    Unsupported(String),
}

struct Worker<B: Backend> {
    backend: Arc<B>,
    shared: Arc<Shared>,
    hw_cancel: Arc<AtomicBool>,
    overflow_count: Arc<AtomicU64>,
    config: Arc<SessionConfig>,
    body_tx: mpsc::Sender<Vec<u8>>,
    lock_tx: Option<oneshot::Sender<LockOutcome>>,
    receiver: Option<B::Receiver>,
    request: OpenRequest,
}

impl<B: Backend> Worker<B> {
    fn set_state(&self, state: StreamState) {
        self.shared.set_state(state);
    }

    fn cancelled(&self) -> bool {
        self.hw_cancel.load(Ordering::SeqCst) || self.body_tx.is_closed()
    }

    /// Queue one TS chunk. Returns `true` when the stream must stop (overflow or
    /// the client went away).
    fn push_chunk(&mut self, chunk: Vec<u8>) -> bool {
        match self.body_tx.try_send(chunk) {
            Ok(()) => false,
            Err(mpsc::error::TrySendError::Full(_)) => {
                self.overflow_count.fetch_add(1, Ordering::Relaxed);
                eprintln!("ts-stream: bounded TS queue overflow; closing this client connection");
                true
            }
            Err(mpsc::error::TrySendError::Closed(_)) => true,
        }
    }

    fn run(mut self) {
        let outcome = self.open_and_acquire();
        match outcome {
            LockOutcome::Locked => {
                self.set_state(StreamState::Streaming);
                let sent = self
                    .lock_tx
                    .take()
                    .map(|tx| tx.send(LockOutcome::Locked).is_ok())
                    .unwrap_or(false);
                if !sent {
                    // The HTTP handler went away before headers. `Drop` performs
                    // the stop cleanup; the supervisor releases the slot.
                    return;
                }
                self.stream();
            }
            other => {
                // Stop the receiver *before* notifying the handler, so the caller
                // only observes a terminal result once cleanup has run.
                self.cleanup();
                if let Some(tx) = self.lock_tx.take() {
                    let _ = tx.send(other);
                }
            }
        }
        // Normal or panic path: `Drop` is idempotent and finishes cleanup. The
        // supervisor releases the reservation only after this thread is joined.
    }

    fn open_and_acquire(&mut self) -> LockOutcome {
        // Check cancellation *before* touching the backend. A blocking `open`
        // cannot be preempted, so if shutdown already won the race there is no
        // reason to open a device we would only have to close immediately.
        if self.cancelled() {
            return LockOutcome::Cancelled;
        }
        self.set_state(StreamState::Tuning);
        let receiver = match self.backend.open(self.request.clone()) {
            Ok(receiver) => receiver,
            Err(BackendError::Unavailable(message)) => return LockOutcome::Unavailable(message),
            Err(BackendError::DecoderUnsupported(message)) => {
                return LockOutcome::Unsupported(message)
            }
            Err(error) => return LockOutcome::Unavailable(error.to_string()),
        };
        self.receiver = Some(receiver);
        self.set_state(StreamState::Locking);

        let deadline = Instant::now() + self.config.initial_lock_timeout;
        loop {
            if self.cancelled() {
                return LockOutcome::Cancelled;
            }
            let now = Instant::now();
            if now >= deadline {
                return LockOutcome::Timeout;
            }
            let wait = (deadline - now).min(self.config.poll_interval);
            let event = self
                .receiver
                .as_mut()
                .expect("receiver set")
                .recv_timeout(wait);
            match event {
                Ok(RxEvent::Data(chunk)) => {
                    if self.push_chunk(chunk) {
                        return LockOutcome::Overflow;
                    }
                    return LockOutcome::Locked;
                }
                // Neither a lock nor a lock loss: keep waiting against the same
                // initial deadline.
                Ok(RxEvent::Pending) | Ok(RxEvent::NoSignal) => {}
                Err(BackendError::DecoderUnsupported(message)) => {
                    return LockOutcome::Unsupported(message)
                }
                Err(BackendError::Unavailable(message)) => {
                    return LockOutcome::Unavailable(message)
                }
                Err(BackendError::Fatal(message)) => return LockOutcome::Unavailable(message),
                Err(BackendError::Stopped) => return LockOutcome::Cancelled,
            }
        }
    }

    fn stream(&mut self) {
        loop {
            if self.cancelled() {
                return;
            }
            let event = self
                .receiver
                .as_mut()
                .expect("receiver set")
                .recv_timeout(self.config.poll_interval);
            match event {
                Ok(RxEvent::Data(chunk)) => {
                    if self.push_chunk(chunk) {
                        return;
                    }
                }
                Ok(RxEvent::NoSignal) => {
                    if !self.reacquire() {
                        return;
                    }
                }
                // No new data yet: stay streaming, do not start reacquiring.
                Ok(RxEvent::Pending) => {}
                Err(BackendError::Stopped) => return,
                Err(_) => {
                    self.set_state(StreamState::Error);
                    return;
                }
            }
        }
    }

    /// Try to regain a lock for at most `reacquire_timeout`, returning `true`
    /// when streaming resumed.
    fn reacquire(&mut self) -> bool {
        self.set_state(StreamState::Reacquiring);
        let deadline = Instant::now() + self.config.reacquire_timeout;
        loop {
            if self.cancelled() {
                return false;
            }
            let now = Instant::now();
            if now >= deadline {
                return false;
            }
            let wait = (deadline - now).min(self.config.poll_interval);
            let event = self
                .receiver
                .as_mut()
                .expect("receiver set")
                .recv_timeout(wait);
            match event {
                Ok(RxEvent::Data(chunk)) => {
                    if self.push_chunk(chunk) {
                        return false;
                    }
                    self.set_state(StreamState::Streaming);
                    return true;
                }
                // A pending event must not reset the reacquire deadline: it is
                // just "no new data yet". `deadline` is fixed before the loop.
                Ok(RxEvent::Pending) | Ok(RxEvent::NoSignal) => {}
                Err(_) => return false,
            }
        }
    }

    /// Enter `stopping`, stop the receiver and let its destructor run. Safe to
    /// call twice.
    ///
    /// The slot is *not* released here. The supervisor task releases it only
    /// after this worker thread has been joined, which means after the
    /// receiver's destructor completed. Holding `stopping` across both the
    /// (possibly blocking) `stop` call and the destructor keeps other requests
    /// at `409` for the whole teardown.
    fn cleanup(&mut self) {
        self.set_state(StreamState::Stopping);
        if let Some(mut receiver) = self.receiver.take() {
            log_phase(StreamState::Stopping, "receiver-stop", "begin", None);
            let started = Instant::now();
            if let Err(error) = receiver.stop() {
                eprintln!("ts-stream: receiver stop during cleanup failed: {error}");
            }
            // `receiver` is dropped here, still on this worker thread and still
            // before the thread exits, so the destructor is part of the join.
            drop(receiver);
            log_phase(
                StreamState::Stopping,
                "receiver-stop",
                "end",
                Some(started.elapsed()),
            );
        }
    }
}

impl<B: Backend> Drop for Worker<B> {
    fn drop(&mut self) {
        // Runs on every exit path, including a panic inside `run`. The receiver
        // is stopped (cancel + destructor) before the thread exits; the
        // supervisor releases the reservation after the join.
        self.cleanup();
    }
}

// ---------------------------------------------------------------------------
// SessionManager
// ---------------------------------------------------------------------------

/// Owns the single stream slot and spawns the receiver worker.
pub struct SessionManager<B: Backend> {
    backend: Arc<B>,
    config: Arc<SessionConfig>,
    shared: Arc<Shared>,
    hw_cancel: Arc<AtomicBool>,
    overflow_count: Arc<AtomicU64>,
}

// Manual impl: cloning only needs `B: Backend`, never `B: Clone`.
impl<B: Backend> Clone for SessionManager<B> {
    fn clone(&self) -> Self {
        Self {
            backend: Arc::clone(&self.backend),
            config: Arc::clone(&self.config),
            shared: Arc::clone(&self.shared),
            hw_cancel: Arc::clone(&self.hw_cancel),
            overflow_count: Arc::clone(&self.overflow_count),
        }
    }
}

impl<B: Backend> SessionManager<B> {
    /// Create a manager. `backend` is shared with every worker it spawns.
    pub fn new(backend: Arc<B>, config: SessionConfig) -> Self {
        let (state_tx, _state_rx) = watch::channel(StreamState::Idle);
        Self {
            backend,
            config: Arc::new(config),
            shared: Arc::new(Shared {
                inner: Mutex::new(Inner {
                    reserved: false,
                    state: StreamState::Idle,
                    generation: 0,
                    active_channel: None,
                    shutting_down: false,
                }),
                state_tx,
            }),
            hw_cancel: Arc::new(AtomicBool::new(false)),
            overflow_count: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Current lifecycle state.
    pub fn state(&self) -> StreamState {
        self.shared
            .inner
            .lock()
            .expect("session mutex poisoned")
            .state
    }

    /// Whether the single stream slot is currently reserved.
    pub fn is_reserved(&self) -> bool {
        self.shared
            .inner
            .lock()
            .expect("session mutex poisoned")
            .reserved
    }

    /// Channel currently holding the slot, if any.
    pub fn active_channel(&self) -> Option<u32> {
        self.shared
            .inner
            .lock()
            .expect("session mutex poisoned")
            .active_channel
    }

    /// Number of times a client was disconnected because the bounded TS queue
    /// overflowed.
    pub fn overflow_count(&self) -> u64 {
        self.overflow_count.load(Ordering::Relaxed)
    }

    /// Subscribe to lifecycle state changes.
    pub fn subscribe(&self) -> watch::Receiver<StreamState> {
        self.shared.state_tx.subscribe()
    }

    /// Force a state transition without touching the backend.
    ///
    /// Intended for diagnostics and tests that must observe `/version` in states
    /// which are otherwise timing dependent. It never changes the reservation.
    #[doc(hidden)]
    pub fn debug_set_state(&self, state: StreamState) {
        self.shared.set_state(state);
    }

    /// Reserve the slot and start a stream for `channel`.
    ///
    /// Returns only once the initial lock has succeeded (with a streaming body)
    /// or the attempt has terminally failed. The returned future owns the
    /// response-side channel; dropping it (client disconnect before headers)
    /// closes that channel, which the worker observes and turns into cleanup.
    ///
    /// The receiver thread is supervised by a detached blocking task that joins
    /// the thread and only then releases the reservation. That supervisor is
    /// installed before the first `await`, so cancelling this future can never
    /// drop the cleanup on the floor.
    pub async fn start_stream(&self, channel: u32) -> Result<StreamResponse, StreamStartError> {
        let frequency_hz =
            channel_frequency_hz(channel).ok_or(StreamStartError::BadChannel(channel))?;
        let reservation = self.try_reserve(channel).ok_or(StreamStartError::Busy)?;

        let capacity = self.config.queue_capacity.max(1);
        let (body_tx, body_rx) = mpsc::channel::<Vec<u8>>(capacity);
        let (lock_tx, lock_rx) = oneshot::channel::<LockOutcome>();

        let worker = Worker {
            backend: Arc::clone(&self.backend),
            shared: Arc::clone(&self.shared),
            hw_cancel: Arc::clone(&self.hw_cancel),
            overflow_count: Arc::clone(&self.overflow_count),
            config: Arc::clone(&self.config),
            body_tx,
            lock_tx: Some(lock_tx),
            receiver: None,
            request: OpenRequest {
                device_index: self.config.device_index,
                frequency_hz,
                gain: self.config.gain,
                ppm: self.config.ppm,
                sample_rate: self.config.sample_rate,
            },
        };

        let join = match std::thread::Builder::new()
            .name("ts-rx".to_string())
            .spawn(move || worker.run())
        {
            Ok(join) => join,
            Err(error) => {
                // `spawn` dropped the worker closure (and thus the `Worker`) when
                // it failed. The reservation is still held here and is released
                // only now, after that worker is gone.
                drop(reservation);
                return Err(StreamStartError::Internal(format!(
                    "failed to spawn receiver thread: {error}"
                )));
            }
        };

        // Installed synchronously, before `lock_rx.await`, so a cancelled
        // handler cannot strand the receiver or lose the slot. `spawn_blocking`
        // tasks are detached: dropping this handle does not cancel the
        // supervisor.
        let (release_tx, release_rx) = oneshot::channel::<()>();
        tokio::task::spawn_blocking(move || {
            // Release the slot only after the worker OS thread has fully exited,
            // which includes running the receiver's destructor.
            let _ = join.join();
            drop(reservation);
            let _ = release_tx.send(());
        });

        let outcome = lock_rx.await;
        if !matches!(&outcome, Ok(LockOutcome::Locked)) {
            // Terminal failure (including a worker panic): wait for the
            // supervisor to join the worker and release the slot before
            // reporting, so the caller never observes a stale reservation and a
            // subsequent request starts from a clean slot.
            let _ = release_rx.await;
        }
        match outcome {
            Ok(LockOutcome::Locked) => Ok(StreamResponse { body: body_rx }),
            Ok(LockOutcome::Timeout) => Err(StreamStartError::InitialLockTimeout),
            Ok(LockOutcome::Unavailable(message)) => Err(StreamStartError::Unavailable(message)),
            Ok(LockOutcome::Unsupported(message)) => Err(StreamStartError::Unsupported(message)),
            Ok(LockOutcome::Cancelled) => Err(StreamStartError::Internal(
                "receiver cancelled before lock".to_string(),
            )),
            Ok(LockOutcome::Overflow) => Err(StreamStartError::Internal(
                "bounded queue overflowed before lock".to_string(),
            )),
            Err(_) => Err(StreamStartError::Internal(
                "receiver worker exited before reporting a lock result".to_string(),
            )),
        }
    }

    /// Atomically take the slot. `None` when it is already reserved or when
    /// shutdown has begun.
    fn try_reserve(&self, channel: u32) -> Option<Reservation> {
        let mut guard = self.shared.inner.lock().expect("session mutex poisoned");
        if guard.reserved || guard.shutting_down {
            return None;
        }
        guard.reserved = true;
        guard.generation = guard.generation.wrapping_add(1);
        guard.active_channel = Some(channel);
        guard.state = StreamState::Tuning;
        let generation = guard.generation;
        // Coherent with the reservation flags while the same lock is held.
        let _ = self.shared.state_tx.send_replace(StreamState::Tuning);
        drop(guard);
        Some(Reservation {
            shared: Arc::clone(&self.shared),
            generation,
            active: true,
        })
    }

    /// Wait until the slot has been released, or `timeout` elapses.
    ///
    /// The slot is released only after the receiver worker thread has been
    /// joined, so `true` means cleanup (including the receiver destructor)
    /// completed.
    pub async fn wait_released(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        while self.is_reserved() {
            if Instant::now() >= deadline {
                return false;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        true
    }

    /// Wait without a bound for the worker to be joined and the slot released.
    async fn await_release(&self) {
        while self.is_reserved() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    /// Mark shutdown permanent and signal any active receiver to stop.
    ///
    /// `shutting_down` is set under the reservation mutex, so a `start_stream`
    /// either reserves before shutdown begins (and is then stopped by the cancel
    /// flag) or is refused afterwards; there is no interleaving that leaks a
    /// fresh worker past shutdown.
    ///
    /// Once `stopping` is published it is monotonic: a worker unwinding out of a
    /// blocking [`Backend::open`] cannot republish `tuning` / `locking` /
    /// `streaming` / `reacquiring` (see [`Shared::set_state`]).
    fn begin_shutdown(&self) {
        self.hw_cancel.store(true, Ordering::SeqCst);
        let mut guard = self.shared.inner.lock().expect("session mutex poisoned");
        guard.shutting_down = true;
        if guard.reserved {
            guard.state = StreamState::Stopping;
            let _ = self.shared.state_tx.send_replace(StreamState::Stopping);
        }
    }

    /// Cancel any active stream and wait for full receiver cleanup.
    ///
    /// Used by the SIGINT/SIGTERM path. This waits until the worker thread has
    /// been joined and the slot released; it does not give up early. After it
    /// completes, no further stream can be started.
    ///
    /// A worker blocked inside [`Backend::open`] (device open / configuration)
    /// cannot be preempted; shutdown waits for that blocking call to return and
    /// then tears the receiver down. Cleanup is intentionally unbounded instead
    /// of force-closing the device on a timeout, because abandoning a live
    /// `librtlsdr` handle would be unsafe. If the backend never returns, the
    /// process cannot exit cleanly (`SIGKILL` is the last resort, not a clean
    /// shutdown).
    pub async fn shutdown(&self) {
        self.begin_shutdown();
        self.await_release().await;
    }

    /// Like [`shutdown`](Self::shutdown) but bounded by `timeout`.
    ///
    /// Returns `true` when cleanup completed within the bound. On timeout the
    /// state stays `stopping` and the slot stays reserved; nothing is faked to
    /// `idle`. Use [`shutdown`](Self::shutdown) when full cleanup is required.
    pub async fn shutdown_with_timeout(&self, timeout: Duration) -> bool {
        self.begin_shutdown();
        if !self.is_reserved() {
            return true;
        }
        if self.wait_released(timeout).await {
            true
        } else {
            eprintln!("ts-stream: shutdown timed out waiting for receiver cleanup");
            false
        }
    }
}

// ---------------------------------------------------------------------------
// Production RTL-SDR backend (real `crate::rtl` glue + ISDB-T decoder)
// ---------------------------------------------------------------------------

/// Production glue over [`crate::rtl`] and [`crate::demod::receiver`].
pub mod rtl_backend {
    use super::{Backend, BackendError, DeviceInfo, OpenRequest, RxEvent, StreamState, TsReceiver};
    use std::io::Write;
    use std::time::{Duration, Instant};

    use crate::demod::receiver::{Decoder, ReceiverError};
    use crate::rtl::{IqBlock, RtlError, RtlLibrary};

    /// Maximum number of IQ bytes fed to the demodulator per `recv_timeout`
    /// call.
    ///
    /// The USB callback hands out buffers of up to 256 KiB. Feeding a whole
    /// buffer in one call would stall cancellation and deadline checks for a
    /// long time, so the adapter retains the remainder and decodes it in bounded
    /// batches instead.
    const MAX_FEED_BYTES: usize = 8192;

    /// A single decode batch slower than this is worth one diagnostic line.
    ///
    /// Normal batches are sub-millisecond; logging every batch would flood the
    /// hardware-test log. This is purely observational and never widens a
    /// session deadline.
    const SLOW_BATCH_LOG_THRESHOLD: Duration = Duration::from_millis(100);

    /// Blocking source of raw interleaved `u8` IQ blocks.
    ///
    /// Abstracted so the adapter's batching and event classification can be
    /// tested without hardware.
    trait IqSource: Send {
        /// Wait up to `timeout` for the next IQ block (`None` on timeout).
        fn recv_timeout(&mut self, timeout: Duration) -> Result<Option<IqBlock>, BackendError>;

        /// Cancel and join any receiver-owned work. Idempotent.
        fn stop(&mut self) -> Result<(), BackendError>;
    }

    impl IqSource for crate::rtl::Receiver {
        fn recv_timeout(&mut self, timeout: Duration) -> Result<Option<IqBlock>, BackendError> {
            crate::rtl::Receiver::recv_timeout(self, timeout).map_err(map_rtl_error)
        }

        fn stop(&mut self) -> Result<(), BackendError> {
            crate::rtl::Receiver::stop(self).map_err(map_rtl_error)
        }
    }

    fn map_rtl_error(error: RtlError) -> BackendError {
        match error {
            // The reader ended because we asked it to; the worker treats this as
            // a cancellation, not as a device failure.
            RtlError::ReadAsync { stopped: true, .. } => BackendError::Stopped,
            other => BackendError::Fatal(other.to_string()),
        }
    }

    /// Loaded `librtlsdr` plus its resolved symbols.
    pub struct RtlBackend {
        library: RtlLibrary,
    }

    impl RtlBackend {
        /// Load `librtlsdr`.
        pub fn load() -> Result<Self, BackendError> {
            let library =
                RtlLibrary::load().map_err(|error| BackendError::Unavailable(error.to_string()))?;
            Ok(Self { library })
        }

        /// Open/configure the device and build the adapter.
        ///
        /// Split out of [`Backend::open`] so the timing begin/end markers wrap
        /// every return path without duplicating the error handling.
        fn open_configured(&self, request: OpenRequest) -> Result<RtlReceiver, BackendError> {
            // Validate and build the demodulator *before* opening the device, so
            // an unsupported sample rate fails without touching hardware.
            let decoder = Decoder::new(request.sample_rate).map_err(|error| {
                BackendError::DecoderUnsupported(format!(
                    "cannot configure the ISDB-T decoder for {} Hz: {error}",
                    request.sample_rate
                ))
            })?;

            let receiver = self
                .library
                .open(
                    request.device_index,
                    request.frequency_hz,
                    request.gain,
                    request.ppm,
                    request.sample_rate,
                )
                .map_err(|error| BackendError::Unavailable(error.to_string()))?;

            Ok(RtlReceiver {
                source: Some(Box::new(receiver)),
                decoder,
                pending: Vec::new(),
                pending_pos: 0,
                pending_gap: false,
                had_ts: false,
            })
        }
    }

    impl Backend for RtlBackend {
        type Receiver = RtlReceiver;

        fn list_devices(&self) -> Result<Vec<DeviceInfo>, BackendError> {
            self.library
                .devices()
                .map(|devices| {
                    devices
                        .into_iter()
                        .map(|device| DeviceInfo {
                            index: device.index,
                            name: device.name,
                            serial: device.serial,
                        })
                        .collect()
                })
                .map_err(|error| BackendError::Unavailable(error.to_string()))
        }

        fn open(&self, request: OpenRequest) -> Result<RtlReceiver, BackendError> {
            // Timing markers separate device open/configure (tuning) from the
            // initial lock wait (locking). The end marker is emitted on both the
            // success and failure paths.
            super::log_phase(StreamState::Tuning, "backend-open", "begin", None);
            let started = Instant::now();
            let result = self.open_configured(request);
            super::log_phase(
                StreamState::Tuning,
                "backend-open",
                "end",
                Some(started.elapsed()),
            );
            result
        }
    }

    /// Outcome of classifying one decoded IQ batch.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum BatchClass {
        /// Real TS packets were decoded.
        Data,
        /// A definite loss of lock: an IQ gap, or a previously established lock
        /// that the decoder has since dropped.
        NoSignal,
        /// No new data yet, and no evidence that a lock was lost.
        Pending,
    }

    /// Pure classifier for one decoder batch.
    ///
    /// * packets decoded -> [`BatchClass::Data`] (even across a gap: real TS is
    ///   never discarded),
    /// * an IQ gap with no output -> [`BatchClass::NoSignal`] immediately,
    /// * a lock that was held and is now gone -> [`BatchClass::NoSignal`],
    /// * otherwise -> [`BatchClass::Pending`].
    fn classify_batch(has_packets: bool, gap: bool, had_ts: bool, locked: bool) -> BatchClass {
        if has_packets {
            BatchClass::Data
        } else if gap || (had_ts && !locked) {
            BatchClass::NoSignal
        } else {
            BatchClass::Pending
        }
    }

    fn flatten_packets(packets: Vec<[u8; 188]>) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(packets.len() * 188);
        for packet in packets {
            bytes.extend_from_slice(&packet);
        }
        bytes
    }

    /// RTL-SDR receiver adapted to [`TsReceiver`].
    ///
    /// Raw IQ is normalised and demodulated by [`crate::demod::receiver::Decoder`].
    /// The adapter retains unprocessed IQ between `recv_timeout` calls so a
    /// large USB buffer cannot stall cancellation or the session deadline, and
    /// it only ever forwards real, RS-validated TS packets.
    pub struct RtlReceiver {
        source: Option<Box<dyn IqSource>>,
        decoder: Decoder,
        /// IQ bytes received but not yet fed to the decoder.
        pending: Vec<u8>,
        /// Offset of the first byte in `pending` that has not been fed.
        pending_pos: usize,
        /// Whether the next not-yet-fed byte starts after an IQ discontinuity.
        pending_gap: bool,
        /// Whether real TS has been produced since the last lock loss.
        had_ts: bool,
    }

    impl RtlReceiver {
        /// Feed the next bounded batch of retained IQ to the decoder.
        fn decode_next_batch(&mut self) -> Result<RxEvent, BackendError> {
            let end = (self.pending_pos + MAX_FEED_BYTES).min(self.pending.len());
            let gap = self.pending_gap;
            self.pending_gap = false;

            // Copy the bounded batch out so the decoder can be borrowed mutably.
            let batch = self.pending[self.pending_pos..end].to_vec();
            self.pending_pos = end;
            if self.pending_pos >= self.pending.len() {
                self.pending.clear();
                self.pending_pos = 0;
            }

            // Time only the decoder call itself. A slow batch is surfaced once,
            // with the batch size and the decoder's cumulative sample counters,
            // so a hardware run can locate a stall without per-batch noise.
            let batch_bytes = batch.len();
            let decode_started = Instant::now();
            let decoded = self.decoder.push_iq(&batch, gap);
            let decode_elapsed = decode_started.elapsed();
            if decode_elapsed > SLOW_BATCH_LOG_THRESHOLD {
                let stats = self.decoder.stats();
                let _ = writeln!(
                    std::io::stderr(),
                    "ts-stream: slow ISDB-T batch elapsed_ms={} batch_bytes={} input_bytes={} \
                     input_samples={} resampled_samples={}",
                    decode_elapsed.as_millis(),
                    batch_bytes,
                    stats.input_bytes,
                    stats.input_samples,
                    stats.resampled_samples,
                );
            }

            let packets = match decoded {
                Ok(packets) => packets,
                Err(ReceiverError::DqpskUnsupported) => {
                    self.had_ts = false;
                    return Err(BackendError::DecoderUnsupported(
                        "ISDB-T layer A uses DQPSK, which this coherent receiver does not \
                         support"
                            .to_string(),
                    ));
                }
                Err(
                    error @ (ReceiverError::UnsupportedSampleRate { .. }
                    | ReceiverError::Resample { .. }),
                ) => {
                    self.had_ts = false;
                    return Err(BackendError::DecoderUnsupported(format!(
                        "ISDB-T decoder rejected the input: {error}"
                    )));
                }
                Err(_) => {
                    // A transient demodulation failure (sync / equalisation /
                    // FEC) is not fatal: report the lost lock and let the
                    // session reacquire instead of tearing the device down.
                    self.had_ts = false;
                    return Ok(RxEvent::NoSignal);
                }
            };

            if gap {
                let _ = writeln!(
                    std::io::stderr(),
                    "ts-stream: IQ gap detected; resynchronising the ISDB-T decoder"
                );
                self.log_stats();
            }

            let has_packets = !packets.is_empty();
            match classify_batch(has_packets, gap, self.had_ts, self.decoder.is_locked()) {
                BatchClass::Data => {
                    if !self.had_ts {
                        let _ = writeln!(
                            std::io::stderr(),
                            "ts-stream: ISDB-T lock acquired; forwarding TS"
                        );
                        self.log_stats();
                    }
                    self.had_ts = true;
                    Ok(RxEvent::Data(flatten_packets(packets)))
                }
                BatchClass::NoSignal => {
                    self.had_ts = false;
                    Ok(RxEvent::NoSignal)
                }
                BatchClass::Pending => Ok(RxEvent::Pending),
            }
        }

        /// Best-effort decoder diagnostics. A failed write is ignored so a
        /// broken stderr can never panic the worker thread.
        ///
        /// The lifetime counters include every reset class (notably
        /// `ts_stall`); the separate `sync` line is the *current* CP-lock
        /// snapshot, so a hardware run can tell "never locked" (`cp_locked=0
        /// mode=None`) apart from "CP locked but no TS" and can read the SFO
        /// trend from `timing_offset`.
        fn log_stats(&self) {
            let stats = self.decoder.stats();
            let _ = writeln!(
                std::io::stderr(),
                "ts-stream: isdb-t stats input_bytes={} samples={} resampled={} gaps={} \
                 tmcc_frames={} ts_packets={} resets={} (gap={} sync={} config={} \
                 tmcc_timeout={} ts_stall={})",
                stats.input_bytes,
                stats.input_samples,
                stats.resampled_samples,
                stats.gaps,
                stats.tmcc_frames,
                stats.ts_packets,
                stats.resets,
                stats.resets_gap,
                stats.resets_sync,
                stats.resets_config,
                stats.resets_tmcc_timeout,
                stats.resets_ts_stall,
            );

            let diagnostics = self.decoder.diagnostics();
            let _ = writeln!(
                std::io::stderr(),
                "ts-stream: isdb-t sync cp_locked={} mode={:?} guard={:?} generation={} \
                 losses={} integer_bins={:?} cfo={:?} timing_offset={:?}",
                diagnostics.cp_locked,
                diagnostics.mode,
                diagnostics.guard,
                diagnostics.generation,
                diagnostics.losses,
                diagnostics.integer_bins,
                diagnostics.cfo,
                diagnostics.timing_offset,
            );
        }
    }

    impl TsReceiver for RtlReceiver {
        fn recv_timeout(&mut self, timeout: Duration) -> Result<RxEvent, BackendError> {
            // Retained IQ has priority and never blocks: feed one bounded batch
            // per call so the caller can re-check cancellation and deadlines.
            if self.pending_pos < self.pending.len() {
                return self.decode_next_batch();
            }

            let source = self.source.as_mut().ok_or(BackendError::Stopped)?;
            let block = match source.recv_timeout(timeout)? {
                Some(block) => block,
                // No new IQ within the deadline: not a lock loss.
                None => return Ok(RxEvent::Pending),
            };

            self.pending = block.samples;
            self.pending_pos = 0;
            self.pending_gap = block.gap;
            if self.pending.is_empty() {
                return if self.pending_gap {
                    self.pending_gap = false;
                    Ok(RxEvent::NoSignal)
                } else {
                    Ok(RxEvent::Pending)
                };
            }
            self.decode_next_batch()
        }

        fn stop(&mut self) -> Result<(), BackendError> {
            // Emit the final decoder snapshot *before* closing the source, so a
            // hardware test can still observe IQ/sample/sync-reset/TMCC/TS/gap
            // counters even when the lock never succeeded and no TS flowed.
            // Best-effort: a broken stderr must not fail teardown.
            self.log_stats();
            if let Some(mut source) = self.source.take() {
                source.stop()?;
            }
            Ok(())
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use std::collections::VecDeque;

        /// Scripted IQ source used by the adapter tests; never touches hardware.
        struct FakeIqSource {
            blocks: VecDeque<Result<Option<IqBlock>, BackendError>>,
        }

        impl FakeIqSource {
            fn blocks(blocks: Vec<IqBlock>) -> Self {
                Self {
                    blocks: blocks.into_iter().map(|block| Ok(Some(block))).collect(),
                }
            }
        }

        impl IqSource for FakeIqSource {
            fn recv_timeout(
                &mut self,
                _timeout: Duration,
            ) -> Result<Option<IqBlock>, BackendError> {
                match self.blocks.pop_front() {
                    Some(result) => result,
                    // No scripted block: behave like a poll timeout.
                    None => Ok(None),
                }
            }

            fn stop(&mut self) -> Result<(), BackendError> {
                Ok(())
            }
        }

        fn receiver_with(source: FakeIqSource) -> RtlReceiver {
            RtlReceiver {
                source: Some(Box::new(source)),
                decoder: Decoder::new(crate::demod::receiver::DEFAULT_INPUT_RATE)
                    .expect("the 1.2 MHz decoder must build without hardware"),
                pending: Vec::new(),
                pending_pos: 0,
                pending_gap: false,
                had_ts: false,
            }
        }

        #[test]
        fn classifier_matches_the_lock_lifecycle() {
            use BatchClass::*;
            // Real packets always win, even across a gap.
            assert_eq!(classify_batch(true, false, false, false), Data);
            assert_eq!(classify_batch(true, true, true, false), Data);
            // A gap is a definite loss even when nothing was decoded.
            assert_eq!(classify_batch(false, true, false, false), NoSignal);
            // Losing a previously established lock is NoSignal...
            assert_eq!(classify_batch(false, false, true, false), NoSignal);
            // ...but no packets while acquiring or still locked is Pending.
            assert_eq!(classify_batch(false, false, false, false), Pending);
            assert_eq!(classify_batch(false, false, true, true), Pending);
        }

        #[test]
        fn silence_never_produces_ts() {
            let mut receiver = receiver_with(FakeIqSource::blocks(vec![IqBlock {
                samples: vec![128u8; 32 * 1024],
                gap: false,
            }]));

            // 32 KiB at 8 KiB per call: enough calls to drain the whole block.
            for _ in 0..8 {
                match receiver.recv_timeout(Duration::from_millis(1)).unwrap() {
                    RxEvent::Data(bytes) => {
                        panic!("silence must not produce TS ({} bytes)", bytes.len())
                    }
                    RxEvent::Pending | RxEvent::NoSignal => {}
                }
            }

            assert_eq!(receiver.decoder.stats().ts_packets, 0);
            assert_eq!(receiver.decoder.stats().gaps, 0);
            assert!(!receiver.decoder.is_locked());
            assert!(!receiver.had_ts);
        }

        #[test]
        fn large_usb_block_is_decoded_in_bounded_batches() {
            let total = MAX_FEED_BYTES * 3 + 5;
            let mut receiver = receiver_with(FakeIqSource::blocks(vec![IqBlock {
                samples: vec![128u8; total],
                gap: false,
            }]));

            // The first call consumes exactly one bounded batch and retains the
            // rest for the following calls.
            assert_eq!(
                receiver.recv_timeout(Duration::from_millis(1)).unwrap(),
                RxEvent::Pending
            );
            assert_eq!(receiver.pending_pos, MAX_FEED_BYTES);
            assert_eq!(receiver.pending.len(), total);

            // Remaining calls drain the retained IQ without blocking.
            for _ in 0..3 {
                let _ = receiver.recv_timeout(Duration::from_millis(1)).unwrap();
            }
            assert!(receiver.pending.is_empty());
            assert_eq!(receiver.pending_pos, 0);
        }

        #[test]
        fn iq_gap_produces_no_signal_even_without_output() {
            let mut receiver = receiver_with(FakeIqSource::blocks(vec![IqBlock {
                samples: vec![100u8; 4096],
                gap: true,
            }]));

            assert_eq!(
                receiver.recv_timeout(Duration::from_millis(1)).unwrap(),
                RxEvent::NoSignal
            );
            assert_eq!(receiver.decoder.stats().gaps, 1);
            assert!(!receiver.had_ts);
        }

        #[test]
        fn no_new_iq_is_pending_not_lock_loss() {
            let mut receiver = receiver_with(FakeIqSource::blocks(Vec::new()));
            assert_eq!(
                receiver.recv_timeout(Duration::from_millis(1)).unwrap(),
                RxEvent::Pending
            );
        }

        #[test]
        fn stop_is_idempotent_and_drops_the_source() {
            let mut receiver = receiver_with(FakeIqSource::blocks(Vec::new()));
            receiver.stop().expect("first stop");
            receiver.stop().expect("second stop");
            assert!(receiver.source.is_none());
        }

        #[test]
        fn stop_after_processing_is_idempotent_and_keeps_processed_samples() {
            let mut receiver = receiver_with(FakeIqSource::blocks(vec![IqBlock {
                samples: vec![128u8; 4096],
                gap: false,
            }]));

            // Process one whole fake block so the decoder has real progress to
            // report; silence produces no lock, so the event stays pending.
            assert_eq!(
                receiver.recv_timeout(Duration::from_millis(1)).unwrap(),
                RxEvent::Pending
            );
            let processed = receiver.decoder.stats();
            assert_eq!(processed.input_bytes, 4096);
            assert_eq!(processed.input_samples, 2048);

            // `stop` logs that final snapshot, tears the source down and stays
            // idempotent without changing the decoder counters.
            receiver.stop().expect("first stop");
            receiver.stop().expect("second stop");
            assert!(receiver.source.is_none());
            assert_eq!(receiver.decoder.stats().input_samples, 2048);
        }
    }
}
