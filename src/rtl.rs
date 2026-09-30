//! Minimal, safe-by-default FFI boundary around `librtlsdr`.
//!
//! This module is the *only* place in the crate that talks to the C library.
//! It exposes a small, owned abstraction:
//!
//! ```text
//! RtlLibrary::load()?
//!   .devices()?              -> Vec<DeviceInfo>   (enumeration)
//!   .open(index, hz, gain, ppm, rate)? -> Receiver
//!
//! Receiver::recv_timeout(dur)? -> Option<IqBlock>  (None == timeout)
//! Receiver::stop()?                                 (cancel + join + close)
//! Receiver::drop()                                  (same cleanup, best effort)
//! ```
//!
//! # Threading / ownership model
//!
//! `librtlsdr` requires persistent handle state for the duration of a synchronous
//! `rtlsdr_read_async` call. The callback it invokes is not thread-safe with the
//! caller, so this module dedicates exactly one OS thread to the blocking read:
//!
//! * [`RtlLibrary`] owns the `dlopen`ed [`libloading::Library`] plus resolved
//!   function pointers, held behind an [`Arc`]. Every [`Receiver`] and every USB
//!   reader thread keeps an `Arc` clone, so the shared object can never be
//!   `dlclose`d while a device handle is alive.
//! * [`Receiver::open`](RtlLibrary::open) spawns the reader thread, which calls
//!   `rtlsdr_read_async` (blocking) and returns when cancelled or on USB failure.
//! * The callback runs on that reader thread only. It must never block, so it
//!   copies the buffer and calls [`std::sync::mpsc::SyncSender::try_send`].
//! * [`Receiver::stop`] may be called from a different thread than the reader.
//!   It drives `rtlsdr_cancel_async` and then joins. `rtlsdr_close` is only ever
//!   called *after* the join completes, so the handle is never used after free.
//!
//! # Safety invariants documented on the `unsafe` blocks
//!
//! * [`Receiver`] is `Send` (it can move between threads) but deliberately **not**
//!   `Sync`: `std::sync::mpsc::Receiver` is single-consumer. `recv_timeout`
//!   therefore takes `&self` but must not be called concurrently from multiple
//!   threads; this is enforced by the missing `Sync` impl.
//! * The raw `rtlsdr_dev_t *` is shared with the reader thread and with the
//!   stopping thread in the exact pattern librtlsdr supports: concurrent
//!   `rtlsdr_cancel_async` during a `rtlsdr_read_async`. No other cross-thread
//!   handle access happens.
//! * The callback pointer passed to `rtlsdr_read_async` points at a
//!   [`ReaderCtx`] that lives on the reader thread's stack and outlives the call.
//! * The reader thread is always joined before `rtlsdr_close`; a panic in the
//!   thread is reported as an error but still followed by `rtlsdr_close`.
//!
//! # Cancellation startup race
//!
//! `rtlsdr_read_async` resets its internal `async_cancel` flag *after* it is
//! entered. A single `rtlsdr_cancel_async` issued between thread spawn and entry
//! can therefore be lost, leaving the join to block forever. To avoid that,
//! cancellation is repeated until the reader thread reports completion (see
//! [`Receiver::stop`]). This is the only safe strategy, because abandoning a
//! still-running `rtlsdr_read_async` would free the handle out from under it.
//!
//! No signal handling is performed here; callers own signal policy.

use std::ffi::CStr;
use std::io::Write;
use std::os::raw::{c_char, c_int, c_uchar, c_void};
use std::ptr;
use std::sync::{
    atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering},
    mpsc::{self, Receiver as MpscReceiver, RecvTimeoutError, SyncSender, TrySendError},
    Arc,
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use libloading::{Library, Symbol};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One IQ buffer handed out by [`Receiver::recv_timeout`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IqBlock {
    /// Raw interleaved 8-bit IQ samples exactly as delivered by librtlsdr.
    pub samples: Vec<u8>,
    /// `true` when at least one earlier buffer was dropped because the bounded
    /// queue was full. The samples are still valid, but they are not contiguous
    /// with the previously delivered block.
    pub gap: bool,
}

/// A device visible to `rtlsdr_get_device_count` / friends.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Index accepted by [`RtlLibrary::open`].
    pub index: u32,
    /// Best-effort human readable name. Never empty; if librtlsdr cannot supply
    /// one, an explicit `"(name unavailable)"` placeholder is used.
    pub name: String,
    /// USB serial string if it could be read, otherwise `None` (explicit).
    pub serial: Option<String>,
}

/// Cumulative counters for one [`Receiver`], useful for diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ReceiverStats {
    /// Buffers successfully delivered to the queue.
    pub delivered_blocks: u64,
    /// Buffers dropped because the bounded queue was full.
    pub overflows: u64,
    /// Samples bytes dropped together with `overflows`.
    pub dropped_bytes: u64,
}

/// Errors produced by the RTL boundary.
#[derive(Debug)]
pub enum RtlError {
    /// The shared library could not be loaded.
    Load { message: String },
    /// A required symbol was absent or could not be resolved.
    Symbol { symbol: String, message: String },
    /// A caller supplied argument failed local validation.
    InvalidArgument(String),
    /// A librtlsdr call returned a failure code.
    Driver { operation: &'static str, code: i32 },
    /// The dedicated reader thread could not be started / panicked.
    Thread(String),
    /// The reader ended without the caller having requested a stop.
    ReadAsync { code: i32, stopped: bool },
}

impl RtlError {
    fn driver(operation: &'static str, code: i32) -> Self {
        RtlError::Driver { operation, code }
    }
}

impl std::fmt::Display for RtlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RtlError::Load { message } => write!(f, "failed to load librtlsdr: {message}"),
            RtlError::Symbol { symbol, message } => {
                write!(
                    f,
                    "failed to resolve librtlsdr symbol `{symbol}`: {message}"
                )
            }
            RtlError::InvalidArgument(message) => write!(f, "invalid RTL argument: {message}"),
            RtlError::Driver { operation, code } => {
                write!(f, "librtlsdr `{operation}` failed with code {code}")
            }
            RtlError::Thread(message) => write!(f, "RTL-SDR reader thread error: {message}"),
            RtlError::ReadAsync { code, stopped } => {
                if *stopped {
                    write!(f, "RTL-SDR async reader stopped (code {code})")
                } else {
                    write!(
                        f,
                        "RTL-SDR USB reader terminated unexpectedly (code {code})"
                    )
                }
            }
        }
    }
}

impl std::error::Error for RtlError {}

/// Convenience alias for fallible RTL operations.
pub type Result<T> = std::result::Result<T, RtlError>;

/// Loaded `librtlsdr` plus its resolved symbols.
///
/// The struct is cheap to clone-through-`Arc` internally; callers normally hold
/// it for the lifetime of the process and borrow it to enumerate/open devices.
pub struct RtlLibrary {
    inner: Arc<LibraryInner>,
}

// ---------------------------------------------------------------------------
// Function pointer table
// ---------------------------------------------------------------------------

type GetDeviceCountFn = unsafe extern "C" fn() -> u32;
type GetDeviceNameFn = unsafe extern "C" fn(u32) -> *const c_char;
type GetDeviceUsbStringsFn =
    unsafe extern "C" fn(u32, *mut c_char, *mut c_char, *mut c_char) -> c_int;
type OpenFn = unsafe extern "C" fn(*mut *mut c_void, u32) -> c_int;
type CloseFn = unsafe extern "C" fn(*mut c_void) -> c_int;
type SetCenterFreqFn = unsafe extern "C" fn(*mut c_void, u32) -> c_int;
type SetSampleRateFn = unsafe extern "C" fn(*mut c_void, u32) -> c_int;
type SetFreqCorrectionFn = unsafe extern "C" fn(*mut c_void, c_int) -> c_int;
type SetTunerGainModeFn = unsafe extern "C" fn(*mut c_void, c_int) -> c_int;
type SetTunerGainFn = unsafe extern "C" fn(*mut c_void, c_int) -> c_int;
type GetTunerGainsFn = unsafe extern "C" fn(*mut c_void, *mut c_int) -> c_int;
type ResetBufferFn = unsafe extern "C" fn(*mut c_void) -> c_int;
type ReadCallback = extern "C" fn(*mut c_uchar, u32, *mut c_void);
type ReadAsyncFn =
    unsafe extern "C" fn(*mut c_void, Option<ReadCallback>, *mut c_void, u32, u32) -> c_int;
type CancelAsyncFn = unsafe extern "C" fn(*mut c_void) -> c_int;

struct LibraryInner {
    // Kept alive for as long as any Receiver / reader thread exists.
    _lib: Library,
    get_device_count: GetDeviceCountFn,
    get_device_name: GetDeviceNameFn,
    get_device_usb_strings: GetDeviceUsbStringsFn,
    open: OpenFn,
    close: CloseFn,
    set_center_freq: SetCenterFreqFn,
    set_sample_rate: SetSampleRateFn,
    set_freq_correction: SetFreqCorrectionFn,
    set_tuner_gain_mode: SetTunerGainModeFn,
    set_tuner_gain: SetTunerGainFn,
    get_tuner_gains: GetTunerGainsFn,
    reset_buffer: ResetBufferFn,
    read_async: ReadAsyncFn,
    cancel_async: CancelAsyncFn,
}

// ---------------------------------------------------------------------------
// Tuning constants / helpers (ABI independent, unit tested)
// ---------------------------------------------------------------------------

/// Shared objects tried in order at [`RtlLibrary::load`] time.
const LIBRARY_CANDIDATES: [&str; 3] = ["librtlsdr.so.2", "librtlsdr.so.0", "librtlsdr.so"];

/// Bytes reserved per USB string when enumerating devices.
const STRING_BUFFER_LEN: usize = 256;

/// Conservative tuner frequency window (E4000 covers the top end; R820T stops
/// near 1.766 GHz and will report its own error for anything it cannot tune).
const MIN_FREQUENCY_HZ: u32 = 24_000_000;
const MAX_FREQUENCY_HZ: u32 = 2_200_000_000;

/// Range librtlsdr accepts for `rtlsdr_set_sample_rate`.
const MIN_SAMPLE_RATE_HZ: u32 = 225_000;
const MAX_SAMPLE_RATE_HZ: u32 = 3_200_000;

/// Frequency correction is only meaningful in a narrow band; reject typos.
const MAX_ABS_PPM: i32 = 1_000;

/// Broad manual-gain guard used only when the tuner does not report a list.
const MAX_ABS_GAIN_DB: f64 = 100.0;

/// Bounded IQ queue depth, measured in callback buffers.
const QUEUE_CAPACITY: usize = 16;
/// Buffers requested from librtlsdr.
const BUFFER_COUNT: u32 = 4;
/// Bytes per callback buffer (multiple of 512, a librtlsdr requirement).
const BUFFER_LEN: u32 = 262_144;

/// Delay between repeated cancellation attempts.
const CANCEL_RETRY_INTERVAL: Duration = Duration::from_millis(2);
/// How often to emit a "still waiting" diagnostic while cancelling.
const CANCEL_LOG_INTERVAL: Duration = Duration::from_secs(2);

fn db_to_tenths(db: f64) -> i32 {
    (db * 10.0).round() as i32
}

fn validate_frequency(frequency_hz: u32) -> Result<()> {
    if !(MIN_FREQUENCY_HZ..=MAX_FREQUENCY_HZ).contains(&frequency_hz) {
        return Err(RtlError::InvalidArgument(format!(
            "frequency {frequency_hz} Hz outside supported range \
             {MIN_FREQUENCY_HZ}..={MAX_FREQUENCY_HZ} Hz"
        )));
    }
    Ok(())
}

fn validate_sample_rate(sample_rate: u32) -> Result<()> {
    if !(MIN_SAMPLE_RATE_HZ..=MAX_SAMPLE_RATE_HZ).contains(&sample_rate) {
        return Err(RtlError::InvalidArgument(format!(
            "sample rate {sample_rate} Hz outside supported range \
             {MIN_SAMPLE_RATE_HZ}..={MAX_SAMPLE_RATE_HZ} Hz"
        )));
    }
    Ok(())
}

fn validate_ppm(ppm: i32) -> Result<()> {
    if (ppm as i64).abs() > MAX_ABS_PPM as i64 {
        return Err(RtlError::InvalidArgument(format!(
            "ppm {ppm} outside supported range -{MAX_ABS_PPM}..={MAX_ABS_PPM}"
        )));
    }
    Ok(())
}

fn validate_gain(gain: Option<f64>) -> Result<()> {
    if let Some(gain_db) = gain {
        if !gain_db.is_finite() {
            return Err(RtlError::InvalidArgument(format!(
                "gain {gain_db} must be a finite number"
            )));
        }
        if gain_db.abs() > MAX_ABS_GAIN_DB {
            return Err(RtlError::InvalidArgument(format!(
                "gain {gain_db} dB outside supported range \
                 -{MAX_ABS_GAIN_DB}..={MAX_ABS_GAIN_DB} dB"
            )));
        }
    }
    Ok(())
}

/// Pick the tuner gain (in tenths of dB) closest to `requested_db`.
///
/// If the tuner reports a supported list, snap to it (the device only accepts
/// those exact values). Otherwise fall back to the requested value after range
/// validation.
fn choose_gain(supported_tenths: &[i32], requested_db: f64) -> Result<i32> {
    if !requested_db.is_finite() {
        return Err(RtlError::InvalidArgument(format!(
            "gain {requested_db} must be a finite number"
        )));
    }
    if requested_db.abs() > MAX_ABS_GAIN_DB {
        return Err(RtlError::InvalidArgument(format!(
            "gain {requested_db} dB outside supported range \
             -{MAX_ABS_GAIN_DB}..={MAX_ABS_GAIN_DB} dB"
        )));
    }

    if supported_tenths.is_empty() {
        return Ok(db_to_tenths(requested_db));
    }

    let requested = requested_db * 10.0;
    supported_tenths
        .iter()
        .copied()
        .min_by(|a, b| {
            let da = (*a as f64 - requested).abs();
            let db = (*b as f64 - requested).abs();
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
        .ok_or_else(|| RtlError::InvalidArgument("tuner reported an empty gain list".into()))
}

/// Convert a NUL-terminated `c_char` buffer to a trimmed `String`, or `None`
/// when empty / missing.
fn c_buf_to_string(buf: &[c_char]) -> Option<String> {
    let bytes: Vec<u8> = buf.iter().map(|c| *c as u8).collect();
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    let text = String::from_utf8_lossy(&bytes[..end]).trim().to_string();
    (!text.is_empty()).then_some(text)
}

/// Convert a C string pointer to an owned trimmed `String`.
fn cstr_to_string(ptr: *const c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    // SAFETY: the caller guarantees `ptr` points at a NUL-terminated C string
    // valid for the duration of this call (librtlsdr owns static/returned data).
    let text = unsafe { CStr::from_ptr(ptr) }
        .to_string_lossy()
        .trim()
        .to_string();
    (!text.is_empty()).then_some(text)
}

// ---------------------------------------------------------------------------
// Queue hand-off (pure, unit tested)
// ---------------------------------------------------------------------------

#[derive(Default)]
struct ReaderCounters {
    /// A previous buffer was dropped; the next delivered block must carry `gap`.
    pending_gap: AtomicBool,
    delivered: AtomicU64,
    overflows: AtomicU64,
    dropped_bytes: AtomicU64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EnqueueOutcome {
    Delivered,
    Overflowed { overflows: u64 },
    Disconnected,
}

/// Non-blocking hand-off used by the USB callback.
///
/// On overflow the buffer is dropped, counters are updated, and the *next*
/// successfully delivered block is flagged with `gap = true`. No block is ever
/// silently reported as contiguous across a drop.
fn enqueue_block(
    tx: &SyncSender<IqBlock>,
    counters: &ReaderCounters,
    samples: Vec<u8>,
) -> EnqueueOutcome {
    let gap = counters.pending_gap.swap(false, Ordering::AcqRel);
    match tx.try_send(IqBlock { samples, gap }) {
        Ok(()) => {
            counters.delivered.fetch_add(1, Ordering::Relaxed);
            EnqueueOutcome::Delivered
        }
        Err(TrySendError::Full(block)) => {
            counters.pending_gap.store(true, Ordering::Release);
            let overflows = counters.overflows.fetch_add(1, Ordering::Relaxed) + 1;
            counters
                .dropped_bytes
                .fetch_add(block.samples.len() as u64, Ordering::Relaxed);
            EnqueueOutcome::Overflowed { overflows }
        }
        Err(TrySendError::Disconnected(block)) => {
            // Consumer is gone; restore the pending gap so a later successful
            // send (if any) still reflects the discontinuity.
            if block.gap {
                counters.pending_gap.store(true, Ordering::Release);
            }
            EnqueueOutcome::Disconnected
        }
    }
}

// ---------------------------------------------------------------------------
// Reader thread plumbing
// ---------------------------------------------------------------------------

struct ReaderCtx {
    tx: SyncSender<IqBlock>,
    counters: Arc<ReaderCounters>,
}

/// Wrapper making the raw `rtlsdr_dev_t *` movable to the reader thread.
struct SendPtr(*mut c_void);

impl SendPtr {
    /// Accessor kept on the wrapper so closure capture grabs the whole wrapper
    /// (Rust 2021 disjoint capture would otherwise move the bare pointer field
    /// and lose the `Send` impl).
    fn as_ptr(&self) -> *mut c_void {
        self.0
    }
}

// SAFETY: the pointer is only dereferenced by librtlsdr inside the dedicated
// reader thread (read_async) and by the stopping thread through the documented
// `rtlsdr_cancel_async` / `rtlsdr_close` API. `Receiver` guarantees close only
// after join, so the pointer never outlives the handle it references.
unsafe impl Send for SendPtr {}

/// librtlsdr read callback. Runs on the reader thread only.
///
/// Must not block, allocate unboundedly, log, or panic: `try_send` is
/// non-blocking and `to_vec` copies exactly `len` bytes. Logging from here is
/// deliberately avoided because `eprintln!` panics when stderr is unwritable,
/// and a panic inside this `extern "C"` frame must never unwind into
/// librtlsdr. The whole body is guarded by [`std::panic::catch_unwind`]; any
/// panic is converted to [`std::process::abort`]. The one allocation (`to_vec`)
/// aborts the process on allocation failure via Rust's allocation-error
/// handler, so that path cannot unwind either. Overflow accounting stays in the
/// atomics in [`ReaderCounters`] and is surfaced by [`Receiver::stats`].
extern "C" fn read_callback(buf: *mut c_uchar, len: u32, ctx: *mut c_void) {
    let result = std::panic::catch_unwind(|| {
        if ctx.is_null() || buf.is_null() || len == 0 {
            return;
        }
        // SAFETY: `ctx` was set by `spawn_reader` to point at a `ReaderCtx` that
        // lives on the reader thread's stack for the whole duration of
        // `rtlsdr_read_async`, and librtlsdr invokes the callback synchronously
        // on that thread. `buf` is valid for `len` bytes as documented by
        // librtlsdr.
        let (ctx, samples) = unsafe {
            let ctx = &*(ctx as *const ReaderCtx);
            let samples = std::slice::from_raw_parts(buf, len as usize).to_vec();
            (ctx, samples)
        };

        // The returned outcome is intentionally discarded: overflow counts live
        // in the atomic counters, not in a logging side effect.
        let _ = enqueue_block(&ctx.tx, &ctx.counters, samples);
    });
    if result.is_err() {
        std::process::abort();
    }
}

fn spawn_reader(
    inner: Arc<LibraryInner>,
    dev: *mut c_void,
    tx: SyncSender<IqBlock>,
    counters: Arc<ReaderCounters>,
    finished: Arc<AtomicBool>,
    read_result: Arc<AtomicI32>,
) -> std::io::Result<JoinHandle<()>> {
    let dev = SendPtr(dev);
    thread::Builder::new()
        .name("rtl-sdr-usb".to_string())
        .spawn(move || {
            let ctx = ReaderCtx { tx, counters };
            // SAFETY: `dev.0` is a valid, open handle owned by the `Receiver`.
            // The callback ctx outlives the call. `rtlsdr_read_async` blocks on
            // this thread until cancelled or the USB transfer fails.
            let ret = unsafe {
                (inner.read_async)(
                    dev.as_ptr(),
                    Some(read_callback),
                    &ctx as *const ReaderCtx as *mut c_void,
                    BUFFER_COUNT,
                    BUFFER_LEN,
                )
            };
            read_result.store(ret, Ordering::Release);
            finished.store(true, Ordering::Release);
            // `ctx` drops here, disconnecting the channel; the consumer observes
            // this as a terminal error via `recv_timeout`.
        })
}

/// Repeatedly request cancellation until the reader thread has returned.
///
/// A single early `rtlsdr_cancel_async` may race with `rtlsdr_read_async`
/// clearing its internal flag at startup. Repeating is the only safe remedy:
/// giving up would leave a live `rtlsdr_read_async` using a handle we are about
/// to close. Diagnostics are emitted at most every [`CANCEL_LOG_INTERVAL`].
fn cancel_until_finished(inner: &LibraryInner, dev: *mut c_void, finished: &AtomicBool) {
    let mut last_log = Instant::now();
    loop {
        if finished.load(Ordering::Acquire) {
            return;
        }
        // SAFETY: `dev` is a valid open handle; cancel is documented safe to
        // call concurrently with an in-flight `rtlsdr_read_async`.
        unsafe {
            let _ = (inner.cancel_async)(dev);
        }
        if finished.load(Ordering::Acquire) {
            return;
        }
        if last_log.elapsed() >= CANCEL_LOG_INTERVAL {
            // A best-effort diagnostic: a broken stderr must not panic the
            // stopping thread (or a destructor) while a reader is still live.
            let _ = writeln!(
                std::io::stderr(),
                "rtl: waiting for the RTL-SDR async reader to stop..."
            );
            last_log = Instant::now();
        }
        thread::sleep(CANCEL_RETRY_INTERVAL);
    }
}

// ---------------------------------------------------------------------------
// RtlLibrary implementation
// ---------------------------------------------------------------------------

impl RtlLibrary {
    /// Load `librtlsdr` (`.so.2`, then `.so.0`, then `.so`) and resolve symbols.
    ///
    /// Returns [`RtlError::Load`] if no candidate could be opened and
    /// [`RtlError::Symbol`] if a required symbol is missing.
    pub fn load() -> Result<Self> {
        let mut attempts = Vec::with_capacity(LIBRARY_CANDIDATES.len());
        for candidate in LIBRARY_CANDIDATES {
            // SAFETY: dlopen of a well-known library name; no user-controlled
            // path is involved.
            match unsafe { Library::new(candidate) } {
                Ok(lib) => {
                    let inner = Self::resolve(lib)?;
                    return Ok(Self {
                        inner: Arc::new(inner),
                    });
                }
                Err(err) => attempts.push(format!("{candidate}: {err}")),
            }
        }
        Err(RtlError::Load {
            message: format!("no librtlsdr candidate loaded ({})", attempts.join("; ")),
        })
    }

    fn resolve(lib: Library) -> Result<LibraryInner> {
        macro_rules! resolve {
            ($name:literal, $ty:ty) => {{
                // SAFETY: the symbol name and type are part of librtlsdr's
                // stable public C API. The `Symbol` borrow ends at the end of
                // this block once the function pointer is copied out.
                let symbol: Symbol<$ty> = unsafe { lib.get(concat!($name, "\0").as_bytes()) }
                    .map_err(|err| RtlError::Symbol {
                        symbol: $name.to_string(),
                        message: err.to_string(),
                    })?;
                *symbol
            }};
        }

        Ok(LibraryInner {
            get_device_count: resolve!("rtlsdr_get_device_count", GetDeviceCountFn),
            get_device_name: resolve!("rtlsdr_get_device_name", GetDeviceNameFn),
            get_device_usb_strings: resolve!(
                "rtlsdr_get_device_usb_strings",
                GetDeviceUsbStringsFn
            ),
            open: resolve!("rtlsdr_open", OpenFn),
            close: resolve!("rtlsdr_close", CloseFn),
            set_center_freq: resolve!("rtlsdr_set_center_freq", SetCenterFreqFn),
            set_sample_rate: resolve!("rtlsdr_set_sample_rate", SetSampleRateFn),
            set_freq_correction: resolve!("rtlsdr_set_freq_correction", SetFreqCorrectionFn),
            set_tuner_gain_mode: resolve!("rtlsdr_set_tuner_gain_mode", SetTunerGainModeFn),
            set_tuner_gain: resolve!("rtlsdr_set_tuner_gain", SetTunerGainFn),
            get_tuner_gains: resolve!("rtlsdr_get_tuner_gains", GetTunerGainsFn),
            reset_buffer: resolve!("rtlsdr_reset_buffer", ResetBufferFn),
            read_async: resolve!("rtlsdr_read_async", ReadAsyncFn),
            cancel_async: resolve!("rtlsdr_cancel_async", CancelAsyncFn),
            _lib: lib,
        })
    }

    /// Enumerate connected RTL-SDR devices.
    ///
    /// `serial` is `None` when the USB strings could not be read (for example a
    /// busy or permission-denied device); the CLI is expected to render that as
    /// "unavailable" rather than an empty string.
    pub fn devices(&self) -> Result<Vec<DeviceInfo>> {
        let index_count = unsafe { (self.inner.get_device_count)() };
        let mut devices = Vec::with_capacity(index_count as usize);

        for index in 0..index_count {
            let mut name = unsafe { cstr_to_string((self.inner.get_device_name)(index)) };

            let mut manufact = [0 as c_char; STRING_BUFFER_LEN];
            let mut product = [0 as c_char; STRING_BUFFER_LEN];
            let mut serial_buf = [0 as c_char; STRING_BUFFER_LEN];

            // SAFETY: all three pointers reference distinct, fixed-size buffers
            // that librtlsdr writes at most `STRING_BUFFER_LEN` bytes into (its
            // documented contract).
            let ret = unsafe {
                (self.inner.get_device_usb_strings)(
                    index,
                    manufact.as_mut_ptr(),
                    product.as_mut_ptr(),
                    serial_buf.as_mut_ptr(),
                )
            };

            let serial = if ret == 0 {
                if name.is_none() {
                    name = c_buf_to_string(&product);
                }
                c_buf_to_string(&serial_buf)
            } else {
                None
            };

            devices.push(DeviceInfo {
                index,
                name: name.unwrap_or_else(|| format!("RTL-SDR device {index} (name unavailable)")),
                serial,
            });
        }

        Ok(devices)
    }

    /// Open `index`, configure tuning, and start the dedicated USB reader.
    ///
    /// * `frequency_hz` is the center frequency.
    /// * `gain` is `None` for automatic gain, or a finite manual gain in dB.
    ///   When the tuner reports supported steps, the request is snapped to the
    ///   nearest one; otherwise it is passed through.
    /// * `ppm` is the frequency correction in parts-per-million.
    /// * `sample_rate` is the IQ sample rate in Hz.
    ///
    /// Any failure after `rtlsdr_open` closes the partially configured handle
    /// before returning, so no device or thread is leaked.
    pub fn open(
        &self,
        index: u32,
        frequency_hz: u32,
        gain: Option<f64>,
        ppm: i32,
        sample_rate: u32,
    ) -> Result<Receiver> {
        validate_frequency(frequency_hz)?;
        validate_sample_rate(sample_rate)?;
        validate_ppm(ppm)?;
        validate_gain(gain)?;

        let inner = Arc::clone(&self.inner);

        let mut dev: *mut c_void = ptr::null_mut();
        // SAFETY: `rtlsdr_open` initializes `dev` on success.
        let ret = unsafe { (inner.open)(&mut dev, index) };
        if ret != 0 || dev.is_null() {
            return Err(RtlError::driver("rtlsdr_open", ret));
        }

        if let Err(err) = configure(&inner, dev, frequency_hz, gain, ppm, sample_rate) {
            // SAFETY: `dev` was opened above and is closed exactly once here.
            unsafe {
                let _ = (inner.close)(dev);
            }
            return Err(err);
        }

        let (tx, rx) = mpsc::sync_channel::<IqBlock>(QUEUE_CAPACITY);
        let counters = Arc::new(ReaderCounters::default());
        let finished = Arc::new(AtomicBool::new(false));
        let read_result = Arc::new(AtomicI32::new(0));

        let thread = match spawn_reader(
            Arc::clone(&inner),
            dev,
            tx,
            Arc::clone(&counters),
            Arc::clone(&finished),
            Arc::clone(&read_result),
        ) {
            Ok(handle) => handle,
            Err(err) => {
                // SAFETY: the reader never started, so closing here cannot race
                // with read_async. `dev` is valid and closed exactly once.
                unsafe {
                    let _ = (inner.close)(dev);
                }
                return Err(RtlError::Thread(format!(
                    "failed to start RTL-SDR reader thread: {err}"
                )));
            }
        };

        Ok(Receiver {
            inner,
            dev,
            rx,
            thread: Some(thread),
            finished,
            read_result,
            stop_requested: Arc::new(AtomicBool::new(false)),
            counters,
        })
    }
}

/// Apply tuning to an open handle. On error the caller closes the handle.
fn configure(
    inner: &LibraryInner,
    dev: *mut c_void,
    frequency_hz: u32,
    gain: Option<f64>,
    ppm: i32,
    sample_rate: u32,
) -> Result<()> {
    check("rtlsdr_set_sample_rate", unsafe {
        (inner.set_sample_rate)(dev, sample_rate)
    })?;
    // ppm before tuning so the center frequency is corrected on the first try.
    check("rtlsdr_set_freq_correction", unsafe {
        (inner.set_freq_correction)(dev, ppm)
    })?;
    check("rtlsdr_set_center_freq", unsafe {
        (inner.set_center_freq)(dev, frequency_hz)
    })?;

    match gain {
        Some(gain_db) => {
            let supported = supported_gains(inner, dev);
            let tenths = choose_gain(&supported, gain_db)?;
            check("rtlsdr_set_tuner_gain_mode", unsafe {
                (inner.set_tuner_gain_mode)(dev, 1)
            })?;
            check("rtlsdr_set_tuner_gain", unsafe {
                (inner.set_tuner_gain)(dev, tenths)
            })?;
        }
        None => {
            check("rtlsdr_set_tuner_gain_mode", unsafe {
                (inner.set_tuner_gain_mode)(dev, 0)
            })?;
        }
    }

    check("rtlsdr_reset_buffer", unsafe { (inner.reset_buffer)(dev) })?;
    Ok(())
}

fn check(operation: &'static str, code: i32) -> Result<()> {
    if code != 0 {
        Err(RtlError::driver(operation, code))
    } else {
        Ok(())
    }
}

/// Query tuner gain steps (tenths of dB). Empty when the tuner reports none.
fn supported_gains(inner: &LibraryInner, dev: *mut c_void) -> Vec<i32> {
    // SAFETY: passing a null buffer asks librtlsdr for the required count.
    let count = unsafe { (inner.get_tuner_gains)(dev, ptr::null_mut()) };
    if count <= 0 {
        return Vec::new();
    }
    let mut gains = vec![0 as c_int; count as usize];
    // SAFETY: `gains` has capacity for exactly `count` entries.
    let got = unsafe { (inner.get_tuner_gains)(dev, gains.as_mut_ptr()) };
    if got <= 0 {
        return Vec::new();
    }
    gains.truncate(got as usize);
    gains
}

// ---------------------------------------------------------------------------
// Receiver
// ---------------------------------------------------------------------------

/// An open RTL-SDR receiver driven by a dedicated USB thread.
///
/// `Receiver` is `Send` but not `Sync`: the underlying `mpsc::Receiver` is
/// single-consumer, so [`recv_timeout`](Receiver::recv_timeout) must not be
/// called concurrently from several threads.
pub struct Receiver {
    inner: Arc<LibraryInner>,
    dev: *mut c_void,
    rx: MpscReceiver<IqBlock>,
    thread: Option<JoinHandle<()>>,
    finished: Arc<AtomicBool>,
    read_result: Arc<AtomicI32>,
    stop_requested: Arc<AtomicBool>,
    counters: Arc<ReaderCounters>,
}

// SAFETY: every field is `Send` except the raw `dev` pointer, which is only
// used with librtlsdr's thread-safe `read_async` / `cancel_async` contract and
// is closed after the reader thread is joined. `Receiver` is intentionally not
// `Sync` so the single-consumer queue cannot be shared.
unsafe impl Send for Receiver {}

impl Receiver {
    /// Wait up to `timeout` for the next IQ block.
    ///
    /// Returns `Ok(None)` on timeout, `Ok(Some(block))` when data is available,
    /// and `Err` when the USB reader has terminated (device failure, unplug, or
    /// an explicit [`stop`](Receiver::stop)).
    pub fn recv_timeout(&self, timeout: Duration) -> Result<Option<IqBlock>> {
        match self.rx.recv_timeout(timeout) {
            Ok(block) => Ok(Some(block)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => {
                let code = self.read_result.load(Ordering::Acquire);
                let stopped = self.stop_requested.load(Ordering::Acquire);
                Err(RtlError::ReadAsync { code, stopped })
            }
        }
    }

    /// Snapshot the delivery / overflow counters.
    pub fn stats(&self) -> ReceiverStats {
        ReceiverStats {
            delivered_blocks: self.counters.delivered.load(Ordering::Relaxed),
            overflows: self.counters.overflows.load(Ordering::Relaxed),
            dropped_bytes: self.counters.dropped_bytes.load(Ordering::Relaxed),
        }
    }

    /// Cancel the reader, join it, then close the device handle.
    ///
    /// Idempotent. Cancellation is repeated until the reader thread reports
    /// completion to defeat the `rtlsdr_read_async` startup race. The handle is
    /// closed only after the join, so it can never be used after free.
    pub fn stop(&mut self) -> Result<()> {
        self.stop_requested.store(true, Ordering::Release);

        if self.thread.is_some() && !self.finished.load(Ordering::Acquire) {
            cancel_until_finished(&self.inner, self.dev, &self.finished);
        }

        let join_error = self
            .thread
            .take()
            .and_then(|handle| handle.join().err())
            .map(|_| "RTL-SDR reader thread panicked during shutdown".to_string());

        let close_result = self.close_device();

        if let Some(message) = join_error {
            return Err(RtlError::Thread(message));
        }
        close_result
    }

    fn close_device(&mut self) -> Result<()> {
        if self.dev.is_null() {
            return Ok(());
        }
        // SAFETY: the reader thread has been joined (or was never started), so
        // no other code can touch `dev`; this closes it exactly once.
        let ret = unsafe { (self.inner.close)(self.dev) };
        self.dev = ptr::null_mut();
        if ret != 0 {
            return Err(RtlError::driver("rtlsdr_close", ret));
        }
        Ok(())
    }
}

impl Drop for Receiver {
    fn drop(&mut self) {
        if let Err(err) = self.stop() {
            // Drop cannot propagate and must not panic; an ignored write keeps
            // a broken stderr from turning a best-effort diagnostic into an
            // abort during unwinding.
            let _ = writeln!(
                std::io::stderr(),
                "rtl: error while shutting down receiver: {err}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Tests (pure; never touch a device)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overflow_marks_next_delivered_block_as_gap() {
        let (tx, rx) = mpsc::sync_channel::<IqBlock>(1);
        let counters = ReaderCounters::default();

        assert_eq!(
            enqueue_block(&tx, &counters, vec![1, 2, 3, 4]),
            EnqueueOutcome::Delivered
        );
        assert_eq!(
            enqueue_block(&tx, &counters, vec![5, 6]),
            EnqueueOutcome::Overflowed { overflows: 1 }
        );

        let first = rx.recv().expect("first block");
        assert_eq!(first.samples, vec![1, 2, 3, 4]);
        assert!(!first.gap, "first block must not be a gap");

        // The next successful enqueue carries the gap flag.
        assert_eq!(
            enqueue_block(&tx, &counters, vec![7, 8, 9]),
            EnqueueOutcome::Delivered
        );
        let second = rx.recv().expect("second block");
        assert_eq!(second.samples, vec![7, 8, 9]);
        assert!(
            second.gap,
            "block after an overflow must be marked as a gap"
        );

        // A following block is contiguous again.
        assert_eq!(
            enqueue_block(&tx, &counters, vec![10]),
            EnqueueOutcome::Delivered
        );
        let third = rx.recv().expect("third block");
        assert!(!third.gap, "gap must be cleared after it is delivered once");

        let stats = ReceiverStats {
            delivered_blocks: counters.delivered.load(Ordering::Relaxed),
            overflows: counters.overflows.load(Ordering::Relaxed),
            dropped_bytes: counters.dropped_bytes.load(Ordering::Relaxed),
        };
        assert_eq!(stats.delivered_blocks, 3);
        assert_eq!(stats.overflows, 1);
        assert_eq!(stats.dropped_bytes, 2);
    }

    #[test]
    fn consecutive_overflows_keep_gap_pending() {
        let (tx, rx) = mpsc::sync_channel::<IqBlock>(3);
        let counters = ReaderCounters::default();

        for sample in 0..3u8 {
            assert_eq!(
                enqueue_block(&tx, &counters, vec![sample]),
                EnqueueOutcome::Delivered
            );
        }

        // Both of these are dropped; the gap stays pending across both.
        assert_eq!(
            enqueue_block(&tx, &counters, vec![9]),
            EnqueueOutcome::Overflowed { overflows: 1 }
        );
        assert_eq!(
            enqueue_block(&tx, &counters, vec![10]),
            EnqueueOutcome::Overflowed { overflows: 2 }
        );

        while rx.try_recv().is_ok() {}
        assert_eq!(
            enqueue_block(&tx, &counters, vec![11]),
            EnqueueOutcome::Delivered
        );
        let block = rx.recv().expect("block after overflows");
        assert!(block.gap);
    }

    #[test]
    fn disconnected_queue_is_reported_without_touching_counters() {
        let (tx, rx) = mpsc::sync_channel::<IqBlock>(1);
        drop(rx);
        let counters = ReaderCounters::default();

        assert_eq!(
            enqueue_block(&tx, &counters, vec![1, 2]),
            EnqueueOutcome::Disconnected
        );
        assert_eq!(counters.overflows.load(Ordering::Relaxed), 0);
        assert!(!counters.pending_gap.load(Ordering::Relaxed));
    }

    #[test]
    fn choose_gain_snaps_to_nearest_supported_step() {
        let supported = [0, 33, 71, 104, 142, 191, 240, 296]; // tenths of dB
        assert_eq!(choose_gain(&supported, 10.0).unwrap(), 104);
        assert_eq!(choose_gain(&supported, 7.0).unwrap(), 71);
        assert_eq!(choose_gain(&supported, 0.0).unwrap(), 0);
        assert_eq!(choose_gain(&supported, 100.0).unwrap(), 296);
    }

    #[test]
    fn choose_gain_without_supported_steps_uses_request() {
        assert_eq!(choose_gain(&[], 12.3).unwrap(), 123);
        assert_eq!(choose_gain(&[], -1.5).unwrap(), -15);
    }

    #[test]
    fn choose_gain_rejects_non_finite_and_extreme_values() {
        assert!(choose_gain(&[], f64::NAN).is_err());
        assert!(choose_gain(&[], f64::INFINITY).is_err());
        assert!(choose_gain(&[], MAX_ABS_GAIN_DB + 1.0).is_err());
    }

    #[test]
    fn validation_rejects_out_of_range_arguments() {
        assert!(validate_frequency(0).is_err());
        assert!(validate_frequency(MIN_FREQUENCY_HZ - 1).is_err());
        assert!(validate_frequency(MIN_FREQUENCY_HZ).is_ok());
        assert!(validate_frequency(MAX_FREQUENCY_HZ).is_ok());
        assert!(validate_frequency(MAX_FREQUENCY_HZ + 1).is_err());

        assert!(validate_sample_rate(MIN_SAMPLE_RATE_HZ - 1).is_err());
        assert!(validate_sample_rate(MIN_SAMPLE_RATE_HZ).is_ok());
        assert!(validate_sample_rate(MAX_SAMPLE_RATE_HZ).is_ok());
        assert!(validate_sample_rate(MAX_SAMPLE_RATE_HZ + 1).is_err());

        assert!(validate_ppm(MAX_ABS_PPM + 1).is_err());
        assert!(validate_ppm(-MAX_ABS_PPM - 1).is_err());
        assert!(validate_ppm(0).is_ok());

        assert!(validate_gain(Some(f64::NAN)).is_err());
        assert!(validate_gain(Some(f64::INFINITY)).is_err());
        assert!(validate_gain(Some(49.6)).is_ok());
        assert!(validate_gain(None).is_ok());
    }

    #[test]
    fn c_buffers_are_trimmed_and_validated() {
        let mut buf = [0 as c_char; 8];
        let text = b"  serial  ";
        for (slot, byte) in buf.iter_mut().zip(text.iter()) {
            *slot = *byte as c_char;
        }
        assert_eq!(c_buf_to_string(&buf).as_deref(), Some("serial"));

        let empty = [0 as c_char; 4];
        assert_eq!(c_buf_to_string(&empty), None);
    }

    #[test]
    fn cstr_helper_handles_null_and_whitespace() {
        assert_eq!(cstr_to_string(ptr::null()), None);

        let value = std::ffi::CString::new("  USB Device  ").unwrap();
        assert_eq!(
            cstr_to_string(value.as_ptr()).as_deref(),
            Some("USB Device")
        );
    }

    #[test]
    fn db_conversion_rounds_to_tenths() {
        assert_eq!(db_to_tenths(12.34), 123);
        assert_eq!(db_to_tenths(12.35), 124);
        assert_eq!(db_to_tenths(-0.05), -1);
        assert_eq!(db_to_tenths(0.0), 0);
    }
}
