//! `ts-stream`: ISDB-T one-seg transport-stream HTTP streamer for RTL-SDR.
//!
//! Layering:
//!
//! * [`rtl`] loads `librtlsdr`, opens/tunes a device and produces raw IQ blocks
//!   (bounded queue, gap flags, cancel/join) on its own thread.
//! * [`demod::receiver`] is the production `u8` IQ -> 188-byte TS decoder: front
//!   end normalisation / DC block / rational resampling, [`demod::sync`], then
//!   [`demod::frame`] TMCC frame acquisition, [`demod::carriers`] coherent or
//!   DQPSK differential demodulation, [`demod::control`] time de-interleaving
//!   and [`demod::coding`] FEC / TS framing.
//! * [`session`] owns the single stream slot, the `tuning`/`locking`/... state
//!   machine, the bounded TS queue and the blocking worker thread. Its production
//!   backend (`session::rtl_backend`) wires [`rtl::Receiver`] into
//!   [`demod::receiver::Decoder`]; the small [`session::Backend`] trait lets
//!   tests inject deterministic mocks instead.
//! * [`http`] maps HTTP/1.1 requests onto the session (`/version`, `/stream`)
//!   and never fabricates TS data.
//! * [`cli`] is the `clap` command surface.

pub mod cli;
pub mod demod;
pub mod dsp;
pub mod fec;
pub mod http;
pub mod rtl;
pub mod session;

/// Version reported by `ts-stream --version` and `GET /version`.
///
/// The release workflow compiles with `TS_STREAM_VERSION` set to the pushed git
/// tag, so released binaries report that tag. Every other build falls back to
/// the crate version declared in `Cargo.toml`. Keeping this as the single
/// source of truth prevents the CLI and the HTTP surface from drifting apart.
pub const VERSION: &str = match option_env!("TS_STREAM_VERSION") {
    Some(tag) => tag,
    None => env!("CARGO_PKG_VERSION"),
};
