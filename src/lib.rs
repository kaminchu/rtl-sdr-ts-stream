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
