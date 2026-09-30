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
//!
//! # Implementation status
//!
//! The ISDB-T demodulator is an original, from-scratch implementation of ARIB
//! STD-B31 v2.2-E1; no GNU Radio / `gr-isdbt` / Python source was consulted or
//! copied (see `docs/standards.md`). It is **wired into the production
//! backend**: `session::rtl_backend::RtlReceiver` feeds raw `u8` IQ to
//! [`demod::receiver::Decoder`] and forwards only real, RS-validated 188-byte TS
//! packets over `GET /stream`. It never invents TS and never reports a fake
//! lock. `503` remains only for an unavailable library/device or an unsupported
//! sample rate; a working device with no lockable signal ends the initial lock
//! attempt with `504` after the configured deadline.
//!
//! Verification so far is **synthetic**: a self-contained transmitter fixture
//! (TS -> RS -> energy dispersal -> interleaving -> K=7 / puncturing -> mapping
//! -> OFDM -> `u8` IQ) is replayed through the decoder in
//! `demod::replay_tests`. There is **no independent recorded-RF fixture**, and
//! hardware performance is unmeasured. TMCC parity is checked but not
//! error-corrected (a bad frame is rejected and re-acquired), the synchroniser
//! tracks timing and **fractional** CFO only, and dedicated sample-clock
//! tracking is not implemented. See `docs/implementation-status.md` and
//! `docs/acceptance.md`.

pub mod cli;
pub mod demod;
pub mod dsp;
pub mod fec;
pub mod http;
pub mod rtl;
pub mod session;
