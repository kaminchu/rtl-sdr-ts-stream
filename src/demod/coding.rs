//! ISDB-T channel-coding (receiver) for the centre (partial-reception) segment.
//!
//! Everything here is written from scratch for this crate. No GNU Radio,
//! `gr-isdbt`, Python or other existing demodulator/decoder source was consulted
//! or copied. The normative behaviour is transcribed from ARIB STD-B31 English
//! translation, version 2.2-E1 (the primary source used by this project; see
//! `docs/standards.md`). Printed page numbers below refer to that document.
//!
//! # Receiver chain owned by this module
//!
//! This module implements the *channel-coding* half of the FEC chain, i.e. the
//! part that runs **after** carrier soft demapping and **before** a TS packet
//! leaves the receiver. The transmitter order (ARIB STD-B31 §3.1, Fig. 3-2,
//! printed page 17) is, per hierarchical layer:
//!
//! ```text
//! RS(204,188)  ->  energy dispersal  ->  byte interleaving  ->  K=7
//! rate-1/2 convolutional coding  ->  puncturing  ->  bit interleaving  ->  mapping
//! ```
//!
//! so the inverse implemented here is:
//!
//! ```text
//! soft demapping (caller)
//!   -> bit de-interleaving          (inverse of §3.9.3, Figs. 3-12/14/16/18)
//!   -> depuncturing                 (inverse of Table 3-8)
//!   -> streaming K=7 soft Viterbi   (inverse of §3.8, Fig. 3-10)
//!   -> bits -> bytes, MSB first     (§3.1)
//!   -> 12-branch byte de-interleaving (inverse of §3.7, Fig. 3-9)
//!   -> energy-dispersal removal     (inverse of §3.5, Fig. 3-8)
//!   -> RS(204,188) decoding         (§3.3)
//!   -> 188-byte TS packet
//! ```
//!
//! Time and frequency de-interleaving (§3.11) and TMCC/AC decoding live in
//! [`crate::demod::control`]; they run before this module.
//!
//! # What is deliberately *not* implemented
//!
//! * Time/frequency de-interleaving and equalisation (upstream modules).
//! * Bit-level or OFDM-symbol acquisition. The caller is responsible for
//!   handing this decoder a soft stream whose first carrier symbol is the first
//!   symbol of an OFDM frame. The *byte* frame phase (where inside the frame the
//!   PRBS starts) is acquired from the stream itself by
//!   [`ChannelDecoder::with_mode`]; see [`packets_per_frame`].
//! * 90-degree carrier-phase ambiguity of coherent modulation. Only the global
//!   180-degree bit inversion (which maps the TS sync byte `0x47` to `0xB8`) is
//!   detected, by the validated sync search described in [`ChannelDecoder`].
//!
//! # Public API limitations
//!
//! * [`ChannelDecoder::reset`] resets every internal state including the PRBS.
//!   It is intended to be called only when the caller can vouch that the next
//!   soft bit is a fresh frame start. The first [`BYTE_INTERLEAVE_LATENCY`]
//!   (2244) de-interleaved bytes are a startup transient and are discarded, so
//!   the first ~11 transmission TSPs after a `reset()` are lost. A continuous
//!   multi-frame stream must **not** call `reset()` per frame: use
//!   [`ChannelDecoder::with_mode`], which re-initialises only the PRBS at each
//!   OFDM frame boundary while keeping the streaming bit/byte de-interleaver and
//!   Viterbi state alive.
//! * [`ChannelDecoder::flush`] drains the Viterbi traceback pipeline at the end
//!   of a stream. It is terminal: pushing more soft bits afterwards would
//!   re-emit already-flushed bits. It exists for tests/one-shot use.
//! * `LayerConfig::interleave` (the time-interleaving length `I` of Table 3-12)
//!   is validated for membership but otherwise unused: time de-interleaving is
//!   done by [`crate::demod::control`].
//! * Only packets that Reed-Solomon-decode *and* pass [`crate::fec::validate_ts_packet`]
//!   are returned. Nothing is ever fabricated.
//!
//! # Standards citations (printed pages of ARIB STD-B31 v2.2-E1)
//!
//! * §3.3 outer code, shortened RS(204,188): p. 22-23.
//! * §3.5 energy dispersal, PRBS `g(x) = X^15 + X^14 + 1`, init
//!   `100101010000000`, re-initialised every OFDM frame at the byte next to the
//!   transmission TSP sync byte: p. 24, Fig. 3-8.
//! * Table 3-3 data rate / transmission TSPs per OFDM frame, derived from the
//!   `96/192/384` data carriers and `204` symbols per frame: p. 15.
//! * §3.7 byte interleaving, 12-byte depth, path `i` memory `17*i`: p. 26,
//!   Fig. 3-9.
//! * §3.8 inner code, `G1 = 171oct`, `G2 = 133oct`, puncturing Table 3-8: p. 27.
//! * §3.9.3.1 DQPSK: p. 29 (Fig. 3-12, Table 3-10, Fig. 3-13).
//! * §3.9.3.2 QPSK: p. 30 (Fig. 3-14, Fig. 3-15).
//! * §3.9.3.3 16QAM: p. 31 (Fig. 3-16, Fig. 3-17).
//! * §3.9.3.4 64QAM: p. 32 (Fig. 3-18, Fig. 3-19).
//! * §3.9.4 modulation-level normalisation, Table 3-11: p. 33.

use std::collections::VecDeque;

use num_complex::Complex32;

use super::control::{data_carriers_per_segment, SYMBOLS_PER_FRAME};
use crate::dsp::Mode;
use crate::fec::{self, PuncturePattern, PunctureRate};

// ---------------------------------------------------------------------------
// Constants (all from ARIB STD-B31 v2.2-E1; no invented values)
// ---------------------------------------------------------------------------

/// Transport-stream packet length in bytes (ISO/IEC 13818-1 / ARIB §3.3).
pub const TS_PACKET_SIZE: usize = 188;

/// Transmission TSP length in bytes: 188 data + 16 Reed-Solomon parity.
pub const TSP_SIZE: usize = 204;

/// Number of branches of the convolutional byte interleaver (§3.7, Fig. 3-9).
pub const BYTE_INTERLEAVE_BRANCHES: usize = 12;

/// Per-branch memory unit of the byte interleaver: 17 bytes (§3.7, Fig. 3-9).
pub const BYTE_INTERLEAVE_STEP: usize = 17;

/// Startup latency of the complementary byte de-interleaver, in bytes.
///
/// A byte written to interleaver branch `i` is read back only on the next visit
/// of that branch, i.e. after `i * 17 * 12` bytes; the largest branch (`i = 11`)
/// therefore delays by `11 * 17 * 12 = 2244` bytes, matching the "11
/// transmission TSPs" (11 * 204) stated in §3.6, p. 25.
pub const BYTE_INTERLEAVE_LATENCY: usize =
    BYTE_INTERLEAVE_BRANCHES * (BYTE_INTERLEAVE_BRANCHES - 1) * BYTE_INTERLEAVE_STEP;

/// Maximum bit-interleaving delay, in bits, over all modulations (§3.9.3).
const MAX_TX_BIT_DELAY: usize = 120;

/// DQPSK / QPSK bit-interleaving delays, Fig. 3-12 / Fig. 3-14.
const DQPSK_QPSK_DELAYS: [usize; 2] = [0, MAX_TX_BIT_DELAY];
/// 16QAM bit-interleaving delays, Fig. 3-16.
const QAM16_DELAYS: [usize; 4] = [0, 40, 80, MAX_TX_BIT_DELAY];
/// 64QAM bit-interleaving delays, Fig. 3-18.
const QAM64_DELAYS: [usize; 6] = [0, 24, 48, 72, 96, MAX_TX_BIT_DELAY];

/// Maximum number of bits carried by one carrier symbol (64QAM).
const MAX_BITS_PER_SYMBOL: usize = 6;

/// Number of states of the K=7 convolutional code (`2^(K-1)`).
const K7_STATES: usize = 64;

/// Sliding-window traceback depth, in input bits.
///
/// `5*K = 35` is the classical rule of thumb; 64 leaves margin for the
/// punctured rates (which erase up to half the metric terms per period).
const K7_TRACEBACK: usize = 64;

/// First generator polynomial, octal 171 (§3.8, Fig. 3-10).
const K7_POLY_G1: u8 = 0o171;

/// Second generator polynomial, octal 133 (§3.8, Fig. 3-10).
const K7_POLY_G2: u8 = 0o133;

/// PRBS register initial value.
///
/// §3.5 (p. 24): the value is `100101010000000`, "arranged in ascending order of
/// bits, from left to right", with the stages labelled `D1..D15` left to right in
/// Fig. 3-8. Representing `D1` in bit 0 and `D15` in bit 14, the value is
/// `0x00A9`. The bit ordering is *not* an invented constant; it is the literal
/// register content of the figure.
const PRBS_INIT: u16 = 0x00A9;

/// TS synchronisation byte (ARIB §3.3, Fig. 3-6).
const TS_SYNC: u8 = 0x47;

/// Bitwise complement of [`TS_SYNC`]; produced by a 180-degree constellation
/// ambiguity. Derived, never hard-coded: `!0x47 == 0xB8`.
const TS_SYNC_INVERTED: u8 = !TS_SYNC;

/// Minimum buffered bytes before a validated sync search is attempted
/// (three consecutive transmission TSPs).
const SYNC_SEARCH_MIN: usize = TSP_SIZE * 3;

/// Number of consecutive 204-byte blocks that fail RS/TS validation while the
/// frame is locked before the byte/PRBS phase is declared lost.
///
/// A single failed block is not enough: a burst of channel errors can cost one
/// codeword, and the (204,188) code already rejects far more than it corrects.
/// Three consecutive rejects (one full acquisition span) cannot be a transient
/// and triggers re-acquisition from the retained raw bytes.
const MAX_BLOCK_FAILURES: usize = 3;

/// Raw (still energy-dispersed) bytes retained while locked so the sync and PRBS
/// phase can be re-acquired after a dropped or inserted byte without waiting for
/// a clean stream. Holds the acquisition search window plus one block, which
/// guarantees at least three post-loss sync bytes are available to [`FrameSync::acquire`].
const RAW_REACQUIRE_BYTES: usize = SYNC_SEARCH_MIN + TSP_SIZE;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by [`ChannelDecoder::new`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CodingError {
    /// The time-interleaving length is not one of the values of Table 3-12.
    #[error(
        "time-interleaving length {0} is invalid; ARIB STD-B31 Table 3-12 allows 0, 1, 2, 4, 8 or 16"
    )]
    InvalidInterleave(usize),
}

// ---------------------------------------------------------------------------
// Modulation
// ---------------------------------------------------------------------------

/// Carrier modulation schemes of ARIB STD-B31 §3.9.3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Modulation {
    /// `pi/4`-shift DQPSK (differential), Fig. 3-12 / Table 3-10.
    Dqpsk,
    /// Coherent QPSK, Fig. 3-14 / Fig. 3-15.
    Qpsk,
    /// Coherent 16QAM, Fig. 3-16 / Fig. 3-17.
    Qam16,
    /// Coherent 64QAM, Fig. 3-18 / Fig. 3-19.
    Qam64,
}

impl Modulation {
    /// Every modulation scheme, in the order of Tables 3-6 and 3-8.
    pub const ALL: [Modulation; 4] = [
        Modulation::Dqpsk,
        Modulation::Qpsk,
        Modulation::Qam16,
        Modulation::Qam64,
    ];

    /// Number of bits carried by one carrier symbol.
    ///
    /// DQPSK/QPSK: 2, 16QAM: 4, 64QAM: 6 (ARIB §3.9.3).
    pub const fn bits_per_symbol(self) -> usize {
        match self {
            Modulation::Dqpsk | Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        }
    }

    /// Number of bits carried per I (or Q) axis.
    pub const fn bits_per_axis(self) -> usize {
        self.bits_per_symbol() / 2
    }

    /// Transmitter bit-interleaving delay `D_i`, in bits, for serial bit `i`.
    ///
    /// Transcribed from the system diagrams: QPSK/DQPSK Fig. 3-12/3-14
    /// (`b0`: 0, `b1`: 120); 16QAM Fig. 3-16 (`0, 40, 80, 120`); 64QAM Fig. 3-18
    /// (`0, 24, 48, 72, 96, 120`). The receiver applies the complementary delay
    /// `120 - D_i`.
    pub const fn tx_bit_delays(self) -> &'static [usize] {
        match self {
            Modulation::Dqpsk | Modulation::Qpsk => &DQPSK_QPSK_DELAYS,
            Modulation::Qam16 => &QAM16_DELAYS,
            Modulation::Qam64 => &QAM64_DELAYS,
        }
    }

    /// Inverse of the Table 3-11 normalisation factor, used by [`demap`].
    ///
    /// Table 3-11 normalises the constellation by `Z/sqrt(2)` (DQPSK, QPSK),
    /// `Z/sqrt(10)` (16QAM) and `Z/sqrt(42)` (64QAM). Multiplying the received
    /// point by `sqrt(2)`, `sqrt(10)` or `sqrt(42)` recovers the integer PAM
    /// levels `{+-1}`, `{+-1,+-3}` or `{+-1,+-3,+-5,+-7}`.
    ///
    /// For DQPSK the caller supplies the *differential* symbol (see
    /// [`demap`]): the product of two average-power-1 symbols, each with the
    /// `Z/sqrt(2)` normalisation of Table 3-11, itself has magnitude `1`, so the
    /// same `sqrt(2)` scale as QPSK recovers the integer levels `{+-1}`.
    pub fn demap_scale(self) -> f32 {
        match self {
            Modulation::Dqpsk => std::f32::consts::SQRT_2,
            Modulation::Qpsk => std::f32::consts::SQRT_2,
            Modulation::Qam16 => 10.0f32.sqrt(),
            Modulation::Qam64 => 42.0f32.sqrt(),
        }
    }
}

// ---------------------------------------------------------------------------
// Soft demapper
// ---------------------------------------------------------------------------

/// Gray-coded PAM bit value for level index `k`.
///
/// The constellations of Figs. 3-15/3-17/3-19 are Gray-coded: reading a level
/// index `k` from most negative to most positive, the bits are the binary-reflected
/// Gray code of `k` with the most significant bit inverted (so `0` selects the
/// positive half). `bit` is numbered from the axis MSB (`0`) downward.
fn pam_bit(bits: usize, k: u32, bit: usize) -> u8 {
    let gray = k ^ (k >> 1);
    let shift = bits - 1 - bit;
    let value = ((gray >> shift) & 1) as u8;
    if bit == 0 {
        value ^ 1
    } else {
        value
    }
}

/// Max-log soft values for one PAM axis carrying `bits` Gray-coded bits.
///
/// Positive means "bit is 0", negative means "bit is 1", matching
/// [`crate::fec::decode_k7_soft`]. `x` is the axis projection already scaled to
/// the integer level set `{-(2^bits - 1), ..., -1, 1, ..., 2^bits - 1}`.
fn pam_axis_soft(x: f32, bits: usize) -> Vec<f32> {
    let levels = 1u32 << bits;
    let span = levels as i32 - 1;
    let mut out = vec![0.0f32; bits];
    for (bit, slot) in out.iter_mut().enumerate() {
        let mut best0 = f32::INFINITY;
        let mut best1 = f32::INFINITY;
        for k in 0..levels {
            let level = (2 * k as i32 - span) as f32;
            let distance = (x - level) * (x - level);
            if pam_bit(bits, k, bit) == 0 {
                if distance < best0 {
                    best0 = distance;
                }
            } else if distance < best1 {
                best1 = distance;
            }
        }
        *slot = best1 - best0;
    }
    out
}

/// Soft-demap one carrier symbol into `bits_per_symbol` soft values.
///
/// The returned order is the transmitter's serial bit order `b0, b1, ...`
/// (ARIB STD-B31 §3.9.3). A positive value favours bit `0`, a negative value
/// favours bit `1`, exactly as required by [`crate::fec::decode_k7_soft`].
///
/// # DQPSK
///
/// `pi/4`-shift DQPSK is differential; the bits describe the *phase change*
/// relative to the previous symbol (Table 3-10). This function is stateless, so
/// for [`Modulation::Dqpsk`] the caller must pass the normalised differential
/// product of the current and previous symbols, e.g.
/// `demap(dqpsk_differential(current, previous), Modulation::Dqpsk)`.
/// The four differential phases map onto the QPSK constellation
/// (Table 3-10: `00 -> +pi/4`, `01 -> -pi/4`, `10 -> +3pi/4`, `11 -> -3pi/4`),
/// so `b0` is the sign of the in-phase component and `b1` the sign of the
/// quadrature component.
///
/// # Panics
///
/// This function does not panic. Non-finite components are treated as `0.0`
/// and yield finite soft values.
pub fn demap(symbol: Complex32, modulation: Modulation) -> Vec<f32> {
    let bits = modulation.bits_per_axis();
    let scale = modulation.demap_scale();
    let i = sanitise(symbol.re) * scale;
    let q = sanitise(symbol.im) * scale;
    let in_phase = pam_axis_soft(i, bits);
    let quadrature = pam_axis_soft(q, bits);
    let mut out = Vec::with_capacity(bits * 2);
    for bit in 0..bits {
        out.push(in_phase[bit]);
        out.push(quadrature[bit]);
    }
    out
}

/// Normalised differential product `current * conj(previous)` for
/// [`Modulation::Dqpsk`].
///
/// This is the quantity [`demap`] expects for DQPSK. Both inputs are the
/// normalised (average-power-1) received symbols; the caller keeps the previous
/// symbol in its differential-detection stage.
pub fn dqpsk_differential(current: Complex32, previous: Complex32) -> Complex32 {
    current * previous.conj()
}

fn sanitise(value: f32) -> f32 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// Layer configuration
// ---------------------------------------------------------------------------

/// Channel-decoding configuration of one hierarchical layer.
///
/// Mirrors the current layer-A transmission parameters decoded from TMCC by
/// [`crate::demod::control`] (`TmccLayer::interleave_value`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LayerConfig {
    /// Carrier modulation.
    pub modulation: Modulation,
    /// Convolutional coding rate / puncturing pattern (Table 3-8).
    pub rate: PunctureRate,
    /// Time-interleaving length `I` (Table 3-12): one of `0, 1, 2, 4, 8, 16`.
    ///
    /// Time de-interleaving is performed by [`crate::demod::control`]; the FEC
    /// core validates the value but does not otherwise use it.
    pub interleave: usize,
}

/// Number of transmission TSPs carried by one OFDM frame (ARIB Table 3-3).
///
/// A frame carries `data_carriers * bits_per_symbol * SYMBOLS_PER_FRAME` coded
/// bits; at convolutional rate `num/den` that is
/// `data_carriers * bits_per_symbol * num * SYMBOLS_PER_FRAME / (den * 8)`
/// information bytes, i.e. that many 204-byte transmission TSPs. The two
/// [`SYMBOLS_PER_FRAME`] factors are kept in the derivation so that the formula
/// can be read directly against Table 3-3; the result is exact for every
/// standard Table 3-3 combination.
///
/// For example, QPSK (`2` bits) at rate `1/2` in Mode 3 (`384` data carriers)
/// gives `384 * 2 * 204 / (2 * 8 * 204) = 48` TSPs, matching Table 3-3.
pub fn packets_per_frame(mode: Mode, modulation: Modulation, rate: PunctureRate) -> usize {
    let carriers = data_carriers_per_segment(mode) as u64;
    let bits = modulation.bits_per_symbol() as u64;
    let (num, den) = rate.ratio();
    let numerator = carriers * bits * num as u64 * SYMBOLS_PER_FRAME as u64;
    let denominator = den as u64 * SYMBOLS_PER_FRAME as u64 * 8;
    debug_assert_eq!(
        numerator % denominator,
        0,
        "Table 3-3 TSP count must be integral for {mode:?}/{modulation:?}/{rate:?}"
    );
    (numerator / denominator) as usize
}

// ---------------------------------------------------------------------------
// Streaming K=7 soft Viterbi
// ---------------------------------------------------------------------------

/// One convolutional-code transition.
///
/// Identical convention to [`crate::fec::K7Encoder`]: the six memory bits are
/// held with the most recent in the most-significant position; the current input
/// bit enters as bit 6 of a 7-bit register.
fn k7_transition(state: u8, input: u8) -> (u8, u8, u8) {
    let register = (state & 0x3F) | ((input & 1) << 6);
    let next = (register >> 1) & 0x3F;
    let out1 = (register & K7_POLY_G1).count_ones() & 1;
    let out2 = (register & K7_POLY_G2).count_ones() & 1;
    (next, out1 as u8, out2 as u8)
}

/// Continuous soft-decision Viterbi decoder for the K=7 rate-1/2 code.
///
/// Unlike [`crate::fec::decode_k7_soft`] this decoder never re-initialises the
/// path metrics and keeps a sliding traceback window, so it can decode an
/// arbitrarily long, unterminated stream in constant memory. Bits at the very
/// start of a stream are decided with the metrics seeded to state 0
/// (correct when the stream starts where the encoder started, and in any case
/// self-correcting within a few constraint lengths for a noiseless input).
#[derive(Debug, Clone)]
struct StreamingViterbi {
    metrics: [f32; K7_STATES],
    /// Predecessor state per destination state, one slot per decoded step.
    trace: Vec<[u8; K7_STATES]>,
    steps: usize,
    pending: VecDeque<u8>,
}

impl StreamingViterbi {
    fn new() -> Self {
        let mut metrics = [f32::NEG_INFINITY; K7_STATES];
        metrics[0] = 0.0;
        Self {
            metrics,
            trace: vec![[0u8; K7_STATES]; K7_TRACEBACK],
            steps: 0,
            pending: VecDeque::new(),
        }
    }

    fn best_state(&self) -> u8 {
        let mut best = 0usize;
        let mut best_metric = f32::NEG_INFINITY;
        for (index, &metric) in self.metrics.iter().enumerate() {
            if metric > best_metric {
                best_metric = metric;
                best = index;
            }
        }
        best as u8
    }

    /// Feed one soft pair `(X_t, Y_t)`.
    fn push(&mut self, soft0: f32, soft1: f32) {
        let mut next = [f32::NEG_INFINITY; K7_STATES];
        let mut predecessor = [0u8; K7_STATES];

        for (state, &base) in self.metrics.iter().enumerate() {
            if !base.is_finite() {
                continue;
            }
            for input in 0..2u8 {
                let (next_state, out1, out2) = k7_transition(state as u8, input);
                let term0 = if out1 == 0 { soft0 } else { -soft0 };
                let term1 = if out2 == 0 { soft1 } else { -soft1 };
                let candidate = base + term0 + term1;
                let slot = next_state as usize;
                if candidate > next[slot] {
                    next[slot] = candidate;
                    predecessor[slot] = state as u8;
                }
            }
        }

        self.metrics = next;
        self.trace[self.steps % K7_TRACEBACK] = predecessor;
        self.steps += 1;

        if self.steps >= K7_TRACEBACK {
            self.emit_oldest();
        }
    }

    /// Decode the bit `K7_TRACEBACK` positions behind the current step by
    /// tracing back from the best current state.
    fn emit_oldest(&mut self) {
        let mut state = self.best_state();
        for j in 1..K7_TRACEBACK {
            let transition = self.steps - j;
            state = self.trace[transition % K7_TRACEBACK][state as usize];
        }
        // `state` is now the state at time `steps - K7_TRACEBACK + 1`; the input
        // bit that produced it is the state MSB (encoder convention).
        self.pending.push_back((state >> 5) & 1);
    }

    /// Drain the remaining traceback pipeline (terminal; see module docs).
    fn flush(&mut self) {
        let steps = self.steps;
        if steps == 0 {
            return;
        }
        let already = steps.saturating_sub(K7_TRACEBACK - 1);
        let mut state = self.best_state();
        let mut reversed = Vec::with_capacity(K7_TRACEBACK);
        let mut time = steps;
        while time > already {
            reversed.push((state >> 5) & 1);
            let transition = time - 1;
            state = self.trace[transition % K7_TRACEBACK][state as usize];
            time -= 1;
        }
        reversed.reverse();
        self.pending.extend(reversed);
    }
}

// ---------------------------------------------------------------------------
// PRBS (energy dispersal)
// ---------------------------------------------------------------------------

/// Advance the `X^15 + X^14 + 1` PRBS by one bit and return the output bit.
///
/// Fig. 3-8 (p. 24): stages `D1..D15` left to right, `D1` at bit 0 of the state
/// and `D15` at bit 14, XOR taps `D14 ^ D15`, feedback shifted into `D1`.
fn prbs_step(state: &mut u16) -> u8 {
    let out = ((*state >> 13) ^ (*state >> 14)) & 1;
    *state = ((*state << 1) | out) & 0x7FFF;
    out as u8
}

/// 8 PRBS bits packed most-significant-first, advancing `state` by 8.
fn prbs_byte(state: &mut u16) -> u8 {
    let mut byte = 0u8;
    for _ in 0..8 {
        byte = (byte << 1) | prbs_step(state);
    }
    byte
}

// ---------------------------------------------------------------------------
// Frame synchroniser / derandomiser
// ---------------------------------------------------------------------------

/// Byte-phase, frame-phase and polarity acquisition for the transmission TSPs.
///
/// The de-interleaved byte stream is XORed with a PRBS that is re-initialised
/// every `frame_bytes` bytes ([`packets_per_frame`] transmission TSPs, §3.5).
/// Because the sync byte of each transmission TSP is not dispersed, sync
/// detection itself is polarity/PRBS independent: the grid of sync bytes is
/// found as `0x47` (or `0xB8` under a 180-degree ambiguity) every 204 bytes.
///
/// Once the packet grid is known, the receiver still has to discover *where in
/// the frame* it started: the PRBS phase at a given sync depends on how many
/// TSPs into the frame that sync lies. All `packets_per_frame` plausible packet
/// positions are therefore tried and accepted only when a derandomised packet
/// both passes RS(204,188) and validates as a TS packet. This bounded search is
/// what protects the lock against false sync patterns. After locking, only the
/// PRBS phase is tracked; the bit/byte de-interleavers and Viterbi state are
/// never reset at a frame boundary, so no packets are lost at frame edges.
///
/// # Re-acquisition after a byte-phase loss
///
/// A dropped or inserted byte shifts the whole sync grid by one, after which
/// every block on the old grid fails RS/TS validation. [`FrameSync::note_block`]
/// counts consecutive failures and, after [`MAX_BLOCK_FAILURES`], releases the
/// lock and replays the retained raw dispersed bytes through the acquisition
/// search, so the byte/PRBS phase is recovered without a caller reset. The raw
/// retention is bounded by [`RAW_REACQUIRE_BYTES`].
#[derive(Debug, Clone)]
struct FrameSync {
    /// Bytes in one OFDM frame (`packets_per_frame * TSP_SIZE`).
    frame_bytes: usize,
    /// Transmission TSPs per frame (Table 3-3).
    packets_per_frame: usize,
    /// PRBS mask byte for every frame offset `0..frame_bytes`.
    masks: Vec<u8>,
    locked: bool,
    inverted: bool,
    /// Frame offset of the next byte consumed once locked.
    frame_pos: usize,
    /// Raw de-interleaved bytes buffered while searching for a lock.
    search: VecDeque<u8>,
    /// Derandomised bytes awaiting grouping into 204-byte blocks.
    post: VecDeque<u8>,
    /// Raw (still dispersed) bytes of the most recent locked blocks, retained so
    /// a byte-phase loss can be recovered by rescanning them.
    raw: VecDeque<u8>,
    /// Consecutive failed 204-byte blocks since the last validated packet.
    failures: usize,
}

impl FrameSync {
    fn new(frame_bytes: usize, packets_per_frame: usize) -> Self {
        let mut prbs = PRBS_INIT;
        let masks: Vec<u8> = (0..frame_bytes).map(|_| prbs_byte(&mut prbs)).collect();
        Self {
            frame_bytes,
            packets_per_frame,
            masks,
            locked: false,
            inverted: false,
            frame_pos: 0,
            search: VecDeque::new(),
            post: VecDeque::new(),
            raw: VecDeque::new(),
            failures: 0,
        }
    }

    /// Add one de-interleaved byte and return any completed 204-byte blocks.
    ///
    /// While unlocked, bytes are buffered; once a lock is acquired the buffer
    /// is drained so that no packet is lost to the acquisition search.
    fn push(&mut self, byte: u8) -> Vec<[u8; TSP_SIZE]> {
        let mut out = Vec::new();

        if !self.locked {
            self.search.push_back(byte);
            if self.search.len() >= SYNC_SEARCH_MIN {
                if let Some((offset, frame_pos, inverted)) = self.acquire() {
                    self.locked = true;
                    self.inverted = inverted;
                    self.frame_pos = frame_pos;
                    self.failures = 0;
                    // The retained raw window belongs to the previous phase.
                    self.raw.clear();
                    for _ in 0..offset {
                        self.search.pop_front();
                    }
                    while let Some(buffered) = self.search.pop_front() {
                        if let Some(block) = self.push_locked(buffered) {
                            out.push(block);
                        }
                    }
                    return out;
                }
            }
            // Keep the search bounded when there is no valid frame.
            while self.search.len() > SYNC_SEARCH_MIN + TSP_SIZE {
                self.search.pop_front();
            }
            return out;
        }

        if let Some(block) = self.push_locked(byte) {
            out.push(block);
        }
        out
    }

    /// XOR one byte with the PRBS byte for the current frame offset, advancing
    /// the offset and wrapping (re-initialising) at the frame boundary.
    fn next_mask(&mut self) -> u8 {
        let mask = self.masks[self.frame_pos];
        self.frame_pos += 1;
        if self.frame_pos == self.frame_bytes {
            self.frame_pos = 0;
        }
        mask
    }

    fn push_locked(&mut self, byte: u8) -> Option<[u8; TSP_SIZE]> {
        // Retain the raw dispersed byte so a byte-phase loss can be recovered
        // by re-running the acquisition search over the retained window.
        self.raw.push_back(byte);
        while self.raw.len() > RAW_REACQUIRE_BYTES {
            self.raw.pop_front();
        }

        let mask = self.next_mask();
        self.post.push_back(byte ^ mask);
        if self.post.len() < TSP_SIZE {
            return None;
        }
        let mut block = [0u8; TSP_SIZE];
        for slot in block.iter_mut() {
            *slot = self.post.pop_front().expect("length checked");
        }
        Some(block)
    }

    /// Record whether the most recently returned block decoded to a validated TS
    /// packet, and release the lock after too many consecutive failures.
    ///
    /// The caller ([`ChannelDecoder::finish_block`]) owns RS(204,188) decoding
    /// and TS validation, so it feeds the result back here. A byte-phase loss
    /// makes every block on the stale grid fail; after [`MAX_BLOCK_FAILURES`] the
    /// retained raw bytes are handed back to the acquisition search so the next
    /// [`push`](Self::push) re-acquires the correct phase.
    fn note_block(&mut self, ok: bool) {
        if !self.locked {
            return;
        }
        if ok {
            self.failures = 0;
            return;
        }
        self.failures += 1;
        if self.failures >= MAX_BLOCK_FAILURES {
            self.unlock();
        }
    }

    /// Release the byte/PRBS lock and re-queue the retained raw bytes for
    /// re-acquisition.
    fn unlock(&mut self) {
        self.locked = false;
        self.inverted = false;
        self.frame_pos = 0;
        self.failures = 0;
        self.post.clear();
        self.search.clear();
        self.search.extend(self.raw.drain(..));
    }

    /// Try to acquire the byte, frame and polarity phase from the buffered
    /// bytes. Returns `(sync_offset, frame_pos, inverted)` on success.
    fn acquire(&self) -> Option<(usize, usize, bool)> {
        if self.search.len() < SYNC_SEARCH_MIN {
            return None;
        }
        for offset in 0..TSP_SIZE {
            if offset + 2 * TSP_SIZE >= self.search.len() {
                break;
            }
            let a = self.search[offset];
            let b = self.search[offset + TSP_SIZE];
            let c = self.search[offset + 2 * TSP_SIZE];
            let normal = a == TS_SYNC && b == TS_SYNC && c == TS_SYNC;
            let inverted = a == TS_SYNC_INVERTED && b == TS_SYNC_INVERTED && c == TS_SYNC_INVERTED;
            if !normal && !inverted {
                continue;
            }
            // The packet grid is fixed; only the position inside the frame is
            // unknown, so test every plausible packet position (bounded).
            for j in 0..self.packets_per_frame {
                let frame_pos = (TSP_SIZE - 1) + TSP_SIZE * j;
                if self.candidate_decodes(offset, frame_pos, inverted)
                    && self.candidate_decodes(offset + TSP_SIZE, frame_pos + TSP_SIZE, inverted)
                {
                    return Some((offset, frame_pos, inverted));
                }
            }
        }
        None
    }

    /// Derandomise the 204-byte block starting at `search[start]` assuming it is
    /// a sync at frame offset `frame_pos`, then RS-decode and validate it.
    fn candidate_decodes(&self, start: usize, frame_pos: usize, inverted: bool) -> bool {
        if start + TSP_SIZE > self.search.len() {
            return false;
        }
        let mut codeword = [0u8; TSP_SIZE];
        let mut pos = frame_pos % self.frame_bytes;
        for (t, slot) in codeword.iter_mut().enumerate() {
            *slot = self.search[start + t] ^ self.masks[pos];
            pos += 1;
            if pos == self.frame_bytes {
                pos = 0;
            }
        }
        if inverted {
            for byte in codeword.iter_mut() {
                *byte ^= 0xFF;
            }
        }
        codeword[0] = TS_SYNC;
        let data = match fec::rs204_188_decode(&codeword) {
            Ok(data) => data,
            Err(_) => return false,
        };
        let mut packet = [0u8; TS_PACKET_SIZE];
        packet.copy_from_slice(&data);
        fec::validate_ts_packet(&packet).is_ok()
    }
}

// ---------------------------------------------------------------------------
// Channel decoder
// ---------------------------------------------------------------------------

/// Streaming ISDB-T channel decoder for one hierarchical layer.
///
/// Feed the output of [`demap`] (or any equivalent soft-bit source whose sign
/// convention is "positive favours 0") to [`ChannelDecoder::push_soft`]; complete,
/// validated 188-byte TS packets are returned as they become available.
///
/// Prefer [`ChannelDecoder::with_mode`], which knows the OFDM frame length and
/// therefore re-initialises only the energy-dispersal PRBS at each frame
/// boundary while keeping the streaming bit/byte de-interleaver and Viterbi
/// state alive. [`ChannelDecoder::new`] is a convenience for the Japanese
/// one-seg default (Mode 3).
///
/// # Example
///
/// ```no_run
/// use ts_stream::demod::coding::{ChannelDecoder, LayerConfig, Modulation};
/// use ts_stream::dsp::Mode;
/// use ts_stream::fec::PunctureRate;
///
/// let config = LayerConfig {
///     modulation: Modulation::Qpsk,
///     rate: PunctureRate::Rate1_2,
///     interleave: 0,
/// };
/// let mut decoder = ChannelDecoder::with_mode(Mode::Mode3, config).unwrap();
/// // Push soft bits continuously: the PRBS frame boundary is tracked internally.
/// let soft = vec![1.0f32; 2];
/// let _packets = decoder.push_soft(&soft);
/// ```
#[derive(Debug, Clone)]
pub struct ChannelDecoder {
    mode: Mode,
    config: LayerConfig,
    bits_per_symbol: usize,
    /// Buffered soft bits not yet forming a whole carrier symbol.
    symbol_input: VecDeque<f32>,
    /// Receiver-side bit delays `120 - D_i` (§3.9.3).
    rx_delays: Vec<usize>,
    lanes: Vec<VecDeque<f32>>,
    symbol_count: usize,
    pattern: PuncturePattern,
    kept_per_period: usize,
    punctured: Vec<f32>,
    viterbi: StreamingViterbi,
    bit_buffer: VecDeque<u8>,
    byte_branches: Vec<VecDeque<u8>>,
    byte_in_index: usize,
    deinterleaved_count: usize,
    frame: FrameSync,
}

impl ChannelDecoder {
    /// Build a decoder for `config`, assuming the Japanese one-seg default
    /// transmission mode (Mode 3).
    ///
    /// Equivalent to [`ChannelDecoder::with_mode(Mode::Mode3, config)`]. Use
    /// [`ChannelDecoder::with_mode`] when the mode is known (e.g. from the
    /// synchroniser) so that the PRBS is re-initialised at the correct frame
    /// boundary.
    ///
    /// # Errors
    ///
    /// [`CodingError::InvalidInterleave`] when `config.interleave` is not one of
    /// the time-interleaving lengths of Table 3-12.
    pub fn new(config: LayerConfig) -> Result<Self, CodingError> {
        Self::with_mode(Mode::Mode3, config)
    }

    /// Build a decoder for `config` in OFDM `mode`.
    ///
    /// The mode and the layer parameters together fix the number of
    /// transmission TSPs per OFDM frame ([`packets_per_frame`], Table 3-3). The
    /// decoder uses that length to reset the energy-dispersal PRBS at every
    /// frame boundary without disturbing the streaming de-interleavers or the
    /// Viterbi decoder, and to acquire the frame phase from an arbitrary byte
    /// start.
    ///
    /// # Errors
    ///
    /// [`CodingError::InvalidInterleave`] when `config.interleave` is not one of
    /// the time-interleaving lengths of Table 3-12.
    pub fn with_mode(mode: Mode, config: LayerConfig) -> Result<Self, CodingError> {
        if !matches!(config.interleave, 0 | 1 | 2 | 4 | 8 | 16) {
            return Err(CodingError::InvalidInterleave(config.interleave));
        }

        let bits_per_symbol = config.modulation.bits_per_symbol();
        let rx_delays: Vec<usize> = config
            .modulation
            .tx_bit_delays()
            .iter()
            .map(|&delay| MAX_TX_BIT_DELAY - delay)
            .collect();
        let pattern = config.rate.pattern();
        let kept_per_period = pattern.kept_per_period();
        let byte_branches: Vec<VecDeque<u8>> = (0..BYTE_INTERLEAVE_BRANCHES)
            .map(|branch| {
                let delay = (BYTE_INTERLEAVE_BRANCHES - 1 - branch) * BYTE_INTERLEAVE_STEP;
                VecDeque::from(vec![0u8; delay])
            })
            .collect();
        let packets_per_frame = packets_per_frame(mode, config.modulation, config.rate);
        let frame = FrameSync::new(packets_per_frame * TSP_SIZE, packets_per_frame);

        Ok(Self {
            mode,
            config,
            bits_per_symbol,
            symbol_input: VecDeque::new(),
            rx_delays,
            lanes: (0..bits_per_symbol).map(|_| VecDeque::new()).collect(),
            symbol_count: 0,
            pattern,
            kept_per_period,
            punctured: Vec::new(),
            viterbi: StreamingViterbi::new(),
            bit_buffer: VecDeque::new(),
            byte_branches,
            byte_in_index: 0,
            deinterleaved_count: 0,
            frame,
        })
    }

    /// The configuration this decoder was built with.
    pub fn config(&self) -> LayerConfig {
        self.config
    }

    /// The OFDM mode this decoder was built with.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Reset every internal state, including the PRBS.
    ///
    /// Intended to be called at an OFDM frame boundary (§3.5, p. 24): the next
    /// soft bit pushed is the first bit of the frame. The K=7 encoder state is
    /// re-assumed to be zero; the first few decoded bits are therefore unreliable
    /// and the first ~11 transmission TSPs are swallowed by the byte
    /// de-interleaver transient.
    pub fn reset(&mut self) {
        let mode = self.mode;
        let config = self.config;
        *self = Self::with_mode(mode, config)
            .expect("configuration accepted at construction stays valid");
    }

    /// Push soft bits and return any complete TS packets.
    ///
    /// `bits` may be split arbitrarily across calls; the decoder buffers partial
    /// carrier symbols. Soft values must use the [`demap`] sign convention
    /// (positive favours bit `0`).
    pub fn push_soft(&mut self, bits: &[f32]) -> Vec<[u8; TS_PACKET_SIZE]> {
        let mut out = Vec::new();
        self.symbol_input.extend(bits.iter().copied());

        while self.symbol_input.len() >= self.bits_per_symbol {
            let mut symbol = [0.0f32; MAX_BITS_PER_SYMBOL];
            for slot in symbol.iter_mut().take(self.bits_per_symbol) {
                *slot = self.symbol_input.pop_front().expect("length checked");
            }
            self.push_symbol(&symbol[..self.bits_per_symbol], &mut out);
        }
        out
    }

    /// Drain the Viterbi pipeline at the end of a stream (terminal; see module
    /// docs).
    pub fn flush(&mut self) -> Vec<[u8; TS_PACKET_SIZE]> {
        let mut out = Vec::new();
        self.viterbi.flush();
        while let Some(bit) = self.viterbi.pending.pop_front() {
            self.bit_buffer.push_back(bit);
        }
        self.pack_bytes(&mut out);
        out
    }

    // -- internal pipeline --------------------------------------------------

    fn push_symbol(&mut self, symbol: &[f32], out: &mut Vec<[u8; TS_PACKET_SIZE]>) {
        let mut complete = self.symbol_count >= MAX_TX_BIT_DELAY;
        let mut group = [0.0f32; MAX_BITS_PER_SYMBOL];

        for (lane, &soft) in symbol.iter().enumerate() {
            self.lanes[lane].push_back(soft);
            if self.lanes[lane].len() > self.rx_delays[lane] {
                group[lane] = self.lanes[lane].pop_front().expect("length checked");
            } else {
                complete = false;
            }
        }
        self.symbol_count += 1;

        if complete {
            self.punctured.extend_from_slice(&group[..symbol.len()]);
            self.run_depuncture(out);
        }
    }

    fn run_depuncture(&mut self, out: &mut Vec<[u8; TS_PACKET_SIZE]>) {
        while self.punctured.len() >= self.kept_per_period {
            let chunk: Vec<f32> = self.punctured.drain(..self.kept_per_period).collect();
            let mother = self
                .pattern
                .depuncture(&chunk)
                .expect("one pattern period always depunctures");
            for pair in mother.chunks(2) {
                self.viterbi.push(pair[0], pair[1]);
            }
        }
        while let Some(bit) = self.viterbi.pending.pop_front() {
            self.bit_buffer.push_back(bit);
        }
        self.pack_bytes(out);
    }

    fn pack_bytes(&mut self, out: &mut Vec<[u8; TS_PACKET_SIZE]>) {
        while self.bit_buffer.len() >= 8 {
            let mut byte = 0u8;
            for _ in 0..8 {
                byte = (byte << 1) | (self.bit_buffer.pop_front().expect("length checked") & 1);
            }
            self.feed_byte(byte, out);
        }
    }

    fn feed_byte(&mut self, byte: u8, out: &mut Vec<[u8; TS_PACKET_SIZE]>) {
        let branch = self.byte_in_index % BYTE_INTERLEAVE_BRANCHES;
        self.byte_in_index += 1;
        let queue = &mut self.byte_branches[branch];
        queue.push_back(byte);
        let delayed = queue
            .pop_front()
            .expect("branch queue is never empty after push");

        if self.deinterleaved_count >= BYTE_INTERLEAVE_LATENCY {
            for block in self.frame.push(delayed) {
                if let Some(packet) = self.finish_block(&block) {
                    out.push(packet);
                }
            }
        }
        self.deinterleaved_count += 1;
    }

    fn finish_block(&mut self, block: &[u8; TSP_SIZE]) -> Option<[u8; TS_PACKET_SIZE]> {
        let mut codeword = *block;
        if self.frame.inverted {
            for byte in codeword.iter_mut() {
                *byte ^= 0xFF;
            }
        }
        // The transmitter skipped the sync byte during energy dispersal
        // (§3.5, p. 24); restore it before RS decoding.
        codeword[0] = TS_SYNC;

        let packet = fec::rs204_188_decode(&codeword).ok().and_then(|data| {
            let mut packet = [0u8; TS_PACKET_SIZE];
            packet.copy_from_slice(&data);
            fec::validate_ts_packet(&packet).is_ok().then_some(packet)
        });
        // Feed the RS/TS result back so a stale byte phase is released after
        // enough consecutive failures (dropped/inserted byte recovery).
        self.frame.note_block(packet.is_some());
        packet
    }
}

// ---------------------------------------------------------------------------
// Test-only transmitter helpers
// ---------------------------------------------------------------------------

/// Transmitter-side inverse of the receiver chain, exposed **only under
/// `cfg(test)`** so that the IQ replay tests can build an independent
/// transmission fixture without duplicating the coding helpers.
///
/// Nothing here is compiled into the production library. It exercises the same
/// private constants ([`PRBS_INIT`], the byte-interleaver geometry, the K=7
/// encoder) as the receiver, so a replay failure points at the receiver rather
/// than at a divergent copy of the transmitter.
#[cfg(test)]
pub(crate) mod test_support {
    use std::collections::VecDeque;

    use super::*;
    use crate::fec::K7Encoder;

    /// Concatenate RS(204,188) codewords and rotate left by one byte so the
    /// stream starts at the information byte immediately after a sync
    /// (§3.4/§3.5, pp. 23-24).
    pub(crate) fn build_frame(packets: &[[u8; TS_PACKET_SIZE]]) -> Vec<u8> {
        let mut stream = Vec::with_capacity(packets.len() * TSP_SIZE);
        for packet in packets {
            stream.extend_from_slice(&fec::rs204_188_encode(packet).expect("188 bytes"));
        }
        let first = stream.remove(0);
        stream.push(first);
        stream
    }

    /// Energy-disperse a complete transmission stream, resetting the PRBS at
    /// every OFDM frame boundary (§3.5, p. 24).
    ///
    /// `frame_bytes` is `packets_per_frame * TSP_SIZE`. Resetting here (rather
    /// than never) is what exercises more than one frame; a continuous PRBS
    /// would silently limit the decoder to the first frame.
    pub(crate) fn energy_disperse(stream: &[u8], frame_bytes: usize) -> Vec<u8> {
        assert!(frame_bytes > 0, "frame must contain at least one byte");
        let mut prbs = PRBS_INIT;
        let mut out = Vec::with_capacity(stream.len());
        for (index, &byte) in stream.iter().enumerate() {
            if index % frame_bytes == 0 {
                prbs = PRBS_INIT;
            }
            let frame_index = index % frame_bytes;
            let is_sync =
                frame_index >= TSP_SIZE - 1 && (frame_index - (TSP_SIZE - 1)) % TSP_SIZE == 0;
            if is_sync {
                // The sync byte is not dispersed but still advances the PRBS
                // by one byte, matching the receiver's frame-offset counter.
                for _ in 0..8 {
                    prbs_step(&mut prbs);
                }
                out.push(byte);
            } else {
                let mut scrambled = byte;
                for bit in 0..8 {
                    let p = prbs_step(&mut prbs);
                    scrambled ^= p << (7 - bit);
                }
                out.push(scrambled);
            }
        }
        out
    }

    /// Transmitter byte interleaver (§3.7, Fig. 3-9).
    pub(crate) fn byte_interleave(input: &[u8]) -> Vec<u8> {
        let mut branches: Vec<VecDeque<u8>> = (0..BYTE_INTERLEAVE_BRANCHES)
            .map(|branch| VecDeque::from(vec![0u8; branch * BYTE_INTERLEAVE_STEP]))
            .collect();
        let mut out = Vec::with_capacity(input.len());
        for (index, &byte) in input.iter().enumerate() {
            let branch = index % BYTE_INTERLEAVE_BRANCHES;
            branches[branch].push_back(byte);
            out.push(branches[branch].pop_front().expect("branch non-empty"));
        }
        out
    }

    /// Convolutional encode + puncture the interleaved byte stream.
    pub(crate) fn punctured_bits(
        interleaved: &[u8],
        modulation: Modulation,
        rate: PunctureRate,
    ) -> Vec<u8> {
        let mut bits = Vec::with_capacity(interleaved.len() * 8);
        for &byte in interleaved {
            for shift in (0..8).rev() {
                bits.push((byte >> shift) & 1);
            }
        }
        let mut encoder = K7Encoder::new();
        let mother = encoder.encode(&bits);
        let mut punctured = rate
            .pattern()
            .puncture(&mother)
            .expect("frame length divisible by pattern period");
        let bits_per_symbol = modulation.bits_per_symbol();
        let remainder = punctured.len() % bits_per_symbol;
        if remainder != 0 {
            punctured.resize(punctured.len() + (bits_per_symbol - remainder), 0);
        }
        punctured
    }

    /// Apply the transmitter bit interleaver to the punctured bits.
    pub(crate) fn bit_interleave_bits(punctured: &[u8], modulation: Modulation) -> Vec<u8> {
        let bits_per_symbol = modulation.bits_per_symbol();
        let mut lanes: Vec<VecDeque<u8>> = modulation
            .tx_bit_delays()
            .iter()
            .map(|&delay| VecDeque::from(vec![0u8; delay]))
            .collect();
        let mut out = Vec::with_capacity(punctured.len());
        for symbol in punctured.chunks(bits_per_symbol) {
            for (lane, &bit) in symbol.iter().enumerate() {
                lanes[lane].push_back(bit);
                out.push(lanes[lane].pop_front().expect("lane non-empty"));
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::test_support::{
        bit_interleave_bits, build_frame, byte_interleave, energy_disperse, punctured_bits,
    };
    use super::*;
    use crate::fec::K7Encoder;

    fn xorshift(state: &mut u32) -> u32 {
        let mut x = *state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *state = x;
        x
    }

    // ---- demapper known vectors ------------------------------------------

    fn level2(bit: u8) -> f32 {
        if bit == 0 {
            1.0
        } else {
            -1.0
        }
    }

    /// I level of 16QAM from the `(b0, b2)` pair, read off Fig. 3-17.
    fn level16(b0: u8, b2: u8) -> f32 {
        match (b0, b2) {
            (0, 0) => 3.0,
            (0, 1) => 1.0,
            (1, 1) => -1.0,
            (1, 0) => -3.0,
            _ => unreachable!(),
        }
    }

    /// Axis level of 64QAM from `(b0, b2, b4)`, read off Fig. 3-19.
    fn level64(b0: u8, b2: u8, b4: u8) -> f32 {
        match (b0, b2, b4) {
            (0, 0, 0) => 7.0,
            (0, 0, 1) => 5.0,
            (0, 1, 1) => 3.0,
            (0, 1, 0) => 1.0,
            (1, 1, 0) => -1.0,
            (1, 1, 1) => -3.0,
            (1, 0, 1) => -5.0,
            (1, 0, 0) => -7.0,
            _ => unreachable!(),
        }
    }

    fn assert_bit_signs(soft: &[f32], bits: &[u8]) {
        assert_eq!(soft.len(), bits.len());
        for (index, (&value, &bit)) in soft.iter().zip(bits.iter()).enumerate() {
            if bit == 0 {
                assert!(value > 0.0, "bit {index} should favour 0, got {value}");
            } else {
                assert!(value < 0.0, "bit {index} should favour 1, got {value}");
            }
        }
    }

    #[test]
    fn demap_qpsk_matches_figure_3_15() {
        let scale = Modulation::Qpsk.demap_scale();
        for b0 in 0..2u8 {
            for b1 in 0..2u8 {
                let symbol = Complex32::new(level2(b0) / scale, level2(b1) / scale);
                let soft = demap(symbol, Modulation::Qpsk);
                assert_bit_signs(&soft, &[b0, b1]);
            }
        }
    }

    #[test]
    fn demap_16qam_matches_figure_3_17() {
        let scale = Modulation::Qam16.demap_scale();
        for b0 in 0..2u8 {
            for b1 in 0..2u8 {
                for b2 in 0..2u8 {
                    for b3 in 0..2u8 {
                        let symbol =
                            Complex32::new(level16(b0, b2) / scale, level16(b1, b3) / scale);
                        let soft = demap(symbol, Modulation::Qam16);
                        assert_bit_signs(&soft, &[b0, b1, b2, b3]);
                    }
                }
            }
        }
    }

    #[test]
    fn demap_64qam_matches_figure_3_19() {
        let scale = Modulation::Qam64.demap_scale();
        for pattern in 0..64u32 {
            let b0 = ((pattern >> 5) & 1) as u8;
            let b1 = ((pattern >> 4) & 1) as u8;
            let b2 = ((pattern >> 3) & 1) as u8;
            let b3 = ((pattern >> 2) & 1) as u8;
            let b4 = ((pattern >> 1) & 1) as u8;
            let b5 = (pattern & 1) as u8;
            let symbol = Complex32::new(level64(b0, b2, b4) / scale, level64(b1, b3, b5) / scale);
            let soft = demap(symbol, Modulation::Qam64);
            assert_bit_signs(&soft, &[b0, b1, b2, b3, b4, b5]);
        }
    }

    #[test]
    fn demap_dqpsk_matches_table_3_10() {
        // Table 3-10: b0b1 = 00 -> +pi/4, 01 -> -pi/4, 10 -> +3pi/4, 11 -> -3pi/4.
        let cases: [(u8, u8, f32); 4] = [
            (0, 0, std::f32::consts::FRAC_PI_4),
            (0, 1, -std::f32::consts::FRAC_PI_4),
            (1, 0, 3.0 * std::f32::consts::FRAC_PI_4),
            (1, 1, -3.0 * std::f32::consts::FRAC_PI_4),
        ];
        for (b0, b1, phase) in cases {
            // Both transmitted symbols are Table 3-11 normalised to unit
            // magnitude, so the differential product `current * conj(previous)`
            // also has magnitude 1; `demap_scale()` is sqrt(2).
            let differential = Complex32::from_polar(1.0, phase);
            let soft = demap(differential, Modulation::Dqpsk);
            assert_bit_signs(&soft, &[b0, b1]);
        }
    }

    #[test]
    fn demap_handles_non_finite_input() {
        let soft = demap(Complex32::new(f32::NAN, f32::INFINITY), Modulation::Qam16);
        assert_eq!(soft.len(), 4);
        assert!(soft.iter().all(|v| v.is_finite()));
    }

    // ---- PRBS -------------------------------------------------------------

    #[test]
    fn prbs_state_decodes_the_spec_initial_value() {
        let expected = [1u8, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0];
        let state = PRBS_INIT;
        for (index, &bit) in expected.iter().enumerate() {
            assert_eq!((state >> index) & 1, bit as u16, "D{}", index + 1);
        }
    }

    /// Independent Fibonacci recurrence: stages `d[0] = D1` .. `d[14] = D15`.
    fn prbs_reference(steps: usize) -> Vec<u8> {
        let mut d = [0u8; 15];
        for (index, bit) in [1u8, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            .iter()
            .enumerate()
        {
            d[index] = *bit;
        }
        let mut out = Vec::with_capacity(steps);
        for _ in 0..steps {
            let output = d[13] ^ d[14];
            out.push(output);
            for index in (1..15).rev() {
                d[index] = d[index - 1];
            }
            d[0] = output;
        }
        out
    }

    #[test]
    fn prbs_matches_independent_shift_register() {
        let mut state = PRBS_INIT;
        let generated: Vec<u8> = (0..333).map(|_| prbs_step(&mut state)).collect();
        assert_eq!(generated, prbs_reference(333));
    }

    #[test]
    fn prbs_byte_is_eight_steps() {
        let mut a = PRBS_INIT;
        let mut b = PRBS_INIT;
        let byte = prbs_byte(&mut a);
        let mut expected = 0u8;
        for _ in 0..8 {
            expected = (expected << 1) | prbs_step(&mut b);
        }
        assert_eq!(byte, expected);
        assert_eq!(a, b);
    }

    // ---- streaming Viterbi ------------------------------------------------

    #[test]
    fn streaming_viterbi_recovers_encoder_input() {
        let mut state = 0x1234_5678u32;
        let bits: Vec<u8> = (0..400).map(|_| (xorshift(&mut state) & 1) as u8).collect();
        let mut encoder = K7Encoder::new();
        let encoded = encoder.encode(&bits);
        let soft: Vec<f32> = encoded
            .iter()
            .map(|&b| if b == 0 { 1.0 } else { -1.0 })
            .collect();

        let mut viterbi = StreamingViterbi::new();
        for pair in soft.chunks(2) {
            viterbi.push(pair[0], pair[1]);
        }
        viterbi.flush();
        let decoded: Vec<u8> = viterbi.pending.iter().copied().collect();
        assert_eq!(decoded, bits);
    }

    #[test]
    fn streaming_viterbi_survives_soft_erasures() {
        let mut state = 0x0BAD_F00Du32;
        let bits: Vec<u8> = (0..300).map(|_| (xorshift(&mut state) & 1) as u8).collect();
        let mut encoder = K7Encoder::new();
        let encoded = encoder.encode(&bits);
        let mut soft: Vec<f32> = encoded
            .iter()
            .map(|&b| if b == 0 { 1.0 } else { -1.0 })
            .collect();
        // Erase a handful of soft values.
        for index in [3usize, 40, 119, 200] {
            soft[index] = 0.0;
        }
        let mut viterbi = StreamingViterbi::new();
        for pair in soft.chunks(2) {
            viterbi.push(pair[0], pair[1]);
        }
        viterbi.flush();
        let decoded: Vec<u8> = viterbi.pending.iter().copied().collect();
        assert_eq!(decoded, bits);
    }

    // ---- transmitter-side test chain --------------------------------------

    fn make_ts(seed: u8) -> [u8; TS_PACKET_SIZE] {
        let mut packet = [0u8; TS_PACKET_SIZE];
        packet[0] = TS_SYNC;
        packet[1] = 0x00;
        packet[2] = 0x20;
        packet[3] = 0b0001_0000 | (seed & 0x0F); // payload-only, continuity counter.
        for (index, byte) in packet[4..].iter_mut().enumerate() {
            *byte = (seed as usize * 31 + index * 7) as u8;
        }
        packet
    }

    fn encode_soft(interleaved: &[u8], modulation: Modulation, rate: PunctureRate) -> Vec<f32> {
        bit_interleave_bits(&punctured_bits(interleaved, modulation, rate), modulation)
            .into_iter()
            .map(|bit| if bit == 0 { 1.0 } else { -1.0 })
            .collect()
    }

    fn decode_chunked(
        mode: Mode,
        soft: &[f32],
        config: LayerConfig,
        chunk: usize,
    ) -> Vec<[u8; TS_PACKET_SIZE]> {
        let mut decoder = ChannelDecoder::with_mode(mode, config).unwrap();
        let mut out = Vec::new();
        for slice in soft.chunks(chunk) {
            out.extend(decoder.push_soft(slice));
        }
        out.extend(decoder.flush());
        out
    }

    /// Build the ideal soft-bit stream for `packets` transmitted in `mode`.
    ///
    /// [`build_frame`] rotates the whole concatenation left by one byte so that
    /// it begins on the information byte next to the first sync (§3.5); the
    /// energy-dispersal helper then restarts the PRBS every `packets_per_frame`
    /// transmission TSPs.
    fn frame_soft(
        mode: Mode,
        packets: &[[u8; TS_PACKET_SIZE]],
        modulation: Modulation,
        rate: PunctureRate,
    ) -> Vec<f32> {
        let frame_bytes = packets_per_frame(mode, modulation, rate) * TSP_SIZE;
        let frame = build_frame(packets);
        let scrambled = energy_disperse(&frame, frame_bytes);
        let interleaved = byte_interleave(&scrambled);
        encode_soft(&interleaved, modulation, rate)
    }

    #[test]
    fn byte_interleaver_deinterleaver_round_trip() {
        // Verify the complementary structure against the transmitter model by
        // driving the decoder's own de-interleaver through its byte stage.
        let mut state = 0xABCD_1234u32;
        let input: Vec<u8> = (0..(BYTE_INTERLEAVE_LATENCY + 600))
            .map(|_| (xorshift(&mut state) & 0xFF) as u8)
            .collect();
        let interleaved = byte_interleave(&input);

        // Decoder-side reference of the same complementary structure.
        let mut branches: Vec<VecDeque<u8>> = (0..BYTE_INTERLEAVE_BRANCHES)
            .map(|branch| {
                VecDeque::from(vec![
                    0u8;
                    (BYTE_INTERLEAVE_BRANCHES - 1 - branch)
                        * BYTE_INTERLEAVE_STEP
                ])
            })
            .collect();
        let mut output = Vec::with_capacity(interleaved.len());
        for (index, &byte) in interleaved.iter().enumerate() {
            let branch = index % BYTE_INTERLEAVE_BRANCHES;
            branches[branch].push_back(byte);
            output.push(branches[branch].pop_front().unwrap());
        }
        for (index, &expected) in input.iter().enumerate() {
            let output_index = index + BYTE_INTERLEAVE_LATENCY;
            if output_index >= output.len() {
                break;
            }
            assert_eq!(output[output_index], expected, "byte {index}");
        }
    }

    #[test]
    fn full_chain_recovers_ts_packets() {
        let combinations = [
            (Modulation::Qpsk, PunctureRate::Rate1_2),
            (Modulation::Dqpsk, PunctureRate::Rate1_2),
            (Modulation::Qam16, PunctureRate::Rate1_2),
            (Modulation::Qam64, PunctureRate::Rate1_2),
            (Modulation::Qpsk, PunctureRate::Rate2_3),
            (Modulation::Qpsk, PunctureRate::Rate3_4),
            (Modulation::Qpsk, PunctureRate::Rate5_6),
            (Modulation::Qpsk, PunctureRate::Rate7_8),
        ];

        for (modulation, rate) in combinations {
            let count = match rate {
                PunctureRate::Rate7_8 => 56,
                _ => 60,
            };
            let packets: Vec<[u8; TS_PACKET_SIZE]> = (0..count).map(|k| make_ts(k as u8)).collect();
            let soft = frame_soft(Mode::Mode3, &packets, modulation, rate);
            let config = LayerConfig {
                modulation,
                rate,
                interleave: 0,
            };
            let decoded = decode_chunked(Mode::Mode3, &soft, config, soft.len().max(1));

            assert!(
                !decoded.is_empty(),
                "{modulation:?}/{rate:?} recovered no packets"
            );
            for packet in &decoded {
                // Every emitted packet must be a real, unmodified packet and the
                // first (partial, pre-sync) TSP must never be emitted.
                assert!(
                    packets[1..].contains(packet),
                    "{modulation:?}/{rate:?} emitted a fabricated packet"
                );
            }
            let distinct: std::collections::HashSet<_> = decoded.iter().collect();
            assert!(
                distinct.len() >= 15,
                "{modulation:?}/{rate:?} only recovered {} packets",
                distinct.len()
            );
        }
    }

    #[test]
    fn chunked_and_single_push_are_identical() {
        let packets: Vec<[u8; TS_PACKET_SIZE]> = (0..60).map(|k| make_ts(k as u8)).collect();
        let soft = frame_soft(
            Mode::Mode3,
            &packets,
            Modulation::Qpsk,
            PunctureRate::Rate2_3,
        );
        let config = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate2_3,
            interleave: 4,
        };
        let whole = decode_chunked(Mode::Mode3, &soft, config, soft.len().max(1));
        for chunk in [1usize, 3, 17, 128, 4096] {
            let split = decode_chunked(Mode::Mode3, &soft, config, chunk);
            assert_eq!(whole, split, "chunk size {chunk}");
        }
    }

    #[test]
    fn corrupted_stream_never_fabricates_packets() {
        let packets: Vec<[u8; TS_PACKET_SIZE]> = (0..60).map(|k| make_ts(k as u8)).collect();
        let mut soft = frame_soft(
            Mode::Mode3,
            &packets,
            Modulation::Qpsk,
            PunctureRate::Rate1_2,
        );
        for (index, value) in soft.iter_mut().enumerate() {
            if index % 997 == 0 {
                *value = -*value;
            }
        }
        let config = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 0,
        };
        let decoded = decode_chunked(Mode::Mode3, &soft, config, soft.len().max(1));
        for packet in &decoded {
            assert!(
                packets[1..].contains(packet),
                "fabricated packet after errors"
            );
        }
    }

    #[test]
    fn reset_restarts_the_frame() {
        let packets: Vec<[u8; TS_PACKET_SIZE]> = (0..60).map(|k| make_ts(k as u8)).collect();
        let soft = frame_soft(
            Mode::Mode3,
            &packets,
            Modulation::Qpsk,
            PunctureRate::Rate1_2,
        );
        let config = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 0,
        };

        let mut decoder = ChannelDecoder::with_mode(Mode::Mode3, config).unwrap();
        decoder.push_soft(&soft[..1000]);
        decoder.reset();
        let decoded = compute_with(&mut decoder, &soft);
        assert!(decoded.iter().all(|packet| packets[1..].contains(packet)));
        assert!(!decoded.is_empty());
    }

    fn compute_with(decoder: &mut ChannelDecoder, soft: &[f32]) -> Vec<[u8; TS_PACKET_SIZE]> {
        let mut out = Vec::new();
        for slice in soft.chunks(1024) {
            out.extend(decoder.push_soft(slice));
        }
        out.extend(decoder.flush());
        out
    }

    #[test]
    fn rejects_invalid_interleave() {
        let config = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 3,
        };
        assert_eq!(
            ChannelDecoder::new(config).unwrap_err(),
            CodingError::InvalidInterleave(3)
        );
    }

    /// Table 3-3 transmission TSPs per OFDM frame for every mode, modulation
    /// and coding rate. DQPSK shares QPSK's two bits per carrier symbol.
    #[test]
    fn packets_per_frame_matches_table_3_3() {
        use PunctureRate::*;
        let rates = [Rate1_2, Rate2_3, Rate3_4, Rate5_6, Rate7_8];
        let table: [(Mode, Modulation, [usize; 5]); 12] = [
            (Mode::Mode1, Modulation::Dqpsk, [12, 16, 18, 20, 21]),
            (Mode::Mode1, Modulation::Qpsk, [12, 16, 18, 20, 21]),
            (Mode::Mode1, Modulation::Qam16, [24, 32, 36, 40, 42]),
            (Mode::Mode1, Modulation::Qam64, [36, 48, 54, 60, 63]),
            (Mode::Mode2, Modulation::Dqpsk, [24, 32, 36, 40, 42]),
            (Mode::Mode2, Modulation::Qpsk, [24, 32, 36, 40, 42]),
            (Mode::Mode2, Modulation::Qam16, [48, 64, 72, 80, 84]),
            (Mode::Mode2, Modulation::Qam64, [72, 96, 108, 120, 126]),
            (Mode::Mode3, Modulation::Dqpsk, [48, 64, 72, 80, 84]),
            (Mode::Mode3, Modulation::Qpsk, [48, 64, 72, 80, 84]),
            (Mode::Mode3, Modulation::Qam16, [96, 128, 144, 160, 168]),
            (Mode::Mode3, Modulation::Qam64, [144, 192, 216, 240, 252]),
        ];

        for (mode, modulation, expected) in table {
            for (&rate, &want) in rates.iter().zip(expected.iter()) {
                assert_eq!(
                    packets_per_frame(mode, modulation, rate),
                    want,
                    "{mode:?}/{modulation:?}/{rate:?}"
                );
            }
        }
    }

    #[test]
    fn differential_product_matches_definition() {
        let current = Complex32::new(0.5, 0.5);
        let previous = Complex32::new(0.5, -0.5);
        let difference = dqpsk_differential(current, previous);
        assert!(difference.re.abs() < 1e-6);
        assert!((difference.im - 0.5).abs() < 1e-6);
    }

    /// Nominal QPSK symbol for `(b0, b1)` (Fig. 3-15) with the Table 3-11
    /// normalisation applied.
    fn map_qpsk(b0: u8, b1: u8) -> Complex32 {
        let scale = Modulation::Qpsk.demap_scale();
        Complex32::new(level2(b0) / scale, level2(b1) / scale)
    }

    #[test]
    fn demap_qpsk_chain_decodes_end_to_end() {
        let packets: Vec<[u8; TS_PACKET_SIZE]> = (0..60).map(|k| make_ts(k as u8)).collect();
        let frame_bytes =
            packets_per_frame(Mode::Mode3, Modulation::Qpsk, PunctureRate::Rate1_2) * TSP_SIZE;
        let frame = build_frame(&packets);
        let scrambled = energy_disperse(&frame, frame_bytes);
        let interleaved = byte_interleave(&scrambled);
        let punctured = punctured_bits(&interleaved, Modulation::Qpsk, PunctureRate::Rate1_2);
        let transmitted = bit_interleave_bits(&punctured, Modulation::Qpsk);

        // Map through the nominal constellation and soft-demap again; the
        // resulting soft stream must decode exactly like the ideal one.
        let mut soft = Vec::with_capacity(transmitted.len());
        for symbol in transmitted.chunks(2) {
            let point = map_qpsk(symbol[0], symbol[1]);
            soft.extend(demap(point, Modulation::Qpsk));
        }

        let config = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 0,
        };
        let decoded = decode_chunked(Mode::Mode3, &soft, config, soft.len().max(1));
        assert!(!decoded.is_empty());
        assert!(decoded.iter().all(|packet| packets[1..].contains(packet)));
    }

    #[test]
    fn inverted_polarity_is_detected() {
        let packets: Vec<[u8; TS_PACKET_SIZE]> = (0..60).map(|k| make_ts(k as u8)).collect();
        let mut soft = frame_soft(
            Mode::Mode3,
            &packets,
            Modulation::Qpsk,
            PunctureRate::Rate1_2,
        );
        // A 180-degree constellation ambiguity inverts every decoded bit, which
        // maps the sync byte 0x47 to 0xB8.
        for value in &mut soft {
            *value = -*value;
        }
        let config = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 0,
        };
        let decoded = decode_chunked(Mode::Mode3, &soft, config, soft.len().max(1));
        assert!(!decoded.is_empty());
        assert!(decoded.iter().all(|packet| packets[1..].contains(packet)));
    }

    /// At least three complete Mode-3 QPSK rate-1/2 frames through the actual
    /// full transmit chain (RS, per-frame energy dispersal, byte interleaving,
    /// K=7, puncturing, bit interleaving), decoded in arbitrary chunks.
    ///
    /// The streaming bit/byte de-interleavers and the Viterbi decoder must keep
    /// their state across the per-frame PRBS restarts, so the recovered packets
    /// must be one unbroken run in transmission order after the startup
    /// transient, not merely a subset that happens to fall in the first frame.
    #[test]
    fn multi_frame_qpsk_half_full_chain_is_contiguous() {
        let mode = Mode::Mode3;
        let modulation = Modulation::Qpsk;
        let rate = PunctureRate::Rate1_2;
        let per_frame = packets_per_frame(mode, modulation, rate);
        assert_eq!(per_frame, 48, "Table 3-3 Mode 3 QPSK 1/2");
        let frames = 3usize;
        let count = frames * per_frame;
        let packets: Vec<[u8; TS_PACKET_SIZE]> = (0..count).map(|k| make_ts(k as u8)).collect();
        let soft = frame_soft(mode, &packets, modulation, rate);
        let config = LayerConfig {
            modulation,
            rate,
            interleave: 0,
        };

        // Chunk sizes deliberately include values that straddle carrier symbols,
        // the 204-byte interleaver unit and the OFDM frame; all must decode to
        // the same sequence because the streaming state belongs to the decoder,
        // not to the push call.
        let mut reference: Option<Vec<[u8; TS_PACKET_SIZE]>> = None;
        for chunk in [1usize, 2, 3, 7, 64, 204, 4096] {
            let decoded = decode_chunked(mode, &soft, config, chunk);
            match &reference {
                None => reference = Some(decoded),
                Some(expected) => assert_eq!(*expected, decoded, "chunk size {chunk}"),
            }
        }
        let decoded = reference.expect("at least one chunk size ran");

        assert!(!decoded.is_empty(), "no packets recovered");
        // The first decoded packet must be a real transmission packet, and the
        // partial pre-sync TSP (rotated to the end by `build_frame`) must never
        // be emitted.
        let start = packets
            .iter()
            .position(|packet| packet == &decoded[0])
            .expect("first decoded packet must be a real transmission packet");
        assert!(start >= 1, "the partial pre-sync TSP must not be emitted");
        for (index, packet) in decoded.iter().enumerate() {
            assert_eq!(
                packet,
                &packets[start + index],
                "packet {index} is not contiguous in transmission order"
            );
        }

        // Packets from the second and third frames must be present: if the PRBS
        // or either de-interleaver had been reset at a frame boundary, the run
        // would stop at the first frame.
        let last = start + decoded.len() - 1;
        assert!(
            last >= 2 * per_frame,
            "run reached only packet {last}; later frames were not decoded"
        );
        // The run must reach essentially the end of the stream. Up to one
        // transmission TSP per byte-interleaver step can remain in the receiver
        // byte de-interleaver at end of stream (`flush` drains only the
        // Viterbi traceback), plus the incomplete rotated TSP.
        let tail_pipeline = BYTE_INTERLEAVE_LATENCY / TSP_SIZE + 2;
        assert!(
            decoded.len() + tail_pipeline >= count - start,
            "recovered only {} of {} decodable packets",
            decoded.len(),
            count - start
        );
    }

    /// The byte stream feeding [`FrameSync`] runs continuously across a frame
    /// boundary while only the PRBS phase restarts. Two full Mode-3 QPSK rate-1/2
    /// frames are rotated and dispersed exactly as a transmitter would, then
    /// pushed byte by byte; every complete codeword, including the one whose
    /// PRBS phase wraps mid-codeword, must decode to the matching TS packet.
    #[test]
    fn frame_sync_wraps_prbs_without_disturbing_bytes() {
        let mode = Mode::Mode3;
        let modulation = Modulation::Qpsk;
        let rate = PunctureRate::Rate1_2;
        let per_frame = packets_per_frame(mode, modulation, rate);
        let frame_bytes = per_frame * TSP_SIZE;
        let packets: Vec<[u8; TS_PACKET_SIZE]> =
            (0..(2 * per_frame)).map(|k| make_ts(k as u8)).collect();
        let rotated = build_frame(&packets);
        let dispersed = energy_disperse(&rotated, frame_bytes);

        let mut sync = FrameSync::new(frame_bytes, per_frame);
        let mut decoded = Vec::new();
        for &byte in &dispersed {
            for block in sync.push(byte) {
                let mut codeword = block;
                codeword[0] = TS_SYNC;
                let data = match fec::rs204_188_decode(&codeword) {
                    Ok(data) => data,
                    Err(_) => continue,
                };
                let mut packet = [0u8; TS_PACKET_SIZE];
                packet.copy_from_slice(&data);
                if fec::validate_ts_packet(&packet).is_ok() {
                    decoded.push(packet);
                }
            }
        }

        // The first block starts at the sync following the rotated-away byte, so
        // the decoded packets are a contiguous tail of `packets[1..]` ending
        // before the final, incomplete rotated TSP.
        assert!(
            decoded.len() >= packets.len() - 3,
            "frame-sync lost packets at the frame boundary: {} of {}",
            decoded.len(),
            packets.len()
        );
        for (index, packet) in decoded.iter().enumerate() {
            assert_eq!(packet, &packets[1 + index], "frame-sync packet {index}");
        }
    }

    /// Decode a raw dispersed byte stream through [`FrameSync`], feeding the
    /// RS/TS result back through [`FrameSync::note_block`] exactly as
    /// [`ChannelDecoder::finish_block`] does.
    fn frame_sync_decode(sync: &mut FrameSync, bytes: &[u8]) -> Vec<[u8; TS_PACKET_SIZE]> {
        let mut out = Vec::new();
        for &byte in bytes {
            for block in sync.push(byte) {
                let mut codeword = block;
                if sync.inverted {
                    for value in codeword.iter_mut() {
                        *value ^= 0xFF;
                    }
                }
                codeword[0] = TS_SYNC;
                let packet = fec::rs204_188_decode(&codeword).ok().and_then(|data| {
                    let mut packet = [0u8; TS_PACKET_SIZE];
                    packet.copy_from_slice(&data);
                    fec::validate_ts_packet(&packet).is_ok().then_some(packet)
                });
                sync.note_block(packet.is_some());
                if let Some(packet) = packet {
                    out.push(packet);
                }
            }
        }
        out
    }

    /// A single dropped byte corrupts the sync grid; the locked synchroniser
    /// must notice the run of RS failures, release the lock and recover the
    /// byte/PRBS phase from the retained raw bytes, so the valid tail after the
    /// drop still decodes.
    #[test]
    fn frame_sync_reacquires_after_dropped_byte() {
        let mode = Mode::Mode3;
        let modulation = Modulation::Qpsk;
        let rate = PunctureRate::Rate1_2;
        let per_frame = packets_per_frame(mode, modulation, rate);
        let frame_bytes = per_frame * TSP_SIZE;
        // Enough frames to establish the lock, drop, and recover afterwards.
        let packets: Vec<[u8; TS_PACKET_SIZE]> =
            (0..(4 * per_frame)).map(|k| make_ts(k as u8)).collect();
        let rotated = build_frame(&packets);
        let dispersed = energy_disperse(&rotated, frame_bytes);

        // Remove one byte well inside the third frame.
        let drop_at = frame_bytes * 2 + TSP_SIZE / 2;
        let mut damaged = Vec::with_capacity(dispersed.len() - 1);
        damaged.extend_from_slice(&dispersed[..drop_at]);
        damaged.extend_from_slice(&dispersed[drop_at + 1..]);

        let mut sync = FrameSync::new(frame_bytes, per_frame);
        let decoded = frame_sync_decode(&mut sync, &damaged);

        assert!(
            sync.locked,
            "the synchroniser did not re-acquire after the byte drop"
        );
        assert!(
            !decoded.is_empty(),
            "no packets decoded after the byte drop"
        );
        for packet in &decoded {
            assert!(
                packets[1..].contains(packet),
                "frame-sync fabricated a packet after the drop"
            );
        }

        // Only the TSP straddling the dropped byte is unrecoverable; every later
        // TSP must come back after re-acquisition.
        assert!(
            decoded.len() >= packets.len() - 5,
            "byte-phase loss was not recovered: {} of {} packets",
            decoded.len(),
            packets.len()
        );

        // The tail after re-acquisition must be a contiguous run ending at the
        // end of the transmitted stream.
        let tail_len = 20.min(decoded.len());
        let tail = &decoded[decoded.len() - tail_len..];
        let last_index = packets
            .iter()
            .position(|packet| packet == tail.last().expect("non-empty tail"))
            .expect("tail packet must be a real transmission packet");
        for (offset, packet) in tail.iter().enumerate() {
            assert_eq!(
                packet,
                &packets[last_index + 1 - tail_len + offset],
                "tail is not contiguous after re-acquisition (offset {offset})"
            );
        }
    }
}
