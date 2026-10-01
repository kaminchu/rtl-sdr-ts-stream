//! OFDM-frame acquisition and TMCC validation for the ISDB-T centre
//! (partial-reception) segment.
//!
//! The normative basis is ARIB STD-B31 version 2.2-E1:
//!
//! * §3.15.2 / Table 3-20 — the 204-bit TMCC assignment `B0..B203`: `B0` is the
//!   differential reference, `B1..B16` the frame synchronising word, `B17..B19`
//!   the segment type, `B20..B121` the information and `B122..B203` the
//!   shortened `(184,102)` parity.
//! * §3.15.4 — the synchronising word alternates between `w0 =
//!   0011010111101110` and `w1 = 1100101000010001` on every OFDM frame.
//! * §3.13.3 — TMCC is a DBPSK signal, one carrier bit per OFDM symbol, with
//!   `B'0 = Wi` and `B'k = B'k-1 XOR Bk`. The signal points are `(+4/3, 0)` for
//!   information `0` and `(-4/3, 0)` for information `1`.
//!
//! # What this module does
//!
//! [`FrameCollector`] consumes the `108 / 216 / 432` centre-segment carriers of
//! one OFDM symbol at a time, exactly as delivered by
//! [`crate::demod::ofdm::extract_symbol`]. It keeps a sliding window of the last
//! [`SYMBOLS_PER_FRAME`] (204) symbols and, for every complete window:
//!
//! 1. **Differentially demodulates the TMCC bit stream.** For each symbol `k`
//!    the sign of `sum_c Re(X_k[c] * conj(X_{k-1}[c]))` over the *coherent* TMCC
//!    carrier positions of [`crate::demod::control::tmcc_carriers`] recovers the
//!    information bit `Bk = B'k XOR B'k-1` (§3.13.3). `B0` is the differential
//!    reference and is deliberately arbitrary: the first window symbol is not
//!    compared with anything and `B0` is fixed to `0`.
//! 2. **Re-differential-encodes** the recovered `B1..B203` into the received
//!    `B'1..B'203` that [`crate::demod::control::decode_tmcc_one_seg`] expects
//!    (`B'k = B'k-1 XOR Bk`) and hands them to that function.
//! 3. **Validates parity and synchronisation** through the existing
//!    [`crate::demod::control`] API, which also checks the segment type, system
//!    identification, switching indicator, partial-reception flag and current
//!    layer-A parameters. A window that fails any check is simply not a frame.
//! 4. **Confirms frame synchronisation** by requiring two valid frames that are
//!    exactly [`SYMBOLS_PER_FRAME`] symbols apart and whose synchronising words
//!    have opposite polarity (§3.15.4). The confirming frame — and every
//!    validated frame after it — is returned as a [`ReceivedFrame`] holding the
//!    complete, frame-aligned 204 carrier vectors.
//!
//! Noise, a wrong mode, a truncated prefix and a frame with broken parity or
//! synchronisation never produce a [`ReceivedFrame`]: the window is only
//! accepted when the normative checks pass *and* the alternating-frame cadence
//! has been established.
//!
//! The TMCC carrier positions used are the coherent ones of Table 3-15. They
//! are a subset of the differential TMCC positions of Table 3-14, so the same
//! DBPSK demodulation also recovers TMCC from a differential (DQPSK) segment
//! (§3.15.6.9: all TMCC carriers carry the same bit). A window is accepted only
//! when the decoded segment-type identification agrees with the decoded layer-A
//! carrier modulation (Table 3-1: DQPSK requires `Differential`, QPSK/16QAM/64QAM
//! require `Coherent`); an inconsistent pair is never reported as a frame.
//!
//! # Out of scope
//!
//! * Equalisation / channel estimation ([`crate::demod::carriers`]).
//! * Time, frequency, bit or byte de-interleaving.
//! * Energy-dispersal removal, Viterbi, Reed-Solomon and TS framing
//!   ([`crate::demod::coding`], [`crate::fec`]).
//! * Any `IQ -> TS` pipeline. This module only turns a stream of centre-segment
//!   carrier symbols into validated, aligned OFDM frames; it never claims to
//!   decode payload data.
//!
//! # Bounds and untrusted input
//!
//! The internal history is hard-capped at [`SYMBOLS_PER_FRAME`] vectors, plus
//! the single preceding symbol retained for differential demodulation, each
//! validated to hold exactly the mode's carrier count, so a caller cannot force
//! unbounded memory growth. [`FrameCollector::push`] rejects a wrong-length
//! symbol with [`FrameError::CarrierCount`] and never panics on malformed
//! input; non-finite samples simply fail the TMCC checks.

use std::collections::VecDeque;

use num_complex::Complex32;

use crate::dsp::Mode;

use super::coding::{LayerConfig, Modulation};
use super::control::{
    carriers_per_segment, decode_tmcc_one_seg, tmcc_carriers, SegmentType, TmccSync,
    SYMBOLS_PER_FRAME, TMCC_TOTAL_BITS,
};

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by [`FrameCollector::push`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FrameError {
    /// A supplied OFDM symbol did not hold the mode's `108 / 216 / 432`
    /// centre-segment carriers.
    #[error("OFDM symbol carrier count mismatch for {mode:?}: expected {expected}, got {got}")]
    CarrierCount {
        /// The mode the collector was built for.
        mode: Mode,
        /// [`carriers_per_segment`] of that mode.
        expected: usize,
        /// Number of carriers actually supplied.
        got: usize,
    },
    /// The decoded segment-type identification disagreed with the decoded
    /// layer-A carrier modulation. `Dqpsk` requires [`SegmentType::Differential`];
    /// QPSK/16QAM/64QAM require [`SegmentType::Coherent`] (ARIB STD-B31
    /// Tables 3-1, 3-14 and 3-15).
    #[error(
        "TMCC segment type {segment_type:?} does not match layer-A modulation {modulation:?} \
         for {mode:?}"
    )]
    SegmentTypeMismatch {
        /// The mode the collector was built for.
        mode: Mode,
        /// Case `B17..B19` of the decoded TMCC frame.
        segment_type: SegmentType,
        /// Layer-A carrier modulation decoded from the same frame.
        modulation: Modulation,
    },
    /// The TMCC window did not decode as a valid one-segment frame.
    ///
    /// This is an ordinary acquisition outcome (noise, wrong mode, broken
    /// parity or synchronisation, partial-reception clear, …), not a hard
    /// failure; the collector keeps searching. It is boxed to keep
    /// [`FrameError`] small.
    #[error("TMCC window is not a valid one-segment frame: {0}")]
    Tmcc(Box<super::control::TmccError>),
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

/// One validated, frame-aligned OFDM frame of centre-segment carriers.
#[derive(Debug, Clone, PartialEq)]
pub struct ReceivedFrame {
    /// Transmission mode the carriers belong to.
    pub mode: Mode,
    /// Current layer-A decoding configuration decoded from TMCC.
    pub config: LayerConfig,
    /// The [`SYMBOLS_PER_FRAME`] carrier vectors of the frame, in frame order:
    /// index `0` carries `B0`, index `203` carries `B203`.
    pub carriers: Vec<Vec<Complex32>>,
    /// The raw centre-segment carriers of the OFDM symbol immediately preceding
    /// this frame, i.e. `B'203` of the previous frame in the differential TMCC
    /// numbering.
    ///
    /// DQPSK data demodulation needs this to form `current * conj(previous)`
    /// (Table 3-10): the first symbol of a frame is differential to the last
    /// symbol of the preceding frame, whose raw carriers are not otherwise part
    /// of `carriers`.
    ///
    /// For a *first* frame returned after a [`FrameCollector::reset`] no
    /// preceding symbol exists, so this is `None`; every subsequent valid frame
    /// of a stream carries `Some`. For the coherent segment it is informational
    /// only.
    pub previous_carriers: Option<Vec<Complex32>>,
}

// ---------------------------------------------------------------------------
// Collector
// ---------------------------------------------------------------------------

/// Streaming TMCC frame collector / synchroniser.
///
/// Built for one [`Mode`]; [`push`](FrameCollector::push) is fed one
/// centre-segment OFDM symbol at a time and returns a [`ReceivedFrame`] whenever
/// a validated frame completes the alternating-frame cadence. See the module
/// documentation for the exact acceptance rules.
#[derive(Debug, Clone)]
pub struct FrameCollector {
    mode: Mode,
    /// Coherent TMCC carrier positions (Table 3-15, segment 0 column).
    tmcc: &'static [usize],
    /// Carriers expected per OFDM symbol.
    per_symbol: usize,
    /// Sliding window of the most recent symbols (at most 204).
    history: VecDeque<Vec<Complex32>>,
    /// The symbol immediately preceding `history.front()`, retained for the
    /// differential demodulation of the first frame symbol. Empty until the
    /// window is full.
    preceding: Vec<Complex32>,
    /// Absolute index (0-based) of the last symbol of the last valid frame.
    last_valid_end: Option<u64>,
    /// Synchronising word of the last valid frame.
    last_sync: Option<TmccSync>,
    /// Total number of symbols accepted by [`push`](FrameCollector::push).
    symbols_pushed: u64,
}

impl FrameCollector {
    /// Create a collector for the centre segment of `mode`.
    ///
    /// The TMCC carrier positions are the coherent-modulation ones of Table
    /// 3-15; all coherent schemes share them, and they are a subset of the
    /// differential positions of Table 3-14, so the same collector acquires
    /// TMCC for either segment type.
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            tmcc: tmcc_carriers(Modulation::Qpsk, mode),
            per_symbol: carriers_per_segment(mode),
            history: VecDeque::with_capacity(SYMBOLS_PER_FRAME),
            preceding: Vec::new(),
            last_valid_end: None,
            last_sync: None,
            symbols_pushed: 0,
        }
    }

    /// The mode this collector was built for.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Feed one OFDM symbol's centre-segment carriers (ascending segment-local
    /// carrier order).
    ///
    /// Returns `Ok(Some(frame))` exactly when this symbol completed a validated
    /// frame whose synchronising word alternates with the immediately preceding
    /// valid frame [`SYMBOLS_PER_FRAME`] symbols earlier. Otherwise returns
    /// `Ok(None)` — including for every symbol of an incomplete window, for
    /// noise, and for windows that fail the TMCC parity/synchronisation checks.
    ///
    /// # Errors
    ///
    /// [`FrameError::CarrierCount`] when `carriers.len()` is not the mode's
    /// carrier count. The offending symbol is not retained.
    pub fn push(&mut self, carriers: &[Complex32]) -> Result<Option<ReceivedFrame>, FrameError> {
        if carriers.len() != self.per_symbol {
            return Err(FrameError::CarrierCount {
                mode: self.mode,
                expected: self.per_symbol,
                got: carriers.len(),
            });
        }

        self.history.push_back(carriers.to_vec());
        if self.history.len() > SYMBOLS_PER_FRAME {
            // The window grew to 205: the oldest symbol leaves the window and
            // becomes the differential predecessor of the new `history.front()`.
            self.preceding = self
                .history
                .pop_front()
                .expect("history is non-empty (length 205)");
        }
        self.symbols_pushed += 1;

        if self.history.len() < SYMBOLS_PER_FRAME {
            return Ok(None);
        }

        let decoded = match self.decode_window() {
            Ok(decoded) => decoded,
            // A window that does not decode as a valid one-segment TMCC frame is
            // simply not a frame; keep searching rather than surfacing TMCC
            // details. A hard inconsistency (segment type vs. modulation) is
            // reported so a caller can never silently assume the wrong
            // demodulator.
            Err(FrameError::Tmcc(_)) => return Ok(None),
            Err(error) => return Err(error),
        };

        let end = self.symbols_pushed - 1;
        let confirmed = self
            .last_valid_end
            .is_some_and(|previous| end == previous + SYMBOLS_PER_FRAME as u64)
            && self
                .last_sync
                .is_some_and(|previous| previous != decoded.sync);

        self.last_valid_end = Some(end);
        self.last_sync = Some(decoded.sync);

        if !confirmed {
            return Ok(None);
        }

        Ok(Some(ReceivedFrame {
            mode: self.mode,
            config: decoded.config,
            carriers: self.history.iter().cloned().collect(),
            // `preceding` is the symbol before the window once it is full, i.e.
            // `B'203` of the previous frame. It is empty only for a frame
            // returned right after a reset (one frame of lost history).
            previous_carriers: if self.preceding.is_empty() {
                None
            } else {
                Some(self.preceding.clone())
            },
        }))
    }

    /// Forget every buffered symbol and the frame-synchronisation state.
    ///
    /// The next [`push`](FrameCollector::push) starts acquisition from scratch.
    pub fn reset(&mut self) {
        self.history.clear();
        self.preceding.clear();
        self.last_valid_end = None;
        self.last_sync = None;
        self.symbols_pushed = 0;
    }

    /// Differentially demodulate and validate the current 204-symbol window.
    ///
    /// Returns the decoded layer-A configuration and synchronising word, or an
    /// error if the window is not a valid one-segment TMCC frame. The segment
    /// type is checked against the layer-A modulation so a DQPSK segment is
    /// never silently accepted as coherent (or vice versa).
    fn decode_window(&self) -> Result<DecodedWindow, FrameError> {
        // The history is exactly `SYMBOLS_PER_FRAME` entries (== TMCC_TOTAL_BITS).
        debug_assert_eq!(self.history.len(), TMCC_TOTAL_BITS);

        // The sign of `Re(X_k * conj(X_{k-1}))` is the *information* bit `Bk`
        // for `k = 1..203` (§3.13.3); the per-carrier Wi reference and any flat
        // channel gain cancel in the product. Re-differential-encode those bits
        // into the transmitted `B'` stream that `decode_tmcc_one_seg` expects
        // (`B'k = B'k-1 XOR Bk`). The control API then performs the differential
        // decode itself, so recovering `B'` here rather than `B` is deliberate
        // and must not be changed to a second decode (that would doubly decode).
        // `B'0` is arbitrary (it is the Wi reference, not compared against
        // anything), so it is fixed to `0`; the parity check ignores it.
        let mut bits = [0u8; TMCC_TOTAL_BITS];
        let consecutive = self.history.iter().zip(self.history.iter().skip(1));
        for (index, (previous, current)) in consecutive.enumerate() {
            let mut accumulator = 0.0f32;
            for &carrier in self.tmcc {
                accumulator += (current[carrier] * previous[carrier].conj()).re;
            }
            // `bits[k] = B'k = B'k-1 XOR Bk`, with `bits[0] = B'0 = 0`.
            bits[index + 1] = bits[index] ^ u8::from(accumulator < 0.0);
        }

        let one_seg = decode_tmcc_one_seg(self.mode, 0, &bits)
            .map_err(|error| FrameError::Tmcc(Box::new(error)))?;

        let expected = match one_seg.layer_a.modulation {
            Modulation::Dqpsk => SegmentType::Differential,
            Modulation::Qpsk | Modulation::Qam16 | Modulation::Qam64 => SegmentType::Coherent,
        };
        if one_seg.info.segment_type != expected {
            return Err(FrameError::SegmentTypeMismatch {
                mode: self.mode,
                segment_type: one_seg.info.segment_type,
                modulation: one_seg.layer_a.modulation,
            });
        }

        Ok(DecodedWindow {
            config: one_seg.layer_a,
            sync: one_seg.info.sync,
        })
    }
}

/// Internal result of a successful window decode.
#[derive(Debug, Clone, Copy)]
struct DecodedWindow {
    config: LayerConfig,
    sync: TmccSync,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    use crate::demod::coding::Modulation;
    use crate::demod::control::{
        pilot_modulating_signal, PilotPrbs, TMCC_CODEWORD_BITS, TMCC_INFO_BITS, TMCC_INFO_OFFSET,
        TMCC_SEGMENT_TYPE_OFFSET, TMCC_SYNC_OFFSET, TMCC_SYNC_W0, TMCC_SYNC_W1,
    };
    use crate::fec::PunctureRate;

    fn set_bits(dst: &mut [u8], start: usize, len: usize, value: u16) {
        for i in 0..len {
            dst[start + i] = ((value >> (len - 1 - i)) & 1) as u8;
        }
    }

    /// Three-bit `B17..B19` encoding of a segment type (Table 3-20).
    fn segment_type_code(segment_type: SegmentType) -> u16 {
        match segment_type {
            SegmentType::Differential => 0b111,
            SegmentType::Coherent => 0b000,
        }
    }

    /// Build the 204 TMCC bits `B0..B203` of a syntactically valid one-segment
    /// coherent frame, exactly as the transmitter would (control-plane
    /// reference).
    #[allow(clippy::too_many_arguments)]
    fn build_frame_bits(
        modulation: Modulation,
        rate: PunctureRate,
        interleave_code: u8,
        segments: u8,
        partial: bool,
        switching: u8,
        sync: TmccSync,
    ) -> [u8; TMCC_TOTAL_BITS] {
        build_frame_bits_with_segment(
            modulation,
            SegmentType::Coherent,
            rate,
            interleave_code,
            segments,
            partial,
            switching,
            sync,
        )
    }

    /// As [`build_frame_bits`], but with an explicitly chosen `B17..B19`
    /// segment type so a mismatched identification can be synthesised.
    #[allow(clippy::too_many_arguments)]
    fn build_frame_bits_with_segment(
        modulation: Modulation,
        segment_type: SegmentType,
        rate: PunctureRate,
        interleave_code: u8,
        segments: u8,
        partial: bool,
        switching: u8,
        sync: TmccSync,
    ) -> [u8; TMCC_TOTAL_BITS] {
        let mut info = vec![1u8; TMCC_INFO_BITS];
        set_bits(&mut info, 0, 2, 0b00); // terrestrial television
        set_bits(&mut info, 2, 4, switching as u16);
        info[6] = 0; // no emergency start
        info[7] = u8::from(partial);
        let modulation_code = match modulation {
            Modulation::Dqpsk => 0b000,
            Modulation::Qpsk => 0b001,
            Modulation::Qam16 => 0b010,
            Modulation::Qam64 => 0b011,
        };
        let rate_code = match rate {
            PunctureRate::Rate1_2 => 0b000,
            PunctureRate::Rate2_3 => 0b001,
            PunctureRate::Rate3_4 => 0b010,
            PunctureRate::Rate5_6 => 0b011,
            PunctureRate::Rate7_8 => 0b100,
        };
        set_bits(&mut info, 8, 3, modulation_code);
        set_bits(&mut info, 11, 3, rate_code);
        set_bits(&mut info, 14, 3, interleave_code as u16);
        set_bits(&mut info, 17, 4, segments as u16);

        let codeword = crate::demod::control::tmcc_encode(&info).unwrap();
        let mut bits = [0u8; TMCC_TOTAL_BITS];
        set_bits(&mut bits, TMCC_SYNC_OFFSET, 16, sync.word());
        set_bits(
            &mut bits,
            TMCC_SEGMENT_TYPE_OFFSET,
            3,
            segment_type_code(segment_type),
        );
        bits[TMCC_INFO_OFFSET..TMCC_INFO_OFFSET + TMCC_CODEWORD_BITS].copy_from_slice(&codeword);
        bits
    }

    /// Modulate the 204 TMCC bits into a frame of raw centre-segment carrier
    /// vectors, using the pilot PRBS as the DBPSK reference of §3.13.3.
    fn build_frame_carriers(mode: Mode, bits: &[u8; TMCC_TOTAL_BITS]) -> Vec<Vec<Complex32>> {
        let per_symbol = carriers_per_segment(mode);
        let tmcc = tmcc_carriers(Modulation::Qpsk, mode);
        let (amplitude, _) = pilot_modulating_signal(0).unwrap();

        // Differential encoding: B'0 = 0 (the per-carrier Wi reference is added
        // below), B'k = B'k-1 XOR Bk. XOR is linear, so the per-carrier
        // reference can be folded in at the end.
        let mut b_prime = [0u8; TMCC_TOTAL_BITS];
        for k in 1..TMCC_TOTAL_BITS {
            b_prime[k] = b_prime[k - 1] ^ bits[k];
        }

        let mut frame = Vec::with_capacity(TMCC_TOTAL_BITS);
        for &encoded in b_prime.iter() {
            let mut symbol = vec![Complex32::new(0.0, 0.0); per_symbol];
            for &carrier in tmcc {
                let reference = PilotPrbs::bit_at(mode, 0, carrier)
                    .expect("TMCC carrier is inside the segment");
                let signal_bit = encoded ^ reference;
                let sign = if signal_bit == 0 { 1.0 } else { -1.0 };
                symbol[carrier] = Complex32::new(amplitude * sign, 0.0);
            }
            frame.push(symbol);
        }
        frame
    }

    fn valid_bits(sync: TmccSync) -> [u8; TMCC_TOTAL_BITS] {
        build_frame_bits(
            Modulation::Qpsk,
            PunctureRate::Rate2_3,
            0b001,
            1,
            true,
            0b1111,
            sync,
        )
    }

    fn expected_config() -> LayerConfig {
        LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate2_3,
            interleave: 4, // Mode 1, interleaving code 001
        }
    }

    #[test]
    fn locks_only_after_two_alternating_frames_and_stays_aligned() {
        let mode = Mode::Mode1;
        let mut collector = FrameCollector::new(mode);

        // A partial prefix of noise-like symbols must not yield anything (and
        // must not alias the frame alignment).
        for i in 0..50usize {
            let value = Complex32::new(((i * 37) % 11) as f32 - 5.0, ((i * 17) % 7) as f32);
            assert!(collector.push(&[value; 108]).unwrap().is_none());
        }

        let frames = [
            (TmccSync::W0, valid_bits(TmccSync::W0)),
            (TmccSync::W1, valid_bits(TmccSync::W1)),
            (TmccSync::W0, valid_bits(TmccSync::W0)),
            (TmccSync::W1, valid_bits(TmccSync::W1)),
        ];

        let mut outputs = Vec::new();
        for (index, (sync, bits)) in frames.iter().enumerate() {
            let carriers = build_frame_carriers(mode, bits);
            let mut last = None;
            for symbol in &carriers {
                last = collector.push(symbol).unwrap();
            }
            // Frame 0 establishes the candidate; frame 1 confirms and is the
            // first output. Every later alternating frame is output.
            if index == 0 {
                assert!(last.is_none(), "a single frame must not be trusted");
            } else {
                let frame = last.expect("confirmed frame must be returned");
                assert_eq!(frame.mode, mode);
                assert_eq!(frame.config, expected_config());
                assert_eq!(frame.carriers.len(), SYMBOLS_PER_FRAME);
                assert_eq!(frame.carriers, carriers, "frame {index} must be aligned");
                outputs.push((*sync, frame));
            }
        }

        assert_eq!(outputs.len(), 3);
        assert_eq!(outputs[0].0, TmccSync::W1);
        assert_eq!(outputs[1].0, TmccSync::W0);
        assert_eq!(outputs[2].0, TmccSync::W1);
    }

    #[test]
    fn wrong_parity_frame_never_produces_a_frame() {
        let mode = Mode::Mode1;
        let mut collector = FrameCollector::new(mode);

        let good = valid_bits(TmccSync::W0);
        let good_carriers = build_frame_carriers(mode, &good);

        // Corrupt a parity bit (B122 region) so the (184,102) check fails.
        let mut bad = valid_bits(TmccSync::W1);
        bad[TMCC_INFO_OFFSET + 110] ^= 1;
        let bad_carriers = build_frame_carriers(mode, &bad);

        let mut outputs = Vec::new();
        let feed = |collector: &mut FrameCollector,
                    carriers: &[Vec<Complex32>],
                    outputs: &mut Vec<ReceivedFrame>| {
            for symbol in carriers {
                if let Some(frame) = collector.push(symbol).unwrap() {
                    outputs.push(frame);
                }
            }
        };

        feed(&mut collector, &good_carriers, &mut outputs); // candidate W0
        feed(&mut collector, &bad_carriers, &mut outputs); // rejected
        assert!(outputs.is_empty());

        // The rejected frame occupied a frame slot, so the next good frame is
        // 2 * 204 symbols after the candidate and only becomes a new candidate.
        let next = build_frame_carriers(mode, &valid_bits(TmccSync::W1));
        feed(&mut collector, &next, &mut outputs);
        assert!(outputs.is_empty(), "a gap breaks the 204-symbol cadence");

        // The frame after that alternates and confirms; it is emitted.
        let confirmed = build_frame_carriers(mode, &valid_bits(TmccSync::W0));
        feed(&mut collector, &confirmed, &mut outputs);
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0].carriers, confirmed);
    }

    #[test]
    fn wrong_sync_frame_never_produces_a_frame() {
        let mode = Mode::Mode2;
        let mut collector = FrameCollector::new(mode);

        let mut bad = valid_bits(TmccSync::W0);
        // Flip a bit inside the 16-bit synchronising word (B1..B16).
        bad[TMCC_SYNC_OFFSET + 3] ^= 1;
        let bad_carriers = build_frame_carriers(mode, &bad);

        let mut last = None;
        for symbol in &bad_carriers {
            last = collector.push(symbol).unwrap();
        }
        assert!(last.is_none());

        // A valid frame still establishes a candidate rather than being lost.
        let good = build_frame_carriers(mode, &valid_bits(TmccSync::W1));
        for symbol in &good {
            last = collector.push(symbol).unwrap();
        }
        assert!(last.is_none());
    }

    #[test]
    fn noise_and_truncation_never_produce_a_frame() {
        let mode = Mode::Mode1;
        let mut collector = FrameCollector::new(mode);

        // Deterministic pseudo-noise (xorshift32) for several frame lengths.
        let mut state = 0x1234_5678u32;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state as f32 / u32::MAX as f32) * 2.0 - 1.0
        };
        for _ in 0..(3 * SYMBOLS_PER_FRAME) {
            let symbol: Vec<Complex32> = (0..108).map(|_| Complex32::new(next(), next())).collect();
            assert!(collector.push(&symbol).unwrap().is_none());
        }

        // A single late valid frame is still not trusted.
        let frame = build_frame_carriers(mode, &valid_bits(TmccSync::W0));
        let mut last = None;
        for symbol in &frame {
            last = collector.push(symbol).unwrap();
        }
        assert!(last.is_none());
    }

    #[test]
    fn reset_restarts_acquisition() {
        let mode = Mode::Mode1;
        let mut collector = FrameCollector::new(mode);

        let first = build_frame_carriers(mode, &valid_bits(TmccSync::W0));
        let second = build_frame_carriers(mode, &valid_bits(TmccSync::W1));

        let mut last = None;
        for symbol in &first {
            last = collector.push(symbol).unwrap();
        }
        assert!(last.is_none(), "one frame is only a candidate");
        for symbol in &second {
            last = collector.push(symbol).unwrap();
        }
        assert!(last.is_some(), "the alternating frame confirms");

        collector.reset();

        // After reset the second frame is only a fresh candidate, and the third
        // (same polarity as the second) is not an alternating confirmation.
        let third = build_frame_carriers(mode, &valid_bits(TmccSync::W1));
        for symbol in &second {
            assert!(collector.push(symbol).unwrap().is_none());
        }
        for symbol in &third {
            assert!(collector.push(symbol).unwrap().is_none());
        }
    }

    #[test]
    fn carrier_count_is_validated() {
        let mode = Mode::Mode2;
        let mut collector = FrameCollector::new(mode);
        assert_eq!(
            collector.push(&[Complex32::new(0.0, 0.0); 108]),
            Err(FrameError::CarrierCount {
                mode,
                expected: 216,
                got: 108,
            })
        );
        assert_eq!(collector.mode(), mode);

        // A rejected symbol must not have advanced the window.
        assert!(collector
            .push(&[Complex32::new(0.0, 0.0); 216])
            .unwrap()
            .is_none());
    }

    #[test]
    fn every_standard_mode_can_lock() {
        for (mode, interleave_code, expected_interleave) in [
            (Mode::Mode1, 0b010, 8),
            (Mode::Mode2, 0b001, 2),
            (Mode::Mode3, 0b011, 4),
        ] {
            let mut collector = FrameCollector::new(mode);
            let bits = |sync| {
                build_frame_bits(
                    Modulation::Qpsk,
                    PunctureRate::Rate2_3,
                    interleave_code,
                    1,
                    true,
                    0b1111,
                    sync,
                )
            };

            let mut output = None;
            for sync in [TmccSync::W0, TmccSync::W1] {
                let frame = build_frame_carriers(mode, &bits(sync));
                for symbol in &frame {
                    output = collector.push(symbol).unwrap();
                }
            }
            let frame = output.expect("alternating pair must lock");
            assert_eq!(frame.mode, mode);
            assert_eq!(frame.config.modulation, Modulation::Qpsk);
            assert_eq!(frame.config.rate, PunctureRate::Rate2_3);
            assert_eq!(frame.config.interleave, expected_interleave);
            assert_eq!(frame.carriers.len(), SYMBOLS_PER_FRAME);
            assert_eq!(frame.carriers[0].len(), carriers_per_segment(mode));
        }
    }

    #[test]
    fn synchronising_words_are_the_normative_pair() {
        // Guard the test helper's assumption against the control-plane constants.
        assert_eq!(TmccSync::W0.word(), TMCC_SYNC_W0);
        assert_eq!(TmccSync::W1.word(), TMCC_SYNC_W1);
        assert_ne!(TMCC_SYNC_W0, TMCC_SYNC_W1);
    }

    /// Syntactically valid differential (DQPSK) one-segment frame bits.
    fn differential_bits(sync: TmccSync) -> [u8; TMCC_TOTAL_BITS] {
        build_frame_bits_with_segment(
            Modulation::Dqpsk,
            SegmentType::Differential,
            PunctureRate::Rate2_3,
            0b001,
            1,
            true,
            0b1111,
            sync,
        )
    }

    #[test]
    fn dqpsk_frame_capture_tracks_previous_carriers() {
        let mode = Mode::Mode1;
        let mut collector = FrameCollector::new(mode);

        let frames = [
            build_frame_carriers(mode, &differential_bits(TmccSync::W0)),
            build_frame_carriers(mode, &differential_bits(TmccSync::W1)),
            build_frame_carriers(mode, &differential_bits(TmccSync::W0)),
        ];

        // The first frame is only a candidate: it must not be reported and must
        // not manufacture a previous symbol.
        let mut output = None;
        for symbol in &frames[0] {
            output = collector.push(symbol).unwrap();
        }
        assert!(output.is_none(), "a single DQPSK frame is not trusted");

        // The alternating frame confirms and is the first output. Its
        // `previous_carriers` must be exactly the last symbol of the preceding
        // frame, never some earlier frame's or a misaligned symbol.
        for symbol in &frames[1] {
            output = collector.push(symbol).unwrap();
        }
        let first = output
            .as_ref()
            .expect("an alternating DQPSK pair must confirm");
        assert_eq!(first.mode, mode);
        assert_eq!(first.config.modulation, Modulation::Dqpsk);
        assert_eq!(first.carriers, frames[1]);
        assert_eq!(
            first.previous_carriers.as_ref(),
            Some(&frames[0][SYMBOLS_PER_FRAME - 1]),
            "previous_carriers must be the symbol just before the frame"
        );

        // The next frame advances the window: its previous symbol is the last
        // symbol of the middle frame, not the first frame's.
        for symbol in &frames[2] {
            output = collector.push(symbol).unwrap();
        }
        let second = output.as_ref().expect("the next alternating DQPSK frame");
        assert_eq!(second.carriers, frames[2]);
        assert_eq!(
            second.previous_carriers.as_ref(),
            Some(&frames[1][SYMBOLS_PER_FRAME - 1])
        );
    }

    #[test]
    fn coherent_frame_capture_tracks_previous_carriers() {
        let mode = Mode::Mode2;
        let mut collector = FrameCollector::new(mode);

        let first = build_frame_carriers(mode, &valid_bits(TmccSync::W0));
        let second = build_frame_carriers(mode, &valid_bits(TmccSync::W1));

        let mut output = None;
        for symbol in &first {
            output = collector.push(symbol).unwrap();
        }
        assert!(output.is_none());
        for symbol in &second {
            output = collector.push(symbol).unwrap();
        }
        let frame = output.expect("an alternating coherent pair must confirm");
        assert_eq!(frame.carriers, second);
        assert_eq!(
            frame.previous_carriers.as_ref(),
            Some(&first[SYMBOLS_PER_FRAME - 1])
        );
    }

    #[test]
    fn segment_type_modulation_mismatch_is_reported() {
        // Table 3-1: differential segments identify `Differential` and must use
        // DQPSK layer A, coherent segments identify `Coherent` and must not use
        // DQPSK. A mismatched pair is a hard error rather than a silent frame.
        let mismatches = [
            (Modulation::Dqpsk, SegmentType::Coherent),
            (Modulation::Qpsk, SegmentType::Differential),
            (Modulation::Qam64, SegmentType::Differential),
        ];

        for (modulation, segment_type) in mismatches {
            let mode = Mode::Mode1;
            let mut collector = FrameCollector::new(mode);
            let bits = build_frame_bits_with_segment(
                modulation,
                segment_type,
                PunctureRate::Rate2_3,
                0b001,
                1,
                true,
                0b1111,
                TmccSync::W0,
            );
            let carriers = build_frame_carriers(mode, &bits);

            let mut error = None;
            for symbol in &carriers {
                match collector.push(symbol) {
                    Ok(_) => {}
                    Err(found) => {
                        error = Some(found);
                        break;
                    }
                }
            }
            assert_eq!(
                error,
                Some(FrameError::SegmentTypeMismatch {
                    mode,
                    segment_type,
                    modulation,
                }),
                "modulation {modulation:?} with segment type {segment_type:?}"
            );
        }
    }

    #[test]
    fn matching_segment_type_is_accepted() {
        // Guard against an over-eager check: the normative pairings decode.
        for (modulation, segment_type) in [
            (Modulation::Dqpsk, SegmentType::Differential),
            (Modulation::Qpsk, SegmentType::Coherent),
            (Modulation::Qam16, SegmentType::Coherent),
        ] {
            let mode = Mode::Mode1;
            let mut collector = FrameCollector::new(mode);
            let bits = build_frame_bits_with_segment(
                modulation,
                segment_type,
                PunctureRate::Rate2_3,
                0b001,
                1,
                true,
                0b1111,
                TmccSync::W0,
            );
            let carriers = build_frame_carriers(mode, &bits);
            let mut err = None;
            for symbol in &carriers {
                if let Err(found) = collector.push(symbol) {
                    err = Some(found);
                    break;
                }
            }
            assert_eq!(err, None, "{modulation:?} / {segment_type:?}");
        }
    }
}
