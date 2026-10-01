//! Streaming `u8` IQ → 188-byte TS receiver for the ISDB-T centre
//! (partial-reception) segment.
//!
//! This is an original implementation written from scratch for this crate. No
//! GNU Radio, `gr-isdbt`, Python or other existing demodulator source was
//! consulted or copied. It wires the independently implemented demodulator
//! primitives into one streaming pipeline:
//!
//! ```text
//! u8 IQ bytes
//!   -> normalise (u8 -> Complex32) + gentle DC block
//!   -> rational resample (input `fs` -> one-segment rate 64/63 MHz)
//!   -> Synchronizer            (Mode / guard / timing / CFO)
//!   -> FrameCollector          (per locked mode, TMCC frame + layer-A config)
//!   -> equalised carriers      (coherent equalise *or* DQPSK differential,
//!                               + frequency de-interleave)
//!   -> TimeDeinterleaver       (per locked config, expected I)
//!   -> demap                   (soft bits)
//!   -> ChannelDecoder          (bit/byte de-interleave, Viterbi, RS, TS)
//! ```
//!
//! # Rates
//!
//! The one-segment rate is `fs = 64/63 MHz` (ARIB STD-B31 Table 3-1: the
//! full-band FFT sampling frequency `2048/252 MHz` divided by eight). For an
//! input rate `R` the resampling ratio is
//!
//! ```text
//! interp / decim = (64e6 / g) / (63 * R / g),  g = gcd(64e6, 63 * R).
//! ```
//!
//! The default RTL-SDR rate 1.2 MHz reduces to `160/189`; 2.048 MHz reduces to
//! `125/252`. Ratios whose reduced factors exceed the existing
//! [`RationalResampler`] limit are rejected by [`Decoder::new`] **before** any
//! per-sample allocation.
//!
//! # Coherent and differential segments
//!
//! [`equalize_symbol`] is the coherent path: it estimates the channel from the
//! scattered pilots and removes the intra-segment frequency interleaving.
//! Differential (DQPSK) segments carry neither scattered pilots nor a channel
//! estimate, so they use [`differential_symbol`], which forms the normalised
//! product `current * conj(previous)` for every data carrier (Table 3-10) and
//! then frequency de-interleaves it. The differential predecessor of a symbol
//! is the immediately preceding OFDM symbol; for the first symbol of a frame
//! that is the last symbol of the preceding frame, which
//! [`ReceivedFrame::previous_carriers`] supplies. A DQPSK frame whose
//! predecessor is unknown is skipped whole rather than demodulated against the
//! wrong reference, so no TS is fabricated.
//!
//! # Streaming and locking rules
//!
//! * The pipeline state (synchroniser, frame collector, time de-interleaver,
//!   channel decoder) is **never reset at a frame boundary**. It is reset only
//!   on an IQ gap, loss of synchronisation, a change of Mode/guard/config, or a
//!   missing/invalid TMCC window that persists past the stale limit.
//! * Every [`SyncedSymbol`] carries the synchroniser lock
//!   [`generation`](SyncedSymbol::generation). A change of generation means the
//!   lock was lost and re-established (possibly inside one `push`), so the
//!   frame collector and the time/FEC pipeline are discarded to prevent symbols
//!   from two generations being concatenated into one frame. The synchroniser
//!   itself is already locked to the new generation and is **not** reset, and
//!   the remaining symbols of the batch are still decoded against the fresh
//!   state.
//! * [`Decoder::is_locked`] means the decoder has *recently produced a real,
//!   RS-validated 188-byte TS packet* — not merely that the OFDM control plane
//!   (TMCC / CP) is synchronised.
//! * Buffers are bounded and partial input is carried across calls, so
//!   fragmented input is handled exactly like contiguous input and a caller
//!   cannot force unbounded memory growth.
//!
//! # Statistics
//!
//! [`Decoder::stats`] exposes counters a `stderr` caller can log: input
//! bytes/samples, gaps, validated TMCC frames, emitted TS packets and reset
//! reasons.

use std::collections::VecDeque;

use num_complex::Complex32;

use crate::dsp::{
    normalize_u8_iq_into, DcBlocker, DspError, GuardInterval, Mode, RationalResampler,
};

use super::carriers::{differential_symbol, equalize_symbol, CarrierError};
use super::coding::{demap, ChannelDecoder, CodingError, LayerConfig, Modulation};
use super::control::{
    data_carriers_per_segment, ControlError, TimeDeinterleaver, MAX_TIME_INTERLEAVE,
    SYMBOLS_PER_FRAME,
};
use super::frame::{FrameCollector, FrameError, ReceivedFrame};
use super::ofdm::segment_fft_size;
use super::sync::{restore_timing_offset, SyncedSymbol, Synchronizer};

// ---------------------------------------------------------------------------
// Rates
// ---------------------------------------------------------------------------

/// Numerator of the one-segment baseband rate `64/63 MHz` (ARIB Table 3-1).
pub const SEGMENT_RATE_NUM: u32 = 64_000_000;
/// Denominator of the one-segment baseband rate `64/63 MHz`.
pub const SEGMENT_RATE_DEN: u32 = 63;

/// Default RTL-SDR input rate accepted by [`Decoder::new`] (1.2 MHz).
pub const DEFAULT_INPUT_RATE: u32 = 1_200_000;

/// Minimum input rate accepted by [`Decoder::new`].
///
/// Only the centre of the channel (approximately 1.15 MHz) is demodulated, so a
/// rate just above the one-segment rate (`64/63 MHz`) is sufficient. The
/// default RTL-SDR rate [`DEFAULT_INPUT_RATE`] is the smallest accepted rate;
/// below this the useful band cannot be represented.
pub const MIN_INPUT_RATE: u32 = DEFAULT_INPUT_RATE;

/// Conservative upper bound on the input rate accepted by [`Decoder::new`].
///
/// [`RationalResampler`] caps each reduced factor at 4096; this ceiling keeps
/// the range check explicit while the resampler still validates the exact ratio
/// (and rejects the rare rate that reduces badly).
pub const MAX_INPUT_RATE: u32 = 6_000_000;

/// Pole of the gentle one-pole DC blocker.
///
/// Close to one gives a narrow notch that suppresses a static centre-frequency
/// offset without eating the occupied signal band. "Gentle" is deliberate: the
/// blocker must not act as a strong high-pass that removes real low-frequency
/// data energy, and its state is continuous across calls (never reset per
/// frame).
const DC_POLE: f32 = 0.9995;

/// Symbols without a validated TMCC frame (or without emitted TS) after the
/// pipeline is established, before the stale pipeline is reset. Two frames.
const STALE_LIMIT_SYMBOLS: u64 = 2 * SYMBOLS_PER_FRAME as u64;

/// Frames of grace beyond the time-deinterleaver warm-up (`I * 95` symbols)
/// before a TMCC-valid but TS-less FEC pipeline is rebuilt.
///
/// The warm-up covers the time de-interleaver; the extra frames cover the
/// byte/Viterbi pipeline and a margin for lossy channels. Initial acquisition
/// always installs the pipeline on a validated frame, so this cannot fire
/// before the first frame is even collected.
const TS_STALL_FRAMES: u64 = 3;

/// Emitted symbols a fresh control-plane (CP) lock may run without ever
/// validating a TMCC frame before the domain is reset.
///
/// A genuine signal needs at least two frame periods to fill the TMCC window
/// and confirm the alternating synchronising words (`2 * 204` symbols); the
/// allowance is doubled to four so acquisition margin and a lossy first frame
/// never truncate a valid lock. This is the only stale check that applies before
/// a pipeline exists: without it a false CP lock (wrong timing / Mode on noise)
/// would never be torn down, because [`enforce_stale`](Decoder::enforce_stale)
/// used to return early while `pipeline` was `None`.
const PRE_TMCC_TIMEOUT_SYMBOLS: u64 = 4 * SYMBOLS_PER_FRAME as u64;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by [`Decoder`].
#[derive(Debug, Clone, thiserror::Error)]
pub enum ReceiverError {
    /// The sample rate is outside the supported range.
    #[error("unsupported input sample rate {rate} Hz; supported range {min}..={max}")]
    UnsupportedSampleRate {
        /// The rejected rate.
        rate: u32,
        /// Minimum accepted rate.
        min: u32,
        /// Maximum accepted rate.
        max: u32,
    },
    /// The resampling ratio could not be built (too large / degenerate). This
    /// is checked by `RationalResampler` before any per-sample allocation.
    #[error("cannot resample {input} Hz to the one-segment rate: {source}")]
    Resample {
        /// Requested input rate.
        input: u32,
        /// Underlying DSP error.
        #[source]
        source: DspError,
    },
    /// The front-end / synchroniser primitive rejected the input.
    #[error(transparent)]
    Dsp(#[from] DspError),
    /// The OFDM frame collector rejected a symbol.
    #[error(transparent)]
    Frame(#[from] FrameError),
    /// Coherent channel estimation / equalisation failed.
    #[error(transparent)]
    Carrier(#[from] CarrierError),
    /// Channel decoding failed.
    #[error(transparent)]
    Coding(#[from] CodingError),
    /// A lower-level control-plane primitive rejected the input.
    #[error(transparent)]
    Control(#[from] ControlError),
    /// Retained for API compatibility.
    ///
    /// DQPSK layer-A segments are now demodulated by
    /// [`differential_symbol`], so the decoder never returns this variant. It is
    /// kept so callers (and the session backend) that still match on it keep
    /// compiling; it can be removed once that match is retired.
    #[error("DQPSK layer A is not supported by the coherent receiver")]
    DqpskUnsupported,
    /// An internal invariant was violated (should not happen for valid input).
    #[error("internal receiver invariant: {0}")]
    Internal(String),
}

// ---------------------------------------------------------------------------
// Configuration / statistics
// ---------------------------------------------------------------------------

/// Immutable configuration captured when a [`Decoder`] is built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReceiverConfig {
    /// Original RTL-SDR sample rate in hertz.
    pub input_rate: u32,
    /// Reduced resampler interpolation factor (maps `fs` to the one-segment
    /// rate as `interp / decim`).
    pub resample_interp: usize,
    /// Reduced resampler decimation factor.
    pub resample_decim: usize,
}

/// Aggregate counters for diagnostics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ReceiverStats {
    /// Number of IQ bytes accepted (including an odd trailing byte).
    pub input_bytes: u64,
    /// Number of complete complex samples fed to the pipeline.
    pub input_samples: u64,
    /// Number of resampled samples fed to the synchroniser.
    pub resampled_samples: u64,
    /// Number of IQ gaps observed.
    pub gaps: u64,
    /// Number of validated, frame-aligned TMCC frames.
    pub tmcc_frames: u64,
    /// Number of 188-byte TS packets returned to the caller.
    pub ts_packets: u64,
    /// Number of pipeline resets (gap, sync loss / generation change, config
    /// change or stale TMCC window).
    pub resets: u64,
    /// Subset of [`ReceiverStats::resets`] caused by an IQ gap.
    pub resets_gap: u64,
    /// Subset caused by sync loss, a Mode / guard change or a change of
    /// synchroniser lock generation.
    pub resets_sync: u64,
    /// Subset caused by a layer-configuration change.
    pub resets_config: u64,
    /// Subset caused by a missing/invalid TMCC window past the stale limit.
    pub resets_tmcc_timeout: u64,
    /// Subset caused by a TMCC-valid but TS-less FEC/time pipeline exceeding its
    /// time-interleaver warm-up plus stall allowance. Only the FEC/time pipeline
    /// is rebuilt; the front end and TMCC acquisition are kept.
    pub resets_ts_stall: u64,
}

/// Plain snapshot of the synchroniser / acquisition diagnostics.
///
/// [`ReceiverStats`] counts the whole pipeline lifetime; these describe the
/// synchroniser's *current* lock. They exist so a session can log whether the
/// control plane ever locked (CP lock), to which Mode / guard, how far the
/// sample clock has drifted (the `timing_offset` SFO trend) and what CFO it
/// settled on — none of which is visible from the aggregate counters alone.
///
/// Every optional field is `None` while the synchroniser is unlocked, so a
/// caller never sees a stale mode / guard / CFO from a previous lock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReceiverDiagnostics {
    /// Whether the synchroniser currently holds a timing/CFO (CP) lock.
    pub cp_locked: bool,
    /// Mode of the current lock, or `None` while unlocked.
    pub mode: Option<Mode>,
    /// Guard interval of the current lock, or `None` while unlocked.
    pub guard: Option<GuardInterval>,
    /// Continuity generation: increments on every (re-)acquisition.
    pub generation: u64,
    /// Internal synchroniser lock losses since construction.
    pub losses: u64,
    /// Confirmed integer-subcarrier CFO, or `None` while unconfirmed.
    pub integer_bins: Option<i32>,
    /// Smoothed CFO estimate in radians per sample, or `None` while unlocked.
    pub cfo: Option<f32>,
    /// Cumulative timing offset in samples (the SFO trend), or `None` while
    /// unlocked.
    pub timing_offset: Option<i64>,
}

// ---------------------------------------------------------------------------
// Pipeline
// ---------------------------------------------------------------------------

/// Mutable demodulator state that is rebuilt when the transmission parameters
/// change. Never reset merely because a frame boundary was crossed.
struct Pipeline {
    mode: Mode,
    config: LayerConfig,
    /// OFDM-symbol counter used for the scattered-pilot residue.
    symbol_index: usize,
    /// Time de-interleaver configured with the expected `I` from TMCC.
    time_deinterleaver: TimeDeinterleaver<Complex32>,
    /// FEC / TS core for the locked configuration.
    channel: ChannelDecoder,
}

impl Pipeline {
    fn new(mode: Mode, config: LayerConfig) -> Result<Self, ReceiverError> {
        if config.interleave > MAX_TIME_INTERLEAVE {
            return Err(ReceiverError::Internal(format!(
                "time-interleaving length {} out of range",
                config.interleave
            )));
        }
        let time_deinterleaver = TimeDeinterleaver::new(mode, config.interleave)?;
        let channel = ChannelDecoder::with_mode(mode, config)?;
        Ok(Self {
            mode,
            config,
            symbol_index: 0,
            time_deinterleaver,
            channel,
        })
    }

    /// Time-deinterleaver warm-up in OFDM symbols (`I * 95`).
    ///
    /// The FEC/byte stages add a few transmission TSPs on top, covered by the
    /// extra stall frames in [`super::receiver::TS_STALL_FRAMES`].
    fn warmup_symbols(&self) -> u64 {
        self.time_deinterleaver.latency_symbols() as u64
    }

    /// Equalise (or differentially demodulate), time de-interleave, demap and
    /// channel-decode one symbol.
    ///
    /// `previous` is the raw centre-segment carriers of the OFDM symbol
    /// immediately preceding `carriers`. It is required for
    /// [`Modulation::Dqpsk`] (the `pi/4`-shift DQPSK phase change of Table 3-10
    /// is differential) and ignored for the coherent modulations, which
    /// estimate the channel from scattered pilots.
    ///
    /// `timing_offset` is the synchroniser's cumulative SFO de-ramp for this
    /// symbol (see [`SyncedSymbol::timing_offset`]). The coherent path undoes it
    /// before estimating the channel. The DQPSK path uses the de-ramped carriers
    /// unchanged: `current` and its `previous` reference are consecutive symbols
    /// whose cumulative offsets differ, so their de-ramps differ by the
    /// per-symbol increment and do **not** cancel exactly. DQPSK behaviour under
    /// SFO is therefore inherited unchanged and is not corrected or verified
    /// here.
    ///
    /// Returns the TS packets completed by this symbol.
    fn consume_symbol(
        &mut self,
        carriers: &[Complex32],
        previous: Option<&[Complex32]>,
        timing_offset: i64,
    ) -> Result<Vec<[u8; 188]>, ReceiverError> {
        // 1. Coherent equalisation / DQPSK differential demodulation, with
        //    intra-segment frequency de-interleaving.
        let equalised = match self.config.modulation {
            Modulation::Dqpsk => {
                // The synchroniser de-ramps every carrier by the cumulative SFO
                // phase `exp(-j 2*pi*signed_bin*timing_offset/N)`. `current` and
                // `previous` are consecutive symbols whose cumulative offsets
                // differ, so their de-ramps differ by the first difference of the
                // SFO track and do not cancel exactly. The de-ramped carriers are
                // used unchanged (behaviour deliberately preserved); DQPSK SFO
                // correction is unverified and not attempted here.
                let previous = previous.ok_or_else(|| {
                    ReceiverError::Internal(
                        "DQPSK symbol without a differential reference".to_string(),
                    )
                })?;
                differential_symbol(self.mode, carriers, previous)?
            }
            _ => {
                // The coherent scattered-pilot estimator linearly interpolates
                // the channel across the 12-carrier pilot spacing. The SFO
                // de-ramp is a linear phase across the carriers whose 12-carrier
                // step grows without bound with the timing-offset trend, and once
                // it approaches `pi` the interpolation is meaningless. Restore
                // the physical (pre-de-ramp) carriers before equalising. The
                // de-ramp stays on the carriers the frame collector uses, which
                // the differential TMCC detector needs.
                let mut physical = carriers.to_vec();
                restore_timing_offset(&mut physical, timing_offset, segment_fft_size(self.mode));
                equalize_symbol(self.mode, self.symbol_index, &physical)?
            }
        };
        self.symbol_index = self.symbol_index.wrapping_add(1);

        // 2. Time de-interleaving with the TMCC-provided I.
        let deinterleaved = self.time_deinterleaver.process_symbol(&equalised)?;

        // 3. Soft demapping in logical data-carrier order.
        let mut soft: Vec<f32> = Vec::with_capacity(
            data_carriers_per_segment(self.mode) * self.config.modulation.bits_per_symbol(),
        );
        for &symbol in &deinterleaved {
            soft.extend_from_slice(&demap(symbol, self.config.modulation));
        }

        // 4. Channel decoding (bit/byte de-interleaving, Viterbi, RS, PRBS).
        Ok(self.channel.push_soft(&soft))
    }
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// Streaming ISDB-T one-segment `u8` IQ → TS decoder.
///
/// Construct with [`Decoder::new`], feed raw interleaved `I, Q, I, Q, ...`
/// unsigned 8-bit bytes with [`Decoder::push_iq`], and forward the complete
/// 188-byte TS packets it returns. See the module documentation for the exact
/// pipeline and locking/reset rules.
pub struct Decoder {
    config: ReceiverConfig,

    // Front end (continuous unless explicitly reset).
    dc: DcBlocker,
    resampler: RationalResampler,
    /// Reusable normalisation buffer for the current `push_iq` call.
    raw: Vec<Complex32>,

    // Domain acquisition.
    synchronizer: Synchronizer,
    /// Frame collector for the currently observed/locked mode. `None` until the
    /// synchroniser reports a mode or after a reset.
    collector: Option<FrameCollector>,
    /// Per-symbol cumulative SFO timing offsets of the most recent
    /// [`SYMBOLS_PER_FRAME`] synchronised symbols.
    ///
    /// The [`FrameCollector`] works on the de-ramped carriers (which the
    /// differential TMCC products need); this parallel history exists so a
    /// validated frame can hand the coherent scattered-pilot equaliser the exact
    /// `timing_offset` of each of its symbols, letting it
    /// [`restore_timing_offset`] to the physical carriers before interpolating
    /// the channel. It is bounded to one frame and cleared wherever the frame
    /// collector is reset.
    timing_history: VecDeque<i64>,

    // Demod / FEC pipeline. `None` until a valid frame has configured it.
    pipeline: Option<Pipeline>,

    /// TS packets completed during the current `push_iq` call.
    pending_out: Vec<[u8; 188]>,
    /// One unpaired IQ byte retained between calls (odd-length fragment).
    remainder: Option<u8>,
    /// Absolute symbol counter across the current domain lock.
    symbol_count: u64,
    /// Synchroniser lock [`generation`](SyncedSymbol::generation) of the last
    /// symbol consumed. `None` until the first symbol after a domain reset.
    ///
    /// A change of generations means the synchroniser lost and re-established
    /// its lock, so the frame collector and the time/FEC pipeline must be
    /// discarded before this symbol is processed.
    epoch: Option<u64>,
    /// `symbol_count` when the last validated TMCC frame arrived.
    last_frame_symbols: u64,
    /// `symbol_count` when TS packets were last produced.
    last_ts_symbols: u64,
    /// Whether TS was produced recently.
    locked: bool,

    stats: ReceiverStats,
}

impl Decoder {
    /// Build a decoder for an RTL-SDR input rate `sample_rate` (Hz).
    ///
    /// The reduced ratio `interp / decim` maps `sample_rate` exactly onto the
    /// one-segment rate `64/63 MHz`. The default 1.2 MHz rate reduces to
    /// `160/189`; 2.048 MHz reduces to `125/252`.
    ///
    /// All resampler constraints (positive factors, `<= 4096`, bounded
    /// prototype) are validated by [`RationalResampler::with_taps`] **before**
    /// any per-sample allocation happens.
    ///
    /// # Errors
    ///
    /// * [`ReceiverError::UnsupportedSampleRate`] when `sample_rate` is outside
    ///   [`MIN_INPUT_RATE`]`..=`[`MAX_INPUT_RATE`].
    /// * [`ReceiverError::Resample`] when the reduced ratio violates the
    ///   existing [`RationalResampler`] constraints.
    /// * [`ReceiverError::Dsp`] only for structurally impossible constants.
    pub fn new(sample_rate: u32) -> Result<Self, ReceiverError> {
        if !(MIN_INPUT_RATE..=MAX_INPUT_RATE).contains(&sample_rate) {
            return Err(ReceiverError::UnsupportedSampleRate {
                rate: sample_rate,
                min: MIN_INPUT_RATE,
                max: MAX_INPUT_RATE,
            });
        }

        let (interp, decim) = resample_ratio(sample_rate).ok_or(ReceiverError::Resample {
            input: sample_rate,
            source: DspError::InvalidArgument("degenerate resampling ratio".into()),
        })?;

        // Rejects oversized / degenerate factors before any allocation.
        let resampler = RationalResampler::with_taps(interp, decim, 16).map_err(|source| {
            ReceiverError::Resample {
                input: sample_rate,
                source,
            }
        })?;

        let dc = DcBlocker::new(DC_POLE)?;
        Ok(Self {
            config: ReceiverConfig {
                input_rate: sample_rate,
                resample_interp: resampler.interp(),
                resample_decim: resampler.decim(),
            },
            dc,
            resampler,
            raw: Vec::new(),
            synchronizer: Synchronizer::new(),
            collector: None,
            timing_history: VecDeque::with_capacity(SYMBOLS_PER_FRAME),
            pipeline: None,
            pending_out: Vec::new(),
            remainder: None,
            symbol_count: 0,
            epoch: None,
            last_frame_symbols: 0,
            last_ts_symbols: 0,
            locked: false,
            stats: ReceiverStats::default(),
        })
    }

    /// The configuration this decoder was built with.
    pub fn config(&self) -> ReceiverConfig {
        self.config
    }

    /// Running diagnostics.
    pub fn stats(&self) -> ReceiverStats {
        self.stats
    }

    /// Current synchroniser / acquisition diagnostics.
    ///
    /// Unlike [`stats`](Self::stats), which counts the pipeline's whole
    /// lifetime, this reports the *current* CP lock: whether it is held, which
    /// Mode / guard it locked to, its continuity generation and loss count, and
    /// the CFO / timing-offset estimates. A session logs it to tell "no signal"
    /// apart from "CP locked but no TS".
    pub fn diagnostics(&self) -> ReceiverDiagnostics {
        let (mode, guard) = match self.synchronizer.locked_mode_guard() {
            Some((mode, guard)) => (Some(mode), Some(guard)),
            None => (None, None),
        };
        ReceiverDiagnostics {
            cp_locked: self.synchronizer.is_locked(),
            mode,
            guard,
            generation: self.synchronizer.generation(),
            losses: self.synchronizer.loss_count(),
            integer_bins: self.synchronizer.integer_bins(),
            cfo: self.synchronizer.cfo_estimate(),
            timing_offset: self.synchronizer.timing_offset(),
        }
    }

    /// Whether the decoder has recently produced a real RS-validated TS packet.
    ///
    /// This is *not* merely "the OFDM control plane is synchronised": it is set
    /// only when [`Decoder::push_iq`] actually returns TS bytes and is cleared
    /// when synchronisation / configuration is lost or TS stops arriving.
    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Drop every pipeline state and return to the un-locked state.
    ///
    /// The front end (DC blocker, resampler) and the retained partial IQ byte
    /// are reset too.
    pub fn reset(&mut self) {
        self.reset_internal(ResetReason::Caller);
    }

    /// Feed raw interleaved `u8` IQ bytes and return complete TS packets.
    ///
    /// `bytes` is `I, Q, I, Q, ...`. A leading unpaired byte from a previous
    /// call is retained and combined with the first byte of this call. When
    /// `gap` is set the pipeline is reset **before** the new samples are
    /// processed, so a dropped/partial IQ block can never leave stale
    /// synchroniser, equaliser, time-de-interleaver or Viterbi state in place.
    ///
    /// The returned packets are the only data a caller may forward; the decoder
    /// never invents TS.
    ///
    /// # Errors
    ///
    /// Errors are not fatal: the offending pipeline is dropped and the decoder
    /// keeps searching. [`ReceiverError::Carrier`] and
    /// [`ReceiverError::Coding`] are ordinary primitive failures for invalid
    /// data; the structural variants report a configuration problem. Any TS
    /// packets already decoded before the failure stay buffered and are
    /// returned by the next call.
    pub fn push_iq(&mut self, bytes: &[u8], gap: bool) -> Result<Vec<[u8; 188]>, ReceiverError> {
        self.stats.input_bytes = self
            .stats
            .input_bytes
            .saturating_add(u64::try_from(bytes.len()).unwrap_or(u64::MAX));

        if gap {
            self.stats.gaps = self.stats.gaps.saturating_add(1);
            self.reset_internal(ResetReason::Gap);
        }

        // `pending_out` is intentionally *not* cleared here: a previous call may
        // have produced real TS and then returned a non-fatal error, leaving the
        // already-decoded packets buffered. They are delivered here, in order,
        // before anything this call produces.
        self.normalize_and_run(bytes)?;
        Ok(std::mem::take(&mut self.pending_out))
    }

    /// Normalise the retained remainder plus `bytes`, then run the pipeline.
    ///
    /// Exactly one trailing byte is retained across calls when the total byte
    /// count is odd, so fragmented input is processed identically to contiguous
    /// input.
    fn normalize_and_run(&mut self, bytes: &[u8]) -> Result<(), ReceiverError> {
        // Byte source: the retained remainder (if any) followed by `bytes`.
        let (pending, first) = match self.remainder.take() {
            Some(prefix) => {
                if bytes.is_empty() {
                    self.remainder = Some(prefix);
                    return Ok(());
                }
                (Some(prefix), &bytes[1..])
            }
            None => (None, bytes),
        };

        // Keep the odd trailing byte for the next call.
        let usable = first.len() - (first.len() % 2);
        self.remainder = if usable < first.len() {
            Some(first[usable])
        } else {
            None
        };

        let complete = if pending.is_some() {
            1 + usable / 2
        } else {
            usable / 2
        };
        self.raw.clear();
        self.raw.resize(complete, Complex32::new(0.0, 0.0));

        // The remainder (when present) pairs with the first byte of `bytes`.
        if let Some(prefix) = pending {
            let second = bytes[0];
            self.raw[0] = normalize_pair(prefix, second);
            if usable > 0 {
                normalize_u8_iq_into(&first[..usable], &mut self.raw[1..])?;
            }
        } else {
            normalize_u8_iq_into(&first[..usable], &mut self.raw)?;
        }

        self.stats.input_samples = self
            .stats
            .input_samples
            .saturating_add(u64::try_from(complete).unwrap_or(u64::MAX));
        self.run_front_end()
    }

    /// DC-block, resample and drive the domain pipeline for `self.raw`.
    fn run_front_end(&mut self) -> Result<(), ReceiverError> {
        if self.raw.is_empty() {
            return Ok(());
        }
        self.dc.process_in_place(&mut self.raw);

        let mut resampled = Vec::new();
        self.resampler.process(&self.raw, &mut resampled);
        if resampled.is_empty() {
            return Ok(());
        }
        self.stats.resampled_samples = self
            .stats
            .resampled_samples
            .saturating_add(u64::try_from(resampled.len()).unwrap_or(u64::MAX));

        // Snapshot the synchroniser lock so `consume_symbols` can tell a clean
        // batch from one in which the lock was lost. The synchroniser may emit
        // symbols and *then* drop the lock at the end of the batch; those
        // symbols are still valid, but the demodulator state must be discarded
        // before the next (new-generation) batch.
        let was_locked = self.synchronizer.is_locked();
        let symbols = self.synchronizer.push(&resampled);
        self.consume_symbols(&symbols, was_locked)
    }

    /// Drive FrameCollector → Pipeline for every synchronised symbol, appending
    /// emitted packets to `self.pending_out`.
    ///
    /// `was_locked` is the synchroniser lock state *before* this batch. The
    /// synchroniser may lose and re-establish its lock inside one batch, which
    /// is signalled by a change of [`SyncedSymbol::generation`]. That discards
    /// the frame collector and the time/FEC pipeline (so symbols from two
    /// generations can never be concatenated into a frame) but keeps the
    /// synchroniser — already locked to the new generation — and keeps
    /// consuming the rest of the batch against the fresh state.
    ///
    /// A configuration change (or a primitive error) still stops the batch
    /// because the remaining symbols were decoded with a pipeline that has just
    /// been rebuilt.
    fn consume_symbols(
        &mut self,
        symbols: &[SyncedSymbol],
        was_locked: bool,
    ) -> Result<(), ReceiverError> {
        for symbol in symbols {
            self.symbol_count = self.symbol_count.saturating_add(1);

            // A new lock generation (possibly a Mode / guard change) invalidates
            // the frame collector and the time/FEC pipeline. The synchroniser
            // is already locked to this generation, so it is deliberately kept.
            let mode_changed = self
                .collector
                .as_ref()
                .is_some_and(|collector| collector.mode() != symbol.mode);
            if self.epoch != Some(symbol.generation) || mode_changed {
                self.reset_for_epoch(symbol.generation);
            }

            if self.collector.is_none() {
                self.collector = Some(FrameCollector::new(symbol.mode));
            }
            // Keep the SFO timing window aligned with the frame collector's
            // symbol window. Push *before* `collector.push` so that, when this
            // symbol completes a frame, the window already includes it and the
            // last `SYMBOLS_PER_FRAME` offsets match the returned carriers.
            self.timing_history.push_back(symbol.timing_offset);
            if self.timing_history.len() > SYMBOLS_PER_FRAME {
                self.timing_history.pop_front();
            }
            let frame = match self
                .collector
                .as_mut()
                .ok_or_else(|| ReceiverError::Internal("frame collector missing".into()))?
                .push(&symbol.carriers)
            {
                Ok(Some(frame)) => frame,
                Ok(None) => {
                    self.enforce_stale();
                    continue;
                }
                Err(error) => {
                    // Primitive failure: drop the domain exactly once and stop
                    // consuming this batch. Packets already emitted this call
                    // stay buffered in `pending_out`.
                    self.reset_domain(ResetReason::Sync);
                    return Err(error.into());
                }
            };
            self.stats.tmcc_frames = self.stats.tmcc_frames.saturating_add(1);
            self.last_frame_symbols = self.symbol_count;

            if self.consume_frame_into(frame)? {
                // The pipeline was rebuilt for a changed configuration; the rest
                // of the batch is stale.
                return Ok(());
            }
            self.enforce_stale();
        }

        // The synchroniser can emit the tail of a lock and then release it
        // inside the same batch. Those symbols were valid, but the demodulator
        // state must not be carried into the next generation. A lock lost *and*
        // re-acquired within one batch is handled above by the generation
        // check, which keeps the re-established lock.
        if was_locked && !self.synchronizer.is_locked() {
            self.reset_domain(ResetReason::Sync);
        }
        Ok(())
    }

    /// Configure/validate the pipeline for one validated frame and process it.
    ///
    /// Returns `true` when a *changed* configuration rebuilt the demodulator
    /// state, so every remaining symbol of the current batch is stale.
    ///
    /// # Errors
    ///
    /// Any primitive error aborts, drops the pipeline and is returned.
    fn consume_frame_into(&mut self, frame: ReceivedFrame) -> Result<bool, ReceiverError> {
        let ReceivedFrame {
            mode,
            config,
            carriers,
            previous_carriers,
        } = frame;

        // A DQPSK frame is differential to the symbol immediately before it.
        // Without that reference its first symbol cannot be demodulated, and
        // dropping only that symbol would shift the bit/FEC phase of the whole
        // frame. Skip the frame whole and drop the pipeline so the next
        // validated frame starts from a clean time/FEC state instead of being
        // decoded against the wrong reference.
        if config.modulation == Modulation::Dqpsk && previous_carriers.is_none() {
            if self.pipeline.is_some() {
                self.reset_pipeline(ResetReason::Config);
            }
            return Ok(false);
        }

        let reconfigure = match &self.pipeline {
            None => true,
            Some(pipeline) => pipeline.mode != mode || pipeline.config != config,
        };
        let mut rebuilt = false;
        if reconfigure {
            // Only a *change* invalidates existing streaming state; the first
            // configuration just installs the pipeline. A layer-configuration
            // change does not invalidate the front end, the synchroniser or the
            // TMCC collector, so only the time/FEC pipeline is rebuilt and the
            // already-locked synchroniser is kept.
            if self.pipeline.is_some() {
                self.reset_pipeline(ResetReason::Config);
                rebuilt = true;
            }
            self.pipeline = Some(Pipeline::new(mode, config)?);
            // A fresh time/FEC pipeline needs its full warm-up before TS can be
            // expected; start the no-TS stall clock now.
            self.last_ts_symbols = self.symbol_count;
        }

        // The frame's carriers are the collector's last `SYMBOLS_PER_FRAME`
        // symbols, and `timing_history` was pushed in lock-step and capped to
        // the same length, so the two windows hold the same symbols in order.
        // Collect the matching SFO offsets before borrowing the pipeline. A
        // length mismatch means the windows have diverged: drop the domain and
        // report it (as a typed error) instead of equalising against the wrong
        // de-ramp.
        let timing_offsets = match self.frame_timing_offsets(carriers.len()) {
            Ok(offsets) => offsets,
            Err(error) => {
                self.reset_domain(ResetReason::Sync);
                return Err(error);
            }
        };

        let pipeline = self.pipeline.as_mut().ok_or_else(|| {
            ReceiverError::Internal("pipeline missing after configuration".into())
        })?;

        // Collect packets first but never return early with them still in the
        // local buffer: a symbol that fails after earlier symbols already
        // produced real TS must not silently drop those packets. They are
        // appended to `pending_out` below even when `failure` is set, and a
        // later `push_iq` delivers them.
        //
        // The first frame symbol is differential to the last symbol of the
        // preceding frame; every later symbol is differential to its immediate
        // predecessor in `carriers`. `previous` is ignored by the coherent path.
        let mut packets = Vec::new();
        let mut failure = None;
        let mut previous = previous_carriers.as_deref();
        for (current, &timing_offset) in carriers.iter().zip(timing_offsets.iter()) {
            let result = pipeline.consume_symbol(current, previous, timing_offset);
            previous = Some(current);
            match result {
                Ok(emitted) => packets.extend(emitted),
                Err(error) => {
                    failure = Some(error);
                    break;
                }
            }
        }

        if !packets.is_empty() {
            self.last_ts_symbols = self.symbol_count;
            self.locked = true;
            self.stats.ts_packets = self
                .stats
                .ts_packets
                .saturating_add(u64::try_from(packets.len()).unwrap_or(u64::MAX));
            self.pending_out.append(&mut packets);
        }

        if let Some(error) = failure {
            self.reset_domain(ResetReason::Sync);
            return Err(error);
        }
        Ok(rebuilt)
    }

    /// The SFO timing offsets aligned with a validated frame's carriers.
    ///
    /// In production the frame collector's window and [`Self::timing_history`]
    /// advance in lock-step and are both capped at [`SYMBOLS_PER_FRAME`], so the
    /// histories always describe exactly the same symbols. Requiring an exact
    /// length match instead of falling back to zero offsets means a divergence
    /// is reported (and the domain is reset) rather than silently equalising a
    /// broken history against the wrong de-ramp in release builds. Tests that
    /// build a [`ReceivedFrame`] directly must supply the matching zero history
    /// themselves.
    fn frame_timing_offsets(&self, count: usize) -> Result<Vec<i64>, ReceiverError> {
        if self.timing_history.len() != count {
            return Err(ReceiverError::Internal(format!(
                "SFO timing history ({} entries) does not match frame carriers ({count})",
                self.timing_history.len(),
            )));
        }
        Ok(self.timing_history.iter().copied().collect())
    }

    /// Keep "locked" honest and rebuild a wedged FEC/time pipeline.
    ///
    /// Before the pipeline is established a false CP lock must still be torn
    /// down: a wrong-timing / wrong-Mode lock on noise never validates a TMCC
    /// frame, so it would otherwise sit in `pipeline == None` (and therefore
    /// `resets == 0`) forever. A genuine signal is never interrupted because the
    /// deadline is several frame periods, well past the two-frame confirmation.
    /// Once a pipeline exists the original per-frame and TS-stall checks run
    /// unchanged.
    fn enforce_stale(&mut self) {
        let since_frame = self.symbol_count.saturating_sub(self.last_frame_symbols);

        if self.pipeline.is_none() {
            if since_frame > PRE_TMCC_TIMEOUT_SYMBOLS {
                // The CP lock outlived several frame periods without ever
                // confirming a TMCC frame: it is false, so drop the domain
                // (front end kept) and re-acquire.
                self.reset_domain(ResetReason::TmccTimeout);
            }
            return;
        }

        let stall_limit = self
            .pipeline
            .as_ref()
            .map(|pipeline| pipeline.warmup_symbols() + TS_STALL_FRAMES * SYMBOLS_PER_FRAME as u64)
            .unwrap_or(0);

        if since_frame > STALE_LIMIT_SYMBOLS {
            // No valid TMCC frame for too long: drop everything so stale
            // time-interleaver / Viterbi state cannot leak forward.
            self.reset_domain(ResetReason::TmccTimeout);
            return;
        }

        let since_ts = self.symbol_count.saturating_sub(self.last_ts_symbols);
        if self.locked && since_ts > STALE_LIMIT_SYMBOLS {
            // Control plane is alive but no real TS has decoded recently, so the
            // decoder is not actually locked to a service.
            self.locked = false;
        }
        if since_ts > stall_limit {
            // TMCC still validates, so the front end and acquisition are healthy
            // and only the FEC/time pipeline is wedged (bad byte phase, stuck
            // Viterbi, time de-interleaver). Rebuild just that pipeline from the
            // next validated frame instead of stalling forever.
            self.reset_pipeline(ResetReason::TsStall);
        }
    }

    /// Handle a change of synchroniser lock generation (including an implicit
    /// Mode / guard change).
    ///
    /// The frame collector and the time/FEC pipeline are dropped so that
    /// symbols from two generations can never be concatenated into a frame, but
    /// the synchroniser — already locked to the new generation — and the front
    /// end are deliberately kept. The current batch continues against the fresh
    /// state.
    ///
    /// When the previous generation's state was already dropped (for example by
    /// [`Decoder::reset_domain`] after a lock loss) this only records the new
    /// generation and does not count a second reset.
    fn reset_for_epoch(&mut self, generation: u64) {
        let had_state = self.collector.is_some() || self.pipeline.is_some() || self.locked;
        self.collector = None;
        self.timing_history.clear();
        self.pipeline = None;
        self.locked = false;
        self.last_frame_symbols = self.symbol_count;
        self.last_ts_symbols = self.symbol_count;
        self.epoch = Some(generation);
        if had_state {
            self.record_reset(ResetReason::Sync);
        }
    }

    /// Reset domain acquisition / demodulator state, keeping the front end.
    fn reset_domain(&mut self, reason: ResetReason) {
        self.synchronizer.reset();
        self.collector = None;
        self.timing_history.clear();
        self.pipeline = None;
        self.symbol_count = 0;
        self.epoch = None;
        self.last_frame_symbols = 0;
        self.last_ts_symbols = 0;
        self.locked = false;
        self.record_reset(reason);
    }

    /// Drop the demod/FEC pipeline but keep the front end, synchroniser and TMCC
    /// collector, so the next validated frame reconfigures it without a full
    /// re-acquisition.
    fn reset_pipeline(&mut self, reason: ResetReason) {
        self.pipeline = None;
        self.last_ts_symbols = self.symbol_count;
        self.locked = false;
        self.record_reset(reason);
    }

    /// Reset everything, including the front end and the partial IQ byte.
    fn reset_internal(&mut self, reason: ResetReason) {
        self.dc.reset();
        self.resampler.reset();
        self.raw.clear();
        self.pending_out.clear();
        self.remainder = None;
        self.synchronizer.reset();
        self.collector = None;
        self.timing_history.clear();
        self.pipeline = None;
        self.symbol_count = 0;
        self.epoch = None;
        self.last_frame_symbols = 0;
        self.last_ts_symbols = 0;
        self.locked = false;
        self.record_reset(reason);
    }

    fn record_reset(&mut self, reason: ResetReason) {
        self.stats.resets = self.stats.resets.saturating_add(1);
        match reason {
            ResetReason::Gap => {
                self.stats.resets_gap = self.stats.resets_gap.saturating_add(1);
            }
            ResetReason::Sync => {
                self.stats.resets_sync = self.stats.resets_sync.saturating_add(1);
            }
            ResetReason::Config => {
                self.stats.resets_config = self.stats.resets_config.saturating_add(1);
            }
            ResetReason::TmccTimeout => {
                self.stats.resets_tmcc_timeout = self.stats.resets_tmcc_timeout.saturating_add(1);
            }
            ResetReason::TsStall => {
                self.stats.resets_ts_stall = self.stats.resets_ts_stall.saturating_add(1);
            }
            ResetReason::Caller => {}
        }
    }
}

impl std::fmt::Debug for Decoder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Decoder")
            .field("config", &self.config)
            .field("locked", &self.locked)
            .field("stats", &self.stats)
            .finish()
    }
}

/// Why the pipeline was reset (internal bookkeeping).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResetReason {
    /// Explicit [`Decoder::reset`] call.
    Caller,
    /// IQ gap reported by the caller.
    Gap,
    /// Loss of synchronisation or a Mode / guard change (a change of
    /// synchroniser lock generation).
    Sync,
    /// Layer-configuration change, or a DQPSK frame whose differential
    /// predecessor is unavailable.
    Config,
    /// No valid TMCC frame for too long.
    TmccTimeout,
    /// TMCC is valid but the FEC/time pipeline produced no TS past its
    /// warm-up plus stall allowance; only that pipeline was rebuilt.
    TsStall,
}

/// Normalise one `(I, Q)` byte pair exactly as [`normalize_u8_iq`] does.
///
/// [`normalize_u8_iq`]: crate::dsp::normalize_u8_iq
fn normalize_pair(i: u8, q: u8) -> Complex32 {
    Complex32::new(
        (i as f32 - crate::dsp::IQ_BIAS) * crate::dsp::IQ_SCALE,
        (q as f32 - crate::dsp::IQ_BIAS) * crate::dsp::IQ_SCALE,
    )
}

/// Greatest common divisor.
fn gcd_u64(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a
}

/// Reduced resampling ratio `(interp, decim)` for `sample_rate`.
///
/// The ratio maps the input rate onto the one-segment rate `64/63 MHz`
/// (`interp / decim * sample_rate == 64e6 / 63`). Returns `None` for a zero
/// rate.
pub fn resample_ratio(sample_rate: u32) -> Option<(usize, usize)> {
    if sample_rate == 0 {
        return None;
    }
    let num = SEGMENT_RATE_NUM as u64;
    let den = SEGMENT_RATE_DEN as u64 * sample_rate as u64;
    let divisor = gcd_u64(num, den);
    let interp = (num / divisor) as usize;
    let decim = (den / divisor) as usize;
    if interp == 0 || decim == 0 {
        return None;
    }
    Some((interp, decim))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Near-zero IQ: RTL-SDR silence centred on the `127.5` bias.
    fn near_zero_iq(len: usize) -> Vec<u8> {
        vec![128u8; len]
    }

    /// Deterministic pseudo-noise (no `rand` dependency).
    fn noise_iq(len: usize) -> Vec<u8> {
        let mut state = 0x1234_5678u32;
        (0..len)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (state >> 24) as u8
            })
            .collect()
    }

    #[test]
    fn default_rates_reduce_to_expected_ratios() {
        assert_eq!(DEFAULT_INPUT_RATE, 1_200_000);
        assert_eq!(MIN_INPUT_RATE, DEFAULT_INPUT_RATE);
        assert_eq!(resample_ratio(0), None);
        assert_eq!(resample_ratio(DEFAULT_INPUT_RATE), Some((160, 189)));
        assert_eq!(resample_ratio(2_048_000), Some((125, 252)));

        let decoder = Decoder::new(DEFAULT_INPUT_RATE).expect("default 1.2 MHz must be valid");
        let config = decoder.config();
        assert_eq!(config.input_rate, 1_200_000);
        assert_eq!((config.resample_interp, config.resample_decim), (160, 189));

        let decoder = Decoder::new(2_048_000).expect("2.048 MHz must be valid");
        let config = decoder.config();
        assert_eq!(config.input_rate, 2_048_000);
        assert_eq!((config.resample_interp, config.resample_decim), (125, 252));
    }

    #[test]
    fn malformed_rates_are_rejected() {
        assert!(matches!(
            Decoder::new(0),
            Err(ReceiverError::UnsupportedSampleRate { rate: 0, .. })
        ));
        assert!(matches!(
            Decoder::new(MIN_INPUT_RATE - 1),
            Err(ReceiverError::UnsupportedSampleRate { .. })
        ));
        assert!(matches!(
            Decoder::new(MAX_INPUT_RATE + 1),
            Err(ReceiverError::UnsupportedSampleRate { .. })
        ));
        // In range, but the reduced ratio exceeds the resampler factor limit.
        assert!(matches!(
            Decoder::new(1_999_999),
            Err(ReceiverError::Resample {
                input: 1_999_999,
                ..
            })
        ));
    }

    #[test]
    fn silence_and_noise_do_not_lock() {
        let sources: [fn(usize) -> Vec<u8>; 2] = [near_zero_iq, noise_iq];
        for source in sources {
            let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
            for _ in 0..3 {
                let packets = decoder.push_iq(&source(8192), false).unwrap();
                assert!(packets.is_empty(), "the decoder must not invent TS");
            }
            assert!(!decoder.is_locked());
            assert_eq!(decoder.stats().ts_packets, 0);
        }
    }

    #[test]
    fn odd_fragment_is_retained_and_paired() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();

        // 3 bytes = one complete sample plus a retained trailing byte.
        assert!(decoder.push_iq(&[1, 2, 3], false).unwrap().is_empty());
        assert_eq!(decoder.stats().input_samples, 1);
        assert_eq!(decoder.remainder, Some(3));

        // The retained byte pairs with the first byte of the next call.
        assert!(decoder.push_iq(&[4], false).unwrap().is_empty());
        assert_eq!(decoder.stats().input_samples, 2);
        assert_eq!(decoder.remainder, None);
    }

    #[test]
    fn fragmented_input_keeps_the_same_sample_count() {
        let bytes: Vec<u8> = (0..=255u8).cycle().take(1001).collect();

        let mut contiguous = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        contiguous.push_iq(&bytes, false).unwrap();

        let mut fragmented = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        // Odd chunk sizes force the retained remainder to cross calls.
        for chunk in bytes.chunks(3) {
            fragmented.push_iq(chunk, false).unwrap();
        }

        assert_eq!(contiguous.stats().input_samples, 500);
        assert_eq!(
            fragmented.stats().input_samples,
            contiguous.stats().input_samples
        );
        assert_eq!(fragmented.remainder, contiguous.remainder);
    }

    #[test]
    fn gap_resets_pipeline_and_counts() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        decoder.push_iq(&[1, 2, 3, 4, 5, 6, 7, 8], false).unwrap();
        assert_eq!(decoder.stats().input_samples, 4);
        assert_eq!(decoder.stats().gaps, 0);

        decoder.push_iq(&[9, 10, 11, 12], true).unwrap();

        let stats = decoder.stats();
        assert_eq!(stats.gaps, 1);
        assert_eq!(stats.resets, 1);
        assert_eq!(stats.resets_gap, 1);
        assert_eq!(stats.input_samples, 6);
        // The gap clears the retained partial byte and the domain pipeline.
        assert_eq!(decoder.remainder, None);
        assert!(decoder.collector.is_none());
        assert!(decoder.pipeline.is_none());
        assert!(!decoder.is_locked());
    }

    #[test]
    fn explicit_reset_clears_pipeline_state_and_counts() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        decoder.push_iq(&[1, 2, 3], false).unwrap();
        assert!(decoder.remainder.is_some());

        decoder.reset();

        assert!(decoder.remainder.is_none());
        assert!(decoder.collector.is_none());
        assert!(decoder.pipeline.is_none());
        assert!(decoder.pending_out.is_empty());
        assert!(!decoder.is_locked());
        assert_eq!(decoder.stats().resets, 1);
    }

    #[test]
    fn buffered_ts_is_not_cleared_by_push_iq() {
        // Packets decoded before a non-fatal error are delivered by the next
        // `push_iq` rather than being silently dropped.
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let packet = [0x47u8; 188];
        decoder.pending_out.push(packet);

        let returned = decoder.push_iq(&[], false).unwrap();

        assert_eq!(returned, vec![packet]);
    }

    // ---- diagnostics accessor ----------------------------------------------

    #[test]
    fn diagnostics_report_absence_while_unlocked() {
        // Unlocked: the current-lock snapshot must not invent a mode / guard /
        // CFO, even though the aggregate counters stay available.
        let decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let diagnostics = decoder.diagnostics();

        assert!(!diagnostics.cp_locked);
        assert_eq!(diagnostics.mode, None);
        assert_eq!(diagnostics.guard, None);
        assert_eq!(diagnostics.generation, 0);
        assert_eq!(diagnostics.losses, 0);
        assert_eq!(diagnostics.integer_bins, None);
        assert_eq!(diagnostics.cfo, None);
        assert_eq!(diagnostics.timing_offset, None);
    }

    // ---- synchroniser-generation (epoch) handling --------------------------

    use crate::demod::control::carriers_per_segment;
    use crate::dsp::GuardInterval;
    use crate::fec::PunctureRate;

    fn test_config() -> LayerConfig {
        LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 0,
        }
    }

    fn differential_config() -> LayerConfig {
        LayerConfig {
            modulation: Modulation::Dqpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 0,
        }
    }

    fn test_symbol(mode: Mode, generation: u64) -> SyncedSymbol {
        SyncedSymbol {
            mode,
            guard: GuardInterval::OneEighth,
            carriers: vec![Complex32::new(0.0, 0.0); carriers_per_segment(mode)],
            timing_offset: 0,
            generation,
        }
    }

    #[test]
    fn generation_change_resets_acquisition_state_and_continues_batch() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        decoder.collector = Some(FrameCollector::new(mode));
        decoder.pipeline = Some(Pipeline::new(mode, test_config()).unwrap());
        decoder.locked = true;
        decoder.epoch = Some(1);
        decoder.symbol_count = 100;
        decoder.last_ts_symbols = 100;

        let symbols = [
            test_symbol(mode, 2),
            test_symbol(mode, 2),
            test_symbol(mode, 2),
        ];
        // The synchroniser is not actually locked in this unit test, so the
        // pre-batch lock flag is false.
        decoder.consume_symbols(&symbols, false).unwrap();

        // The new generation dropped the old collector/pipeline, but the rest
        // of the batch was still consumed (the previous implementation returned
        // on the first symbol) and no data was fabricated.
        assert_eq!(decoder.epoch, Some(2));
        assert!(decoder.collector.is_some());
        assert!(decoder.pipeline.is_none());
        assert!(!decoder.is_locked());
        assert_eq!(decoder.stats().resets, 1);
        assert_eq!(decoder.stats().resets_sync, 1);
        assert_eq!(decoder.symbol_count, 103);
        assert_eq!(decoder.stats().tmcc_frames, 0);
    }

    #[test]
    fn mode_change_within_a_generation_resets_and_continues() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        decoder.collector = Some(FrameCollector::new(Mode::Mode1));
        decoder.pipeline = Some(Pipeline::new(Mode::Mode1, test_config()).unwrap());
        decoder.locked = true;
        decoder.epoch = Some(5);

        let symbols = [test_symbol(Mode::Mode2, 5), test_symbol(Mode::Mode2, 5)];
        decoder.consume_symbols(&symbols, false).unwrap();

        assert_eq!(decoder.epoch, Some(5));
        assert!(decoder.pipeline.is_none());
        assert_eq!(
            decoder.collector.as_ref().map(FrameCollector::mode),
            Some(Mode::Mode2)
        );
        assert_eq!(decoder.stats().resets_sync, 1);
    }

    #[test]
    fn first_generation_after_a_clean_reset_is_not_counted() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        // No collector, pipeline or lock: the first symbol of the first
        // generation must not be reported as a reset.
        decoder
            .consume_symbols(&[test_symbol(Mode::Mode1, 1)], false)
            .unwrap();

        assert_eq!(decoder.epoch, Some(1));
        assert!(decoder.collector.is_some());
        assert_eq!(decoder.stats().resets, 0);
    }

    // ---- differential (DQPSK) pipeline ------------------------------------

    #[test]
    fn dqpsk_pipeline_requires_a_differential_reference() {
        let mode = Mode::Mode1;
        let total = carriers_per_segment(mode);
        let mut pipeline = Pipeline::new(mode, differential_config()).unwrap();
        let carriers = vec![Complex32::new(1.0, 0.0); total];

        assert!(matches!(
            pipeline.consume_symbol(&carriers, None, 0),
            Err(ReceiverError::Internal(_))
        ));

        let previous = vec![Complex32::new(1.0, 0.0); total];
        assert!(pipeline
            .consume_symbol(&carriers, Some(&previous), 0)
            .is_ok());
    }

    #[test]
    fn dqpsk_frame_without_previous_reference_is_skipped() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        let total = carriers_per_segment(mode);
        // A directly-constructed frame has no live synchroniser history; supply
        // the identity (zero) offsets the production path would have carried.
        decoder.timing_history = vec![0i64; SYMBOLS_PER_FRAME].into();
        let frame = ReceivedFrame {
            mode,
            config: differential_config(),
            carriers: vec![vec![Complex32::new(1.0, 0.0); total]; SYMBOLS_PER_FRAME],
            previous_carriers: None,
        };

        let rebuilt = decoder.consume_frame_into(frame).unwrap();

        assert!(!rebuilt, "skipping a frame must not stop the batch");
        assert!(
            decoder.pipeline.is_none(),
            "the stale pipeline must be dropped"
        );
        assert!(decoder.pending_out.is_empty(), "no TS may be fabricated");
        assert_eq!(decoder.stats().ts_packets, 0);
    }

    #[test]
    fn dqpsk_frame_with_previous_reference_is_demodulated() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        let total = carriers_per_segment(mode);
        decoder.timing_history = vec![0i64; SYMBOLS_PER_FRAME].into();
        let frame = ReceivedFrame {
            mode,
            config: differential_config(),
            carriers: vec![vec![Complex32::new(1.0, 0.0); total]; SYMBOLS_PER_FRAME],
            previous_carriers: Some(vec![Complex32::new(1.0, 0.0); total]),
        };

        let rebuilt = decoder.consume_frame_into(frame).unwrap();

        assert!(!rebuilt);
        assert!(
            decoder.pipeline.is_some(),
            "a complete DQPSK frame must be consumed"
        );
        assert_eq!(decoder.stats().resets, 0);
    }

    // ---- SFO timing-history window ----------------------------------------

    #[test]
    fn timing_history_is_capped_and_cleared_by_domain_resets() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;

        // More than one frame of symbols: the window must stay at one frame and
        // hold the newest offsets.
        let symbols: Vec<SyncedSymbol> = (0..SYMBOLS_PER_FRAME + 5)
            .map(|index| SyncedSymbol {
                timing_offset: index as i64,
                ..test_symbol(mode, 1)
            })
            .collect();
        decoder.consume_symbols(&symbols, false).unwrap();
        assert_eq!(
            decoder.timing_history.len(),
            SYMBOLS_PER_FRAME,
            "the SFO window must be capped to one frame"
        );
        assert_eq!(decoder.timing_history.front().copied(), Some(5));
        assert_eq!(
            decoder.timing_history.back().copied(),
            Some(SYMBOLS_PER_FRAME as i64 + 4)
        );

        // A generation (epoch) change clears the stale window before the new
        // symbol is pushed.
        decoder
            .consume_symbols(&[test_symbol(mode, 2)], false)
            .unwrap();
        assert_eq!(
            decoder.timing_history.len(),
            1,
            "a generation change must clear the old SFO window"
        );

        // A gap and an explicit reset both drop the window entirely.
        decoder.push_iq(&[], true).unwrap();
        assert!(
            decoder.timing_history.is_empty(),
            "a gap must clear the SFO window"
        );

        decoder.timing_history.push_back(7);
        decoder.reset();
        assert!(
            decoder.timing_history.is_empty(),
            "an explicit reset must clear the SFO window"
        );
    }

    #[test]
    fn timing_history_mismatch_is_a_typed_error_without_ts() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        let total = carriers_per_segment(mode);

        // A deliberately inconsistent window (1 entry for a 2-symbol frame) must
        // be reported rather than silently zero-filled into a bogus
        // equalisation.
        decoder.timing_history.push_back(0);
        let frame = ReceivedFrame {
            mode,
            config: test_config(),
            carriers: vec![vec![Complex32::new(1.0, 0.0); total]; 2],
            previous_carriers: None,
        };

        let error = decoder.consume_frame_into(frame).unwrap_err();
        assert!(
            matches!(error, ReceiverError::Internal(_)),
            "a timing-history mismatch must be a typed Internal error, got {error:?}"
        );
        assert!(
            decoder.pending_out.is_empty(),
            "a mismatch must not emit TS"
        );
        assert_eq!(decoder.stats().ts_packets, 0);
        assert!(
            decoder.pipeline.is_none(),
            "the inconsistent domain must be dropped"
        );
        assert_eq!(decoder.stats().resets_sync, 1);
    }

    // ---- stale / stall recovery -------------------------------------------

    #[test]
    fn ts_stall_rebuilds_only_the_pipeline() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        // A long time interleave gives a large FEC warm-up to exceed.
        let config = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 16,
        };
        decoder.collector = Some(FrameCollector::new(mode));
        decoder.pipeline = Some(Pipeline::new(mode, config).unwrap());
        decoder.locked = true;
        decoder.symbol_count = 100_000;
        decoder.last_frame_symbols = 100_000;
        decoder.last_ts_symbols = 0; // no TS has ever been produced

        decoder.enforce_stale();

        let stats = decoder.stats();
        assert_eq!(stats.resets, 1);
        assert_eq!(stats.resets_ts_stall, 1);
        assert!(
            decoder.pipeline.is_none(),
            "the wedged pipeline must be rebuilt"
        );
        assert!(
            !decoder.is_locked(),
            "a TS-less pipeline must not stay locked"
        );
        assert!(
            decoder.collector.is_some(),
            "only the FEC/time pipeline is rebuilt; TMCC acquisition is kept"
        );
    }

    #[test]
    fn ts_stall_rebuild_resets_the_stall_clock() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        decoder.pipeline = Some(Pipeline::new(mode, test_config()).unwrap());
        decoder.locked = true;
        decoder.symbol_count = 100_000;
        decoder.last_frame_symbols = 100_000;
        decoder.last_ts_symbols = 0;

        decoder.enforce_stale();
        assert_eq!(decoder.stats().resets_ts_stall, 1);

        // A freshly rebuilt pipeline starts a new stall clock, so enforcing
        // again at the same symbol count must not loop into another reset.
        decoder.pipeline = Some(Pipeline::new(mode, test_config()).unwrap());
        decoder.enforce_stale();
        assert_eq!(decoder.stats().resets_ts_stall, 1);
    }

    #[test]
    fn pre_tmcc_deadline_does_not_interrupt_two_frame_acquisition() {
        // A genuine signal needs two frame periods to fill the TMCC window and
        // confirm the alternating sync words; the pre-TMCC deadline must leave
        // that acquisition alone.
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        decoder.collector = Some(FrameCollector::new(Mode::Mode1));
        decoder.symbol_count = 2 * SYMBOLS_PER_FRAME as u64;
        decoder.last_frame_symbols = 0;

        decoder.enforce_stale();

        assert_eq!(decoder.stats().resets, 0);
        assert!(decoder.collector.is_some());
        assert!(decoder.pipeline.is_none());
    }

    #[test]
    fn false_cp_lock_without_tmcc_is_bounded() {
        // A fresh CP lock that never validates a TMCC frame must not live
        // forever with `resets == 0` (the hardware false-lock symptom).
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        decoder.collector = Some(FrameCollector::new(Mode::Mode1));
        decoder.symbol_count = PRE_TMCC_TIMEOUT_SYMBOLS + 1;
        decoder.last_frame_symbols = 0;

        decoder.enforce_stale();

        let stats = decoder.stats();
        assert_eq!(stats.resets, 1);
        assert_eq!(stats.resets_tmcc_timeout, 1);
        assert_eq!(
            stats.resets_ts_stall, 0,
            "a pre-TMCC reset must not be counted as a TS stall"
        );
        assert!(
            decoder.collector.is_none(),
            "the domain must be re-acquired"
        );
        assert!(decoder.pipeline.is_none());
        assert!(!decoder.is_locked());
        // The reset restarts the domain counters, so the false lock is bounded:
        // enforcing at the same symbol count again must not fire.
        assert_eq!(decoder.symbol_count, 0);
        decoder.enforce_stale();
        assert_eq!(decoder.stats().resets, 1);
    }

    #[test]
    fn tmcc_timeout_drops_domain_state() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        decoder.collector = Some(FrameCollector::new(mode));
        decoder.pipeline = Some(Pipeline::new(mode, test_config()).unwrap());
        decoder.locked = true;
        decoder.symbol_count = 10_000;
        decoder.last_frame_symbols = 0; // no valid TMCC frame for ages

        decoder.enforce_stale();

        let stats = decoder.stats();
        assert_eq!(stats.resets, 1);
        assert_eq!(stats.resets_tmcc_timeout, 1);
        assert!(decoder.collector.is_none());
        assert!(decoder.pipeline.is_none());
        assert!(!decoder.is_locked());
    }

    // ---- batch primitive failure ------------------------------------------

    #[test]
    fn frame_error_keeps_already_decoded_packets() {
        let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).unwrap();
        let mode = Mode::Mode1;
        let config = test_config();
        decoder.pipeline = Some(Pipeline::new(mode, config).unwrap());
        decoder.epoch = Some(1);

        // A real packet decoded before the failure must survive the reset.
        let packet = [0x47u8; 188];
        decoder.pending_out.push(packet);

        // Wrong carrier count makes the equaliser fail mid-frame.
        let total = carriers_per_segment(mode);
        // One carrier vector in the frame, so exactly one SFO offset is needed.
        decoder.timing_history = vec![0i64; 1].into();
        let frame = ReceivedFrame {
            mode,
            config,
            carriers: vec![vec![Complex32::new(1.0, 0.0); total - 1]],
            previous_carriers: None,
        };

        let error = decoder.consume_frame_into(frame).unwrap_err();
        assert!(matches!(error, ReceiverError::Carrier(_)));

        assert_eq!(decoder.pending_out, vec![packet]);
        assert!(
            decoder.pipeline.is_none(),
            "a primitive failure resets the pipeline"
        );
        assert_eq!(decoder.stats().resets_sync, 1);

        // The buffered packet is delivered by the next call, in order.
        assert_eq!(decoder.push_iq(&[], false).unwrap(), vec![packet]);
    }
}
