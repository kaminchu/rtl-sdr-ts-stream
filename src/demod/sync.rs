//! Streaming OFDM synchroniser for the ISDB-T centre (partial-reception)
//! segment.
//!
//! The normative basis is ARIB STD-B31 version 2.2-E1:
//!
//! * §3.14.1 / Table 3-1 — the three transmission modes (effective symbol
//!   lengths `252 / 504 / 1008 us`, i.e. `256 / 512 / 1024` samples at the
//!   one-segment rate `fs = 64/63 MHz`) and the four guard-interval ratios
//!   `1/4, 1/8, 1/16, 1/32`.
//! * §3.14.2 / Fig. 3-33 — the guard interval is an unmodified copy of the
//!   latter part of the IFFT output, prepended to the effective symbol.
//!
//! The guard-interval copy is exactly what the classic normalised cyclic-prefix
//! (CP) correlation exploits: at the start of a symbol the first `guard_len`
//! samples are identical to the samples `fft_size` later, so
//!
//! ```text
//! corr(d) = sum_{k=0}^{G-1} conj(x[d+k]) * x[d+N+k]
//! ```
//!
//! has magnitude `sum |x|^2` at the true guard start `d` and (for a valid
//! symbol) drops off to zero within `G` samples. The normalised score
//! `|corr| / sqrt(E_prefix * E_tail)` is `~1` for real CP structure and
//! `~1/sqrt(G)` for complex Gaussian noise, which is what keeps the detector
//! from reporting false locks.
//!
//! # What this module does
//!
//! [`Synchronizer`] consumes a stream of complex baseband samples at
//! `fs = 64/63 MHz`, estimates the transmission mode and guard interval, and
//! emits one [`SyncedSymbol`] (mode, guard and the `108 / 216 / 432` corrected
//! centre-segment carriers) for every complete OFDM symbol. It reuses
//! [`extract_symbol`](super::ofdm::extract_symbol) for guard removal, CFO de-rotation and the unitary DFT;
//! everything else (acquisition, timing tracking, CFO estimation) lives here.
//!
//! * **Acquisition** evaluates all twelve `(Mode, GuardInterval)` candidates
//!   with a *multi-symbol* normalised CP score. A candidate must produce a high
//!   score for every one of [`ACQUIRE_SYMBOLS`] consecutive symbol periods, which
//!   is what rejects the smaller-guard alias (a smaller guard correlates on the
//!   first symbol but not on the repeats, because its symbol period is wrong).
//! * **Tracking** searches a small window (`±`[`SEARCH_RADIUS`] samples) around
//!   the predicted next symbol start and re-estimates the CFO each symbol. A
//!   metric that fails the power/normalisation gate resets the receiver to the
//!   un-locked state, so a gap or a noise-only segment cannot keep a stale lock.
//! * **CFO phase continuity.** [`extract_symbol`](super::ofdm::extract_symbol) de-rotates by the *local*
//!   useful-part index, which leaves a constant per-symbol phase
//!   `exp(j * cfo * (start + guard))`. That phase is removed with a *continuous*
//!   numerically-controlled-oscillator phase: the phase is anchored once when
//!   the lock is taken and then advanced by `cfo * symbol_len` for every emitted
//!   symbol. It is deliberately **not** recomputed as `cfo * elapsed_samples`
//!   from the freshly smoothed estimate, because a tiny change in that estimate
//!   would then be multiplied by the (ever growing) absolute sample index and
//!   jump the common phase by a large, essentially random amount between
//!   symbols. The accumulated oscillator keeps the residual common phase
//!   continuous, which the differential TMCC detector and the coherent
//!   equaliser both rely on.
//! * **Integer CFO acquisition (coherent segments).** The cyclic-prefix phase
//!   is periodic in the subcarrier spacing, so acquisition only sees the
//!   *fractional* part of the CFO. While the coarse stage is unlocked the
//!   synchroniser additionally runs
//!   [`coarse::estimate_integer_cfo`](super::coarse::estimate_integer_cfo) on the
//!   full DFT spectrum of each symbol (the same transform
//!   [`full_spectrum`] already produces) and requires
//!   [`COARSE_CONFIRM_SYMBOLS`] *consecutive, identical* integer estimates
//!   before acting. Once confirmed it adds the integer offset
//!   `2*pi*bins/fft_size` to the CFO used by [`extract_symbol`](super::ofdm::extract_symbol) and re-anchors
//!   the NCO to the total CFO, so both the spectral bin shift **and** the
//!   symbol-common phase `exp(j*2*pi*bins*(start+guard)/fft_size)` are removed.
//!   Because the first confirmed symbols may already have been emitted with the
//!   wrong integer alignment, applying a *non-zero* correction increments the
//!   lock [`generation`](Synchronizer::generation) so a downstream stage drops
//!   its continuity-sensitive state. A confirmation of `bins == 0` changes
//!   nothing and needs no new generation. A differential (DQPSK) segment has no
//!   scattered pilots, so the estimator never confirms and the fractional-CFO
//!   behaviour is retained unchanged: **integer CFO for DQPSK is not supported
//!   here**; a caller that knows the receiver's ppm error should use it as a
//!   fallback.
//!
//! # Re-acquisition observability
//!
//! Every symbol carries the [`SyncedSymbol::generation`] of the lock it was
//! emitted under, and [`Synchronizer::loss_count`] counts internal lock losses.
//! Together they let a downstream stage notice that the synchroniser silently
//! lost and re-acquired a lock (for example mid-`push`) without having to
//! inspect private state.
//!
//! # Bounds and untrusted input
//!
//! The internal [`VecDeque`] is hard-capped at [`WINDOW_CAP`] samples and the
//! acquisition window is smaller still, so a caller can never force unbounded
//! memory growth. Public entry points take plain slices and never panic on
//! malformed or empty input.

use std::collections::VecDeque;

use num_complex::Complex32;

use super::coarse::{estimate_integer_cfo, MAX_SEARCH_BINS};
use super::ofdm::{centre_carriers, full_spectrum, segment_fft_size, segment_symbol_len};
use crate::dsp::{GuardInterval, Mode};

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/// Hard upper bound on the retained sample window, in samples.
///
/// The longest candidate symbol is Mode 3 with a 1/4 guard (`1280` samples), so
/// this holds more than twenty-five maximum-length symbols.
const WINDOW_CAP: usize = 32768;

/// Samples are appended to the internal window in steps of at most this many
/// samples, so acquisition work and memory stay bounded regardless of the
/// caller's chunk size.
const PUSH_STEP: usize = 4096;

/// Number of consecutive symbol periods that an acquisition candidate must
/// agree on.
const ACQUIRE_SYMBOLS: usize = 4;

/// Longest candidate symbol: Mode 3 with a 1/4 guard (`1024 + 256`).
const MAX_SYMBOL_LEN: usize = 1280;

/// Samples that must be buffered before acquisition is attempted.
///
/// Requiring the *longest* candidate's [`ACQUIRE_SYMBOLS`] span means every
/// candidate is judged over a comparable observation time. A short-guard alias
/// then has to sustain its accidental correlation for many more periods than
/// the true candidate, which is what rejects it.
const ACQ_MIN_SAMPLES: usize = ACQUIRE_SYMBOLS * MAX_SYMBOL_LEN;

/// Acquisition window. Must be at least [`ACQ_MIN_SAMPLES`].
const ACQ_WINDOW: usize = 6144;

/// Upper bound on the number of symbol periods used per acquisition candidate.
/// More periods give a stronger small-guard-alias rejection at modest cost.
const MAX_ACQ_PERIODS: usize = 16;

/// Number of samples searched on either side of the predicted symbol start
/// while tracking.
const SEARCH_RADIUS: usize = 4;

/// Minimum *mean* normalised CP score over [`ACQUIRE_SYMBOLS`] periods.
const ACQ_MEAN_THRESHOLD: f32 = 0.72;

/// Minimum normalised CP score for *each* acquisition period.
const ACQ_MIN_SYMBOL: f32 = 0.60;

/// Minimum coherence of the per-period CFO phase over the acquisition periods.
///
/// A genuine candidate sees the same `arg(conj(prefix) * tail) = cfo * fft_size`
/// on every period, so the unit phasors add constructively (coherence `~1`).
/// Accidental correlations from the wrong symbol period (or short-guard aliases
/// on structured signals) have unrelated phases and add incoherently. This is
/// the "repeated symbol-length coherence" test.
const ACQ_CFO_COHERENCE: f32 = 0.85;

/// Score hysteresis when choosing the acquisition offset. Scores within this
/// margin are treated as ties so the earliest offset wins, keeping chunked and
/// contiguous processing identical.
const ACQ_SCORE_EPS: f32 = 1e-3;

/// Minimum normalised CP score for a tracked symbol.
const TRACK_MIN: f32 = 0.60;

/// Minimum mean tail power (mean squared magnitude) accepted as signal rather
/// than silence.
const MIN_POWER: f32 = 1e-6;

/// Leaky-integrator weight applied to each new CFO measurement.
const CFO_SMOOTHING: f32 = 0.25;

/// Number of consecutive, identical integer-CFO estimates required before the
/// coarse stage applies a correction.
///
/// The single-symbol scattered-pilot estimator is already high-confidence, but
/// requiring three consistent symbols prevents a noisy one-off estimate from
/// shifting the whole carrier readout (and bumping the lock generation). A run
/// of `0` confirmations (no integer offset) clears the stage just as well.
const COARSE_CONFIRM_SYMBOLS: u8 = 3;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One synchronised OFDM symbol.
///
/// `carriers` holds the `108 / 216 / 432` centre-segment carriers for `mode` in
/// ascending segment-local order. The guard interval has been removed, the CFO
/// has been de-rotated with a continuous common phase reference (see the module
/// docs) and the unitary DFT has been applied, exactly as [`extract_symbol`](super::ofdm::extract_symbol)
/// documents.
#[derive(Debug, Clone, PartialEq)]
pub struct SyncedSymbol {
    /// Transmission mode detected for this symbol.
    pub mode: Mode,
    /// Guard interval detected for this symbol.
    pub guard: GuardInterval,
    /// Corrected centre-segment carriers.
    ///
    /// These carry the carrier-dependent timing de-ramp described in
    /// [`compensate_timing_offset`]: a cumulative sample-clock-offset (SFO)
    /// track is removed with `exp(-j 2*pi * signed_bin * timing_offset / N)`.
    /// That de-ramp is what keeps the differential TMCC and DQPSK products
    /// carrier-phase-coherent across the integer sample jumps of the timing
    /// tracker. Because it is cumulative it is also a linear phase ramp across
    /// the carriers, so a coherent scattered-pilot estimator must undo it first
    /// (see [`restore_timing_offset`]) with [`SyncedSymbol::timing_offset`].
    pub carriers: Vec<Complex32>,
    /// Cumulative timing offset, in samples, applied as the carrier-dependent
    /// de-ramp to [`SyncedSymbol::carriers`].
    ///
    /// This is the sample-frequency-offset (SFO) trend. It is the exact value
    /// needed to undo the de-ramp, so downstream stages that must see the
    /// physical (pre-de-ramp) carriers — in particular the coherent
    /// scattered-pilot channel estimator, whose linear interpolation fails once
    /// the 12-carrier pilot phase step approaches `pi` — can call
    /// [`restore_timing_offset`].
    pub timing_offset: i64,
    /// Lock generation this symbol belongs to.
    ///
    /// The synchroniser increments its generation every time it establishes
    /// (or re-establishes) a lock. Consecutive symbols therefore share a
    /// generation unless the lock was lost between them, even when both are
    /// returned by a single [`Synchronizer::push`] call. A downstream stage can
    /// compare this against the previous symbol to detect an internal
    /// re-acquisition and drop continuity-sensitive state.
    pub generation: u64,
}

impl SyncedSymbol {
    /// Effective symbol (FFT) length in samples.
    pub fn fft_size(&self) -> usize {
        segment_fft_size(self.mode)
    }

    /// Total `guard + useful` symbol length in samples.
    pub fn symbol_len(&self) -> usize {
        segment_symbol_len(self.mode, self.guard)
    }
}

/// Locked parameters and predicted position of the next symbol.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LockState {
    mode: Mode,
    guard: GuardInterval,
    /// Absolute sample index of the predicted next guard start.
    next_start: u64,
    /// Smoothed carrier-frequency offset in radians per sample.
    cfo: f32,
    /// Continuous NCO phase in radians to remove at the next symbol's useful
    /// start.
    ///
    /// Anchored once when the lock is taken and then advanced by
    /// `cfo * (symbol_len + actual timing delta)` per emitted symbol. Kept in
    /// `f64` because the stream can run for many symbols; the phase is only
    /// narrowed to `f32` when it is applied to the carriers.
    nco_phase: f64,
    /// Cumulative timing offset, in samples, of the chosen symbol start
    /// relative to the fixed symbol grid that was anchored at acquisition.
    ///
    /// The tracker searches `±`[`SEARCH_RADIUS`] around the predicted start and
    /// may pick a sample whose position differs from the grid by
    /// `offset - predicted`. Because the next prediction is derived from the
    /// *actual* chosen start, that per-symbol difference accumulates here. It is
    /// applied as a carrier-dependent phase
    /// `exp(-j 2*pi * signed_bin * timing_offset / fft_size)` so the DFT time
    /// shift relative to the acquisition grid is removed. A single fixed grid
    /// origin matters: compensating only the per-symbol jump (or letting the
    /// grid follow the actual start) leaves a timing error that changes between
    /// symbols, which rotates the differential TMCC products by a
    /// carrier-dependent phase and destroys frame validation.
    timing_offset: i64,
    /// Integer-subcarrier part of the CFO already folded into `cfo`.
    integer_bins: i32,
    /// Whether the coarse integer-CFO stage has reached the confirmation count
    /// and stopped searching.
    coarse_locked: bool,
    /// Integer offset currently being confirmed, and how many consecutive
    /// symbols have produced it.
    coarse_candidate: i32,
    coarse_hits: u8,
    /// Symbols emitted under this lock so far. Used to decide whether applying
    /// an integer correction must bump the generation (only if the synchroniser
    /// has already handed downstream stages a symbol with the wrong alignment).
    emitted_symbols: u64,
}

/// Result of a successful acquisition search.
#[derive(Debug, Clone, Copy)]
struct Acquired {
    mode: Mode,
    guard: GuardInterval,
    /// Offset (in samples) of the first detected symbol inside the window.
    offset: usize,
    /// Initial fractional CFO estimate in radians per sample.
    cfo: f32,
}

/// Streaming Mode / guard / timing / CFO synchroniser.
///
/// Feed it complex samples at `fs = 64/63 MHz` with [`push`](Self::push) and it
/// returns the symbols that became complete during that call.
pub struct Synchronizer {
    buf: VecDeque<Complex32>,
    /// Absolute index of `buf[0]` since the last [`reset`](Self::reset).
    base_index: u64,
    lock: Option<LockState>,
    /// Continuity generation, incremented on every successful acquisition.
    generation: u64,
    /// Number of internal lock losses since construction (never reset).
    loss_count: u64,
}

impl Synchronizer {
    /// Create an un-locked synchroniser with an empty window.
    pub fn new() -> Self {
        Self {
            buf: VecDeque::new(),
            base_index: 0,
            lock: None,
            generation: 0,
            loss_count: 0,
        }
    }

    /// Whether the synchroniser currently holds a timing/CFO lock.
    pub fn is_locked(&self) -> bool {
        self.lock.is_some()
    }

    /// Current continuity generation.
    ///
    /// Starts at `0`, increments by one for every successful acquisition
    /// (including every re-acquisition after an internal lock loss) and never
    /// decreases. [`SyncedSymbol::generation`] records the value in force when
    /// each symbol was emitted.
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Number of times a held lock was dropped internally (loss of CP
    /// structure, a symbol that fell out of the window, or an extraction /
    /// non-finite-CFO failure).
    ///
    /// This counts only losses the synchroniser detected by itself; it is not
    /// incremented by [`reset`](Self::reset), which is caller-initiated. It
    /// never decreases and is not cleared by `reset`, so a caller can poll it to
    /// notice that a lock was lost between two `push` calls.
    pub fn loss_count(&self) -> u64 {
        self.loss_count
    }

    /// The confirmed integer-subcarrier CFO of the current lock, in subcarriers.
    ///
    /// `None` while the synchroniser is unlocked or while the coarse stage is
    /// still collecting [`COARSE_CONFIRM_SYMBOLS`] consecutive estimates. For a
    /// coherent segment this becomes `Some(bins)` (including `Some(0)`) once the
    /// stage locks; for a differential (DQPSK) segment, which carries no
    /// scattered pilots, it stays `None` and the synchroniser keeps its
    /// fractional-only CFO.
    pub fn integer_bins(&self) -> Option<i32> {
        self.lock
            .filter(|lock| lock.coarse_locked)
            .map(|lock| lock.integer_bins)
    }

    /// The detected mode and guard interval of the current lock.
    ///
    /// `None` while unlocked. Both values describe the same lock, so they are
    /// returned together: a caller can never pair a mode and a guard read from
    /// two different locks. Used for diagnostics only.
    pub fn locked_mode_guard(&self) -> Option<(Mode, GuardInterval)> {
        self.lock.map(|lock| (lock.mode, lock.guard))
    }

    /// The current smoothed CFO estimate in radians per sample, or `None` while
    /// unlocked.
    ///
    /// When [`integer_bins`](Self::integer_bins) is `Some` this includes the
    /// confirmed integer-subcarrier correction; for a differential (DQPSK)
    /// segment it is the fractional-only estimate. Used for diagnostics only.
    pub fn cfo_estimate(&self) -> Option<f32> {
        self.lock.map(|lock| lock.cfo)
    }

    /// Cumulative timing offset, in samples, of the chosen symbol start relative
    /// to the fixed acquisition grid, or `None` while unlocked.
    ///
    /// This is the sample-frequency-offset (SFO) trend: a steadily growing
    /// magnitude means the sample clock is drifting relative to the transmitter
    /// and the tracker is compensating. Used for diagnostics only.
    pub fn timing_offset(&self) -> Option<i64> {
        self.lock.map(|lock| lock.timing_offset)
    }

    /// Drop all buffered samples and return to the un-locked state.
    ///
    /// The continuity [`generation`](Self::generation) and
    /// [`loss_count`](Self::loss_count) are *not* cleared; they describe the
    /// synchroniser's whole lifetime.
    pub fn reset(&mut self) {
        self.buf.clear();
        self.base_index = 0;
        self.lock = None;
    }

    /// Append samples and return every OFDM symbol completed by this call.
    ///
    /// The input may be split at arbitrary boundaries: all state (window,
    /// absolute sample index, lock and CFO) is carried across calls, so
    /// splitting a contiguous stream changes neither the emitted symbols nor
    /// their order.
    pub fn push(&mut self, samples: &[Complex32]) -> Vec<SyncedSymbol> {
        let mut out = Vec::new();
        for chunk in samples.chunks(PUSH_STEP) {
            self.buf.extend(chunk.iter().copied());
            // Process before trimming so a symbol that starts at the front of
            // the appended span is not dropped before acquisition can align to
            // it. The append step bounds the transient buffer size.
            self.process(&mut out);
            self.trim();
        }
        out
    }

    /// Run acquisition if un-locked, then emit every symbol the window allows.
    fn process(&mut self, out: &mut Vec<SyncedSymbol>) {
        if self.lock.is_none() {
            self.try_acquire();
        }
        if self.lock.is_some() {
            self.track(out);
        }
    }

    /// Acquire on the best `(mode, guard, offset)` candidate, if any.
    fn try_acquire(&mut self) {
        let base = self.base_index;
        let acquired = {
            let data = self.buf.make_contiguous();
            acquire_best(data)
        };
        if let Some(found) = acquired {
            let (_, guard_len, _) = fft_guard_len(found.mode, found.guard);
            // Anchor the continuous oscillator at the first symbol's useful
            // start. Any constant offset is harmless (a constant phase across
            // the stream); what matters is that it is never recomputed from the
            // absolute index afterwards.
            let nco_phase = found.cfo as f64 * (found.offset as f64 + guard_len as f64);
            self.generation = self.generation.saturating_add(1);
            self.lock = Some(LockState {
                mode: found.mode,
                guard: found.guard,
                next_start: base + found.offset as u64,
                cfo: found.cfo,
                nco_phase,
                timing_offset: 0,
                integer_bins: 0,
                coarse_locked: false,
                coarse_candidate: 0,
                coarse_hits: 0,
                emitted_symbols: 0,
            });
        }
    }

    /// Track the locked symbol and emit every complete symbol available.
    ///
    /// The buffer borrow is scoped so the lock can be written back afterwards.
    fn track(&mut self, out: &mut Vec<SyncedSymbol>) {
        let lock = match self.lock {
            Some(lock) => lock,
            None => return,
        };
        let (fft_size, guard_len, symbol_len) = fft_guard_len(lock.mode, lock.guard);
        let base = self.base_index;
        let mut cfo = lock.cfo;
        let mut nco_phase = lock.nco_phase;
        let mut timing_offset = lock.timing_offset;
        let mut next_start = lock.next_start;
        let mut generation = self.generation;
        let mut alive = true;
        let mut integer_bins = lock.integer_bins;
        let mut coarse_locked = lock.coarse_locked;
        let mut coarse_candidate = lock.coarse_candidate;
        let mut coarse_hits = lock.coarse_hits;
        let mut emitted_symbols = lock.emitted_symbols;

        {
            let data = self.buf.make_contiguous();
            loop {
                if next_start < base {
                    // The predicted symbol fell off the back of the window.
                    alive = false;
                    break;
                }
                let local = (next_start - base) as usize;
                let needed = match local.checked_add(symbol_len) {
                    Some(end) => end,
                    None => {
                        alive = false;
                        break;
                    }
                };
                if needed > data.len() {
                    // Wait for more samples; this is not a loss of lock.
                    break;
                }

                let lo = local.saturating_sub(SEARCH_RADIUS);
                let hi = local
                    .saturating_add(SEARCH_RADIUS)
                    .min(data.len() - symbol_len);
                let mut best: Option<(usize, f32, Complex32)> = None;
                for offset in lo..=hi {
                    if let Some((corr, norm, power)) = cp_metric(data, offset, fft_size, guard_len)
                    {
                        if power < MIN_POWER || norm < TRACK_MIN {
                            continue;
                        }
                        let better = match &best {
                            None => true,
                            Some((best_offset, best_norm, _)) => {
                                if norm > *best_norm + ACQ_SCORE_EPS {
                                    true
                                } else if (norm - *best_norm).abs() <= ACQ_SCORE_EPS {
                                    // Break near-ties towards the predicted
                                    // start so the timing estimate cannot drift.
                                    offset.abs_diff(local) < best_offset.abs_diff(local)
                                } else {
                                    false
                                }
                            }
                        };
                        if better {
                            best = Some((offset, norm, corr));
                        }
                    }
                }

                let (offset, _norm, corr) = match best {
                    Some(value) => value,
                    None => {
                        // No valid CP structure around the predicted start:
                        // release the lock instead of emitting garbage.
                        alive = false;
                        break;
                    }
                };

                let start_abs = base + offset as u64;
                // Accumulate the chosen start's deviation from the fixed grid
                // anchored at acquisition. `next_start` follows the actual
                // start, so `offset - local` is exactly this symbol's grid
                // advance beyond the nominal `symbol_len`.
                let delta = offset as i64 - local as i64;
                timing_offset += delta;
                let measured = cfo_from_correlation(corr, fft_size, cfo);
                cfo = (1.0 - CFO_SMOOTHING) * cfo + CFO_SMOOTHING * measured;
                if !cfo.is_finite() {
                    alive = false;
                    break;
                }

                let symbol_samples = &data[offset..offset + symbol_len];
                let mut spectrum = match full_spectrum(symbol_samples, lock.mode, lock.guard, cfo) {
                    Ok(spectrum) => spectrum,
                    Err(_) => {
                        alive = false;
                        break;
                    }
                };

                let mut symbol_generation = generation;

                // Integer-subcarrier CFO acquisition. Only while the coarse
                // stage is unlocked: once it is confirmed (or a run of zeros
                // confirms there is no integer offset) the extra search stops.
                // A coherent segment has scattered pilots; a differential
                // (DQPSK) segment has none, so this never confirms and the
                // fractional-CFO path is retained unchanged.
                if !coarse_locked {
                    match estimate_integer_cfo(lock.mode, &spectrum, MAX_SEARCH_BINS) {
                        Some(estimate) => {
                            if estimate.bins == coarse_candidate {
                                coarse_hits = coarse_hits.saturating_add(1);
                            } else {
                                coarse_candidate = estimate.bins;
                                coarse_hits = 1;
                            }
                            if coarse_hits >= COARSE_CONFIRM_SYMBOLS {
                                coarse_locked = true;
                                integer_bins = estimate.bins;
                                if estimate.bins != 0 {
                                    let integer_radians = std::f32::consts::TAU
                                        * estimate.bins as f32
                                        / fft_size as f32;
                                    let total = cfo + integer_radians;
                                    if total.is_finite() {
                                        cfo = total;
                                        // Re-extract *this* symbol with the total
                                        // (fractional + integer) CFO so the
                                        // integer bin shift is removed from its
                                        // DFT, not merely from the carrier
                                        // readout.
                                        match full_spectrum(
                                            symbol_samples,
                                            lock.mode,
                                            lock.guard,
                                            cfo,
                                        ) {
                                            Ok(corrected) => spectrum = corrected,
                                            Err(_) => {
                                                alive = false;
                                                break;
                                            }
                                        }
                                        // Deterministic common-phase anchor:
                                        // `full_spectrum` de-rotates with the
                                        // local useful index, leaving
                                        // `exp(j * cfo_total * (start + guard))`.
                                        // Anchor the NCO to exactly the total
                                        // CFO at this symbol's useful start
                                        // once, then let it run continuously.
                                        nco_phase =
                                            cfo as f64 * (start_abs as f64 + guard_len as f64);
                                        // Carriers already emitted used the
                                        // wrong integer alignment, so a
                                        // downstream continuity-sensitive stage
                                        // must drop its state. A zero-bin
                                        // confirmation changes nothing.
                                        if emitted_symbols > 0 {
                                            symbol_generation = generation.saturating_add(1);
                                            generation = symbol_generation;
                                        }
                                    } else {
                                        // Degenerate total: stay unlocked and
                                        // keep the fractional estimate.
                                        coarse_locked = false;
                                    }
                                }
                            }
                        }
                        None => {
                            // Three *consecutive* confident estimates are
                            // required; any miss (no pilots, noise, a
                            // differential segment) restarts the run.
                            coarse_hits = 0;
                        }
                    }
                }

                let mut carriers = centre_carriers(&spectrum, lock.mode);
                // `full_spectrum` de-rotates with the local useful-part index
                // and therefore leaves a common phase
                // `exp(j * cfo * (start + guard))`. Remove the continuous NCO
                // phase accumulated across symbols. The phase is *not*
                // recomputed as `cfo * elapsed_samples`: the smoothed `cfo`
                // changes a little every symbol, and multiplying that change by
                // the ever-growing absolute index would jump the common phase
                // discontinuously and destroy the differential TMCC / coherent
                // equaliser reference.
                let phase = (-nco_phase) as f32;
                let (sin, cos) = phase.sin_cos();
                for carrier in &mut carriers {
                    let re = carrier.re * cos - carrier.im * sin;
                    let im = carrier.re * sin + carrier.im * cos;
                    *carrier = Complex32::new(re, im);
                }
                // Compensate the carrier-dependent phase left by the DFT time
                // shift relative to the fixed acquisition grid. A start that is
                // `timing_offset` samples away from the nominal grid multiplies
                // bin `k` by `exp(+j 2*pi*k*timing_offset/fft_size)`, so undo it
                // with the opposite sign. This must use the *cumulative* offset
                // (never the per-symbol jump alone): the differential TMCC
                // detector and the coherent equaliser only tolerate a
                // carrier-dependent phase that is constant across symbols. The
                // helper is a no-op at zero offset.
                compensate_timing_offset(&mut carriers, timing_offset, fft_size);
                // Advance the oscillator by the frequency actually applied to
                // this symbol's local de-rotation, over the *actual* samples
                // elapsed since the previous symbol (`symbol_len` plus this
                // symbol's timing delta).
                nco_phase += cfo as f64 * (symbol_len as f64 + delta as f64);
                out.push(SyncedSymbol {
                    mode: lock.mode,
                    guard: lock.guard,
                    carriers,
                    timing_offset,
                    generation: symbol_generation,
                });
                next_start = start_abs + symbol_len as u64;
                emitted_symbols = emitted_symbols.saturating_add(1);
            }
        }

        if alive {
            self.lock = Some(LockState {
                next_start,
                cfo,
                nco_phase,
                timing_offset,
                integer_bins,
                coarse_locked,
                coarse_candidate,
                coarse_hits,
                emitted_symbols,
                ..lock
            });
        } else {
            self.loss_count = self.loss_count.saturating_add(1);
            self.lock = None;
        }

        // Persist the (possibly coarse-correction-bumped) generation. Without
        // this, a bump made while emitting inside one `push` chunk would be
        // forgotten at the next chunk boundary, because `track` only held a
        // local copy; subsequent symbols would then be labelled with the stale
        // pre-correction generation.
        self.generation = generation;
    }

    /// Drop samples that can no longer contribute.
    fn trim(&mut self) {
        let drop = if let Some(lock) = self.lock {
            // Keep a little history before the predicted start for the search
            // window; everything earlier is already consumed.
            let keep_abs = lock.next_start.saturating_sub(SEARCH_RADIUS as u64);
            keep_abs
                .saturating_sub(self.base_index)
                .min(self.buf.len() as u64) as usize
        } else {
            self.buf.len().saturating_sub(ACQ_WINDOW)
        };
        for _ in 0..drop {
            self.buf.pop_front();
        }
        self.base_index = self.base_index.saturating_add(drop as u64);

        // Hard cap, independent of the acquisition/tracking target.
        let overflow = self.buf.len().saturating_sub(WINDOW_CAP);
        for _ in 0..overflow {
            self.buf.pop_front();
        }
        self.base_index = self.base_index.saturating_add(overflow as u64);
    }
}

impl Default for Synchronizer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// `(fft_size, guard_len, symbol_len)` for a Mode / guard pair.
fn fft_guard_len(mode: Mode, guard: GuardInterval) -> (usize, usize, usize) {
    let fft_size = segment_fft_size(mode);
    let symbol_len = segment_symbol_len(mode, guard);
    (fft_size, symbol_len - fft_size, symbol_len)
}

/// Mean squared magnitude of the correlation tail.
///
/// Normalised CP metric over a `guard_len`-sample prefix/tail pair:
/// returns `(sum conj(prefix) * tail, normalized, mean_tail_power)`.
///
/// The normalised value is `|corr| / sqrt(E_prefix * E_tail)` clamped to
/// `[0, 1]`; it is `0` when either energy is zero (silence).
fn cp_metric(
    data: &[Complex32],
    offset: usize,
    fft_size: usize,
    guard_len: usize,
) -> Option<(Complex32, f32, f32)> {
    let end = offset.checked_add(fft_size)?.checked_add(guard_len)?;
    if end > data.len() {
        return None;
    }

    let mut corr = Complex32::new(0.0, 0.0);
    let mut prefix_energy = 0.0f64;
    let mut tail_energy = 0.0f64;
    for k in 0..guard_len {
        let prefix = data[offset + k];
        let tail = data[offset + fft_size + k];
        corr += prefix.conj() * tail;
        prefix_energy +=
            (prefix.re as f64) * (prefix.re as f64) + (prefix.im as f64) * (prefix.im as f64);
        tail_energy += (tail.re as f64) * (tail.re as f64) + (tail.im as f64) * (tail.im as f64);
    }

    let denom = (prefix_energy * tail_energy).sqrt();
    let magnitude =
        ((corr.re as f64) * (corr.re as f64) + (corr.im as f64) * (corr.im as f64)).sqrt();
    let normalized = if denom > 0.0 {
        (magnitude / denom).clamp(0.0, 1.0) as f32
    } else {
        0.0
    };
    let power = (tail_energy / guard_len as f64) as f32;
    Some((corr, normalized, power))
}

/// Unit phasor of a correlation, or zero when it is degenerate.
fn normalize_phase(corr: Complex32) -> Complex32 {
    let norm = corr.norm();
    if norm > 0.0 && norm.is_finite() {
        corr / norm
    } else {
        Complex32::new(0.0, 0.0)
    }
}

/// Fractional CFO in radians per sample from a CP correlation.
///
/// The phase `arg(sum conj(prefix) * tail)` advances by `cfo * fft_size` over
/// the correlation lag, so the estimate is `arg(corr) / fft_size`. The result
/// is wrapped to the branch nearest `reference` so that smoothing does not see
/// a `2*pi/fft_size` discontinuity.
fn cfo_from_correlation(corr: Complex32, fft_size: usize, reference: f32) -> f32 {
    if !corr.re.is_finite() || !corr.im.is_finite() {
        return reference;
    }
    let raw = corr.arg() / fft_size as f32;
    let step = std::f32::consts::TAU / fft_size as f32;
    let mut value = raw;
    while value - reference > step * 0.5 {
        value -= step;
    }
    while reference - value > step * 0.5 {
        value += step;
    }
    value
}

/// Undo the carrier-dependent phase left by a DFT time shift.
///
/// If the extracted symbol start sits `timing_offset` samples away from the
/// fixed grid anchored at acquisition, the unitary DFT of the window multiplies
/// signed bin `k` by `exp(+j 2*pi*k*timing_offset/fft_size)`. Applying the
/// conjugate here removes it. Segment-local carrier `local` maps to signed bin
/// `local - len/2` (the centre-segment carriers are laid out around DC), which is
/// the same mapping [`extract_symbol`](super::ofdm::extract_symbol) returns them in.
///
/// This must be the *cumulative* offset across symbols, not the per-symbol
/// jump: the differential TMCC detector and coherent equaliser only tolerate a
/// carrier-dependent phase that is constant from symbol to symbol.
fn compensate_timing_offset(carriers: &mut [Complex32], timing_offset: i64, fft_size: usize) {
    if timing_offset == 0 || fft_size == 0 {
        return;
    }
    let half = (carriers.len() / 2) as i64;
    let scale = -std::f64::consts::TAU * timing_offset as f64 / fft_size as f64;
    for (local, carrier) in carriers.iter_mut().enumerate() {
        let signed_bin = local as i64 - half;
        let angle = (scale * signed_bin as f64) as f32;
        let (sin, cos) = angle.sin_cos();
        let re = carrier.re * cos - carrier.im * sin;
        let im = carrier.re * sin + carrier.im * cos;
        *carrier = Complex32::new(re, im);
    }
}

/// Re-apply the carrier-dependent phase ramp that
/// [`compensate_timing_offset`] removed, recovering the physical
/// (pre-de-ramp) carriers.
///
/// The de-ramp is a unitary per-carrier rotation, so its inverse is the same
/// rotation with the opposite `timing_offset`. A coherent scattered-pilot
/// channel estimator must run on the recovered carriers: the cumulative de-ramp
/// is a linear phase ramp across the carriers whose 12-carrier scattered-pilot
/// step grows without bound with the SFO trend, and once that step approaches
/// `pi` the complex linear pilot interpolation can no longer follow it. The
/// de-ramp must still be applied to [`SyncedSymbol::carriers`] for the
/// differential TMCC / DQPSK products, which is why it is undone here rather
/// than removed from the synchroniser.
pub(crate) fn restore_timing_offset(
    carriers: &mut [Complex32],
    timing_offset: i64,
    fft_size: usize,
) {
    compensate_timing_offset(carriers, timing_offset.saturating_neg(), fft_size);
}

/// Search all twelve Mode / guard candidates for the highest multi-symbol
/// normalised CP score.
fn acquire_best(data: &[Complex32]) -> Option<Acquired> {
    if data.len() < ACQ_MIN_SAMPLES {
        return None;
    }

    let mut best: Option<(f32, Acquired)> = None;

    for mode in Mode::ALL {
        for guard in GuardInterval::ALL {
            let (fft_size, guard_len, symbol_len) = fft_guard_len(mode, guard);
            // Use every full period that fits, up to the cap. Short guard
            // candidates therefore have to agree over many more periods than
            // long ones, which is the small-guard-alias rejection.
            let periods = (data.len() / symbol_len).min(MAX_ACQ_PERIODS);
            if periods < ACQUIRE_SYMBOLS {
                continue;
            }
            let need = periods * symbol_len;
            let max_offset = data.len() - need;

            for offset in 0..=max_offset {
                // The first period is evaluated first and used as a cheap gate:
                // it prunes almost every offset before the remaining periods are
                // computed, which keeps the search affordable.
                let (first_corr, first_norm, first_power) =
                    match cp_metric(data, offset, fft_size, guard_len) {
                        Some(metric) => metric,
                        None => continue,
                    };
                if first_power < MIN_POWER || first_norm < ACQ_MIN_SYMBOL {
                    continue;
                }

                let mut sum = first_norm;
                let mut phase_sum = normalize_phase(first_corr);
                let mut valid = true;
                for period in 1..periods {
                    let start = offset + period * symbol_len;
                    match cp_metric(data, start, fft_size, guard_len) {
                        Some((corr, norm, power)) => {
                            if power < MIN_POWER || norm < ACQ_MIN_SYMBOL {
                                valid = false;
                                break;
                            }
                            sum += norm;
                            phase_sum += normalize_phase(corr);
                        }
                        None => {
                            valid = false;
                            break;
                        }
                    }
                }
                if !valid {
                    continue;
                }

                // Every period must agree on the CFO phase; this additionally
                // rejects noise-only accidental matches.
                let coherence = phase_sum.norm() / periods as f32;
                if coherence < ACQ_CFO_COHERENCE {
                    continue;
                }

                let mean = sum / periods as f32;
                if mean < ACQ_MEAN_THRESHOLD {
                    continue;
                }
                let better = match &best {
                    None => true,
                    // Treat near-equal scores as ties so the earliest offset in
                    // the window wins deterministically.
                    Some((best_mean, _)) => mean > best_mean + ACQ_SCORE_EPS,
                };
                if better {
                    best = Some((
                        mean,
                        Acquired {
                            mode,
                            guard,
                            offset,
                            cfo: cfo_from_correlation(first_corr, fft_size, 0.0),
                        },
                    ));
                }
            }
        }
    }

    best.map(|(_, acquired)| acquired)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::demod::coding::Modulation;
    use crate::demod::control::{
        carriers_per_segment, pilot_modulating_signal, scattered_pilot_carriers, tmcc_carriers,
        PilotPrbs, SYMBOLS_PER_FRAME,
    };
    use rustfft::FftPlanner;

    /// Deterministic pseudo-random complex values (no external RNG dependency).
    fn test_carriers(count: usize, state: &mut u32) -> Vec<Complex32> {
        (0..count)
            .map(|_| {
                *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let re = ((*state >> 8) & 0xFFFF) as f32 / 32768.0 - 1.0;
                *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let im = ((*state >> 8) & 0xFFFF) as f32 / 32768.0 - 1.0;
                Complex32::new(re, im)
            })
            .collect()
    }

    /// Deterministic pseudo-random time-domain signal.
    fn test_signal(len: usize, seed: u32) -> Vec<Complex32> {
        let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        test_carriers(len, &mut state)
    }

    /// Unitary inverse DFT: `x[n] = (1/sqrt(N)) sum X[k] exp(+j 2 pi k n / N)`.
    fn unitary_ifft(spectrum: &[Complex32]) -> Vec<Complex32> {
        let mut planner = FftPlanner::<f32>::new();
        let inverse = planner.plan_fft_inverse(spectrum.len());
        let mut time = spectrum.to_vec();
        inverse.process(&mut time);
        let scale = 1.0 / (spectrum.len() as f32).sqrt();
        for value in &mut time {
            *value *= scale;
        }
        time
    }

    /// Map segment-local carrier `local` to its baseband FFT bin.
    fn centre_bin(fft_size: usize, carriers: usize, local: usize) -> usize {
        (local + fft_size - carriers / 2) % fft_size
    }

    /// Synthesise `count` contiguous OFDM symbols with a global CFO.
    ///
    /// Returns the sample stream and the expected (uncorrected) carrier vectors.
    fn synthesize(
        mode: Mode,
        guard: GuardInterval,
        count: usize,
        cfo: f32,
        seed: u32,
    ) -> (Vec<Complex32>, Vec<Vec<Complex32>>) {
        let fft_size = segment_fft_size(mode);
        let guard_len = guard.guard_len(fft_size);
        let carriers = carriers_per_segment(mode);
        let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);

        let mut stream = Vec::with_capacity(count * (fft_size + guard_len));
        let mut expected = Vec::with_capacity(count);
        for _ in 0..count {
            let values = test_carriers(carriers, &mut state);
            let mut spectrum = vec![Complex32::new(0.0, 0.0); fft_size];
            for (local, value) in values.iter().enumerate() {
                spectrum[centre_bin(fft_size, carriers, local)] = *value;
            }
            let useful = unitary_ifft(&spectrum);
            stream.extend_from_slice(&useful[fft_size - guard_len..]);
            stream.extend_from_slice(&useful);
            expected.push(values);
        }

        // Apply the CFO over the whole contiguous capture (global phase ramp).
        for (m, value) in stream.iter_mut().enumerate() {
            let angle = cfo * m as f32;
            let (sin, cos) = angle.sin_cos();
            *value = Complex32::new(
                value.re * cos - value.im * sin,
                value.re * sin + value.im * cos,
            );
        }

        (stream, expected)
    }

    fn assert_carriers_match(got: &[SyncedSymbol], expected: &[Vec<Complex32>], context: &str) {
        assert_eq!(got.len(), expected.len(), "{context}: symbol count");
        for (index, (symbol, want)) in got.iter().zip(expected.iter()).enumerate() {
            assert_eq!(
                symbol.carriers.len(),
                want.len(),
                "{context}: carriers {index}"
            );
            for (local, (actual, base)) in symbol.carriers.iter().zip(want.iter()).enumerate() {
                assert!(
                    (actual.re - base.re).abs() < 2e-3 && (actual.im - base.im).abs() < 2e-3,
                    "{context}: symbol {index} carrier {local}: got ({}, {}), want ({}, {})",
                    actual.re,
                    actual.im,
                    base.re,
                    base.im
                );
            }
        }
    }

    /// Deterministic **coherent** centre-segment carriers for global `symbol`:
    /// the normative scattered pilots and coherent TMCC landmarks (Table 3-15)
    /// for that symbol, with deterministic random data on every other carrier.
    fn coherent_carriers(mode: Mode, symbol: usize, state: &mut u32) -> Vec<Complex32> {
        let total = carriers_per_segment(mode);
        let mut carriers = test_carriers(total, state);
        for &carrier in &scattered_pilot_carriers(mode, Modulation::Qpsk, symbol) {
            let bit = PilotPrbs::bit_at(mode, 0, carrier).expect("SP carrier in segment");
            let (re, im) = pilot_modulating_signal(bit).expect("PRBS bit is 0/1");
            carriers[carrier] = Complex32::new(re, im);
        }
        for &carrier in tmcc_carriers(Modulation::Qpsk, mode) {
            // A coherent TMCC carrier is BPSK keyed by `signal XOR PRBS`; a
            // deterministic signal bit of 0 makes it the pilot value itself.
            let reference = PilotPrbs::bit_at(mode, 0, carrier).expect("TMCC carrier in segment");
            let (re, im) = pilot_modulating_signal(reference).expect("PRBS bit is 0/1");
            carriers[carrier] = Complex32::new(re, im);
        }
        carriers
    }

    /// Synthesise `count` coherent symbols and apply one global CFO.
    fn synthesize_coherent_cfo(
        mode: Mode,
        guard: GuardInterval,
        count: usize,
        cfo_radians_per_sample: f32,
        seed: u32,
    ) -> (Vec<Complex32>, Vec<Vec<Complex32>>) {
        let fft_size = segment_fft_size(mode);
        let guard_len = guard.guard_len(fft_size);
        let carriers = carriers_per_segment(mode);
        let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);

        let mut stream = Vec::with_capacity(count * (fft_size + guard_len));
        let mut expected = Vec::with_capacity(count);
        for symbol in 0..count {
            let values = coherent_carriers(mode, symbol, &mut state);
            let mut spectrum = vec![Complex32::new(0.0, 0.0); fft_size];
            for (local, value) in values.iter().enumerate() {
                spectrum[centre_bin(fft_size, carriers, local)] = *value;
            }
            let useful = unitary_ifft(&spectrum);
            stream.extend_from_slice(&useful[fft_size - guard_len..]);
            stream.extend_from_slice(&useful);
            expected.push(values);
        }

        for (m, value) in stream.iter_mut().enumerate() {
            let angle = cfo_radians_per_sample * m as f32;
            let (sin, cos) = angle.sin_cos();
            *value = Complex32::new(
                value.re * cos - value.im * sin,
                value.re * sin + value.im * cos,
            );
        }

        (stream, expected)
    }

    /// Synthesise `count` coherent symbols with an integer-subcarrier CFO.
    fn synthesize_coherent_integer(
        mode: Mode,
        guard: GuardInterval,
        count: usize,
        bins: i32,
        seed: u32,
    ) -> (Vec<Complex32>, Vec<Vec<Complex32>>) {
        let cfo = std::f32::consts::TAU * bins as f32 / segment_fft_size(mode) as f32;
        synthesize_coherent_cfo(mode, guard, count, cfo, seed)
    }

    /// Compare every symbol emitted under the *final* lock generation against
    /// the transmitted carriers, and return the index of the first one.
    ///
    /// Symbols emitted before an integer-CFO correction are aligned to the
    /// wrong bins and are deliberately skipped.
    fn assert_corrected_carriers_match(
        got: &[SyncedSymbol],
        expected: &[Vec<Complex32>],
        context: &str,
    ) -> usize {
        assert_eq!(got.len(), expected.len(), "{context}: symbol count");
        let final_generation = got.last().expect("at least one symbol").generation;
        let first = got
            .iter()
            .position(|symbol| symbol.generation == final_generation)
            .expect("the final generation must appear");
        for (index, (symbol, want)) in got.iter().zip(expected.iter()).enumerate().skip(first) {
            assert_eq!(
                symbol.carriers.len(),
                want.len(),
                "{context}: carriers {index}"
            );
            for (local, (actual, base)) in symbol.carriers.iter().zip(want.iter()).enumerate() {
                assert!(
                    (actual.re - base.re).abs() < 2e-3 && (actual.im - base.im).abs() < 2e-3,
                    "{context}: symbol {index} carrier {local}: got ({}, {}), want ({}, {})",
                    actual.re,
                    actual.im,
                    base.re,
                    base.im
                );
            }
        }
        first
    }

    #[test]
    fn synchronizes_every_mode_and_guard_without_cfo() {
        for mode in Mode::ALL {
            for guard in GuardInterval::ALL {
                let (stream, expected) = synthesize(mode, guard, 20, 0.0, 0x5EED_0000);
                let mut sync = Synchronizer::new();
                let got = sync.push(&stream);
                assert!(sync.is_locked(), "{mode:?}/{guard:?}: not locked");
                for symbol in &got {
                    assert_eq!(symbol.mode, mode);
                    assert_eq!(symbol.guard, guard);
                }
                assert_carriers_match(&got, &expected, "no-cfo");
                for symbol in &got {
                    assert_eq!(symbol.fft_size(), segment_fft_size(mode));
                    assert_eq!(symbol.symbol_len(), segment_symbol_len(mode, guard));
                }
            }
        }
    }

    #[test]
    fn lock_diagnostics_accessors_track_the_lock() {
        // Unlocked: every accessor must report absence rather than a default.
        let sync = Synchronizer::new();
        assert_eq!(sync.locked_mode_guard(), None);
        assert_eq!(sync.cfo_estimate(), None);
        assert_eq!(sync.timing_offset(), None);
        assert_eq!(sync.integer_bins(), None);

        for mode in Mode::ALL {
            for guard in GuardInterval::ALL {
                let (stream, _) = synthesize(mode, guard, 20, 0.0, 0x5EED_0000);
                let mut sync = Synchronizer::new();
                sync.push(&stream);
                assert!(sync.is_locked(), "{mode:?}/{guard:?}: not locked");
                assert_eq!(sync.locked_mode_guard(), Some((mode, guard)));
                assert!(sync.cfo_estimate().is_some_and(f32::is_finite));
                assert!(sync.timing_offset().is_some());
            }
        }
    }

    #[test]
    fn synchronizes_every_mode_and_guard_with_cfo() {
        // `cfo = 1/N` keeps `cfo * N = 1 rad` inside the unambiguous range.
        for mode in Mode::ALL {
            let fft_size = segment_fft_size(mode);
            let cfo = 1.0 / fft_size as f32;
            for guard in GuardInterval::ALL {
                let (stream, expected) = synthesize(mode, guard, 20, cfo, 0xCF0_0000);
                let mut sync = Synchronizer::new();
                let got = sync.push(&stream);
                assert!(sync.is_locked(), "{mode:?}/{guard:?}: not locked");
                assert_carriers_match(&got, &expected, "cfo");
            }
        }
    }

    #[test]
    fn cfo_phase_is_continuous_between_symbols() {
        // A coherent modulation reference compares consecutive symbols, so a
        // residual per-symbol phase ramp would break it. With the absolute
        // reference the extracted carriers must match the transmitted ones for
        // every symbol, including symbols far from the start.
        let mode = Mode::Mode3;
        let guard = GuardInterval::OneQuarter;
        let fft_size = segment_fft_size(mode);
        let cfo = 0.9 / fft_size as f32;
        let (stream, expected) = synthesize(mode, guard, 8, cfo, 0xABCD_1234);
        let mut sync = Synchronizer::new();
        let got = sync.push(&stream);
        assert!(sync.is_locked());
        assert_carriers_match(&got, &expected, "cfo continuity");
    }

    #[test]
    fn compensate_timing_offset_removes_known_phase_ramp() {
        // A DFT window that starts `timing_offset` samples away from the
        // acquisition grid multiplies signed bin `k` by
        // `exp(+j 2*pi*k*timing_offset/fft_size)`. Applying that exact ramp and
        // then compensating must return the original carriers for every mode
        // and a mix of positive, negative and zero offsets.
        for mode in Mode::ALL {
            let fft_size = segment_fft_size(mode);
            let count = carriers_per_segment(mode);
            let half = (count / 2) as i64;
            let mut state = 0x00C0_FFEEu32;
            let original = test_carriers(count, &mut state);

            for offset in [-4i64, -1, 0, 2, 5] {
                let mut carriers = original.clone();
                let scale = std::f64::consts::TAU * offset as f64 / fft_size as f64;
                for (local, carrier) in carriers.iter_mut().enumerate() {
                    let signed_bin = local as i64 - half;
                    let angle = (scale * signed_bin as f64) as f32;
                    let (sin, cos) = angle.sin_cos();
                    let re = carrier.re * cos - carrier.im * sin;
                    let im = carrier.re * sin + carrier.im * cos;
                    *carrier = Complex32::new(re, im);
                }

                compensate_timing_offset(&mut carriers, offset, fft_size);

                for (local, (recovered, base)) in carriers.iter().zip(original.iter()).enumerate() {
                    assert!(
                        (recovered.re - base.re).abs() < 1e-4
                            && (recovered.im - base.im).abs() < 1e-4,
                        "{mode:?} offset {offset} carrier {local}: got ({}, {}), want ({}, {})",
                        recovered.re,
                        recovered.im,
                        base.re,
                        base.im
                    );
                }
            }
        }
    }

    #[test]
    fn arbitrary_chunks_produce_identical_symbols() {
        let mode = Mode::Mode2;
        let guard = GuardInterval::OneSixteenth;
        let cfo = 0.5 / segment_fft_size(mode) as f32;
        let (stream, expected) = synthesize(mode, guard, 12, cfo, 0x1234_5678);

        let mut reference = Synchronizer::new();
        let reference_out = reference.push(&stream);
        assert_eq!(reference_out.len(), expected.len());
        assert_carriers_match(&reference_out, &expected, "contiguous");

        for chunk in [1usize, 2, 3, 5, 7, 13, 64, 1000] {
            let mut sync = Synchronizer::new();
            let mut out = Vec::new();
            for piece in stream.chunks(chunk) {
                out.extend(sync.push(piece));
            }
            assert!(sync.is_locked(), "chunk {chunk}: not locked");
            assert_eq!(
                out.len(),
                reference_out.len(),
                "chunk {chunk}: symbol count"
            );
            for (index, (actual, want)) in out.iter().zip(reference_out.iter()).enumerate() {
                assert_eq!(actual.mode, want.mode, "chunk {chunk} symbol {index}");
                assert_eq!(actual.guard, want.guard, "chunk {chunk} symbol {index}");
                for (local, (a, b)) in actual.carriers.iter().zip(want.carriers.iter()).enumerate()
                {
                    assert!(
                        (a.re - b.re).abs() < 1e-4 && (a.im - b.im).abs() < 1e-4,
                        "chunk {chunk} symbol {index} carrier {local}: ({}, {}) vs ({}, {})",
                        a.re,
                        a.im,
                        b.re,
                        b.im
                    );
                }
            }
        }
    }

    #[test]
    fn acquisition_is_robust_across_seeds() {
        // The detector must not depend on one lucky carrier draw: run every
        // Mode / guard combination against several deterministic seeds.
        for seed in [0x1111_2222u32, 0x3333_4444, 0xAAAA_BBBB, 0xF00D_1357] {
            for mode in Mode::ALL {
                let cfo = 0.7 / segment_fft_size(mode) as f32;
                for guard in GuardInterval::ALL {
                    let (stream, expected) = synthesize(mode, guard, 20, cfo, seed);
                    let mut sync = Synchronizer::new();
                    let got = sync.push(&stream);
                    assert!(sync.is_locked(), "seed {seed:#x} {mode:?}/{guard:?}");
                    assert_carriers_match(&got, &expected, "seed sweep");
                }
            }
        }
    }

    #[test]
    fn silence_and_noise_do_not_false_lock() {
        // Pure silence.
        let mut sync = Synchronizer::new();
        let silence = vec![Complex32::new(0.0, 0.0); 24000];
        let out = sync.push(&silence);
        assert!(out.is_empty(), "silence produced {} symbols", out.len());
        assert!(!sync.is_locked(), "false lock on silence");

        // Deterministic broadband noise with a comparable amplitude.
        let mut sync = Synchronizer::new();
        let noise = test_signal(24000, 0xDEAD_BEEF);
        let out = sync.push(&noise);
        assert!(out.is_empty(), "noise produced {} symbols", out.len());
        assert!(!sync.is_locked(), "false lock on noise");
    }

    #[test]
    fn gap_releases_lock_and_signal_reacquires() {
        let mode = Mode::Mode1;
        let guard = GuardInterval::OneEighth;
        let (stream, expected) = synthesize(mode, guard, 20, 0.0, 0x600D_0000);

        let mut sync = Synchronizer::new();
        let first = sync.push(&stream);
        assert!(sync.is_locked());
        assert_eq!(first.len(), expected.len());

        // A silent gap must release the lock, not keep emitting garbage.
        let silence = vec![Complex32::new(0.0, 0.0); 4000];
        let during = sync.push(&silence);
        assert!(during.is_empty(), "gap produced {} symbols", during.len());
        assert!(!sync.is_locked(), "lock survived a gap");

        // The same signal again must re-acquire.
        let again = sync.push(&stream);
        assert!(sync.is_locked(), "did not re-acquire after the gap");
        assert_carriers_match(&again, &expected, "re-acquire");
    }

    #[test]
    fn reset_clears_lock_and_is_locked_reports_state() {
        let (stream, _) = synthesize(Mode::Mode1, GuardInterval::OneQuarter, 20, 0.0, 7);
        let mut sync = Synchronizer::new();
        assert!(!sync.is_locked());
        sync.push(&stream);
        assert!(sync.is_locked());

        sync.reset();
        assert!(!sync.is_locked());

        // After reset the same stream must lock again from scratch.
        let out = sync.push(&stream);
        assert!(sync.is_locked());
        assert!(!out.is_empty());
    }

    #[test]
    fn rejects_short_and_empty_input_without_panicking() {
        let mut sync = Synchronizer::new();
        assert!(sync.push(&[]).is_empty());
        assert!(sync.push(&[Complex32::new(1.0, 0.0); 10]).is_empty());
        assert!(!sync.is_locked());
    }

    #[test]
    fn coarse_corrects_known_integer_cfo_across_modes() {
        // Every mode, a spread of negative / positive integer offsets, with the
        // normative scattered pilots present. After the required consecutive
        // confirmations the synchroniser must fold the integer offset into its
        // CFO and return the transmitted carriers (not merely a shifted readout).
        for mode in Mode::ALL {
            for &bins in &[-9i32, -2, 3, 11] {
                let (stream, expected) = synthesize_coherent_integer(
                    mode,
                    GuardInterval::OneEighth,
                    24,
                    bins,
                    0xB1A5_0000,
                );
                let mut sync = Synchronizer::new();
                let got = sync.push(&stream);
                assert!(sync.is_locked(), "{mode:?} bins {bins}: not locked");
                assert_eq!(
                    sync.integer_bins(),
                    Some(bins),
                    "{mode:?} bins {bins}: coarse estimate"
                );
                let first =
                    assert_corrected_carriers_match(&got, &expected, "coherent integer cfo");
                assert!(
                    first >= 1,
                    "{mode:?} bins {bins}: correction must not precede any output"
                );
                assert!(
                    got.iter()
                        .any(|symbol| symbol.generation != got.last().unwrap().generation),
                    "{mode:?} bins {bins}: a non-zero correction must bump the generation"
                );
            }
        }
    }

    #[test]
    fn coarse_corrects_fractional_plus_integer_cfo() {
        // The fractional CFO loop must keep working while the integer stage
        // supplies the missing subcarrier shift.
        for mode in Mode::ALL {
            let bins = 3i32;
            let fractional = 0.4 / segment_fft_size(mode) as f32;
            let cfo =
                std::f32::consts::TAU * bins as f32 / segment_fft_size(mode) as f32 + fractional;
            let (stream, expected) =
                synthesize_coherent_cfo(mode, GuardInterval::OneQuarter, 24, cfo, 0x0FAC_0000);
            let mut sync = Synchronizer::new();
            let got = sync.push(&stream);
            assert!(sync.is_locked(), "{mode:?}: not locked");
            assert_eq!(sync.integer_bins(), Some(bins), "{mode:?}: coarse estimate");
            assert_corrected_carriers_match(&got, &expected, "coherent fractional+integer");
        }
    }

    #[test]
    fn coarse_corrects_a_whole_coherent_tmcc_frame_stream() {
        // A full frame plus margin exercises all four scattered-pilot phases
        // (including the wrap into the next frame) on a realistic coherent
        // stream carrying normative pilots and TMCC carriers.
        let mode = Mode::Mode3;
        let bins = -2i32;
        let (stream, expected) = synthesize_coherent_integer(
            mode,
            GuardInterval::OneEighth,
            SYMBOLS_PER_FRAME + 8,
            bins,
            0x7C0F_FEE0,
        );
        let mut sync = Synchronizer::new();
        let got = sync.push(&stream);
        assert!(sync.is_locked());
        assert_eq!(sync.integer_bins(), Some(bins));
        assert_corrected_carriers_match(&got, &expected, "coherent TMCC frame stream");
    }

    #[test]
    fn random_data_without_pilots_has_no_integer_coarse_lock() {
        // A differential / no-scattered-pilot stream offers the coarse estimator
        // no frequency reference: it must never confirm, so the synchroniser
        // keeps its fractional-only CFO and never bumps the generation. (This is
        // why integer CFO for DQPSK has no automatic correction.)
        for mode in Mode::ALL {
            let bins = 5i32;
            let cfo = std::f32::consts::TAU * bins as f32 / segment_fft_size(mode) as f32;
            let (stream, _expected) =
                synthesize(mode, GuardInterval::OneEighth, 24, cfo, 0xD1FF_0000);
            let mut sync = Synchronizer::new();
            let got = sync.push(&stream);
            assert!(sync.is_locked(), "{mode:?}: not locked");
            assert_eq!(
                sync.integer_bins(),
                None,
                "{mode:?}: random data must not confirm an integer offset"
            );
            assert!(
                got.iter()
                    .all(|symbol| symbol.generation == got[0].generation),
                "{mode:?}: an unconfirmed coarse stage must not bump the generation"
            );
        }
    }

    #[test]
    fn coarse_zero_estimate_stops_the_extra_search() {
        // A coherent stream with no integer offset confirms `bins == 0` and then
        // stops looking; the fractional CFO continuity is untouched and no
        // generation bump occurs.
        let mode = Mode::Mode2;
        let (stream, expected) =
            synthesize_coherent_integer(mode, GuardInterval::OneSixteenth, 24, 0, 0x0BEE_F00D);
        let mut sync = Synchronizer::new();
        let got = sync.push(&stream);
        assert!(sync.is_locked());
        assert_eq!(sync.integer_bins(), Some(0));
        assert_carriers_match(&got, &expected, "coherent zero integer");
    }
}
