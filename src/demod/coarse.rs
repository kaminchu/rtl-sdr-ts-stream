//! Coarse (integer-subcarrier) carrier-frequency-offset estimation for the
//! coherent ISDB-T centre (partial-reception) segment.
//!
//! The normative basis is ARIB STD-B31 version 2.2-E1 §3.12.2 and Fig. 3-30
//! (scattered pilots: one carrier every 12, one of four residues per OFDM
//! symbol) and §3.13.1/Table 3-16/Table 3-17 (the BPSK pilot PRBS reference
//! `W_i` and its `+-4/3` signal points).
//!
//! # What this module does
//!
//! [`crate::demod::sync`] estimates only the *fractional* carrier-frequency
//! offset, because the cyclic-prefix phase `arg(corr)` is periodic with the
//! subcarrier spacing. This module adds the missing *integer* subcarrier part
//! for a **coherent** segment, using the scattered pilots as a known frequency
//! reference.
//!
//! [`estimate_integer_cfo`] takes the **unitary full forward DFT** of one
//! centre-segment OFDM symbol (`256 / 512 / 1024` bins for Modes 1/2/3) that has
//! already been de-rotated by the fractional CFO, i.e. exactly the signed-bin
//! spectrum from which [`crate::demod::ofdm::extract_symbol`] would later read
//! the `108 / 216 / 432` centre carriers. For every candidate integer shift
//! `s in [-max_bins, +max_bins]` (capped at [`MAX_SEARCH_BINS`]) and every one
//! of the four possible scattered-pilot symbol phases it:
//!
//! 1. reads the observed bin `local - C/2 + s` (wrapped) for each scattered
//!    pilot `local` of that symbol phase, where `C = carriers_per_segment(mode)`;
//! 2. divides by the normative pilot PRBS value
//!    `P_local = +-4/3` (control::[`PilotPrbs`]) to obtain a single-tap channel
//!    sample `H_local = observed / P_local`;
//! 3. measures how *smooth* that `H` sequence is with the mean magnitude of the
//!    **normalized complex neighbour products**
//!    `L = |mean_i  H_i * conj(H_{i+1}) / (|H_i| |H_{i+1}|)|`, which is
//!    invariant to a common complex gain and, crucially, to the constant phase
//!    step produced by a timing offset (a linear phase ramp across frequency).
//!
//! For the true integer shift and the true symbol phase the `H` sequence is the
//! actual channel response and the neighbour products all point the same way
//! (score `~1`). For any other shift the "pilots" are data carriers or the wrong
//! PRBS positions, so `H` is random and the score collapses. Because only the
//! *phase* coherence is used, a timing offset that rotates every carrier by a
//! carrier-dependent but symbol-common linear phase cannot break the metric.
//!
//! The reported score also carries a scale-invariant **magnitude-consistency**
//! factor `mean_i 2 min(|H_i|, |H_{i+1}|) / (|H_i| + |H_{i+1}|)`, which
//! penalizes estimates whose magnitudes are not those of a smooth channel (for
//! example Gaussian noise). It is a secondary factor only; the primary evidence
//! is the complex phase coherence above, never amplitude alone.
//!
//! # Acceptance
//!
//! A result is returned only when
//!
//! * `score > `[`SCORE_THRESHOLD`] (`0.8`), and
//! * `score - runner_up > `[`MARGIN_THRESHOLD`] (`0.1`),
//!
//! where the runner-up is the best candidate whose pilot lattice alignment
//! differs from the winner's. Candidates with the same alignment (the lattice
//! duplicates / phase aliases produced by shifting both the carrier and the
//! symbol phase by a matching amount) are not counted: they read the same
//! observed carrier set and are the same physical hypothesis. When neither
//! condition holds — silence, an all-zero spectrum, random data, a burst that is
//! not a coherent one-segment symbol — `None` is returned rather than a guess.
//!
//! # Applying the estimate
//!
//! Let `C = carriers_per_segment(mode)` and `N = segment_fft_size(mode)`. Once
//! `estimate = estimate_integer_cfo(...)` is `Some`, carrier extraction is the
//! standard centre-segment demap shifted by the integer offset:
//!
//! ```text
//! out[local] = spectrum[(local + N - C/2 + estimate.bins) mod N]
//!              for local = 0 .. C-1
//! ```
//!
//! which is [`crate::demod::ofdm::extract_symbol`]'s mapping with `+ bins`.
//! `estimate.symbol_phase` (= `OFDM symbol index mod 4`) selects the SP residue
//! and therefore also tells a subsequent channel estimator which pilot phase to
//! use.
//!
//! After the integer shift is removed, a residual **symbol-common phase** remains
//! on every carrier. It is not estimated here. It must be removed before
//! coherent equalisation / differential detection, e.g. by the synchroniser's
//! continuous NCO common-phase reference (exactly as [`crate::demod::sync`]
//! already removes the phase left by the fractional CFO), or by a per-symbol
//! pilot common-phase estimate. Integer correction and common-phase correction
//! are two separate steps.
//!
//! # Limitations
//!
//! This estimator only works for a **coherent** segment with scattered pilots
//! (QPSK/16QAM/64QAM layer A). A differential (DQPSK) segment carries no
//! scattered pilots (Table 3-1), so it has no such frequency reference and
//! returns `None` in practice: coarse CFO for DQPSK is **not** supported here.
//! A caller that knows the receiver's ppm error should use that as a fallback.
//! This is a coarse, high-confidence detector for a single symbol; it does not
//! track the offset over time and does not replace the fractional-CFO loop.
//!
//! # Untrusted input
//!
//! [`estimate_integer_cfo`] validates the spectrum length and never panics on a
//! short, long or non-finite input: a mismatched length or a candidate that
//! touches a non-finite sample simply yields `None` / a zero score.

use num_complex::Complex32;

use crate::dsp::Mode;

use super::control::{
    carriers_per_segment, pilot_modulating_signal, scattered_pilot_residue, PilotPrbs,
};
use super::ofdm::segment_fft_size;

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/// Hard cap on `max_bins`, i.e. the search is always within `+-32` subcarriers.
///
/// The fractional-CFO stage leaves an error well under one subcarrier, so the
/// integer part is expected to be small; the cap keeps the search bounded for
/// untrusted arguments.
pub const MAX_SEARCH_BINS: usize = 32;

/// Scattered-pilot spacing in subcarriers (§3.12.2, Fig. 3-30).
const SPACING: usize = 12;

/// Number of symbol phases in the four-symbol scattered-pilot cycle.
const SYMBOL_PHASES: usize = 4;

/// Minimum accepted score.
const SCORE_THRESHOLD: f32 = 0.8;

/// Minimum required lead of the winner over the best non-alias runner-up.
const MARGIN_THRESHOLD: f32 = 0.1;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// An integer-subcarrier CFO estimate with the confidence that produced it.
///
/// `bins` is signed in subcarriers and is the value to *add* inside the
/// centre-segment demap index: `out[local] = spectrum[(local - C/2 + bins) mod N]`.
/// A positive value means the received spectrum is shifted up (towards higher
/// frequencies) relative to the transmitter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoarseEstimate {
    /// Integer carrier-frequency offset in subcarriers.
    pub bins: i32,
    /// The scattered-pilot symbol phase that scored best (`OFDM symbol index
    /// mod 4`, so `scattered_pilot_residue(symbol_phase) = 3 * symbol_phase`).
    pub symbol_phase: usize,
    /// Confidence in `[0, 1]`; the primary evidence is the normalized complex
    /// neighbour-product coherence, penalized by magnitude inconsistency.
    pub score: f32,
}

// ---------------------------------------------------------------------------
// Estimator
// ---------------------------------------------------------------------------

/// Estimate the integer subcarrier part of the carrier-frequency offset.
///
/// `spectrum` is the unitary full forward DFT of one centre-segment OFDM
/// symbol (`segment_fft_size(mode)` bins), already de-rotated by the fractional
/// CFO, with the centre segment at signed bins
/// `local - carriers_per_segment(mode) / 2`.
///
/// The search covers `bins in [-max_bins, max_bins]`, capped at
/// [`MAX_SEARCH_BINS`], crossed with the four scattered-pilot symbol phases. A
/// result is returned only for a high-confidence, unambiguous match; see the
/// module documentation for the acceptance rule.
///
/// Returns `None` when `spectrum.len()` is not the mode's segment FFT size,
/// when `max_bins` is `0` and the only candidate is not convincing, or when no
/// candidate clears the score/margin thresholds (silence, noise, all-zero
/// spectrum, random data, a differential segment, …).
pub fn estimate_integer_cfo(
    mode: Mode,
    spectrum: &[Complex32],
    max_bins: usize,
) -> Option<CoarseEstimate> {
    if spectrum.len() != segment_fft_size(mode) {
        return None;
    }

    let pilots = expected_pilot_table(mode);
    let limit = max_bins.min(MAX_SEARCH_BINS) as i32;

    // Evaluate every candidate once: (alignment class, estimate).
    let mut candidates: Vec<(i32, CoarseEstimate)> =
        Vec::with_capacity(((2 * limit + 1) as usize) * SYMBOL_PHASES);
    for bins in -limit..=limit {
        for symbol_phase in 0..SYMBOL_PHASES {
            let score = candidate_score(mode, spectrum, &pilots, bins, symbol_phase);
            candidates.push((
                alignment_class(bins, symbol_phase),
                CoarseEstimate {
                    bins,
                    symbol_phase,
                    score,
                },
            ));
        }
    }

    // Best candidate; strict `>` keeps the earliest (most negative bins, lowest
    // phase) candidate on exact ties, which is deterministic.
    let mut best_index = 0usize;
    for (index, candidate) in candidates.iter().enumerate() {
        if candidate.1.score > candidates[best_index].1.score {
            best_index = index;
        }
    }
    let best = candidates[best_index];

    // `partial_cmp` (rather than `!`) keeps a NaN score rejecting the estimate.
    if best.1.score.partial_cmp(&SCORE_THRESHOLD) != Some(std::cmp::Ordering::Greater) {
        return None;
    }

    // Runner-up among candidates of a *different* pilot-lattice alignment.
    // Same-alignment candidates are duplicates / phase aliases of the winner
    // (they read the same observed carriers) and carry no independent evidence.
    let mut runner_up = f32::NEG_INFINITY;
    for candidate in &candidates {
        if candidate.0 != best.0 && candidate.1.score > runner_up {
            runner_up = candidate.1.score;
        }
    }

    // `partial_cmp` (rather than `!`) keeps a NaN margin rejecting the estimate.
    if (best.1.score - runner_up).partial_cmp(&MARGIN_THRESHOLD)
        != Some(std::cmp::Ordering::Greater)
    {
        return None;
    }

    Some(best.1)
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Pilot-lattice alignment class of a `(bins, symbol_phase)` candidate.
///
/// The observed carrier residues of the candidate are `3 * symbol_phase + bins`
/// (mod 12). Two candidates with the same class read the same scattered-pilot
/// lattice and are physical duplicates, even though their `(bins,
/// symbol_phase)` split differs.
fn alignment_class(bins: i32, symbol_phase: usize) -> i32 {
    (bins + 3 * symbol_phase as i32).rem_euclid(SPACING as i32)
}

/// Normative BPSK pilot value for every segment-local carrier, indexed by
/// `local` (`0 .. carriers_per_segment(mode) - 1`).
///
/// `table[local]` is the value of `PilotPrbs::bit_at(mode, 0, local)` mapped
/// through [`pilot_modulating_signal`] (`W = 0 -> +4/3`, `W = 1 -> -4/3`,
/// Table 3-17).
fn expected_pilot_table(mode: Mode) -> Vec<Complex32> {
    let carriers = carriers_per_segment(mode);
    let mut generator = PilotPrbs::center(mode);
    let mut table = Vec::with_capacity(carriers);
    for _ in 0..carriers {
        let bit = generator.next_bit();
        let (re, im) = pilot_modulating_signal(bit).expect("PRBS output bit is 0 or 1");
        table.push(Complex32::new(re, im));
    }
    table
}

/// Confidence of one `(bins, symbol_phase)` candidate.
///
/// Returns a value in `[0, 1]`, or `0.0` when the candidate reads a non-finite
/// sample or has fewer than two pilots.
fn candidate_score(
    mode: Mode,
    spectrum: &[Complex32],
    pilots: &[Complex32],
    bins: i32,
    symbol_phase: usize,
) -> f32 {
    let fft_size = segment_fft_size(mode);
    let carriers = carriers_per_segment(mode);
    let half = (carriers / 2) as isize;
    let residue = scattered_pilot_residue(symbol_phase);

    let mut phase_re = 0.0f64;
    let mut phase_im = 0.0f64;
    let mut magnitude = 0.0f64;
    let mut pairs = 0usize;
    let mut previous: Option<Complex32> = None;

    for local in (residue..carriers).step_by(SPACING) {
        let index = (local as isize - half + bins as isize).rem_euclid(fft_size as isize) as usize;
        let observed = spectrum[index];
        if !observed.re.is_finite() || !observed.im.is_finite() {
            return 0.0;
        }
        // The pilot value is always non-zero (|P| = 4/3), so this division is
        // well defined for finite input.
        let estimate = observed / pilots[local];

        if let Some(prev) = previous {
            pairs += 1;
            let pr = prev.re as f64;
            let pi = prev.im as f64;
            let cr = estimate.re as f64;
            let ci = estimate.im as f64;
            let prev_norm = (pr * pr + pi * pi).sqrt();
            let curr_norm = (cr * cr + ci * ci).sqrt();
            if prev_norm > 0.0 && curr_norm > 0.0 {
                // unit(prev) * conj(unit(estimate)): the normalized complex
                // neighbour product. A common complex gain and a common phase
                // step (timing offset) cancel, so this is the channel smoothness.
                let ur = pr / prev_norm;
                let ui = pi / prev_norm;
                let vr = cr / curr_norm;
                let vi = -ci / curr_norm;
                phase_re += ur * vr - ui * vi;
                phase_im += ur * vi + ui * vr;
                magnitude += 2.0 * prev_norm.min(curr_norm) / (prev_norm + curr_norm);
            }
        }
        previous = Some(estimate);
    }

    if pairs == 0 {
        return 0.0;
    }

    let pair_count = pairs as f64;
    let phase = ((phase_re * phase_re + phase_im * phase_im).sqrt() / pair_count).clamp(0.0, 1.0);
    let magnitude = (magnitude / pair_count).clamp(0.0, 1.0);
    (phase * magnitude) as f32
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::demod::coding::Modulation;
    use crate::demod::control::{scattered_pilot_carriers, PilotPrbs};

    /// Deterministic xorshift32; no external RNG dependency.
    fn next_state(state: &mut u32) -> u32 {
        let mut x = *state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *state = x;
        x
    }

    /// Deterministic QPSK value from two PSK bits (unit amplitude).
    fn qpsk(state: &mut u32) -> Complex32 {
        let bits = next_state(state) & 0b11;
        match bits {
            0b00 => Complex32::new(1.0, 0.0),
            0b01 => Complex32::new(0.0, 1.0),
            0b10 => Complex32::new(-1.0, 0.0),
            _ => Complex32::new(0.0, -1.0),
        }
    }

    /// Normative pilot value at segment-local carrier `local`.
    fn pilot_value(mode: Mode, local: usize) -> Complex32 {
        let bit = PilotPrbs::bit_at(mode, 0, local).unwrap();
        let (re, im) = pilot_modulating_signal(bit).unwrap();
        Complex32::new(re, im)
    }

    /// Build the unitary full spectrum of one symbol for `symbol_phase` and an
    /// integer CFO of `cfo` subcarriers, under `channel(local)`.
    ///
    /// Scattered-pilot carriers carry the normative PRBS pilots; every other
    /// carrier carries a deterministic QPSK data symbol. The spectrum is then
    /// shifted by `cfo` (`rx[b] = tx[b - cfo]`), which is what an integer CFO
    /// does to the received DFT.
    fn synthesize(
        mode: Mode,
        symbol_phase: usize,
        cfo: i32,
        channel: impl Fn(usize) -> Complex32,
        seed: u32,
    ) -> Vec<Complex32> {
        let fft_size = segment_fft_size(mode);
        let carriers = carriers_per_segment(mode);
        let half = carriers / 2;

        let mut is_pilot = vec![false; carriers];
        for local in scattered_pilot_carriers(mode, Modulation::Qpsk, symbol_phase) {
            is_pilot[local] = true;
        }

        let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        let mut transmitted = vec![Complex32::new(0.0, 0.0); fft_size];
        for (local, &is_pilot_carrier) in is_pilot.iter().enumerate() {
            let value = if is_pilot_carrier {
                pilot_value(mode, local)
            } else {
                qpsk(&mut state)
            };
            let bin = (local + fft_size - half) % fft_size;
            transmitted[bin] = channel(local) * value;
        }

        let mut received = vec![Complex32::new(0.0, 0.0); fft_size];
        for (b, slot) in received.iter_mut().enumerate() {
            let source = (b as isize - cfo as isize).rem_euclid(fft_size as isize) as usize;
            *slot = transmitted[source];
        }
        received
    }

    fn flat_channel(_local: usize) -> Complex32 {
        Complex32::from_polar(0.7, 0.9)
    }

    /// A frequency-selective channel with a linear phase (a fixed timing offset)
    /// and a mild amplitude ripple. Its phase ramp must not be mistaken for an
    /// integer CFO.
    fn linear_phase_selective(mode: Mode) -> impl Fn(usize) -> Complex32 {
        let fft_size = segment_fft_size(mode) as f32;
        move |local: usize| {
            let amplitude = 1.0 + 0.2 * (std::f32::consts::TAU * local as f32 / 41.0).cos();
            let phase = std::f32::consts::TAU * 4.0 * local as f32 / fft_size;
            Complex32::from_polar(amplitude, phase)
        }
    }

    #[test]
    fn pilot_table_matches_control_prbs() {
        for mode in Mode::ALL {
            let table = expected_pilot_table(mode);
            assert_eq!(table.len(), carriers_per_segment(mode));
            for (local, value) in table.iter().enumerate() {
                assert_eq!(*value, pilot_value(mode, local), "{mode:?} local {local}");
            }
        }
    }

    #[test]
    fn estimates_all_modes_phases_and_bins_on_flat_and_selective_channels() {
        let cfo_values: [i32; 5] = [-12, -3, 0, 2, 11];

        for mode in Mode::ALL {
            let selective = linear_phase_selective(mode);
            for symbol_phase in 0..SYMBOL_PHASES {
                for &cfo in &cfo_values {
                    let flat = synthesize(mode, symbol_phase, cfo, flat_channel, 0xC0FFEE);
                    let shaped = synthesize(mode, symbol_phase, cfo, &selective, 0xC0FFEE);
                    for (name, spectrum) in [("flat", flat), ("selective", shaped)] {
                        let estimate = estimate_integer_cfo(mode, &spectrum, MAX_SEARCH_BINS)
                            .unwrap_or_else(|| {
                                panic!(
                                    "{mode:?} {name} phase {symbol_phase} cfo {cfo}: no estimate"
                                )
                            });
                        assert_eq!(
                            estimate.bins, cfo,
                            "{mode:?} {name} phase {symbol_phase}: bins"
                        );
                        assert_eq!(
                            estimate.symbol_phase, symbol_phase,
                            "{mode:?} {name} cfo {cfo}: symbol phase"
                        );
                        assert!(
                            estimate.score > SCORE_THRESHOLD,
                            "{mode:?} {name} phase {symbol_phase} cfo {cfo}: score {}",
                            estimate.score
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn timing_phase_ramp_is_not_reported_as_cfo() {
        // A pure linear phase ramp (a timing offset), no integer CFO. The
        // normalized complex neighbour products are invariant to the common
        // phase step, so the estimate must be exactly 0.
        for mode in Mode::ALL {
            let fft_size = segment_fft_size(mode) as f32;
            let channel = move |local: usize| {
                Complex32::from_polar(1.0, std::f32::consts::TAU * 7.0 * local as f32 / fft_size)
            };
            let spectrum = synthesize(mode, 2, 0, channel, 0x1234_5678);
            let estimate = estimate_integer_cfo(mode, &spectrum, MAX_SEARCH_BINS)
                .unwrap_or_else(|| panic!("{mode:?}: no estimate"));
            assert_eq!(estimate.bins, 0, "{mode:?}: timing ramp became CFO");
        }
    }

    #[test]
    fn rejects_zero_silence_and_noise() {
        for mode in Mode::ALL {
            let fft_size = segment_fft_size(mode);
            let zeros = vec![Complex32::new(0.0, 0.0); fft_size];
            assert!(
                estimate_integer_cfo(mode, &zeros, MAX_SEARCH_BINS).is_none(),
                "{mode:?}: all-zero spectrum must not produce a coarse estimate"
            );

            // Deterministic broadband noise at a comparable amplitude. Both a
            // flat (uniform) and a Gaussian-like (sum of uniforms) draw are
            // exercised across many seeds; neither may produce a coarse
            // estimate.
            for seed in 0..32u32 {
                let mut uniform_state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
                let uniform: Vec<Complex32> = (0..fft_size)
                    .map(|_| {
                        let re = (next_state(&mut uniform_state) >> 8) as f32 / 32768.0 - 1.0;
                        let im = (next_state(&mut uniform_state) >> 8) as f32 / 32768.0 - 1.0;
                        Complex32::new(re, im)
                    })
                    .collect();
                assert!(
                    estimate_integer_cfo(mode, &uniform, MAX_SEARCH_BINS).is_none(),
                    "{mode:?} seed {seed}: uniform noise produced a coarse estimate"
                );

                let mut gaussian_state = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let gaussian = |state: &mut u32| {
                    let sum: f32 = (0..8)
                        .map(|_| (next_state(state) >> 8) as f32 / 32768.0)
                        .sum();
                    (sum - 4.0) * 1.7
                };
                let normal: Vec<Complex32> = (0..fft_size)
                    .map(|_| {
                        Complex32::new(gaussian(&mut gaussian_state), gaussian(&mut gaussian_state))
                    })
                    .collect();
                assert!(
                    estimate_integer_cfo(mode, &normal, MAX_SEARCH_BINS).is_none(),
                    "{mode:?} seed {seed}: Gaussian noise produced a coarse estimate"
                );
            }
        }
    }

    #[test]
    fn rejects_out_of_range_cfo_and_bounds_the_search() {
        let mode = Mode::Mode2;
        let fft_size = segment_fft_size(mode);

        // Within the search range: found.
        let in_range = synthesize(mode, 1, 5, flat_channel, 1);
        assert_eq!(
            estimate_integer_cfo(mode, &in_range, 8).map(|e| e.bins),
            Some(5)
        );

        // A CFO outside `max_bins` is not searched, so no high-confidence match.
        let out_of_range = synthesize(mode, 1, 20, flat_channel, 1);
        assert!(estimate_integer_cfo(mode, &out_of_range, 10).is_none());

        // `max_bins` is capped at MAX_SEARCH_BINS: an enormous request still
        // can see a CFO up to the cap (20 <= 32) ...
        assert_eq!(
            estimate_integer_cfo(mode, &out_of_range, 1_000_000).map(|e| e.bins),
            Some(20)
        );
        // ... but not one beyond it (40 > 32).
        let beyond_cap = synthesize(mode, 1, 40, flat_channel, 1);
        assert!(estimate_integer_cfo(mode, &beyond_cap, 1_000_000).is_none());

        // `max_bins = 0` searches only bin 0.
        let zero = synthesize(mode, 3, 0, flat_channel, 2);
        assert_eq!(
            estimate_integer_cfo(mode, &zero, 0).map(|e| e.bins),
            Some(0)
        );
        assert!(estimate_integer_cfo(mode, &in_range, 0).is_none());

        // Sanity: the cap constant is the documented 32.
        assert_eq!(MAX_SEARCH_BINS, 32);
        let _ = fft_size;
    }

    #[test]
    fn validates_spectrum_length_and_non_finite_input() {
        for mode in Mode::ALL {
            let fft_size = segment_fft_size(mode);
            assert!(estimate_integer_cfo(mode, &[], MAX_SEARCH_BINS).is_none());
            assert!(
                estimate_integer_cfo(mode, &vec![Complex32::new(0.0, 0.0); fft_size - 1], 8)
                    .is_none()
            );
            assert!(
                estimate_integer_cfo(mode, &vec![Complex32::new(0.0, 0.0); fft_size + 1], 8)
                    .is_none()
            );

            // A non-finite sample can never yield a confident estimate: every
            // candidate that reads it is disqualified.
            assert!(
                estimate_integer_cfo(mode, &vec![Complex32::new(f32::NAN, 0.0); fft_size], 8)
                    .is_none()
            );
            assert!(estimate_integer_cfo(
                mode,
                &vec![Complex32::new(0.0, f32::INFINITY); fft_size],
                8
            )
            .is_none());
            let spectrum = synthesize(mode, 0, 0, flat_channel, 3);
            let mut partial = spectrum.clone();
            // The bin a zero-shift phase-0 candidate reads for pilot 0.
            let half = carriers_per_segment(mode) / 2;
            partial[(fft_size - half) % fft_size] = Complex32::new(f32::NAN, 0.0);
            // A single poisoned pilot read can only lower that candidate; the
            // estimator must still never panic and must not return a non-finite
            // score.
            if let Some(estimate) = estimate_integer_cfo(mode, &partial, MAX_SEARCH_BINS) {
                assert!(estimate.score.is_finite());
            }
        }
    }
}
