//! Centre-segment carrier recovery for ISDB-T: coherent channel estimation /
//! equalisation and differential (DQPSK) demodulation.
//!
//! This is an original implementation written from scratch for this crate. No
//! GNU Radio, `gr-isdbt`, Python or other existing demodulator source was
//! consulted or copied. The normative basis is ARIB STD-B31 version 2.2-E1,
//! §3.12.1/Table 3-14 (differential OFDM-segment configuration), §3.12.2
//! (coherent OFDM-segment configuration), §3.13.1 (scattered pilots),
//! §3.9.3.1/Table 3-10 (`pi/4`-shift DQPSK) and §3.11.2.2 (intra-segment
//! frequency interleaving).
//!
//! # What this module does
//!
//! [`equalize_symbol`] takes the `108 / 216 / 432` centre-segment carriers of
//! one coherent OFDM symbol, in ascending segment-local carrier order, exactly
//! as returned by [`crate::demod::ofdm::extract_symbol`], and performs the
//! coherent-demodulation channel correction:
//!
//! 1. **Locate the scattered pilots (SPs).** SPs sit every 12 carriers and
//!    every 4 symbols; for symbol `j` the residue is `3 * (j mod 4) mod 12`
//!    (§3.12.2, Fig. 3-30). The positions are taken verbatim from
//!    [`crate::demod::control::scattered_pilot_carriers`].
//! 2. **Estimate the channel at each pilot.** The transmitted pilot is the
//!    BPSK signal of Table 3-17 (`W = 0 -> +4/3`, `W = 1 -> -4/3`) driven by
//!    the centre-segment PRBS of Table 3-16 (§3.13.1). The least-squares
//!    single-tap estimate is `H = received / normative`.
//! 3. **Interpolate the complex channel.** Between adjacent pilots the channel
//!    is linearly interpolated; outside the pilot span it is linearly
//!    extrapolated from the two nearest pilots, which is exact for a linear
//!    channel response (the standard does not prescribe an estimator).
//! 4. **Equalise and de-interleave.** Every data carrier is divided by its
//!    channel estimate, and the resulting `96 / 192 / 384` values — indexed by
//!    the segment-local *data-carrier number*, i.e. the rank in ascending
//!    physical order (Fig. 3-30) — are passed through
//!    [`crate::demod::control::frequency_deinterleave`] to restore the original
//!    logical data order (§3.11.2.2, Table 3-13).
//!
//! [`differential_symbol`] is the DQPSK counterpart: differential segments
//! carry no scattered pilots and no coherent channel estimate (§3.12.1,
//! Table 3-1), so the transmitted phase change of each data carrier (Table
//! 3-14) is recovered as `current * conj(previous)` normalised to unit
//! amplitude, which cancels a flat complex channel gain shared by two
//! consecutive symbols, and the `96 / 192 / 384` results are frequency
//! de-interleaved exactly as above.
//!
//! # Deliberately not implemented
//!
//! * Decision-directed refinement of the coherent channel estimate and any
//!   differential amplitude/phase tracking beyond the unit-amplitude
//!   normalisation.
//! * TMCC decoding. The caller must already know whether the segment is
//!   coherent or differential; no transmission parameters are read here.
//! * Time/frequency *interleaving*; the de-interleaver is the final stage.
//! * Real-signal validation. The behaviour is checked against synthetically
//!   constructed flat/linear channels and DQPSK phase steps, not a recorded
//!   signal.
//!
//! # Untrusted input
//!
//! [`equalize_symbol`] and [`differential_symbol`] validate the carrier count
//! and reject non-finite carriers (and non-finite normalised products) with an
//! error; they never panic on malformed input and never return a non-finite
//! sample.

use num_complex::Complex32;

use crate::dsp::Mode;

use super::coding::Modulation;
use super::control::{
    carriers_per_segment, data_carriers, data_carriers_per_segment, frequency_deinterleave,
    pilot_modulating_signal, scattered_pilot_carriers, ControlError, PilotPrbs,
};

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by [`equalize_symbol`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CarrierError {
    /// The input did not hold the mode's `108 / 216 / 432` carriers.
    #[error("carrier count mismatch: expected {expected}, got {got}")]
    CarrierCount { expected: usize, got: usize },
    /// A supplied carrier (or an equalised result) was NaN or infinite.
    #[error("non-finite carrier value at local carrier {carrier}")]
    NonFinite { carrier: usize },
    /// A pilot channel estimate was degenerate (magnitude below
    /// [`MIN_CHANNEL_MAGNITUDE`]), so dividing by it would be unstable.
    #[error("degenerate channel estimate at local carrier {carrier} (magnitude {magnitude})")]
    TinyEstimate { carrier: usize, magnitude: f32 },
    /// A lower-level control-plane primitive rejected the input.
    #[error(transparent)]
    Control(#[from] ControlError),
}

// ---------------------------------------------------------------------------
// Equalisation
// ---------------------------------------------------------------------------

/// Smallest channel magnitude accepted at any carrier.
///
/// Pilot estimates are formed from the normative `+-4/3` BPSK pilots (Table
/// 3-17). A magnitude this small can only come from a deep fade or a broken
/// input, and dividing by it would produce unstable (or non-finite) samples.
pub const MIN_CHANNEL_MAGNITUDE: f32 = 1e-6;

/// Equalise one coherent centre-segment OFDM symbol.
///
/// `carriers` must contain exactly the mode's `108 / 216 / 432` centre-segment
/// carriers in ascending segment-local carrier order, as delivered by
/// [`crate::demod::ofdm::extract_symbol`]. `symbol_index` is the OFDM-symbol
/// number within the frame and selects the scattered-pilot residue
/// `3 * (symbol_index mod 4) mod 12` (§3.12.2).
///
/// The returned vector holds the equalised *data* carriers only, in the
/// original logical data order after
/// [`crate::demod::control::frequency_deinterleave`], and therefore has
/// `96 / 192 / 384` entries.
///
/// # Errors
///
/// * [`CarrierError::CarrierCount`] — `carriers.len()` is not the mode size.
/// * [`CarrierError::NonFinite`] — any supplied carrier, or any sample produced
///   by the equaliser, is NaN or infinite.
/// * [`CarrierError::TinyEstimate`] — a pilot channel estimate (or the channel
///   interpolated from it) has a magnitude below [`MIN_CHANNEL_MAGNITUDE`].
/// * [`CarrierError::Control`] — a lower-level control primitive rejected the
///   input. This is defensive: with the carrier count already validated the
///   control helpers cannot fail for a genuine ISDB-T mode.
pub fn equalize_symbol(
    mode: Mode,
    symbol_index: usize,
    carriers: &[Complex32],
) -> Result<Vec<Complex32>, CarrierError> {
    let total = carriers_per_segment(mode);
    if carriers.len() != total {
        return Err(CarrierError::CarrierCount {
            expected: total,
            got: carriers.len(),
        });
    }

    // Never let a non-finite observed sample leak into the estimate or output.
    for (carrier, value) in carriers.iter().enumerate() {
        if !value.re.is_finite() || !value.im.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
    }

    // --- 1./2. Channel estimate at every scattered pilot -------------------
    let pilot_positions = scattered_pilot_carriers(mode, Modulation::Qpsk, symbol_index);
    let mut pilot_estimates = Vec::with_capacity(pilot_positions.len());
    for &carrier in &pilot_positions {
        let w = PilotPrbs::bit_at(mode, 0, carrier).ok_or(CarrierError::Control(
            ControlError::LocalCarrierOutOfRange {
                mode,
                local: carrier,
            },
        ))?;
        let (re, im) = pilot_modulating_signal(w)?;
        let estimate = carriers[carrier] / Complex32::new(re, im);
        let magnitude = estimate.norm();
        if !magnitude.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
        if magnitude < MIN_CHANNEL_MAGNITUDE {
            return Err(CarrierError::TinyEstimate { carrier, magnitude });
        }
        pilot_estimates.push(estimate);
    }

    // --- 3. Interpolate/extrapolate the channel over all carriers ----------
    let mut channel = vec![Complex32::new(0.0, 0.0); total];
    for (carrier, slot) in channel.iter_mut().enumerate() {
        let estimate = interpolate_channel(&pilot_positions, &pilot_estimates, carrier);
        let magnitude = estimate.norm();
        if !magnitude.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
        if magnitude < MIN_CHANNEL_MAGNITUDE {
            return Err(CarrierError::TinyEstimate { carrier, magnitude });
        }
        *slot = estimate;
    }

    // --- 4. Equalise the data carriers and restore the logical order -------
    let data_positions = data_carriers(mode, Modulation::Qpsk, symbol_index);
    let data_count = data_carriers_per_segment(mode);
    debug_assert_eq!(data_positions.len(), data_count);

    let mut by_data_index = Vec::with_capacity(data_count);
    for &carrier in &data_positions {
        let value = carriers[carrier] / channel[carrier];
        if !value.re.is_finite() || !value.im.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
        by_data_index.push(value);
    }

    let mut logical = vec![Complex32::new(0.0, 0.0); data_count];
    frequency_deinterleave(mode, &by_data_index, &mut logical)?;
    Ok(logical)
}

/// Complex linear interpolation/extrapolation from the two pilots bracketing
/// `carrier`, with the nearest pair used outside the pilot span.
///
/// `positions` and `estimates` are the ascending pilot carriers and their
/// channel estimates and must be non-empty.
fn interpolate_channel(positions: &[usize], estimates: &[Complex32], carrier: usize) -> Complex32 {
    let last = positions.len() - 1;
    if last == 0 {
        return estimates[0];
    }
    if carrier <= positions[0] {
        return lerp_two(
            positions[0],
            estimates[0],
            positions[1],
            estimates[1],
            carrier,
        );
    }
    if carrier >= positions[last] {
        return lerp_two(
            positions[last - 1],
            estimates[last - 1],
            positions[last],
            estimates[last],
            carrier,
        );
    }
    // `positions` is ascending and strictly inside the span, so the bracket is
    // `upper - 1 ..= upper` with `1 <= upper <= last`.
    let upper = positions.partition_point(|&position| position <= carrier);
    lerp_two(
        positions[upper - 1],
        estimates[upper - 1],
        positions[upper],
        estimates[upper],
        carrier,
    )
}

/// Linear interpolation of two complex points, evaluated at (possibly
/// out-of-range, i.e. extrapolated) carrier `at`.
fn lerp_two(
    start: usize,
    start_value: Complex32,
    end: usize,
    end_value: Complex32,
    at: usize,
) -> Complex32 {
    let t = (at as f32 - start as f32) / (end as f32 - start as f32);
    start_value + (end_value - start_value) * t
}

// ---------------------------------------------------------------------------
// Differential (DQPSK) demodulation
// ---------------------------------------------------------------------------

/// Differentially demodulate one DQPSK centre-segment OFDM symbol.
///
/// `current` is the symbol at frame position `j`; `previous` is the OFDM symbol
/// immediately preceding it in time (`j - 1`). Each must hold exactly the
/// mode's `108 / 216 / 432` centre-segment carriers in ascending segment-local
/// order, as delivered by [`crate::demod::ofdm::extract_symbol`].
///
/// For every *data* carrier of the differential layout (Table 3-14: every
/// carrier that is not the CP, an AC1/AC2 carrier or a TMCC carrier) the
/// `pi/4`-shift DQPSK phase change of Table 3-10 is recovered as
/// `current * conj(previous)` and normalised to unit amplitude. The
/// normalisation cancels any flat complex channel gain shared by the two
/// symbols; the phase of the gain cancels in the product itself. The resulting
/// `96 / 192 / 384` values — indexed by segment-local data-carrier number — are
/// passed through [`crate::demod::control::frequency_deinterleave`] to restore
/// the logical data order expected by
/// [`crate::demod::coding::demap`].
///
/// A product whose magnitude is exactly zero (either input carrier is zero) is
/// returned as zero rather than divided; the soft demapper treats that as an
/// erasure.
///
/// # Errors
///
/// * [`CarrierError::CarrierCount`] — `current` or `previous` does not hold the
///   mode's carrier count.
/// * [`CarrierError::NonFinite`] — a supplied carrier, or a normalised product,
///   is NaN or infinite.
/// * [`CarrierError::Control`] — the frequency de-interleaver rejected the
///   input. This is defensive: with the carrier count already validated the
///   control helper cannot fail for a genuine ISDB-T mode.
pub fn differential_symbol(
    mode: Mode,
    current: &[Complex32],
    previous: &[Complex32],
) -> Result<Vec<Complex32>, CarrierError> {
    let total = carriers_per_segment(mode);
    if current.len() != total {
        return Err(CarrierError::CarrierCount {
            expected: total,
            got: current.len(),
        });
    }
    if previous.len() != total {
        return Err(CarrierError::CarrierCount {
            expected: total,
            got: previous.len(),
        });
    }

    // Never let a non-finite observed sample leak into the normalised product.
    for (carrier, value) in current.iter().enumerate() {
        if !value.re.is_finite() || !value.im.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
    }
    for (carrier, value) in previous.iter().enumerate() {
        if !value.re.is_finite() || !value.im.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
    }

    // Differential segments use no scattered pilots, so the data-carrier set is
    // symbol-independent (Table 3-14); symbol 0 is representative.
    let data_positions = data_carriers(mode, Modulation::Dqpsk, 0);
    let data_count = data_carriers_per_segment(mode);
    debug_assert_eq!(data_positions.len(), data_count);

    let mut by_data_index = Vec::with_capacity(data_count);
    for &carrier in &data_positions {
        let product = current[carrier] * previous[carrier].conj();
        let magnitude = product.norm();
        if !magnitude.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
        let normalised = if magnitude > 0.0 {
            product / magnitude
        } else {
            Complex32::new(0.0, 0.0)
        };
        if !normalised.re.is_finite() || !normalised.im.is_finite() {
            return Err(CarrierError::NonFinite { carrier });
        }
        by_data_index.push(normalised);
    }

    let mut logical = vec![Complex32::new(0.0, 0.0); data_count];
    frequency_deinterleave(mode, &by_data_index, &mut logical)?;
    Ok(logical)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::demod::control::frequency_interleave;

    /// Deterministic pseudo-random complex values (no external RNG dependency).
    fn test_values(count: usize, seed: u32) -> Vec<Complex32> {
        let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        (0..count)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let re = ((state >> 8) & 0xFFFF) as f32 / 32768.0 - 1.0;
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let im = ((state >> 8) & 0xFFFF) as f32 / 32768.0 - 1.0;
                Complex32::new(re, im)
            })
            .collect()
    }

    /// Transmitter-side synthesis of one coherent symbol: normative pilots,
    /// `logical` data placed through the normative frequency interleaver, and a
    /// constant sentinel on the AC1/TMCC carriers. The result is multiplied by
    /// `channel(k)`.
    fn synthesize(
        mode: Mode,
        symbol: usize,
        logical: &[Complex32],
        channel: impl Fn(usize) -> Complex32,
    ) -> Vec<Complex32> {
        let total = carriers_per_segment(mode);
        let data_count = data_carriers_per_segment(mode);
        assert_eq!(logical.len(), data_count);

        // Logical data -> segment-local data-carrier order (Fig. 3-30).
        let mut physical_data = vec![Complex32::new(0.0, 0.0); data_count];
        frequency_interleave(mode, logical, &mut physical_data).unwrap();

        let mut transmitted = vec![Complex32::new(0.7, -0.3); total];
        for (rank, &carrier) in data_carriers(mode, Modulation::Qpsk, symbol)
            .iter()
            .enumerate()
        {
            transmitted[carrier] = physical_data[rank];
        }
        for &carrier in &scattered_pilot_carriers(mode, Modulation::Qpsk, symbol) {
            let w = PilotPrbs::bit_at(mode, 0, carrier).unwrap();
            let (re, im) = pilot_modulating_signal(w).unwrap();
            transmitted[carrier] = Complex32::new(re, im);
        }

        transmitted
            .iter()
            .enumerate()
            .map(|(carrier, &value)| channel(carrier) * value)
            .collect()
    }

    fn assert_close(actual: &[Complex32], expected: &[Complex32], tolerance: f32, context: &str) {
        assert_eq!(actual.len(), expected.len(), "{context}: length");
        for (index, (got, want)) in actual.iter().zip(expected.iter()).enumerate() {
            assert!(
                (got.re - want.re).abs() < tolerance && (got.im - want.im).abs() < tolerance,
                "{context}: data {index} got ({}, {}), want ({}, {})",
                got.re,
                got.im,
                want.re,
                want.im
            );
        }
    }

    #[test]
    fn equalize_has_the_expected_input_and_output_sizes() {
        for (mode, total, data) in [
            (Mode::Mode1, 108usize, 96usize),
            (Mode::Mode2, 216, 192),
            (Mode::Mode3, 432, 384),
        ] {
            let logical = test_values(data, 0x5555_0000 ^ total as u32);
            // A benign flat channel keeps every estimate well above the
            // degenerate threshold.
            let observed = synthesize(mode, 0, &logical, |_| Complex32::new(1.0, 0.0));
            assert_eq!(observed.len(), total, "{mode:?}");
            let out = equalize_symbol(mode, 0, &observed).unwrap();
            assert_eq!(out.len(), data, "{mode:?}");
        }
    }

    #[test]
    fn equalize_recovers_flat_complex_gain_all_modes_and_phases() {
        let phases: [f32; 6] = [
            0.0,
            std::f32::consts::FRAC_PI_4,
            std::f32::consts::FRAC_PI_2,
            std::f32::consts::PI,
            -std::f32::consts::FRAC_PI_2,
            -2.5,
        ];
        let magnitudes: [f32; 3] = [1.0, 0.5, 2.0];

        for (mode, seed) in [
            (Mode::Mode1, 0x1111_0001u32),
            (Mode::Mode2, 0x2222_0002),
            (Mode::Mode3, 0x3333_0003),
        ] {
            let count = data_carriers_per_segment(mode);
            let logical = test_values(count, seed);
            for symbol in [0usize, 1, 2, 3] {
                for &magnitude in &magnitudes {
                    for &phase in &phases {
                        let gain = Complex32::from_polar(magnitude, phase);
                        let observed = synthesize(mode, symbol, &logical, |_| gain);
                        let out = equalize_symbol(mode, symbol, &observed).unwrap();
                        assert_close(
                            &out,
                            &logical,
                            1e-4,
                            &format!("{mode:?} symbol {symbol} gain {magnitude}@{phase}"),
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn equalize_recovers_linear_complex_channel_response() {
        for mode in Mode::ALL {
            let count = data_carriers_per_segment(mode);
            let logical = test_values(count, 0x0F0F_1234);
            // H(k) = a + b * k, linear in the segment-local carrier index.
            let a = Complex32::new(0.8, -0.2);
            let b = Complex32::new(1.0e-3, 5.0e-4);
            for symbol in 0..4 {
                let observed = synthesize(mode, symbol, &logical, |carrier| a + b * carrier as f32);
                let out = equalize_symbol(mode, symbol, &observed).unwrap();
                assert_close(&out, &logical, 1e-3, &format!("{mode:?} symbol {symbol}"));
            }
        }
    }

    #[test]
    fn equalize_rejects_invalid_carrier_counts() {
        for mode in Mode::ALL {
            let total = carriers_per_segment(mode);
            let short = vec![Complex32::new(1.0, 0.0); total - 1];
            assert_eq!(
                equalize_symbol(mode, 0, &short),
                Err(CarrierError::CarrierCount {
                    expected: total,
                    got: total - 1,
                })
            );
            let long = vec![Complex32::new(1.0, 0.0); total + 1];
            assert!(matches!(
                equalize_symbol(mode, 0, &long),
                Err(CarrierError::CarrierCount { .. })
            ));
        }
    }

    #[test]
    fn equalize_rejects_non_finite_carriers() {
        let mode = Mode::Mode1;
        let good = synthesize(mode, 0, &test_values(96, 7), |_| Complex32::new(1.0, 0.0));

        for &carrier in &[0usize, 17, 107] {
            let mut bad = good.clone();
            bad[carrier] = Complex32::new(f32::NAN, 0.0);
            assert_eq!(
                equalize_symbol(mode, 0, &bad),
                Err(CarrierError::NonFinite { carrier })
            );

            bad[carrier] = Complex32::new(0.0, f32::INFINITY);
            assert_eq!(
                equalize_symbol(mode, 0, &bad),
                Err(CarrierError::NonFinite { carrier })
            );
        }
    }

    #[test]
    fn equalize_rejects_deep_fades_at_the_pilots() {
        let mode = Mode::Mode3;
        let logical = test_values(data_carriers_per_segment(mode), 9);
        let good = synthesize(mode, 2, &logical, |_| Complex32::new(1.0, 0.0));
        let pilots = scattered_pilot_carriers(mode, Modulation::Qpsk, 2);
        assert!(!pilots.is_empty());

        let mut faded = good;
        let pilot = pilots[0];
        // Zero the observed pilot: the estimated channel is ~0 and must be
        // rejected rather than inverted.
        faded[pilot] = Complex32::new(0.0, 0.0);
        match equalize_symbol(mode, 2, &faded) {
            Err(CarrierError::TinyEstimate { carrier, magnitude }) => {
                assert_eq!(carrier, pilot);
                assert_eq!(magnitude, 0.0);
            }
            other => panic!("expected a tiny-estimate rejection, got {other:?}"),
        }

        // A tiny but non-zero pilot is rejected too.
        let mut tiny = synthesize(mode, 2, &logical, |_| Complex32::new(1.0, 0.0));
        tiny[pilot] = Complex32::new(1e-12, 0.0);
        assert!(matches!(
            equalize_symbol(mode, 2, &tiny),
            Err(CarrierError::TinyEstimate { carrier, .. }) if carrier == pilot
        ));
    }

    // ---- differential (DQPSK) demodulation --------------------------------

    /// The four Table 3-10 phase changes, in the order `00, 01, 10, 11`.
    const TABLE_3_10_PHASES: [f32; 4] = [
        std::f32::consts::FRAC_PI_4,
        -std::f32::consts::FRAC_PI_4,
        3.0 * std::f32::consts::FRAC_PI_4,
        -3.0 * std::f32::consts::FRAC_PI_4,
    ];

    #[test]
    fn differential_symbol_has_normative_data_count() {
        for (mode, data) in [
            (Mode::Mode1, 96usize),
            (Mode::Mode2, 192),
            (Mode::Mode3, 384),
        ] {
            let total = carriers_per_segment(mode);
            let current: Vec<Complex32> = (0..total)
                .map(|carrier| Complex32::new(1.0, 0.25 * carrier as f32))
                .collect();
            let previous: Vec<Complex32> = (0..total)
                .map(|carrier| Complex32::new(0.5, -0.125 * carrier as f32))
                .collect();
            let out = differential_symbol(mode, &current, &previous).unwrap();
            assert_eq!(out.len(), data, "{mode:?}");
            assert!(out
                .iter()
                .all(|value| value.re.is_finite() && value.im.is_finite()));
        }
    }

    #[test]
    fn differential_symbol_recovers_table_3_10_phase_steps_under_flat_gain() {
        // Non-trivial nonzero flat gains: a pure real gain, an imaginary gain
        // and a gain with magnitude above one. `current * conj(previous)` must
        // cancel all of them, because both symbols carry the same gain.
        let gains = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 0.75),
            Complex32::new(-2.0, 1.0),
        ];

        for mode in Mode::ALL {
            let total = carriers_per_segment(mode);
            let data_count = data_carriers_per_segment(mode);
            let positions = data_carriers(mode, Modulation::Dqpsk, 0);
            assert_eq!(positions.len(), data_count, "{mode:?} Table 3-14 data set");

            // One of the four normative phase changes per logical data carrier.
            let logical: Vec<Complex32> = (0..data_count)
                .map(|index| Complex32::from_polar(1.0, TABLE_3_10_PHASES[index % 4]))
                .collect();
            let mut physical = vec![Complex32::new(0.0, 0.0); data_count];
            frequency_interleave(mode, &logical, &mut physical).unwrap();

            for &gain in &gains {
                let mut current = vec![Complex32::new(0.0, 0.0); total];
                let mut previous = vec![Complex32::new(0.0, 0.0); total];
                for (rank, &carrier) in positions.iter().enumerate() {
                    // A per-carrier reference with varying magnitude and phase:
                    // the differential product removes it, leaving the phase
                    // change. The flat channel `gain` is common to both symbols.
                    let reference =
                        Complex32::from_polar(0.5 + (carrier % 5) as f32, carrier as f32 * 0.3)
                            * gain;
                    previous[carrier] = reference;
                    current[carrier] = physical[rank] * reference;
                }
                let out = differential_symbol(mode, &current, &previous).unwrap();
                assert_close(&out, &logical, 1e-5, &format!("{mode:?} gain {gain:?}"));
            }
        }
    }

    #[test]
    fn differential_symbol_validates_inputs_and_handles_erasure() {
        let mode = Mode::Mode1;
        let total = carriers_per_segment(mode);
        let good = vec![Complex32::new(1.0, 0.5); total];

        assert_eq!(
            differential_symbol(mode, &good[..total - 1], &good),
            Err(CarrierError::CarrierCount {
                expected: total,
                got: total - 1,
            })
        );
        assert_eq!(
            differential_symbol(mode, &good, &good[..total - 1]),
            Err(CarrierError::CarrierCount {
                expected: total,
                got: total - 1,
            })
        );

        let mut nan = good.clone();
        nan[17] = Complex32::new(f32::NAN, 0.0);
        assert_eq!(
            differential_symbol(mode, &nan, &good),
            Err(CarrierError::NonFinite { carrier: 17 })
        );

        // A zero reference is an erasure, not a division by zero.
        let zero = vec![Complex32::new(0.0, 0.0); total];
        let out = differential_symbol(mode, &good, &zero).unwrap();
        assert!(out.iter().all(|value| *value == Complex32::new(0.0, 0.0)));
    }
}
