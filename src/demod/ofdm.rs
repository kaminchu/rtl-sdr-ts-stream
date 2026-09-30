//! OFDM symbol extraction for the ISDB-T centre (partial-reception) segment.
//!
//! This is an original implementation written from scratch for this crate. The
//! normative basis is ARIB STD-B31 version 2.2-E1 §3.14 (transmission spectrum
//! configuration and guard-interval insertion):
//!
//! * §3.14.1 defines the subcarrier spacing `1/Tu`, the carrier numbering `k`
//!   (0 at the lowest frequency) and the centre carrier `Kc`
//!   (`702 / 1404 / 2808` for Modes 1/2/3).
//! * §3.14.2 and Fig. 3-33 define the guard interval as an unmodified copy of
//!   the *latter* part of the IFFT output, prepended to the effective symbol.
//!
//! A one-segment receiver digitises at `fs = 64/63 MHz = 1.015873... MHz`
//! (Table 3-1 gives the full-band FFT sampling frequency `2048/252 MHz`; the
//! segment rate is that divided by eight). The effective symbol therefore has
//! [`segment_fft_size`] samples (`256 / 512 / 1024`) and the guard interval has
//! `fft_size / (4, 8, 16, 32)` samples via [`GuardInterval`].
//!
//! [`extract_symbol`] takes exactly one complete `guard + useful` symbol,
//! removes the cyclic prefix, de-rotates a carrier-frequency offset (CFO) using
//! the useful-part sample index, applies a unitary forward DFT and returns the
//! `108 / 216 / 432` centre-segment carriers in ascending carrier order.
//!
//! # DFT normalisation
//!
//! The transmitter is assumed to use the unitary IFFT
//!
//! ```text
//! x[n] = (1 / sqrt(N)) * sum_k X[k] * exp(+j 2*pi*k*n / N),
//! ```
//!
//! so the receiver applies the matching unitary forward DFT
//! (`X[k] = (1 / sqrt(N)) * sum_n x[n] * exp(-j 2*pi*k*n / N)`).
//! `rustfft` is unnormalised, so the result is scaled by `1 / sqrt(N)` after the
//! transform. Round-tripping the transmitter and receiver is then the identity.
//!
//! # Centre-segment carrier mapping
//!
//! Absolute carrier `k` maps to baseband bin `k - Kc`, and the centre segment
//! (segment No. 0) occupies absolute carriers `6*C .. 6*C + C - 1` with
//! `C = carriers_per_segment(mode)` and `Kc = 6.5*C`. Segment-local carrier `l`
//! therefore sits at bin `l - C/2`. [`extract_symbol`] returns
//! `fft[(l + N - C/2) mod N]` for `l = 0 .. C - 1`.
//!
//! # Full spectrum
//!
//! [`full_spectrum`] is the same guard removal, CFO de-rotation and unitary DFT
//! as [`extract_symbol`] but returns **all** `N` baseband bins instead of just
//! the centre segment. [`extract_symbol`] is a thin wrapper around it and
//! [`centre_carriers`]. This lets a caller read the centre carriers *and* hand
//! the full signed-bin spectrum to the scattered-pilot integer-CFO estimator
//! ([`crate::demod::coarse::estimate_integer_cfo`]) without transforming the
//! same symbol twice.
//!
//! # Not implemented
//!
//! Symbol/frame synchronisation, mode and guard estimation, channel estimation,
//! equalisation and any `IQ -> TS` pipeline are deliberately *not* part of this
//! module. [`extract_symbol`] documents exactly what it does with the samples it
//! is given and never invents carrier values for malformed input.

use std::sync::OnceLock;

use num_complex::Complex32;

use super::control::carriers_per_segment;
use crate::dsp::{Fft, GuardInterval, Mode};

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by [`extract_symbol`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum OfdmError {
    /// The input did not hold exactly one complete `guard + useful` symbol.
    #[error(
        "OFDM symbol length mismatch: expected {expected} samples (guard {guard} + FFT {fft_size}), got {got}"
    )]
    SymbolLength {
        /// `guard_len + fft_size` for the requested mode/guard.
        expected: usize,
        /// Number of samples actually supplied.
        got: usize,
        /// Guard length used for the expected symbol.
        guard: usize,
        /// Segment FFT size used for the expected symbol.
        fft_size: usize,
    },
    /// The carrier-frequency offset was NaN or infinite.
    #[error("carrier-frequency offset must be finite, got {0}")]
    NonFiniteCfo(f32),
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

/// Effective-symbol (FFT) length of one OFDM segment, in samples.
///
/// This is the full-band FFT size divided by eight: `256 / 512 / 1024` for
/// Modes 1/2/3. At the one-segment rate `fs = 64/63 MHz` this equals the
/// `252 / 504 / 1008 us` effective symbol length of ARIB STD-B31 Table 3-1.
pub fn segment_fft_size(mode: Mode) -> usize {
    mode.fft_size() / 8
}

/// Number of samples in one complete `guard + useful` symbol.
pub fn segment_symbol_len(mode: Mode, guard: GuardInterval) -> usize {
    let fft_size = segment_fft_size(mode);
    fft_size + guard.guard_len(fft_size)
}

/// Cached forward FFT for `mode`.
///
/// The one-segment FFT sizes are fixed and always valid, so planning them lazily
/// once avoids re-planning on every symbol while keeping [`extract_symbol`] a
/// plain free function.
fn fft_for(mode: Mode) -> &'static Fft {
    static CACHE: [OnceLock<Fft>; 3] = [OnceLock::new(), OnceLock::new(), OnceLock::new()];
    let slot = match mode {
        Mode::Mode1 => &CACHE[0],
        Mode::Mode2 => &CACHE[1],
        Mode::Mode3 => &CACHE[2],
    };
    slot.get_or_init(|| {
        Fft::new(segment_fft_size(mode))
            .expect("ISDB-T one-segment FFT sizes (256/512/1024) are always valid")
    })
}

/// Compute the unitary full forward DFT of exactly one OFDM symbol.
///
/// `samples` must contain one complete symbol at `fs = 64/63 MHz`, i.e. exactly
/// [`segment_symbol_len`] samples with the guard interval first. The cyclic
/// prefix (the first `guard_len` samples) is discarded, the useful part is
/// de-rotated by `exp(-j * cfo_radians_per_sample * n)` with the useful-part
/// index `n = 0 .. fft_size - 1`, transformed with the unitary forward DFT and
/// scaled by `1 / sqrt(fft_size)`.
///
/// The returned vector holds all [`segment_fft_size`] baseband bins in
/// `rustfft`'s natural order (`0 .. N-1`). This is the exact signed-bin spectrum
/// from which [`extract_symbol`] reads the centre segment and which the
/// scattered-pilot integer-CFO estimator
/// ([`crate::demod::coarse::estimate_integer_cfo`]) expects, so a caller that
/// needs both does not transform the symbol twice.
///
/// # Errors
///
/// * [`OfdmError::NonFiniteCfo`] when `cfo_radians_per_sample` is NaN or
///   infinite.
/// * [`OfdmError::SymbolLength`] when `samples.len()` is not exactly
///   `guard_len + fft_size`. Nothing is computed for a malformed length.
pub fn full_spectrum(
    samples: &[Complex32],
    mode: Mode,
    guard: GuardInterval,
    cfo_radians_per_sample: f32,
) -> Result<Vec<Complex32>, OfdmError> {
    if !cfo_radians_per_sample.is_finite() {
        return Err(OfdmError::NonFiniteCfo(cfo_radians_per_sample));
    }

    let fft_size = segment_fft_size(mode);
    let guard_len = guard.guard_len(fft_size);
    let expected = fft_size + guard_len;
    if samples.len() != expected {
        return Err(OfdmError::SymbolLength {
            expected,
            got: samples.len(),
            guard: guard_len,
            fft_size,
        });
    }

    // Discard the cyclic prefix and de-rotate the CFO. The phase reference is
    // the first useful sample, so the rotated index is the local useful index
    // `n` (not the capture-relative index that includes the guard interval).
    let mut buffer = vec![Complex32::new(0.0, 0.0); fft_size];
    for (n, dst) in buffer.iter_mut().enumerate() {
        let angle = -cfo_radians_per_sample * n as f32;
        let (sin, cos) = angle.sin_cos();
        let sample = samples[guard_len + n];
        *dst = Complex32::new(
            sample.re * cos - sample.im * sin,
            sample.re * sin + sample.im * cos,
        );
    }

    // Unitary forward DFT: rustfft is unnormalised, so scale by 1/sqrt(N).
    let mut spectrum = vec![Complex32::new(0.0, 0.0); fft_size];
    fft_for(mode)
        .forward(&buffer, &mut spectrum)
        .expect("buffer and spectrum both have the cached FFT length");
    let scale = 1.0 / (fft_size as f32).sqrt();
    for bin in &mut spectrum {
        *bin *= scale;
    }
    Ok(spectrum)
}

/// Demap the centre-segment carriers out of a full unitary `spectrum`.
///
/// `spectrum` must be the output of [`full_spectrum`] for `mode` (length
/// [`segment_fft_size`]). The centre segment lies at baseband bins
/// `l - C/2` with `C = carriers_per_segment(mode)`, so the returned vector holds
/// the `108 / 216 / 432` carriers in ascending segment-local carrier order:
/// `out[l] = spectrum[(l + N - C/2) mod N]`.
pub(crate) fn centre_carriers(spectrum: &[Complex32], mode: Mode) -> Vec<Complex32> {
    let fft_size = segment_fft_size(mode);
    debug_assert_eq!(spectrum.len(), fft_size);
    let carriers = carriers_per_segment(mode);
    let half = carriers / 2;
    let mut out = Vec::with_capacity(carriers);
    for local in 0..carriers {
        let bin = (local + fft_size - half) % fft_size;
        out.push(spectrum[bin]);
    }
    out
}

/// Extract the centre-segment carriers of exactly one OFDM symbol.
///
/// This is [`full_spectrum`] followed by [`centre_carriers`]: `samples` must
/// contain one complete symbol at `fs = 64/63 MHz` (guard interval first), and
/// the returned vector holds the `108 / 216 / 432` centre-segment carriers in
/// ascending segment-local carrier order.
///
/// # Errors
///
/// * [`OfdmError::NonFiniteCfo`] when `cfo_radians_per_sample` is NaN or
///   infinite.
/// * [`OfdmError::SymbolLength`] when `samples.len()` is not exactly
///   `guard_len + fft_size`. Nothing is computed for a malformed length.
pub fn extract_symbol(
    samples: &[Complex32],
    mode: Mode,
    guard: GuardInterval,
    cfo_radians_per_sample: f32,
) -> Result<Vec<Complex32>, OfdmError> {
    let spectrum = full_spectrum(samples, mode, guard, cfo_radians_per_sample)?;
    Ok(centre_carriers(&spectrum, mode))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use rustfft::FftPlanner;

    /// Deterministic pseudo-random complex values (no external RNG dependency).
    fn test_carriers(count: usize, seed: u32) -> Vec<Complex32> {
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

    #[test]
    fn segment_sizes_match_isdb_t() {
        assert_eq!(segment_fft_size(Mode::Mode1), 256);
        assert_eq!(segment_fft_size(Mode::Mode2), 512);
        assert_eq!(segment_fft_size(Mode::Mode3), 1024);
        assert_eq!(carriers_per_segment(Mode::Mode1), 108);
        assert_eq!(carriers_per_segment(Mode::Mode2), 216);
        assert_eq!(carriers_per_segment(Mode::Mode3), 432);

        // Guard lengths and full symbol lengths per Table 3-1.
        let m1 = Mode::Mode1;
        assert_eq!(segment_symbol_len(m1, GuardInterval::OneQuarter), 320);
        assert_eq!(segment_symbol_len(m1, GuardInterval::OneEighth), 288);
        assert_eq!(segment_symbol_len(m1, GuardInterval::OneSixteenth), 272);
        assert_eq!(segment_symbol_len(m1, GuardInterval::OneThirtySecond), 264);
    }

    #[test]
    fn extract_symbol_inverts_synthetic_transmitter_all_modes_and_guards() {
        for mode in Mode::ALL {
            for guard in GuardInterval::ALL {
                let fft_size = segment_fft_size(mode);
                let guard_len = guard.guard_len(fft_size);
                let carriers = carriers_per_segment(mode);
                let expected = test_carriers(carriers, 0x51ED_0000 ^ (guard_len as u32) << 8);

                // Place the centre segment on the transmitter spectrum.
                let mut spectrum = vec![Complex32::new(0.0, 0.0); fft_size];
                for (local, value) in expected.iter().enumerate() {
                    spectrum[centre_bin(fft_size, carriers, local)] = *value;
                }
                let useful = unitary_ifft(&spectrum);

                // Build the full symbol (guard first) and apply a capture-relative
                // CFO over the whole symbol, starting at capture index 0.
                let mut symbol = Vec::with_capacity(fft_size + guard_len);
                symbol.extend_from_slice(&useful[fft_size - guard_len..]);
                symbol.extend_from_slice(&useful);
                for cfo in [0.0f32, 0.01, -0.02] {
                    let mut received = symbol.clone();
                    for (m, value) in received.iter_mut().enumerate() {
                        let angle = cfo * m as f32;
                        let (sin, cos) = angle.sin_cos();
                        *value = Complex32::new(
                            value.re * cos - value.im * sin,
                            value.re * sin + value.im * cos,
                        );
                    }

                    let got = extract_symbol(&received, mode, guard, cfo).unwrap();
                    assert_eq!(got.len(), carriers);

                    // De-rotating with the useful-part index leaves a constant
                    // phase exp(+j*cfo*guard_len) on every carrier.
                    let residual = cfo * guard_len as f32;
                    let (sin, cos) = residual.sin_cos();
                    for (local, (actual, base)) in got.iter().zip(expected.iter()).enumerate() {
                        let want_re = base.re * cos - base.im * sin;
                        let want_im = base.re * sin + base.im * cos;
                        assert!(
                            (actual.re - want_re).abs() < 1e-3
                                && (actual.im - want_im).abs() < 1e-3,
                            "{mode:?}/{guard:?} cfo={cfo} carrier {local}: \
                             got ({}, {}), want ({want_re}, {want_im})",
                            actual.re,
                            actual.im
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn extract_symbol_reads_carrier_order_with_unitary_normalisation() {
        // An independent check that does not use the inverse FFT: build the
        // useful symbol directly from a unit-amplitude complex exponential with
        // the unitary transmitter scaling A/sqrt(N), then verify the matching
        // output carrier is 1 and all others are ~0.
        for mode in Mode::ALL {
            for guard in GuardInterval::ALL {
                let fft_size = segment_fft_size(mode);
                let guard_len = guard.guard_len(fft_size);
                let carriers = carriers_per_segment(mode);
                let half = carriers / 2;
                let amplitude = 1.0 / (fft_size as f32).sqrt();

                for &local in &[0usize, half - 1, half, carriers - 1] {
                    let signed_bin = local as isize - half as isize;
                    let mut useful = Vec::with_capacity(fft_size);
                    for n in 0..fft_size {
                        let phase = 2.0 * std::f32::consts::PI * signed_bin as f32 * n as f32
                            / fft_size as f32;
                        let (sin, cos) = phase.sin_cos();
                        useful.push(Complex32::new(amplitude * cos, amplitude * sin));
                    }
                    let mut symbol = Vec::with_capacity(fft_size + guard_len);
                    symbol.extend_from_slice(&useful[fft_size - guard_len..]);
                    symbol.extend_from_slice(&useful);

                    let got = extract_symbol(&symbol, mode, guard, 0.0).unwrap();
                    assert_eq!(got.len(), carriers);
                    for (index, value) in got.iter().enumerate() {
                        if index == local {
                            assert!(
                                (value.re - 1.0).abs() < 1e-3 && value.im.abs() < 1e-3,
                                "{mode:?}/{guard:?} local {local}: want (1, 0), got ({}, {})",
                                value.re,
                                value.im
                            );
                        } else {
                            assert!(
                                value.re.abs() < 1e-3 && value.im.abs() < 1e-3,
                                "{mode:?}/{guard:?} local {local}: leakage at {index} = ({}, {})",
                                value.re,
                                value.im
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn extract_symbol_rejects_bad_lengths_and_non_finite_cfo() {
        let mode = Mode::Mode1;
        let guard = GuardInterval::OneEighth;
        let fft_size = segment_fft_size(mode);
        let guard_len = guard.guard_len(fft_size);
        let symbol_len = fft_size + guard_len;

        let short = vec![Complex32::new(0.0, 0.0); symbol_len - 1];
        let long = vec![Complex32::new(0.0, 0.0); symbol_len + 1];
        assert_eq!(
            extract_symbol(&short, mode, guard, 0.0),
            Err(OfdmError::SymbolLength {
                expected: symbol_len,
                got: symbol_len - 1,
                guard: guard_len,
                fft_size,
            })
        );
        assert!(extract_symbol(&long, mode, guard, 0.0).is_err());

        let symbol = vec![Complex32::new(0.0, 0.0); symbol_len];
        assert!(matches!(
            extract_symbol(&symbol, mode, guard, f32::NAN),
            Err(OfdmError::NonFiniteCfo(_))
        ));
        assert!(matches!(
            extract_symbol(&symbol, mode, guard, f32::INFINITY),
            Err(OfdmError::NonFiniteCfo(_))
        ));
        assert!(matches!(
            extract_symbol(&symbol, mode, guard, f32::NEG_INFINITY),
            Err(OfdmError::NonFiniteCfo(_))
        ));
    }

    #[test]
    fn full_spectrum_is_the_de_rotated_unitary_dft_and_slices_to_carriers() {
        // Independent reference: de-rotate the useful samples with the local
        // index, run an unnormalised rustfft forward transform and scale by
        // 1/sqrt(N). `full_spectrum` must match bin for bin, and its centre
        // slice must equal `extract_symbol`.
        let mut planner = FftPlanner::<f32>::new();
        for mode in Mode::ALL {
            let fft_size = segment_fft_size(mode);
            let carriers = carriers_per_segment(mode);
            let fft = planner.plan_fft_forward(fft_size);
            for guard in GuardInterval::ALL {
                let guard_len = guard.guard_len(fft_size);
                let expected = test_carriers(carriers, 0xF011_0000 ^ fft_size as u32);
                let mut spectrum = vec![Complex32::new(0.0, 0.0); fft_size];
                for (local, value) in expected.iter().enumerate() {
                    spectrum[centre_bin(fft_size, carriers, local)] = *value;
                }
                let useful = unitary_ifft(&spectrum);
                let mut symbol = Vec::with_capacity(fft_size + guard_len);
                symbol.extend_from_slice(&useful[fft_size - guard_len..]);
                symbol.extend_from_slice(&useful);

                for cfo in [0.0f32, 0.017, -0.031] {
                    let full = full_spectrum(&symbol, mode, guard, cfo).unwrap();
                    assert_eq!(full.len(), fft_size);

                    let mut reference: Vec<Complex32> = (0..fft_size)
                        .map(|n| {
                            let angle = -cfo * n as f32;
                            let (sin, cos) = angle.sin_cos();
                            let sample = symbol[guard_len + n];
                            Complex32::new(
                                sample.re * cos - sample.im * sin,
                                sample.re * sin + sample.im * cos,
                            )
                        })
                        .collect();
                    fft.process(&mut reference);
                    let scale = 1.0 / (fft_size as f32).sqrt();
                    for value in &mut reference {
                        *value *= scale;
                    }
                    for (bin, (got, want)) in full.iter().zip(reference.iter()).enumerate() {
                        assert!(
                            (got.re - want.re).abs() < 1e-4 && (got.im - want.im).abs() < 1e-4,
                            "{mode:?}/{guard:?} cfo {cfo} bin {bin}: got ({}, {}), want ({}, {})",
                            got.re,
                            got.im,
                            want.re,
                            want.im
                        );
                    }

                    assert_eq!(
                        centre_carriers(&full, mode),
                        extract_symbol(&symbol, mode, guard, cfo).unwrap()
                    );
                }
            }
        }
    }
}
