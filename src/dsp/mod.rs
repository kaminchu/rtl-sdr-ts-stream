//! Original, reusable front-end DSP primitives.
//!
//! Everything here is written from scratch for this crate. No GNU Radio,
//! `gr-isdb`, Python, or other existing demodulator source was consulted or
//! copied. The only external DSP dependency is [`rustfft`], used for the FFT as
//! permitted by the project plan ("FFT ... reuse a mature library").
//!
//! # What is implemented
//!
//! * [`normalize_u8_iq`] / [`normalize_u8_iq_into`]: convert librtlsdr's
//!   interleaved unsigned 8-bit IQ stream into normalised [`Complex32`] in
//!   `[-1, 1]`. Runtime-dispatched SIMD (AVX2 / NEON) with a scalar fallback.
//! * [`DcBlocker`]: a one-pole streaming DC tracker/remover.
//! * [`RationalResampler`]: a streaming polyphase FIR rational resampler with
//!   exact chunk-boundary continuity (splitting the input into arbitrary chunks
//!   produces bit-identical output to one contiguous call).
//! * [`Mode`], [`GuardInterval`], [`OfdmParameters`]: ISDB-T Mode / guard
//!   interval parameter representation and derived sizes.
//! * [`CpCorrelator`]: cyclic-prefix correlation metric for a fixed
//!   `(fft_size, guard_len)` and a best-offset search helper.
//! * [`Fft`]: a thin validated wrapper over `rustfft`.
//!
//! # Explicitly **not** implemented / not integrated
//!
//! These are deliberately out of scope for this module. Nothing here claims a
//! working receiver, and none of the following is stubbed as if it worked:
//!
//! * TMCC decoding (hierarchical modulation / coding / time-interleaving
//!   parameters).
//! * Channel estimation and equalisation.
//! * Frequency, time, symbol or bit de-interleaving.
//! * Puncturing / Viterbi / Reed-Solomon / TS framing (see [`crate::fec`] for
//!   the parts that *are* implemented, which are still not wired together).
//! * Any `IQ -> TS` receiver or full demodulation pipeline. The production
//!   backend still reports "decoder not implemented".
//! * Mode / guard-interval *estimation* from a live signal. The parameter types
//!   exist; choosing them from samples is not implemented.
//!
//! # Verification status
//!
//! All primitives are unit-tested against independently constructed expected
//! values and against each other (scalar vs SIMD, chunked vs contiguous,
//! polyphase vs a naive `f64` reference). No real IQ recording, over-the-air
//! signal, or hardware run was used, and **no performance measurement was
//! performed**. SIMD is present but "not measured"; it is validated only for
//! numerical equivalence with the scalar path.
//!
//! # Untrusted input
//!
//! Public entry points validate lengths and the numeric arguments that must be
//! finite (for example the DC-blocker pole), and never panic on malformed
//! slices. Internal invariants (e.g. the polyphase tap index) are derived from
//! validated constructor arguments.

use std::fmt;
use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft as RustFft, FftPlanner};

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by the front-end DSP primitives.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DspError {
    /// An interleaved IQ byte slice had an odd length.
    #[error("IQ byte slice has odd length {0}; interleaved I/Q needs an even byte count")]
    OddIqByteCount(usize),
    /// The caller-provided output slice was too small.
    #[error("output slice too small: need {needed} complex samples, got {got}")]
    OutputTooSmall { needed: usize, got: usize },
    /// A structurally invalid argument (zero length, out of range, ...).
    #[error("invalid argument: {0}")]
    InvalidArgument(String),
    /// A value that must be finite was NaN or infinite.
    #[error("non-finite value in {0}")]
    NonFinite(&'static str),
    /// A buffer length did not match what the operation requires.
    #[error("length mismatch: expected {expected}, got {got} in {what}")]
    LengthMismatch {
        what: &'static str,
        expected: usize,
        got: usize,
    },
}

// ---------------------------------------------------------------------------
// 1. u8 IQ normalisation
// ---------------------------------------------------------------------------

/// Offset removed from each unsigned byte (RTL-SDR 8-bit samples are centred
/// at 127.5).
pub const IQ_BIAS: f32 = 127.5;

/// Scale applied after the bias so the result lands in `[-1, 1]`.
pub const IQ_SCALE: f32 = 1.0 / 127.5;

/// Largest FFT/parameter size accepted by constructors, to keep allocations
/// bounded for untrusted arguments.
const MAX_FFT_SIZE: usize = 1 << 22;

/// Normalise an interleaved `I, Q, I, Q, ...` unsigned 8-bit IQ slice.
///
/// The mapping is `(byte - 127.5) / 127.5`, so `0 -> -1.0` and `255 -> 1.0`.
///
/// # Errors
///
/// Returns [`DspError::OddIqByteCount`] when `bytes.len()` is odd.
pub fn normalize_u8_iq(bytes: &[u8]) -> Result<Vec<Complex32>, DspError> {
    if bytes.len() % 2 != 0 {
        return Err(DspError::OddIqByteCount(bytes.len()));
    }
    let mut out = vec![Complex32::new(0.0, 0.0); bytes.len() / 2];
    normalize_u8_iq_into(bytes, &mut out)?;
    Ok(out)
}

/// Normalise an interleaved unsigned 8-bit IQ slice into a caller-owned buffer.
///
/// `out` must have room for `bytes.len() / 2` complex samples; any extra room
/// is left untouched.
///
/// # Errors
///
/// * [`DspError::OddIqByteCount`] when `bytes.len()` is odd.
/// * [`DspError::OutputTooSmall`] when `out` is shorter than `bytes.len() / 2`.
pub fn normalize_u8_iq_into(bytes: &[u8], out: &mut [Complex32]) -> Result<(), DspError> {
    if bytes.len() % 2 != 0 {
        return Err(DspError::OddIqByteCount(bytes.len()));
    }
    let needed = bytes.len() / 2;
    if out.len() < needed {
        return Err(DspError::OutputTooSmall {
            needed,
            got: out.len(),
        });
    }
    if bytes.is_empty() {
        return Ok(());
    }

    #[cfg(target_arch = "x86_64")]
    {
        if std::arch::is_x86_feature_detected!("avx2") {
            // SAFETY: the feature was detected above; `normalize_avx2` only
            // reads `bytes` and writes the first `bytes.len() / 2` entries of
            // `out`, both validated above.
            unsafe { normalize_avx2(bytes, out) };
            return Ok(());
        }
    }

    #[cfg(target_arch = "aarch64")]
    {
        if std::arch::is_aarch64_feature_detected!("neon") {
            // SAFETY: NEON is present; bounds are the same as the scalar path.
            unsafe { normalize_neon(bytes, out) };
            return Ok(());
        }
    }

    normalize_scalar(bytes, out);
    Ok(())
}

/// Scalar normalisation used as the reference and fallback implementation.
fn normalize_scalar(bytes: &[u8], out: &mut [Complex32]) {
    for (slot, pair) in out.iter_mut().zip(bytes.chunks_exact(2)) {
        let re = (pair[0] as f32 - IQ_BIAS) * IQ_SCALE;
        let im = (pair[1] as f32 - IQ_BIAS) * IQ_SCALE;
        *slot = Complex32::new(re, im);
    }
}

/// AVX2 normalisation. Requires AVX2; callers must check the feature.
///
/// # Safety
///
/// `bytes.len()` must be even and `out.len() >= bytes.len() / 2`.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn normalize_avx2(bytes: &[u8], out: &mut [Complex32]) {
    use std::arch::x86_64::*;

    let bias = _mm256_set1_ps(IQ_BIAS);
    let scale = _mm256_set1_ps(IQ_SCALE);
    let pairs = bytes.len() / 2;

    let mut byte = 0usize;
    let mut index = 0usize;
    while byte + 16 <= bytes.len() {
        // One 16-byte load -> 16 f32 -> 8 complex outputs.
        let raw = _mm_loadu_si128(bytes.as_ptr().add(byte) as *const __m128i);
        let lo_i32 = _mm256_cvtepu8_epi32(raw);
        let hi_i32 = _mm256_cvtepu8_epi32(_mm_srli_si128(raw, 8));
        let lo = _mm256_mul_ps(_mm256_sub_ps(_mm256_cvtepi32_ps(lo_i32), bias), scale);
        let hi = _mm256_mul_ps(_mm256_sub_ps(_mm256_cvtepi32_ps(hi_i32), bias), scale);

        let mut values = [0f32; 16];
        _mm256_storeu_ps(values.as_mut_ptr(), lo);
        _mm256_storeu_ps(values.as_mut_ptr().add(8), hi);

        for c in 0..8 {
            out[index + c] = Complex32::new(values[2 * c], values[2 * c + 1]);
        }
        index += 8;
        byte += 16;
    }

    while index < pairs {
        let re = (bytes[byte] as f32 - IQ_BIAS) * IQ_SCALE;
        let im = (bytes[byte + 1] as f32 - IQ_BIAS) * IQ_SCALE;
        out[index] = Complex32::new(re, im);
        index += 1;
        byte += 2;
    }
}

/// NEON normalisation. Requires NEON; callers must check the feature.
///
/// # Safety
///
/// `bytes.len()` must be even and `out.len() >= bytes.len() / 2`.
#[cfg(target_arch = "aarch64")]
#[target_feature(enable = "neon")]
unsafe fn normalize_neon(bytes: &[u8], out: &mut [Complex32]) {
    use std::arch::aarch64::*;

    let bias = vdupq_n_f32(IQ_BIAS);
    let scale = vdupq_n_f32(IQ_SCALE);
    let pairs = bytes.len() / 2;

    let mut byte = 0usize;
    let mut index = 0usize;
    while byte + 8 <= bytes.len() {
        let raw = vld1_u8(bytes.as_ptr().add(byte));
        let widened = vmovl_u8(raw); // u16x8
        let lo_u32 = vmovl_u16(vget_low_u16(widened)); // u32x4
        let hi_u32 = vmovl_u16(vget_high_u16(widened)); // u32x4
        let lo = vmulq_f32(vsubq_f32(vcvtq_f32_u32(lo_u32), bias), scale);
        let hi = vmulq_f32(vsubq_f32(vcvtq_f32_u32(hi_u32), bias), scale);

        let mut values = [0f32; 8];
        vst1q_f32(values.as_mut_ptr(), lo);
        vst1q_f32(values.as_mut_ptr().add(4), hi);

        for c in 0..4 {
            out[index + c] = Complex32::new(values[2 * c], values[2 * c + 1]);
        }
        index += 4;
        byte += 8;
    }

    while index < pairs {
        let re = (bytes[byte] as f32 - IQ_BIAS) * IQ_SCALE;
        let im = (bytes[byte + 1] as f32 - IQ_BIAS) * IQ_SCALE;
        out[index] = Complex32::new(re, im);
        index += 1;
        byte += 2;
    }
}

// ---------------------------------------------------------------------------
// 2. DC blocker
// ---------------------------------------------------------------------------

/// Streaming one-pole DC remover.
///
/// Implements `y[n] = x[n] - x[n-1] + r * y[n-1]` independently on I and Q
/// (via [`Complex32`] arithmetic). The pole `r` must satisfy `0 <= r < 1`;
/// values close to 1 give a narrow notch and a long settling time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DcBlocker {
    pole: f32,
    prev_in: Complex32,
    prev_out: Complex32,
}

impl DcBlocker {
    /// Create a DC blocker with the given feedback pole.
    ///
    /// # Errors
    ///
    /// Returns [`DspError::NonFinite`] for NaN/infinity and
    /// [`DspError::InvalidArgument`] when `pole` is outside `[0, 1)`.
    pub fn new(pole: f32) -> Result<Self, DspError> {
        if !pole.is_finite() {
            return Err(DspError::NonFinite("dc blocker pole"));
        }
        if !(0.0..1.0).contains(&pole) {
            return Err(DspError::InvalidArgument(format!(
                "dc blocker pole {pole} must be in [0, 1)"
            )));
        }
        Ok(Self {
            pole,
            prev_in: Complex32::new(0.0, 0.0),
            prev_out: Complex32::new(0.0, 0.0),
        })
    }

    /// The configured pole.
    pub fn pole(&self) -> f32 {
        self.pole
    }

    /// Filter one sample, advancing the internal state.
    pub fn process(&mut self, sample: Complex32) -> Complex32 {
        let out = sample - self.prev_in + self.prev_out * self.pole;
        self.prev_in = sample;
        self.prev_out = out;
        out
    }

    /// Filter a slice in place.
    pub fn process_in_place(&mut self, samples: &mut [Complex32]) {
        for sample in samples.iter_mut() {
            *sample = self.process(*sample);
        }
    }

    /// Filter `input` into `output` without modifying `input`.
    ///
    /// # Errors
    ///
    /// Returns [`DspError::LengthMismatch`] when the slices differ in length.
    pub fn process_into(
        &mut self,
        input: &[Complex32],
        output: &mut [Complex32],
    ) -> Result<(), DspError> {
        if input.len() != output.len() {
            return Err(DspError::LengthMismatch {
                what: "dc blocker input/output",
                expected: input.len(),
                got: output.len(),
            });
        }
        for (dst, src) in output.iter_mut().zip(input.iter()) {
            *dst = self.process(*src);
        }
        Ok(())
    }

    /// Reset the internal state.
    pub fn reset(&mut self) {
        self.prev_in = Complex32::new(0.0, 0.0);
        self.prev_out = Complex32::new(0.0, 0.0);
    }
}

// ---------------------------------------------------------------------------
// 3. Streaming rational resampler
// ---------------------------------------------------------------------------

/// Largest interpolation / decimation factor accepted by [`RationalResampler`].
const MAX_RATE_FACTOR: usize = 4096;
/// Largest taps-per-phase accepted by [`RationalResampler`].
const MAX_TAPS_PER_PHASE: usize = 1024;
/// Largest prototype FIR length accepted by [`RationalResampler`].
const MAX_PROTOTYPE_TAPS: usize = 1 << 20;

fn gcd(mut a: usize, mut b: usize) -> usize {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a
}

/// Design a windowed-sinc lowpass prototype for rate `interp / decim`.
///
/// The prototype has `taps_per_phase * interp` taps and is scaled so that the
/// sum of all taps equals `interp` (unity gain per polyphase branch for
/// interpolation). The cutoff is placed at
/// `0.5 * min(1 / interp, 1 / decim)` cycles per (upsampled) sample.
fn design_lowpass(
    interp: usize,
    decim: usize,
    taps_per_phase: usize,
) -> Result<Vec<f32>, DspError> {
    let len = interp
        .checked_mul(taps_per_phase)
        .ok_or_else(|| DspError::InvalidArgument("prototype length overflow".into()))?;
    if len == 0 {
        return Err(DspError::InvalidArgument(
            "prototype length must be non-zero".into(),
        ));
    }

    let cutoff = 0.5 * (1.0 / interp as f64).min(1.0 / decim as f64);
    let center = (len - 1) as f64 / 2.0;
    let mut taps = Vec::with_capacity(len);
    let mut sum = 0.0f64;

    for m in 0..len {
        let x = 2.0 * cutoff * (m as f64 - center);
        let sinc = if x == 0.0 {
            1.0
        } else {
            let pix = std::f64::consts::PI * x;
            pix.sin() / pix
        };
        let window = if len == 1 {
            1.0
        } else {
            0.5 - 0.5 * (2.0 * std::f64::consts::PI * m as f64 / (len - 1) as f64).cos()
        };
        let value = 2.0 * cutoff * sinc * window;
        sum += value;
        taps.push(value as f32);
    }

    if sum == 0.0 || !sum.is_finite() {
        return Err(DspError::InvalidArgument(
            "degenerate lowpass prototype (zero sum)".into(),
        ));
    }

    // Scale to unity gain per phase (total gain == interp).
    let gain = interp as f64 / sum;
    for tap in taps.iter_mut() {
        *tap = (*tap as f64 * gain) as f32;
    }
    Ok(taps)
}

/// Streaming polyphase FIR rational resampler.
///
/// The resampler consumes input in arbitrary chunks. Output is bit-for-bit
/// identical to processing the concatenation as a single slice, because all
/// state (phase, output counter and a retained input history) is carried
/// between calls. See the module tests for the exact-continuity check.
///
/// The prototype introduces a group delay of roughly
/// `(taps_per_phase - 1) / 2` input samples.
#[derive(Debug, Clone)]
pub struct RationalResampler {
    interp: usize,
    decim: usize,
    taps_per_phase: usize,
    prototype: Vec<f32>,
    history: Vec<Complex32>,
    history_start_index: u64,
    total_in: u64,
    next_out: u64,
}

impl RationalResampler {
    /// Build a resampler with a windowed-sinc prototype of `taps_per_phase`
    /// phases per branch and a default of 16 taps per phase.
    ///
    /// # Errors
    ///
    /// Rejects zero / excessively large factors.
    pub fn new(interp: usize, decim: usize) -> Result<Self, DspError> {
        Self::with_taps(interp, decim, 16)
    }

    /// Build a resampler with an explicit number of taps per phase.
    ///
    /// `interp` and `decim` are reduced by their greatest common divisor.
    ///
    /// # Errors
    ///
    /// * [`DspError::InvalidArgument`] for zero factors, factors above
    ///   `4096`, or `taps_per_phase` outside `1..=1024`.
    /// * [`DspError::InvalidArgument`] when the prototype would exceed
    ///   `2^20` taps.
    pub fn with_taps(interp: usize, decim: usize, taps_per_phase: usize) -> Result<Self, DspError> {
        if interp == 0 || decim == 0 {
            return Err(DspError::InvalidArgument(
                "interpolation and decimation factors must be non-zero".into(),
            ));
        }
        if interp > MAX_RATE_FACTOR || decim > MAX_RATE_FACTOR {
            return Err(DspError::InvalidArgument(format!(
                "rate factors must be <= {MAX_RATE_FACTOR}"
            )));
        }
        if taps_per_phase == 0 || taps_per_phase > MAX_TAPS_PER_PHASE {
            return Err(DspError::InvalidArgument(format!(
                "taps_per_phase must be in 1..={MAX_TAPS_PER_PHASE}"
            )));
        }

        let divisor = gcd(interp, decim);
        let interp = interp / divisor;
        let decim = decim / divisor;

        // Bound the prototype before `design_lowpass` allocates it: the
        // prototype holds `interp * taps_per_phase` taps, so the product is
        // checked (and capped) first instead of after a potentially huge
        // allocation.
        let prototype_len = interp
            .checked_mul(taps_per_phase)
            .ok_or_else(|| DspError::InvalidArgument("prototype length overflow".into()))?;
        if prototype_len > MAX_PROTOTYPE_TAPS {
            return Err(DspError::InvalidArgument(format!(
                "prototype length {prototype_len} exceeds {MAX_PROTOTYPE_TAPS}"
            )));
        }

        let prototype = design_lowpass(interp, decim, taps_per_phase)?;

        Ok(Self {
            interp,
            decim,
            taps_per_phase,
            prototype,
            history: Vec::new(),
            history_start_index: 0,
            total_in: 0,
            next_out: 0,
        })
    }

    /// Reduced interpolation factor.
    pub fn interp(&self) -> usize {
        self.interp
    }

    /// Reduced decimation factor.
    pub fn decim(&self) -> usize {
        self.decim
    }

    /// Taps per polyphase branch.
    pub fn taps_per_phase(&self) -> usize {
        self.taps_per_phase
    }

    /// The prototype lowpass coefficients.
    pub fn prototype(&self) -> &[f32] {
        &self.prototype
    }

    /// Approximate processing delay expressed in input samples.
    pub fn group_delay_input(&self) -> f64 {
        (self.prototype.len() - 1) as f64 / (2.0 * self.interp as f64)
    }

    /// Append `input` and push every newly available output sample onto `out`.
    ///
    /// Returns the number of output samples produced by this call. Calling with
    /// an empty slice is valid and produces whatever the retained state allows
    /// (normally nothing).
    pub fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) -> usize {
        let produced_before = out.len();

        self.history.extend_from_slice(input);
        self.total_in = self
            .total_in
            .saturating_add(u64::try_from(input.len()).unwrap_or(u64::MAX));

        let interp = self.interp as u128;
        let decim = self.decim as u128;
        let taps = self.taps_per_phase;

        loop {
            let numerator = (self.next_out as u128) * decim;
            let phase = (numerator % interp) as usize;
            let base = numerator / interp; // input sample index (>= 0)
            if base >= self.total_in as u128 {
                break;
            }

            let mut acc = Complex32::new(0.0, 0.0);
            let history_start = self.history_start_index as i128;
            let base_i = base as i128;

            for j in 0..taps {
                let index = base_i - j as i128;
                if index < 0 {
                    continue; // zero initial state before the first sample
                }
                let offset = index - history_start;
                if offset < 0 {
                    // Should not happen: retention keeps everything needed.
                    continue;
                }
                let offset = offset as usize;
                if offset >= self.history.len() {
                    // Should not happen: `base < total_in` guarantees it.
                    break;
                }
                let tap = self.prototype[phase + j * self.interp];
                acc += self.history[offset] * tap;
            }

            out.push(acc);
            self.next_out += 1;
        }

        // Drop history that no future output can reference.
        let numerator = (self.next_out as u128) * decim;
        let next_base = numerator / interp;
        let keep_from = next_base.saturating_sub((taps as u128).saturating_sub(1));
        let keep_from = keep_from.min(self.total_in as u128) as u64;
        let drop_count = keep_from.saturating_sub(self.history_start_index) as usize;
        let drop_count = drop_count.min(self.history.len());
        if drop_count > 0 {
            self.history.drain(0..drop_count);
            self.history_start_index = self
                .history_start_index
                .saturating_add(u64::try_from(drop_count).unwrap_or(u64::MAX));
        }

        out.len() - produced_before
    }

    /// Reset all streaming state, keeping the prototype.
    pub fn reset(&mut self) {
        self.history.clear();
        self.history_start_index = 0;
        self.total_in = 0;
        self.next_out = 0;
    }
}

// ---------------------------------------------------------------------------
// 4. ISDB-T Mode / guard interval and cyclic-prefix correlation
// ---------------------------------------------------------------------------

/// ISDB-T transmission mode (FFT size class).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Mode 1: 2048-point FFT.
    Mode1,
    /// Mode 2: 4096-point FFT.
    Mode2,
    /// Mode 3: 8192-point FFT.
    Mode3,
}

impl Mode {
    /// All three modes, in order.
    pub const ALL: [Mode; 3] = [Mode::Mode1, Mode::Mode2, Mode::Mode3];

    /// FFT / effective symbol length in samples.
    pub fn fft_size(self) -> usize {
        match self {
            Mode::Mode1 => 2048,
            Mode::Mode2 => 4096,
            Mode::Mode3 => 8192,
        }
    }

    /// Nominal number of modulated carriers (segments * 108 + 1).
    pub fn nominal_carriers(self) -> usize {
        match self {
            Mode::Mode1 => 1405,
            Mode::Mode2 => 2809,
            Mode::Mode3 => 5617,
        }
    }
}

/// ISDB-T guard interval fraction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GuardInterval {
    /// 1/4 of the effective symbol length.
    OneQuarter,
    /// 1/8.
    OneEighth,
    /// 1/16.
    OneSixteenth,
    /// 1/32.
    OneThirtySecond,
}

impl GuardInterval {
    /// All four guard interval fractions, longest first.
    pub const ALL: [GuardInterval; 4] = [
        GuardInterval::OneQuarter,
        GuardInterval::OneEighth,
        GuardInterval::OneSixteenth,
        GuardInterval::OneThirtySecond,
    ];

    /// `(numerator, denominator)` of the guard fraction.
    pub fn fraction(self) -> (usize, usize) {
        match self {
            GuardInterval::OneQuarter => (1, 4),
            GuardInterval::OneEighth => (1, 8),
            GuardInterval::OneSixteenth => (1, 16),
            GuardInterval::OneThirtySecond => (1, 32),
        }
    }

    /// Guard length in samples for an effective symbol of `fft_size` samples.
    ///
    /// This is exact for the four ISDB-T fractions because the FFT sizes are
    /// all divisible by 32; the calculation is nonetheless saturating so it
    /// cannot overflow for arbitrary `usize` input.
    pub fn guard_len(self, fft_size: usize) -> usize {
        let (num, den) = self.fraction();
        fft_size.saturating_mul(num) / den
    }
}

/// A fully specified Mode / guard-interval pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct OfdmParameters {
    /// Transmission mode.
    pub mode: Mode,
    /// Guard interval fraction.
    pub guard: GuardInterval,
}

impl OfdmParameters {
    /// Construct from a mode and guard interval.
    pub fn new(mode: Mode, guard: GuardInterval) -> Self {
        Self { mode, guard }
    }

    /// Effective symbol length (FFT size).
    pub fn fft_size(&self) -> usize {
        self.mode.fft_size()
    }

    /// Guard interval length in samples.
    pub fn guard_len(&self) -> usize {
        self.guard.guard_len(self.fft_size())
    }

    /// Total OFDM symbol length (`fft_size + guard_len`).
    pub fn symbol_len(&self) -> usize {
        self.fft_size().saturating_add(self.guard_len())
    }
}

/// Result of one cyclic-prefix correlation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CpMetric {
    /// Raw correlation `sum x[n+k] * conj(x[n+k+fft])`.
    pub correlation: Complex32,
    /// Energy of the copied (post-FFT) part, `sum |x[n+k+fft]|^2`.
    pub power: f32,
    /// Normalised metric `|correlation| / sqrt(power_prefix * power)`, clamped
    /// to `[0, 1]`. `0.0` when either energy is zero.
    pub normalized: f32,
}

/// Cyclic-prefix correlator for a fixed `(fft_size, guard_len)` pair.
///
/// This computes the classic normalised guard-interval correlation. It does
/// **not** estimate the mode/guard from a signal, and it does not perform
/// timing recovery or tracking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CpCorrelator {
    fft_size: usize,
    guard_len: usize,
}

impl CpCorrelator {
    /// Create a correlator for the given sizes.
    ///
    /// # Errors
    ///
    /// Rejects zero sizes and sizes above `2^22`, and a combined symbol length
    /// that overflows.
    pub fn new(fft_size: usize, guard_len: usize) -> Result<Self, DspError> {
        if fft_size == 0 || guard_len == 0 {
            return Err(DspError::InvalidArgument(
                "fft_size and guard_len must be non-zero".into(),
            ));
        }
        if fft_size > MAX_FFT_SIZE || guard_len > MAX_FFT_SIZE {
            return Err(DspError::InvalidArgument(format!(
                "fft_size and guard_len must be <= {MAX_FFT_SIZE}"
            )));
        }
        if fft_size.checked_add(guard_len).is_none() {
            return Err(DspError::InvalidArgument(
                "fft_size + guard_len overflows usize".into(),
            ));
        }
        Ok(Self {
            fft_size,
            guard_len,
        })
    }

    /// Build a correlator from Mode / guard parameters.
    pub fn from_parameters(params: OfdmParameters) -> Self {
        Self {
            fft_size: params.fft_size(),
            guard_len: params.guard_len(),
        }
    }

    /// Effective symbol length.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Guard interval length.
    pub fn guard_len(&self) -> usize {
        self.guard_len
    }

    /// Number of offsets at which [`correlate`](Self::correlate) can be called
    /// for a buffer of `sample_len` samples (`0` when it does not fit).
    pub fn valid_offsets(&self, sample_len: usize) -> usize {
        let symbol = self.fft_size + self.guard_len;
        if sample_len < symbol {
            0
        } else {
            sample_len - symbol + 1
        }
    }

    /// Correlate the guard interval starting at `offset`.
    ///
    /// Returns `None` when `offset + fft_size + guard_len` exceeds
    /// `samples.len()`, so malformed input can never panic.
    pub fn correlate(&self, samples: &[Complex32], offset: usize) -> Option<CpMetric> {
        let end = offset
            .checked_add(self.fft_size)?
            .checked_add(self.guard_len)?;
        if end > samples.len() {
            return None;
        }

        let mut correlation = Complex32::new(0.0, 0.0);
        let mut prefix_energy = 0.0f64;
        let mut post_energy = 0.0f64;

        for k in 0..self.guard_len {
            let prefix = samples[offset + k];
            let post = samples[offset + k + self.fft_size];
            correlation += prefix * post.conj();
            prefix_energy +=
                (prefix.re as f64) * (prefix.re as f64) + (prefix.im as f64) * (prefix.im as f64);
            post_energy +=
                (post.re as f64) * (post.re as f64) + (post.im as f64) * (post.im as f64);
        }

        let denom = (prefix_energy * post_energy).sqrt();
        let magnitude = ((correlation.re as f64) * (correlation.re as f64)
            + (correlation.im as f64) * (correlation.im as f64))
            .sqrt();
        let normalized = if denom > 0.0 {
            (magnitude / denom).clamp(0.0, 1.0) as f32
        } else {
            0.0
        };

        Some(CpMetric {
            correlation,
            power: post_energy as f32,
            normalized,
        })
    }

    /// Search offsets in `start..end` for the highest normalised correlation.
    ///
    /// `end` is clamped to the last valid offset. Returns `None` for an empty
    /// or out-of-range search window.
    pub fn find_best_offset(
        &self,
        samples: &[Complex32],
        start: usize,
        end: usize,
    ) -> Option<(usize, CpMetric)> {
        let valid = self.valid_offsets(samples.len());
        let end = end.min(valid);
        if start >= end {
            return None;
        }
        let mut best: Option<(usize, CpMetric)> = None;
        for offset in start..end {
            if let Some(metric) = self.correlate(samples, offset) {
                match &best {
                    Some((_, current)) if current.normalized >= metric.normalized => {}
                    _ => best = Some((offset, metric)),
                }
            }
        }
        best
    }
}

// ---------------------------------------------------------------------------
// 5. FFT wrapper
// ---------------------------------------------------------------------------

/// A validated forward FFT of a fixed length, backed by `rustfft`.
pub struct Fft {
    len: usize,
    forward: Arc<dyn RustFft<f32>>,
}

impl fmt::Debug for Fft {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Fft").field("len", &self.len).finish()
    }
}

impl Fft {
    /// Plan a forward FFT of length `len`.
    ///
    /// # Errors
    ///
    /// Rejects `len == 0` and lengths above `2^22`.
    pub fn new(len: usize) -> Result<Self, DspError> {
        if len == 0 {
            return Err(DspError::InvalidArgument(
                "FFT length must be non-zero".into(),
            ));
        }
        if len > MAX_FFT_SIZE {
            return Err(DspError::InvalidArgument(format!(
                "FFT length {len} exceeds {MAX_FFT_SIZE}"
            )));
        }
        let mut planner = FftPlanner::<f32>::new();
        let forward = planner.plan_fft_forward(len);
        Ok(Self { len, forward })
    }

    /// Transform length.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Always `false`; a zero-length FFT cannot be constructed.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Compute the forward DFT of `input` into `output`.
    ///
    /// The transform is unnormalised, matching `rustfft`: a subsequent inverse
    /// transform scales by `len`.
    ///
    /// # Errors
    ///
    /// Returns [`DspError::LengthMismatch`] unless both slices have length
    /// [`len`](Self::len).
    pub fn forward(&self, input: &[Complex32], output: &mut [Complex32]) -> Result<(), DspError> {
        if input.len() != self.len {
            return Err(DspError::LengthMismatch {
                what: "fft input",
                expected: self.len,
                got: input.len(),
            });
        }
        if output.len() != self.len {
            return Err(DspError::LengthMismatch {
                what: "fft output",
                expected: self.len,
                got: output.len(),
            });
        }
        output.copy_from_slice(input);
        self.forward.process(output);
        Ok(())
    }

    /// Convenience forward transform returning a fresh vector.
    pub fn forward_vec(&self, input: &[Complex32]) -> Result<Vec<Complex32>, DspError> {
        let mut output = vec![Complex32::new(0.0, 0.0); self.len];
        self.forward(input, &mut output)?;
        Ok(output)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn test_signal(len: usize, seed: u32) -> Vec<Complex32> {
        // Deterministic pseudo-random signal; no external RNG dependency.
        let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        (0..len)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let re = ((state >> 8) & 0xFFFF) as f32 / 32768.0 - 1.0;
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let im = ((state >> 8) & 0xFFFF) as f32 / 32768.0 - 1.0;
                Complex32::new(re, im)
            })
            .collect()
    }

    // ---- normalisation ---------------------------------------------------

    #[test]
    fn normalize_maps_bytes_into_unit_range() {
        let bytes = [0u8, 255, 127, 128, 128, 0];
        let out = normalize_u8_iq(&bytes).unwrap();
        assert_eq!(out.len(), 3);
        assert!((out[0].re + 1.0).abs() < 1e-6);
        assert!((out[0].im - 1.0).abs() < 1e-6);
        // (127 - 127.5)/127.5 and (128 - 127.5)/127.5
        assert!((out[1].re + (0.5 / 127.5)).abs() < 1e-7);
        assert!((out[1].im - (0.5 / 127.5)).abs() < 1e-7);
        assert!((out[2].re - (0.5 / 127.5)).abs() < 1e-7);
        assert!((out[2].im + 1.0).abs() < 1e-6);
    }

    #[test]
    fn normalize_rejects_odd_length_and_small_output() {
        assert_eq!(
            normalize_u8_iq(&[1, 2, 3]).unwrap_err(),
            DspError::OddIqByteCount(3)
        );
        let mut out = [Complex32::new(0.0, 0.0); 1];
        let err = normalize_u8_iq_into(&[1, 2, 3, 4], &mut out).unwrap_err();
        assert_eq!(err, DspError::OutputTooSmall { needed: 2, got: 1 });
        // Extra output room is allowed and left untouched.
        let mut out = [Complex32::new(9.0, 9.0); 3];
        normalize_u8_iq_into(&[10, 20], &mut out).unwrap();
        assert_eq!(out[1], Complex32::new(9.0, 9.0));
        // Empty input is a no-op.
        normalize_u8_iq(&[]).unwrap();
    }

    #[test]
    fn normalize_simd_matches_scalar_exactly() {
        let bytes: Vec<u8> = (0..=255u8).collect();
        // Also exercise the scalar tail with a non-multiple-of-16 suffix.
        let bytes: Vec<u8> = bytes.into_iter().chain([0u8, 1, 2, 3]).collect();
        assert!(bytes.len() % 2 == 0);

        let mut scalar = vec![Complex32::new(0.0, 0.0); bytes.len() / 2];
        normalize_scalar(&bytes, &mut scalar);

        #[cfg(target_arch = "x86_64")]
        if std::arch::is_x86_feature_detected!("avx2") {
            let mut simd = vec![Complex32::new(0.0, 0.0); bytes.len() / 2];
            // SAFETY: AVX2 was detected; the function's contract is met.
            unsafe { normalize_avx2(&bytes, &mut simd) };
            for (a, b) in scalar.iter().zip(simd.iter()) {
                assert_eq!(a.re.to_bits(), b.re.to_bits());
                assert_eq!(a.im.to_bits(), b.im.to_bits());
            }
        }

        #[cfg(target_arch = "aarch64")]
        if std::arch::is_aarch64_feature_detected!("neon") {
            let mut simd = vec![Complex32::new(0.0, 0.0); bytes.len() / 2];
            // SAFETY: NEON was detected; the function's contract is met.
            unsafe { normalize_neon(&bytes, &mut simd) };
            for (a, b) in scalar.iter().zip(simd.iter()) {
                assert_eq!(a.re.to_bits(), b.re.to_bits());
                assert_eq!(a.im.to_bits(), b.im.to_bits());
            }
        }
    }

    // ---- DC blocker ------------------------------------------------------

    #[test]
    fn dc_blocker_removes_constant_input() {
        let mut blocker = DcBlocker::new(0.99).unwrap();
        let mut last = Complex32::new(0.0, 0.0);
        for _ in 0..20_000 {
            last = blocker.process(Complex32::new(0.75, -0.5));
        }
        assert!(last.re.abs() < 1e-3, "re residual {}", last.re);
        assert!(last.im.abs() < 1e-3, "im residual {}", last.im);
    }

    #[test]
    fn dc_blocker_preserves_ac_after_settling() {
        let mut blocker = DcBlocker::new(0.999).unwrap();
        let mut peak = 0.0f32;
        for n in 0..50_000 {
            let phase = 2.0 * std::f32::consts::PI * 0.01 * n as f32;
            let out = blocker.process(Complex32::new(phase.sin(), 0.0));
            if n > 40_000 {
                peak = peak.max(out.re.abs());
            }
        }
        assert!((peak - 1.0).abs() < 0.05, "peak {peak}");
    }

    #[test]
    fn dc_blocker_validates_pole() {
        assert!(DcBlocker::new(f32::NAN).is_err());
        assert!(DcBlocker::new(f32::INFINITY).is_err());
        assert!(DcBlocker::new(-0.1).is_err());
        assert!(DcBlocker::new(1.0).is_err());
        assert!(DcBlocker::new(0.0).is_ok());
    }

    #[test]
    fn dc_blocker_process_into_checks_lengths() {
        let mut blocker = DcBlocker::new(0.9).unwrap();
        let input = [Complex32::new(1.0, 0.0); 3];
        let mut output = [Complex32::new(0.0, 0.0); 2];
        assert!(blocker.process_into(&input, &mut output).is_err());
    }

    // ---- resampler -------------------------------------------------------

    #[test]
    fn resampler_is_streaming_exact_across_chunks() {
        let signal = test_signal(257, 0xC0FFEE);
        for (interp, decim) in [(1, 1), (2, 1), (1, 2), (2, 3), (3, 2), (5, 4)] {
            let mut whole = RationalResampler::with_taps(interp, decim, 8).unwrap();
            let mut expected = Vec::new();
            whole.process(&signal, &mut expected);

            for chunk in [1usize, 2, 3, 5, 7, 16] {
                let mut split = RationalResampler::with_taps(interp, decim, 8).unwrap();
                let mut actual = Vec::new();
                for piece in signal.chunks(chunk) {
                    split.process(piece, &mut actual);
                }
                // A final empty call must not change anything.
                split.process(&[], &mut actual);
                assert_eq!(
                    actual, expected,
                    "ratio {interp}/{decim} chunk {chunk} did not match contiguous output"
                );
            }
        }
    }

    #[test]
    fn resampler_matches_naive_reference() {
        // Independent f64 reference: zero-stuff, convolve, decimate.
        fn reference(
            prototype: &[f32],
            interp: usize,
            decim: usize,
            input: &[Complex32],
        ) -> Vec<(f64, f64)> {
            let mut up = vec![(0.0f64, 0.0f64); input.len() * interp];
            for (i, x) in input.iter().enumerate() {
                up[i * interp] = (x.re as f64, x.im as f64);
            }
            let mut out = Vec::new();
            let mut n = 0usize;
            loop {
                let center = n * decim;
                if center >= up.len() {
                    break;
                }
                let mut re = 0.0f64;
                let mut im = 0.0f64;
                for (m, tap) in prototype.iter().enumerate() {
                    if center < m {
                        break;
                    }
                    let (ure, uim) = up[center - m];
                    let h = *tap as f64;
                    re += ure * h;
                    im += uim * h;
                }
                out.push((re, im));
                n += 1;
            }
            out
        }

        let signal = test_signal(200, 0x1234_5678);
        for (interp, decim) in [(2, 1), (1, 2), (2, 3)] {
            let mut resampler = RationalResampler::with_taps(interp, decim, 12).unwrap();
            let mut actual = Vec::new();
            resampler.process(&signal, &mut actual);

            let expected = reference(
                resampler.prototype(),
                resampler.interp(),
                resampler.decim(),
                &signal,
            );
            assert_eq!(actual.len(), expected.len());
            for (a, e) in actual.iter().zip(expected.iter()) {
                assert!(
                    (a.re as f64 - e.0).abs() < 1e-4,
                    "ratio {interp}/{decim} re {} vs {}",
                    a.re,
                    e.0
                );
                assert!(
                    (a.im as f64 - e.1).abs() < 1e-4,
                    "ratio {interp}/{decim} im {} vs {}",
                    a.im,
                    e.1
                );
            }
        }
    }

    #[test]
    fn resampler_dc_gain_is_unity() {
        let mut resampler = RationalResampler::with_taps(2, 1, 24).unwrap();
        let input = vec![Complex32::new(1.0, 0.0); 400];
        let mut output = Vec::new();
        resampler.process(&input, &mut output);

        let delay = resampler.group_delay_input() as usize;
        let tail = &output[delay + 10..];
        let mean: f32 = tail.iter().map(|s| s.re).sum::<f32>() / tail.len() as f32;
        assert!((mean - 1.0).abs() < 0.02, "mean {mean}");
    }

    #[test]
    fn resampler_validates_arguments() {
        assert!(RationalResampler::new(0, 1).is_err());
        assert!(RationalResampler::new(1, 0).is_err());
        assert!(RationalResampler::new(MAX_RATE_FACTOR + 1, 1).is_err());
        assert!(RationalResampler::with_taps(1, 1, 0).is_err());
        assert!(RationalResampler::with_taps(1, 1, MAX_TAPS_PER_PHASE + 1).is_err());
        assert!(RationalResampler::new(2, 1).is_ok());
    }

    #[test]
    fn resampler_rejects_oversized_prototype_before_allocating() {
        // `MAX_RATE_FACTOR * MAX_TAPS_PER_PHASE = 2^22` exceeds
        // `MAX_PROTOTYPE_TAPS = 2^20`, so construction must fail. The pre-check
        // rejects the product before `design_lowpass` can allocate it.
        let err = RationalResampler::with_taps(MAX_RATE_FACTOR, 1, MAX_TAPS_PER_PHASE)
            .expect_err("oversized prototype must be rejected");
        assert!(
            matches!(err, DspError::InvalidArgument(_)),
            "unexpected error: {err:?}"
        );
    }

    #[test]
    fn resampler_reset_restarts_stream() {
        let signal = test_signal(64, 7);
        let mut resampler = RationalResampler::new(2, 1).unwrap();
        let mut first = Vec::new();
        resampler.process(&signal, &mut first);
        resampler.reset();
        let mut second = Vec::new();
        resampler.process(&signal, &mut second);
        assert_eq!(first, second);
    }

    // ---- Mode / guard ----------------------------------------------------

    #[test]
    fn mode_and_guard_sizes_match_isdb_t() {
        assert_eq!(Mode::Mode1.fft_size(), 2048);
        assert_eq!(Mode::Mode2.fft_size(), 4096);
        assert_eq!(Mode::Mode3.fft_size(), 8192);
        assert_eq!(Mode::Mode1.nominal_carriers(), 1405);
        assert_eq!(Mode::Mode2.nominal_carriers(), 2809);
        assert_eq!(Mode::Mode3.nominal_carriers(), 5617);

        let p = OfdmParameters::new(Mode::Mode1, GuardInterval::OneQuarter);
        assert_eq!(p.guard_len(), 512);
        assert_eq!(p.symbol_len(), 2560);

        let p = OfdmParameters::new(Mode::Mode2, GuardInterval::OneSixteenth);
        assert_eq!(p.guard_len(), 256);

        let p = OfdmParameters::new(Mode::Mode3, GuardInterval::OneEighth);
        assert_eq!(p.guard_len(), 1024);
        assert_eq!(p.symbol_len(), 9216);

        let p = OfdmParameters::new(Mode::Mode3, GuardInterval::OneThirtySecond);
        assert_eq!(p.guard_len(), 256);

        assert_eq!(GuardInterval::OneQuarter.fraction(), (1, 4));
        assert_eq!(GuardInterval::OneThirtySecond.fraction(), (1, 32));
    }

    // ---- CP correlation --------------------------------------------------

    #[test]
    fn cp_correlation_peaks_at_the_guard_interval() {
        let fft = 64usize;
        let guard = 16usize;
        let block = test_signal(fft, 0xABCD);
        // OFDM symbol: copy of the last `guard` samples followed by the block.
        let mut symbol = Vec::new();
        symbol.extend_from_slice(&block[fft - guard..]);
        symbol.extend_from_slice(&block);

        let mut stream = vec![Complex32::new(0.0, 0.0); 5];
        stream.extend_from_slice(&symbol);
        stream.extend_from_slice(&[Complex32::new(0.0, 0.0); 4]);

        let correlator = CpCorrelator::new(fft, guard).unwrap();
        let metric = correlator.correlate(&stream, 5).unwrap();
        assert!(
            (metric.normalized - 1.0).abs() < 1e-5,
            "correct offset normalized {}",
            metric.normalized
        );

        let (best_offset, best) = correlator.find_best_offset(&stream, 0, 20).unwrap();
        assert_eq!(best_offset, 5);
        assert!((best.normalized - 1.0).abs() < 1e-5);
    }

    #[test]
    fn cp_correlation_rejects_insufficient_input() {
        let correlator = CpCorrelator::new(64, 16).unwrap();
        let short = vec![Complex32::new(1.0, 0.0); 10];
        assert!(correlator.correlate(&short, 0).is_none());
        assert!(correlator.find_best_offset(&short, 0, 100).is_none());
        assert_eq!(correlator.valid_offsets(10), 0);
        assert_eq!(correlator.valid_offsets(80), 1);
        assert_eq!(correlator.valid_offsets(81), 2);
    }

    #[test]
    fn cp_correlator_validates_sizes() {
        assert!(CpCorrelator::new(0, 1).is_err());
        assert!(CpCorrelator::new(1, 0).is_err());
        assert!(CpCorrelator::new(MAX_FFT_SIZE + 1, 1).is_err());
        assert!(CpCorrelator::new(2048, 512).is_ok());
        let correlator = CpCorrelator::from_parameters(OfdmParameters::new(
            Mode::Mode1,
            GuardInterval::OneQuarter,
        ));
        assert_eq!(correlator.fft_size(), 2048);
        assert_eq!(correlator.guard_len(), 512);
    }

    // ---- FFT -------------------------------------------------------------

    #[test]
    fn fft_of_impulse_is_flat() {
        let fft = Fft::new(4).unwrap();
        let input = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 0.0),
            Complex32::new(0.0, 0.0),
            Complex32::new(0.0, 0.0),
        ];
        let out = fft.forward_vec(&input).unwrap();
        for value in out {
            assert!((value.re - 1.0).abs() < 1e-6);
            assert!(value.im.abs() < 1e-6);
        }
    }

    #[test]
    fn fft_of_shifted_impulse_matches_dft_phase() {
        let len = 8;
        let fft = Fft::new(len).unwrap();
        let mut input = vec![Complex32::new(0.0, 0.0); len];
        input[1] = Complex32::new(1.0, 0.0);
        let out = fft.forward_vec(&input).unwrap();
        for (k, value) in out.iter().enumerate() {
            let angle = -2.0 * std::f64::consts::PI * k as f64 / len as f64;
            let expected = Complex32::new(angle.cos() as f32, angle.sin() as f32);
            assert!((value.re - expected.re).abs() < 1e-5, "k={k}");
            assert!((value.im - expected.im).abs() < 1e-5, "k={k}");
        }
    }

    #[test]
    fn fft_validates_lengths() {
        assert!(Fft::new(0).is_err());
        assert!(Fft::new(MAX_FFT_SIZE + 1).is_err());
        let fft = Fft::new(8).unwrap();
        assert_eq!(fft.len(), 8);
        assert!(!fft.is_empty());
        assert!(fft.forward_vec(&[Complex32::new(0.0, 0.0); 7]).is_err());
        let mut out = [Complex32::new(0.0, 0.0); 7];
        assert!(fft
            .forward(&[Complex32::new(0.0, 0.0); 8], &mut out)
            .is_err());
    }
}
