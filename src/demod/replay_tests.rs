//! End-to-end IQ-replay integration tests for the ISDB-T one-segment receiver.
//!
//! These tests build a **complete, independent transmitter fixture** and drive
//! it through the real receiver. No TS is injected into the receiver and no
//! decoder stage is faked: the fixture starts from 188-byte TS packets and runs
//!
//! ```text
//! TS -> RS(204,188) -> per-frame energy dispersal -> byte interleave
//!    -> K=7 rate-1/2 -> puncturing -> bit interleave -> QPSK mapping
//!    -> time interleave -> frequency interleave -> SP / TMCC / AC1 insertion
//!    -> unitary IFFT + guard interval
//!    -> 64/63 MHz segment waveform
//!    -> reverse rational resample 189/160 to 1.2 MHz -> u8 I/Q quantise
//! ```
//!
//! and then feeds those raw `u8` IQ bytes to [`Decoder::push_iq`], exactly as
//! the RTL-SDR front end would.
//!
//! # Assertion policy
//!
//! The recovered TS must be an **exact, contiguous run** of the transmitted
//! packets in transmission order starting at the first RS-decoded packet. A
//! result that merely contains a subset of the transmitted packets, or no
//! packets at all, is a failure. The returned stage counters (TMCC frames,
//! decoded TS packets, resets) are surfaced in every failure message so the
//! first failing stage is identifiable.
//!
//! # Runtime
//!
//! The raw-IQ tests are intentionally run by default (`cargo test`). They are
//! heavier than a unit test because they exercise the resampler, synchroniser
//! and 204-symbol frame collector; the carrier-level test is cheap and isolates
//! the FEC/control path from the front end.

#![allow(clippy::too_many_lines)]

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use num_complex::Complex32;
use rustfft::FftPlanner;

use crate::dsp::{GuardInterval, Mode, RationalResampler};
use crate::fec::{self, ContinuityEvent, PunctureRate, TsContinuityTracker};

use super::carriers::equalize_symbol;
use super::coding::test_support;
use super::coding::{
    demap, packets_per_frame, ChannelDecoder, LayerConfig, Modulation, BYTE_INTERLEAVE_LATENCY,
    TSP_SIZE,
};
use super::control::{
    carriers_per_segment, data_carriers, data_carriers_per_segment, frequency_interleave,
    pilot_modulating_signal, scattered_pilot_carriers, time_interleave_max_delay, tmcc_carriers,
    tmcc_encode, PilotPrbs, TimeDeinterleaver, TmccSync, SYMBOLS_PER_FRAME, TMCC_CODEWORD_BITS,
    TMCC_INFO_BITS, TMCC_INFO_OFFSET, TMCC_SEGMENT_TYPE_OFFSET, TMCC_SYNC_OFFSET, TMCC_TOTAL_BITS,
};
use super::frame::FrameCollector;
use super::ofdm::{segment_fft_size, segment_symbol_len};
use super::receiver::{Decoder, ReceiverStats, DEFAULT_INPUT_RATE};

/// A 188-byte transport-stream packet.
type Packet = [u8; 188];

/// Carriers of one synthesized OFDM symbol.
type CarrierVector = Vec<Complex32>;

/// A complete synthesized carrier stream, in transmission order.
type CarrierStream = Vec<CarrierVector>;

/// Fixture cache keyed by `(frames, interleave)`. Factored out of the `static`
/// so the cache's type stays readable.
type CarrierCache = Mutex<HashMap<(usize, usize), Arc<CarrierStream>>>;

/// Transmission parameters of the fixture: Mode 3, GI 1/8, coherent QPSK,
/// rate 1/2, coherent one-segment partial reception.
const MODE: Mode = Mode::Mode3;
const GUARD: GuardInterval = GuardInterval::OneEighth;
const MODULATION: Modulation = Modulation::Qpsk;
const RATE: PunctureRate = PunctureRate::Rate1_2;

/// Normalised amplitude the quantiser targets, so the `u8` samples stay well
/// away from the `0`/`255` rails.
const TX_NORMALISED_SCALE: f32 = 0.2;

// ---------------------------------------------------------------------------
// Deterministic TS fixture: PAT with valid CRC + multiple payload PIDs
// ---------------------------------------------------------------------------

const PAT_PID: u16 = 0x0000;
const PAYLOAD_PID_A: u16 = 0x0100;
const PAYLOAD_PID_B: u16 = 0x0101;

fn set_bits(dst: &mut [u8], start: usize, len: usize, value: u16) {
    for i in 0..len {
        dst[start + i] = ((value >> (len - 1 - i)) & 1) as u8;
    }
}

/// Build a Program Association Table in a single-packet section with a valid
/// MPEG-2 CRC-32.
fn build_pat(continuity: u8) -> Packet {
    let mut packet = [0xFFu8; 188];
    packet[0] = 0x47;
    packet[1] = 0x40 | ((PAT_PID >> 8) as u8 & 0x1F); // PUSI + PID high bits
    packet[2] = (PAT_PID & 0xFF) as u8;
    packet[3] = 0x10 | (continuity & 0x0F); // payload only
    packet[4] = 0x00; // pointer_field: the section starts immediately

    let section = 5usize; // 4-byte header + pointer_field
    let section_length: u16 = 13; // tsid..CRC inclusive beyond the length field
    packet[section] = 0x00; // table_id
    packet[section + 1] = 0xB0 | ((section_length >> 8) as u8 & 0x0F);
    packet[section + 2] = (section_length & 0xFF) as u8;
    packet[section + 3] = 0x00;
    packet[section + 4] = 0x01; // transport_stream_id
    packet[section + 5] = 0xC1; // reserved, version 0, current_next = 1
    packet[section + 6] = 0x00; // section_number
    packet[section + 7] = 0x00; // last_section_number
    packet[section + 8] = 0x00;
    packet[section + 9] = 0x01; // program_number 1
    packet[section + 10] = 0xE0 | ((PAYLOAD_PID_A >> 8) as u8 & 0x1F);
    packet[section + 11] = (PAYLOAD_PID_A & 0xFF) as u8;

    let crc_region = &packet[section..section + 12];
    let crc = fec::crc32_mpeg2(crc_region);
    packet[section + 12..section + 16].copy_from_slice(&crc.to_be_bytes());
    packet
}

/// Deterministic TS stream: a PAT every 20 packets, payload on two PIDs in
/// between, each PID carrying a strictly incrementing continuity counter.
fn build_ts_packets(count: usize) -> Vec<Packet> {
    let mut packets = Vec::with_capacity(count);
    let mut continuity = [0u8; 3]; // 0 = PAT, 1 = PID A, 2 = PID B
    let mut state = 0x1234_5678u32;

    for index in 0..count {
        if index % 20 == 0 {
            let packet = build_pat(continuity[0]);
            continuity[0] = (continuity[0] + 1) & 0x0F;
            packets.push(packet);
            continue;
        }

        let (slot, pid, tag) = if index % 2 == 0 {
            (1usize, PAYLOAD_PID_A, 0xA5u8)
        } else {
            (2usize, PAYLOAD_PID_B, 0x5Au8)
        };
        let counter = continuity[slot];
        continuity[slot] = (counter + 1) & 0x0F;

        let mut packet = [0u8; 188];
        packet[0] = 0x47;
        packet[1] = ((pid >> 8) as u8) & 0x1F;
        packet[2] = (pid & 0xFF) as u8;
        packet[3] = 0x10 | (counter & 0x0F);
        for (offset, slot_byte) in packet[4..].iter_mut().enumerate() {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *slot_byte = ((state >> 24) as u8) ^ tag ^ (index as u8) ^ (offset as u8);
        }
        packets.push(packet);
    }
    packets
}

// ---------------------------------------------------------------------------
// Transmitter fixture
// ---------------------------------------------------------------------------

fn modulation_code(modulation: Modulation) -> u16 {
    match modulation {
        Modulation::Dqpsk => 0b000,
        Modulation::Qpsk => 0b001,
        Modulation::Qam16 => 0b010,
        Modulation::Qam64 => 0b011,
    }
}

fn rate_code(rate: PunctureRate) -> u16 {
    match rate {
        PunctureRate::Rate1_2 => 0b000,
        PunctureRate::Rate2_3 => 0b001,
        PunctureRate::Rate3_4 => 0b010,
        PunctureRate::Rate5_6 => 0b011,
        PunctureRate::Rate7_8 => 0b100,
    }
}

/// Inverse of [`super::control::tmcc_interleave_value`] for Mode 3.
fn interleave_code(interleave: usize) -> u8 {
    match (MODE, interleave) {
        (_, 0) => 0b000,
        (Mode::Mode3, 1) => 0b001,
        (Mode::Mode3, 2) => 0b010,
        (Mode::Mode3, 4) => 0b011,
        other => panic!("unsupported (mode, interleave) fixture parameter: {other:?}"),
    }
}

/// Build the 204 TMCC bits `B0..B203` of a coherent one-segment frame.
fn build_tmcc_bits(sync: TmccSync, interleave: usize) -> [u8; TMCC_TOTAL_BITS] {
    let mut info = vec![1u8; TMCC_INFO_BITS];
    set_bits(&mut info, 0, 2, 0b00); // terrestrial television
    set_bits(&mut info, 2, 4, 0b1111); // no parameter switching
    info[6] = 0; // no emergency start
    info[7] = 1; // partial reception on segment 0
    set_bits(&mut info, 8, 3, modulation_code(MODULATION));
    set_bits(&mut info, 11, 3, rate_code(RATE));
    set_bits(&mut info, 14, 3, interleave_code(interleave) as u16);
    set_bits(&mut info, 17, 4, 1); // one segment

    let codeword = tmcc_encode(&info).expect("102 info bits");
    let mut bits = [0u8; TMCC_TOTAL_BITS];
    set_bits(&mut bits, TMCC_SYNC_OFFSET, 16, sync.word());
    set_bits(&mut bits, TMCC_SEGMENT_TYPE_OFFSET, 3, 0b000); // coherent
    bits[TMCC_INFO_OFFSET..TMCC_INFO_OFFSET + TMCC_CODEWORD_BITS].copy_from_slice(&codeword);
    bits
}

/// Build the complete per-symbol centre-segment carrier vectors for `frames`
/// OFDM frames, in transmission order.
fn synthesize_carriers(packets: &[Packet], frames: usize, interleave: usize) -> CarrierStream {
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    assert_eq!(
        packets.len(),
        frames * per_frame,
        "fixture must supply whole frames of transmission TSPs"
    );
    let frame_bytes = per_frame * TSP_SIZE;

    // FEC: RS, per-frame energy dispersal, byte interleave, K=7 + puncture,
    // bit interleave. Uses the shared test-only transmitter helpers, which
    // operate on the *private* receiver constants so a divergence shows here.
    let rotated = test_support::build_frame(packets);
    let dispersed = test_support::energy_disperse(&rotated, frame_bytes);
    let interleaved = test_support::byte_interleave(&dispersed);
    let punctured = test_support::punctured_bits(&interleaved, MODULATION, RATE);
    let tx_bits = test_support::bit_interleave_bits(&punctured, MODULATION);

    let data_per_symbol = data_carriers_per_segment(MODE) * MODULATION.bits_per_symbol();
    let total_symbols = frames * SYMBOLS_PER_FRAME;
    assert_eq!(
        tx_bits.len(),
        total_symbols * data_per_symbol,
        "coded-bit count does not fill the requested frames"
    );

    let total_carriers = carriers_per_segment(MODE);
    let qpsk_amplitude = 1.0 / MODULATION.demap_scale();
    let pilot_amplitude = pilot_modulating_signal(0).expect("W=0 pilot").0;

    let mut time_interleaver =
        super::control::TimeInterleaver::<Complex32>::new(MODE, interleave).expect("valid I");
    let mut out = Vec::with_capacity(total_symbols);

    for frame in 0..frames {
        let sync = if frame % 2 == 0 {
            TmccSync::W0
        } else {
            TmccSync::W1
        };
        let tmcc_bits = build_tmcc_bits(sync, interleave);
        // `B'0 = 0`, `B'k = B'k-1 XOR Bk` (§3.13.3).
        let mut b_prime = [0u8; TMCC_TOTAL_BITS];
        for k in 1..TMCC_TOTAL_BITS {
            b_prime[k] = b_prime[k - 1] ^ tmcc_bits[k];
        }

        for (symbol, &signal_reference) in b_prime.iter().enumerate().take(SYMBOLS_PER_FRAME) {
            let global = frame * SYMBOLS_PER_FRAME + symbol;
            let bits = &tx_bits[global * data_per_symbol..(global + 1) * data_per_symbol];

            let mut logical = Vec::with_capacity(data_carriers_per_segment(MODE));
            for pair in bits.chunks_exact(2) {
                let re = if pair[0] == 0 {
                    qpsk_amplitude
                } else {
                    -qpsk_amplitude
                };
                let im = if pair[1] == 0 {
                    qpsk_amplitude
                } else {
                    -qpsk_amplitude
                };
                logical.push(Complex32::new(re, im));
            }

            let timed = time_interleaver
                .process_symbol(&logical)
                .expect("one data carrier per input");
            let mut physical = vec![Complex32::new(0.0, 0.0); data_carriers_per_segment(MODE)];
            frequency_interleave(MODE, &timed, &mut physical).expect("data carrier count");

            let mut carriers = vec![Complex32::new(0.0, 0.0); total_carriers];
            for (rank, &carrier) in data_carriers(MODE, MODULATION, symbol).iter().enumerate() {
                carriers[carrier] = physical[rank];
            }
            for &carrier in &scattered_pilot_carriers(MODE, MODULATION, symbol) {
                let w = PilotPrbs::bit_at(MODE, 0, carrier).expect("SP carrier in segment");
                let (re, im) = pilot_modulating_signal(w).expect("PRBS bit is 0/1");
                carriers[carrier] = Complex32::new(re, im);
            }
            for &carrier in tmcc_carriers(MODULATION, MODE) {
                let reference =
                    PilotPrbs::bit_at(MODE, 0, carrier).expect("TMCC carrier in segment");
                let signal_bit = signal_reference ^ reference;
                let sign = if signal_bit == 0 { 1.0 } else { -1.0 };
                carriers[carrier] = Complex32::new(pilot_amplitude * sign, 0.0);
            }

            out.push(carriers);
        }
    }
    out
}

/// Deterministic cache of synthesized carrier streams keyed by
/// `(frames, interleave)`.
///
/// `synthesize_carriers` runs the entire transmitter FEC/interleave chain, so
/// tests that share transmission parameters (for example the carrier-level and
/// raw-IQ multi-frame checks) reuse one stream instead of rebuilding it.
/// `OnceLock` + a mutex keep this safe under the default parallel test runner.
fn cached_carriers(frames: usize, interleave: usize) -> Arc<CarrierStream> {
    static CACHE: OnceLock<CarrierCache> = OnceLock::new();
    let cache = CACHE.get_or_init(|| Mutex::new(HashMap::new()));
    let mut guard = cache.lock().expect("carrier cache mutex poisoned");
    guard
        .entry((frames, interleave))
        .or_insert_with(|| {
            let per_frame = packets_per_frame(MODE, MODULATION, RATE);
            let packets = build_ts_packets(frames * per_frame);
            Arc::new(synthesize_carriers(&packets, frames, interleave))
        })
        .clone()
}

/// Turn per-symbol carrier vectors into the baseband segment-rate waveform with
/// guard intervals (Fig. 3-33: a copy of the latter part of the IFFT output).
fn synthesize_waveform(carriers: &[CarrierVector]) -> Vec<Complex32> {
    let fft_size = segment_fft_size(MODE);
    let guard_len = GUARD.guard_len(fft_size);
    let total_carriers = carriers_per_segment(MODE);
    let half = total_carriers / 2;

    // Unitary inverse DFT: `x[n] = 1/sqrt(N) sum X[k] exp(+j 2 pi k n / N)`.
    let mut planner = FftPlanner::<f32>::new();
    let inverse = planner.plan_fft_inverse(fft_size);
    let scale = 1.0 / (fft_size as f32).sqrt();

    let mut out = Vec::with_capacity(carriers.len() * (fft_size + guard_len));
    for symbol in carriers {
        assert_eq!(symbol.len(), total_carriers, "carrier vector length");
        let mut spectrum = vec![Complex32::new(0.0, 0.0); fft_size];
        for (local, &value) in symbol.iter().enumerate() {
            spectrum[(local + fft_size - half) % fft_size] = value;
        }
        inverse.process(&mut spectrum);
        for value in spectrum.iter_mut() {
            *value *= scale;
        }
        out.extend_from_slice(&spectrum[fft_size - guard_len..]);
        out.extend_from_slice(&spectrum);
    }
    out
}

/// Extra segment-rate zero samples appended after the final OFDM symbol so both
/// causal FIR resamplers flush their group delay before the stream ends.
///
/// The fixture upsamples `189/160` and the receiver downsamples `160/189`.
/// Without the pad the final symbol's tail is still inside the receiver FIR when
/// the input ends, so the last frame never completes: raw-IQ validation stalled
/// at four TMCC frames instead of the expected five and dropped a whole frame's
/// TS. The pad is computed from the resamplers' own group delay (the receiver's
/// delay is expressed at the segment rate where the pad is appended) plus one
/// full prototype span of margin (`taps_per_phase`).
///
/// This is a **fixed, computed flush**, not a tunable knob. It only compensates
/// the resampler group delay at end-of-stream (EOF); it is deliberately not
/// grown to "find" more frames. Two finite-window effects set the real fixture
/// length and are handled elsewhere instead:
///
/// * The synchroniser acquires on a finite window ([`super::sync`]'s
///   `ACQUIRE_SYMBOLS` consecutive periods), so the first partial frame(s) of a
///   capture are consumed before the first TMCC frame can be validated.
/// * At EOF the receiver has no `ChannelDecoder::flush`, so the time/FEC tail is
///   retained and the last frame's TS is only partially emitted; see
///   [`tail_pipeline_packets`].
///
/// If a raw-IQ test under-counts validated frames, the correct fix is to add
/// whole transmission frames to the fixture, not to inflate this pad.
fn resampler_flush_pad_samples() -> usize {
    let fixture = RationalResampler::new(189, 160).expect("reverse ratio is valid");
    let receiver = RationalResampler::new(160, 189).expect("receiver ratio is valid");
    let fixture_delay = fixture.group_delay_input().ceil() as usize;
    let receiver_delay = (receiver.group_delay_input() * 189.0 / 160.0).ceil() as usize;
    fixture_delay + receiver_delay + 2 * fixture.taps_per_phase()
}

/// Reverse-resample the segment-rate waveform to the decoder's 1.2 MHz input
/// rate and quantise to interleaved `u8` I/Q.
///
/// The decoder upsamples 1.2 MHz -> 64/63 MHz with the reduced ratio `160/189`;
/// the exact inverse is `189/160`. The waveform is peak-normalised and then
/// scaled to [`TX_NORMALISED_SCALE`] so no sample reaches the `u8` rails. A
/// trailing zero pad (see [`resampler_flush_pad_samples`]) flushes the
/// resampler group delays so the last symbol is not truncated.
fn quantize_to_u8_iq(segment: &[Complex32]) -> Vec<u8> {
    let mut resampler = RationalResampler::new(189, 160).expect("reverse ratio is valid");
    let pad = resampler_flush_pad_samples();
    let mut flushed = Vec::with_capacity(segment.len() + pad);
    flushed.extend_from_slice(segment);
    flushed.resize(segment.len() + pad, Complex32::new(0.0, 0.0));

    let mut baseband = Vec::new();
    resampler.process(&flushed, &mut baseband);
    quantize_baseband_to_u8(&baseband)
}

/// Peak-normalise a real-valued baseband capture and quantise it to interleaved
/// `u8` I/Q, exactly as [`quantize_to_u8_iq`] does. Shared by the nominal
/// rational-resample path and the fractional sample-clock-offset path so both
/// quantise identically.
fn quantize_baseband_to_u8(baseband: &[Complex32]) -> Vec<u8> {
    assert!(!baseband.is_empty(), "resampler produced no samples");

    let peak = baseband
        .iter()
        .map(|value| value.norm())
        .fold(0.0f32, f32::max);
    assert!(peak > 0.0, "synthesized waveform is silent");
    let gain = TX_NORMALISED_SCALE / peak;

    let mut bytes = Vec::with_capacity(baseband.len() * 2);
    for value in baseband {
        for component in [value.re * gain, value.im * gain] {
            let quantised = (component * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8;
            bytes.push(quantised);
        }
    }
    assert!(
        bytes.iter().any(|&b| b != 0 && b != 255),
        "all samples railed; normalisation is broken"
    );
    bytes
}

/// Interpolate the segment-rate waveform at fractional sample positions with a
/// centred Hann-windowed sinc kernel.
///
/// The one-segment signal is band-limited to well under half the segment rate
/// (`~432` of `1024` bins), so a short windowed sinc reproduces it accurately;
/// this is the standard way to evaluate a non-integer sampling grid without the
/// artificial per-frame whole-sample insertion that would change the OFDM
/// timing discontinuously.
fn sample_windowed_sinc(x: &[Complex32], position: f64) -> Complex32 {
    /// Kernel half-width in input samples.
    const HALF: isize = 12;
    let base = position.floor();
    let frac = position - base;
    let base = base as isize;

    let mut acc = Complex32::new(0.0, 0.0);
    for tap in -HALF + 1..=HALF {
        let index = base + tap;
        if index < 0 || index as usize >= x.len() {
            continue;
        }
        let distance = frac - tap as f64;
        let sinc = if distance == 0.0 {
            1.0
        } else {
            let arg = std::f64::consts::PI * distance;
            arg.sin() / arg
        };
        // Hann window over the kernel support keeps the truncation ripple low.
        let window = if distance.abs() <= HALF as f64 {
            0.5 * (1.0 + (std::f64::consts::PI * distance / HALF as f64).cos())
        } else {
            0.0
        };
        acc += x[index as usize] * (window * sinc) as f32;
    }
    acc
}

/// Quantise the segment-rate waveform to `u8` I/Q as if the RTL-SDR sample
/// clock ran `sfo_ppm` parts-per-million off its nominal 1.2 MHz.
///
/// The receiver resamples the 1.2 MHz input to the one-segment rate with the
/// fixed ratio `160/189`. A hardware clock
/// `1.2e6 * (1 + sfo_ppm * 1e-6)` therefore presents the receiver with the
/// segment-rate waveform resampled by `1 / (1 + sfo_ppm * 1e-6)`: output sample
/// `m` lands at segment-waveform position
/// `m * (160/189) / (1 + sfo_ppm * 1e-6)`, which [`sample_windowed_sinc`]
/// evaluates. This is a continuous fractional time-warp of the whole capture,
/// not a per-frame sample insert.
///
/// The end-of-stream zero pad is the same computed resampler flush as the
/// nominal path ([`resampler_flush_pad_samples`]), so the receiver's FIR still
/// drains the final symbol.
fn quantize_to_u8_iq_at_clock(segment: &[Complex32], sfo_ppm: f64) -> Vec<u8> {
    let pad = resampler_flush_pad_samples();
    let mut flushed = Vec::with_capacity(segment.len() + pad);
    flushed.extend_from_slice(segment);
    flushed.resize(segment.len() + pad, Complex32::new(0.0, 0.0));

    let ratio = (160.0 / 189.0) / (1.0 + sfo_ppm * 1e-6);
    // Keep the final kernel half-width away from the padded end so the last
    // output sample never reads past the zero pad.
    let usable = flushed.len() as f64 - 16.0;
    let out_len = (usable / ratio).floor().max(0.0) as usize;
    let mut baseband = Vec::with_capacity(out_len);
    for m in 0..out_len {
        baseband.push(sample_windowed_sinc(&flushed, m as f64 * ratio));
    }
    quantize_baseband_to_u8(&baseband)
}

/// Apply a carrier-frequency offset of `bins` subcarriers to a segment-rate
/// waveform, **before** the reverse resample and `u8` quantisation.
///
/// One subcarrier is `1 / N` of the sample rate, so the per-sample phase advance
/// is `omega = TAU * bins / N` (Mode 3: `N = 1024`, waveform at `fs = 64/63 MHz`).
/// The rotation is applied over the **absolute** sample index, so the whole
/// capture carries one continuous phase ramp: `x[m] -> x[m] * exp(j * omega * m)`.
/// A fractional `bins` (`3.2`, `-4.15`) is therefore an integer subcarrier shift
/// plus a fractional carrier offset, exactly what the cyclic-prefix and coarse
/// integer stages have to resolve together.
///
/// The phase is accumulated in `f64`: over a multi-frame capture the absolute
/// angle reaches tens of thousands of radians, where `f32` `sin_cos` loses the
/// small per-sample increment.
fn apply_cfo_bins(waveform: &mut [Complex32], bins: f32) {
    let fft_size = segment_fft_size(MODE) as f64;
    let omega = std::f64::consts::TAU * bins as f64 / fft_size;
    for (index, value) in waveform.iter_mut().enumerate() {
        let angle = omega * index as f64;
        let (sin, cos) = angle.sin_cos();
        let (sin, cos) = (sin as f32, cos as f32);
        *value = Complex32::new(
            value.re * cos - value.im * sin,
            value.re * sin + value.im * cos,
        );
    }
}

// ---------------------------------------------------------------------------
// Assertions / diagnostics
// ---------------------------------------------------------------------------

/// End-of-stream pipeline latency, in transmission TSPs, for time-interleaving
/// `I`.
///
/// At the end of a stream (no `ChannelDecoder::flush`) the receiver still holds
/// the `I * 95` symbols resident in the time de-interleaver, plus the Viterbi
/// traceback, the bit-interleaver delay and the `BYTE_INTERLEAVE_LATENCY`-byte
/// byte-de-interleaver backlog. All of those are expressed as source bytes and
/// rounded up to whole transmission TSPs.
fn tail_pipeline_packets(interleave: usize) -> usize {
    /// K=7 sliding-window traceback depth (`coding::K7_TRACEBACK`).
    const VITERBI_TRACEBACK_BITS: usize = 64;

    let data_per_symbol = data_carriers_per_segment(MODE) * MODULATION.bits_per_symbol();
    let (num, den) = RATE.ratio();
    // Coded bits still resident in the time de-interleaver, converted back to
    // source (information) bits at the puncturing rate.
    let time_bits = time_interleave_max_delay(interleave) * data_per_symbol * num / den;
    let bit_delay = MODULATION
        .tx_bit_delays()
        .iter()
        .copied()
        .max()
        .unwrap_or(0);

    let source_bytes = (time_bits + VITERBI_TRACEBACK_BITS + bit_delay).div_ceil(8);
    (source_bytes + BYTE_INTERLEAVE_LATENCY).div_ceil(TSP_SIZE)
}

/// Assert that `decoded` is an exact, contiguous run of `packets` in
/// transmission order, and return the index of the first decoded packet.
fn assert_contiguous(decoded: &[Packet], packets: &[Packet], label: &str, interleave: usize) {
    assert!(
        !decoded.is_empty(),
        "{label}: recovered no TS packets at all (transmitted {})",
        packets.len()
    );

    let start = packets
        .iter()
        .position(|packet| packet == &decoded[0])
        .unwrap_or_else(|| {
            panic!(
                "{label}: first decoded packet is not any transmitted packet; \
                 decoder fabricated or mis-framed output"
            )
        });
    assert!(
        start >= 1,
        "{label}: the rotated partial first TSP must never be emitted"
    );

    for (index, actual) in decoded.iter().enumerate() {
        match packets.get(start + index) {
            Some(expected) => assert_eq!(
                actual,
                expected,
                "{label}: packet {index} (transmission index {}) is not contiguous",
                start + index
            ),
            None => panic!(
                "{label}: decoder emitted more packets than were transmitted \
                 ({} after start {start})",
                decoded.len()
            ),
        }
    }

    // Continuity must be intact across the recovered run.
    let mut tracker = TsContinuityTracker::new();
    for packet in decoded {
        let (_, event) = tracker
            .observe_packet(packet)
            .expect("decoded packet parses as TS");
        assert!(
            matches!(event, ContinuityEvent::First | ContinuityEvent::Incremented),
            "{label}: continuity break: {event:?}"
        );
    }

    // The run must reach essentially the end of the stream. The receiver holds
    // a deterministic end-of-stream tail (time de-interleaver + Viterbi + bit
    // and byte de-interleavers); compute it from the transmission parameters
    // rather than guessing a fixed packet count.
    let tail_pipeline = tail_pipeline_packets(interleave);
    assert!(
        decoded.len() + tail_pipeline >= packets.len() - start,
        "{label}: recovered only {} of {} decodable packets (start {start}, tail {tail_pipeline})",
        decoded.len(),
        packets.len() - start
    );
}

/// Decode the synthesized carrier vectors directly through the carrier-level
/// pipeline (`FrameCollector` -> equalise -> time de-interleave -> demap ->
/// `ChannelDecoder`), bypassing the front end / synchroniser / resampler.
///
/// This isolates a carrier/FEC failure from a front-end failure when the raw-IQ
/// test fails.
fn decode_carrier_stream(
    carriers: &[Vec<Complex32>],
    interleave: usize,
) -> (Vec<Packet>, u64, u64) {
    let config = LayerConfig {
        modulation: MODULATION,
        rate: RATE,
        interleave,
    };
    let mut collector = FrameCollector::new(MODE);
    let mut time = TimeDeinterleaver::<Complex32>::new(MODE, interleave).expect("valid I");
    let mut channel = ChannelDecoder::with_mode(MODE, config).expect("valid config");

    let mut symbol_index = 0usize;
    let mut frames = 0u64;
    let mut symbols = 0u64;
    let mut packets = Vec::new();

    for symbol in carriers {
        let frame = collector
            .push(symbol)
            .expect("fixture emits the correct carrier count");
        let Some(frame) = frame else {
            continue;
        };
        frames += 1;
        for carrier in &frame.carriers {
            let equalised = equalize_symbol(MODE, symbol_index, carrier)
                .unwrap_or_else(|error| panic!("equalise symbol {symbol_index}: {error}"));
            symbol_index += 1;
            let deinterleaved = time
                .process_symbol(&equalised)
                .expect("data carrier count matches");
            let mut soft = Vec::with_capacity(deinterleaved.len() * MODULATION.bits_per_symbol());
            for value in &deinterleaved {
                soft.extend(demap(*value, MODULATION));
            }
            packets.extend(channel.push_soft(&soft));
            symbols += 1;
        }
    }
    (packets, frames, symbols)
}

/// Decode raw `u8` IQ in `chunk_len`-byte pieces; `gap` marks the whole feed as
/// a gap. Returns the decoded packets and the decoder statistics.
fn decode_raw_iq(iq: &[u8], chunk_len: usize, gap: bool) -> (Vec<Packet>, ReceiverStats) {
    assert!(chunk_len > 0, "chunk length must be non-zero");
    let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).expect("1.2 MHz is a supported rate");
    let mut packets = Vec::new();
    let mut first = true;
    for chunk in iq.chunks(chunk_len) {
        packets.extend(
            decoder
                .push_iq(chunk, gap && first)
                .unwrap_or_else(|error| panic!("push_iq failed: {error}")),
        );
        first = false;
    }
    let stats = decoder.stats();
    (packets, stats)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn pat_section_has_zero_crc() {
    let packet = build_pat(0);
    let section = 5usize;
    assert_eq!(fec::crc32_mpeg2(&packet[section..section + 16]), 0);
    assert_eq!(packet[1] & 0x40, 0x40, "PUSI must be set");
    assert_eq!(packet[4], 0x00, "pointer_field must be zero");
}

/// The cheap, front-end-free check: the carrier/FEC chain must already recover
/// the exact contiguous TS run. This pins a failure to the carrier pipeline
/// before the raw-IQ test blames the resampler or synchroniser.
#[test]
fn carrier_level_pipeline_recovers_exact_contiguous_ts() {
    let frames = 6usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 0);

    let (decoded, tmcc_frames, symbols) = decode_carrier_stream(&carriers, 0);
    eprintln!(
        "carrier-level: tmcc_frames={tmcc_frames} symbols={symbols} ts_packets={}",
        decoded.len()
    );

    assert_contiguous(&decoded, &packets, "carrier-level", 0);
    assert!(
        tmcc_frames >= 5,
        "carrier-level: only {tmcc_frames} TMCC frames validated"
    );
    assert!(decoded.len() >= 150, "carrier-level: too few TS packets");
}

/// Full raw-IQ replay: synthesized segment waveform -> 1.2 MHz `u8` IQ ->
/// [`Decoder::push_iq`]. This is the primary integration test.
///
/// # Why eight frames
///
/// The synchroniser needs a finite acquisition window before it can lock, so
/// the leading partial frame produces no TMCC frame; TMCC is also confirmed
/// against the alternating `W0`/`W1` sync words, which costs another frame.
/// With only six transmitted frames the raw front end validates just four
/// frames (`>= 5` is the required threshold) even though the carrier-level path
/// already recovers the exact contiguous TS. Eight whole frames leave enough
/// post-acquisition margin while keeping the raw fixture finite; the assertions
/// on TMCC count and contiguous TS are intentionally not relaxed. The EOF tail
/// (`tail_pipeline_packets`) is still retained by the receiver.
#[test]
fn raw_iq_replay_decodes_exact_contiguous_ts() {
    let frames = 8usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 0);
    let waveform = synthesize_waveform(&carriers);
    let iq = quantize_to_u8_iq(&waveform);

    let (decoded, stats) = decode_raw_iq(&iq, iq.len(), false);
    eprintln!(
        "raw-IQ single push: stats={stats:?} ts_packets={}",
        decoded.len()
    );

    assert!(
        stats.tmcc_frames >= 5,
        "raw-IQ: only {} validated TMCC frames (want >= 5); stats={stats:?}",
        stats.tmcc_frames
    );
    assert!(
        stats.ts_packets > 0,
        "raw-IQ: decoder produced no RS-validated TS; stats={stats:?}"
    );
    assert_contiguous(&decoded, &packets, "raw-IQ", 0);
}

/// Raw-IQ replay with a nonzero **integer plus fractional** carrier-frequency
/// offset applied to the segment waveform before resampling / quantisation.
///
/// The offset is synthesised as `omega = TAU * bins / N` over the absolute
/// sample index as [`apply_cfo_bins`], so:
///
/// * `bins = 3.2` is integer `+3` with fractional `+0.2`;
/// * `bins = -4.15` is integer `-4` with fractional `-0.15`.
///
/// The cyclic-prefix stage sees only the fractional part, so the coarse
/// integer-CFO stage must fold in the integer subcarrier shift before a
/// coherent frame can be decoded. Both signs are exercised; the existing
/// 8-frame QPSK 1/2 `I = 0` fixture and [`assert_contiguous`] are reused
/// unchanged, only the waveform gains the CFO. Input is fed in 4096-byte chunks.
///
/// The integer correction happens within the first few tracked symbols, long
/// before the first TMCC frame is collected, so the recovered TS stays one
/// contiguous run. It does bump the synchroniser generation once (a downstream
/// continuity reset); more than that would be reset thrash rather than a single
/// correction.
#[test]
fn raw_iq_integer_plus_fractional_cfo_decodes_contiguous_ts() {
    let frames = 8usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 0);
    let base_waveform = synthesize_waveform(&carriers);

    for &bins in &[3.2f32, -4.15] {
        let mut waveform = base_waveform.clone();
        apply_cfo_bins(&mut waveform, bins);
        let iq = quantize_to_u8_iq(&waveform);

        let (decoded, stats) = decode_raw_iq(&iq, 4096, false);
        eprintln!(
            "raw-IQ CFO {bins:+} bins (chunk 4096): stats={stats:?} ts_packets={}",
            decoded.len()
        );

        assert!(
            stats.tmcc_frames >= 5,
            "raw-IQ CFO {bins:+} bins: only {} validated TMCC frames (want >= 5); stats={stats:?}",
            stats.tmcc_frames
        );
        assert!(
            stats.gaps == 0 && stats.resets_gap == 0,
            "raw-IQ CFO {bins:+} bins: unexpected gap; stats={stats:?}"
        );
        // A nonzero integer correction bumps the generation once, which the
        // receiver counts as one sync reset. More resets mean the lock is
        // thrashing instead of correcting once.
        assert!(
            stats.resets <= 1,
            "raw-IQ CFO {bins:+} bins: reset thrash ({} resets, {} sync); stats={stats:?}",
            stats.resets,
            stats.resets_sync
        );
        // The recovered run must span at least three transmission frames, not
        // merely a handful of TS packets.
        assert!(
            decoded.len() >= 3 * per_frame,
            "raw-IQ CFO {bins:+} bins: only {} TS packets (< 3 frames = {}); stats={stats:?}",
            decoded.len(),
            3 * per_frame
        );
        assert_contiguous(&decoded, &packets, &format!("raw-IQ CFO {bins:+} bins"), 0);
    }
}

/// Raw-IQ replay with a **relative sample-clock offset (SFO)** between the
/// transmitter fixture and the receiver, the practical failure mode of an
/// RTL-SDR whose crystal runs a few tens of ppm off nominal.
///
/// The fixture is the same 8-frame QPSK 1/2 `I = 0` stream used by
/// [`raw_iq_replay_decodes_exact_contiguous_ts`]. [`quantize_to_u8_iq_at_clock`]
/// warps the 1.2 MHz sampling grid by `ppm` before `u8` quantisation, so the
/// receiver's fixed `160/189` resampler produces a segment-rate waveform whose
/// symbol timing drifts monotonically. No samples are inserted or dropped per
/// symbol: the drift accumulates continuously and the synchroniser must follow
/// it with its timing tracker.
///
/// Both signs and a larger offset are exercised. The recovered TS must still be
/// the exact contiguous transmitted run, with TMCC validating at least five
/// frames, so this pins the coherent-channel / equaliser behaviour under a
/// growing timing offset rather than merely "something locks".
#[test]
fn raw_iq_sample_clock_offset_decodes_contiguous_ts() {
    let frames = 8usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 0);
    let waveform = synthesize_waveform(&carriers);

    for &ppm in &[0.0f64, 20.0, -20.0, 50.0, -50.0] {
        let iq = quantize_to_u8_iq_at_clock(&waveform, ppm);
        let (decoded, stats) = decode_raw_iq(&iq, 4096, false);
        eprintln!(
            "raw-IQ SFO {ppm:+} ppm (chunk 4096): stats={stats:?} ts_packets={}",
            decoded.len()
        );

        assert!(
            stats.tmcc_frames >= 5,
            "raw-IQ SFO {ppm:+} ppm: only {} validated TMCC frames (want >= 5); stats={stats:?}",
            stats.tmcc_frames
        );
        assert!(
            decoded.len() >= 3 * per_frame,
            "raw-IQ SFO {ppm:+} ppm: only {} TS packets (< 3 frames = {}); stats={stats:?}",
            decoded.len(),
            3 * per_frame
        );
        assert_contiguous(&decoded, &packets, &format!("raw-IQ SFO {ppm:+} ppm"), 0);
    }
}

/// Fragmented input must decode identically to a single `push_iq` call.
#[test]
fn chunked_raw_iq_replay_matches_single_push() {
    let frames = 5usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 0);
    let waveform = synthesize_waveform(&carriers);
    let iq = quantize_to_u8_iq(&waveform);

    let (reference, reference_stats) = decode_raw_iq(&iq, iq.len(), false);
    assert_contiguous(&reference, &packets, "raw-IQ contiguous reference", 0);

    // Odd chunk sizes force the retained I/Q byte to cross calls.
    for chunk_len in [4097usize, 8191, 3] {
        let (chunked, chunked_stats) = decode_raw_iq(&iq, chunk_len, false);
        eprintln!(
            "raw-IQ chunk {chunk_len}: stats={chunked_stats:?} ts_packets={}",
            chunked.len()
        );
        assert_eq!(
            chunked, reference,
            "chunk size {chunk_len} changed the decoded TS"
        );
        assert_eq!(
            chunked_stats.ts_packets, reference_stats.ts_packets,
            "chunk size {chunk_len} changed the TS packet count"
        );
    }
}

/// A gap covering at least three transmission frames must drop the lock, reset
/// the pipeline and re-acquire from the remaining signal.
#[test]
fn raw_iq_gap_reacquires_and_decodes_again() {
    let frames = 10usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 0);
    let waveform = synthesize_waveform(&carriers);
    let iq = quantize_to_u8_iq(&waveform);

    // Segment symbol count -> 1.2 MHz IQ byte count for one original frame.
    let frame_samples = SYMBOLS_PER_FRAME * segment_symbol_len(MODE, GUARD);
    let iq_bytes_per_frame = ((frame_samples as u128 * 189) / 160) as usize * 2;
    // Keep three frames (enough to lock), drop three (>= stale timeout), then
    // resume with the final four frames.
    let before = 3 * iq_bytes_per_frame;
    let after = before + 3 * iq_bytes_per_frame;

    let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).expect("1.2 MHz is supported");
    let first = decoder.push_iq(&iq[..before], false).expect("first push");
    assert!(
        decoder.is_locked(),
        "decoder did not lock on the first three frames: stats={:?}",
        decoder.stats()
    );
    assert!(!first.is_empty(), "no TS before the gap");

    // Signal a genuine gap and drop three frames' worth of samples before
    // resuming, so re-acquisition needs a fresh two-frame TMCC lock.
    let middle = decoder.push_iq(&[], true).expect("gap push");
    assert!(middle.is_empty(), "a gap must not fabricate TS");
    let resumed = decoder.push_iq(&iq[after..], false).expect("resumed push");
    let stats = decoder.stats();
    eprintln!(
        "raw-IQ gap: before={before} first={} resumed={} stats={stats:?}",
        first.len(),
        resumed.len()
    );

    assert!(stats.gaps >= 1, "gap was not counted: {stats:?}");
    assert!(
        stats.resets_gap >= 1,
        "gap did not reset the pipeline: {stats:?}"
    );
    assert!(
        decoder.is_locked(),
        "decoder did not re-acquire after the gap: stats={stats:?}"
    );

    // The resumed packets must again be a contiguous run of transmitted TS.
    assert_contiguous(&resumed, &packets, "raw-IQ gap+resume", 0);
}

/// The `I = 1` time-interleaving variant of the carrier-level check. Mode 3
/// resolves TMCC interleaving code `001` to `I = 1` (Table 3-30).
#[test]
fn carrier_level_time_interleave_i1_recovers_contiguous_ts() {
    let frames = 8usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 1);

    let (decoded, tmcc_frames, symbols) = decode_carrier_stream(&carriers, 1);
    eprintln!(
        "carrier-level I=1: tmcc_frames={tmcc_frames} symbols={symbols} ts_packets={}",
        decoded.len()
    );
    assert!(tmcc_frames >= 5, "I=1: only {tmcc_frames} TMCC frames");
    assert_contiguous(&decoded, &packets, "carrier-level I=1", 1);
}

/// Raw-IQ replay of the `I = 1` time-interleaving fixture through the full
/// front end (resampler + synchroniser + frame collector + FEC).
///
/// Mode 3 resolves TMCC interleaving code `001` to `I = 1` (Table 3-30). The
/// end-of-stream tail is not hard-coded: [`assert_contiguous`] derives it from
/// the interleave argument via [`tail_pipeline_packets`], which accounts for the
/// `I * 95` resident time-deinterleaver symbols plus the Viterbi / bit / byte
/// de-interleaver backlog. Input is fed in 4096-byte chunks.
#[test]
fn raw_iq_time_interleave_i1_decodes_contiguous_ts() {
    let frames = 8usize;
    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(frames * per_frame);
    let carriers = cached_carriers(frames, 1);
    let waveform = synthesize_waveform(&carriers);
    let iq = quantize_to_u8_iq(&waveform);

    let (decoded, stats) = decode_raw_iq(&iq, 4096, false);
    eprintln!(
        "raw-IQ I=1 (chunk 4096): stats={stats:?} ts_packets={}",
        decoded.len()
    );

    assert!(
        stats.tmcc_frames >= 5,
        "raw-IQ I=1: only {} validated TMCC frames (want >= 5); stats={stats:?}",
        stats.tmcc_frames
    );
    assert!(
        decoded.len() >= 3 * per_frame,
        "raw-IQ I=1: only {} TS packets (< 3 frames = {}); stats={stats:?}",
        decoded.len(),
        3 * per_frame
    );
    assert_contiguous(&decoded, &packets, "raw-IQ I=1", 1);
}

/// Sustained-stability regression: 48 frames of QPSK 1/2 `I = 1` with a
/// **concurrent** `+20 ppm` sample-clock offset and `+3.2`-bin RF carrier
/// offset, fed in 8192-byte chunks.
///
/// The short SFO/CFO tests pin acquisition and a few seconds of tracking; this
/// one is deliberately long enough (`~11 s` at 1.2 MHz) that a slow timing /
/// carrier drift or a TS-less stall would surface. The front-end injection is
/// the existing continuous fractional time-warp ([`quantize_to_u8_iq_at_clock`],
/// applied after [`apply_cfo_bins`]), so the drift accumulates over the whole
/// capture rather than being reset per frame.
///
/// Per-second TS output is tracked on the input byte offset. Acquisition may
/// cost the first second; after that *every* second must carry TS, and the last
/// quarter must be active, which is what catches a late `resets_ts_stall` or a
/// silent lock loss. A single coarse-CFO generation bump is allowed (as in
/// [`raw_iq_integer_plus_fractional_cfo_decodes_contiguous_ts`]); more resets
/// are thrash. The recovered run must still be the whole exact contiguous
/// transmitted sequence via [`assert_contiguous`] with the `I = 1` tail.
///
/// # Runtime
///
/// The 48-frame fixture plus the windowed-sinc resample over the full capture
/// is intentionally heavy, so the test is `#[ignore]`d and run explicitly in
/// release:
///
/// ```text
/// cargo test --release raw_iq_sustained_sfo_plus_cfo_decodes_contiguous_ts \
///     -- --ignored --nocapture --test-threads=1
/// ```
#[test]
#[ignore = "long-running stability regression; run release explicitly"]
fn raw_iq_sustained_sfo_plus_cfo_decodes_contiguous_ts() {
    const FRAMES: usize = 48;
    const SFO_PPM: f64 = 20.0;
    const CFO_BINS: f32 = 3.2;
    const CHUNK_BYTES: usize = 8192;
    /// 1.2 MHz, two bytes per complex sample.
    const BYTES_PER_SECOND: usize = DEFAULT_INPUT_RATE as usize * 2;

    let per_frame = packets_per_frame(MODE, MODULATION, RATE);
    let packets = build_ts_packets(FRAMES * per_frame);
    let carriers = cached_carriers(FRAMES, 1);

    // Concurrent front-end impairments: RF CFO first (segment waveform), then
    // the sample-clock warp at quantisation, exactly as the short tests do.
    let mut waveform = synthesize_waveform(&carriers);
    apply_cfo_bins(&mut waveform, CFO_BINS);
    let iq = quantize_to_u8_iq_at_clock(&waveform, SFO_PPM);

    let seconds = iq.len() as f64 / BYTES_PER_SECOND as f64;
    let buckets = iq.len().div_ceil(BYTES_PER_SECOND);
    let mut per_second_packets = vec![0usize; buckets];

    let mut decoder = Decoder::new(DEFAULT_INPUT_RATE).expect("1.2 MHz is supported");
    let mut decoded = Vec::new();
    let mut cursor = 0usize;
    for chunk in iq.chunks(CHUNK_BYTES) {
        let output = decoder
            .push_iq(chunk, false)
            .unwrap_or_else(|error| panic!("push_iq failed: {error}"));
        per_second_packets[cursor / BYTES_PER_SECOND] += output.len();
        decoded.extend(output);
        cursor += chunk.len();
    }
    let stats = decoder.stats();

    eprintln!(
        "raw-IQ sustained {FRAMES} frames (~{seconds:.2} s) SFO {SFO_PPM:+} ppm + CFO {CFO_BINS:+} bins \
         (chunk {CHUNK_BYTES}): stats={stats:?} ts_packets={} per_second={per_second_packets:?}",
        decoded.len()
    );

    assert!(
        stats.gaps == 0 && stats.resets_gap == 0,
        "sustained: unexpected gap; stats={stats:?}"
    );
    assert_eq!(
        stats.resets_ts_stall, 0,
        "sustained: TS-less stall under continuous input; stats={stats:?}"
    );
    // A nonzero integer/subcarrier CFO is corrected once during acquisition.
    assert!(
        stats.resets <= 1,
        "sustained: reset thrash ({} resets, {} sync); stats={stats:?}",
        stats.resets,
        stats.resets_sync
    );
    assert!(
        stats.tmcc_frames >= FRAMES as u64 - 2,
        "sustained: only {} validated TMCC frames (want >= {}); stats={stats:?}",
        stats.tmcc_frames,
        FRAMES - 2
    );

    // Acquisition may clear the first second; afterwards every second of input
    // must produce TS, and the final quarter must be active (no late stall).
    for (second, &count) in per_second_packets.iter().enumerate().skip(1) {
        assert!(
            count > 0,
            "sustained: no TS in second {second} (stall); per_second={per_second_packets:?}"
        );
    }
    let last_quarter = buckets * 3 / 4;
    let late_packets: usize = per_second_packets[last_quarter..].iter().sum();
    assert!(
        late_packets > 0,
        "sustained: final quarter produced no TS; per_second={per_second_packets:?}"
    );

    assert!(
        decoded.len() >= 3 * per_frame,
        "sustained: only {} TS packets (< 3 frames = {}); stats={stats:?}",
        decoded.len(),
        3 * per_frame
    );
    assert_contiguous(&decoded, &packets, "raw-IQ sustained SFO+CFO", 1);
}
