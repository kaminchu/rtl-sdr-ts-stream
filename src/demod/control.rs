//! ISDB-T OFDM control-plane primitives for the centre (partial-reception)
//! segment.
//!
//! Everything here is written from scratch for this crate. No GNU Radio,
//! `gr-isdbt`, Python or other existing demodulator source was consulted or
//! copied. The normative values are transcribed from the ARIB STD-B31 English
//! translation, version 2.2-E1 (the primary source used by this project; see
//! `docs/standards.md`). Every table below carries the section/table number it
//! came from.
//!
//! # Scope
//!
//! This module owns the *control plane* of a one-segment receiver:
//!
//! * **Centre-segment carrier layout** ([`cp_carrier`], [`ac1_carriers`],
//!   [`ac2_carriers`], [`tmcc_carriers`], [`scattered_pilot_carriers`],
//!   [`data_carriers`]) for the OFDM segment No. 0, in the segment-local
//!   carrier numbering of ARIB STD-B31 §3.12 (Tables 3-1, 3-14, 3-15).
//! * **Absolute vs. segment-local carrier numbering** on the 6 MHz
//!   transmission spectrum ([`segment_local_to_abs`], [`abs_to_segment_local`],
//!   [`center_segment_first_abs_carrier`]) per §3.14.1 and Fig. 3-32.
//! * **Pilot PRBS** generation (§3.13.1, Fig. 3-31, Table 3-16, Table 3-17).
//! * **Intra-segment frequency interleaving** — carrier rotation and the
//!   carrier-randomizing permutations of Table 3-13 (a)/(b)/(c), and their
//!   receiver inverses. Only the single centre segment is handled; the
//!   inter-segment interleaving of §3.11.2.1 is deliberately *not* implemented
//!   because a one-segment receiver never needs it.
//! * **Streaming time interleaving/de-interleaving** (§3.11.1, Fig. 3-23,
//!   Table 3-12): the per-carrier convolutional delay
//!   `I * ((carrier * 5) mod 96)` and the complementary de-interleaver that
//!   inverts it.
//! * **TMCC decoding** (§3.15): differential decoding of the 204 carrier bits,
//!   frame synchronisation word (w0/w1, alternating per frame), segment-type
//!   identification, the shortened (184,102) difference-set cyclic parity
//!   check, current/next information parsing and the current layer-A
//!   transmission parameters.
//!
//! # Not implemented
//!
//! * Inter-segment frequency interleaving (§3.11.2.1) and the 13-segment
//!   layout beyond the absolute/local index helpers.
//! * AC (Auxiliary Channel) payload decoding, byte/bit interleaving, energy
//!   dispersal, Viterbi and RS — those live outside this module.
//! * Any over-the-air validation. The values here are checked against the
//!   normative tables and against independently constructed reference code,
//!   not against a recorded signal.
//!
//! # Untrusted input
//!
//! Public entry points validate lengths, carrier indices and bit values and
//! never panic on malformed slices.

use std::collections::VecDeque;
use std::mem::discriminant;

use crate::dsp::{GuardInterval, Mode, OfdmParameters};
use crate::fec::PunctureRate;

use super::coding::{LayerConfig, Modulation};

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by the centre-segment control primitives (positions,
/// interleaving, PRBS).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ControlError {
    /// A symbol did not carry the expected number of carriers.
    #[error("carrier count mismatch: expected {expected}, got {got}")]
    CarrierCount { expected: usize, got: usize },
    /// An output buffer was too small.
    #[error("output buffer too small: need {needed}, got {got}")]
    OutputTooSmall { needed: usize, got: usize },
    /// A segment number outside `0..=12` was supplied.
    #[error("OFDM segment {0} is out of range 0..=12")]
    SegmentOutOfRange(usize),
    /// A segment-local carrier index was outside the mode's OFDM segment.
    #[error("segment-local carrier {local} is out of range for {mode:?}")]
    LocalCarrierOutOfRange { mode: Mode, local: usize },
    /// An absolute carrier index was outside the mode's transmission band.
    #[error("absolute carrier {k} is out of range for {mode:?}")]
    AbsoluteCarrierOutOfRange { mode: Mode, k: usize },
    /// A time-interleaving length outside the supported range.
    #[error("time-interleaving length {0} is invalid")]
    InvalidInterleave(usize),
    /// A value expected to be a bit (0/1) was not.
    #[error("invalid bit value {value} at index {index}")]
    InvalidBit { index: usize, value: u8 },
    /// A pilot amplitude selector other than 0/1 was supplied.
    #[error("invalid pilot PRBS bit {0}; expected 0 or 1")]
    InvalidPilotBit(u8),
}

/// Errors returned by TMCC decoding.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum TmccError {
    /// A value expected to be a bit (0/1) was not.
    #[error("TMCC invalid bit value {value} at index {index}")]
    InvalidBit { index: usize, value: u8 },
    /// The 16-bit synchronising word matched neither `w0` nor `w1`.
    #[error("TMCC synchronising signal 0x{0:04X} matches neither w0 nor w1")]
    SyncWord(u16),
    /// The 3-bit segment-type word was neither `111` nor `000`.
    #[error("TMCC segment type {0:#05b} is neither differential (111) nor coherent (000)")]
    SegmentType(u8),
    /// The error-correction parity check failed.
    #[error("TMCC parity check failed")]
    Parity,
    /// A parity codeword had the wrong length.
    #[error("TMCC codeword length must be {expected}, got {got}")]
    CodewordLength { expected: usize, got: usize },
    /// A field coding reserved by the standard was observed.
    #[error("TMCC reserved or unsupported field: {0}")]
    Reserved(&'static str),
    /// The system-identification bits were not `00` (terrestrial television).
    #[error("TMCC system identification {0:#04b} is not digital terrestrial television (00)")]
    UnsupportedSystem(u8),
    /// The transmission-parameter switching indicator was counting down.
    #[error("TMCC transmission-parameter switching in progress (indicator {0:#06b})")]
    ConfigSwitching(u8),
    /// The partial-reception flag was clear.
    #[error("TMCC partial-reception flag is clear; segment 0 is not a one-segment service")]
    PartialReceptionUnavailable,
    /// Partial reception was signalled but hierarchical layer A is unused.
    #[error("TMCC partial reception is set but hierarchical layer A is unused")]
    LayerAMissing,
    /// The active layer A transmission parameters are unsupported/incomplete.
    #[error("TMCC layer A uses unsupported or incomplete transmission parameters")]
    UnsupportedLayerA,
    /// A layer was internally inconsistent (for example modulation present but
    /// coding rate marked unused).
    #[error("TMCC hierarchy layer is inconsistent: {0}")]
    InconsistentLayer(&'static str),
    /// The locked transmission parameters changed.
    #[error("TMCC transmission parameters changed after lock")]
    ConfigChanged,
    /// Consecutive frames carried the same synchronising polarity.
    #[error("TMCC synchronising signal did not alternate between frames")]
    SyncNotAlternating,
}

// ---------------------------------------------------------------------------
// Carrier geometry (ARIB STD-B31 Table 3-1, §3.14.1, Fig. 3-32)
// ---------------------------------------------------------------------------

/// Number of OFDM symbols in one frame (Table 3-1: "Symbols per frame 204").
pub const SYMBOLS_PER_FRAME: usize = 204;

/// OFDM segment numbers ordered from the lowest to the highest frequency
/// (ARIB STD-B31 §3.14.1, Fig. 3-32). Segment 0 is the centre segment used by
/// partial reception.
pub const SEGMENT_ORDER_LOW_TO_HIGH: [usize; 13] = [11, 9, 7, 5, 3, 1, 0, 2, 4, 6, 8, 10, 12];

/// Total carrier count per OFDM segment (`108`, `216`, `432`; Table 3-1).
pub fn carriers_per_segment(mode: Mode) -> usize {
    match mode {
        Mode::Mode1 => 108,
        Mode::Mode2 => 216,
        Mode::Mode3 => 432,
    }
}

/// Number of *data* carriers per OFDM segment (`96`, `192`, `384`; Table 3-1).
pub fn data_carriers_per_segment(mode: Mode) -> usize {
    match mode {
        Mode::Mode1 => 96,
        Mode::Mode2 => 192,
        Mode::Mode3 => 384,
    }
}

/// Total carrier count across the whole 6 MHz band (`1405`, `2809`, `5617`;
/// Table 3-2). This is `13 * carriers_per_segment(mode) + 1`.
pub fn total_carriers(mode: Mode) -> usize {
    match mode {
        Mode::Mode1 => 1405,
        Mode::Mode2 => 2809,
        Mode::Mode3 => 5617,
    }
}

/// Position of `segment` in [`SEGMENT_ORDER_LOW_TO_HIGH`].
fn segment_order_position(segment: usize) -> Option<usize> {
    SEGMENT_ORDER_LOW_TO_HIGH.iter().position(|&s| s == segment)
}

/// Absolute carrier index `k` of local carrier `local` within `segment`.
///
/// Absolute numbering is the one defined in ARIB STD-B31 §3.14.1: number 0 is
/// carrier 0 of segment 11 (the lowest frequency), and `k` increases with
/// frequency. Segment-local numbering is the per-OFDM-segment carrier number
/// `0..carriers_per_segment(mode)` used in §3.12.
///
/// # Errors
///
/// [`ControlError::SegmentOutOfRange`] for `segment > 12` and
/// [`ControlError::LocalCarrierOutOfRange`] when `local` does not fit the
/// mode's OFDM segment.
pub fn segment_local_to_abs(
    mode: Mode,
    segment: usize,
    local: usize,
) -> Result<usize, ControlError> {
    if local >= carriers_per_segment(mode) {
        return Err(ControlError::LocalCarrierOutOfRange { mode, local });
    }
    let position =
        segment_order_position(segment).ok_or(ControlError::SegmentOutOfRange(segment))?;
    Ok(position * carriers_per_segment(mode) + local)
}

/// Split an absolute carrier index `k` into `(segment, local)`.
///
/// Returns `None` for `k` at or beyond the last in-segment carrier (the single
/// rightmost continuous carrier of Table 3-19 has no OFDM segment).
pub fn abs_to_segment_local(mode: Mode, k: usize) -> Option<(usize, usize)> {
    let per_segment = carriers_per_segment(mode);
    let position = k / per_segment;
    if position >= SEGMENT_ORDER_LOW_TO_HIGH.len() {
        return None;
    }
    Some((SEGMENT_ORDER_LOW_TO_HIGH[position], k % per_segment))
}

/// Absolute carrier index of segment-local carrier 0 of the centre segment
/// (segment No. 0), i.e. `6 * carriers_per_segment(mode)`.
pub fn center_segment_first_abs_carrier(mode: Mode) -> usize {
    6 * carriers_per_segment(mode)
}

/// Absolute carrier index of local carrier `local` of the centre segment.
///
/// # Errors
///
/// [`ControlError::LocalCarrierOutOfRange`] when `local` does not fit the
/// mode's OFDM segment.
pub fn center_local_to_abs(mode: Mode, local: usize) -> Result<usize, ControlError> {
    if local >= carriers_per_segment(mode) {
        return Err(ControlError::LocalCarrierOutOfRange { mode, local });
    }
    Ok(center_segment_first_abs_carrier(mode) + local)
}

// ---------------------------------------------------------------------------
// Centre-segment control/pilot carrier positions
// (ARIB STD-B31 Tables 3-14 and 3-15, segment No. 0 column)
// ---------------------------------------------------------------------------

/// Whether a carrier modulation is differentially demodulated (DQPSK) rather
/// than coherently (Table 3-1: "SP 9/0", "CP 0/1", "AC2 0/4").
pub fn is_differential(modulation: Modulation) -> bool {
    matches!(modulation, Modulation::Dqpsk)
}

/// CP carrier position inside the centre segment, if this modulation puts a
/// continual pilot there.
///
/// Differential segments carry one CP at segment-local carrier 0 (Table 3-14).
/// Coherent segments carry none inside the segment (Table 3-15; the shared
/// boundary CP is provided by the neighbouring differential segment, §3.12.1).
pub fn cp_carrier(modulation: Modulation) -> Option<usize> {
    if is_differential(modulation) {
        Some(0)
    } else {
        None
    }
}

/// AC1 carrier positions of the centre segment (Tables 3-14/3-15, segment 0).
///
/// Table 3-1 notes AC1 is "available in an equal number in all segments", so
/// these positions are the same for differential and coherent modulation.
pub fn ac1_carriers(mode: Mode) -> &'static [usize] {
    match mode {
        Mode::Mode1 => &[35, 79],
        Mode::Mode2 => &[98, 101, 118, 136],
        Mode::Mode3 => &[7, 89, 206, 209, 226, 244, 377, 407],
    }
}

/// AC2 carrier positions of the centre segment (Table 3-14, segment 0).
///
/// AC2 exists only for differential modulation (Table 3-1).
pub fn ac2_carriers(modulation: Modulation, mode: Mode) -> &'static [usize] {
    if !is_differential(modulation) {
        return &[];
    }
    match mode {
        Mode::Mode1 => &[3, 72, 85, 89],
        Mode::Mode2 => &[10, 30, 55, 81, 108, 111, 153, 167, 185],
        Mode::Mode3 => &[
            25, 30, 42, 104, 108, 118, 138, 163, 189, 216, 219, 261, 275, 293, 324, 327, 339, 364,
            382,
        ],
    }
}

/// TMCC carrier positions of the centre segment.
///
/// Differential modulation uses five/ten/twenty TMCC carriers (Table 3-14);
/// coherent modulation uses one/two/four (Table 3-15). The segment-0 columns
/// are transcribed below.
pub fn tmcc_carriers(modulation: Modulation, mode: Mode) -> &'static [usize] {
    match (is_differential(modulation), mode) {
        (true, Mode::Mode1) => &[49, 61, 96, 99, 104],
        (true, Mode::Mode2) => &[23, 37, 51, 68, 105, 121, 158, 178, 191, 195],
        (true, Mode::Mode3) => &[
            34, 48, 54, 70, 101, 131, 145, 159, 176, 213, 229, 266, 286, 299, 303, 349, 387, 397,
            404, 417,
        ],
        (false, Mode::Mode1) => &[49],
        (false, Mode::Mode2) => &[23, 178],
        (false, Mode::Mode3) => &[101, 131, 286, 349],
    }
}

/// Residue, modulo 12, of the scattered pilots for OFDM symbol `symbol`.
///
/// SPs are inserted once every 12 carriers and once every 4 symbols, so the SP
/// set is `{ k : k mod 12 == 3 * (symbol mod 4) }` (§3.12.2, Fig. 3-30).
pub fn scattered_pilot_residue(symbol: usize) -> usize {
    (3 * (symbol % 4)) % 12
}

/// Whether segment-local carrier `local` is an SP in `symbol`.
pub fn is_scattered_pilot(local: usize, symbol: usize) -> bool {
    local % 12 == scattered_pilot_residue(symbol)
}

/// Scattered-pilot carrier positions inside the centre segment for `symbol`.
///
/// Returns an empty vector for differential modulation, which uses no SP
/// (Table 3-1).
pub fn scattered_pilot_carriers(mode: Mode, modulation: Modulation, symbol: usize) -> Vec<usize> {
    if is_differential(modulation) {
        return Vec::new();
    }
    let residue = scattered_pilot_residue(symbol);
    (residue..carriers_per_segment(mode)).step_by(12).collect()
}

/// Data-carrier positions (segment-local numbering) of the centre segment,
/// i.e. every carrier that is not CP, SP, AC1, AC2 or TMCC.
///
/// The result always has [`data_carriers_per_segment`] entries.
pub fn data_carriers(mode: Mode, modulation: Modulation, symbol: usize) -> Vec<usize> {
    let total = carriers_per_segment(mode);
    let mut reserved = vec![false; total];
    if let Some(cp) = cp_carrier(modulation) {
        reserved[cp] = true;
    }
    for &c in ac1_carriers(mode) {
        reserved[c] = true;
    }
    for &c in ac2_carriers(modulation, mode) {
        reserved[c] = true;
    }
    for &c in tmcc_carriers(modulation, mode) {
        reserved[c] = true;
    }
    for c in scattered_pilot_carriers(mode, modulation, symbol) {
        reserved[c] = true;
    }
    (0..total).filter(|&c| !reserved[c]).collect()
}

// ---------------------------------------------------------------------------
// Pilot PRBS (ARIB STD-B31 §3.13.1, Fig. 3-31, Table 3-16, Table 3-17)
// ---------------------------------------------------------------------------

/// Initial register values of the pilot PRBS, indexed by segment number 0..12
/// (ARIB STD-B31 Table 3-16). The strings are copied verbatim; the table notes
/// they are "arranged in ascending order of bits from left to right", so the
/// leftmost character is register bit 0.
const PILOT_PRBS_INIT_MODE1: [&str; 13] = [
    "11001000010",
    "00101111010",
    "00010000100",
    "11011100101",
    "10010100000",
    "01000101110",
    "11110110000",
    "01101011110",
    "00001011000",
    "11011001111",
    "10100100111",
    "11111111111",
    "01110001001",
];
const PILOT_PRBS_INIT_MODE2: [&str; 13] = [
    "01110001001",
    "00001011000",
    "00000100100",
    "10010100000",
    "00100011001",
    "11001000010",
    "01100111001",
    "11011100101",
    "11100110110",
    "01101011110",
    "00101010001",
    "11111111111",
    "00100001011",
];
const PILOT_PRBS_INIT_MODE3: [&str; 13] = [
    "00100001011",
    "11100110110",
    "11100111101",
    "00100011001",
    "01101010011",
    "01110001001",
    "10111010010",
    "10010100000",
    "01100010010",
    "11011100101",
    "11110100101",
    "11111111111",
    "00010011100",
];

/// Initial-value strings for `mode` (indexed by segment number).
fn pilot_prbs_init_table(mode: Mode) -> &'static [&'static str; 13] {
    match mode {
        Mode::Mode1 => &PILOT_PRBS_INIT_MODE1,
        Mode::Mode2 => &PILOT_PRBS_INIT_MODE2,
        Mode::Mode3 => &PILOT_PRBS_INIT_MODE3,
    }
}

/// Parse an 11-character initial-value string into a register, leftmost
/// character = bit 0.
fn parse_prbs_init(value: &str) -> u16 {
    debug_assert_eq!(value.len(), 11);
    let mut register = 0u16;
    for (i, ch) in value.chars().enumerate() {
        if ch == '1' {
            register |= 1 << i;
        }
    }
    register
}

/// The 11-bit pilot PRBS generator of Fig. 3-31
/// (`g(x) = x^11 + x^9 + 1`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PilotPrbs {
    register: u16,
}

impl PilotPrbs {
    /// Build the generator for `segment` of `mode` with the Table 3-16 initial
    /// value. Returns `None` for `segment > 12`.
    pub fn for_segment(mode: Mode, segment: usize) -> Option<Self> {
        let initial = pilot_prbs_init_table(mode).get(segment)?;
        Some(Self {
            register: parse_prbs_init(initial),
        })
    }

    /// The centre segment (segment No. 0) generator.
    pub fn center(mode: Mode) -> Self {
        Self::for_segment(mode, 0).expect("segment 0 always exists")
    }

    /// Current 11-bit register state.
    pub fn state(&self) -> u16 {
        self.register
    }

    /// Produce the next output bit `W_i` and advance the register.
    ///
    /// The feedback tap is `x^11 + x^9 + 1`; the output is the bit shifted out
    /// of stage 11.
    pub fn next_bit(&mut self) -> u8 {
        let output = ((self.register >> 10) & 1) as u8;
        let feedback = ((self.register >> 10) ^ (self.register >> 8)) & 1;
        self.register = ((self.register << 1) | feedback) & 0x07FF;
        output
    }

    /// `W_i` for carrier `i` of `segment` of `mode`.
    pub fn bit_at(mode: Mode, segment: usize, index: usize) -> Option<u8> {
        let mut generator = Self::for_segment(mode, segment)?;
        let mut bit = 0;
        for _ in 0..=index {
            bit = generator.next_bit();
        }
        Some(bit)
    }
}

/// BPSK modulating signal `(I, Q)` for pilot PRBS bit `w`
/// (ARIB STD-B31 Table 3-17): `W=1 -> (-4/3, 0)`, `W=0 -> (+4/3, 0)`.
pub fn pilot_modulating_signal(w: u8) -> Result<(f32, f32), ControlError> {
    match w {
        0 => Ok((4.0 / 3.0, 0.0)),
        1 => Ok((-4.0 / 3.0, 0.0)),
        other => Err(ControlError::InvalidPilotBit(other)),
    }
}

// ---------------------------------------------------------------------------
// Intra-segment frequency interleaving
// (ARIB STD-B31 §3.11.2.2, Table 3-13 (a)/(b)/(c))
// ---------------------------------------------------------------------------

/// Carrier-randomizing permutation for Mode 1, Table 3-13 (a).
///
/// `FREQ_RANDOMIZE_MODE1[before] = after`: the symbol at position `before` of
/// the carrier-rotated sequence is assigned to carrier `after`.
const FREQ_RANDOMIZE_MODE1: [usize; 96] = [
    80, 93, 63, 92, 94, 55, 17, 81, 6, 51, 9, 85, 89, 65, 52, 15, 73, 66, 46, 71, 12, 70, 18, 13,
    95, 34, 1, 38, 78, 59, 91, 64, 0, 28, 11, 4, 45, 35, 16, 7, 48, 22, 23, 77, 56, 19, 8, 36, 39,
    61, 21, 3, 26, 69, 67, 20, 74, 86, 72, 25, 31, 5, 49, 42, 54, 87, 43, 60, 29, 2, 76, 84, 83,
    40, 14, 79, 27, 57, 44, 37, 30, 68, 47, 88, 75, 41, 90, 10, 33, 32, 62, 50, 58, 82, 53, 24,
];

/// Carrier-randomizing permutation for Mode 2, Table 3-13 (b).
const FREQ_RANDOMIZE_MODE2: [usize; 192] = [
    98, 35, 67, 116, 135, 17, 5, 93, 73, 168, 54, 143, 43, 74, 165, 48, 37, 69, 154, 150, 107, 76,
    176, 79, 175, 36, 28, 78, 47, 128, 94, 163, 184, 72, 142, 2, 86, 14, 130, 151, 114, 68, 46,
    183, 122, 112, 180, 42, 105, 97, 33, 134, 177, 84, 170, 45, 187, 38, 167, 10, 189, 51, 117,
    156, 161, 25, 89, 125, 139, 24, 19, 57, 71, 39, 77, 191, 88, 85, 0, 162, 181, 113, 140, 61, 75,
    82, 101, 174, 118, 20, 136, 3, 121, 190, 120, 92, 160, 52, 153, 127, 65, 60, 133, 147, 131, 87,
    22, 58, 100, 111, 141, 83, 49, 132, 12, 155, 146, 102, 164, 66, 1, 62, 178, 15, 182, 96, 80,
    119, 23, 6, 166, 56, 99, 123, 138, 137, 21, 145, 185, 18, 70, 129, 95, 90, 149, 109, 124, 50,
    11, 152, 4, 31, 172, 40, 13, 32, 55, 159, 41, 8, 7, 144, 16, 26, 173, 81, 44, 103, 64, 9, 30,
    157, 126, 179, 148, 63, 188, 171, 106, 104, 158, 115, 34, 186, 29, 108, 53, 91, 169, 110, 27,
    59,
];

/// Carrier-randomizing permutation for Mode 3, Table 3-13 (c).
const FREQ_RANDOMIZE_MODE3: [usize; 384] = [
    62, 13, 371, 11, 285, 336, 365, 220, 226, 92, 56, 46, 120, 175, 298, 352, 172, 235, 53, 164,
    368, 187, 125, 82, 5, 45, 173, 258, 135, 182, 141, 273, 126, 264, 286, 88, 233, 61, 249, 367,
    310, 179, 155, 57, 123, 208, 14, 227, 100, 311, 205, 79, 184, 185, 328, 77, 115, 277, 112, 20,
    199, 178, 143, 152, 215, 204, 139, 234, 358, 192, 309, 183, 81, 129, 256, 314, 101, 43, 97,
    324, 142, 157, 90, 214, 102, 29, 303, 363, 261, 31, 22, 52, 305, 301, 293, 177, 116, 296, 85,
    196, 191, 114, 58, 198, 16, 167, 145, 119, 245, 113, 295, 193, 232, 17, 108, 283, 246, 64, 237,
    189, 128, 373, 302, 320, 239, 335, 356, 39, 347, 351, 73, 158, 276, 243, 99, 38, 287, 3, 330,
    153, 315, 117, 289, 213, 210, 149, 383, 337, 339, 151, 241, 321, 217, 30, 334, 161, 322, 49,
    176, 359, 12, 346, 60, 28, 229, 265, 288, 225, 382, 59, 181, 170, 319, 341, 86, 251, 133, 344,
    361, 109, 44, 369, 268, 257, 323, 55, 317, 381, 121, 360, 260, 275, 190, 19, 63, 18, 248, 9,
    240, 211, 150, 230, 332, 231, 71, 255, 350, 355, 83, 87, 154, 218, 138, 269, 348, 130, 160,
    278, 377, 216, 236, 308, 223, 254, 25, 98, 300, 201, 137, 219, 36, 325, 124, 66, 353, 169, 21,
    35, 107, 50, 106, 333, 326, 262, 252, 271, 263, 372, 136, 0, 366, 206, 159, 122, 188, 6, 284,
    96, 26, 200, 197, 186, 345, 340, 349, 103, 84, 228, 212, 2, 67, 318, 1, 74, 342, 166, 194, 33,
    68, 267, 111, 118, 140, 195, 105, 202, 291, 259, 23, 171, 65, 281, 24, 165, 8, 94, 222, 331,
    34, 238, 364, 376, 266, 89, 80, 253, 163, 280, 247, 4, 362, 379, 290, 279, 54, 78, 180, 72,
    316, 282, 131, 207, 343, 370, 306, 221, 132, 7, 148, 299, 168, 224, 48, 47, 357, 313, 75, 104,
    70, 147, 40, 110, 374, 69, 146, 37, 375, 354, 174, 41, 32, 304, 307, 312, 15, 272, 134, 242,
    203, 209, 380, 162, 297, 327, 10, 93, 42, 250, 156, 338, 292, 144, 378, 294, 329, 127, 270, 76,
    95, 91, 244, 274, 27, 51,
];

/// The normative carrier-randomizing permutation (`before -> after`) for `mode`
/// (Table 3-13). This is a copy of the standard's table, not a generator.
pub fn frequency_randomize_permutation(mode: Mode) -> &'static [usize] {
    match mode {
        Mode::Mode1 => &FREQ_RANDOMIZE_MODE1,
        Mode::Mode2 => &FREQ_RANDOMIZE_MODE2,
        Mode::Mode3 => &FREQ_RANDOMIZE_MODE3,
    }
}

/// Apply the intra-segment frequency interleaver of the centre segment.
///
/// `input` is the data-carrier sequence in logical order (`data_carriers(
/// mode).len()` entries); the output is indexed by segment-local *data* carrier
/// number. The centre segment uses rotation by segment 0, which is the
/// identity, so this is exactly the Table 3-13 randomizing step.
///
/// # Errors
///
/// [`ControlError::CarrierCount`] when `input.len()` differs from
/// [`data_carriers_per_segment`], or [`ControlError::OutputTooSmall`] when
/// `output` is too short.
pub fn frequency_interleave<T: Copy>(
    mode: Mode,
    input: &[T],
    output: &mut [T],
) -> Result<(), ControlError> {
    let n = data_carriers_per_segment(mode);
    if input.len() != n {
        return Err(ControlError::CarrierCount {
            expected: n,
            got: input.len(),
        });
    }
    if output.len() < n {
        return Err(ControlError::OutputTooSmall {
            needed: n,
            got: output.len(),
        });
    }
    for (before, &value) in input.iter().enumerate() {
        output[frequency_randomize_permutation(mode)[before]] = value;
    }
    Ok(())
}

/// Apply the inverse (receiver) intra-segment frequency de-interleaver.
///
/// `input` is indexed by segment-local data carrier number; `output` receives
/// the logical data order. `output[i] = input[TABLE_3_13(mode)[i]]`.
///
/// # Errors
///
/// As for [`frequency_interleave`].
pub fn frequency_deinterleave<T: Copy>(
    mode: Mode,
    input: &[T],
    output: &mut [T],
) -> Result<(), ControlError> {
    let n = data_carriers_per_segment(mode);
    if input.len() != n {
        return Err(ControlError::CarrierCount {
            expected: n,
            got: input.len(),
        });
    }
    if output.len() < n {
        return Err(ControlError::OutputTooSmall {
            needed: n,
            got: output.len(),
        });
    }
    let permutation = frequency_randomize_permutation(mode);
    for (i, slot) in output.iter_mut().take(n).enumerate() {
        *slot = input[permutation[i]];
    }
    Ok(())
}

/// Receiver mapping from segment-local data-carrier number to logical index.
///
/// `mapping()[c]` is the logical data index carried by segment-local data
/// carrier `c`; equivalently `data[mapping()[c]] == carriers[c]`. This is the
/// exact inverse of the transmitter's `before -> after` table
/// (`mapping()[after] == before`).
pub fn frequency_deinterleave_permutation(mode: Mode) -> Vec<usize> {
    let table = frequency_randomize_permutation(mode);
    let mut inverse = vec![0usize; table.len()];
    for (before, &after) in table.iter().enumerate() {
        inverse[after] = before;
    }
    inverse
}

// ---------------------------------------------------------------------------
// Streaming time interleaving (§3.11.1, Fig. 3-23, Table 3-12)
// ---------------------------------------------------------------------------

/// Modulus of the per-carrier delay pattern. ARIB STD-B31 Fig. 3-23 states
/// `m_i = (i * 5) mod 96` for all modes (the figure is shared for Modes 1/2/3;
/// the number of branches `nc` is 96/192/384). Table 3-12 confirms the maximum
/// delayed-symbol count is `I * 95` in every mode.
pub const TIME_INTERLEAVE_MODULUS: usize = 96;

/// Largest time-interleaving length accepted by the constructors. The standard
/// selects `I` from `{0, 1, 2, 4, 8, 16}` per mode; 16 is the global maximum.
pub const MAX_TIME_INTERLEAVE: usize = 16;

/// Transmitter delay (in OFDM symbols) of data carrier `carrier`:
/// `I * ((carrier * 5) mod 96)` (Fig. 3-23).
///
/// `mode` does not change the formula (the figure is shared by all modes and
/// Table 3-12 confirms a maximum delay of `I * 95` in every mode); it is kept
/// in the signature so call sites cannot silently mix up the carrier domain.
pub fn time_interleave_delay(_mode: Mode, interleave: usize, carrier: usize) -> usize {
    interleave * ((carrier * 5) % TIME_INTERLEAVE_MODULUS)
}

/// Maximum transmitter delay over all branches: `I * 95`.
pub fn time_interleave_max_delay(interleave: usize) -> usize {
    interleave * (TIME_INTERLEAVE_MODULUS - 1)
}

/// Receiver de-interleaver delay of carrier `carrier`:
/// `I * 95 - I * ((carrier * 5) mod 96)`, so that the total latency is `I * 95`
/// symbols for every carrier.
pub fn time_deinterleave_delay(mode: Mode, interleave: usize, carrier: usize) -> usize {
    time_interleave_max_delay(interleave)
        .saturating_sub(time_interleave_delay(mode, interleave, carrier))
}

/// Duration of one OFDM symbol (effective symbol plus guard interval).
pub fn symbol_duration_samples(mode: Mode, guard: GuardInterval) -> usize {
    OfdmParameters::new(mode, guard).symbol_len()
}

/// Total time-interleaver latency expressed in baseband samples.
pub fn time_interleave_latency_samples(
    mode: Mode,
    guard: GuardInterval,
    interleave: usize,
) -> usize {
    time_interleave_max_delay(interleave).saturating_mul(symbol_duration_samples(mode, guard))
}

fn validate_interleave(interleave: usize) -> Result<(), ControlError> {
    if interleave > MAX_TIME_INTERLEAVE {
        return Err(ControlError::InvalidInterleave(interleave));
    }
    Ok(())
}

/// Streaming per-carrier convolutional time interleaver (transmitter side).
///
/// Each OFDM symbol carries [`data_carriers_per_segment`] values. Carrier `i`
/// is delayed by [`time_interleave_delay`] symbols; before enough symbols have
/// been seen, `T::default()` is emitted.
#[derive(Debug, Clone)]
pub struct TimeInterleaver<T> {
    lines: Vec<VecDeque<T>>,
    delay: Vec<usize>,
}

impl<T: Copy + Default> TimeInterleaver<T> {
    /// Build a transmitter for `mode` with time-interleaving length `I`.
    ///
    /// # Errors
    ///
    /// [`ControlError::InvalidInterleave`] when `I > MAX_TIME_INTERLEAVE`.
    pub fn new(mode: Mode, interleave: usize) -> Result<Self, ControlError> {
        validate_interleave(interleave)?;
        let n = data_carriers_per_segment(mode);
        let delay = (0..n)
            .map(|carrier| time_interleave_delay(mode, interleave, carrier))
            .collect();
        Ok(Self {
            lines: vec![VecDeque::new(); n],
            delay,
        })
    }

    /// Number of carriers per symbol.
    pub fn carriers(&self) -> usize {
        self.lines.len()
    }

    /// Transmitter delay of `carrier`.
    pub fn delay_of(&self, carrier: usize) -> Option<usize> {
        self.delay.get(carrier).copied()
    }

    /// Delay one OFDM symbol's carriers and return the interleaved symbol.
    ///
    /// # Errors
    ///
    /// [`ControlError::CarrierCount`] when `input.len()` is not the data-carrier
    /// count.
    pub fn process_symbol(&mut self, input: &[T]) -> Result<Vec<T>, ControlError> {
        process_delay_bank(&mut self.lines, &self.delay, input)
    }

    /// Drop all buffered state.
    pub fn reset(&mut self) {
        for line in &mut self.lines {
            line.clear();
        }
    }
}

/// Streaming per-carrier convolutional time de-interleaver (receiver side).
///
/// This is the inverse of [`TimeInterleaver`]: each carrier is delayed by
/// `I * 95 - D_i`, so after a warm-up of `I * 95` symbols the output equals the
/// transmitter's input delayed by exactly `I * 95` symbols.
#[derive(Debug, Clone)]
pub struct TimeDeinterleaver<T> {
    lines: Vec<VecDeque<T>>,
    delay: Vec<usize>,
}

impl<T: Copy + Default> TimeDeinterleaver<T> {
    /// Build a receiver for `mode` with time-interleaving length `I`.
    ///
    /// # Errors
    ///
    /// [`ControlError::InvalidInterleave`] when `I > MAX_TIME_INTERLEAVE`.
    pub fn new(mode: Mode, interleave: usize) -> Result<Self, ControlError> {
        validate_interleave(interleave)?;
        let n = data_carriers_per_segment(mode);
        let delay = (0..n)
            .map(|carrier| time_deinterleave_delay(mode, interleave, carrier))
            .collect();
        Ok(Self {
            lines: vec![VecDeque::new(); n],
            delay,
        })
    }

    /// Number of carriers per symbol.
    pub fn carriers(&self) -> usize {
        self.lines.len()
    }

    /// Receiver delay of `carrier`.
    pub fn delay_of(&self, carrier: usize) -> Option<usize> {
        self.delay.get(carrier).copied()
    }

    /// Total latency in OFDM symbols (`I * 95`).
    pub fn latency_symbols(&self) -> usize {
        self.delay.iter().copied().max().unwrap_or(0)
    }

    /// De-interleave one OFDM symbol's carriers.
    ///
    /// # Errors
    ///
    /// [`ControlError::CarrierCount`] when `input.len()` is not the data-carrier
    /// count.
    pub fn process_symbol(&mut self, input: &[T]) -> Result<Vec<T>, ControlError> {
        process_delay_bank(&mut self.lines, &self.delay, input)
    }

    /// Drop all buffered state.
    pub fn reset(&mut self) {
        for line in &mut self.lines {
            line.clear();
        }
    }
}

fn process_delay_bank<T: Copy + Default>(
    lines: &mut [VecDeque<T>],
    delay: &[usize],
    input: &[T],
) -> Result<Vec<T>, ControlError> {
    if input.len() != lines.len() {
        return Err(ControlError::CarrierCount {
            expected: lines.len(),
            got: input.len(),
        });
    }
    let mut output = Vec::with_capacity(lines.len());
    for (i, &value) in input.iter().enumerate() {
        lines[i].push_back(value);
        if lines[i].len() > delay[i] {
            // `len > delay >= 0` guarantees a front element.
            output.push(lines[i].pop_front().expect("delay line is non-empty"));
        } else {
            output.push(T::default());
        }
    }
    Ok(output)
}

// ---------------------------------------------------------------------------
// TMCC signal (ARIB STD-B31 §3.15)
// ---------------------------------------------------------------------------

/// Total TMCC carrier bits per frame (Table 3-20).
pub const TMCC_TOTAL_BITS: usize = 204;
/// Number of TMCC information bits `B20..B121`.
pub const TMCC_INFO_BITS: usize = 102;
/// Number of parity bits `B122..B203`.
pub const TMCC_PARITY_BITS: usize = 82;
/// Length of the shortened (184,102) codeword `B20..B203`.
pub const TMCC_CODEWORD_BITS: usize = TMCC_INFO_BITS + TMCC_PARITY_BITS;
/// Index of the synchronising signal `B1`.
pub const TMCC_SYNC_OFFSET: usize = 1;
/// Index of the segment-type identification `B17`.
pub const TMCC_SEGMENT_TYPE_OFFSET: usize = 17;
/// Index of the TMCC information `B20`.
pub const TMCC_INFO_OFFSET: usize = 20;

/// Synchronising signal `w0` (§3.15.4), `B1` as the most-significant bit.
pub const TMCC_SYNC_W0: u16 = 0b0011_0101_1110_1110;
/// Synchronising signal `w1` = bitwise inverse of `w0` (§3.15.4).
pub const TMCC_SYNC_W1: u16 = 0b1100_1010_0001_0001;

/// Generator polynomial of the (273,191) difference-set cyclic code whose
/// shortened (184,102) code protects `B20..B121` (§3.15.6.9):
/// `x^82 + x^77 + x^76 + x^71 + x^67 + x^66 + x^56 + x^52 + x^48 + x^40 +
/// x^36 + x^34 + x^24 + x^22 + x^18 + x^10 + x^4 + 1`.
pub const TMCC_GENERATOR: u128 = (1u128 << 82)
    | (1 << 77)
    | (1 << 76)
    | (1 << 71)
    | (1 << 67)
    | (1 << 66)
    | (1 << 56)
    | (1 << 52)
    | (1 << 48)
    | (1 << 40)
    | (1 << 36)
    | (1 << 34)
    | (1 << 24)
    | (1 << 22)
    | (1 << 18)
    | (1 << 10)
    | (1 << 4)
    | 1;

const TMCC_GENERATOR_MASK: u128 = (1u128 << 82) - 1;

/// One of the two synchronising words, indicating the OFDM frame phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TmccSync {
    /// `w0 = 0011010111101110`.
    W0,
    /// `w1 = 1100101000010001`.
    W1,
}

impl TmccSync {
    /// The 16-bit word (`B1` as MSB).
    pub fn word(self) -> u16 {
        match self {
            TmccSync::W0 => TMCC_SYNC_W0,
            TmccSync::W1 => TMCC_SYNC_W1,
        }
    }

    /// The other phase.
    pub fn invert(self) -> Self {
        match self {
            TmccSync::W0 => TmccSync::W1,
            TmccSync::W1 => TmccSync::W0,
        }
    }
}

/// Segment-type identification `B17..B19` (§3.15.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SegmentType {
    /// `111`: the segment uses differential modulation (DQPSK).
    Differential,
    /// `000`: the segment uses coherent modulation (QPSK/16QAM/64QAM).
    Coherent,
}

/// Layer index of the TMCC current/next information.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Layer {
    /// Hierarchical layer A (`B28..B40` / `B68..B80`).
    A,
    /// Hierarchical layer B (`B41..B53` / `B81..B93`).
    B,
    /// Hierarchical layer C (`B54..B66` / `B94..B106`).
    C,
}

impl Layer {
    /// Index `0/1/2` for A/B/C.
    pub fn index(self) -> usize {
        match self {
            Layer::A => 0,
            Layer::B => 1,
            Layer::C => 2,
        }
    }
}

/// Decoded transmission parameters of one hierarchical layer (Table 3-23).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TmccLayer {
    /// Carrier modulation, or `None` for an unused layer (code `111`).
    pub modulation: Option<Modulation>,
    /// Convolutional coding rate, or `None` for an unused layer (code `111`).
    pub rate: Option<PunctureRate>,
    /// Raw 3-bit time-interleaving-length code.
    pub interleave_code: u8,
    /// Number of segments, or `None` for an unused layer (code `1111`).
    pub segments: Option<u8>,
}

impl TmccLayer {
    /// Whether the layer is marked unused.
    pub fn is_unused(&self) -> bool {
        self.modulation.is_none()
    }

    /// Time-interleaving length `I` for this layer in `mode` (Table 3-30).
    pub fn interleave_value(&self, mode: Mode) -> Option<usize> {
        tmcc_interleave_value(mode, self.interleave_code)
    }
}

/// Decoded TMCC information (Tables 3-20 and 3-22).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TmccInfo {
    /// Frame phase indicated by `B1..B16`.
    pub sync: TmccSync,
    /// Segment type indicated by `B17..B19`.
    pub segment_type: SegmentType,
    /// Raw system-identification bits `B20..B21` (Table 3-24).
    pub system: u8,
    /// Raw transmission-parameter switching indicator `B22..B25` (Table 3-25).
    pub switching: u8,
    /// Startup control signal `B26` (Table 3-26).
    pub emergency: bool,
    /// Partial-reception flag `B27` (Table 3-27).
    pub partial_reception: bool,
    /// Current transmission parameters for layers A, B and C.
    pub current: [TmccLayer; 3],
    /// Next-information partial-reception flag `B67`.
    pub next_partial_reception: bool,
    /// Next transmission parameters for layers A, B and C.
    pub next: [TmccLayer; 3],
    /// Phase-shift-correction value `B107..B109`.
    pub phase_shift_correction: u8,
}

impl TmccInfo {
    /// Current transmission parameters of `layer`.
    pub fn current_layer(&self, layer: Layer) -> &TmccLayer {
        &self.current[layer.index()]
    }

    /// Next transmission parameters of `layer`.
    pub fn next_layer(&self, layer: Layer) -> &TmccLayer {
        &self.next[layer.index()]
    }

    /// The current layer-A decoding configuration.
    ///
    /// `mode` is required because the 3-bit interleaving code resolves to a
    /// different `I` per mode (Table 3-30).
    ///
    /// # Errors
    ///
    /// [`TmccError::LayerAMissing`] when layer A is unused, and
    /// [`TmccError::UnsupportedLayerA`] when a component is absent.
    pub fn current_layer_a(&self, mode: Mode) -> Result<LayerConfig, TmccError> {
        let layer = &self.current[0];
        let modulation = layer.modulation.ok_or(TmccError::LayerAMissing)?;
        let rate = layer.rate.ok_or(TmccError::UnsupportedLayerA)?;
        let interleave = layer
            .interleave_value(mode)
            .ok_or(TmccError::UnsupportedLayerA)?;
        Ok(LayerConfig {
            modulation,
            rate,
            interleave,
        })
    }
}

/// A TMCC frame decoded and validated for one-segment reception.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TmccOneSeg {
    /// The full decoded TMCC information.
    pub info: TmccInfo,
    /// The current layer-A configuration used for the centre segment.
    pub layer_a: LayerConfig,
}

fn bits_to_u16(bits: &[u8], start: usize, len: usize) -> u16 {
    bits.iter()
        .skip(start)
        .take(len)
        .fold(0u16, |acc, &bit| (acc << 1) | (bit & 1) as u16)
}

/// Resolve a Table 3-30 time-interleaving code to `I`.
pub fn tmcc_interleave_value(mode: Mode, code: u8) -> Option<usize> {
    match (mode, code) {
        (_, 0b000) => Some(0),
        (Mode::Mode1, 0b001) => Some(4),
        (Mode::Mode2, 0b001) => Some(2),
        (Mode::Mode3, 0b001) => Some(1),
        (Mode::Mode1, 0b010) => Some(8),
        (Mode::Mode2, 0b010) => Some(4),
        (Mode::Mode3, 0b010) => Some(2),
        (Mode::Mode1, 0b011) => Some(16),
        (Mode::Mode2, 0b011) => Some(8),
        (Mode::Mode3, 0b011) => Some(4),
        _ => None,
    }
}

fn modulation_from_code(code: u8) -> Result<Option<Modulation>, TmccError> {
    Ok(match code {
        0b000 => Some(Modulation::Dqpsk),
        0b001 => Some(Modulation::Qpsk),
        0b010 => Some(Modulation::Qam16),
        0b011 => Some(Modulation::Qam64),
        0b111 => None,
        _ => return Err(TmccError::Reserved("carrier modulation mapping scheme")),
    })
}

fn rate_from_code(code: u8) -> Result<Option<PunctureRate>, TmccError> {
    Ok(match code {
        0b000 => Some(PunctureRate::Rate1_2),
        0b001 => Some(PunctureRate::Rate2_3),
        0b010 => Some(PunctureRate::Rate3_4),
        0b011 => Some(PunctureRate::Rate5_6),
        0b100 => Some(PunctureRate::Rate7_8),
        0b111 => None,
        _ => return Err(TmccError::Reserved("convolutional coding rate")),
    })
}

fn segments_from_code(code: u8) -> Result<Option<u8>, TmccError> {
    Ok(match code {
        0b0000 | 0b1110 => return Err(TmccError::Reserved("number of segments")),
        0b1111 => None,
        n => Some(n),
    })
}

fn parse_layer(bits: &[u8], start: usize) -> Result<TmccLayer, TmccError> {
    let modulation = modulation_from_code(bits_to_u16(bits, start, 3) as u8)?;
    let rate = rate_from_code(bits_to_u16(bits, start + 3, 3) as u8)?;
    let interleave_code = bits_to_u16(bits, start + 6, 3) as u8;
    if !matches!(interleave_code, 0b000..=0b011 | 0b111) {
        return Err(TmccError::Reserved("time interleaving length"));
    }
    let segments = segments_from_code(bits_to_u16(bits, start + 9, 4) as u8)?;

    // An unused layer must have every field marked unused; an active layer must
    // have modulation, rate and segment count all present.
    let unused = modulation.is_none();
    if unused != rate.is_none()
        || unused != segments.is_none()
        || unused != (interleave_code == 0b111)
    {
        return Err(TmccError::InconsistentLayer(
            "modulation/rate/interleaving/segments do not agree on used/unused",
        ));
    }
    Ok(TmccLayer {
        modulation,
        rate,
        interleave_code,
        segments,
    })
}

/// Differentially decode the 204 received TMCC carrier bits `B'0..B'203`.
///
/// `reference` is the known reference `W_i` (`B0`). The returned array is
/// `B0..B203` with `B0 = reference` and `B_k = B'_k XOR B'_{k-1}` (§3.13.3).
///
/// # Errors
///
/// [`TmccError::InvalidBit`] when `reference` or any `raw` element is not 0/1.
pub fn tmcc_differential_decode(
    reference: u8,
    raw: &[u8; TMCC_TOTAL_BITS],
) -> Result<[u8; TMCC_TOTAL_BITS], TmccError> {
    if reference > 1 {
        return Err(TmccError::InvalidBit {
            index: 0,
            value: reference,
        });
    }
    for (index, &value) in raw.iter().enumerate() {
        if value > 1 {
            return Err(TmccError::InvalidBit { index, value });
        }
    }
    let mut bits = [0u8; TMCC_TOTAL_BITS];
    bits[0] = reference;
    for k in 1..TMCC_TOTAL_BITS {
        bits[k] = raw[k] ^ raw[k - 1];
    }
    Ok(bits)
}

/// Encode 102 TMCC information bits into the 184-bit codeword `B20..B203`
/// using the shortened (184,102) difference-set cyclic code (§3.15.6.9).
///
/// Bit `i` of `info` is `B(20+i)`; the most-significant polynomial coefficient
/// is transmitted first. The 82 parity bits are appended.
///
/// # Errors
///
/// [`TmccError::CodewordLength`] when `info.len() != 102`, and
/// [`TmccError::InvalidBit`] when an element is not 0/1.
pub fn tmcc_encode(info: &[u8]) -> Result<[u8; TMCC_CODEWORD_BITS], TmccError> {
    if info.len() != TMCC_INFO_BITS {
        return Err(TmccError::CodewordLength {
            expected: TMCC_INFO_BITS,
            got: info.len(),
        });
    }
    let mut remainder = 0u128;
    for (index, &bit) in info.iter().enumerate() {
        if bit > 1 {
            return Err(TmccError::InvalidBit { index, value: bit });
        }
        let feedback = ((remainder >> 81) & 1) as u8 ^ bit;
        remainder = (remainder << 1) & TMCC_GENERATOR_MASK;
        if feedback == 1 {
            remainder ^= TMCC_GENERATOR & TMCC_GENERATOR_MASK;
        }
    }

    let mut codeword = [0u8; TMCC_CODEWORD_BITS];
    codeword[..TMCC_INFO_BITS].copy_from_slice(info);
    for j in 0..TMCC_PARITY_BITS {
        codeword[TMCC_INFO_BITS + j] = ((remainder >> (81 - j)) & 1) as u8;
    }
    Ok(codeword)
}

/// Check the parity of a 184-bit TMCC codeword `B20..B203`.
///
/// # Errors
///
/// [`TmccError::CodewordLength`] when `codeword.len() != 184`, and
/// [`TmccError::InvalidBit`] when an element is not 0/1.
pub fn tmcc_parity_valid(codeword: &[u8]) -> Result<bool, TmccError> {
    if codeword.len() != TMCC_CODEWORD_BITS {
        return Err(TmccError::CodewordLength {
            expected: TMCC_CODEWORD_BITS,
            got: codeword.len(),
        });
    }
    let mut remainder = 0u128;
    for (index, &bit) in codeword.iter().enumerate() {
        if bit > 1 {
            return Err(TmccError::InvalidBit { index, value: bit });
        }
        let feedback = ((remainder >> 81) & 1) as u8 ^ bit;
        remainder = (remainder << 1) & TMCC_GENERATOR_MASK;
        if feedback == 1 {
            remainder ^= TMCC_GENERATOR & TMCC_GENERATOR_MASK;
        }
    }
    Ok(remainder == 0)
}

/// Decode the 204 received TMCC carrier bits.
///
/// This performs differential decoding, frame-synchronisation word matching,
/// segment-type validation and the (184,102) parity check, then parses every
/// Table 3-22 field. It does **not** enforce one-segment-specific constraints —
/// use [`decode_tmcc_one_seg`] for that.
///
/// # Errors
///
/// [`TmccError::InvalidBit`], [`TmccError::SyncWord`],
/// [`TmccError::SegmentType`], [`TmccError::Parity`],
/// [`TmccError::Reserved`] or [`TmccError::InconsistentLayer`].
pub fn decode_tmcc(reference: u8, raw: &[u8; TMCC_TOTAL_BITS]) -> Result<TmccInfo, TmccError> {
    let bits = tmcc_differential_decode(reference, raw)?;

    let sync_word = bits_to_u16(&bits, TMCC_SYNC_OFFSET, 16);
    let sync = match sync_word {
        TMCC_SYNC_W0 => TmccSync::W0,
        TMCC_SYNC_W1 => TmccSync::W1,
        other => return Err(TmccError::SyncWord(other)),
    };

    let segment_type = match bits_to_u16(&bits, TMCC_SEGMENT_TYPE_OFFSET, 3) as u8 {
        0b111 => SegmentType::Differential,
        0b000 => SegmentType::Coherent,
        other => return Err(TmccError::SegmentType(other)),
    };

    let codeword = &bits[TMCC_INFO_OFFSET..TMCC_INFO_OFFSET + TMCC_CODEWORD_BITS];
    if !tmcc_parity_valid(codeword)? {
        return Err(TmccError::Parity);
    }

    let current = [
        parse_layer(&bits, 28)?,
        parse_layer(&bits, 41)?,
        parse_layer(&bits, 54)?,
    ];
    let next = [
        parse_layer(&bits, 68)?,
        parse_layer(&bits, 81)?,
        parse_layer(&bits, 94)?,
    ];

    Ok(TmccInfo {
        sync,
        segment_type,
        system: bits_to_u16(&bits, 20, 2) as u8,
        switching: bits_to_u16(&bits, 22, 4) as u8,
        emergency: bits[26] != 0,
        partial_reception: bits[27] != 0,
        current,
        next_partial_reception: bits[67] != 0,
        next,
        phase_shift_correction: bits_to_u16(&bits, 107, 3) as u8,
    })
}

/// Decode and validate a TMCC frame for one-segment reception of the centre
/// segment.
///
/// In addition to [`decode_tmcc`] this rejects:
///
/// * a non-television system identification (§3.15.6.1);
/// * an in-progress transmission-parameter switch (§3.15.6.2);
/// * a clear partial-reception flag, i.e. a service that is not using segment
///   No. 0 for partial reception (§3.15.6.4);
/// * an unused layer A, which must be assigned to segment No. 0 when partial
///   reception is set;
/// * an unsupported/incomplete layer-A configuration.
///
/// # Errors
///
/// Any [`TmccError`] from [`decode_tmcc`] plus [`TmccError::UnsupportedSystem`],
/// [`TmccError::ConfigSwitching`],
/// [`TmccError::PartialReceptionUnavailable`] or
/// [`TmccError::UnsupportedLayerA`].
pub fn decode_tmcc_one_seg(
    mode: Mode,
    reference: u8,
    raw: &[u8; TMCC_TOTAL_BITS],
) -> Result<TmccOneSeg, TmccError> {
    let info = decode_tmcc(reference, raw)?;
    if info.system != 0 {
        return Err(TmccError::UnsupportedSystem(info.system));
    }
    if info.switching != 0b1111 {
        return Err(TmccError::ConfigSwitching(info.switching));
    }
    if !info.partial_reception {
        return Err(TmccError::PartialReceptionUnavailable);
    }
    let layer_a = info.current_layer_a(mode)?;
    Ok(TmccOneSeg { info, layer_a })
}

/// Stateful checker of the alternating TMCC synchronising polarity.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TmccFrameSync {
    last: Option<TmccSync>,
    frames: u64,
}

impl TmccFrameSync {
    /// A new, unlocked detector.
    pub fn new() -> Self {
        Self::default()
    }

    /// Observe the synchronising word of one frame.
    ///
    /// The first observation locks the detector. Every later observation must
    /// carry the opposite polarity; a repeat is rejected and unlocks.
    ///
    /// # Errors
    ///
    /// [`TmccError::SyncNotAlternating`].
    pub fn observe(&mut self, sync: TmccSync) -> Result<(), TmccError> {
        if let Some(last) = self.last {
            if last == sync {
                self.last = None;
                self.frames = 0;
                return Err(TmccError::SyncNotAlternating);
            }
        }
        self.last = Some(sync);
        self.frames += 1;
        Ok(())
    }

    /// Whether a phase is currently locked.
    pub fn is_locked(&self) -> bool {
        self.last.is_some()
    }

    /// Number of consecutive alternating frames observed.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// The polarity expected on the next frame, if locked.
    pub fn next_expected(&self) -> Option<TmccSync> {
        self.last.map(TmccSync::invert)
    }
}

/// Detects a change of the locked transmission parameters after lock.
#[derive(Debug, Clone, Default)]
pub struct TmccConfigTracker {
    locked: Option<LayerConfig>,
}

impl TmccConfigTracker {
    /// A new tracker with no locked configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Forget the locked configuration.
    pub fn reset(&mut self) {
        self.locked = None;
    }

    /// The locked configuration, if any.
    pub fn locked(&self) -> Option<&LayerConfig> {
        self.locked.as_ref()
    }

    /// Observe the current layer-A configuration.
    ///
    /// The first call locks `configuration`. A later call whose configuration
    /// differs returns [`TmccError::ConfigChanged`] and unlocks.
    ///
    /// # Errors
    ///
    /// [`TmccError::ConfigChanged`].
    pub fn observe(&mut self, configuration: &LayerConfig) -> Result<(), TmccError> {
        match &self.locked {
            None => {
                self.locked = Some(*configuration);
                Ok(())
            }
            Some(previous) => {
                let same = discriminant(&previous.modulation)
                    == discriminant(&configuration.modulation)
                    && previous.rate == configuration.rate
                    && previous.interleave == configuration.interleave;
                if same {
                    Ok(())
                } else {
                    self.locked = None;
                    Err(TmccError::ConfigChanged)
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn set_bits(dst: &mut [u8], start: usize, len: usize, value: u16) {
        for i in 0..len {
            dst[start + i] = ((value >> (len - 1 - i)) & 1) as u8;
        }
    }

    // ---- carrier geometry ------------------------------------------------

    #[test]
    fn carrier_geometry_matches_arib_tables() {
        assert_eq!(carriers_per_segment(Mode::Mode1), 108);
        assert_eq!(carriers_per_segment(Mode::Mode2), 216);
        assert_eq!(carriers_per_segment(Mode::Mode3), 432);
        assert_eq!(data_carriers_per_segment(Mode::Mode1), 96);
        assert_eq!(data_carriers_per_segment(Mode::Mode2), 192);
        assert_eq!(data_carriers_per_segment(Mode::Mode3), 384);
        assert_eq!(total_carriers(Mode::Mode1), 1405);
        assert_eq!(total_carriers(Mode::Mode2), 2809);
        assert_eq!(total_carriers(Mode::Mode3), 5617);
        for mode in Mode::ALL {
            assert_eq!(
                total_carriers(mode),
                13 * carriers_per_segment(mode) + 1,
                "Table 3-2 total count"
            );
            assert_eq!(
                center_segment_first_abs_carrier(mode),
                6 * carriers_per_segment(mode)
            );
        }
        assert_eq!(center_segment_first_abs_carrier(Mode::Mode1), 648);
        assert_eq!(center_segment_first_abs_carrier(Mode::Mode2), 1296);
        assert_eq!(center_segment_first_abs_carrier(Mode::Mode3), 2592);

        // Segment 11 is the lowest frequency; segment 12 the highest.
        assert_eq!(segment_local_to_abs(Mode::Mode1, 11, 0).unwrap(), 0);
        assert_eq!(segment_local_to_abs(Mode::Mode1, 12, 107).unwrap(), 1403);
        assert_eq!(abs_to_segment_local(Mode::Mode1, 0), Some((11, 0)));
        assert_eq!(abs_to_segment_local(Mode::Mode1, 1403), Some((12, 107)));
        // The rightmost continuous carrier is not inside a segment.
        assert_eq!(abs_to_segment_local(Mode::Mode1, 1404), None);
        for mode in Mode::ALL {
            for segment in 0..13 {
                for local in [0, 1, 7, carriers_per_segment(mode) - 1] {
                    let abs = segment_local_to_abs(mode, segment, local).unwrap();
                    assert_eq!(abs_to_segment_local(mode, abs), Some((segment, local)));
                }
            }
            assert_eq!(
                center_local_to_abs(mode, 0).unwrap(),
                center_segment_first_abs_carrier(mode)
            );
        }
    }

    #[test]
    fn carrier_geometry_validates_bounds_without_panicking() {
        assert_eq!(
            segment_local_to_abs(Mode::Mode1, 13, 0),
            Err(ControlError::SegmentOutOfRange(13))
        );
        assert_eq!(
            segment_local_to_abs(Mode::Mode1, 0, 108),
            Err(ControlError::LocalCarrierOutOfRange {
                mode: Mode::Mode1,
                local: 108
            })
        );
        assert!(center_local_to_abs(Mode::Mode3, 432).is_err());
        assert_eq!(abs_to_segment_local(Mode::Mode2, 2809), None);
    }

    // ---- centre-segment positions ---------------------------------------

    #[test]
    fn centre_segment_control_carriers_match_tables_3_14_and_3_15() {
        // Table 3-15 (a)/(b)/(c), segment 0.
        assert_eq!(ac1_carriers(Mode::Mode1), &[35, 79]);
        assert_eq!(ac1_carriers(Mode::Mode2), &[98, 101, 118, 136]);
        assert_eq!(
            ac1_carriers(Mode::Mode3),
            &[7, 89, 206, 209, 226, 244, 377, 407]
        );
        assert_eq!(tmcc_carriers(Modulation::Qpsk, Mode::Mode1), &[49]);
        assert_eq!(tmcc_carriers(Modulation::Qam64, Mode::Mode2), &[23, 178]);
        assert_eq!(
            tmcc_carriers(Modulation::Qam16, Mode::Mode3),
            &[101, 131, 286, 349]
        );

        // Table 3-14 (a)/(b)/(c), segment 0.
        assert_eq!(
            ac2_carriers(Modulation::Dqpsk, Mode::Mode1),
            &[3, 72, 85, 89]
        );
        assert_eq!(
            ac2_carriers(Modulation::Dqpsk, Mode::Mode2),
            &[10, 30, 55, 81, 108, 111, 153, 167, 185]
        );
        assert_eq!(
            ac2_carriers(Modulation::Dqpsk, Mode::Mode3),
            &[
                25, 30, 42, 104, 108, 118, 138, 163, 189, 216, 219, 261, 275, 293, 324, 327, 339,
                364, 382
            ]
        );
        assert_eq!(
            tmcc_carriers(Modulation::Dqpsk, Mode::Mode1),
            &[49, 61, 96, 99, 104]
        );
        assert_eq!(
            tmcc_carriers(Modulation::Dqpsk, Mode::Mode2),
            &[23, 37, 51, 68, 105, 121, 158, 178, 191, 195]
        );
        assert_eq!(
            tmcc_carriers(Modulation::Dqpsk, Mode::Mode3),
            &[
                34, 48, 54, 70, 101, 131, 145, 159, 176, 213, 229, 266, 286, 299, 303, 349, 387,
                397, 404, 417
            ]
        );

        // Coherent segments have no AC2 and no in-segment CP (Table 3-1).
        assert!(ac2_carriers(Modulation::Qpsk, Mode::Mode1).is_empty());
        assert_eq!(cp_carrier(Modulation::Dqpsk), Some(0));
        assert_eq!(cp_carrier(Modulation::Qpsk), None);
    }

    #[test]
    fn scattered_pilots_follow_the_12_by_4_pattern() {
        assert_eq!(scattered_pilot_residue(0), 0);
        assert_eq!(scattered_pilot_residue(1), 3);
        assert_eq!(scattered_pilot_residue(2), 6);
        assert_eq!(scattered_pilot_residue(3), 9);
        assert_eq!(scattered_pilot_residue(4), 0);

        for (mode, expected) in [(Mode::Mode1, 9usize), (Mode::Mode2, 18), (Mode::Mode3, 36)] {
            for symbol in 0..8 {
                let pilots = scattered_pilot_carriers(mode, Modulation::Qpsk, symbol);
                assert_eq!(pilots.len(), expected, "Table 3-1 SP count");
                for &p in &pilots {
                    assert_eq!(p % 12, scattered_pilot_residue(symbol));
                    assert!(p < carriers_per_segment(mode));
                }
            }
            // Differential modulation has no scattered pilots.
            assert!(scattered_pilot_carriers(mode, Modulation::Dqpsk, 0).is_empty());
        }
    }

    #[test]
    fn data_carriers_are_the_exact_complement_of_the_pilots() {
        for mode in Mode::ALL {
            let total = carriers_per_segment(mode);
            let data_len = data_carriers_per_segment(mode);
            for modulation in [
                Modulation::Dqpsk,
                Modulation::Qpsk,
                Modulation::Qam16,
                Modulation::Qam64,
            ] {
                for symbol in 0..8 {
                    let data = data_carriers(mode, modulation, symbol);
                    assert_eq!(data.len(), data_len, "{mode:?} {modulation:?}");

                    let mut occupied = vec![false; total];
                    if let Some(cp) = cp_carrier(modulation) {
                        occupied[cp] = true;
                    }
                    for &c in ac1_carriers(mode) {
                        occupied[c] = true;
                    }
                    for &c in ac2_carriers(modulation, mode) {
                        occupied[c] = true;
                    }
                    for &c in tmcc_carriers(modulation, mode) {
                        occupied[c] = true;
                    }
                    for c in scattered_pilot_carriers(mode, modulation, symbol) {
                        occupied[c] = true;
                    }
                    for &c in &data {
                        assert!(!occupied[c], "data carrier overlaps a pilot at {c}");
                        occupied[c] = true;
                    }
                    assert!(
                        occupied.iter().all(|&used| used),
                        "union must cover the segment"
                    );
                }
            }
        }
    }

    // ---- pilot PRBS ------------------------------------------------------

    #[test]
    fn pilot_prbs_table_is_consistent_with_the_global_sequence_note() {
        // Table 3-16 note: starting from all ones and running continuously over
        // the whole band (segment 11 first, segment 12 last) reproduces every
        // segment's initial value.
        for (mode, per_segment) in [
            (Mode::Mode1, 108usize),
            (Mode::Mode2, 216),
            (Mode::Mode3, 432),
        ] {
            let mut generator = PilotPrbs::for_segment(mode, 11).unwrap();
            assert_eq!(generator.state(), 0x07FF);
            for (position, &segment) in SEGMENT_ORDER_LOW_TO_HIGH.iter().enumerate() {
                if position > 0 {
                    for _ in 0..per_segment {
                        generator.next_bit();
                    }
                }
                let table = PilotPrbs::for_segment(mode, segment).unwrap();
                assert_eq!(
                    generator.state(),
                    table.state(),
                    "{mode:?} segment {segment} initial value"
                );
            }
        }
    }

    #[test]
    fn pilot_prbs_is_maximal_length_and_matches_an_independent_register() {
        fn reference_step(register: &mut u16) -> u8 {
            let out = ((*register >> 10) & 1) as u8;
            let feedback = ((*register >> 10) ^ (*register >> 8)) & 1;
            *register = ((*register << 1) | feedback) & 0x07FF;
            out
        }

        for mode in Mode::ALL {
            let mut generator = PilotPrbs::center(mode);
            let mut reference = parse_prbs_init(pilot_prbs_init_table(mode)[0]);
            let mut seen = std::collections::HashSet::new();
            for i in 0..2047 {
                assert_eq!(
                    generator.next_bit(),
                    reference_step(&mut reference),
                    "bit {i}"
                );
                assert!(seen.insert(generator.state()), "state repeated before 2047");
            }
            // The polynomial is primitive: the register returns to its start
            // after 2^11 - 1 steps and never hits zero.
            assert_eq!(generator.state(), PilotPrbs::center(mode).state());
        }
    }

    #[test]
    fn pilot_amplitude_matches_table_3_17() {
        assert_eq!(pilot_modulating_signal(0).unwrap(), (4.0 / 3.0, 0.0));
        assert_eq!(pilot_modulating_signal(1).unwrap(), (-4.0 / 3.0, 0.0));
        assert_eq!(
            pilot_modulating_signal(2),
            Err(ControlError::InvalidPilotBit(2))
        );
    }

    // ---- frequency interleaving -----------------------------------------

    #[test]
    fn frequency_permutation_is_a_bijection_for_every_mode() {
        for mode in Mode::ALL {
            let table = frequency_randomize_permutation(mode);
            let n = data_carriers_per_segment(mode);
            assert_eq!(table.len(), n);
            let mut seen = vec![false; n];
            for &after in table {
                assert!(after < n && !seen[after], "{mode:?} not a permutation");
                seen[after] = true;
            }
            assert!(seen.iter().all(|&s| s));

            // Independently built inverse: inv[after] = before.
            let inverse = frequency_deinterleave_permutation(mode);
            for (before, &after) in table.iter().enumerate() {
                assert_eq!(inverse[after], before, "{mode:?}");
            }
        }
    }

    #[test]
    fn frequency_tables_match_normative_spot_values() {
        // Table 3-13 (a): before 0 -> 80, before 1 -> 93, before 95 -> 24.
        let m1 = frequency_randomize_permutation(Mode::Mode1);
        assert_eq!(m1[0], 80);
        assert_eq!(m1[1], 93);
        assert_eq!(m1[95], 24);
        // Table 3-13 (b).
        let m2 = frequency_randomize_permutation(Mode::Mode2);
        assert_eq!(m2[0], 98);
        assert_eq!(m2[191], 59);
        // Table 3-13 (c).
        let m3 = frequency_randomize_permutation(Mode::Mode3);
        assert_eq!(m3[0], 62);
        assert_eq!(m3[383], 51);
    }

    #[test]
    fn frequency_interleave_and_deinterleave_round_trip() {
        for mode in Mode::ALL {
            let n = data_carriers_per_segment(mode);
            let input: Vec<i32> = (0..n as i32).collect();
            let mut interleaved = vec![0i32; n];
            frequency_interleave(mode, &input, &mut interleaved).unwrap();

            let mut restored = vec![0i32; n];
            frequency_deinterleave(mode, &interleaved, &mut restored).unwrap();
            assert_eq!(restored, input, "{mode:?}");

            // The receiver mapping (carrier -> logical index), built
            // independently in `frequency_deinterleave_permutation`, must
            // reconstruct the logical data order.
            let permutation = frequency_deinterleave_permutation(mode);
            let mut via_mapping = vec![0i32; n];
            for (carrier, &logical) in permutation.iter().enumerate() {
                via_mapping[logical] = interleaved[carrier];
            }
            assert_eq!(via_mapping, input);
        }
    }

    #[test]
    fn frequency_functions_validate_buffers() {
        let short = vec![0u8; 95];
        let mut out = vec![0u8; 96];
        assert_eq!(
            frequency_interleave(Mode::Mode1, &short, &mut out),
            Err(ControlError::CarrierCount {
                expected: 96,
                got: 95
            })
        );
        let input = vec![0u8; 96];
        let mut small = vec![0u8; 95];
        assert_eq!(
            frequency_deinterleave(Mode::Mode1, &input, &mut small),
            Err(ControlError::OutputTooSmall {
                needed: 96,
                got: 95
            })
        );
    }

    // ---- time interleaving ----------------------------------------------

    #[test]
    fn time_interleave_delays_follow_figure_3_23() {
        for mode in [Mode::Mode1, Mode::Mode2, Mode::Mode3] {
            for carrier in 0..data_carriers_per_segment(mode) {
                assert_eq!(
                    time_interleave_delay(mode, 8, carrier),
                    8 * ((carrier * 5) % 96)
                );
                assert_eq!(
                    time_deinterleave_delay(mode, 8, carrier),
                    8 * 95 - 8 * ((carrier * 5) % 96)
                );
            }
        }
        assert_eq!(time_interleave_max_delay(8), 760);
        // Table 3-12 maximum delay values (I=4 mode 1 => 380, etc.).
        assert_eq!(time_interleave_max_delay(4), 380);
        assert_eq!(time_interleave_max_delay(2), 190);
        assert_eq!(time_interleave_max_delay(1), 95);
    }

    #[test]
    fn time_deinterleaver_inverts_the_transmitter_streaming() {
        for (mode, interleave) in [(Mode::Mode1, 4usize), (Mode::Mode2, 2), (Mode::Mode3, 1)] {
            let n = data_carriers_per_segment(mode);
            let max_delay = time_interleave_max_delay(interleave);
            let symbols = max_delay + 40;

            let mut transmitter = TimeInterleaver::<i64>::new(mode, interleave).unwrap();
            let mut receiver = TimeDeinterleaver::<i64>::new(mode, interleave).unwrap();
            assert_eq!(receiver.latency_symbols(), max_delay);

            let value = |symbol: usize, carrier: usize| (symbol * n + carrier) as i64 + 1;
            let mut aligned = Vec::new();
            for symbol in 0..symbols {
                let input: Vec<i64> = (0..n).map(|c| value(symbol, c)).collect();
                let transmitted = transmitter.process_symbol(&input).unwrap();
                let received = receiver.process_symbol(&transmitted).unwrap();
                aligned.push(received);
            }

            for (symbol, received) in aligned.iter().enumerate().skip(max_delay) {
                let expected: Vec<i64> = (0..n).map(|c| value(symbol - max_delay, c)).collect();
                assert_eq!(received, &expected, "{mode:?} symbol {symbol}");
            }
        }
    }

    #[test]
    fn time_interleavers_validate_lengths_and_interleave() {
        let mut interleaver = TimeInterleaver::<i32>::new(Mode::Mode1, 4).unwrap();
        assert_eq!(
            interleaver.process_symbol(&[0i32; 95]),
            Err(ControlError::CarrierCount {
                expected: 96,
                got: 95
            })
        );
        let mut deinterleaver = TimeDeinterleaver::<i32>::new(Mode::Mode1, 4).unwrap();
        assert!(deinterleaver.process_symbol(&[0i32; 96]).is_ok());
        assert_eq!(
            TimeInterleaver::<i32>::new(Mode::Mode1, MAX_TIME_INTERLEAVE + 1).unwrap_err(),
            ControlError::InvalidInterleave(MAX_TIME_INTERLEAVE + 1)
        );

        // Reset must restart the stream exactly.
        let mut first = TimeInterleaver::<i32>::new(Mode::Mode1, 4).unwrap();
        let symbol: Vec<i32> = (0..96).collect();
        let a = first.process_symbol(&symbol).unwrap();
        first.reset();
        let b = first.process_symbol(&symbol).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn time_interleave_latency_uses_guard_interval() {
        assert_eq!(
            symbol_duration_samples(Mode::Mode1, GuardInterval::OneQuarter),
            2560
        );
        assert_eq!(
            time_interleave_latency_samples(Mode::Mode1, GuardInterval::OneQuarter, 4),
            380 * 2560
        );
    }

    // ---- TMCC ------------------------------------------------------------

    /// Build a syntactically valid one-segment TMCC frame.
    #[allow(clippy::too_many_arguments)]
    fn build_raw_frame(
        modulation: Modulation,
        rate: PunctureRate,
        interleave_code: u8,
        segments: u8,
        partial: bool,
        switching: u8,
        sync: TmccSync,
    ) -> (u8, [u8; TMCC_TOTAL_BITS]) {
        let mut info = vec![1u8; TMCC_INFO_BITS];
        set_bits(&mut info, 0, 2, 0b00); // system: terrestrial television
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
        // Layers B/C, next information and reserved bits stay all ones.

        let codeword = tmcc_encode(&info).unwrap();
        let mut bits = [0u8; TMCC_TOTAL_BITS];
        set_bits(&mut bits, TMCC_SYNC_OFFSET, 16, sync.word());
        set_bits(&mut bits, TMCC_SEGMENT_TYPE_OFFSET, 3, 0b000);
        bits[TMCC_INFO_OFFSET..TMCC_INFO_OFFSET + TMCC_CODEWORD_BITS].copy_from_slice(&codeword);

        let reference = 1u8;
        let mut raw = [0u8; TMCC_TOTAL_BITS];
        raw[0] = reference;
        for k in 1..TMCC_TOTAL_BITS {
            raw[k] = raw[k - 1] ^ bits[k];
        }
        (reference, raw)
    }

    #[test]
    fn tmcc_parity_round_trip_and_corruption_rejection() {
        let info: Vec<u8> = (0..TMCC_INFO_BITS).map(|i| (i % 3 == 0) as u8).collect();
        let codeword = tmcc_encode(&info).unwrap();
        assert_eq!(codeword.len(), TMCC_CODEWORD_BITS);
        assert!(tmcc_parity_valid(&codeword).unwrap());

        for bit in [0usize, 1, 50, 101, 102, 120, 183] {
            let mut corrupt = codeword;
            corrupt[bit] ^= 1;
            assert!(
                !tmcc_parity_valid(&corrupt).unwrap(),
                "single-bit error at {bit} must be detected"
            );
        }
        assert_eq!(
            tmcc_encode(&[0u8; 101]),
            Err(TmccError::CodewordLength {
                expected: 102,
                got: 101
            })
        );
        assert_eq!(
            tmcc_parity_valid(&[0u8; 10]),
            Err(TmccError::CodewordLength {
                expected: 184,
                got: 10
            })
        );
        assert_eq!(
            tmcc_encode(&{
                let mut bad = vec![0u8; 102];
                bad[5] = 2;
                bad
            }),
            Err(TmccError::InvalidBit { index: 5, value: 2 })
        );
    }

    #[test]
    fn tmcc_diff_decode_and_one_seg_extraction() {
        let (reference, raw) = build_raw_frame(
            Modulation::Qam16,
            PunctureRate::Rate3_4,
            0b011,
            1,
            true,
            0b1111,
            TmccSync::W0,
        );
        let decoded = decode_tmcc_one_seg(Mode::Mode1, reference, &raw).unwrap();
        assert_eq!(decoded.info.sync, TmccSync::W0);
        assert_eq!(decoded.info.segment_type, SegmentType::Coherent);
        assert_eq!(decoded.info.system, 0);
        assert!(decoded.info.partial_reception);
        assert_eq!(decoded.info.switching, 0b1111);
        assert_eq!(decoded.info.current[0].modulation, Some(Modulation::Qam16));
        assert_eq!(decoded.info.current[0].rate, Some(PunctureRate::Rate3_4));
        assert_eq!(decoded.info.current[0].segments, Some(1));
        assert_eq!(
            decoded.info.current[0].interleave_value(Mode::Mode1),
            Some(16)
        );
        assert_eq!(decoded.layer_a.modulation, Modulation::Qam16);
        assert_eq!(decoded.layer_a.rate, PunctureRate::Rate3_4);
        assert_eq!(decoded.layer_a.interleave, 16);
    }

    #[test]
    fn tmcc_decode_rejects_bad_parity_and_sync() {
        let (reference, raw) = build_raw_frame(
            Modulation::Dqpsk,
            PunctureRate::Rate1_2,
            0b000,
            1,
            true,
            0b1111,
            TmccSync::W1,
        );
        // Corrupt a parity-region bit: B'150 affects B150/B151, both parity.
        let mut corrupt = raw;
        corrupt[150] ^= 1;
        assert_eq!(decode_tmcc(reference, &corrupt), Err(TmccError::Parity));

        // Destroy the synchronising word (B2/B3).
        let mut bad_sync = raw;
        bad_sync[2] ^= 1;
        assert!(matches!(
            decode_tmcc(reference, &bad_sync),
            Err(TmccError::SyncWord(_))
        ));

        // An unexpected bit value is rejected.
        let mut non_bit = raw;
        non_bit[10] = 7;
        assert_eq!(
            decode_tmcc(reference, &non_bit),
            Err(TmccError::InvalidBit {
                index: 10,
                value: 7
            })
        );
    }

    #[test]
    fn tmcc_one_seg_rejects_partial_reception_and_config_switching() {
        let (reference, raw) = build_raw_frame(
            Modulation::Qpsk,
            PunctureRate::Rate2_3,
            0b001,
            1,
            false,
            0b1111,
            TmccSync::W0,
        );
        assert_eq!(
            decode_tmcc_one_seg(Mode::Mode2, reference, &raw),
            Err(TmccError::PartialReceptionUnavailable)
        );

        let (reference, raw) = build_raw_frame(
            Modulation::Qpsk,
            PunctureRate::Rate2_3,
            0b001,
            1,
            true,
            0b1110,
            TmccSync::W0,
        );
        assert_eq!(
            decode_tmcc_one_seg(Mode::Mode2, reference, &raw),
            Err(TmccError::ConfigSwitching(0b1110))
        );

        // Sound-broadcasting system identification is not a TV service.
        let mut sound = vec![1u8; TMCC_INFO_BITS];
        set_bits(&mut sound, 0, 2, 0b01);
        set_bits(&mut sound, 2, 4, 0b1111);
        sound[7] = 1;
        set_bits(&mut sound, 8, 3, 0b001);
        set_bits(&mut sound, 11, 3, 0b001);
        set_bits(&mut sound, 14, 3, 0b001);
        set_bits(&mut sound, 17, 4, 1);
        let codeword = tmcc_encode(&sound).unwrap();
        let mut bits = [0u8; TMCC_TOTAL_BITS];
        set_bits(&mut bits, TMCC_SYNC_OFFSET, 16, TmccSync::W0.word());
        bits[TMCC_INFO_OFFSET..TMCC_INFO_OFFSET + TMCC_CODEWORD_BITS].copy_from_slice(&codeword);
        let reference = 1u8;
        let mut raw_sound = [0u8; TMCC_TOTAL_BITS];
        raw_sound[0] = reference;
        for k in 1..TMCC_TOTAL_BITS {
            raw_sound[k] = raw_sound[k - 1] ^ bits[k];
        }
        assert_eq!(
            decode_tmcc_one_seg(Mode::Mode2, reference, &raw_sound),
            Err(TmccError::UnsupportedSystem(0b01))
        );
    }

    #[test]
    fn tmcc_reserved_codes_are_rejected() {
        // Reserved carrier-modulation code 100.
        let mut info = vec![1u8; TMCC_INFO_BITS];
        set_bits(&mut info, 2, 4, 0b1111);
        info[7] = 1;
        set_bits(&mut info, 8, 3, 0b100);
        set_bits(&mut info, 11, 3, 0b000);
        set_bits(&mut info, 14, 3, 0b000);
        set_bits(&mut info, 17, 4, 1);
        let codeword = tmcc_encode(&info).unwrap();
        let mut bits = [0u8; TMCC_TOTAL_BITS];
        set_bits(&mut bits, TMCC_SYNC_OFFSET, 16, TmccSync::W0.word());
        bits[TMCC_INFO_OFFSET..TMCC_INFO_OFFSET + TMCC_CODEWORD_BITS].copy_from_slice(&codeword);
        let mut raw = [0u8; TMCC_TOTAL_BITS];
        raw[0] = 1;
        for k in 1..TMCC_TOTAL_BITS {
            raw[k] = raw[k - 1] ^ bits[k];
        }
        assert_eq!(
            decode_tmcc(1, &raw),
            Err(TmccError::Reserved("carrier modulation mapping scheme"))
        );
    }

    #[test]
    fn tmcc_frame_sync_requires_alternation() {
        let mut tracker = TmccFrameSync::new();
        assert!(!tracker.is_locked());
        tracker.observe(TmccSync::W0).unwrap();
        assert!(tracker.is_locked());
        assert_eq!(tracker.next_expected(), Some(TmccSync::W1));
        tracker.observe(TmccSync::W1).unwrap();
        assert_eq!(tracker.frames(), 2);
        assert_eq!(
            tracker.observe(TmccSync::W1),
            Err(TmccError::SyncNotAlternating)
        );
        assert!(!tracker.is_locked());
    }

    #[test]
    fn tmcc_config_tracker_detects_changes() {
        let mut tracker = TmccConfigTracker::new();
        let a = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate1_2,
            interleave: 4,
        };
        tracker.observe(&a).unwrap();
        assert_eq!(
            discriminant(&tracker.locked().unwrap().modulation),
            discriminant(&a.modulation)
        );
        tracker.observe(&a).unwrap();

        let changed = LayerConfig {
            modulation: Modulation::Qpsk,
            rate: PunctureRate::Rate2_3,
            interleave: 4,
        };
        assert_eq!(tracker.observe(&changed), Err(TmccError::ConfigChanged));
        assert!(tracker.locked().is_none());
    }

    #[test]
    fn tmcc_differential_decode_is_exact() {
        let mut raw = [0u8; TMCC_TOTAL_BITS];
        for (i, slot) in raw.iter_mut().enumerate() {
            *slot = (i % 2) as u8;
        }
        let bits = tmcc_differential_decode(1, &raw).unwrap();
        assert_eq!(bits[0], 1);
        for k in 1..TMCC_TOTAL_BITS {
            assert_eq!(bits[k], raw[k] ^ raw[k - 1]);
        }
        assert_eq!(
            tmcc_differential_decode(2, &raw),
            Err(TmccError::InvalidBit { index: 0, value: 2 })
        );
    }
}
