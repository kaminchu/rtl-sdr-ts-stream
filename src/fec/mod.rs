//! Forward-error-correction and transport-stream validation primitives.
//!
//! # Scope
//!
//! * [`crc32_mpeg2`] / [`Crc32Mpeg2`]: the MPEG-2 / PSI CRC-32 (non-reflected,
//!   polynomial `0x04C1_1DB7`, initial value `0xFFFF_FFFF`, no final XOR).
//! * [`parse_ts_header`], [`validate_ts_packet`], [`find_ts_sync_offset`],
//!   [`TsContinuityTracker`]: 188-byte TS packet parsing, validation and
//!   per-PID continuity checking.
//! * [`K7Encoder`], [`decode_k7_soft`]: the K=7 rate-1/2 convolutional code with
//!   generator polynomials `171`/`133` (octal), soft-decision Viterbi decoding.
//! * [`PuncturePattern`] / [`PunctureRate`]: generic puncturing plus the
//!   rate 1/2, 2/3, 3/4, 5/6 and 7/8 patterns commonly used by DVB-T / ISDB-T.
//! * [`rs204_188_encode`] / [`rs204_188_decode`]: a shortened
//!   Reed-Solomon (204, 188) code over `GF(2^8)` (primitive polynomial
//!   `x^8 + x^4 + x^3 + x^2 + 1`, generator roots `alpha^0 .. alpha^15`,
//!   `t = 8`).
//!
//! # Limitations
//!
//! * Energy dispersal (de-randomisation) is not implemented here; it is applied
//!   in [`crate::demod::coding`], outside this module.
//! * No byte / bit / frequency / time de-interleaving and no TMCC parsing; those
//!   live in [`crate::demod::coding`] / [`crate::demod::control`].
//! * No TS de-multiplexing, PID filtering or re-multiplexing. The TS helpers
//!   only parse, validate and continuity-check existing packets.
//! * No erasure decoding for RS and no soft-output (SOVA/BCJR) Viterbi.
//!
//! # Standards
//!
//! * The puncturing patterns follow the commonly published ETSI EN 300 744
//!   tables.
//! * The Reed-Solomon convention (`b = 0`) is stated explicitly because the
//!   first-consecutive-root convention can differ between systems.
//!
//! # Untrusted input
//!
//! Public entry points validate lengths, sync bytes and finiteness and never
//! panic on malformed slices.

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by the FEC / TS primitives.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FecError {
    /// A TS packet was not exactly 188 bytes.
    #[error("invalid TS packet length: expected {expected}, got {got}")]
    TsLength { expected: usize, got: usize },
    /// The first byte of a TS packet was not `0x47`.
    #[error("TS sync byte missing: got 0x{0:02X}")]
    TsSync(u8),
    /// `adaptation_field_control` was the reserved value `0b00`.
    #[error("reserved TS adaptation_field_control value 0")]
    TsAdaptationControl,
    /// The adaptation field declared more bytes than fit in the packet.
    #[error("TS adaptation field length {length} does not fit in the packet")]
    TsAdaptationLength { length: u8 },
    /// The adaptation field left no room for the payload the header promised.
    #[error("TS adaptation field length {length} leaves no room for the declared payload")]
    TsAdaptationPayloadOverlap { length: u8 },
    /// The transport error indicator was set.
    #[error("transport error indicator set on PID {pid}")]
    TsTransportError { pid: u16 },
    /// A soft-decision Viterbi input had an odd number of values.
    #[error("Viterbi input must contain an even number of soft values, got {0}")]
    ViterbiOddInput(usize),
    /// A soft value was NaN or infinite.
    #[error("non-finite soft value at index {index}")]
    NonFinite { index: usize },
    /// The Viterbi input exceeded the supported size bound.
    #[error("Viterbi input has {0} symbols, exceeding the supported maximum")]
    ViterbiTooLarge(usize),
    /// A puncture pattern was empty or kept no bits.
    #[error("puncture pattern is empty or keeps no bits")]
    InvalidPuncture,
    /// A puncturing / depuncturing input length was not a whole number of
    /// periods.
    #[error("input length {len} is not a multiple of the puncture period {period}")]
    PunctureLength { len: usize, period: usize },
    /// A Reed-Solomon block had the wrong length.
    #[error("Reed-Solomon block length {got} invalid; expected {expected}")]
    RsLength { expected: usize, got: usize },
    /// Reed-Solomon could not correct the block (too many errors).
    #[error("Reed-Solomon block is uncorrectable")]
    RsUncorrectable,
    /// A structurally invalid argument.
    #[error("invalid argument: {0}")]
    InvalidArgument(String),
}

// ---------------------------------------------------------------------------
// MPEG-2 CRC-32
// ---------------------------------------------------------------------------

/// MPEG-2 / PSI CRC-32 polynomial (`x^32 + x^26 + x^23 + x^22 + x^16 + x^12 +
/// x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1`).
pub const MPEG2_CRC32_POLY: u32 = 0x04C1_1DB7;

/// Initial register value used by the MPEG-2 CRC.
pub const MPEG2_CRC32_INIT: u32 = 0xFFFF_FFFF;

/// Compute the MPEG-2 CRC-32 of `bytes`.
///
/// Non-reflected, initial value [`MPEG2_CRC32_INIT`], no final XOR. The check
/// value for the ASCII string `"123456789"` is `0x0376E6E7`.
pub fn crc32_mpeg2(bytes: &[u8]) -> u32 {
    let mut state = MPEG2_CRC32_INIT;
    for &byte in bytes {
        state ^= (byte as u32) << 24;
        for _ in 0..8 {
            if state & 0x8000_0000 != 0 {
                state = (state << 1) ^ MPEG2_CRC32_POLY;
            } else {
                state <<= 1;
            }
        }
    }
    state
}

/// Streaming MPEG-2 CRC-32 calculator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Crc32Mpeg2 {
    state: u32,
}

impl Default for Crc32Mpeg2 {
    fn default() -> Self {
        Self::new()
    }
}

impl Crc32Mpeg2 {
    /// Start a new CRC computation.
    pub fn new() -> Self {
        Self {
            state: MPEG2_CRC32_INIT,
        }
    }

    /// Feed more bytes.
    pub fn update(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.state ^= (byte as u32) << 24;
            for _ in 0..8 {
                if self.state & 0x8000_0000 != 0 {
                    self.state = (self.state << 1) ^ MPEG2_CRC32_POLY;
                } else {
                    self.state <<= 1;
                }
            }
        }
    }

    /// Current CRC value. For a PSI section, a valid trailer makes this `0`.
    pub fn value(&self) -> u32 {
        self.state
    }

    /// Reset to [`MPEG2_CRC32_INIT`].
    pub fn reset(&mut self) {
        self.state = MPEG2_CRC32_INIT;
    }
}

// ---------------------------------------------------------------------------
// Transport stream packets
// ---------------------------------------------------------------------------

/// Size of a transport-stream packet, in bytes.
pub const TS_PACKET_SIZE: usize = 188;

/// Transport-stream sync byte.
pub const TS_SYNC_BYTE: u8 = 0x47;

/// Parsed 4-byte transport-stream header (plus adaptation-field extras).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TsHeader {
    /// Transport error indicator.
    pub transport_error_indicator: bool,
    /// Payload unit start indicator.
    pub payload_unit_start_indicator: bool,
    /// Transport priority.
    pub transport_priority: bool,
    /// Packet identifier.
    pub pid: u16,
    /// Transport scrambling control (2 bits).
    pub transport_scrambling_control: u8,
    /// Adaptation field control (2 bits, never `0b00`).
    pub adaptation_field_control: u8,
    /// Continuity counter (4 bits).
    pub continuity_counter: u8,
    /// Length of the adaptation field if present.
    pub adaptation_field_length: Option<u8>,
    /// Adaptation-field discontinuity indicator (only meaningful when the
    /// adaptation field is present and has at least one flags byte).
    pub discontinuity_indicator: bool,
}

impl TsHeader {
    /// Whether the packet carries a payload.
    pub fn has_payload(&self) -> bool {
        self.adaptation_field_control & 0b01 != 0
    }

    /// Whether the packet carries an adaptation field.
    pub fn has_adaptation_field(&self) -> bool {
        self.adaptation_field_control & 0b10 != 0
    }
}

/// Parse the header of a single 188-byte TS packet.
///
/// This validates structural fields (`sync`, `adaptation_field_control`,
/// adaptation field length) but **not** the transport error indicator; use
/// [`validate_ts_packet`] for that.
///
/// # Errors
///
/// * [`FecError::TsLength`] when `packet.len() != 188`.
/// * [`FecError::TsSync`] when the first byte is not `0x47`.
/// * [`FecError::TsAdaptationControl`] for the reserved value `0b00`.
/// * [`FecError::TsAdaptationLength`] when the adaptation field overruns.
pub fn parse_ts_header(packet: &[u8]) -> Result<TsHeader, FecError> {
    if packet.len() != TS_PACKET_SIZE {
        return Err(FecError::TsLength {
            expected: TS_PACKET_SIZE,
            got: packet.len(),
        });
    }
    if packet[0] != TS_SYNC_BYTE {
        return Err(FecError::TsSync(packet[0]));
    }

    let byte1 = packet[1];
    let byte3 = packet[3];
    let adaptation_field_control = (byte3 >> 4) & 0x03;
    if adaptation_field_control == 0 {
        return Err(FecError::TsAdaptationControl);
    }

    let mut adaptation_field_length = None;
    let mut discontinuity_indicator = false;
    if adaptation_field_control & 0b10 != 0 {
        let length = packet[4];
        // The length byte itself plus `length` bytes must fit in the packet
        // (4 header bytes + 1 length byte + `length` bytes <= 188).
        if 1 + length as usize > TS_PACKET_SIZE - 4 {
            return Err(FecError::TsAdaptationLength { length });
        }
        // When a payload is also signalled the adaptation field must leave at
        // least one byte for it.
        if adaptation_field_control & 0b01 != 0 && length as usize > TS_PACKET_SIZE - 6 {
            return Err(FecError::TsAdaptationPayloadOverlap { length });
        }
        adaptation_field_length = Some(length);
        if length >= 1 {
            discontinuity_indicator = packet[5] & 0x80 != 0;
        }
    }

    Ok(TsHeader {
        transport_error_indicator: byte1 & 0x80 != 0,
        payload_unit_start_indicator: byte1 & 0x40 != 0,
        transport_priority: byte1 & 0x20 != 0,
        pid: (((byte1 as u16) & 0x1F) << 8) | packet[2] as u16,
        transport_scrambling_control: (byte3 >> 6) & 0x03,
        adaptation_field_control,
        continuity_counter: byte3 & 0x0F,
        adaptation_field_length,
        discontinuity_indicator,
    })
}

/// Parse and validate a TS packet, rejecting transport-error packets.
///
/// # Errors
///
/// Any [`parse_ts_header`] error, plus [`FecError::TsTransportError`] when the
/// transport error indicator is set.
pub fn validate_ts_packet(packet: &[u8]) -> Result<TsHeader, FecError> {
    let header = parse_ts_header(packet)?;
    if header.transport_error_indicator {
        return Err(FecError::TsTransportError { pid: header.pid });
    }
    Ok(header)
}

/// Find the first plausible TS sync offset in `buf`.
///
/// An offset is plausible when `buf[offset] == 0x47` and, wherever the buffer
/// is long enough, the same byte repeats at `+188` (and `+376`). Returns the
/// offset only if at least one full 188-byte packet fits.
pub fn find_ts_sync_offset(buf: &[u8]) -> Option<usize> {
    if buf.len() < TS_PACKET_SIZE {
        return None;
    }
    for offset in 0..=buf.len() - TS_PACKET_SIZE {
        if buf[offset] != TS_SYNC_BYTE {
            continue;
        }
        if offset + TS_PACKET_SIZE < buf.len() && buf[offset + TS_PACKET_SIZE] != TS_SYNC_BYTE {
            continue;
        }
        if offset + 2 * TS_PACKET_SIZE < buf.len()
            && buf[offset + 2 * TS_PACKET_SIZE] != TS_SYNC_BYTE
        {
            continue;
        }
        return Some(offset);
    }
    None
}

/// Outcome of feeding one packet to a [`TsContinuityTracker`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContinuityEvent {
    /// First packet observed for this PID.
    First,
    /// Continuity counter advanced by exactly one.
    Incremented,
    /// The packet has no payload, so its counter is not meaningful.
    NoPayload,
    /// Same counter as the previous payload packet (a duplicate).
    Duplicate,
    /// An adaptation field signalled an intentional discontinuity.
    Reset,
    /// The counter jumped (packets may have been lost).
    UnexpectedGap { expected: u8, got: u8 },
}

/// Tracks TS continuity counters per PID across packets.
#[derive(Debug, Default, Clone)]
pub struct TsContinuityTracker {
    last: HashMap<u16, u8>,
}

impl TsContinuityTracker {
    /// Create an empty tracker.
    pub fn new() -> Self {
        Self::default()
    }

    /// Forget all PIDs.
    pub fn clear(&mut self) {
        self.last.clear();
    }

    /// Observe one parsed header and update state.
    pub fn observe(&mut self, header: &TsHeader) -> ContinuityEvent {
        if !header.has_payload() {
            return ContinuityEvent::NoPayload;
        }

        let counter = header.continuity_counter;
        if header.discontinuity_indicator {
            self.last.insert(header.pid, counter);
            return ContinuityEvent::Reset;
        }

        match self.last.get(&header.pid).copied() {
            None => {
                self.last.insert(header.pid, counter);
                ContinuityEvent::First
            }
            Some(previous) => {
                let expected = (previous + 1) & 0x0F;
                if counter == expected {
                    self.last.insert(header.pid, counter);
                    ContinuityEvent::Incremented
                } else if counter == previous {
                    // Duplicate packet: keep the expected counter unchanged.
                    ContinuityEvent::Duplicate
                } else {
                    self.last.insert(header.pid, counter);
                    ContinuityEvent::UnexpectedGap {
                        expected,
                        got: counter,
                    }
                }
            }
        }
    }

    /// Parse and observe one packet.
    ///
    /// # Errors
    ///
    /// Returns any [`parse_ts_header`] error.
    pub fn observe_packet(
        &mut self,
        packet: &[u8],
    ) -> Result<(TsHeader, ContinuityEvent), FecError> {
        let header = parse_ts_header(packet)?;
        let event = self.observe(&header);
        Ok((header, event))
    }
}

// ---------------------------------------------------------------------------
// K=7 rate-1/2 convolutional code (171 / 133 octal) and soft Viterbi
// ---------------------------------------------------------------------------

/// Constraint length of the convolutional code.
pub const K7_CONSTRAINT_LENGTH: usize = 7;
/// Number of encoder states (`2^(K-1)`).
pub const K7_STATES: usize = 64;
/// First generator polynomial, octal 171.
pub const K7_POLY_G1: u8 = 0o171;
/// Second generator polynomial, octal 133.
pub const K7_POLY_G2: u8 = 0o133;
/// Maximum number of symbols accepted by one Viterbi call (memory bound: the
/// traceback stores one byte per state per symbol).
pub const MAX_VITERBI_SYMBOLS: usize = 1 << 18;

/// Compute the two generator output bits for a 7-bit encoder register.
fn k7_outputs(register: u8) -> (u8, u8) {
    let g1 = (register & K7_POLY_G1).count_ones() & 1;
    let g2 = (register & K7_POLY_G2).count_ones() & 1;
    (g1 as u8, g2 as u8)
}

/// Compute `(next_state, out1, out2)` for a transition.
///
/// The 7-bit register is `[current, u(n-1), ..., u(n-6)]` from bit 6 down to
/// bit 0, i.e. the current input bit is the most-significant tap. This matches
/// the usual polynomial orientation in which the octal generator
/// `171 = 1111001` has its D^0 (current-bit) coefficient as the MSB.
fn k7_transition(state: u8, input: u8) -> (u8, u8, u8) {
    let register = (state & 0x3F) | ((input & 1) << 6);
    let next_state = (register >> 1) & 0x3F;
    let (out1, out2) = k7_outputs(register);
    (next_state, out1, out2)
}

/// Renormalise cumulative path metrics so the best finite value is exactly
/// `0.0`.
///
/// Path metrics are cumulative sums of soft branch values, so they grow without
/// bound over a long block (or, in `crate::demod::coding`, a long stream). Once
/// the magnitude approaches the precision limit, adding a soft value to two
/// nearly-equal survivors rounds both to the same value and the decoder can no
/// longer separate them. Subtracting the maximum finite metric every ACS step
/// keeps the finite window bounded independently of the number of steps while
/// preserving the relative ordering, so the same survivor is selected.
/// `NEG_INFINITY` marks an unreachable state and is left untouched.
///
/// Metrics are held as `f64` even though soft values are `f32`. The largest
/// finite soft value is `f32::MAX`, and the spread of one ACS step's candidates
/// is bounded by a small multiple of it; a worst-case K=7 merge-window spread
/// is around `1e41`, far inside the `f64` range but already overflowing `f32`.
/// Casting to `f64` before the branch sum therefore keeps every finite `f32`
/// input well-defined with no arithmetic clamp on the public API.
fn renormalise_metrics(metrics: &mut [f64; K7_STATES]) {
    let mut maximum = f64::NEG_INFINITY;
    for &value in metrics.iter() {
        if value > maximum {
            maximum = value;
        }
    }
    if !maximum.is_finite() {
        return;
    }
    for value in metrics.iter_mut() {
        if value.is_finite() {
            *value -= maximum;
        }
    }
    debug_assert!(
        metrics.iter().all(|&value| value <= 0.0),
        "renormalised metrics must be non-positive"
    );
    debug_assert!(
        metrics.contains(&0.0),
        "the best reachable state must renormalise to zero"
    );
}

/// K=7 rate-1/2 convolutional encoder.
///
/// The 6-bit state holds the last six input bits, with the most recent bit in
/// the most-significant position. A new input bit is inserted as the MSB of the
/// 7-bit register and the six memory bits shift down; the two generator outputs
/// are read from that register. This is the convention used by the matching
/// [`decode_k7_soft`] trellis.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct K7Encoder {
    state: u8,
}

impl K7Encoder {
    /// Create an encoder in state 0.
    pub fn new() -> Self {
        Self { state: 0 }
    }

    /// Current 6-bit state.
    pub fn state(&self) -> u8 {
        self.state
    }

    /// Reset to state 0.
    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Encode one input bit (only bit 0 is used), returning `(out1, out2)`.
    pub fn push_bit(&mut self, bit: u8) -> (u8, u8) {
        let (next_state, out1, out2) = k7_transition(self.state, bit);
        self.state = next_state;
        (out1, out2)
    }

    /// Encode a slice of bits, returning the interleaved hard output bits
    /// (`out1_0, out2_0, out1_1, out2_1, ...`).
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(bits.len() * 2);
        for &bit in bits {
            let (a, b) = self.push_bit(bit);
            out.push(a);
            out.push(b);
        }
        out
    }
}

/// Core soft Viterbi decoder.
///
/// `final_state` is the state the traceback starts from. `None` selects the
/// best-metric final state (unterminated decoding).
fn k7_viterbi(soft: &[f32], final_state: Option<u8>) -> Result<Vec<u8>, FecError> {
    if soft.len() % 2 != 0 {
        return Err(FecError::ViterbiOddInput(soft.len()));
    }
    for (index, &value) in soft.iter().enumerate() {
        if !value.is_finite() {
            return Err(FecError::NonFinite { index });
        }
    }

    let steps = soft.len() / 2;
    if steps == 0 {
        return Ok(Vec::new());
    }
    if steps > MAX_VITERBI_SYMBOLS {
        return Err(FecError::ViterbiTooLarge(steps));
    }

    let mut metrics = [f64::NEG_INFINITY; K7_STATES];
    metrics[0] = 0.0;
    let mut trace: Vec<[u8; K7_STATES]> = vec![[0u8; K7_STATES]; steps];

    for step in 0..steps {
        // Widen before the branch sum so even `f32::MAX` inputs cannot overflow
        // the accumulator; see `renormalise_metrics`.
        let soft0 = f64::from(soft[2 * step]);
        let soft1 = f64::from(soft[2 * step + 1]);
        let mut next = [f64::NEG_INFINITY; K7_STATES];
        let mut predecessor = [0u8; K7_STATES];

        for state in 0..K7_STATES as u8 {
            let base = metrics[state as usize];
            if base == f64::NEG_INFINITY {
                continue;
            }
            for input in 0..2u8 {
                let (next_state, out1, out2) = k7_transition(state, input);
                let bit0 = if out1 == 0 { soft0 } else { -soft0 };
                let bit1 = if out2 == 0 { soft1 } else { -soft1 };
                let candidate = base + bit0 + bit1;
                let slot = next_state as usize;
                if candidate > next[slot] {
                    next[slot] = candidate;
                    predecessor[slot] = state;
                }
            }
        }

        renormalise_metrics(&mut next);
        metrics = next;
        trace[step] = predecessor;
    }

    let mut state = match final_state {
        Some(state) => state,
        None => {
            let mut best_state = 0u8;
            let mut best_metric = f64::NEG_INFINITY;
            for (index, &metric) in metrics.iter().enumerate() {
                if metric > best_metric {
                    best_metric = metric;
                    best_state = index as u8;
                }
            }
            best_state
        }
    };

    let mut bits = vec![0u8; steps];
    for step in (0..steps).rev() {
        // The input bit is stored as the MSB of the 6-bit next state.
        bits[step] = (state >> 5) & 1;
        state = trace[step][state as usize];
    }
    Ok(bits)
}

/// Soft-decision Viterbi decoding terminated at state 0.
///
/// Each pair of soft values corresponds to the two generator outputs for one
/// input bit. A soft value `> 0` favours bit 0 and `< 0` favours bit 1 (so a
/// hard bit 0 is ideally `+1` and a hard bit 1 is `-1`); `0.0` is an erasure.
///
/// The traceback starts from state 0, so the encoder is expected to have been
/// flushed with `K - 1 = 6` zero tail bits. See [`decode_k7_soft_best_end`] for
/// unterminated use.
///
/// # Errors
///
/// * [`FecError::ViterbiOddInput`] when `soft.len()` is odd.
/// * [`FecError::NonFinite`] when any value is NaN/infinite.
/// * [`FecError::ViterbiTooLarge`] beyond [`MAX_VITERBI_SYMBOLS`] symbols.
pub fn decode_k7_soft(soft: &[f32]) -> Result<Vec<u8>, FecError> {
    k7_viterbi(soft, Some(0))
}

/// Soft-decision Viterbi decoding without an assumed terminating state.
///
/// The traceback starts from the final state with the best accumulated metric.
pub fn decode_k7_soft_best_end(soft: &[f32]) -> Result<Vec<u8>, FecError> {
    k7_viterbi(soft, None)
}

// ---------------------------------------------------------------------------
// Puncturing
// ---------------------------------------------------------------------------

/// Standard rate-1/2 mother-code puncturing rates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PunctureRate {
    /// 1/2 (no puncturing).
    Rate1_2,
    /// 2/3.
    Rate2_3,
    /// 3/4.
    Rate3_4,
    /// 5/6.
    Rate5_6,
    /// 7/8.
    Rate7_8,
}

impl PunctureRate {
    /// All standard rates.
    pub const ALL: [PunctureRate; 5] = [
        PunctureRate::Rate1_2,
        PunctureRate::Rate2_3,
        PunctureRate::Rate3_4,
        PunctureRate::Rate5_6,
        PunctureRate::Rate7_8,
    ];

    /// `(period, kept bits)` over one pattern period. The number of mother-code
    /// bits consumed per period is `2 * period`.
    pub fn ratio(self) -> (usize, usize) {
        match self {
            PunctureRate::Rate1_2 => (1, 2),
            PunctureRate::Rate2_3 => (2, 3),
            PunctureRate::Rate3_4 => (3, 4),
            PunctureRate::Rate5_6 => (5, 6),
            PunctureRate::Rate7_8 => (7, 8),
        }
    }

    /// Build the corresponding pattern.
    pub fn pattern(self) -> PuncturePattern {
        match self {
            PunctureRate::Rate1_2 => {
                PuncturePattern::new(vec![true], vec![true]).expect("1/2 pattern is valid")
            }
            PunctureRate::Rate2_3 => PuncturePattern::new(vec![true, false], vec![true, true])
                .expect("2/3 pattern is valid"),
            PunctureRate::Rate3_4 => {
                PuncturePattern::new(vec![true, false, true], vec![true, true, false])
                    .expect("3/4 pattern is valid")
            }
            PunctureRate::Rate5_6 => PuncturePattern::new(
                vec![true, false, true, false, true],
                vec![true, true, false, true, false],
            )
            .expect("5/6 pattern is valid"),
            PunctureRate::Rate7_8 => PuncturePattern::new(
                vec![true, false, false, false, true, false, true],
                vec![true, true, true, true, false, true, false],
            )
            .expect("7/8 pattern is valid"),
        }
    }
}

/// Periodic puncturing pattern for the rate-1/2 mother code.
///
/// The encoder emits, for each symbol time `t`, `(X_t, Y_t)`. A pattern holds
/// one keep/drop flag per symbol time for each generator; the punctured stream
/// is read in time order, appending `X_t` then `Y_t` when kept.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PuncturePattern {
    keep_x: Vec<bool>,
    keep_y: Vec<bool>,
}

impl PuncturePattern {
    /// Build a pattern from per-symbol-time keep flags.
    ///
    /// # Errors
    ///
    /// Returns [`FecError::InvalidPuncture`] when the two vectors are empty,
    /// have different lengths, or keep no bits at all.
    pub fn new(keep_x: Vec<bool>, keep_y: Vec<bool>) -> Result<Self, FecError> {
        if keep_x.is_empty() || keep_x.len() != keep_y.len() {
            return Err(FecError::InvalidPuncture);
        }
        if !keep_x.iter().chain(keep_y.iter()).any(|&keep| keep) {
            return Err(FecError::InvalidPuncture);
        }
        Ok(Self { keep_x, keep_y })
    }

    /// Number of symbol times per period.
    pub fn period(&self) -> usize {
        self.keep_x.len()
    }

    /// Number of bits kept per period.
    pub fn kept_per_period(&self) -> usize {
        self.keep_x.iter().filter(|&&k| k).count() + self.keep_y.iter().filter(|&&k| k).count()
    }

    /// Number of mother-code bits consumed per period (always `2 * period`).
    pub fn consumed_per_period(&self) -> usize {
        2 * self.period()
    }

    /// Puncture a hard rate-1/2 encoded bit stream.
    ///
    /// # Errors
    ///
    /// [`FecError::PunctureLength`] when `input.len()` is not a multiple of
    /// `2 * period`.
    pub fn puncture(&self, input: &[u8]) -> Result<Vec<u8>, FecError> {
        let period_bits = self.consumed_per_period();
        if input.len() % period_bits != 0 {
            return Err(FecError::PunctureLength {
                len: input.len(),
                period: period_bits,
            });
        }
        let mut out = Vec::with_capacity(input.len() / period_bits * self.kept_per_period());
        for block in input.chunks_exact(period_bits) {
            for t in 0..self.period() {
                if self.keep_x[t] {
                    out.push(block[2 * t]);
                }
                if self.keep_y[t] {
                    out.push(block[2 * t + 1]);
                }
            }
        }
        Ok(out)
    }

    /// Inverse of [`puncture`](Self::puncture) for soft values.
    ///
    /// Punctured positions are filled with `0.0` (erasure). The reconstructed
    /// length is `periods * 2 * period`.
    ///
    /// # Errors
    ///
    /// [`FecError::PunctureLength`] when `punctured.len()` is not a multiple of
    /// [`kept_per_period`](Self::kept_per_period).
    pub fn depuncture(&self, punctured: &[f32]) -> Result<Vec<f32>, FecError> {
        let kept = self.kept_per_period();
        if punctured.len() % kept != 0 {
            return Err(FecError::PunctureLength {
                len: punctured.len(),
                period: kept,
            });
        }
        let periods = punctured.len() / kept;
        let consumed = self.consumed_per_period();
        let total = periods
            .checked_mul(consumed)
            .ok_or_else(|| FecError::InvalidArgument("depuncture output length overflow".into()))?;
        let mut out = vec![0.0f32; total];
        let mut cursor = 0usize;
        for p in 0..periods {
            let base = p.checked_mul(consumed).ok_or_else(|| {
                FecError::InvalidArgument("depuncture output offset overflow".into())
            })?;
            for t in 0..self.period() {
                if self.keep_x[t] {
                    out[base + 2 * t] = punctured[cursor];
                    cursor += 1;
                }
                if self.keep_y[t] {
                    out[base + 2 * t + 1] = punctured[cursor];
                    cursor += 1;
                }
            }
        }
        Ok(out)
    }
}

// ---------------------------------------------------------------------------
// Reed-Solomon (204, 188) over GF(2^8)
// ---------------------------------------------------------------------------

/// Reed-Solomon primitive polynomial `x^8 + x^4 + x^3 + x^2 + 1`.
pub const RS_PRIMITIVE_POLY: u16 = 0x11D;
/// Number of parity bytes in RS(204, 188).
pub const RS204_188_PARITY: usize = 16;
/// Number of data bytes in RS(204, 188).
pub const RS204_188_DATA: usize = 188;
/// Codeword length of RS(204, 188).
pub const RS204_188_TOTAL: usize = RS204_188_DATA + RS204_188_PARITY;

/// Build the antilog table `exp[i] = alpha^i`.
///
/// `alpha = 2` is a primitive element of `GF(2^8)`, so `exp[0..255]` enumerates
/// every nonzero element exactly once and the sequence then repeats with period
/// 255. The table is doubled to `512` entries so two logarithms can be summed
/// and used directly as an index without a conditional reduction.
const fn build_gf_exp() -> [u8; 512] {
    let mut exp = [0u8; 512];
    let mut value = 1u8;
    let mut i = 0;
    while i < 255 {
        exp[i] = value;
        let high = value & 0x80;
        value <<= 1;
        if high != 0 {
            value ^= (RS_PRIMITIVE_POLY & 0xFF) as u8;
        }
        i += 1;
    }
    // `alpha^255 == alpha^0 == 1`; repeat the period to fill the doubling.
    let mut i = 255;
    while i < 512 {
        exp[i] = exp[i - 255];
        i += 1;
    }
    exp
}

/// Antilog table `alpha^i` for `i = 0..=511` (see [`build_gf_exp`]).
const GF_EXP: [u8; 512] = build_gf_exp();

/// Build the logarithm table inverse to [`GF_EXP`]: `log[v]` is the exponent
/// with `alpha^log[v] == v` for every nonzero `v`; `log[0]` is unused and left 0.
const fn build_gf_log() -> [u8; 256] {
    let mut log = [0u8; 256];
    let mut i = 0;
    while i < 255 {
        log[GF_EXP[i] as usize] = i as u8;
        i += 1;
    }
    log
}

/// Logarithm table inverse to [`GF_EXP`].
const GF_LOG: [u8; 256] = build_gf_log();

/// Multiply two field elements modulo [`RS_PRIMITIVE_POLY`].
///
/// Uses the compile-time log/antilog tables: `a * b = alpha^(log a + log b)`.
/// The hot Reed-Solomon syndrome loop performs one table lookup per byte, which
/// dominates the acquisition and decode cost on real captures.
pub fn gf_mul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        0
    } else {
        GF_EXP[GF_LOG[a as usize] as usize + GF_LOG[b as usize] as usize]
    }
}

/// Raise `a` to the power `exponent` in `GF(2^8)`.
pub fn gf_pow(mut a: u8, mut exponent: u32) -> u8 {
    let mut result = 1u8;
    while exponent > 0 {
        if exponent & 1 != 0 {
            result = gf_mul(result, a);
        }
        a = gf_mul(a, a);
        exponent >>= 1;
    }
    result
}

/// Multiplicative inverse in `GF(2^8)` (returns `0` for input `0`).
pub fn gf_inv(a: u8) -> u8 {
    if a == 0 {
        0
    } else {
        // The multiplicative group has order 255: `a^-1 = alpha^(255 - log a)`.
        GF_EXP[255 - GF_LOG[a as usize] as usize]
    }
}

/// Build the RS generator polynomial `prod_{i=0}^{15}(x + alpha^i)`.
///
/// Coefficients are natural order (highest degree first), monic, length 17.
fn rs_generator() -> [u8; RS204_188_PARITY + 1] {
    let mut generator = [0u8; RS204_188_PARITY + 1];
    generator[0] = 1;

    for i in 0..RS204_188_PARITY {
        let root = gf_pow(2, i as u32);
        // Before iteration `i` the polynomial has degree `i`; multiply it by
        // (x + root), which raises the degree to `i + 1`.
        let mut next = [0u8; RS204_188_PARITY + 1];
        for j in 0..=i {
            next[j] ^= generator[j]; // x * generator
            next[j + 1] ^= gf_mul(root, generator[j]); // root * generator
        }
        generator = next;
    }
    generator
}

/// Compute the parity bytes of a 188-byte RS data block.
fn rs_parity(data: &[u8]) -> [u8; RS204_188_PARITY] {
    let generator = rs_generator();
    let mut remainder = [0u8; RS204_188_PARITY];

    for &byte in data {
        let feedback = byte ^ remainder[0];
        remainder.copy_within(1..RS204_188_PARITY, 0);
        remainder[RS204_188_PARITY - 1] = 0;
        if feedback != 0 {
            for (slot, &coefficient) in remainder.iter_mut().zip(generator[1..].iter()) {
                *slot ^= gf_mul(feedback, coefficient);
            }
        }
    }
    remainder
}

/// Compute the 16 RS syndromes `S_i = C(alpha^i)` for `i = 0..15`.
fn rs_syndromes(block: &[u8]) -> [u8; RS204_188_PARITY] {
    let mut syndromes = [0u8; RS204_188_PARITY];
    for (i, slot) in syndromes.iter_mut().enumerate() {
        // `alpha = 2` and `i < 16`, so the root is a direct antilog lookup.
        let x = GF_EXP[i];
        let mut acc = 0u8;
        for &coefficient in block {
            acc = gf_mul(acc, x) ^ coefficient;
        }
        *slot = acc;
    }
    syndromes
}

/// Evaluate an ascending-order polynomial at `x`.
fn poly_eval(poly: &[u8], x: u8) -> u8 {
    let mut acc = 0u8;
    for &coefficient in poly.iter().rev() {
        acc = gf_mul(acc, x) ^ coefficient;
    }
    acc
}

/// Multiply two ascending-order polynomials.
fn poly_mul(a: &[u8], b: &[u8]) -> Vec<u8> {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let mut out = vec![0u8; a.len() + b.len() - 1];
    for (i, &ai) in a.iter().enumerate() {
        if ai == 0 {
            continue;
        }
        for (j, &bj) in b.iter().enumerate() {
            out[i + j] ^= gf_mul(ai, bj);
        }
    }
    out
}

/// Berlekamp-Massey error-locator polynomial (ascending order).
fn berlekamp_massey(syndromes: &[u8]) -> Vec<u8> {
    let n = syndromes.len();
    let mut current = vec![0u8; n + 1];
    current[0] = 1;
    let mut previous = vec![0u8; n + 1];
    previous[0] = 1;

    let mut shift = 1usize;
    let mut degree = 0usize;
    let mut previous_discrepancy = 1u8;

    for step in 0..n {
        let mut discrepancy = syndromes[step];
        for i in 1..=degree {
            discrepancy ^= gf_mul(current[i], syndromes[step - i]);
        }

        if discrepancy == 0 {
            shift += 1;
            continue;
        }

        let scale = gf_mul(discrepancy, gf_inv(previous_discrepancy));
        if 2 * degree <= step {
            let saved = current.clone();
            for (i, &coefficient) in previous.iter().enumerate() {
                let index = i + shift;
                if index < current.len() && coefficient != 0 {
                    current[index] ^= gf_mul(scale, coefficient);
                }
            }
            degree = step + 1 - degree;
            previous = saved;
            previous_discrepancy = discrepancy;
            shift = 1;
        } else {
            for (i, &coefficient) in previous.iter().enumerate() {
                let index = i + shift;
                if index < current.len() && coefficient != 0 {
                    current[index] ^= gf_mul(scale, coefficient);
                }
            }
            shift += 1;
        }
    }

    current.truncate(degree + 1);
    current
}

/// Encode 188 data bytes into a 204-byte RS(204, 188) codeword.
///
/// # Errors
///
/// [`FecError::RsLength`] when `data.len() != 188`.
pub fn rs204_188_encode(data: &[u8]) -> Result<Vec<u8>, FecError> {
    if data.len() != RS204_188_DATA {
        return Err(FecError::RsLength {
            expected: RS204_188_DATA,
            got: data.len(),
        });
    }
    let parity = rs_parity(data);
    let mut codeword = Vec::with_capacity(RS204_188_TOTAL);
    codeword.extend_from_slice(data);
    codeword.extend_from_slice(&parity);
    Ok(codeword)
}

/// Correct up to `t = 8` byte errors in a 204-byte RS(204, 188) block.
///
/// Returns the 188 data bytes. Uncorrectable blocks (which include blocks with
/// more than 8 errors) return [`FecError::RsUncorrectable`]; a returned block
/// has been verified to satisfy all 16 syndromes.
///
/// # Errors
///
/// * [`FecError::RsLength`] when `block.len() != 204`.
/// * [`FecError::RsUncorrectable`] when correction fails verification.
pub fn rs204_188_decode(block: &[u8]) -> Result<Vec<u8>, FecError> {
    if block.len() != RS204_188_TOTAL {
        return Err(FecError::RsLength {
            expected: RS204_188_TOTAL,
            got: block.len(),
        });
    }

    let mut codeword = block.to_vec();
    let syndromes = rs_syndromes(&codeword);
    if syndromes.iter().all(|&s| s == 0) {
        codeword.truncate(RS204_188_DATA);
        return Ok(codeword);
    }

    let locator = berlekamp_massey(&syndromes);
    let error_count = locator.len().saturating_sub(1);
    if error_count == 0 || error_count > RS204_188_PARITY / 2 {
        return Err(FecError::RsUncorrectable);
    }

    // Chien search over the shortened codeword positions. `alpha_inverse^l`
    // and `alpha^l` are advanced by one multiplication per step instead of a
    // fresh exponentiation, which removes ~3 000 field multiplications from
    // every attempted correction.
    let alpha_inverse = gf_inv(2);
    let mut positions = Vec::with_capacity(error_count);
    let mut locators = Vec::with_capacity(error_count);
    let mut x_inverses = Vec::with_capacity(error_count);
    let mut x_inverse = 1u8;
    let mut x = 1u8;
    for l in 0..RS204_188_TOTAL {
        if poly_eval(&locator, x_inverse) == 0 {
            positions.push(RS204_188_TOTAL - 1 - l);
            locators.push(x);
            x_inverses.push(x_inverse);
        }
        x_inverse = gf_mul(x_inverse, alpha_inverse);
        x = gf_mul(x, 2);
    }
    if positions.len() != error_count {
        return Err(FecError::RsUncorrectable);
    }

    // Error evaluator Omega(x) = S(x) * Lambda(x) mod x^16.
    let mut evaluator = poly_mul(&syndromes, &locator);
    evaluator.truncate(RS204_188_PARITY);

    // Formal derivative of Lambda (only odd powers survive in GF(2^m)).
    let mut derivative = vec![0u8; locator.len().saturating_sub(1)];
    for (j, &coefficient) in locator.iter().enumerate().skip(1) {
        if j % 2 == 1 {
            derivative[j - 1] = coefficient;
        }
    }

    for ((&position, &locator_value), &x_inverse) in
        positions.iter().zip(locators.iter()).zip(x_inverses.iter())
    {
        let numerator = poly_eval(&evaluator, x_inverse);
        let denominator = poly_eval(&derivative, x_inverse);
        if denominator == 0 {
            return Err(FecError::RsUncorrectable);
        }
        // Forney with b = 0: e = X * Omega(X^-1) / Lambda'(X^-1).
        let magnitude = gf_mul(locator_value, gf_mul(numerator, gf_inv(denominator)));
        codeword[position] ^= magnitude;
    }

    if !rs_syndromes(&codeword).iter().all(|&s| s == 0) {
        return Err(FecError::RsUncorrectable);
    }
    codeword.truncate(RS204_188_DATA);
    Ok(codeword)
}

/// Return whether a 204-byte block satisfies all RS(204, 188) syndromes.
///
/// # Errors
///
/// [`FecError::RsLength`] when `block.len() != 204`.
pub fn rs204_188_is_valid(block: &[u8]) -> Result<bool, FecError> {
    if block.len() != RS204_188_TOTAL {
        return Err(FecError::RsLength {
            expected: RS204_188_TOTAL,
            got: block.len(),
        });
    }
    Ok(rs_syndromes(block).iter().all(|&s| s == 0))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn xorshift(state: &mut u32) -> u32 {
        let mut x = *state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *state = x;
        x
    }

    // ---- CRC -------------------------------------------------------------

    #[test]
    fn crc32_mpeg2_matches_known_check_value() {
        assert_eq!(crc32_mpeg2(b"123456789"), 0x0376_E6E7);
        assert_eq!(crc32_mpeg2(b""), 0xFFFF_FFFF);
    }

    #[test]
    fn crc32_mpeg2_streaming_matches_one_shot() {
        let data: Vec<u8> = (0..=255u8).collect();
        let mut crc = Crc32Mpeg2::new();
        crc.update(&data[..100]);
        crc.update(&data[100..]);
        assert_eq!(crc.value(), crc32_mpeg2(&data));
        crc.reset();
        assert_eq!(crc.value(), 0xFFFF_FFFF);
    }

    // ---- TS --------------------------------------------------------------

    fn make_ts_packet(pid: u16, cc: u8, afc: u8, payload: &[u8]) -> [u8; 188] {
        let mut packet = [0u8; 188];
        packet[0] = TS_SYNC_BYTE;
        packet[1] = ((pid >> 8) as u8) & 0x1F;
        packet[2] = (pid & 0xFF) as u8;
        packet[3] = (afc << 4) | (cc & 0x0F);
        let start = 4;
        for (i, byte) in payload.iter().enumerate() {
            if start + i < 188 {
                packet[start + i] = *byte;
            }
        }
        packet
    }

    #[test]
    fn ts_header_parsing_and_validation() {
        let mut packet = make_ts_packet(0x1234, 7, 0b01, &[0xAA, 0xBB]);
        let header = parse_ts_header(&packet).unwrap();
        assert_eq!(header.pid, 0x1234);
        assert_eq!(header.continuity_counter, 7);
        assert_eq!(header.adaptation_field_control, 0b01);
        assert!(header.has_payload());
        assert!(!header.has_adaptation_field());
        assert!(!header.transport_error_indicator);
        validate_ts_packet(&packet).unwrap();

        // Transport error indicator is rejected by validate but parsed.
        packet[1] |= 0x80;
        assert!(parse_ts_header(&packet).unwrap().transport_error_indicator);
        assert_eq!(
            validate_ts_packet(&packet).unwrap_err(),
            FecError::TsTransportError { pid: 0x1234 }
        );
    }

    #[test]
    fn ts_header_rejects_malformed_packets() {
        assert_eq!(
            parse_ts_header(&[0u8; 10]).unwrap_err(),
            FecError::TsLength {
                expected: 188,
                got: 10
            }
        );
        let mut packet = make_ts_packet(1, 0, 0b01, &[]);
        packet[0] = 0x00;
        assert_eq!(parse_ts_header(&packet).unwrap_err(), FecError::TsSync(0));

        let packet = make_ts_packet(1, 0, 0b00, &[]);
        assert_eq!(
            parse_ts_header(&packet).unwrap_err(),
            FecError::TsAdaptationControl
        );

        // Adaptation field claiming more than fits.
        let mut packet = make_ts_packet(1, 0, 0b10, &[]);
        packet[4] = 200;
        assert_eq!(
            parse_ts_header(&packet).unwrap_err(),
            FecError::TsAdaptationLength { length: 200 }
        );

        // Adaptation field that leaves no room for the signalled payload.
        let mut packet = make_ts_packet(1, 0, 0b11, &[]);
        packet[4] = 183;
        assert_eq!(
            parse_ts_header(&packet).unwrap_err(),
            FecError::TsAdaptationPayloadOverlap { length: 183 }
        );
    }

    #[test]
    fn ts_adaptation_discontinuity_is_parsed() {
        let mut packet = make_ts_packet(9, 3, 0b10, &[]);
        packet[4] = 183; // adaptation-only packet fills the remainder
        packet[5] = 0x80; // discontinuity indicator
        let header = parse_ts_header(&packet).unwrap();
        assert!(header.has_adaptation_field());
        assert!(!header.has_payload());
        assert!(header.discontinuity_indicator);
        assert_eq!(header.adaptation_field_length, Some(183));
    }

    #[test]
    fn ts_continuity_tracker_detects_gaps_and_duplicates() {
        let mut tracker = TsContinuityTracker::new();
        let h0 = parse_ts_header(&make_ts_packet(100, 0, 0b01, &[])).unwrap();
        assert_eq!(tracker.observe(&h0), ContinuityEvent::First);

        let h1 = parse_ts_header(&make_ts_packet(100, 1, 0b01, &[])).unwrap();
        assert_eq!(tracker.observe(&h1), ContinuityEvent::Incremented);

        let duplicate = parse_ts_header(&make_ts_packet(100, 1, 0b01, &[])).unwrap();
        assert_eq!(tracker.observe(&duplicate), ContinuityEvent::Duplicate);

        let gap = parse_ts_header(&make_ts_packet(100, 5, 0b01, &[])).unwrap();
        assert_eq!(
            tracker.observe(&gap),
            ContinuityEvent::UnexpectedGap {
                expected: 2,
                got: 5
            }
        );

        // Independent PID starts fresh.
        let other = parse_ts_header(&make_ts_packet(200, 9, 0b01, &[])).unwrap();
        assert_eq!(tracker.observe(&other), ContinuityEvent::First);

        // Adaptation-only packets are ignored.
        let no_payload = parse_ts_header(&make_ts_packet(100, 9, 0b10, &[])).unwrap();
        assert_eq!(tracker.observe(&no_payload), ContinuityEvent::NoPayload);

        // Discontinuity indicator resets without a gap report. The packet
        // carries both an adaptation field and a payload, so the adaptation
        // length must leave room for the signalled payload.
        let mut packet = make_ts_packet(100, 12, 0b11, &[]);
        packet[4] = 10;
        packet[5] = 0x80; // discontinuity indicator
        let reset = parse_ts_header(&packet).unwrap();
        assert_eq!(reset.adaptation_field_length, Some(10));
        assert!(reset.has_payload());
        assert!(reset.discontinuity_indicator);
        assert_eq!(tracker.observe(&reset), ContinuityEvent::Reset);
    }

    #[test]
    fn ts_sync_offset_search() {
        assert_eq!(find_ts_sync_offset(&[0u8; 10]), None);
        let mut buf = vec![0x00u8; 5];
        buf.extend_from_slice(&make_ts_packet(1, 0, 0b01, &[]));
        buf.extend_from_slice(&make_ts_packet(1, 1, 0b01, &[]));
        assert_eq!(find_ts_sync_offset(&buf), Some(5));
    }

    // ---- convolutional encoding / Viterbi --------------------------------

    /// Independent structural encoder: keeps a 7-bit register with the current
    /// bit at bit 6 and shifts right, reading the polished taps.
    fn reference_encode(bits: &[u8]) -> Vec<u8> {
        let mut register = 0u8;
        let mut out = Vec::with_capacity(bits.len() * 2);
        for &bit in bits {
            register = (register >> 1) | ((bit & 1) << 6);
            out.push((register & K7_POLY_G1).count_ones() as u8 & 1);
            out.push((register & K7_POLY_G2).count_ones() as u8 & 1);
        }
        out
    }

    #[test]
    fn k7_encoder_matches_hand_calculated_vector() {
        // Input bits followed by the 6 zero tail bits that return the encoder
        // to state 0. The expected output was computed by hand from the
        // 7-bit register rule reg = (u << 6) | previous_state,
        // out_i = parity(reg & poly_i), next_state = reg >> 1.
        let input = [1u8, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let expected = [
            1u8, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0,
        ];
        let mut encoder = K7Encoder::new();
        assert_eq!(encoder.encode(&input), expected);
        assert_eq!(encoder.state(), 0);
    }

    #[test]
    fn k7_encoder_matches_independent_shift_register_reference() {
        let mut state = 0x00C0_FFEEu32;
        let bits: Vec<u8> = (0..200).map(|_| (xorshift(&mut state) & 1) as u8).collect();
        let mut encoder = K7Encoder::new();
        assert_eq!(encoder.encode(&bits), reference_encode(&bits));
    }

    #[test]
    fn k7_viterbi_decodes_noiseless_and_noisy_vectors() {
        let mut state = 0x1234_5678u32;
        let mut data = Vec::new();
        for _ in 0..80 {
            data.push((xorshift(&mut state) & 1) as u8);
        }
        let mut framed = data.clone();
        framed.extend_from_slice(&[0u8; 6]); // tail

        let mut encoder = K7Encoder::new();
        let encoded = encoder.encode(&framed);

        let to_soft = |bits: &[u8]| -> Vec<f32> {
            bits.iter()
                .map(|&b| if b == 0 { 1.0 } else { -1.0 })
                .collect()
        };

        let soft = to_soft(&encoded);
        assert_eq!(decode_k7_soft(&soft).unwrap(), framed);

        // Flip a handful of hard decisions; the soft decoder should still win.
        let mut noisy = soft.clone();
        noisy[3] = -noisy[3];
        noisy[40] = -noisy[40];
        noisy[119] = -noisy[119];
        assert_eq!(decode_k7_soft(&noisy).unwrap(), framed);

        // Erasures decode too.
        let mut erased = soft.clone();
        erased[10] = 0.0;
        erased[11] = 0.0;
        assert_eq!(decode_k7_soft(&erased).unwrap(), framed);
    }

    #[test]
    fn k7_viterbi_validates_input() {
        assert_eq!(
            decode_k7_soft(&[1.0, 2.0, 3.0]).unwrap_err(),
            FecError::ViterbiOddInput(3)
        );
        assert_eq!(
            decode_k7_soft(&[1.0, f32::NAN]).unwrap_err(),
            FecError::NonFinite { index: 1 }
        );
        assert!(decode_k7_soft(&[]).unwrap().is_empty());
    }

    #[test]
    fn k7_unterminated_decode_finds_best_end_state() {
        let mut encoder = K7Encoder::new();
        let data = [1u8, 0, 1, 1, 0, 1, 0, 1, 1];
        let encoded = encoder.encode(&data);
        let soft: Vec<f32> = encoded
            .iter()
            .map(|&b| if b == 0 { 1.0 } else { -1.0 })
            .collect();
        assert_eq!(decode_k7_soft_best_end(&soft).unwrap(), data);
    }

    /// `f32::MAX` is the largest value the block API accepts. An ACS sum of two
    /// such values overflows `f32` to `+inf`; the `f64` accumulator must keep
    /// the decode well-defined and still recover the encoder input exactly,
    /// because sign (not magnitude) carries the decision.
    #[test]
    fn k7_viterbi_decodes_max_finite_magnitude() {
        let mut state = 0x7A31_0C9Du32;
        let bits: Vec<u8> = (0..120).map(|_| (xorshift(&mut state) & 1) as u8).collect();
        let mut framed = bits.clone();
        framed.extend_from_slice(&[0u8; 6]); // tail to state 0

        let mut encoder = K7Encoder::new();
        let encoded = encoder.encode(&framed);
        let soft: Vec<f32> = encoded
            .iter()
            .map(|&bit| if bit == 0 { f32::MAX } else { -f32::MAX })
            .collect();

        assert_eq!(decode_k7_soft(&soft).unwrap(), framed);
    }

    /// The renormalisation helper is `f64` and keeps the "unreachable" marker
    /// semantics: finite metrics are shifted so the best is exactly `0.0`, while
    /// `NEG_INFINITY` states are preserved untouched.
    #[test]
    fn renormalise_metrics_zeroes_best_and_preserves_unreachable() {
        let mut metrics = [f64::NEG_INFINITY; K7_STATES];
        metrics[0] = 5.0;
        metrics[1] = -3.0;
        metrics[2] = 5.0;
        renormalise_metrics(&mut metrics);
        assert_eq!(metrics[0], 0.0);
        assert_eq!(metrics[2], 0.0);
        assert_eq!(metrics[1], -8.0);
        for &metric in metrics.iter().skip(3) {
            assert_eq!(metric, f64::NEG_INFINITY);
        }
    }

    // ---- puncturing ------------------------------------------------------

    #[test]
    fn puncture_patterns_have_expected_rate() {
        for rate in PunctureRate::ALL {
            let pattern = rate.pattern();
            let (input, kept) = rate.ratio();
            assert_eq!(pattern.period(), input);
            assert_eq!(pattern.kept_per_period(), kept);
        }
    }

    #[test]
    fn puncture_hand_calculated_examples() {
        let r23 = PunctureRate::Rate2_3.pattern();
        assert_eq!(
            r23.puncture(&[1, 2, 3, 4]).unwrap(),
            vec![1, 2, 4],
            "2/3 keeps X0, Y0, Y1"
        );

        let r34 = PunctureRate::Rate3_4.pattern();
        assert_eq!(
            r34.puncture(&[1, 2, 3, 4, 5, 6]).unwrap(),
            vec![1, 2, 4, 5],
            "3/4 keeps X0, Y0, Y1, X2"
        );

        let r78 = PunctureRate::Rate7_8.pattern();
        assert_eq!(
            r78.puncture(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])
                .unwrap(),
            vec![1, 2, 4, 6, 8, 9, 12, 13],
            "7/8 keeps X0, Y0, Y1, Y2, Y3, X4, Y5, X6"
        );
    }

    #[test]
    fn depuncture_restores_erasure_positions() {
        let pattern = PunctureRate::Rate3_4.pattern();
        let punctured = pattern.puncture(&[1, 2, 3, 4, 5, 6]).unwrap();
        assert_eq!(punctured, vec![1, 2, 4, 5]);
        let soft: Vec<f32> = punctured.iter().map(|&b| b as f32).collect();
        assert_eq!(
            pattern.depuncture(&soft).unwrap(),
            vec![1.0, 2.0, 0.0, 4.0, 5.0, 0.0]
        );
    }

    #[test]
    fn puncture_rejects_bad_lengths() {
        let pattern = PunctureRate::Rate3_4.pattern();
        assert_eq!(
            pattern.puncture(&[1, 2, 3]).unwrap_err(),
            FecError::PunctureLength { len: 3, period: 6 }
        );
        assert_eq!(
            pattern.depuncture(&[1.0, 2.0, 3.0]).unwrap_err(),
            FecError::PunctureLength { len: 3, period: 4 }
        );
        assert!(PuncturePattern::new(vec![], vec![]).is_err());
        assert!(PuncturePattern::new(vec![false], vec![false]).is_err());
        assert!(PuncturePattern::new(vec![true], vec![true, false]).is_err());
    }

    #[test]
    fn punctured_chain_round_trips_through_viterbi() {
        // 210 framed bits is divisible by every standard pattern period.
        let framed_len = 210usize;
        let mut state = 0xDEAD_BEEFu32;
        let mut framed = Vec::new();
        for _ in 0..framed_len {
            framed.push((xorshift(&mut state) & 1) as u8);
        }
        // Force the tail to return to state 0.
        for bit in framed.iter_mut().rev().take(6) {
            *bit = 0;
        }

        let mut encoder = K7Encoder::new();
        let encoded = encoder.encode(&framed);
        assert_eq!(encoded.len(), framed_len * 2);

        for rate in PunctureRate::ALL {
            let pattern = rate.pattern();
            let punctured = pattern.puncture(&encoded).unwrap();
            let soft: Vec<f32> = punctured
                .iter()
                .map(|&b| if b == 0 { 1.0 } else { -1.0 })
                .collect();
            let restored = pattern.depuncture(&soft).unwrap();
            assert_eq!(restored.len(), encoded.len());
            assert_eq!(
                decode_k7_soft(&restored).unwrap(),
                framed,
                "rate {:?} failed",
                rate
            );
        }
    }

    // ---- Reed-Solomon ----------------------------------------------------

    /// The log/antilog tables must be bit-for-bit identical to the original
    /// carry-less multiplier for every one of the 65 536 operand pairs. This
    /// pins the optimisation to the definition instead of trusting the tables.
    #[test]
    fn gf_mul_table_matches_carryless_reference() {
        fn reference(mut a: u8, mut b: u8) -> u8 {
            let mut product = 0u8;
            for _ in 0..8 {
                if b & 1 != 0 {
                    product ^= a;
                }
                let high = a & 0x80;
                a <<= 1;
                if high != 0 {
                    a ^= (RS_PRIMITIVE_POLY & 0xFF) as u8;
                }
                b >>= 1;
            }
            product
        }
        for a in 0..=255u8 {
            for b in 0..=255u8 {
                assert_eq!(gf_mul(a, b), reference(a, b), "gf_mul({a}, {b})");
            }
        }
    }

    #[test]
    fn gf256_basic_identities() {
        assert_eq!(gf_mul(1, 42), 42);
        assert_eq!(gf_mul(0, 42), 0);
        for a in 1u8..=255 {
            assert_eq!(gf_mul(a, gf_inv(a)), 1, "inverse of {a}");
            assert_eq!(gf_inv(gf_mul(a, 7)), gf_mul(gf_inv(a), gf_inv(7)));
        }
        assert_eq!(gf_pow(2, 0), 1);
        assert_eq!(gf_pow(2, 8), 0x1D); // x^8 = x^4 + x^3 + x^2 + 1
    }

    #[test]
    fn rs_encoder_produces_zero_syndromes() {
        let data: Vec<u8> = (0..188u8).collect();
        let codeword = rs204_188_encode(&data).unwrap();
        assert_eq!(codeword.len(), 204);
        assert!(rs204_188_is_valid(&codeword).unwrap());
        assert_eq!(&codeword[..188], &data[..]);
    }

    #[test]
    fn rs_encoder_zero_data_has_zero_parity() {
        let data = [0u8; 188];
        let codeword = rs204_188_encode(&data).unwrap();
        assert!(codeword.iter().all(|&b| b == 0));
        assert_eq!(rs204_188_decode(&codeword).unwrap(), data.to_vec());
    }

    #[test]
    fn rs_decoder_corrects_up_to_eight_errors() {
        let mut state = 0x0BAD_F00Du32;
        for _ in 0..32 {
            let mut data = vec![0u8; 188];
            for byte in data.iter_mut() {
                *byte = (xorshift(&mut state) & 0xFF) as u8;
            }
            let mut codeword = rs204_188_encode(&data).unwrap();

            // Inject `errors` distinct byte errors at pseudo-random positions.
            let errors = ((xorshift(&mut state) % 8) + 1) as usize;
            let mut hit = [false; 204];
            let mut placed = 0;
            while placed < errors {
                let position = (xorshift(&mut state) as usize) % 204;
                if hit[position] {
                    continue;
                }
                hit[position] = true;
                let magnitude = (xorshift(&mut state) & 0xFF) as u8 | 1;
                codeword[position] ^= magnitude;
                placed += 1;
            }

            let corrected = rs204_188_decode(&codeword).unwrap();
            assert_eq!(corrected, data, "failed with {errors} errors");
        }
    }

    #[test]
    fn rs_decoder_rejects_a_deterministic_nine_error_block() {
        // Nine byte errors exceed t = 8 and this specific pattern is reported
        // uncorrectable. We deliberately do not assert that *every* nine-error
        // pattern is detected: RS(204, 188) has minimum distance 17, so a
        // 9-error received word can fall within distance 8 of a different valid
        // codeword and be silently miscorrected.
        let data: Vec<u8> = (0..RS204_188_DATA as u8).collect();
        let mut codeword = rs204_188_encode(&data).unwrap();
        for byte in codeword.iter_mut().take(9) {
            *byte ^= 0x5A;
        }
        assert_eq!(
            rs204_188_decode(&codeword).unwrap_err(),
            FecError::RsUncorrectable
        );
    }

    #[test]
    fn rs_decoder_returns_data_when_clean() {
        let data: Vec<u8> = (0..188).map(|i| (i * 7 + 3) as u8).collect();
        let codeword = rs204_188_encode(&data).unwrap();
        assert_eq!(rs204_188_decode(&codeword).unwrap(), data);
    }

    #[test]
    fn rs_lengths_are_validated() {
        assert!(matches!(
            rs204_188_encode(&[0u8; 10]),
            Err(FecError::RsLength {
                expected: 188,
                got: 10
            })
        ));
        assert!(matches!(
            rs204_188_decode(&[0u8; 100]),
            Err(FecError::RsLength {
                expected: 204,
                got: 100
            })
        ));
        assert!(matches!(
            rs204_188_is_valid(&[0u8; 3]),
            Err(FecError::RsLength { .. })
        ));
    }
}
