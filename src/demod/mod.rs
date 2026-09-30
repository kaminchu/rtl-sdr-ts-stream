//! ISDB-T demodulator/decoder building blocks.
//!
//! This module owns the from-scratch ISDB-T receiver primitives:
//!
//! * [`coding`] — channel coding (bit de-interleaving, depuncturing, Viterbi,
//!   byte de-interleaving, energy-dispersal removal, RS(204,188) decoding).
//! * [`control`] — OFDM control plane (carrier layout, pilot PRBS, frequency
//!   and time interleaving, TMCC decoding).
//! * [`ofdm`] — centre-segment OFDM symbol extraction (guard removal, CFO
//!   de-rotation, unitary forward DFT, carrier demapping).
//! * [`sync`] — streaming Mode / guard / timing / CFO synchronisation on top
//!   of [`ofdm`].
//! * [`frame`] — TMCC frame acquisition: differential TMCC demodulation and
//!   parity/synchronisation validation of frame-aligned carrier windows.
//! * [`coarse`] — high-confidence integer-subcarrier CFO estimation for a
//!   coherent segment from its scattered pilots.

pub mod carriers;
pub mod coarse;
pub mod coding;
pub mod control;
pub mod frame;
pub mod ofdm;
pub mod receiver;
pub mod sync;

#[cfg(test)]
mod replay_tests;
