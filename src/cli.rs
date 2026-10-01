//! Command-line surface.
//!
//! Confirmed defaults (see `plans/実装計画.md` §2.2): host `127.0.0.1`,
//! port `3000`, device `0`, gain `auto`, ppm `0`.

use std::fmt;
use std::str::FromStr;

use clap::{Args, Parser, Subcommand};

/// Default IQ sample rate. One-seg occupies a single segment (~430 kHz), so a
/// 1.2 MHz complex sample rate leaves margin for the central segment while
/// staying well inside librtlsdr's supported range.
pub const DEFAULT_SAMPLE_RATE: u32 = 1_200_000;

/// The `ts-stream` command line.
#[derive(Debug, Parser)]
#[command(
    name = "ts-stream",
    version = crate::VERSION,
    about = "Receive an ISDB-T one-seg channel from an RTL-SDR and stream its TS over HTTP"
)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Debug, Subcommand)]
pub enum Command {
    /// Enumerate connected RTL-SDR devices.
    ListDevices,
    /// Run the single-client HTTP TS server.
    Server(ServerArgs),
}

/// Arguments for `ts-stream server`.
#[derive(Debug, Clone, Args)]
pub struct ServerArgs {
    /// Address to bind. Defaults to loopback; there is no authentication.
    #[arg(long, default_value = "127.0.0.1")]
    pub host: String,

    /// TCP port to listen on.
    #[arg(long, default_value_t = 3000)]
    pub port: u16,

    /// RTL-SDR device index.
    #[arg(long, default_value_t = 0)]
    pub device: u32,

    /// Tuner gain: `auto` or a manual gain in dB.
    #[arg(long, default_value = "auto")]
    pub gain: Gain,

    /// Frequency correction in parts per million.
    #[arg(long, default_value_t = 0)]
    pub ppm: i32,

    /// IQ sample rate in Hz.
    #[arg(long, default_value_t = DEFAULT_SAMPLE_RATE)]
    pub sample_rate: u32,
}

/// Tuner gain setting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gain {
    /// Automatic gain control.
    Auto,
    /// Manual gain in dB.
    Manual(f64),
}

impl Gain {
    /// `None` requests AGC; `Some(db)` requests a manual gain.
    pub fn as_option(self) -> Option<f64> {
        match self {
            Gain::Auto => None,
            Gain::Manual(db) => Some(db),
        }
    }
}

impl FromStr for Gain {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        if value.eq_ignore_ascii_case("auto") {
            return Ok(Gain::Auto);
        }
        let db = value
            .parse::<f64>()
            .map_err(|_| format!("invalid gain `{value}`: expected `auto` or a number of dB"))?;
        if !db.is_finite() {
            return Err(format!("invalid gain `{value}`: must be finite"));
        }
        Ok(Gain::Manual(db))
    }
}

impl fmt::Display for Gain {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Gain::Auto => f.write_str("auto"),
            Gain::Manual(db) => write!(f, "{db}"),
        }
    }
}
