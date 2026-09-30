//! `ts-stream` entry point.

use std::sync::Arc;

use anyhow::{Context, Result};
use clap::Parser;
use ts_stream::cli::{Cli, Command, ServerArgs};
use ts_stream::http;
use ts_stream::session::{
    rtl_backend::RtlBackend, Backend, SessionConfig, SessionManager, UnavailableBackendWithReason,
};

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::ListDevices => list_devices(),
        Command::Server(args) => {
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .context("failed to build the Tokio runtime")?;
            runtime.block_on(serve(args))
        }
    }
}

fn list_devices() -> Result<()> {
    let backend = RtlBackend::load().context("failed to initialise the RTL-SDR backend")?;
    match backend.list_devices() {
        Ok(devices) => {
            if devices.is_empty() {
                eprintln!("ts-stream: no RTL-SDR devices found");
                return Ok(());
            }
            for device in devices {
                match device.serial {
                    Some(serial) => {
                        println!(
                            "index={} name={} serial={}",
                            device.index, device.name, serial
                        )
                    }
                    None => println!(
                        "index={} name={} serial=<unavailable>",
                        device.index, device.name
                    ),
                }
            }
            Ok(())
        }
        Err(error) => {
            eprintln!("ts-stream: list-devices failed: {error}");
            std::process::exit(1);
        }
    }
}

async fn serve(args: ServerArgs) -> Result<()> {
    // Loading the backend must not prevent the HTTP server from starting:
    // `/version` has to answer even when no device/library is available. A
    // failure here downgrades `/stream` to an honest `503`.
    match RtlBackend::load() {
        Ok(backend) => {
            eprintln!("ts-stream: librtlsdr loaded");
            serve_with_backend(Arc::new(backend), args).await
        }
        Err(error) => {
            eprintln!(
                "ts-stream: RTL-SDR backend unavailable ({error}); \
                 serving /version and returning 503 for /stream"
            );
            let reason = format!("RTL-SDR backend unavailable: {error}");
            serve_with_backend(Arc::new(UnavailableBackendWithReason::new(reason)), args).await
        }
    }
}

async fn serve_with_backend<B: Backend>(backend: Arc<B>, args: ServerArgs) -> Result<()> {
    let config = SessionConfig {
        device_index: args.device,
        gain: args.gain.as_option(),
        ppm: args.ppm,
        sample_rate: args.sample_rate,
        ..SessionConfig::default()
    };
    let manager = SessionManager::new(backend, config);
    let app = http::build_router(manager.clone());

    let listener = tokio::net::TcpListener::bind((args.host.as_str(), args.port))
        .await
        .with_context(|| format!("failed to bind {}:{}", args.host, args.port))?;
    let local = listener
        .local_addr()
        .map(|addr| addr.to_string())
        .unwrap_or_else(|_| format!("{}:{}", args.host, args.port));
    eprintln!(
        "ts-stream: listening on {local}; streaming one UHF channel (13..=62) to a single client"
    );

    let shutdown_manager = manager.clone();
    let shutdown = async move {
        shutdown_signal().await;
        eprintln!("ts-stream: shutdown signal received, stopping receiver...");
        shutdown_manager.shutdown().await;
    };

    axum::serve(listener, app)
        .with_graceful_shutdown(shutdown)
        .await
        .context("HTTP server failed")?;
    Ok(())
}

#[cfg(unix)]
async fn shutdown_signal() {
    use tokio::signal::unix::{signal, SignalKind};

    let mut interrupt = match signal(SignalKind::interrupt()) {
        Ok(stream) => stream,
        Err(error) => {
            eprintln!("ts-stream: failed to install SIGINT handler: {error}");
            return;
        }
    };
    let mut terminate = match signal(SignalKind::terminate()) {
        Ok(stream) => stream,
        Err(error) => {
            eprintln!("ts-stream: failed to install SIGTERM handler: {error}");
            return;
        }
    };

    tokio::select! {
        _ = interrupt.recv() => {}
        _ = terminate.recv() => {}
    }
}

#[cfg(not(unix))]
async fn shutdown_signal() {
    // The project targets Linux only; this keeps non-Linux compiles honest.
    let _ = tokio::signal::ctrl_c().await;
}
