//! HTTP/1.1 surface: `GET /version` and `GET /stream`.
//!
//! `/version` is deliberately independent of the session and the USB/DSP state:
//! it never touches the reservation and always answers `200` JSON while the
//! server accepts requests.
//!
//! `/stream` validates the query strictly, then delegates to
//! [`SessionManager::start_stream`]. Status mapping:
//!
//! * `400` malformed/missing/duplicate `ch`, unknown query parameters, or a
//!   channel outside `13..=62`.
//! * `409` the single stream slot is reserved (`tuning`/`locking`/`streaming`/
//!   `reacquiring`/`stopping`), regardless of requested channel.
//! * `503` device/library unavailable, an unsupported receiver configuration
//!   (rejected sample rate / resampling), or an internal start failure. The
//!   response body still uses the legacy `decoder not implemented:` prefix for
//!   the rejected-configuration case.
//! * `504` the initial lock deadline expired.
//!
//! A successful response carries `Content-Type: video/mp2t` and a streaming body
//! (chunked framing is supplied by the HTTP stack). Only the first valid lock
//! produces that response, so non-TS bytes are never sent as an error body after
//! a `200`.

use std::convert::Infallible;
use std::pin::Pin;
use std::task::{Context, Poll};

use axum::body::Body;
use axum::extract::{RawQuery, State};
use axum::http::{header, HeaderValue, Response, StatusCode};
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use futures_core::Stream;
use serde::Serialize;
use tokio::sync::mpsc;

use crate::session::{Backend, SessionManager, StreamStartError, UHF_MAX_CHANNEL, UHF_MIN_CHANNEL};

/// `/version` payload. `git` is present only when a commit id was supplied at
/// build time.
#[derive(Debug, Clone, Serialize)]
pub struct VersionInfo {
    pub name: &'static str,
    pub version: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub git: Option<&'static str>,
}

impl VersionInfo {
    /// Crate name reported by `/version`.
    pub const NAME: &'static str = "ts-stream";

    /// Current build information.
    pub fn current() -> Self {
        Self {
            name: Self::NAME,
            version: crate::VERSION,
            git: option_env!("TS_STREAM_GIT"),
        }
    }
}

/// Why a `/stream` query is invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelError {
    Missing,
    Malformed,
    UnknownParameter,
    DuplicateChannel,
    NotANumber,
    OutOfRange(u32),
}

impl std::fmt::Display for ChannelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ChannelError::Missing => {
                write!(f, "missing `ch` query parameter")
            }
            ChannelError::Malformed => write!(f, "malformed query string"),
            ChannelError::UnknownParameter => write!(f, "unknown query parameter"),
            ChannelError::DuplicateChannel => write!(f, "duplicate `ch` query parameter"),
            ChannelError::NotANumber => write!(f, "`ch` must be a number"),
            ChannelError::OutOfRange(channel) => write!(
                f,
                "channel {channel} out of range {UHF_MIN_CHANNEL}..={UHF_MAX_CHANNEL}"
            ),
        }
    }
}

/// Strictly parse `?ch=NN`.
///
/// Accepts exactly one `ch` parameter and nothing else, so typos do not silently
/// default to a different channel.
pub fn parse_channel(raw: Option<&str>) -> Result<u32, ChannelError> {
    let raw = raw.ok_or(ChannelError::Missing)?;
    if raw.is_empty() {
        return Err(ChannelError::Missing);
    }
    let mut channel: Option<u32> = None;
    for pair in raw.split('&') {
        if pair.is_empty() {
            return Err(ChannelError::Malformed);
        }
        let (key, value) = pair.split_once('=').ok_or(ChannelError::Malformed)?;
        if key != "ch" {
            return Err(ChannelError::UnknownParameter);
        }
        if channel.is_some() {
            return Err(ChannelError::DuplicateChannel);
        }
        let parsed = value.parse::<u32>().map_err(|_| ChannelError::NotANumber)?;
        if !(UHF_MIN_CHANNEL..=UHF_MAX_CHANNEL).contains(&parsed) {
            return Err(ChannelError::OutOfRange(parsed));
        }
        channel = Some(parsed);
    }
    channel.ok_or(ChannelError::Missing)
}

/// Build the router for a session manager.
pub fn build_router<B: Backend>(manager: SessionManager<B>) -> Router {
    Router::new()
        .route("/version", get(version_handler))
        .route("/stream", get(stream_handler::<B>))
        .with_state(manager)
}

async fn version_handler() -> impl IntoResponse {
    axum::Json(VersionInfo::current())
}

async fn stream_handler<B: Backend>(
    State(manager): State<SessionManager<B>>,
    RawQuery(raw): RawQuery,
) -> Response<Body> {
    match parse_channel(raw.as_deref()) {
        Err(error) => error_response(StatusCode::BAD_REQUEST, error.to_string()),
        Ok(channel) => match manager.start_stream(channel).await {
            Ok(stream) => {
                let body = Body::from_stream(SessionStream {
                    rx: stream.into_body_receiver(),
                });
                let mut response = Response::new(body);
                response
                    .headers_mut()
                    .insert(header::CONTENT_TYPE, HeaderValue::from_static("video/mp2t"));
                response
            }
            Err(StreamStartError::BadChannel(channel)) => error_response(
                StatusCode::BAD_REQUEST,
                format!("channel {channel} out of range {UHF_MIN_CHANNEL}..={UHF_MAX_CHANNEL}"),
            ),
            Err(StreamStartError::Busy) => error_response(
                StatusCode::CONFLICT,
                "stream slot already in use".to_string(),
            ),
            Err(StreamStartError::InitialLockTimeout) => error_response(
                StatusCode::GATEWAY_TIMEOUT,
                "initial lock timeout".to_string(),
            ),
            Err(error) => error_response(StatusCode::SERVICE_UNAVAILABLE, error.to_string()),
        },
    }
}

fn error_response(status: StatusCode, message: String) -> Response<Body> {
    (status, message).into_response()
}

/// Streaming body that yields TS chunks as the worker produces them.
pub struct SessionStream {
    rx: mpsc::Receiver<Vec<u8>>,
}

impl Stream for SessionStream {
    type Item = Result<Vec<u8>, Infallible>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();
        match this.rx.poll_recv(cx) {
            Poll::Ready(Some(chunk)) => Poll::Ready(Some(Ok(chunk))),
            Poll::Ready(None) => Poll::Ready(None),
            Poll::Pending => Poll::Pending,
        }
    }
}
