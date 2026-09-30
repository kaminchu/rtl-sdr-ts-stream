//! Strict query parsing and `/version` contract.

mod common;

use std::sync::Arc;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use tower::ServiceExt;
use ts_stream::http::{build_router, parse_channel, ChannelError};
use ts_stream::session::{
    channel_frequency_hz, SessionConfig, SessionManager, StreamState, UnavailableBackend,
    UHF_MAX_CHANNEL,
};

fn unavailable_manager() -> SessionManager<UnavailableBackend> {
    SessionManager::new(Arc::new(UnavailableBackend), SessionConfig::default())
}

async fn status_of(manager: &SessionManager<UnavailableBackend>, uri: &str) -> StatusCode {
    let app = build_router(manager.clone());
    let request = Request::builder()
        .uri(uri)
        .body(Body::empty())
        .expect("request");
    app.oneshot(request).await.expect("infallible").status()
}

#[test]
fn channel_frequency_plan_is_correct_and_bounded() {
    assert_eq!(channel_frequency_hz(13), Some(473_142_857));
    assert_eq!(channel_frequency_hz(14), Some(479_142_857));
    assert_eq!(channel_frequency_hz(62), Some(767_142_857));

    assert_eq!(channel_frequency_hz(12), None);
    assert_eq!(channel_frequency_hz(63), None);
    assert_eq!(channel_frequency_hz(0), None);
    assert_eq!(channel_frequency_hz(u32::MAX), None);
}

#[test]
fn parse_channel_is_strict() {
    assert_eq!(parse_channel(Some("ch=13")), Ok(13));
    assert_eq!(parse_channel(Some("ch=19")), Ok(19));
    assert_eq!(parse_channel(Some("ch=62")), Ok(UHF_MAX_CHANNEL));

    assert_eq!(parse_channel(None), Err(ChannelError::Missing));
    assert_eq!(parse_channel(Some("")), Err(ChannelError::Missing));
    assert_eq!(
        parse_channel(Some("other=19")),
        Err(ChannelError::UnknownParameter)
    );
    assert_eq!(parse_channel(Some("ch=abc")), Err(ChannelError::NotANumber));
    assert_eq!(parse_channel(Some("ch=-1")), Err(ChannelError::NotANumber));
    assert_eq!(
        parse_channel(Some("ch=12")),
        Err(ChannelError::OutOfRange(12))
    );
    assert_eq!(
        parse_channel(Some("ch=63")),
        Err(ChannelError::OutOfRange(63))
    );
    assert_eq!(
        parse_channel(Some("ch=19&x=1")),
        Err(ChannelError::UnknownParameter)
    );
    assert_eq!(
        parse_channel(Some("ch=19&ch=20")),
        Err(ChannelError::DuplicateChannel)
    );
    assert_eq!(parse_channel(Some("ch")), Err(ChannelError::Malformed));
}

#[tokio::test]
async fn invalid_queries_return_400() {
    let manager = unavailable_manager();
    for uri in [
        "/stream",
        "/stream?",
        "/stream?ch=",
        "/stream?ch=abc",
        "/stream?ch=12",
        "/stream?ch=63",
        "/stream?ch=19&foo=1",
        "/stream?ch=19&ch=20",
    ] {
        assert_eq!(
            status_of(&manager, uri).await,
            StatusCode::BAD_REQUEST,
            "expected 400 for {uri}"
        );
    }
}

#[tokio::test]
async fn valid_query_reaches_the_backend_and_reports_503() {
    let manager = unavailable_manager();
    // A valid channel is accepted by the query layer; the unavailable backend
    // then yields 503 (never 400).
    assert_eq!(
        status_of(&manager, "/stream?ch=13").await,
        StatusCode::SERVICE_UNAVAILABLE
    );
    assert_eq!(
        status_of(&manager, "/stream?ch=62").await,
        StatusCode::SERVICE_UNAVAILABLE
    );
    assert!(!manager.is_reserved(), "failed start must release the slot");
}

#[tokio::test]
async fn version_json_is_stable_and_state_independent() {
    let manager = unavailable_manager();
    let states = [
        StreamState::Idle,
        StreamState::Tuning,
        StreamState::Locking,
        StreamState::Streaming,
        StreamState::Reacquiring,
        StreamState::Stopping,
        StreamState::Error,
    ];
    for state in states {
        manager.debug_set_state(state);
        let app = build_router(manager.clone());
        let response = app
            .oneshot(common::version_request())
            .await
            .expect("infallible");
        assert_eq!(response.status(), StatusCode::OK, "state {state:?}");
        let bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .expect("body");
        let json: serde_json::Value = serde_json::from_slice(&bytes).expect("json");
        assert_eq!(json["name"], "ts-stream");
        assert_eq!(json["version"], env!("CARGO_PKG_VERSION"));
        assert!(
            !manager.is_reserved(),
            "/version must not reserve the stream slot"
        );
    }
}

#[tokio::test]
async fn version_works_when_the_device_backend_is_unavailable() {
    let manager = unavailable_manager();
    let app = build_router(manager.clone());
    let response = app
        .oneshot(common::version_request())
        .await
        .expect("infallible");
    assert_eq!(response.status(), StatusCode::OK);
}

#[tokio::test]
async fn watch_state_updates_without_existing_subscribers() {
    let manager = unavailable_manager();
    // No receiver is subscribed while the state changes. `watch::Sender::send`
    // is a no-op without subscribers; the published value must still be visible
    // to a late observer.
    manager.debug_set_state(StreamState::Reacquiring);
    let late = manager.subscribe();
    assert_eq!(*late.borrow(), StreamState::Reacquiring);

    manager.debug_set_state(StreamState::Idle);
    assert_eq!(*manager.subscribe().borrow(), StreamState::Idle);
}
