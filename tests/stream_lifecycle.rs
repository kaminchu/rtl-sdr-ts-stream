//! Exclusivity, state machine and deadline contracts.

mod common;

use std::sync::Arc;
use std::time::Duration;

use axum::http::StatusCode;
use tower::ServiceExt;
use ts_stream::session::{SessionConfig, SessionManager, StreamState};

use common::{Gate, MockBackend, OpenBehavior, Step};

fn manager_with(backend: MockBackend, config: SessionConfig) -> SessionManager<MockBackend> {
    SessionManager::new(Arc::new(backend), config)
}

fn data_chunk() -> Step {
    Step::Data(vec![0x47u8; 188])
}

#[tokio::test]
async fn reservation_is_taken_during_tuning_and_blocks_other_requests() {
    let backend = MockBackend::new();
    let gate = Gate::new();
    backend.park_open(gate.clone());
    backend.push_behavior(OpenBehavior::Unavailable("no device in test".into()));

    let config = SessionConfig {
        initial_lock_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend, config);
    let app = ts_stream::http::build_router(manager.clone());

    let first = tokio::spawn({
        let app = app.clone();
        async move { app.oneshot(common::stream_request(19)).await }
    });

    common::wait_for_state(&manager, StreamState::Tuning, Duration::from_secs(2)).await;
    assert!(
        manager.is_reserved(),
        "slot must be reserved from tuning onward"
    );
    assert_eq!(manager.active_channel(), Some(19));

    // Same channel and a different channel are both rejected while reserved.
    let same = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(same.status(), StatusCode::CONFLICT);
    let other = app
        .clone()
        .oneshot(common::stream_request(20))
        .await
        .unwrap();
    assert_eq!(other.status(), StatusCode::CONFLICT);

    gate.release();
    let first = first.await.unwrap().unwrap();
    assert_eq!(first.status(), StatusCode::SERVICE_UNAVAILABLE);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    assert!(!manager.is_reserved());
}

#[tokio::test]
async fn unavailable_backend_returns_503_and_releases() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Unavailable("librtlsdr missing".into()));
    let manager = manager_with(backend, common::test_config());
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn unsupported_decoder_returns_503_with_explicit_diagnostic() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Unsupported(
        "ISDB-T demodulator not implemented".into(),
    ));
    let manager = manager_with(backend, common::test_config());
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    let body = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .unwrap();
    let text = String::from_utf8_lossy(&body);
    assert!(
        text.contains("decoder not implemented"),
        "diagnostic must be explicit, got: {text}"
    );
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn initial_lock_timeout_returns_504_after_cleanup() {
    let backend = MockBackend::new();
    // Default behaviour is an empty receiver: only `NoSignal` until the deadline.
    let config = SessionConfig {
        initial_lock_timeout: Duration::from_millis(80),
        poll_interval: Duration::from_millis(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::GATEWAY_TIMEOUT);
    assert!(manager.wait_released(Duration::from_secs(2)).await);

    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    assert!(
        receiver.stop_calls() >= 1,
        "the receiver must be stopped on initial-lock timeout"
    );
}

#[tokio::test]
async fn pending_before_lock_is_not_a_lock() {
    let backend = MockBackend::new();
    // The receiver is still acquiring: two `Pending` events must not be treated
    // as a lock (no `200`) and must not end the initial deadline. The third
    // event is real TS and completes the lock.
    backend.push_behavior(OpenBehavior::Receiver(vec![
        Step::Pending,
        Step::Pending,
        data_chunk(),
    ]));
    let config = SessionConfig {
        initial_lock_timeout: Duration::from_secs(5),
        poll_interval: Duration::from_millis(10),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    common::wait_for_state(&manager, StreamState::Streaming, Duration::from_secs(2)).await;

    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    receiver.push(Step::Stopped);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn busy_locking_returns_409() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(Vec::new()));
    let config = SessionConfig {
        initial_lock_timeout: Duration::from_secs(10),
        ..common::test_config()
    };
    let manager = manager_with(backend, config);
    let app = ts_stream::http::build_router(manager.clone());

    let first = tokio::spawn({
        let app = app.clone();
        async move { app.oneshot(common::stream_request(19)).await }
    });
    common::wait_for_state(&manager, StreamState::Locking, Duration::from_secs(2)).await;

    let second = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(second.status(), StatusCode::CONFLICT);

    manager.shutdown_with_timeout(Duration::from_secs(2)).await;
    let first = first.await.unwrap().unwrap();
    assert!(first.status().is_server_error() || first.status() == StatusCode::SERVICE_UNAVAILABLE);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn busy_streaming_returns_409_for_same_and_other_channel() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(vec![data_chunk()]));
    let config = SessionConfig {
        poll_interval: Duration::from_secs(5),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    common::wait_for_state(&manager, StreamState::Streaming, Duration::from_secs(2)).await;

    let same = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(same.status(), StatusCode::CONFLICT);
    let other = app
        .clone()
        .oneshot(common::stream_request(42))
        .await
        .unwrap();
    assert_eq!(other.status(), StatusCode::CONFLICT);

    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    receiver.push(Step::Stopped);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn busy_reacquiring_returns_409() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(vec![data_chunk()]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_secs(10),
        ..common::test_config()
    };
    let manager = manager_with(backend, config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    common::wait_for_state(&manager, StreamState::Reacquiring, Duration::from_secs(2)).await;

    let second = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(second.status(), StatusCode::CONFLICT);

    manager.shutdown_with_timeout(Duration::from_secs(2)).await;
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn busy_stopping_returns_409_until_cleanup_completes() {
    let backend = MockBackend::new();
    let stop_gate = Gate::new();
    backend.park_stop_on_open(stop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(vec![data_chunk()]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_millis(60),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);

    // Lock is lost, reacquire times out, worker moves to `stopping`; `stop()`
    // blocks on the gate, so the slot must remain reserved.
    common::wait_for_state(&manager, StreamState::Stopping, Duration::from_secs(2)).await;
    assert!(manager.is_reserved());

    let second = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(second.status(), StatusCode::CONFLICT);

    stop_gate.release();
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    assert_eq!(receiver.stop_calls(), 1);
}

#[tokio::test]
async fn version_requests_do_not_disturb_an_active_stream() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(vec![data_chunk()]));
    let config = SessionConfig {
        poll_interval: Duration::from_secs(5),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    common::wait_for_state(&manager, StreamState::Streaming, Duration::from_secs(2)).await;

    for _ in 0..5 {
        let version = app
            .clone()
            .oneshot(common::version_request())
            .await
            .unwrap();
        assert_eq!(version.status(), StatusCode::OK);
    }

    assert!(manager.is_reserved());
    assert_eq!(manager.active_channel(), Some(19));
    assert_eq!(manager.state(), StreamState::Streaming);

    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    receiver.push(Step::Stopped);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}
