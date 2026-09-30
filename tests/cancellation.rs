//! Cancellation, overflow, shutdown and worker-panic cleanup.

mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};

use axum::http::StatusCode;
use tower::ServiceExt;
use ts_stream::session::{SessionConfig, SessionManager, StreamStartError, StreamState};

use common::{Gate, MockBackend, OpenBehavior, Step};

fn manager_with(backend: MockBackend, config: SessionConfig) -> SessionManager<MockBackend> {
    SessionManager::new(Arc::new(backend), config)
}

async fn wait_for<F: Fn() -> bool>(predicate: F, timeout: Duration, what: &str) {
    let deadline = Instant::now() + timeout;
    while !predicate() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

#[tokio::test]
async fn disconnect_after_headers_stops_receiver_and_releases() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;

    // Simulate the client dropping the connection after headers.
    drop(response);

    assert!(
        manager.wait_released(Duration::from_secs(2)).await,
        "disconnect must release the slot"
    );
    assert!(receiver.stop_calls() >= 1, "receiver must be stopped");
    assert!(receiver.is_stopped(), "receiver join must have completed");
}

#[tokio::test]
async fn disconnect_before_headers_cancels_and_releases() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(Vec::new()));
    let config = SessionConfig {
        initial_lock_timeout: Duration::from_secs(30),
        poll_interval: Duration::from_millis(20),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let handle = tokio::spawn({
        let app = app.clone();
        async move { app.oneshot(common::stream_request(19)).await }
    });
    common::wait_for_state(&manager, StreamState::Locking, Duration::from_secs(2)).await;
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    assert!(manager.is_reserved());

    handle.abort();
    let _ = handle.await;

    assert!(
        manager.wait_released(Duration::from_secs(2)).await,
        "pre-header disconnect must release the slot"
    );
    assert!(receiver.stop_calls() >= 1);
}

#[tokio::test]
async fn reservation_is_released_only_after_stop_returns() {
    let backend = MockBackend::new();
    let stop_gate = Gate::new();
    backend.park_stop_on_open(stop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    drop(response);

    // `stop()` is entered and parked: the slot must still be reserved.
    wait_for(
        || receiver.stop_calls() >= 1,
        Duration::from_secs(2),
        "stop() to be entered",
    )
    .await;
    assert!(
        manager.is_reserved(),
        "slot must not be released while stop() is still running"
    );

    stop_gate.release();
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    assert!(!manager.is_reserved());
}

#[tokio::test]
async fn bounded_queue_overflow_disconnects_client_and_releases() {
    let backend = MockBackend::new();
    let steps: Vec<Step> = (0..128).map(|_| Step::Data(vec![0x47; 188])).collect();
    backend.push_behavior(OpenBehavior::Receiver(steps));
    let config = SessionConfig {
        queue_capacity: 2,
        poll_interval: Duration::from_millis(5),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend, config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);

    // Do not read the body: the bounded queue must overflow and the worker must
    // close the connection.
    wait_for(
        || manager.overflow_count() >= 1,
        Duration::from_secs(2),
        "queue overflow",
    )
    .await;
    assert!(manager.wait_released(Duration::from_secs(2)).await);

    let bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .unwrap();
    assert!(
        !bytes.is_empty(),
        "the buffered chunks before the overflow must still be delivered"
    );
    assert!(
        bytes.len() <= 3 * 188,
        "overflow must not buffer more than roughly the queue capacity"
    );
}

#[tokio::test]
async fn worker_panic_still_stops_receiver_and_releases() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Panic]));
    let manager = manager_with(backend.clone(), common::test_config());
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);

    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    assert!(
        receiver.stop_calls() >= 1,
        "panic unwinding must still stop the receiver"
    );
    assert!(
        receiver.is_stopped(),
        "panic unwinding must complete the receiver stop"
    );
}

#[tokio::test]
async fn shutdown_signal_path_awaits_cleanup() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    assert!(manager.is_reserved());

    // This is exactly what the SIGINT/SIGTERM handler invokes.
    manager.shutdown_with_timeout(Duration::from_secs(2)).await;

    assert!(!manager.is_reserved(), "shutdown must await slot release");
    assert_eq!(manager.state(), StreamState::Idle);
    assert!(receiver.stop_calls() >= 1);
}

#[tokio::test]
async fn released_slot_accepts_the_next_request() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Unavailable("first attempt".into()));
    backend.push_behavior(OpenBehavior::Unavailable("second attempt".into()));
    let manager = manager_with(backend, common::test_config());
    let app = ts_stream::http::build_router(manager.clone());

    let first = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(first.status(), StatusCode::SERVICE_UNAVAILABLE);
    assert!(manager.wait_released(Duration::from_secs(2)).await);

    let second = app.oneshot(common::stream_request(20)).await.unwrap();
    assert_eq!(second.status(), StatusCode::SERVICE_UNAVAILABLE);
}

#[tokio::test]
async fn reservation_survives_until_receiver_destructor_completes() {
    // True join ordering: `stop()` returns, then the receiver destructor is
    // parked. The slot must stay reserved until the destructor finishes and the
    // worker thread is joined.
    let backend = MockBackend::new();
    let drop_gate = Gate::new();
    backend.park_drop_on_open(drop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;

    drop(response);

    wait_for(
        || receiver.drop_calls() >= 1,
        Duration::from_secs(2),
        "receiver destructor to be entered",
    )
    .await;
    assert!(
        receiver.stop_calls() >= 1,
        "stop() must have returned first"
    );
    assert!(
        manager.is_reserved(),
        "slot must not be released until the destructor/join finishes"
    );
    assert_eq!(manager.state(), StreamState::Stopping);
    assert!(!receiver.is_dropped());

    drop_gate.release();
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    assert!(
        receiver.is_dropped(),
        "destructor must complete before release"
    );
    assert_eq!(manager.state(), StreamState::Idle);
}

#[tokio::test]
async fn worker_panic_releases_only_after_receiver_destructor() {
    let backend = MockBackend::new();
    let drop_gate = Gate::new();
    backend.park_drop_on_open(drop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Panic]));
    let manager = manager_with(backend.clone(), common::test_config());
    let app = ts_stream::http::build_router(manager.clone());

    let handle = tokio::spawn({
        let app = app.clone();
        async move { app.oneshot(common::stream_request(19)).await }
    });
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;

    wait_for(
        || receiver.drop_calls() >= 1,
        Duration::from_secs(2),
        "receiver destructor to be entered during unwinding",
    )
    .await;
    assert!(
        manager.is_reserved(),
        "slot must survive the whole unwinding/join"
    );
    assert_eq!(manager.state(), StreamState::Stopping);

    drop_gate.release();
    let response = handle.await.unwrap().unwrap();
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    assert!(receiver.is_stopped());
}

#[tokio::test]
async fn initial_lock_timeout_transitions_through_stopping() {
    let backend = MockBackend::new();
    let stop_gate = Gate::new();
    backend.park_stop_on_open(stop_gate.clone());
    // Default behaviour is an empty receiver: only `NoSignal` until the deadline.
    let config = SessionConfig {
        initial_lock_timeout: Duration::from_millis(40),
        poll_interval: Duration::from_millis(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let handle = tokio::spawn({
        let app = app.clone();
        async move { app.oneshot(common::stream_request(19)).await }
    });

    common::wait_for_state(&manager, StreamState::Stopping, Duration::from_secs(2)).await;
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    assert!(receiver.stop_calls() >= 1, "stop must be entered");
    assert!(
        manager.is_reserved(),
        "slot held while the timeout cleanup is blocked in stop()"
    );

    stop_gate.release();
    let response = handle.await.unwrap().unwrap();
    assert_eq!(response.status(), StatusCode::GATEWAY_TIMEOUT);
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    assert_eq!(manager.state(), StreamState::Idle);
}

#[tokio::test]
async fn disconnect_after_headers_transitions_through_stopping() {
    let backend = MockBackend::new();
    let stop_gate = Gate::new();
    backend.park_stop_on_open(stop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;

    drop(response);

    common::wait_for_state(&manager, StreamState::Stopping, Duration::from_secs(2)).await;
    assert!(manager.is_reserved());
    wait_for(
        || receiver.stop_calls() >= 1,
        Duration::from_secs(2),
        "stop() to be entered on disconnect",
    )
    .await;

    stop_gate.release();
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn disconnect_before_headers_transitions_through_stopping() {
    let backend = MockBackend::new();
    let stop_gate = Gate::new();
    backend.park_stop_on_open(stop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(Vec::new()));
    let config = SessionConfig {
        initial_lock_timeout: Duration::from_secs(30),
        poll_interval: Duration::from_millis(20),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let handle = tokio::spawn({
        let app = app.clone();
        async move { app.oneshot(common::stream_request(19)).await }
    });
    common::wait_for_state(&manager, StreamState::Locking, Duration::from_secs(2)).await;
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;

    handle.abort();
    let _ = handle.await;

    common::wait_for_state(&manager, StreamState::Stopping, Duration::from_secs(2)).await;
    assert!(manager.is_reserved());
    wait_for(
        || receiver.stop_calls() >= 1,
        Duration::from_secs(2),
        "stop() to be entered on pre-header disconnect",
    )
    .await;

    stop_gate.release();
    assert!(manager.wait_released(Duration::from_secs(2)).await);
}

#[tokio::test]
async fn shutdown_waits_for_full_cleanup_without_giving_up() {
    let backend = MockBackend::new();
    let stop_gate = Gate::new();
    backend.park_stop_on_open(stop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let config = SessionConfig {
        poll_interval: Duration::from_millis(10),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = manager_with(backend.clone(), config);
    let app = ts_stream::http::build_router(manager.clone());

    let response = app.oneshot(common::stream_request(19)).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;

    let shutdown_manager = manager.clone();
    let shutdown = tokio::spawn(async move { shutdown_manager.shutdown().await });

    // Production shutdown must park on the blocked cleanup instead of returning
    // after a fixed grace period and pretending the slot was released.
    common::wait_for_state(&manager, StreamState::Stopping, Duration::from_secs(2)).await;
    assert!(manager.is_reserved());
    assert!(
        !shutdown.is_finished(),
        "shutdown must keep waiting for the join"
    );

    stop_gate.release();
    shutdown.await.unwrap();
    assert!(!manager.is_reserved());
    assert_eq!(manager.state(), StreamState::Idle);
    assert!(receiver.is_stopped());
}

#[tokio::test]
async fn shutdown_denies_future_reservations() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Unavailable("before shutdown".into()));
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let manager = manager_with(backend, common::test_config());
    let app = ts_stream::http::build_router(manager.clone());

    let first = app
        .clone()
        .oneshot(common::stream_request(19))
        .await
        .unwrap();
    assert_eq!(first.status(), StatusCode::SERVICE_UNAVAILABLE);
    assert!(!manager.is_reserved());

    assert!(
        manager.shutdown_with_timeout(Duration::from_secs(2)).await,
        "shutdown with no active stream completes immediately"
    );

    // A new request must be refused rather than opening the device again.
    let second = app
        .clone()
        .oneshot(common::stream_request(20))
        .await
        .unwrap();
    assert_eq!(second.status(), StatusCode::CONFLICT);
    assert!(!manager.is_reserved());

    assert!(matches!(
        manager.start_stream(21).await,
        Err(StreamStartError::Busy)
    ));
}

#[tokio::test]
async fn shutdown_during_gated_open_keeps_stopping_until_cleanup() {
    // Deterministic half: once shutdown has begun, a worker unwinding out of a
    // blocking backend call must not be able to publish a progress state over
    // `stopping`. This does not depend on catching the tiny worker race window.
    let idle = manager_with(MockBackend::new(), common::test_config());
    assert!(idle.shutdown_with_timeout(Duration::from_millis(50)).await);
    for progress in [
        StreamState::Tuning,
        StreamState::Locking,
        StreamState::Streaming,
        StreamState::Reacquiring,
    ] {
        idle.debug_set_state(progress);
        assert_eq!(
            idle.state(),
            StreamState::Idle,
            "shutdown must suppress a backwards transition to {progress:?}"
        );
    }

    // End-to-end half: park `open`, begin shutdown, then let `open` return. The
    // worker may not preempt the blocking open, so it only notices cancellation
    // afterwards; `stopping` must hold through open, cancellation and the whole
    // cleanup, and only the supervisor's join may return the slot to `idle`.
    let backend = MockBackend::new();
    let open_gate = Gate::new();
    let stop_gate = Gate::new();
    let drop_gate = Gate::new();
    backend.park_open(open_gate.clone());
    backend.park_stop_on_open(stop_gate.clone());
    backend.park_drop_on_open(drop_gate.clone());
    backend.push_behavior(OpenBehavior::Receiver(Vec::new()));
    let manager = manager_with(backend.clone(), common::test_config());
    let app = ts_stream::http::build_router(manager.clone());

    let request = tokio::spawn({
        let app = app.clone();
        async move { app.oneshot(common::stream_request(19)).await }
    });
    common::wait_for_state(&manager, StreamState::Tuning, Duration::from_secs(2)).await;
    assert!(manager.is_reserved());

    let shutdown_manager = manager.clone();
    let shutdown = tokio::spawn(async move { shutdown_manager.shutdown().await });

    // Shutdown publishes `stopping` while `open` is still parked in the backend.
    common::wait_for_state(&manager, StreamState::Stopping, Duration::from_secs(2)).await;
    assert!(manager.is_reserved());

    // Let `open` return and reach cleanup; `stop()` is parked so we can observe.
    open_gate.release();
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    wait_for(
        || receiver.stop_calls() >= 1,
        Duration::from_secs(2),
        "stop() to be entered after the gated open returns",
    )
    .await;
    assert_eq!(
        manager.state(),
        StreamState::Stopping,
        "the worker must not republish a progress state after shutdown"
    );
    assert!(manager.is_reserved());

    // Destructor parked: `stopping` must survive the join too.
    stop_gate.release();
    wait_for(
        || receiver.drop_calls() >= 1,
        Duration::from_secs(2),
        "receiver destructor to be entered",
    )
    .await;
    assert_eq!(manager.state(), StreamState::Stopping);
    assert!(manager.is_reserved());

    drop_gate.release();
    shutdown.await.unwrap();
    let _ = request.await.unwrap();
    assert!(manager.wait_released(Duration::from_secs(2)).await);
    assert_eq!(manager.state(), StreamState::Idle);
    assert!(receiver.is_stopped());
}
