//! Real-socket HTTP/1.1 contract: `video/mp2t` and chunked framing.
//!
//! These tests bind a TCP listener and speak raw HTTP/1.1 so the transport
//! framing produced by axum/hyper is observable, rather than hidden behind
//! `tower`'s in-process service call.

mod common;

use std::sync::Arc;
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use ts_stream::session::{SessionConfig, SessionManager};

use common::{MockBackend, OpenBehavior, Step};

async fn read_until_headers(socket: &mut TcpStream) -> String {
    let mut buffer = Vec::new();
    let mut byte = [0u8; 1];
    loop {
        let read = socket.read(&mut byte).await.expect("read header");
        assert_ne!(read, 0, "connection closed before headers completed");
        buffer.push(byte[0]);
        if buffer.ends_with(b"\r\n\r\n") {
            return String::from_utf8_lossy(&buffer).to_string();
        }
        assert!(buffer.len() < 64 * 1024, "response headers too large");
    }
}

#[tokio::test]
async fn successful_stream_uses_video_mp2t_and_chunked_framing() {
    let backend = MockBackend::new();
    let chunk = vec![0x47u8; 188];
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(chunk.clone())]));
    let config = SessionConfig {
        poll_interval: Duration::from_secs(5),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = SessionManager::new(Arc::new(backend.clone()), config);
    let app = ts_stream::http::build_router(manager.clone());

    let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
        .await
        .expect("bind");
    let address = listener.local_addr().expect("addr");
    let server = tokio::spawn(async move {
        let _ = axum::serve(listener, app).await;
    });

    let mut socket = TcpStream::connect(address).await.expect("connect");
    socket
        .write_all(b"GET /stream?ch=19 HTTP/1.1\r\nHost: localhost\r\n\r\n")
        .await
        .expect("write request");

    let headers = tokio::time::timeout(Duration::from_secs(5), read_until_headers(&mut socket))
        .await
        .expect("headers in time");
    let headers_lower = headers.to_ascii_lowercase();
    assert!(
        headers.starts_with("HTTP/1.1 200"),
        "unexpected status line: {headers}"
    );
    assert!(
        headers_lower.contains("content-type: video/mp2t"),
        "missing TS content type: {headers}"
    );
    assert!(
        headers_lower.contains("transfer-encoding: chunked"),
        "stream must be chunked: {headers}"
    );
    assert!(
        !headers_lower.contains("content-length"),
        "chunked streaming must not set content-length: {headers}"
    );

    // Body must be chunked-framed TS, not JSON. Read the chunk-size line and
    // parse it as hex; HTTP hex digits are case-insensitive (`bc` or `BC`).
    let mut size_line = Vec::new();
    while !size_line.ends_with(b"\r\n") {
        let mut byte = [0u8; 1];
        tokio::time::timeout(Duration::from_secs(5), socket.read_exact(&mut byte))
            .await
            .expect("chunk size line in time")
            .expect("read chunk size line");
        size_line.push(byte[0]);
        assert!(size_line.len() <= 16, "chunk size line too long");
    }
    let size_text =
        std::str::from_utf8(&size_line[..size_line.len() - 2]).expect("ascii chunk size");
    let chunk_size = u64::from_str_radix(size_text, 16).expect("hex chunk size");
    assert_eq!(
        chunk_size,
        chunk.len() as u64,
        "chunk size must equal the TS payload length"
    );

    let mut body = vec![0u8; chunk_size as usize];
    tokio::time::timeout(Duration::from_secs(5), socket.read_exact(&mut body))
        .await
        .expect("chunk data in time")
        .expect("read chunk data");
    assert_eq!(body, chunk, "unexpected chunk data");

    // The first lock was delivered; the stream is alive. Kill the test server.
    server.abort();
    let _ = server.await;
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    receiver.push(Step::Stopped);
    let _ = manager.wait_released(Duration::from_secs(2)).await;
}

#[tokio::test]
async fn busy_stream_returns_409_on_the_wire() {
    let backend = MockBackend::new();
    backend.push_behavior(OpenBehavior::Receiver(vec![Step::Data(vec![0x47; 188])]));
    let config = SessionConfig {
        poll_interval: Duration::from_secs(5),
        reacquire_timeout: Duration::from_secs(5),
        ..common::test_config()
    };
    let manager = SessionManager::new(Arc::new(backend.clone()), config);
    let app = ts_stream::http::build_router(manager.clone());

    let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
        .await
        .expect("bind");
    let address = listener.local_addr().expect("addr");
    let server = tokio::spawn(async move {
        let _ = axum::serve(listener, app).await;
    });

    // Hold the first stream open.
    let mut first = TcpStream::connect(address).await.expect("connect");
    first
        .write_all(b"GET /stream?ch=19 HTTP/1.1\r\nHost: localhost\r\n\r\n")
        .await
        .expect("write");
    let headers = tokio::time::timeout(Duration::from_secs(5), read_until_headers(&mut first))
        .await
        .expect("headers");
    assert!(headers.starts_with("HTTP/1.1 200"), "{headers}");

    // A second connection must be rejected with 409.
    let mut second = TcpStream::connect(address).await.expect("connect");
    second
        .write_all(b"GET /stream?ch=20 HTTP/1.1\r\nHost: localhost\r\n\r\n")
        .await
        .expect("write");
    let headers = tokio::time::timeout(Duration::from_secs(5), read_until_headers(&mut second))
        .await
        .expect("headers");
    assert!(headers.starts_with("HTTP/1.1 409"), "{headers}");

    server.abort();
    let _ = server.await;
    let receiver = backend.wait_for_receiver(Duration::from_secs(2)).await;
    receiver.push(Step::Stopped);
    let _ = manager.wait_released(Duration::from_secs(2)).await;
}
