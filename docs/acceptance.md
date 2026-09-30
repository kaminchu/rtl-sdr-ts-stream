# 受入手順と判定基準

- 対象: `ts-stream`（RTL-SDR / ISDB-T ワンセグ / TS HTTP ストリーマ）
- 基準日: 2026-09-30
- 関連: `plans/実装計画.md` §9・§10、`docs/implementation-status.md`

## 0. 原則

- **未実施・未測定を「合格」と書かない。** 実行した日付・コミット・マシン・
  実行コマンド・生出力を記録する。
- **合成（synthetic）受入と実機（hardware）受入を分離する。** 合成受入の成功は
  実機受入の代替にならない。cross build / cross check は実行時検証の代替にも
  ならない。
- **禁止:** Python / GNU Radio / gr-isdb を build 時・実行時・テスト時のいずれの
  依存・ツールとしても用いない。受入は Rust の `cargo`・`curl`・シェルのみで行う。
- 本番経路はダミー TS を出さない。`tests/` のモックは本番バイナリに含まれない。
- **全テスト成功を前提にしない。** 2026-09-30 時点で raw IQ replay の 1 本が失敗
  しています（[`docs/implementation-status.md`](implementation-status.md) §1）。

## 1. 使用ツール

- `cargo` / `rustc`（要件 Rust 1.85 以上。実測環境は 1.98.1、1.85 での実行は未検証）
- `curl`
- POSIX shell
- （実機受入のみ）`librtlsdr`、USB 権限、RTL-SDR、UHF アンテナ

Python・GNU Radio・gr-isdb は不要であり、使用しない。

## 2. A. 合成受入（ハードウェア不要・決定的）

`tests/` のモック、単体テスト、および自作送信機の IQ replay を使う。実デバイスには
一切触れない。

```sh
cargo fmt --check
cargo check --all-targets
cargo test
cargo clippy --all-targets -- -D warnings
cargo check --target aarch64-unknown-linux-gnu --lib   # コンパイル確認のみ

# IQ→TS 復調の主検証（自作送信機の IQ を replay。Python は使わない）
cargo test --release --lib demod::replay_tests -- --test-threads=1
```

期待（2026-09-30 時点の実測）:

- `cargo test --release --lib demod::replay_tests -- --test-threads=1`:
  **7 テスト中 6 成功・1 失敗**。失敗は
  `raw_iq_replay_decodes_exact_contiguous_ts`（TMCC 4、TS 179 に対し
  `tmcc_frames >= 5` assertion）。fixture / 取得条件の修正が進行中。
  - 成功側の観測: carrier レベル 6 フレームで TMCC 5 / TS 227、carrier レベル
    I=1 8 フレームで TMCC 7 / TS 314、chunked（4097 / 8191 / 3 byte）が単一 push
    と一致、gap 10 フレームで `resets_gap=1` の後に再取得。
- `cargo fmt --check` / `cargo check --all-targets` / `cargo test` /
  `cargo clippy --all-targets -- -D warnings` / `cargo build --release`:
  **修正完了後に再実行**する。過去の成功を現在の成功として流用しない。
- `cargo check --target aarch64-unknown-linux-gnu --lib`: 以前は成功を観測。
  ただし**コンパイル確認のみ**で実行時検証ではない。
- RTL-SDR は列挙されず、実機受信は未実施。ARM 実機上での実行も未実施。

この受入が検証する範囲:

- HTTP/1.1 の `/version`（全状態で 200 JSON、枠を消費しない）と `/stream`
  （`video/mp2t`、chunked、400/409/503/504）の契約。
- 単一ストリーム枠の原子的予約と排他、`tuning`/`locking`/`streaming`/
  `reacquiring`/`stopping` の追加要求 409。
- 実デバイス無信号相当（空 receiver が `NoSignal` のみを返す）で、既定 10 秒の
  初期ロックデッドライン後に `504 Gateway Timeout` になり、cleanup されること。
- 切断・キャンセル・worker panic・queue 溢れ・シャットダウンでの cleanup と、
  receiver join / デストラクタ完了後のみの予約解放。
- IQ→TS 復調: 自作送信機が生成した u8 IQ を `Decoder::push_iq` に流し、送信 TS の
  **連続した完全一致区間**が復元されること（CRC・continuity を含む）。carrier
  レベルと raw IQ の両方で検証する。**これは独立録画 RF ではない。**
- DSP / FEC プリミティブの数値一致（scalar vs SIMD、chunked vs 連続、
  パンクチャ往復、RS 誤り訂正、CRC 既知値、TS 解析）。

**検証しない範囲（重要）:**

- **独立した録画 RF 信号 / 既知受信ベクトルは存在せず、検証していない。** 合成
  replay の成功は実伝搬・実機の証明ではない。
- **raw DQPSK replay は未検証**（fixture は QPSK 同期変調のみ）。DQPSK のコードは
  統合済みだが、raw 経路での確認は残る。
- 実機の実時間性能 / CPU / RSS / 30 分連続は未測定。
- 実機経路での `200 video/mp2t` は未検証（モックと合成 replay のみ）。

判定: 上記が現状の合成受入。**replay の 1 本が失敗しているため、現時点で合成受入
は合格ではありません。** 実機なしではこれ以上を主張しない。

## 3. B. 実機受入（RTL-SDR 必須・現状は目標未達）

前提:

- N100（x86_64 / AVX2）または OrangePi PC2（aarch64 / NEON）。
- `librtlsdr` 導入済み、USB 権限（udev / `plugdev` 等）。
- UHF アンテナと、対象 ch のワンセグを受信できる電波環境。

手順:

```sh
# 1. デバイス列挙（stdout に 1 デバイス 1 行、診断は stderr）
ts-stream list-devices

# 2. サーバ起動（別端末で）
ts-stream server --host 127.0.0.1 --port 3000 --device 0

# 3. バージョン（受信状態・枠に依存しない）
curl -sS -D - http://127.0.0.1:3000/version

# 4. ストリーム（信号があれば 200 video/mp2t。無信号なら約 10 秒で 504）
curl -N "http://127.0.0.1:3000/stream?ch=19"
```

期待（現状）:

- `list-devices`: 実デバイスが 1 行以上（`index=... name=... serial=...`）。
- `/version`: `200` JSON（例 `{"name":"ts-stream","version":"0.1.0"}`）。
- `/stream`: 復調器は本番に接続済みなので、実信号でロックすれば
  `200 video/mp2t` chunked を返す。**実機での 200 は未確認である。**
  - 信号が無い / ロックできない場合は、既定 10 秒の初期ロック期限後に
    `504 Gateway Timeout` を返す（`503 decoder not implemented` ではない）。
  - `503 Service Unavailable` は `librtlsdr` / デバイス使用不可か、非対応
    サンプルレートのときのみ。
- 本受入は**未実施**であり、現時点で合格ではありません。

判定: 合否は「エラーコードを返したか」ではなく「**正しい 188 byte TS が得られたか**」
で判定する。現時点で実機の合格記録は無い。

実機で追加確認できる項目:

- 追加 `GET /stream?ch=19`（同一 ch）と `ch=20`（別 ch）がともに `409`。
- `/version` が `tuning` / `locking` / `streaming` / `reacquiring` / `stopping` /
  `error` のどの状態でも `200`（状態機械はモックで検証済み。実機は追加確認）。
- SIGINT / SIGTERM で停止し、receiver の `stop()`・デストラクタ・スレッド join
  の完了後にプロセスが終了する。USB が stuck した場合は無期限に待つことがある
  （**SIGKILL は最終手段であり、クリーンな終了ではない**）。
- 権限不足時に `list-devices` が空または open エラーになること（正直な失敗）。
- 無信号時に約 10 秒で `504` になること。

## 4. C. 最終目標の受入（recorded RF / N100 は未達）

以下は**未実施**です。合成 replay のソフトウェアパイプラインは存在しますが、
**録画 RF・実機での合格はまだ記録されていません。** 測定条件と生値を記録せずに
「達成」と書かないこと。

- 既知ベクトル / 合法な録画 IQ（自作録画、配布権利確認済み）から正しい
  188 byte TS を再現する（CRC・continuity を含む）。replay はテスト内部用途とし、
  公開 CLI にはしない。
- 実デバイス経路で `200 video/mp2t` の chunked TS を取得する。
- IQ queue 満杯時の drop を検出して再同期し、無言で隠さないこと。
- 30 分連続稼働で USB 取りこぼし・メモリ増加・ハンドル漏れがないこと
  （技術側の初期検証基準。ユーザー承認済みとは扱わない）。
- AVX2 / NEON と scalar の結果一致（許容誤差含む）。
- CPU / RSS / 遅延を測定し、性能予算を確定すること。
- N100 と OrangePi PC2 の**両実機**で実時間処理を実測すること。

## 5. 記録フォーマット（必須）

各実行について次を残す:

```text
日付:
コミット:
マシン / OS / CPU:
デバイス（実機の場合）:
実行コマンド:
生出力（要約せず貼る）:
判定: 合格 / 不合格 / 未実施
```

- 未実施は「未実施」と明記する。過去の実行結果を最新として流用しない。
- 「通るはず」「計画では」は受入記録に書かない。
