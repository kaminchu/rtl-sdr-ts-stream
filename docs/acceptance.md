# 受入手順と判定基準

- 対象: `ts-stream`（RTL-SDR / ISDB-T ワンセグ / TS HTTP ストリーマ）
- 基準日: 2026-10-01
- 関連: `plans/実装計画.md` §9・§10、`docs/implementation-status.md`、
  [`stability.md`](stability.md)、[`hardware-n100.md`](hardware-n100.md)

## 0. 原則

- **未実施・未測定を「合格」と書かない。** 実行した日付・コミット・マシン・
  実行コマンド・生出力を記録する。
- **合成（synthetic）受入と実機（hardware）受入を分離する。** 合成受入の成功は
  実機受入の代替にならない。cross build / cross check は実行時検証の代替にも
  ならない。
- **禁止:** Python / GNU Radio / gr-isdb を build 時・実行時・テスト時のいずれの
  依存・ツールとしても用いない。受入は Rust の `cargo`・`curl`・シェルのみで行う。
- 本番経路はダミー TS を出さない。`tests/` のモックは本番バイナリに含まれない。
- **全テスト成功を前提にしない。** 2026-10-01 の release full suite は
  **241 passed / 0 failed / 1 ignored**、明示実行した長時間 ignore テストは
  **1 passed** です（[`stability.md`](stability.md) §6）。ただし Debug profile の
  full suite は再実行しておらず、独立録画 RF・実機受入の合格を意味しません。

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

期待（2026-10-01 時点の実測）:

- `cargo test --release --lib demod::replay_tests -- --test-threads=1`:
  **10 テスト中 9 成功・1 ignore**。合成 replay の SFO 追随と長時間 ignore の内容は
  [`stability.md`](stability.md) §3–§4 を参照。旧版の失敗
  （`raw_iq_replay_decodes_exact_contiguous_ts`、TMCC 4 / TS 179）は履歴である。
- `cargo fmt --check` / `cargo check --all-targets` /
  `cargo clippy --all-targets -- -D warnings` / `cargo test --release`:
  **241 passed / 0 failed / 1 ignored**、明示実行した長時間 ignore テストは
  **1 passed**（[`stability.md`](stability.md) §6）。
- Debug profile の full suite は再実行していない。**Debug での全成功は主張しない。**
- `cargo check --target aarch64-unknown-linux-gnu --lib`: 成功を観測。ただし
  **コンパイル確認のみ**で実行時検証ではない。ARM 実機上での実行は未実施。
- 実機（N100）では USB 受信・HTTP の再試験を実施。2026-10-01 10:04 JST の live
  2 回（25 s / 15 s、ch19 / auto gain / ppm 0）は CP 同期して `200 video/mp2t` を
  返し、TS を出力（attempt 1: 6426 TS / 1208088 byte、attempt 2: 3742 TS /
  703496 byte）。以前の live `504` と録画 IQ 3 秒（7200000 byte）の
  **608 TS / 114304 byte** は履歴として保持
  （[`hardware-n100.md`](hardware-n100.md) §2–§4・§10、[`stability.md`](stability.md) §5）。

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

- **独立した録画 RF 信号 / 既知受信ベクトルはリポジトリの fixture としては存在せず、
  検証していない。** N100 の 3 秒録画 IQ はリポジトリ外で **608 TS / 114304 byte**
  まで decode したが、独立録画 RF による CRC・continuity の独立検証は未完である。
  合成 replay の成功は実伝搬・実機の証明ではない。
- **raw DQPSK replay は未検証**（fixture は QPSK 同期変調のみ）。DQPSK のコードは
  統合済みだが、raw 経路での確認は残る。
- 実機の実時間性能 / CPU / RSS / 30 分連続は未測定。
- 実機経路での `200 video/mp2t` は 2026-10-01 10:04 JST の live 2 回（25 s / 15 s）
  で観測した。以前の再試験では live `504` も観測しており
  （[`hardware-n100.md`](hardware-n100.md) §2・§4）、30 分連続など安定した連続
  配信は未検証（主な検証はモックと合成 replay）。

判定: 上記が現状の合成受入。release full suite は
**241 passed / 0 failed / 1 ignored**、明示実行した長時間 ignore テストは
**1 passed** であり、**合成受入は合格**です。ただし独立録画 RF・実機受入の代替には
なりません。

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
  `200 video/mp2t` chunked を返す。**2026-10-01 10:04 JST の live 2 回
  （25 s / 15 s、ch19 / auto gain / ppm 0）は CP 同期して `200` と TS を返した
  （attempt 1: 6426 TS / 1208088 byte、attempt 2: 3742 TS / 703496 byte）。**
  以前の再試験では CP 同期できず `504` も観測しており、この `504` は履歴として
  保持する。
  - 信号が無い / ロックできない場合は、既定 10 秒の初期ロック期限後に
    `504 Gateway Timeout` を返す（`503 decoder not implemented` ではない）。
  - `503 Service Unavailable` は `librtlsdr` / デバイス使用不可、非対応
    サンプルレート / リサンプル設定、または開始時の内部エラーのときのみ。
    非対応サンプルレート / リサンプル設定のエラー文字列は後方互換で
    `decoder not implemented:` を含む。
- 本受入は**未合格**である。N100 で live 2 回（25 s / 15 s）の `200` を観測したが、
  30 分連続・長時間安定受信は未達。ARM 実機（OrangePi PC2）は未実施
  （[`hardware-n100.md`](hardware-n100.md) §4・§10、[`stability.md`](stability.md) §5）。

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

以下は**未達**です。合成 replay のソフトウェアパイプラインは存在し、N100 の 3 秒
録画 IQ から **608 TS / 114304 byte** を一度 decode し、実機 HTTP `200` も
2026-10-01 10:04 JST の live 2 回（25 s / 15 s）で観測しましたが、**安定した
録画 RF・実機での合格はまだ記録されていません。**
測定条件と生値を記録せずに「達成」と書かないこと。

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
