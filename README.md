# ts-stream

RTL-SDR で受信した日本の ISDB-T ワンセグ（部分受信）を TS として HTTP で
stream 配信することを目標とする Rust 製 CLI / サーバです。

- コマンド名: `ts-stream`
- 対応 OS / アーキテクチャ: Linux のみ。x86_64 / aarch64 を対象（32bit・Windows・macOS は対象外）
- 対象方式: 日本の UHF 物理チャンネル・国内ワンセグのみ（フルセグは対象外）
- ライセンス: MIT（[`LICENSE`](LICENSE) を参照。依存と librtlsdr の扱いは
  [`docs/license.md`](docs/license.md)）

> **重要 — 実装の現状**
>
> ユーザー目標（raw IQ → ワンセグ復調 → TS の HTTP 配信）は、**本番経路として
> 実装されています**。`src/demod/receiver.rs` の `Decoder` が `src/session.rs` の
> RTL-SDR バックエンド（`rtl_backend::RtlReceiver`）に接続され、実信号でロック
> すれば `GET /stream` は `200 video/mp2t` chunked を返します。旧版の README が
> 書いていた「復調器未実装のため常に 503 `decoder not implemented`」は**誤りに
> なりました**。ダミー TS を流す偽装はしません。`503` は現在、`librtlsdr` /
> デバイス使用不可、非対応サンプルレート / リサンプル設定、または開始時の内部
> エラーで返ります。非対応サンプルレート / リサンプル設定の応答文字列は、後方
> 互換で今も `decoder not implemented: ...` です。
>
> 実装方針は、ARIB STD-B31 v2.2-E1 の**一次資料を読んだ original 実装**です。
> 隣接 PoC（GNU Radio / `gr-isdbt` / Python 等）のソースは参照・コピー・流用して
> いません（[`docs/standards.md`](docs/standards.md) §2）。一次資料 PDF は取得済み
> で、`copy:yes`（複製許可）のため `pdftotext` による本文抽出に成功し、§3.3–3.15
> の数値パラメータを規格値として転記済みです。**PDF は再配布せず**、出典・節/表・
> 印刷ページ・sha256（`757e156f…36b3`）のみを引用します（同 §1、§6）。
>
> 本番に接続済みの構成:
>
> - 受信フロントエンド（`src/rtl.rs`）: `librtlsdr` の `dlopen`・デバイス列挙・
>   open/チューニング・非同期 IQ 取得（有界 queue、gap 検出、cancel/join、
>   close-after-join）。
> - 復調パイプライン（`src/demod/`）: u8 IQ 正規化 + 緩い DC 除去 → 有理リサンプル
>   （`64/63` MHz へ）→ 同期（Mode / GI / タイミング / CFO）→ TMCC フレーム取得 →
>   同期等化 or DQPSK 差動復調 + 周波数デインタリーブ → 時間デインタリーブ →
>   軟判定 → ビット/バイトデインタリーブ・Viterbi・RS(204,188) → 188 byte TS。
> - 単一ストリーム枠の排他と状態機械、HTTP/1.1 の `/version` と `/stream` 契約、
>   テスト限定のモック。予約の解放（receiver スレッド join 後）・有界 queue・
>   切断時のセマンティクスは**変更していません**。
>
> **合成 replay の fixture は本リポジトリ内製です。** 送信機を本リポジトリ内に
> 自作し（TS → RS → エネルギー拡散 → インタリーブ → K=7 / パンクチャ → マッピング
> → OFDM → u8 IQ）、その IQ を decoder に replay しています
> （`src/demod/replay_tests.rs`）。**独立した録画 RF fixture はリポジトリに含めて
> いません。**
>
> 2026-10-01 の release 品質ゲート実測（`cargo test --release`）:
> **241 passed / 0 failed / 1 ignored**。明示実行した長時間 ignore テストは
> **1 passed**。`cargo fmt --check` / `cargo check --all-targets` /
> `cargo clippy --all-targets -- -D warnings` / aarch64 lib check も成功。
> 現在の安定性検証の詳細は [`docs/stability.md`](docs/stability.md)。
>
> N100 実機の実 IQ / 録画 IQ replay の記録は
> [`docs/hardware-n100.md`](docs/hardware-n100.md) に分離しています（2026-09-30
> 当時の部分・受入未完了記録）。現在の実機観測は
> [`docs/stability.md`](docs/stability.md) §5 を参照。
>
> 既知の未対応・未検証:
>
> - **N100 実機受入は部分**。2026-09-30 の部分記録に続き、**2026-10-01 10:04 JST
>   の live 2 回（25 s / 15 s、ch19 / auto gain / ppm 0）は CP 同期して
>   `200 video/mp2t` chunked を返し、TS を出力**しました（attempt 1: 6426 TS /
>   body 1208088 byte、attempt 2: 3742 TS / 703496 byte）。**30 分連続・
>   OrangePi PC2・エンドツーエンド CPU / RSS 予算は未実施**で、slow batch の
>   headroom リスクは残ります。最新は
>   [`docs/hardware-n100.md`](docs/hardware-n100.md) §10 と
>   [`docs/stability.md`](docs/stability.md) §5.1。
> - **DQPSK はコードとしては統合済み**（`carriers::differential_symbol` と
>   `Decoder` の差動経路）ですが、**raw DQPSK の replay は未検証**です（replay
>   fixture は QPSK 同期変調のみ）。**DQPSK の SFO 補正は未検証**で、**DQPSK の
>   整数 CFO 自動補正もありません**（既知の ppm は呼び出し側が指定する想定）。
> - **TMCC パリティは誤り訂正ではなく検査のみ**です。パリティ不一致のフレームは
>   破棄し、取得からやり直します（reject / re-acquire）。
> - **整数 CFO はコヒーレントセグメントで実装済み**です。粗探索を同期器に統合し、
>   同一の整数推定が連続確認されたときだけ補正します（非ゼロ補正はロック世代を
>   進める）。DQPSK は SP が無いため自動補正しません。
> - **SFO 累積 timing de-ramp は維持**しつつ、コヒーレント SP 等化の前に巻き戻し、
>   シンボル履歴を 1 フレーム分保持して厳密に対応付けます。DQPSK は de-ramp を
>   そのまま使うため SFO 下の挙動は未検証です。
> - Viterbi は **ACS ごとに再正規化**し、メトリックは `f64`。ストリーミングの
>   NaN / ±inf は消失として扱い、ブロック API は `NonFinite` で拒否します。
> - SIMD は IQ 正規化にのみ存在し、**性能測定は未実施**。
>
> テスト用モックは `tests/` 配下にのみ存在し、本番バイナリには含まれません。
> 本番経路で無信号時にダミー TS を返すことはありません。
>
> **実機での受入は未実施・未確認です。** 本記録環境では RTL-SDR は列挙されて
> いません。`aarch64-unknown-linux-gnu` ターゲットは導入済みで
> `cargo check --target aarch64-unknown-linux-gnu --lib` は成功していますが、
> これは**コンパイル確認のみ**であり、ARM 実機実行の証明ではありません。
> 実 RF / 録画 IQ / 既知ベクトルによる検証は未実施です。
> 詳細は [`docs/implementation-status.md`](docs/implementation-status.md)。

---

## 1. ビルド

前提: Rust 1.85 以上（`Cargo.toml` の `rust-version`。依存クレートが要求する
MSRV に合わせています）。実測は Rust 1.98.1 で行っており、1.85 でのテストを
主張するものではありません。`librtlsdr` は
ビルド時にはリンクせず、実行時に `dlopen` するため、ビルドに C コンパイラや
`librtlsdr-dev` は不要です。

```sh
cargo build --release
# 生成物: target/release/ts-stream
```

品質ゲート:

```sh
cargo fmt --check
cargo check --all-targets
cargo test
cargo clippy --all-targets -- -D warnings
```

> 日付つきの実測スナップショットを
> [`docs/implementation-status.md`](docs/implementation-status.md) と
> [`docs/stability.md`](docs/stability.md) に記録しています。2026-10-01 の release
> 実測は `cargo test --release` **241 passed / 0 failed / 1 ignored**、明示実行の
> 長時間 ignore テスト **1 passed**、`fmt` / `check` / `clippy` / aarch64 lib
> check 成功です。本 README は「通るはず」ではなく「実際に走らせた結果」を
> 実装状況ドキュメント側に記録する方針です。

## 2. 実行 (CLI)

```sh
# 接続されている RTL-SDR を列挙
ts-stream list-devices

# 単一クライアント向け HTTP TS サーバ
ts-stream server --host 0.0.0.0 --port 3000 --device 0
```

### サブコマンドとオプション

`server` の既定値（`src/cli.rs`）:

| オプション | 既定値 | 説明 |
| --- | --- | --- |
| `--host` | `127.0.0.1` | bind するアドレス。既定はループバック。 |
| `--port` | `3000` | TCP ポート。 |
| `--device` | `0` | RTL-SDR のデバイス index。 |
| `--gain` | `auto` | `auto` または手動ゲイン (dB)。チューナが対応値を返す場合は最近傍に丸める。 |
| `--ppm` | `0` | 周波数補正 (ppm)。 |
| `--sample-rate` | `1200000` | IQ サンプルレート (Hz)。 |

`list-devices` は stdout に 1 デバイス 1 行で
`index=<n> name=<name> serial=<serial|unavailable>` を出力します。診断は
stderr に出ます。

## 3. HTTP API

サーバは HTTP/1.1 で待ち受けます。

| メソッド / パス | 説明 |
| --- | --- |
| `GET /version` | 常に `200` JSON。例 `{"name":"ts-stream","version":"0.1.0"}`。ビルド時に `TS_STREAM_GIT` が設定されていれば `git` フィールドを含む。受信枠を取得・消費せず、USB / DSP の状態に依存しない。 |
| `GET /stream?ch=NN` | チューニングと初期ロックの後、`Content-Type: video/mp2t` の chunked body を返す。 |

`/stream` の状態コード:

| コード | 条件 |
| --- | --- |
| `400 Bad Request` | `ch` が無い / 不正 / 未知のクエリ / 範囲外（現状は `13..=62` を受理） |
| `409 Conflict` | 単一ストリーム枠が使用中（`tuning` / `locking` / `streaming` / `reacquiring` / `stopping`）。同一 ch・別 ch を問わない |
| `503 Service Unavailable` | `librtlsdr` 不在 / デバイス使用不可、非対応サンプルレート / リサンプル設定、または開始時の内部エラー。**復調器は本番に接続済みで、常時 503 ではありません**（非対応サンプルレート / リサンプル設定のエラー文字列は今も `decoder not implemented:`） |
| `504 Gateway Timeout` | 初期ロックが既定 10 秒でタイムアウト（実デバイスで信号がない場合を含む） |

> `GET /stream` の `200 video/mp2t` chunked 応答は、テスト限定モックと、合成
> replay でデコードした TS で確認されています。**N100 実機でも 2026-10-01 10:04
> JST の live 2 回（25 s / 15 s）で `200` と TS を観測しました**が、**30 分連続・
> 実機受入は未達**です（[`docs/hardware-n100.md`](docs/hardware-n100.md) §10、
> [`docs/stability.md`](docs/stability.md) §5.1）。実デバイスで復調器が
> ロックできない場合は `503` ではなく **既定 10 秒で `504 Gateway Timeout`** に
> なります。この 504 経路はモック（空 receiver =
> `NoSignal` のみ）で検証されています。ダミー TS を流す偽装はしません。
>
> 初期ロックの既定 10 秒は `Backend::open` が戻った後から計測します。デバイスの
> open / チューニング / 設定は同期・**非プリエンプティブ**で、停止要求が来ても
> 呼び出しが戻るまでキャンセルできません。安全のためタイムアウトによる強制
> close は行いません。

使用例:

```sh
curl http://127.0.0.1:3000/version
curl -N "http://127.0.0.1:3000/stream?ch=19"
```

> 上記の `ch` 範囲について: 物理チャンネルの中心周波数式と、現行の国内割当が
> `13..52` である点の相違は [`docs/standards.md`](docs/standards.md) に整理して
> います。現状のコードは歴史的な `13..=62` を受理します（未解決の残課題）。

## 4. セキュリティ（既定の安全側設定）

- **認証はありません。** 既定 bind は `127.0.0.1` で、ローカルからのみ到達可能です。
- `--host 0.0.0.0` 等を指定すると**認証なしで LAN / 公開網へ露出**します。信頼できる
  ネットワーク以外へ公開しないでください。TLS / アクセス制御は実装されていません。
- `/version` は受信枠や USB / DSP の状態に依存せず応答しますが、`/stream` は
  単一枠排他です。

## 5. 環境変数

| 変数 | 用途 |
| --- | --- |
| `TS_STREAM_VERSION` | **ビルド時**に参照。設定されていれば `ts-stream --version` と `/version` の `version` に使われる（`option_env!`）。未設定なら `Cargo.toml` の `version`。 |
| `TS_STREAM_GIT` | **ビルド時**に参照。設定されていれば `/version` の `git` フィールドに埋め込まれる（`option_env!`）。未設定なら `git` は省略。 |

実行時に参照する環境変数は現時点でありません。

## 6. システムライブラリとデバイス権限

- **`librtlsdr` は実行時依存**です。`libloading` により `librtlsdr.so.2` →
  `librtlsdr.so.0` → `librtlsdr.so` の順に `dlopen` します。いずれも無い場合、
  サーバは起動を継続し、`/version` に応答しつつ `/stream` は `503` を返します。
- **USB デバイスへのアクセス権限が必要**です。通常は udev ルールで `plugdev`
  グループ等に許可を与えるか、root で実行します。権限が無い場合、列挙や open が
  失敗します（`list-devices` が空、または `open` エラー）。
- ビルド時に `librtlsdr` のヘッダや pkg-config は不要です。本クレートは
  `librtlsdr` を通常のリンクでは組み込みません（`docs/license.md` の GPL に関する
  注意も参照）。

## 7. テストとモック

- 単体テスト・結合テストは `tests/` と各モジュール内 `#[cfg(test)]` にあります。
- 決定的な `Backend` モックは `tests/common/mod.rs` のみにあり、本番経路へは
  コンパイルされません。
- モックは chunked framing、排他 (`409`)、`/version` の状態非依存、キャンセル /
  join 後の枠解放、queue 溢れによる接続終了、実デバイス無信号相当（空 receiver が
  `NoSignal` のみを返す）での既定 10 秒 `504` などを検証します。
- **合成 replay（送信機自作）:** `src/demod/replay_tests.rs` は、このリポジトリ内で
  送信側を自作し（TS → RS → エネルギー拡散 → インタリーブ → K=7 / パンクチャ →
  マッピング → OFDM → u8 IQ）、その IQ を `Decoder::push_iq` に流して TS を復元
  します。carrier レベルと raw IQ の両方を検証します。**独立した録画 RF 信号の
  fixture はリポジトリに含めていません**（N100 実機の録画 IQ replay は
  [`docs/hardware-n100.md`](docs/hardware-n100.md) の別記録）。実行する受入コマンドは
  `cargo test --release --lib demod::replay_tests -- --test-threads=1` です
  （[`docs/acceptance.md`](docs/acceptance.md) §2）。
- テストの性格は [`docs/implementation-status.md`](docs/implementation-status.md)
  §3 のとおり区別します。
  - **規格値アンカー（spec-anchored）:** Mode の FFT 長 2048/4096/8192、総キャリア数
    1405/2809/5617、GI 比 1/4〜1/32、RS/GF の原始多項式、K=7 生成多項式 171/133 oct、
    パンクチャ率の系列など。一次資料の表と突き合わせた値です。
  - **往復・自己整合（roundtrip）:** IQ 正規化の scalar vs SIMD、リサンプラの
    チャンク連続一致、パンクチャ往復 + Viterbi 往復、RS の自己生成語の訂正、
    CP 相関の合成信号、FFT と DFT の一致。実装同士の整合を見るもので、規格適合の
    独立証明ではありません。
  - **送信機生成 IQ（transmitter-generated）:** 自作送信機の IQ を replay する統合
    検証。復調器の各段が揃って初めて通りますが、**独立録画 RF ではない**ため
    実伝搬・実機の証明にはなりません。
  - **実機検証（real-RF）:** N100 実機の録画 IQ を取得し replay で TS まで到達
    （部分。[`docs/hardware-n100.md`](docs/hardware-n100.md)）。ただし**リポジトリ
    fixture としては未整備**で、実機の独立受入・性能測定は未実施です。
- `src/demod/` は `src/lib.rs` から公開され、本番バックエンドに接続されています。
  2026-10-01 の release 品質ゲートは
  `cargo test --release` **241 passed / 0 failed / 1 ignored**、明示実行の長時間
  ignore テスト **1 passed** です（[`docs/stability.md`](docs/stability.md)）。

## 8. ドキュメント

- [`docs/implementation-status.md`](docs/implementation-status.md) — Phase 0〜7 の
  現状、実装済みプリミティブ、未実装の残作業、日付つきの品質ゲート実測
  （主張ではなく観測）。
- [`docs/standards.md`](docs/standards.md) — ISDB-T / ARIB / ITU の一次資料、
  UHF 周波数対応、ARIB STD-B31 v2.2-E1 §3.3–3.15 の数値パラメータ（キャリア/
  SP/CP/TMCC/パリティ/各種インターリーブ）、送信順序の逆順として導出した受信
  チェーン、印刷ページ ↔ PDF ページ対応（p+10）。PDF は再配布せず出典のみ引用。
- [`docs/license.md`](docs/license.md) — 依存ライセンスの確認メモと librtlsdr の
  配布上の注意（広範な法的保証はしない）。
- [`docs/acceptance.md`](docs/acceptance.md) — 合成受入と実機受入を分離した
  受入コマンド一覧と判定基準（Python / GNU Radio / gr-isdb を使用しない）。
- [`docs/hardware-n100.md`](docs/hardware-n100.md) — N100 実機の受信記録。
  2026-09-30 の部分・未完了記録（環境、録画 IQ / TS の構造検証、live `504` の
  切り分け未了）を履歴として保持しつつ、§10 に 2026-10-01 10:04 JST の live 成功
  （SHORT）を追記。
- [`docs/stability.md`](docs/stability.md) — **現在**の安定性検証。Viterbi / SFO /
  整数 CFO の修正、release 品質ゲート **241/0/1**、長時間 ignore 1 passed、
  2026-10-01 10:04 JST の live 2 回（25 s / 15 s、CP 同期・`200`）と、以前の
  CP false 観測を履歴として分離して記録（§5）。
- 要件と実装計画: [`plans/SPEC.md`](plans/SPEC.md)、
  [`plans/実装計画.md`](plans/実装計画.md)（計画を正とする。本書はその実装状況を
  記録するもの）。

`docs/standards.md` と `docs/license.md` は Phase 0 の一次調査記録であり、
`[本文確認]` / `[導出]` / `[検索]` / `[未検証]` / `[未実装]` の識別を保持します
（[`docs/implementation-status.md`](docs/implementation-status.md) §7 の更新方針を
参照）。ARIB の PDF 原本はリポジトリに含めず、数値は節/表・印刷ページを付して
引用します。

## 9. 既知の制約（要約）

- ISDB-T の**復調・FEC・TS 出力パイプラインは本番に接続済み**（`src/demod/` を
  `src/session.rs` の `rtl_backend` が駆動）。ユーザー目標はソフトウェア経路として
  存在する。
- 検証の中心は**合成 replay**。**独立した録画 RF fixture はリポジトリに含めていない**
  （N100 実機の録画 IQ replay は [`docs/hardware-n100.md`](docs/hardware-n100.md)）。
- release 品質ゲートは **241 passed / 0 failed / 1 ignored**（2026-10-01 実測）。
  長時間 ignore テストを明示実行すると **1 passed**。
- **DQPSK はコード統合済みだが、raw DQPSK replay は未検証**（fixture は QPSK）。
  **DQPSK の**整数 CFO 自動補正と SFO 補正も未対応・未検証。
- **TMCC パリティは検査のみで誤り訂正なし**。不一致フレームは破棄して再取得する。
- **整数 CFO はコヒーレントセグメントで実装済み**（粗探索の連続確認後に補正）。
- **SFO 累積 de-ramp は TMCC / DQPSK 用に維持**し、コヒーレント SP 等化の前に
  巻き戻す。シンボル履歴は 1 フレーム分を厳密に検証する。
- 本番 `/stream` はロック時に `200 video/mp2t` を返す。`503` は `librtlsdr` /
  デバイス不可、非対応サンプルレート / リサンプル設定、または開始時の内部エラーの
  ときのみ。無信号は既定 10 秒で `504`。ダミー TS は流さない。
- SIMD 最適化（AVX2 / NEON）は IQ 正規化のみ。**性能測定は未実施**。
- N100 実機受入は**部分**。2026-10-01 10:04 JST の live 2 回（25 s / 15 s）は
  CP 同期して `200 video/mp2t` を返し TS を出力（[`docs/hardware-n100.md`](docs/hardware-n100.md)
  §10、[`docs/stability.md`](docs/stability.md) §5.1）。ただし 30 分連続・
  **OrangePi PC2**・エンドツーエンド CPU / RSS 予算は**未実施**。
  aarch64 は `cargo check --target aarch64-unknown-linux-gnu --lib` の成功のみで、
  実機実行の証明ではない。
- サブコマンドや HTTP 契約の一部はモックでのみ検証済み。
- 停止は receiver の `stop()`・デストラクタ・スレッド join の完了まで待つ。
  デバイスの open / 設定は同期・非プリエンプティブで、停止要求でも呼び出しが
  戻るまでキャンセルできない。USB が stuck した場合は cleanup が無期限になり得る
  （SIGKILL は最終手段で、クリーンな終了ではない）。タイムアウトによる強制 close
  は安全のため行わない。
- 物理チャンネル範囲が歴史的な `13..=62` のまま（現行 `13..52` との整合は未解決）。
  この `13..=62` 受理は**後方互換のため維持**し、本ドキュメント作業では変更しない。
  歴史的 `13..62` と現行運用 `13..52` の相違は
  [`docs/standards.md`](docs/standards.md) §3.2 に整理。

## 10. リリースとバイナリ配布

`v0.1.0` のようなタグを push すると、GitHub Actions
（[`.github/workflows/release.yml`](.github/workflows/release.yml)）が Linux
x86_64 / aarch64 のネイティブバイナリをビルドし、同名の GitHub Release へ
アップロードします。手動 trigger はありません（タグがそのままリリースです）。

```sh
git tag v0.1.0
git push origin v0.1.0
```

リリース成果物:

| ファイル | アーキテクチャ | ビルド環境 |
| --- | --- | --- |
| `ts-stream-x86_64-unknown-linux-gnu.tar.gz` | x86_64 / amd64 | Ubuntu 24.04 (glibc 2.39) |
| `ts-stream-aarch64-unknown-linux-gnu.tar.gz` | aarch64 / arm64 | Ubuntu 24.04 arm64 (glibc 2.39) |
| `SHA256SUMS` | — | 上記 2 アーカイブの SHA-256 |

各 tar.gz には実行ファイル `ts-stream` がアーカイブ直下（root）に 1 つだけ入り
ます。成果物名はタグではなく Rust target で固定しているため、任意のタグ名を
安全に使えます。ビルドは GNU (glibc) ターゲットで、musl 静的リンクでは
**ありません**。したがって glibc 2.39 以上（Ubuntu 24.04 以降相当）が必要です。

### mise での導入

[mise](https://mise.jdx.dev/) の GitHub backend で導入できます。

```sh
# 最新リリース
mise use -g github:kaminchu/rtl-sdr-ts-stream

# バージョン固定（`@0.1.0` はタグ `v0.1.0` に対応）
mise use -g github:kaminchu/rtl-sdr-ts-stream@0.1.0
```

### バージョンの扱い

- リリースビルドでは、ビルド時に環境変数 `TS_STREAM_VERSION` へ push された
  タグ名が設定され、`ts-stream --version` と `GET /version` はそのタグ名を返し
  ます（例: タグ `v0.1.0` なら `v0.1.0`）。
- タグを経由しない通常の `cargo build` では `Cargo.toml` の `version`
  （`0.1.0`）にフォールバックします。CLI と HTTP は `ts_stream::VERSION` という
  単一の定数を共有します。
- `TS_STREAM_GIT` には push されたタグが指すコミット SHA（`git rev-parse HEAD`。
  annotated tag でもタグオブジェクトではなくコミット）が埋め込まれ、`/version`
  の `git` フィールドに現れます。
- ハイフンを含むタグのうち、セマンティックバージョン風の
  `v?X.Y.Z-...`（例 `v0.1.0-rc.1`、`1.2.3-beta`）だけを GitHub の prerelease
  として公開します。それ以外のダッシュ入りタグは通常リリースとして公開します。

### 実行時の依存

- アーカイブに `librtlsdr` は**同梱していません**。実行時に `libloading` で
  `librtlsdr.so.2` → `.so.0` → `.so` の順に `dlopen` します（§6）。
- RTL-SDR の USB デバイスへアクセスする権限が別途必要です。udev ルールで
  `plugdev` 等に許可を与えるか root で実行してください（§6）。
- `librtlsdr` が無くてもサーバは起動し、`/version` に応答します。`/stream`
  は `503` を返します。

「未測定・未実装の項目を対応済みと宣言しない」を本プロジェクトの文書方針とします。
