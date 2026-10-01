# 実装状況（Phase 0–7・観測記録）

- 対象: `ts-stream`（RTL-SDR / ISDB-T ワンセグ / TS HTTP ストリーマ）
- 記録日: 2026-09-30（実装・検証担当の実行結果）
- 位置づけ: 本ドキュメントは `plans/実装計画.md` の Phase 0–7 に対する
  **観測記録**であり、計画そのものではない。計画を正とする。
- 原則: **未測定・未実装の項目を「対応済み」と宣言しない。** コードが存在する
  だけ、テストが通っただけ、cross check が通っただけでは「達成」と書かない。
  **全テスト成功の主張は、修正完了後に再実行してから行う。**

## 0. 要約（先に結論）

ユーザー目標「RTL-SDR の raw IQ を ISDB-T ワンセグとしてデコードし、TS を HTTP
chunked で配信する」は、**ソフトウェア経路として本番に接続されました**。

- `src/demod/receiver.rs::Decoder`（original 実装）が `src/session.rs` の
  `rtl_backend::RtlReceiver` から駆動され、raw `u8` IQ → 188 byte TS を生成します。
  本番の `GET /stream?ch=NN` は実信号でロックすれば `200 video/mp2t` chunked を
  返します。**旧版の「復調器未接続のため常に 503 `decoder not implemented`」は
  誤りになりました。**
- `503` は現在、`librtlsdr` / デバイス使用不可か、非対応サンプルレートのときだけ
  です。実デバイスでロックできない場合は **既定 10 秒で `504`** になります
  （モックの空 receiver = `NoSignal` のみ、で検証）。
- 実装は ARIB STD-B31 v2.2-E1 の一次資料を読んだ **original 実装**です。隣接 PoC
  （GNU Radio / `gr-isdbt` / Python 等）のソースは参照・コピー・流用していません。
- **検証は現時点ですべて合成（synthetic）です。** 送信機を本リポジトリ内に自作し、
  その IQ を replay しています。**独立した録画 RF 信号の fixture は存在しません。**
  実機性能は未測定です。
- 2026-10-01 の安定性修正と品質ゲート再実行で、release full suite は
  **241 passed / 0 failed / 1 ignored**、明示実行の長時間 ignore テストは
  **1 passed** です（§1）。旧版の raw IQ replay 失敗（TMCC 4 / TS 179）と
  8/8 の暫定観測は**履歴**であり、現在の検証は
  [`stability.md`](stability.md) に記録します。

## 1. 品質ゲート（2026-10-01 時点・全成功）

以下は release で再実行した**現在**の観測です。旧版の replay 失敗記録は履歴として
扱い、現在の成功に流用しません。詳細な内訳と実機観測は
[`stability.md`](stability.md) を参照。

| コマンド | 結果（2026-10-01） |
| --- | --- |
| `cargo fmt --check` | 成功 |
| `cargo check --all-targets` | 成功 |
| `cargo clippy --all-targets -- -D warnings` | 成功 |
| `cargo test --release` | **241 passed / 0 failed / 1 ignored**（lib 207 中 206 pass・1 ignore、統合 34 pass、doc 1 pass） |
| `cargo test --release raw_iq_sustained_sfo_plus_cfo_decodes_contiguous_ts -- --ignored --nocapture --test-threads=1` | **1 passed**（48 フレーム / ~11.1 s / +20 ppm + 3.2 bin。TMCC 46 / TS 2186 / resets 1 / ts_stall 0） |
| `cargo check --target aarch64-unknown-linux-gnu --lib` | 成功。**コンパイル確認のみ（ARM 実機実行ではない）** |

補足:

- Debug profile の full suite は再実行していない。**Debug での全成功は主張しない。**
- この品質ゲート実行にはハードウェア試験を含めていない（実機観測は
  [`stability.md`](stability.md) §5 と [`hardware-n100.md`](hardware-n100.md)）。
- 本記録の初版環境では RTL-SDR は列挙されておらず、実機受信は未実施だった。
  別途 N100 実機 run を実施（当時の部分記録は
  [`hardware-n100.md`](hardware-n100.md)、現在の観測は
  [`stability.md`](stability.md) §5）。
- OrangePi PC2 と実 IQ を用いた受入は未実施。
- 上記は `src/lib.rs` が公開する全モジュール（`dsp` / `fec` / `rtl` / `demod` /
  `http` / `session` / `cli`）を対象にビルド・テストされます。`demod` は
  `src/lib.rs` から公開され、本番バックエンドに接続済みです。

## 2. Phase 別ステータス

| Phase | 計画の完了条件（要約） | 実装・観測 | 判定 |
| --- | --- | --- | --- |
| 0 一次資料・ライセンス・規格対応表 | Phase 1 着手に必要な契約が確定。調査未了は明記 | `docs/standards.md`・`docs/license.md` 作成済み。**S1 は `pdftotext` 抽出に成功**し、§3.3–3.15 の数値パラメータを `[本文確認]` として転記。BT.1306/2016 本文との突き合わせは `[未検証]` | **部分**（主要規格値は確認済み。ITU 本文の突き合わせは残） |
| 1 crate 骨格・CLI/API 契約・モック | `fmt/check/test/clippy` 全通過、`/version` JSON、モックの chunked TS、本番にモックを出さない | crate・CLI・`/version`・`/stream` 契約・テスト限定モックを実装 | **機能条件達成**（品質ゲートは replay 修正待ちで未確定） |
| 2 N100 実機受信 / IQ | `list-devices` が実デバイスを列挙。IQ replay で後段へ供給。N100 で継続受信。対応表検証 | `src/rtl.rs` に `dlopen`・列挙・open/チューニング・非同期 IQ・有界 queue・gap 検出・cancel/join・close-after-join を実装。**実機受入は未実施**。IQ replay は合成 fixture のみ | **部分** |
| 3 scalar DSP / FEC TS PoC | 既知ベクトル / 録画 IQ から正しい TS（CRC・continuity 含む）を再現 | **IQ→TS ソフトウェアパイプラインを実装し本番に接続**。キャリア推定・等化・DQPSK・各種デインタリーブ・エネルギー拡散除去・Viterbi・RS を統合。**合成 replay で検証**。**録画 RF / 既知ベクトルは未入手・未実施**。raw replay は解消（現在 release full suite 241/0/1） | **部分**（software pipeline は存在。recorded RF / N100 は未達） |
| 4 単一 HTTP 統合 | 実デバイスからの chunked TS 配信、排他・切断・エラー、リソース解放 | 状態機械・原子的予約・409・reacquiring・有界 queue・遅読切断・`/version` 全状態応答・join 後解放・shutdown をモックで検証。**実デバイスからの TS 配信は未検証**（実機受入待ち） | **部分**（HTTP 層とバックエンド接続は実装済み） |
| 5 N100 SIMD（AVX2）と長時間受入 | N100 で実時間処理を実測。30 分連続。CPU/RSS/遅延測定 | IQ 正規化に AVX2 と scalar 一致テストのみ。**ホットスポット特定・最適化・性能測定・長時間は未実施** | **未達** |
| 6 arm64 NEON と OrangePi PC2 | N100 と OrangePi PC2 の両実機で実時間処理 | NEON 正規化コードと aarch64 `check --lib` 成功のみ。**OrangePi 実機・NEON 実行検証は未実施** | **未達** |
| 7 安定化・配布・文書 | 目標時間の連続処理で取りこぼし・メモリ増・ハンドル漏れなし | README・`LICENSE`・`docs/*` を整備。セッションの join/停止/シャットダウンを強化。**長時間安定性・配布物は未確認。USB stuck 時の無期限 cleanup が既知の残課題** | **部分** |

## 3. 実装済み

### 3.1 IQ→TS 復調パイプライン（`src/demod/`、本番接続済み）

`src/session.rs` の `rtl_backend::RtlReceiver` が `crate::rtl::Receiver` の IQ を
`Decoder::push_iq` に渡し、戻った 188 byte TS だけを転送します。段構成:

```text
u8 IQ -> 正規化 (AVX2/NEON/scalar) + 緩い DC 除去
      -> 有理リサンプル (入力 fs -> 64/63 MHz)
      -> Synchronizer            (Mode / GI / タイミング / 小数 CFO)
      -> FrameCollector          (TMCC フレーム取得 + layer A 設定)
      -> 同期等化 or DQPSK 差動 + 周波数デインタリーブ
      -> TimeDeinterleaver       (TMCC の I)
      -> 軟判定 -> bit/byte デインタリーブ -> Viterbi -> RS(204,188) -> TS
```

- `ofdm.rs`: ガード除去・CFO 逆回転・unitary DFT・キャリア demap。
- `sync.rs`: 12 候補 `(Mode, GI)` の複数シンボル正規化 CP 相関で取得し、シンボル
  ごとに `±4` サンプル探索で追従。CP 相関の小数 CFO に加え、コヒーレント
  セグメントでは粗探索（`coarse.rs`）による**整数 CFO を実装済み**です（同一推定の
  連続確認後に補正し、非ゼロ補正はロック世代を進める）。DQPSK は SP が無いため
  自動補正しません。累積 SFO de-ramp は TMCC / DQPSK 用に維持し、コヒーレント
  SP 等化の前に `restore_timing_offset` で巻き戻します。
- `frame.rs`: TMCC の差動復調と、パリティ・同期語・セグメント種別の検証。
  **パリティは検査のみで誤り訂正はしません**。不一致フレームは破棄し、取得を
  やり直します（reject / re-acquire）。
- `carriers.rs`: 同期変調の SP チャネル推定 + 線形補間 + 等化、DQPSK の
  `current * conj(previous)` 差動復調、周波数デインタリーブ。
- `control.rs`: キャリア配置、パイロット PRBS、時間/周波数インターリーブ、
  TMCC 復号（(184,102) パリティ検査を含む）。
- `coding.rs`: bit/byte デインタリーブ、デパンクチャ、K=7 軟判定 Viterbi、
  エネルギー拡散除去、RS(204,188)、TS 検証。
- `receiver.rs`: 上記をストリーミングに統合し、gap / 同期喪失 / 設定変更 /
  TMCC 途絶 / TS 停滞でリセット理由を統計に記録。`is_locked` は「実際に
  RS 検証済み TS を最近出した」ことを意味します。

### 3.2 frontend DSP（`src/dsp/mod.rs`）

- `normalize_u8_iq` / `normalize_u8_iq_into`: u8 インターリーブ IQ を `Complex32`
  へ正規化。**AVX2 / NEON を実行時ディスパッチ**し scalar fallback を持つ。
  ビット一致テストは検出した ISA に対して走ります（x86_64 では AVX2。NEON は
  aarch64 ビルドが通るだけで実行検証は未実施）。
- `DcBlocker`、`RationalResampler`（chunk 境界で連続）、`Mode` / `GuardInterval`、
  `CpCorrelator`、`Fft`。

### 3.3 FEC / TS（`src/fec/mod.rs`）

MPEG-2 CRC-32、TS 解析・検証・continuity、K=7 軟判定 Viterbi、パンクチャ、
RS(204,188)（t=8）。復調パイプラインの `coding.rs` から利用されます。

### 3.4 RTL / FFI（`src/rtl.rs`）

`RtlLibrary::load`（`.so.2` → `.so.0` → `.so`）、`devices`、`open`（周波数・
サンプルレート・ppm・ゲインの検証と丸め）、専用スレッドでの `rtlsdr_read_async`、
有界 queue と gap フラグ、cancel を繰り返して join、join 後 `rtlsdr_close`。

### 3.5 セッション / HTTP / CLI

`SessionManager` の単一枠予約と状態機械、`src/http.rs` の `/version`・`/stream`、
`src/cli.rs`、`src/main.rs`。統合テストは `tests/` のモックに対して実行されます。

## 4. 検証の性格（合成 replay）

`src/demod/replay_tests.rs` は、このリポジトリ内に**送信機を自作**し、

```text
TS -> RS(204,188) -> フレーム単位エネルギー拡散 -> バイトインターリーブ
   -> K=7 rate-1/2 -> パンクチャ -> ビットインターリーブ -> QPSK マッピング
   -> 時間インターリーブ -> 周波数インターリーブ -> SP/TMCC/AC1 挿入
   -> unitary IFFT + ガード -> 64/63 MHz 波形
   -> 逆リサンプル 189/160 で 1.2 MHz u8 I/Q へ量子化
```

を実行し、その raw `u8` IQ を `Decoder::push_iq` に流します。受信器に TS を
直接注入せず、decoder 段も fake しません。ただしこれは**自作送信機の自己整合**で
あり、**独立した録画 RF 信号ではありません**。

- raw replay を含む release full suite は **241 passed / 0 failed / 1 ignored**、
  明示実行の長時間 ignore テストは **1 passed** です（§1、
  [`stability.md`](stability.md)）。
- **DQPSK はコードとしては統合済み**（`carriers::differential_symbol` と
  `Decoder` の差動経路、`frame.rs` の差動 predecessor 追跡）ですが、**raw DQPSK
  の replay は未検証**で、**SFO 補正と整数 CFO の自動補正も未対応**です。replay
  fixture は QPSK 同期変調のみです。

## 5. 未実装 / 未検証（残作業の一覧）

| 項目 | 状態 |
| --- | --- |
| 独立した録画 RF / 既知受信ベクトルによる検証（CRC・continuity 含む） | 未入手・未実施 |
| 実機（N100 / OrangePi PC2）での受信・実時間処理 | 未実施 |
| 性能測定・30 分連続・AVX2 ホットスポット最適化 | 未測定 |
| OrangePi PC2 実機 / NEON 実行検証 | 未実施 |
| raw IQ replay の TMCC 検証数不足（旧版で 4、期待 5） | 解消（現在 release full suite 241/0/1） |
| raw DQPSK replay / DQPSK の SFO 補正 | 未検証（コードは統合済み） |
| コヒーレント整数 CFO | 実装済み（粗探索の連続確認後に補正） |
| DQPSK の整数 CFO 自動補正 | 未対応（SP が無いため。既知 ppm は呼び出し側） |
| SFO 累積 de-ramp とコヒーレント等化の整合 | 実装済み（1 フレームのシンボル履歴を厳密検証） |
| TMCC パリティの誤り訂正 | 未実装（検査のみ。不一致は破棄して再取得） |
| 実デバイス経路での `200 video/mp2t` | 未検証（モックと合成 replay のみ） |
| 実デバイス無信号時の `504` | モック（空 receiver）で検証。実機は未実施 |
| 中心周波数受理範囲 `13..62` と現行 `13..52` の整合 | 未解決（後方互換のため `13..=62` を維持） |

## 6. セッション / シャットダウンの現状（意味論は変更なし）

- 予約（stream slot）は `tuning` から `stopping` まで保持します。追加
  `GET /stream` は同一 ch・別 ch を問わず `409`。
- 予約解放は**スーパーバイザが receiver ワーカースレッドを join した後**にのみ
  行います。`stopping` は receiver の `stop()` と**デストラクタ（およびスレッド
  終了）**が完了するまで維持します。worker panic でも `Worker::Drop` → cleanup →
  join の順で解放され、release を先に偽装しません。
- `shutdown()` は `shutting_down` を恒久的に立て、以降の**新規予約を拒否**します。
  停止待ちは**無期限**。`shutdown_with_timeout` は診断用の有界版で、タイムアウト
  時に偽って `idle` にしません。
- **USB が stuck した場合の cleanup は意図的に無期限**です。`src/rtl.rs` の
  `cancel_until_finished` は `rtlsdr_read_async` が戻るまで cancel を繰り返し、
  live な呼び出しを放置しません。最終手段は **SIGKILL であり、クリーンな終了では
  ありません**。
- CLI / HTTP の契約、有界 queue、切断時のセマンティクスは今回の変更で
  **変更していません**。

## 7. 実機 / アーキテクチャ

- `aarch64-unknown-linux-gnu` ターゲットは導入済み。以前
  `cargo check --target aarch64-unknown-linux-gnu --lib` の成功を観測しました。
  **コンパイル確認であり、実機実行・NEON 実行検証の証明ではありません。**
- **N100 実機の受信記録は別紙 [`hardware-n100.md`](hardware-n100.md) に分離**
  （2026-09-30 JST、観測記録）。real IQ decode と 1 回の HTTP `200` を観測したが、
  再現安定・30 分連続・OrangePi PC2 は未達。**受入完了ではない。**
  現在（2026-10-01）の安定性・品質ゲート・実機観測は
  [`stability.md`](stability.md) §5–§6 を参照。
- OrangePi PC2 の実機受入は未実施。N100 では RTL-SDR の列挙・open・
  autodetach/reattach を確認（`hardware-n100.md`）。
- `librtlsdr` はビルド時にリンクせず、実行時に `dlopen` します。実機では udev /
  `plugdev` 等の USB 権限が必要です。

## 8. ドキュメントの所有と更新方針

- `docs/standards.md` / `docs/license.md` は Phase 0 の**一次調査記録**。その確認
  ステータス（`[本文確認]` / `[導出]` / `[取得]` / `[検索]` / `[未検証]` /
  `[未実装]`）を**保持**します。確定した項目は根拠（版・節）を追記して更新し、
  未確認を「対応済み」に書き換えません。
- `docs/implementation-status.md`（本ドキュメント）は Phase 0–7 の**観測記録**。
  全テスト成功は、修正完了後の再実行で確認するまで主張しません。
- `docs/acceptance.md` は受入コマンドと判定基準。
- `README.md` は利用者向け入口。
- 要件と実装計画（`plans/SPEC.md` / `plans/実装計画.md`）は**正**であり、本書は
  その実装状況を記録するものです。計画自体は変更しません。
- `LICENSE` は MIT。著作権表示は
  `Copyright (c) 2026 ts-stream contributors`。
