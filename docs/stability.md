# 安定性・最終検証記録（現在の観測）

- 対象: `ts-stream`（RTL-SDR / ISDB-T ワンセグ / TS HTTP ストリーマ）
- 記録日: **2026-10-01**（安定性修正と品質ゲート再実行の観測）
- 位置づけ: 本書は**現在の**安定性検証と品質ゲートの観測記録です。
  [`hardware-n100.md`](hardware-n100.md) の 2026-09-30 記録は**当時の履歴**として
  そのまま保持し、本書で上書きしません。
- 原則: 未測定・未実装を「達成」と書かない。過去の成功を現在の成功として流用
  しない。

## 0. 結論（現在の主張範囲）

- **今回特定した不具合のソフトウェア修正（IQ→TS 経路）は完了しました。ただし
  安定性の全項目が完了したわけではありません。** release full suite は
  **241 passed / 0 failed / 1 ignored**、明示実行した ignore テストは **1 passed**。
- 合成 replay の SFO 追随（`0` / `±20` / `±50` ppm、8 フレーム）は各 run で
  **TMCC 7 / TS 323 / resets 0 / ts_stall 0** まで回復しました。旧版は +50 ppm で
  **TMCC 7 / TS 101 / ts_stall 1** でした。
- 長時間 ignore テスト（48 フレーム、約 11.1 秒、+20 ppm と +3.2 bin CFO を同時
  印加）は **TMCC 46 / TS 2186 / resets 1（coarse 整数 CFO）/ resets_ts_stall 0**。
  これは **30 分連続や実機性能の証明ではありません**。
- N100 実機の**元の real IQ 3 秒 → 608 TS / 114304 byte** の記録は保持します。
  2026-10-01 10:04 JST の live 2 回（25 s / 15 s）は CP 同期して
  `200 video/mp2t` と TS を返しました（詳細は §5.1）。以前の CP false / TMCC 0 の
  観測は §5.2 の**履歴**であり、現在の主張ではありません。

## 1. Viterbi の安定化

- **ACS ごとの再正規化**: 最良の有限パスメトリックを毎ステップ `0.0` に正規化
  します。累積加算の桁落ちで近接サバイバが同値に丸められるのを防ぎます。
- **メトリックは `f64`**: 軟判定値は `f32` のまま公開 API を変えず、分岐和の前に
  `f64` へ拡張します。`f32::MAX` の ACS 和でもオーバーフローせず、符号だけで
  正しく復号できることをテストで固定しています
  （`k7_viterbi_decodes_max_finite_magnitude`、
  `streaming_viterbi_decodes_at_max_finite_magnitude`）。
- **ストリーミングの NaN / ±inf は消失（erasure）**: `StreamingViterbi` は
  非有限の軟判定値を `0.0` として扱い、メトリックを汚染せずパニックしません
  （`streaming_viterbi_treats_non_finite_soft_as_erasure`）。
- **ブロック API は拒否**: `fec::decode_k7_soft` は非有限値を
  `FecError::NonFinite` で明示的に拒否します。ストリーミング側にエラーチャネルが
  無いための「消失」との意図的な差です。

## 2. SFO（サンプルクロック）と整数 CFO

- **SFO 累積 de-ramp は維持**: シンボルごとの累積タイミングオフセットによる
  キャリア位相 de-ramp は、差動 TMCC / DQPSK の位相連続性のためにそのまま
  残しています。
- **コヒーレント等化では巻き戻す**: 同期変調の SP チャネル推定は、de-ramp を
  [`restore_timing_offset`](../src/demod/sync.rs) で物理キャリアに戻してから行い
  ます。12 キャリア間隔のパイロット位相が `pi` に近づくと線形補間が破綻するため
  です。
- **シンボル履歴は 1 フレーム分**: [`Decoder`](../src/demod/receiver.rs) は直近
  `SYMBOLS_PER_FRAME` 個の `timing_offset` をフレーム収集器と歩調を合わせて保持
  し、検証済みフレームの各シンボルに対応付けます。
- **厳密検証**: 履歴長とフレームのキャリア数が一致しなければゼロ埋めせず、型付き
  `Internal` エラーとしてドメインを破棄し TS を出しません
  （`timing_history_mismatch_is_a_typed_error_without_ts`）。
- **コヒーレント整数 CFO は実装済み**: 粗探索
  [`coarse::estimate_integer_cfo`](../src/demod/coarse.rs) を同期器に統合し、同一の
  整数推定が `COARSE_CONFIRM_SYMBOLS` 回連続したときだけ補正します。非ゼロ補正は
  ロック世代を進め、下流の連続性依存状態を捨てさせます。
- **DQPSK の整数 CFO は自動補正しません**: DQPSK には SP が無いため粗探索は確定
  せず、小数 CFO のみです。**DQPSK の SFO 補正は未検証**で、de-ramp は現状のまま
  継承します（別途 ppm 既知なら呼び出し側が指定する想定）。

## 3. 合成 replay（短期・SFO）

`cargo test --release raw_iq_sample_clock_offset_decodes_contiguous_ts --
--nocapture --test-threads=1`（8 フレーム、chunk 4096）:

| ppm | TMCC | TS | resets | ts_stall |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 7 | 323 | 0 | 0 |
| +20 | 7 | 323 | 0 | 0 |
| -20 | 7 | 323 | 0 | 0 |
| +50 | 7 | 323 | 0 | 0 |
| -50 | 7 | 323 | 0 | 0 |

いずれも送信 TS の完全な連続一致区間を復元します。**独立録画 RF の証明では
ありません。**

## 4. 長時間 ignore テスト

`cargo test --release raw_iq_sustained_sfo_plus_cfo_decodes_contiguous_ts --
--ignored --nocapture --test-threads=1`:

- 48 フレーム、`~11.10 s`、SFO `+20 ppm`、RF CFO `+3.2 bin`、chunk 8192。
- 結果: `tmcc_frames=46`、`ts_packets=2186`、`resets=1`（`resets_sync=1`、
  coarse 整数補正）、`gaps=0`、`resets_ts_stall=0`。
- 起動後、TS を出さない秒はありません（1 秒目は取得コスト、以降は各秒 > 0）。
- **位置づけ**: 約 11 秒の合成信号に対する回帰であり、**30 分連続・USB 取りこぼし
 ・メモリ増加・実機性能の証明ではありません。**

## 5. 実機（N100）

### 5.1 最新（2026-10-01 10:04 JST）

- **ch19 / auto gain / ppm 0 の live `GET /stream?ch=19` が 2 回とも
  `200 video/mp2t` chunked**。run ディレクトリ（リポジトリ外）は
  `/tmp/opencode/hardware-run/replacement-20261001T100406/`。
  - attempt 1: TTFB `1.728372 s`、body `1208088 byte` / `6426` TS packets、
    ffprobe `23.210667 s`（curl は 25 s で意図的に打ち切り）。
  - attempt 2（再接続）: TTFB `1.443616 s`、body `703496 byte` / `3742` TS
    packets、ffprobe `13.680333 s`（curl 15 s）。
- **バリデータ（既存 validator）**: 両 attempt とも all sync / TEI 0 / continuity
  gap 0。**有効な PAT が無いため PAT / PMT / CRC は主張しない。** ffprobe は
  H264 + AAC を認識（2 回目は mid-GOP 開始）。
- `/version` は `200`、同時 `/stream?ch=20` は `409`。
- 終了時 stats: attempt 1 `TMCC 103 / TS 6490`、attempt 2 `TMCC 61 / TS 3806`
  （2 回目の初期ロック時の 30 パケットではなく、終了時の集計）。両回とも `gaps 0` / `ts_stall 0` /
  `TMCC timeout 0` / `sync_losses 0`。サーバ側 TS カウントは打ち切り時点の body より
  `64` 多く、ドロップ原因の証明とはせず queue / in-flight の可能性として留保する。
- デバイスは index 0 / Generic RTL2832U / FC0013、**報告 serial は従来と同じ
  `77771111153705700`**。**物理的に同一固体かは主張しない**。
- 起動したプロセスは停止済み、ドライバは reattach 済み。
- **位置づけ**: 25 s + 15 s の SHORT live が通過。**30 分連続・OrangePi PC2・
  エンドツーエンド CPU / RSS 予算の証明ではない**。slow batch は 131–150 ms で
  headroom のリスクは残る。詳細は [`hardware-n100.md`](hardware-n100.md) §10。

### 5.2 以前の観測（履歴・現在の主張ではない）

- 以下は最新 run（§5.1）より前の記録であり、**現在の主張として流用しない**。
- **保持する元記録**: 2026-09-30 の real IQ 3 秒（7200000 byte）の offline replay が
  **608 TS / 114304 byte** に到達。当該 body の新しい PAT / CRC 検証は主張しません
  （[`hardware-n100.md`](hardware-n100.md) §2–§3）。
- **当時の 20 秒 capture / 閾値**: 既知 Mode 3 / GI 1/8 の CP 相関スコアは、以前の
  capture で `0.9914`、新しい capture で `0.142–0.166`。**いずれも同一
  アルゴリズムで測定した別 capture の値であり、旧実装と新実装の比較では
  ありません。** 取得閾値（平均 `0.72`、各シンボル最小 `0.60`、CFO コヒーレンス
  `0.85`）を大きく下回ります。**これは false positive を棄却する側であり、閾値を
  下げて通しません。**
- **当時の live 2 回（504）**: ch19 / auto gain / ppm 0。2 回とも HTTP `504` で、
  継続時間は `10.6471 s` と `10.6520 s`。ともに `cp_locked=false`・
  `tmcc_frames=0`。IQ フローは健全（入力約 2000–2100 万 byte、gaps 0）。
  **RF が無いとは結論しません**（RF 条件とアルゴリズムの切り分けは未了）。
- 当時、起動した hardware プロセスは停止済み、カーネルドライバも reattach 済み。

## 6. 品質ゲート（2026-10-01 実測）

| コマンド | 結果 |
| --- | --- |
| `cargo fmt --check` | 成功 |
| `cargo check --all-targets` | 成功 |
| `cargo clippy --all-targets -- -D warnings` | 成功 |
| `cargo test --release` | **241 passed / 0 failed / 1 ignored**（lib 207 中 206 pass・1 ignore、統合 34 pass、doc 1 pass） |
| `cargo test --release raw_iq_sustained_sfo_plus_cfo_decodes_contiguous_ts -- --ignored --nocapture --test-threads=1` | **1 passed**（§4） |
| `cargo check --target aarch64-unknown-linux-gnu --lib` | 成功（**コンパイル確認のみ。ARM 実機実行ではない**） |

Debug profile の full suite は再実行していないため、Debug での全成功は主張しません。
N100 実機では USB 受信・HTTP の再試験を実施し、2026-10-01 10:04 JST の
SHORT live（25 s / 15 s）では CP 同期して `200 video/mp2t` と TS を観測しました
（§5.1）。ただし **30 分連続・OrangePi PC2・エンドツーエンド CPU / RSS 予算は
未達・未実施**です。

## 7. 主張しないこと

- 30 分連続稼働・USB 取りこぼしなし・メモリ増加なし・CPU/RSS 性能予算。
- 実機（N100 / OrangePi PC2）でのリアルタイム処理・ARM 実行検証。
- 独立した録画 RF / 既知ベクトルによる CRC・continuity の独立検証。
- 現在の live における「RF が無い」という結論。
- raw DQPSK replay と DQPSK の SFO 補正。
