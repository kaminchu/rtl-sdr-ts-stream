# N100 実機受信記録（部分・未完了）

- 対象: `ts-stream` の N100 実機 bring-up
- 記録日: **2026-09-30（JST）**。日付は実行環境の `date` とログのタイムスタンプに
  基づく（ログ最終は同日 23:28 JST。日跨ぎは観測していない）。
- 判定: **部分（PARTIAL）。受入完了ではない。** 未測定・未再現を「合格」と書かない。
- 生 IQ / TS / ログ / バリデータはリポジトリ外
  `/tmp/opencode/hardware-run/` にあり、**リポジトリには含めない**。
- 後半の replay / live は**作業ツリーの未コミット修正込み**のバイナリで実施した。
  本記録は特定コミットを指さない（ソースは未コミット）。

## 1. 環境

| 項目 | 値 |
| --- | --- |
| マシン / OS / CPU | `n100` / Ubuntu 24.04.5 LTS / Intel N100 (x86_64) |
| カーネル | 7.0.0-34-generic |
| デバイス | Generic RTL2832U, USB `0bda:2832`, index 0, serial `77771111153705700` |
| チューナ | Fitipower FC0013 |
| librtlsdr | 2.0.1（Ubuntu `librtlsdr2:amd64 2.0.1-2build1`） |
| 物理 ch / 中心周波数 | ch19 / 509142857 Hz |
| サンプルレート | 1200000 Hz |
| ゲイン / ppm | 手動 5.8 dB / 0（auto と ppm は §5） |

autodetach / 再 attach（`Detached kernel driver` → `Reattached kernel driver`）は各 run
で観測。永続設定は書き込んでいない。本記録が起動したサーバは停止し、drivers は
reattach 済み。

## 2. 確認できたこと（established）

- `list-devices`: `index=0 name=Generic RTL2832U serial=77771111153705700`。
- open / tune / IQ 取得 / stop / close が動作し、USB フローは有効（gap 0、所有
  プロセスのみ）。追加要求は `409`、`/version` は 200。
- **録画 IQ の offline replay が TS まで到達**。ch19 / 1200000 / gain 5.8 / ppm 0 の
  3 秒（7200000 byte）録画 `iq-gain/ch19_g1.0.cu8` を replay し、**608 TS /
  114304 byte** を出力（`final-live/replay-ch19.out`）。
- **HTTP 成功を 1 回観測**。`GET /stream?ch=19` が `200 video/mp2t` chunked、
  ttfb 1.473 s、121072 byte / 644 packet。ffprobe は H264 + AAC を認識。ただし
  **content は約 2.176 s で TS が stall**。切断後の reopen は `504`。
- Phase A の**元レスポンス body は後のテストで上書きされ、生 artifact は残っていない**。
  したがって**その body の新しい CRC 検証結果は主張しない**。

## 3. 録画 IQ / TS の構造検証

`final-live/recorded-ch19.ts`（上記 replay 出力）を、リポジトリの `fec` ヘルパを
使う `tsvalidate` で検証:

- sync 608/608、complete 608、TEI 0、trailing 0、continuity gap 0、duplicate 63、
  reset 0。
- **duplicate 63 は良性と証明していない**（発生機構は未特定）。
- PAT: 完全な section は 0（`assembled=0`）。PMT は有効な PAT が無いため未取得。
  **CRC は未検証**。
- ffprobe: H264 320x180 約 15 fps、HE-AAC 48 kHz stereo。SDT のサービス名は省略。
- 観測された TMCC（**より前の capture** の観測。本録画から再導出したものではない）:
  Mode 3、GI 1/8、QPSK、rate 2/3、interleave I=4、CFO 約 +11 kHz。

## 4. 未達 / 不明（honest gaps）

- **再現安定性・30 分連続は未達**。直近の live run 3 回はすべて `504`
  （10.646 / 10.594 / 10.615 s）、`cp_locked=false`・`tmcc_frames=0`。ただし USB
  フローは有効（input_bytes 約 2300 万、gaps 0）。無 RF かアルゴリズム側かは
  **切り分けていない**（「RF が無い」とは結論しない）。
- HTTP 200 の TS は stall し、連続配信の安定は未確認。
- PMT / CRC 未検証、duplicate 63 未説明、SDT 名は省略。
- OrangePi PC2 は**未実施**（aarch64 は cross-check のみ）。
- 受入は未合格（[`acceptance.md`](acceptance.md) §3/§4）。

## 5. ゲインと ppm

- auto gain: 約 19% のサンプルが clip。手動 5.8 dB: 約 0.008% clip。
- `ppm = -2` は no-op 実験（`rtlsdr_set_freq_correction` の `-2` は「既にその値」）。
- ppm を動かした録画 replay（±2 ppm）は decode せず（TMCC 0）。RF 条件差か ppm
  効果かは不明。

## 6. 作業ツリーの修正（未コミット）

- `src/rtl.rs`: `rtlsdr_set_freq_correction` の `-2` を「既に設定済み」として許容
  （既定 `ppm = 0` で open が失敗しない）。
- `src/fec/mod.rs`: GF(256) の log/antilog テーブル化、Chien search の逐次化。
- `src/demod/coding.rs`: byte デインタリーバ commutator 位相を raw Viterbi の sync
  grid から推定、取得 sweep の rotating cursor、取得コストの bounded 化。
- `src/demod/receiver.rs` / `sync.rs`: pre-TMCC watchdog（false CP lock を破棄）、
  lock diagnostics（`cp_locked` / `mode` / `guard` / `cfo` / `timing_offset`）。
- `src/session.rs`: phase タイミングと slow batch / sync diagnostics（stderr）。

検証（作業ツリー、2026-09-30）:

- `cargo test --release --lib demod::replay_tests -- --test-threads=1` → **8/8 成功**。
- `cargo test --release --lib demod::coding` → **29 成功**。
- full suite は再実行していない。**最後の full 実行記録 228 を保持し、229 は主張
  しない**。

## 7. 再現コマンド（release）

```sh
cargo build --release
target/release/ts-stream list-devices
target/release/ts-stream server --host 127.0.0.1 --port 3000 \
  --device 0 --gain 5.8 --ppm 0 --sample-rate 1200000
curl -N "http://127.0.0.1:3000/stream?ch=19"
```

（録画 IQ の replay はツール／テスト経由。リポジトリ fixture ではない。）

## 8. 関連

- [`implementation-status.md`](implementation-status.md) §7
- [`acceptance.md`](acceptance.md) §3/§4
- `README.md`「実装の現状」

## 9. 現在の観測へのポインタ

- 本記録（2026-09-30）は**当時の履歴**として保持します。2026-10-01 の安定性修正・
  品質ゲート再実行・現在の実機観測は [`stability.md`](stability.md) §5–§6 を
  参照してください。

## 10. 2026-10-01 10:04 JST（replacement・最新）

- **記録日時**: 2026-10-01 10:04 JST。run ディレクトリ（リポジトリ外）は
  `/tmp/opencode/hardware-run/replacement-20261001T100406/`。
- **デバイス**: index 0、Generic RTL2832U / FC0013。**報告 serial は従来と同じ
  `77771111153705700`**。これは報告値であり、**物理的に同一固体かは主張しない**
  （ユーザーによる交換の可能性がある）。
- **受信条件**: ch19 / 509142857 Hz、サンプルレート 1200000 Hz、gain auto、ppm 0。
- **HTTP `GET /stream?ch=19` は 2 回とも `200 video/mp2t` chunked**:

  | # | TTFB | body | TS packets | ffprobe duration | curl |
  | --- | ---: | ---: | ---: | ---: | ---: |
  | 1 | 1.728372 s | 1208088 byte | 6426 | 23.210667 s | 25 s（意図的に打ち切り） |
  | 2（再接続） | 1.443616 s | 703496 byte | 3742 | 13.680333 s | 15 s |

- **バリデータ（既存 validator）**: 両 attempt とも all sync / TEI 0 /
  continuity gap 0。**有効な PAT が無いため、PAT / PMT / CRC は主張しない。**
- ffprobe は H264 + AAC を認識。2 回目は mid-GOP 開始の警告が出るが、過度な
  解釈はしない。
- `/version` は `200`。同時 `/stream?ch=20` は `409`（単一枠排他）。
- **終了時 stats**: attempt 1 `TMCC 103 / TS 6490`、attempt 2 `TMCC 61 / TS 3806`
  （2 回目の初期ロック時の 30 パケットではなく、終了時の集計）。両回とも `gaps 0` / `ts_stall 0` /
  `TMCC timeout 0` / `sync_losses 0`。
- サーバ側の TS カウントは打ち切り時点の body より **64 多い**。これは
  ドロップ原因の証明ではなく、queue / in-flight 分の可能性として留保する。
- 起動した own プロセスは停止済み、カーネルドライバは reattach 済み。
- **判定**: 25 s + 15 s の SHORT live は通過。**30 分連続・OrangePi PC2・
  エンドツーエンド CPU / RSS 予算の検証ではない**。slow batch は 131–150 ms で、
  headroom リスクは残る。
- 本節は実機 run の記録。本ドキュメント作業では**ソース変更・テスト追加・
  commit を行っていない**。
