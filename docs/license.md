# 依存ライセンス確認メモ

本メモは `plans/実装計画.md` §3・§4・Phase 0 に対応するライセンス確認の記録です。
**広範な法的保証を与えるものではありません。** 配布・再許諾の判断は、必要に応じて
法務確認を行ってください。

- 自プロジェクトのライセンス: **MIT**（[`LICENSE`](../LICENSE)）
- プロジェクトの著作権表示: 特定の個人・法人を創作せず
  `Copyright (c) 2026 ts-stream contributors` としています。実際の権利者名が
  決まった場合は `LICENSE` の当該行を置き換えてください。
- 確認方法: `Cargo.lock`（version 3）に記録された各パッケージの
  name / version をキーに、`~/.cargo/registry/src/` 内の該当クレートの
  `Cargo.toml` の `license` フィールドをオフラインで読み取りました。
  ネットワークからの再取得や `cargo metadata` によるオンライン照会はしていません。

## 1. 結論（MIT 選択との整合）

- 直接依存・推移依存のうち、ローカルで確認できたすべてのクレートは
  **MIT / Apache-2.0 / BSD-3-Clause / ISC / Unlicense / BSL-1.0 などの
  許諾型（permissive）** ライセンスで、コピーレフトは含まれていません。
  したがって、本プロジェクトを MIT で配布することと矛盾しません。
- ただし MIT 以外の条件を含む依存（BSD-3-Clause、ISC、Unicode-3.0、
  Unlicense、BSL-1.0）があるため、**各ライセンスの帰属表示・通知文の保持**が
  必要です。バイナリを再配布する場合は third-party notices を同梱してください。
- Apache-2.0 の依存は MIT プロジェクトへ組み込めますが、再配布時は Apache-2.0
  の NOTICE 要件に注意が必要です。
- 本リポジトリは現時点で第三者クレートのライセンス全文を `vendor/` 等に同梱して
  いません。配布形態（crates.io / バイナリ / コンテナ）に応じて追加してください。

> 本メモは「MIT で問題ない」と断定する法的判断ではありません。ライセンス文面の
> 最終確認と NOTICE 同梱の要否は法務確認の対象です。

## 2. 直接依存

`Cargo.toml` の `[dependencies]` / `[dev-dependencies]` に対応します。

| クレート | version | 宣言ライセンス | 用途 |
| --- | --- | --- | --- |
| anyhow | 1.0.104 | MIT OR Apache-2.0 | エラー伝播 |
| axum | 0.8.9 | MIT | HTTP サーバ |
| clap | 4.6.7 | MIT OR Apache-2.0 | CLI |
| futures-core | 0.3.34 | MIT OR Apache-2.0 | Stream trait |
| libloading | 0.8.9 | **ISC** | `librtlsdr` の `dlopen` |
| num-complex | 0.4.6 | MIT OR Apache-2.0 | 複素数 |
| rustfft | 6.4.1 | MIT OR Apache-2.0 | FFT |
| serde | 1.0.229 | MIT OR Apache-2.0 | JSON シリアライズ |
| thiserror | 2.0.21 | MIT OR Apache-2.0 | エラー型 |
| tokio | 1.53.1 | MIT | 非同期ランタイム |
| http-body-util (dev) | 0.1.5 | MIT | テスト |
| serde_json (dev) | 1.0.151 | MIT OR Apache-2.0 | テスト |
| tower (dev) | 0.5.3 | MIT | テスト |

## 3. 推移依存（`Cargo.lock` より）

ローカルレジストリで `Cargo.toml` を確認できた全パッケージです（アルファベット順）。

| クレート | version | 宣言ライセンス |
| --- | --- | --- |
| anstream | 1.0.0 | MIT OR Apache-2.0 |
| anstyle | 1.0.14 | MIT OR Apache-2.0 |
| anstyle-parse | 1.0.0 | MIT OR Apache-2.0 |
| anstyle-query | 1.1.5 | MIT OR Apache-2.0 |
| anstyle-wincon | 3.0.11 | MIT OR Apache-2.0 |
| atomic-waker | 1.1.2 | Apache-2.0 OR MIT |
| autocfg | 1.5.1 | Apache-2.0 OR MIT |
| axum-core | 0.5.6 | MIT |
| bytes | 1.12.1 | MIT |
| cfg-if | 1.0.5 | MIT OR Apache-2.0 |
| clap_builder | 4.6.7 | MIT OR Apache-2.0 |
| clap_derive | 4.6.7 | MIT OR Apache-2.0 |
| clap_lex | 1.1.1 | MIT OR Apache-2.0 |
| colorchoice | 1.0.5 | MIT OR Apache-2.0 |
| errno | 0.3.14 | MIT OR Apache-2.0 |
| form_urlencoded | 1.2.2 | MIT OR Apache-2.0 |
| futures-channel | 0.3.34 | MIT OR Apache-2.0 |
| futures-task | 0.3.34 | MIT OR Apache-2.0 |
| futures-util | 0.3.34 | MIT OR Apache-2.0 |
| heck | 0.5.0 | MIT OR Apache-2.0 |
| http | 1.5.0 | MIT OR Apache-2.0 |
| http-body | 1.1.0 | MIT |
| httparse | 1.10.1 | MIT OR Apache-2.0 |
| httpdate | 1.0.3 | MIT OR Apache-2.0 |
| hyper | 1.11.1 | MIT |
| hyper-util | 0.1.21 | MIT |
| is_terminal_polyfill | 1.70.2 | MIT OR Apache-2.0 |
| itoa | 1.0.18 | MIT OR Apache-2.0 |
| libc | 0.2.189 | MIT OR Apache-2.0 |
| log | 0.4.34 | MIT OR Apache-2.0 |
| matchit | 0.8.4 | **MIT AND BSD-3-Clause** |
| memchr | 2.8.3 | **Unlicense OR MIT** |
| mime | 0.3.17 | MIT OR Apache-2.0 |
| mio | 1.2.3 | MIT |
| num-integer | 0.1.47 | MIT OR Apache-2.0 |
| num-traits | 0.2.19 | MIT OR Apache-2.0 |
| once_cell | 1.21.4 | MIT OR Apache-2.0 |
| once_cell_polyfill | 1.70.2 | MIT OR Apache-2.0 |
| percent-encoding | 2.3.2 | MIT OR Apache-2.0 |
| pin-project-lite | 0.2.17 | Apache-2.0 OR MIT |
| primal-check | 0.3.4 | MIT OR Apache-2.0 |
| proc-macro2 | 1.0.107 | MIT OR Apache-2.0 |
| quote | 1.0.47 | MIT OR Apache-2.0 |
| ryu | 1.0.23 | **Apache-2.0 OR BSL-1.0** |
| serde_core | 1.0.229 | MIT OR Apache-2.0 |
| serde_derive | 1.0.229 | MIT OR Apache-2.0 |
| serde_path_to_error | 0.1.20 | MIT OR Apache-2.0 |
| serde_urlencoded | 0.7.1 | MIT/Apache-2.0 |
| signal-hook-registry | 1.4.8 | MIT OR Apache-2.0 |
| slab | 0.4.12 | MIT |
| smallvec | 1.16.2 | MIT OR Apache-2.0 |
| socket2 | 0.6.5 | MIT OR Apache-2.0 |
| strength_reduce | 0.2.4 | MIT OR Apache-2.0 |
| strsim | 0.11.1 | MIT |
| syn | 3.0.6 | MIT OR Apache-2.0 |
| sync_wrapper | 1.0.2 | **Apache-2.0** |
| thiserror-impl | 2.0.21 | MIT OR Apache-2.0 |
| tokio-macros | 2.7.2 | MIT |
| tower-layer | 0.3.3 | MIT |
| tower-service | 0.3.3 | MIT |
| tracing | 0.1.44 | MIT |
| tracing-core | 0.1.36 | MIT |
| transpose | 0.2.3 | MIT OR Apache-2.0 |
| unicode-ident | 1.0.26 | **(MIT OR Apache-2.0) AND Unicode-3.0** |
| utf8parse | 0.2.2 | Apache-2.0 OR MIT |
| windows-link | 0.2.1 | MIT OR Apache-2.0 |
| windows-sys | 0.61.2 | MIT OR Apache-2.0 |
| zmij | 1.0.23 | MIT |

### 3.1 ローカルで確認できなかったパッケージ

| クレート | 理由 | 扱い |
| --- | --- | --- |
| wasi 0.11.1+wasi-snapshot-preview1 | 対象外プラットフォーム向けでレジストリに未展開 | 未確認。Linux ビルドには使われません。配布対象にする場合は改めて確認。 |
| ts-stream 0.1.0 | 本クレート自身 | 自プロジェクトの MIT |

### 3.2 注意が必要なライセンス

- `matchit`（MIT **AND** BSD-3-Clause）: 両方の条件を満たす必要があり、
  BSD-3-Clause の再配布条件（著作権表示・免責）を保持してください。
- `unicode-ident`（(MIT OR Apache-2.0) **AND** Unicode-3.0）: Unicode
  データのライセンス条項が加わります。
- `libloading`（ISC）: ISC の著作権表示・許諾文の保持が必要です。
- `memchr`（Unlicense **OR** MIT）: MIT 側を選べば MIT と同じ扱いです。
- `ryu`（Apache-2.0 **OR** BSL-1.0）: BSL-1.0 は Boost Software License
  （許諾型）であり、Business Source License ではありません。
- `sync_wrapper`（Apache-2.0）: MIT プロジェクトへ組み込み可。NOTICE 要件に注意。

## 4. 実行時依存 `librtlsdr` と GPL

- 本クレートは `librtlsdr` を**ビルド時にリンクせず**、実行時に `libloading` で
  `dlopen` します（`src/rtl.rs`）。
- `librtlsdr`（Osmocom rtl-sdr プロジェクト）は **GPL-2.0-or-later** で配布されて
  います。本メモでは SPDX / プロジェクト表記に基づく事実として記載し、最終的な
  法的解釈はしません。
- GPL のソフトウェアを**再配布する場合**（例: `librtlsdr` のバイナリやソースを
  同梱する、配布パッケージに含める）は、その部分について GPL の条件
  （ソース提供・ライセンス全文・著作権表示）が適用されます。
- 本プロジェクトは `librtlsdr` を同梱していません。利用者が各自インストールする
  前提です。`dlopen` による動的利用と、GPL の「派生著作物」該当性の評価は
  状況依存であり、**本メモは「安全」とも「危険」とも断定しません**。再配布の
  形態が決まった時点で法務確認を行ってください。
- 注意: `plans/実装計画.md` §3 のとおり、「GPL 可」は個別の既存復調コードの
  コピー許諾を意味しません。本プロジェクトは GNU Radio / gr-isdb / Python を
  実行時・ビルド時・テスト時のいずれの依存としても持ち込みません。

## 5. 更新手順

`Cargo.lock` を更新したら、上表を再生成・再確認してください。確認コマンド例:

```sh
# name/version の一覧
cargo metadata --format-version 1 --offline >/dev/null 2>&1 || true
grep -n '^name\|^version' Cargo.lock

# 各クレートの宣言ライセンス（ローカルレジストリ）
for d in ~/.cargo/registry/src/*/<name>-<version>; do
  grep -m1 '^license' "$d/Cargo.toml"
done
```

新しい依存を追加する際は、MIT と両立する許諾型ライセンスであること、および
帰属表示要件を確認してから採用してください。
