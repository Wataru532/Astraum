# Astraum プログラム仕様書

- 対象バージョン: 1.1.1（`src/shared/brand.hpp` の `APP_VERSION`）
- 旧名称: StellarForge（名称変更前の開発版。ファイルとデータはそのまま引き継げます）
- 作成日: 2026-11-19
- 対象: `src/` 以下のソースコード、`tools/` のスクリプト、ビルド設定、埋め込みリソース

---

## 目次

1. [概要](#1-概要)
2. [アーキテクチャ（クリーンアーキテクチャ）](#2-アーキテクチャクリーンアーキテクチャ)
3. [実行の流れ](#3-実行の流れ)
4. [ビルドとリソース](#4-ビルドとリソース)
5. [ソースファイル別仕様](#5-ソースファイル別仕様)
   - 5.1 [shared（共有）](#51-shared共有)
   - 5.2 [domain（ドメイン層）](#52-domainドメイン層)
   - 5.3 [application（アプリケーション層）](#53-applicationアプリケーション層)
   - 5.4 [infrastructure（インフラストラクチャ層）](#54-infrastructureインフラストラクチャ層)
   - 5.5 [presentation（プレゼンテーション層）](#55-presentationプレゼンテーション層)
   - 5.6 [main.cpp と rl.hpp](#56-maincpp-と-rlhpp)
   - 5.7 [tests（自動テスト）](#57-tests自動テスト)
6. [地表基地](#6-地表基地)
7. [データファイルと保存形式](#7-データファイルと保存形式)
8. [開発用ツール（tools/）](#8-開発用ツールtools)
9. [セキュリティ上の性質](#9-セキュリティ上の性質)

---

## 1. 概要

Astraum は Windows 向けの宇宙飛行シミュレーターです。C++20 と raylib 5.5（OpenGL 3.3）で書かれています。

| 区分 | 内容 |
|---|---|
| 機体設計 | 部品を縦に積む「工場」と、自由形状の部品を組み合わせる「CAD 設計」 |
| 飛行 | 6 自由度の剛体運動、重力、大気抵抗と揚力、推力、パラシュート、着陸脚と車輪、ドッキング |
| 宇宙 | 太陽系（惑星、主要衛星、準惑星、彗星 7 つ）とアルファケンタウリ、プロキシマ、バーナード星系 |
| 地表基地 | 月や火星などに居住・温室・ISRU（現地資源による推進剤製造）・電源モジュールを降ろして基地を建設する。条件を満たすと基地に発射台ができ、そこからロケットを打ち上げられる（6 章） |
| 描画 | 実測の地表テクスチャと高度、物理ベースの大気散乱、輪、BSC5 恒星カタログ、天の川、小惑星帯とカイパーベルト、基地の建物 |
| 補助 | ミッション支援（目標の天体への打ち上げ時刻・出発噴射・軌道修正・捕獲噴射の案内）、チュートリアル（8 ミッション）、実績、ランデブー誘導、時間加速、ハイパースペース航行 |
| 表示 | GUI のサイズ（75〜150 %）、F11 で枠なし全画面（設定画面でも切り替え可）、英語・日本語・ドイツ語 |

実行ファイルは `Astraum.exe` の 1 つ（約 140 MB）です。地図、恒星データ、効果音、BGM、ライセンス表記はすべてリソースとして埋め込まれています。実行時にネットワーク通信は行いません。

---

## 2. アーキテクチャ（クリーンアーキテクチャ）

### 2.1 層とディレクトリ

```
src/
├── shared/          共有（翻訳、名称）            … 純粋な C++
├── domain/          ドメイン層（ゲームの規則）    … 純粋な C++（raylib・Windows・ファイル操作なし）
├── application/     アプリケーション層（ユースケース） … C++ 標準ライブラリのみ
├── infrastructure/  インフラストラクチャ層（raylib・Win32・地形データ・音声）
├── presentation/    プレゼンテーション層（画面、UI、入力、3D 描画）
├── tests/           自動テスト
├── main.cpp         コンポジションルート（各層の組み立てとメインループ）
├── rl.hpp           raylib への橋渡し（外側の層だけが使う）
├── app.rc           埋め込みリソース
└── audio/           効果音・BGM の原本
```

依存の向きは常に外側から内側です。

```
 presentation ─┐
               ├─> application ─> domain ─> shared
 infrastructure┘        │            ▲
       │                └────────────┘
       └──（ポートの実装）──> domain / application のポート
 main.cpp: すべてを組み立てる
```

### 2.2 依存規則の強制

`CMakeLists.txt` は内側の層を独立した静的ライブラリとしてビルドします。

| ターゲット | 含むもの | 見えるインクルードディレクトリ |
|---|---|---|
| `astraum_domain` | `shared/*.cpp`、`domain/*.cpp` | `shared`、`domain` のみ |
| `astraum_application` | `application/*.cpp` | 上記と `application` |
| `Astraum`（実行ファイル） | `infrastructure`、`presentation`、`tests`、`main.cpp` | すべて＋raylib |

内側のライブラリからは raylib や外側のヘッダーが見えません。誤って `#include "raylib.h"` や `#include "ui.hpp"` を書くとコンパイルエラーになるので、依存規則が守られていることをビルドで確認できます。

### 2.3 ポート（依存性逆転）

内側の層が外部の機能を必要とするときは、内側でインターフェース（ポート）を定義し、外側が実装を差し込みます。差し込みは `main.cpp` が起動時に行います。

| ポート | 定義 | 実装 | 内容 |
|---|---|---|---|
| `GroundPort` | `domain/ground.hpp` | `infrastructure/terrain.cpp` の `MapGround` | 天体の標高、地形の改訂番号、詳細タイルの追従、基地周囲の地面の平坦化 |
| `diag` | `domain/diag.hpp` | `infrastructure/crash.cpp`（`crash::breadcrumb`） | クラッシュ解析用の操作履歴 |
| `ports::Platform` | `application/ports.hpp` | `main.cpp`（raylib の `GetTime`、`TraceLog`）、`infrastructure/platform.cpp` | 時計、OS の表示言語、ログ |
| `ports::Presentation` | `application/ports.hpp` | `presentation/map_dialogs.cpp`、`presentation/tutorial.cpp` | マップのサムネイル、チュートリアルの状態 |

実装が差し込まれていなくても、既定の動作で動きます（地面は滑らかな球、言語は英語、時計は `std::chrono`）。

### 2.4 型の境界

ドメインは raylib の `Color` と `Vector3` の代わりに、独自の `Rgba8` と `Vec3f`（`domain/types.hpp`）を使います。これらは同じ並びの raylib の型と暗黙に相互変換できるため、外側の層は描画関数へそのまま渡せます。倍精度の `Vec3d` を raylib の型に変える `toF`、`toMatrix` は `rl.hpp` にあります。

---

## 3. 実行の流れ

### 3.1 起動（`main()`）

1. `migrateDataFolder`：旧名称のデータフォルダ（`%APPDATA%\StellarForge`）があり、新しいフォルダ（`%APPDATA%\Astraum`）がまだなければ、中身をコピーします（元のフォルダは残します）。
2. `crash::init`：クラッシュレポート機構を有効にします。
3. ウィンドウを作ります（MSAA、サイズ変更可、DPI に合わせた初期サイズ、最大化）。
4. `CATALOG.init`（部品カタログ）、`gfx::init`（シェーダー、メッシュ、恒星データ）。
5. ポートの差し込み：`diag::setSink`、`ground::install`、`ports::install`（2.3 の表）。
6. `App` を作り、`ensureDirs`、`loadPrefs`、`keys::load` を実行します。
7. `ui::init`、`sfx::init`、`gpumon::start`、`world.newGame`、`terrain::init`、`autotestInit`。
8. 前回全画面で終了していれば、全画面に戻します。

### 3.2 メインループ

1. 閉じる要求と、未保存時の確認ダイアログ（`closeDialog`）。
2. 自動保存（`serviceSaves`）。
3. メモリ逼迫時のキャッシュ削減。
4. フレームレートの調整（FPS 上限、適応的な詳細度）。
5. キー入力：F3 でデバッグ画面、F11 で全画面（`setFullscreen`）、F2 でスクリーンショット。
6. 画面ごとの関数を呼び出します（`menuFrame`、`factoryFrame`、`flightFrame`、`cadFrame`。`App::screen` で切り替え）。
7. 通知（`drawNotice`）と FPS 表示。

### 3.3 データの流れ

```
部品カタログ(parts) ─┬─> Design（スタック）─┐
                    └─> Craft（CAD）     ─┴─> World::launch ─> Vessel
着陸機 ─> planBase/buildBase ─> SurfaceBase ─> 発射地点（padSite）
Universe（天体の軌道）────────────────────> World::update（物理積分）
World ──> gfx（描画）／terrain（地形）──> 画面
World::serialize / deserialize <──> application/storage（ファイル）
```

---

## 4. ビルドとリソース

### 4.1 ビルド

```
cmake -B build -G Ninja
cmake --build build
```

- C++20、既定は Release ビルドです。
- `third_party/raylib` を静的ライブラリとしてビルドします。
- MinGW では静的リンクし、シンボルを除去します（`-s`）。リンクマップ `Astraum.map` は常に出力します。`-DSF_NOSTRIP=1` を付けると、シンボル付きの診断ビルドになります。
- 出力：`build/Astraum.exe`。

### 4.2 `src/app.rc`（埋め込みリソース）

| ID | 内容 | 利用箇所 |
|---|---|---|
| 1 | アイコン | ウィンドウ、エクスプローラー |
| 201–206 | 地球・月・火星の色地図と高度地図 | `infrastructure/terrain.cpp` |
| 211–216 | 同じ 3 天体の高解像度タイル（`.pak`） | `terrain.cpp` の詳細タイル |
| 221 | `stars.bin`（BSC5 恒星カタログ） | `presentation/render.cpp` |
| 222 | `milkyway.jpg`（天の川） | `render.cpp` |
| 231–242 | 水星、ガリレオ衛星、エンケラドゥス、トリトン、冥王星、カロン、ケレス、フォボス、木星の色地図 | `terrain.cpp` |
| 301–312 | 効果音 WAV 12 種 | `infrastructure/audio.cpp` |
| 321–323 | BGM 3 曲 | `audio.cpp` |
| 401 | `THIRD_PARTY_NOTICES.txt`（クレジットとライセンス文） | `presentation/menu.cpp` の `creditsModal` |
| VERSIONINFO | 製品名 Astraum、バージョン 1.0.0 | エクスプローラーのプロパティ |

リソースは `winshell::resource(id, data, size)` でメモリ上のポインターとして取り出します。

### 4.3 `assets/` とライセンス

- `maps/`、`sky/`：地図と空のデータ。出典は各 `SOURCES.txt`。
- `licenses/`：表記ファイルに転記するライセンス原文（MinGW-w64 ランタイム、Apache License 2.0）。
- プロジェクト直下の `THIRD_PARTY_NOTICES.txt`：`tools/make_notices.py` が生成します。配布時は実行ファイルと一緒に置きます。

---

## 5. ソースファイル別仕様

行数は概数です。

### 5.1 shared（共有）

| ファイル | 行数 | 役割 |
|---|---|---|
| `brand.hpp` | 24 | 名称（`APP_NAME` = Astraum）、バージョン、データフォルダ名、ファイルのタグ（`ASTRAUM_SAVE` など）。旧タグ `STELLARFORGE_*` も読み込み可能と判定する `hasTag` |
| `i18n.hpp/.cpp` | 20 / 1930 | 英語・日本語・ドイツ語の翻訳表（約 1500 項目）。`tr`、`trf`。フォントに必要な文字の一覧（`codepoints`、UTF-8 を自前で解読） |

### 5.2 domain（ドメイン層）

raylib、Windows、ファイル操作に依存しない、ゲームの規則そのものです。

#### `mathx.hpp`、`types.hpp`、`site.hpp`
- `mathx.hpp`：倍精度の `Vec3d`、`Mat3d`、`Quatd`、定数（`PI`、`AU`、`G0` など）、赤経赤緯から黄道座標への変換。
- `types.hpp`：`Rgba8`（色）、`Vec3f`（float ベクトル）。raylib の型と相互変換できます（2.4）。
- `site.hpp`：地球の発射台の高さ（`PAD_DECK_H`）と滑走路の寸法。

#### `ground.hpp`、`diag.hpp`、`ports.cpp`（ポート）
- `GroundPort`：地形のインターフェース（2.3）。`ground::height` などの関数でシミュレーションから使います。
- `diag::breadcrumb`：クラッシュ解析用の記録。
- `ports.cpp`：既定の実装（滑らかな地面、出力なし）。

#### `universe.hpp/.cpp`（70 / 470 行）
天体のデータベースです。
- `Body`：半径、GM、軌道要素、自転、大気、色（`Vec3f`）、輪、SOI。
- `Universe::init`：恒星 4、惑星 8、準惑星 2、衛星 12、系外惑星 3、彗星 7 を登録します。
- 天体の位置と自転の計算。SOI がヒル球より小さくなる小天体の補正。

#### `orbit.hpp/.cpp`（24 / 137 行）
二体問題の軌道計算です（`stateToConic`、`keplerPropagate`、`timeToTrueAnomaly`）。

#### `parts.hpp/.cpp`（160 / 1020 行）
工場の部品カタログ（36 種）とスタック式設計（`Design`）です。
- 部品の種類 `PK` と分類 `Cat`。地表基地用の分類 `Cat::Base` と種類 `PK::Module` があります。
- 基地モジュール 4 種：
  - 地表居住モジュール SH-4（4 人、9.5 t）
  - 温室モジュール GH-2
  - ISRU 推進剤プラント IP-5
  - 地表核分裂電源 FSP-40（40 kW）
- `analyzeDesign`（Δv、TWR など）、`.sfd` のテキスト変換（`designToText`、`designFromText`）。

#### `craft.hpp/.cpp`（250 / 2010 行）
CAD 用の自由形状機体のモデルと解析です。
- 収納状態の形状（フェアリング用）、質量特性。
- 空力表 `AeroTable` と、そのキャッシュ（上限 `AERO_CACHE_BUDGET` = 768 MB）。
- `.sfc` のテキスト変換。

#### `content.hpp/.cpp`（8 / 1200 行）
組み込みの設計データです。
- `builtinTemplates()`：工場のテンプレート 7 種（Kestrel Orbiter、Aurora Crew Mission など）。
- `builtinCrafts()`：CAD の作例（スペースシャトル、ローバー、ヘリコプター、ステーションなど）。

#### `sim.hpp/.cpp`（425 / 4400 行）
ゲーム世界の中核です。
- `World`：天体、機体、時刻、時間加速、実績、地表基地（`bases`）。
- `Vessel`：部品、質量、位置と速度、姿勢、燃料、電力、ステージ。
- 主な機能：
  - 物理（重力、推力、空力、姿勢制御、接地）
  - ステージ分離、ドッキング、ハイパースペース、巻き戻し（基地の状態も含む）
- 遭遇予測 `predictEncounter`：必要な天体（祖先と子天体）だけを各ステップ 1 回計算します（太陽周回で以前は 1 回 30 ms、現在 約 2 ms）。
- 探査状況：発射台から実際に飛行した機体（デブリを除く）だけを数えます。旧セーブの誤った「着陸済み」は読み込み時に修復します（`repairExplored`）。
- 発射地点：
  - `padSite()`：地球の発射場、または発射台のある基地を返します。
  - `deployBodies()`：ローバーや航空機の展開先（地球と、基地のある天体）。基地のある天体では、基地の北側に置きます（`launchAircraft`）。
  - `cycleLaunchSite`：発射地点を順に切り替えます。
  - `launchSiteName`：発射地点の名前を返します。
- 基地関連：
  - `unloadComponents`：着陸機から部品を降ろし、積み重ねの隙間を詰めて接地させ直します。
  - `syncBaseTerrain`：基地の周囲の地面を平らにします。
  - `groundHeight`：指定した地点の地面の高さを返します。
- 保存：`serialize` / `deserialize` でテキストとの変換だけを行います。ファイル操作は application 層が担当します。

#### `base.hpp/.cpp`（82 / 350 行）
地表基地の規則です（6 章）。
- `ModuleKind`（8 種）、`BaseModule`、`SurfaceBase`、`BaseStatus`。
- `baseAllowed`：建設できる天体かどうかの判定。
- `planBase`：建設計画（条件の確認、モジュールの抽出）。
- `buildBase`：建設の実行（新規、または既存の基地への追加）。
- `placeModule`：建物の配置。
- `completeLaunchComplex`：条件を満たしたときに発射台と推進剤タンクを追加します。

#### その他
| ファイル | 役割 |
|---|---|
| `achieve.hpp/.cpp` | 天体ごとの実績（到達、周回、着陸、ドッキング） |
| `navguide.hpp/.cpp` | ランデブーの計算（ランベルト問題、近距離誘導） |
| `mission.hpp/.cpp` | 目標の天体へのミッション支援。現在地と目標の関係（衛星・同じ親の惑星・親天体）から、局面（打ち上げ前、上昇、出発、遷移、到着、周回）と次の操作を決める。打ち上げ時刻は発射地点の真東の軌道面が目標の到着位置を含む時刻、惑星間は遷移ウィンドウ（ランベルト問題を出発日×飛行時間で探索し、最も早い窓を選ぶ）。出発噴射は狙いの近点高度になるよう、遭遇予測を使って照準を反復修正する。結果は 0.5 秒（実時間）キャッシュする |
| `features.hpp/.cpp` | IAU 命名の地形データ（ラベル表示、地形への彫り込み） |

### 5.3 application（アプリケーション層）

ユースケースです。C++ 標準ライブラリ以外には依存しません。

| ファイル | 行数 | 役割 |
|---|---|---|
| `app.hpp` | 90 | `App`：遊んでいる世界、編集中の設計、データの場所、設定値、画面間で共有する表示状態。画面の列挙 `Screen` |
| `ports.hpp/.cpp` | 40 / 47 | アプリケーション層のポート（2.3）と既定の実装 |
| `session.hpp/.cpp` | 34 / 300 | マップ（1 マップ = 1 セーブ）の一覧、新規作成、読み込み、保存（バックグラウンド書き込み）、名前の変更、削除、自動保存の判定、クイックセーブ、最後の設計の保存と復元、旧クイックセーブの移行 |
| `storage.hpp/.cpp` | 21 / 73 | ファイル入出力：一時ファイル経由の書き込み（`writeFileAtomic`）、サイズ上限付きの読み込み、`saveWorld`/`loadWorld`、設計と機体の保存・読み込み |
| `settings.cpp` | 83 | 設定（`prefs.txt`）の読み書き、データフォルダの作成（`ensureDirs`）、文字サイズや FPS 上限の定数 |
| `designs.cpp` | 170 | 保存済み設計の一覧、名前からのファイル名生成（Windows で使えない文字の置換、予約名の回避） |

### 5.4 infrastructure（インフラストラクチャ層）

| ファイル | 行数 | 役割 |
|---|---|---|
| `terrain.hpp/.cpp` | 73 / 940 | 地表データ（実測地図、生成地形、詳細タイルのワーカースレッド、微細な起伏、基地周囲の平坦化）。`GroundPort` の実装 `MapGround` |
| `crash.hpp/.cpp` | 22 / 286 | 例外や異常終了を捕捉し、スタックトレースを `%APPDATA%\Astraum\crash` に保存 |
| `audio.hpp/.cpp` | 38 / 475 | 効果音、環境音の合成、BGM の順番再生 |
| `platform.hpp/.cpp` | 6 / 11 | Windows の表示言語を取得（`ports::Platform` 用） |
| `screenshot.hpp/.cpp` | 19 / 78 | F2 で PNG を `ピクチャ\Astraum` に保存（別スレッド） |
| `winshell.hpp/.cpp` | 12 / 35 | ピクチャフォルダの取得、フォルダを開く、埋め込みリソースの取得 |
| `gpumon.hpp/.cpp` | 15 / 140 | GPU 使用率と温度（PDH） |
| `gputime.hpp/.cpp` | 15 / 136 | GL のタイムスタンプによる GPU 時間の計測 |
| `mem.hpp/.cpp` | 27 / 44 | メモリ予算（上限 4 GB、キャッシュ 768 MB）とヒープの圧縮 |

### 5.5 presentation（プレゼンテーション層）

| ファイル | 行数 | 役割 |
|---|---|---|
| `screens.hpp` | 33 | 画面とダイアログの関数の宣言（メインループから呼ばれる） |
| `menu.cpp` | 670 | タイトル画面、設定画面（GUI のサイズ、全画面切り替え、発射地点に基地を含む）、キー割り当て、クレジットとライセンス、クラッシュ報告、画面上部の通知 |
| `map_dialogs.cpp` | 310 | マップの新規作成・読み込み・終了確認のダイアログ、マップのサムネイル（3D 表示の縮小 PNG） |
| `factory.cpp` | 915 | 工場画面（部品ブラウザ、組み立て、性能表示、発射地点の切り替え、打ち上げ） |
| `cad.cpp` | 1900 | CAD 設計画面。ローバーや航空機の展開先は、地球と基地のある天体だけを選べる |
| `flight.cpp` | 3850 | 飛行画面、マップ画面、HUD。ミッション支援パネル（目標の天体があるとき。ウィンドウまでワープ、SAS 上昇誘導、SAS 噴射、噴射までワープ）。基地関連として、基地の描画、基地名ラベル、「基地を建設」ボタン、建設ダイアログ、基地一覧（発射地点の選択、名前の変更、放棄）を含む |
| `render.hpp/.cpp` | 176 / 5050 | 3D 描画（天体シェーダー、物理ベース大気、星空、天の川、機体、発射台）。テクスチャを使う材質（3D プリントのレゴリス、布地、作物、六角形の舗装）を追加 |
| `render_base.inc` | 805 | 基地の建物の 3D モデルと生成テクスチャ（`render.cpp` に取り込まれる。6.4 参照） |
| `ui.hpp/.cpp` | 121 / 1355 | 即時モード UI（ボタン、入力欄、スクロール、ツールチップ、ベクターアイコン。基地アイコン `Icon::Base`）。`text`（翻訳あり）と `textRaw`（翻訳なし）。拡大率は DPI × GUI のサイズ（`setUserScale`）で、レイアウトの最小サイズ（100 % 以下は 1280×740、拡大時は 1100×680）に収まるよう制限 |
| `nav.hpp/.cpp` | 61 / 406 | Fusion 360 風の視点操作（`OrbitCam`、ViewCube） |
| `keys.hpp/.cpp` | 72 / 375 | キー割り当ての変更（`keys.txt`）。全画面切り替え `A::Fullscreen`（既定 F11）を追加 |
| `tutorial.hpp/.cpp` | 34 / 1085 | チュートリアル 8 種 |
| `debugview.hpp/.cpp` | 9 / 324 | F3 のデバッグ画面 |
| `icon_png.hpp` | 359 | アイコン画像のバイト列 |

### 5.6 `main.cpp` と `rl.hpp`

- `main.cpp`（約 390 行）：コンポジションルートです。旧データの移行、ウィンドウの作成、ポートの差し込み、メインループ、フレームレートの調整、FPS 表示、終了処理を担当します。
- `rl.hpp`：外側の層が raylib を使うための唯一の入口です。raylib が定義する float の `PI` マクロを取り消し、`toF`、`toMatrix` を提供します。

### 5.7 tests（自動テスト）

`tests/autotest.cpp`（約 6700 行）は、アプリを起動したまま決められた操作を自動で行い、ログとスクリーンショットを残します。

```
Astraum.exe --autotest <出力フォルダ> --<テスト名> [オプション]
```

| テスト | 内容 |
|---|---|
| （指定なし） | 既定の総合テスト（設定画面、クレジット画面、組み込み設計の打ち上げなど） |
| `--basetest` | 地表基地（建設条件、火星での建設、発射台、基地からの打ち上げと上昇、保存と読み込み、旧形式のセーブ、F11 の全画面切り替え） |
| `--missiontest` | ミッション支援（月：打ち上げウィンドウ→出発噴射→遭遇→捕獲→周回、火星：遷移ウィンドウ→出発→軌道修正→遭遇）、太陽周回時の遭遇予測とマップの速さ、探査状況の判定と旧セーブの修復、GUI のサイズ |
| `--uitest` | UI 全般 |
| `--achtest` `--airtest` `--bgmtest` `--bodytest` `--cadonly` `--ctrltest` `--designtest` `--docktest` `--duptest` `--labeltest` `--navtest` `--orbitall` `--orbitlinetest` `--perftest` `--picktest` `--realtest` `--rewindtest` `--rovertest` `--savetest` `--seatest` `--skytest` `--solartest` `--spaceplanetest` `--startest` `--terraintest` `--trusstest` `--tutflight` `--warptest` `--tliexp` | 各機能のテスト（前版から継続） |

共通オプション：`--lang`、`--pbr`、`--textsize`、`--terrainq`、`--epoch`、`--nomaps`、`--nobake`、`--paced`、`--fpslimit`、`--crashtest`、`--sfxdump`

環境変数：`SF_SPDIAG`、`SF_BODIES`、`SF_BODYALT`、`SF_PBRDBG`、`SF_SKIPMAP`（開発用。旧名称の接頭辞のまま）

テスト中は設定ファイルを書き換えず、全画面も無効にします。

---

## 6. 地表基地

### 6.1 建設できる天体

固体の地表を持つ天体（月、火星、フォボス、ダイモス、ケレス、ガリレオ衛星、タイタン、冥王星など）に建設できます。

建設できない天体と、その理由は次のとおりです。

| 天体 | 理由 |
|---|---|
| 恒星 | 建設できない |
| 巨大ガス惑星 | 地面がない |
| 彗星 | 核が小さく、活動が激しい |
| 地球 | 既存の発射場がある |
| 金星 | 地表が高温・高圧 |

### 6.2 建設の手順

1. 基地モジュール（工場の「基地」分類）を積んだ着陸機を、目的の天体に着陸させます。
2. 機体が静止すると、HUD に「基地を建設」ボタンが出ます。新しい基地には居住モジュールが必要です。
3. 確認ダイアログで基地名を付けて建設します。
   - モジュールは機体から降ろされ、基地の周りに配置されます。
   - 着陸機の残りは着陸した場所に残ります。
   - 既存の基地から 3 km 以内なら、その基地にモジュールを追加します。
4. 着陸機に積んだ核分裂炉と太陽電池も電源として設置されます。太陽電池の出力は恒星からの距離で決まります。

### 6.3 発射台ができる条件

次の 3 つがそろうと、発射台と推進剤タンク 4 基が自動で建設されます。

- 居住モジュール 1 つ以上
- ISRU 推進剤プラント 1 つ以上
- 電力 40 kW 以上

発射台のある基地は、工場・CAD・設定画面の発射地点に並びます。打ち上げるロケットの燃料は、ISRU で製造したものという扱いです。

地球以外の天体からの打ち上げや展開は、その天体に基地がある場合だけです。発射台がなくても、基地があればローバーや航空機をその天体に展開できます（CAD の「開始地点」。基地の北側に置かれます）。

### 6.4 建物の外観

建物の種類は 8 つです。主な建物には外観が 3 通りずつあり、基地と配置場所ごとに選ばれるので、同じ見た目の基地はできません。

| 種類 | 外観の例 |
|---|---|
| 居住モジュール | 3D プリントのレゴリスドーム、脚付きの膨張式モジュール、先細りの塔 |
| 温室 | ガラスのトンネル、ジオデシックドーム、縦型農場 |
| ISRU プラント | 氷の採掘とサバティエ反応装置、電解施設と掘削車、酸素抽出装置の群れ |
| 推進剤タンク | 球形、横置き、縦置き |
| 核分裂発電所 | 土嚢で囲った炉と放熱板、Kilopower 型 |
| 太陽電池 | 傾斜アレイ、垂直タワー（VSAT 型）、地面に広げる膜型 |
| 通信塔 | 格子塔とパラボラアンテナ |
| 発射台 | 焼結舗装の円形パッド、防爆土手、発射架台、整備塔、照明塔、基地への道路 |

このほか、居住モジュールと温室の間は与圧通路でつながれ、ローバーが駐車されています。夜は窓、育成灯、航空障害灯が光ります。

テクスチャは起動時にプログラムが生成します（3D プリントの積層、膨張式モジュールの布地とベルト、育成灯の下の作物、六角形の舗装）。

### 6.5 保存

基地は `.sfs` に `base` 行と `bmod` 行として保存します（7 章）。巻き戻しのスナップショットにも基地の状態が含まれるため、巻き戻すと建設前の状態に戻ります。

---

## 7. データファイルと保存形式

データフォルダは `%APPDATA%\Astraum` です。初回起動時に旧フォルダ `%APPDATA%\StellarForge` の中身をコピーします。

| パス | 先頭行 | 内容 |
|---|---|---|
| `maps\map_NNNN.sfs` | `ASTRAUM_SAVE 3`（旧 `STELLARFORGE_SAVE 2` も読み込み可） | ゲーム世界（時刻、機体、実績、基地） |
| `maps\*.png` | – | マップのサムネイル |
| `designs\*.sfd` | `ASTRAUM_DESIGN 1` | 工場の設計 |
| `designs\*.sfc` | `ASTRAUM_CRAFT 1` | CAD の機体 |
| `last_design.sfd` | 同上 | 最後に編集した設計 |
| `prefs.txt` | – | 設定（`キー 値` の行） |
| `keys.txt` | – | キー割り当て |
| `crash\*.txt` | – | クラッシュレポート |

**`.sfs` に追加した行**

```
settings <燃料無限> <ハイパースペース> <速度> <地球の発射場> <発射基地 ID (0 = 地球)>
nextbase <次の基地 ID>
base <ID> <天体番号> <緯度 rad> <経度 rad> <建設時刻> <名前>
bmod <種類> <東 m> <北 m> <向き rad> <kW> <外観>
```

読み込み時には、すべての値の範囲と数値の有限性を確認します（基地は最大 64、モジュールは基地ごとに最大 160）。

**`prefs.txt` のキー**：`version`、`lang`、`textsize`、`uisize`、`reversezoom`、`sfxpct`、`music`、`musicpct`、`terrainq`、`detailrange`、`pbr`、`autosave`、`fpslimit`、`showfps`、`adaptive`、`belts`、`fullscreen`、`tutorials2`

スクリーンショットは `ピクチャ\Astraum\` に保存されます。

---

## 8. 開発用ツール（`tools/`）

| スクリプト | 役割 |
|---|---|
| `make_maps.py` | 地球、月、火星の色地図と高度地図 |
| `make_detail_maps.py` | 高解像度タイル（`.pak`） |
| `make_body_maps.py` | その他の天体の色地図（USGS、JPL） |
| `make_sky.py` | 恒星カタログと天の川 |
| `make_icon.py` | アイコン |
| `make_notices.py` | `THIRD_PARTY_NOTICES.txt`（データのクレジットと、ライブラリのライセンス原文） |
| `resolve_crash.py` | クラッシュレポートのアドレスを関数名に変換（`Astraum.map` を使用） |

---

## 9. セキュリティ上の性質

- 実行時にネットワーク通信は行いません。
- 読み込むのは利用者自身のデータフォルダにあるファイルだけです。
  - サイズの上限を確認してから読み込みます（セーブ 256 MB、設計 1 MB、機体 4 MB）。
  - 値の範囲を確認します。
  - 保存は一時ファイルからの置き換えで行います。
- ファイル名は無害化してから使います。データフォルダの外には書き込みません。
- 旧データの移行はコピーのみで、元のフォルダの移動や削除は行いません。
- 金銭に関わる処理はありません。
- クラッシュレポートはローカルに保存するだけで、送信しません。
- 同梱データとライブラリのクレジットとライセンス文は `THIRD_PARTY_NOTICES.txt` にあり、設定画面の「クレジットとライセンス」からも読めます。
