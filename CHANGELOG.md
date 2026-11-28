# Changelog

Astraum のアップデート履歴です。バージョン番号はセマンティックバージョニングに従います。

## v1.2.1 (2026-11-28)

既存機能の重大なクラッシュ・データ保護・文字コード互換性・GPU リソース管理・ビルド構成・シミュレーション上の不具合修正を中心とする安定性向上アップデートのため、パッチバージョン `v1.2.1` としました。

### 起動・文字コード・ファイル保存の修正
- **非 ASCII Windows ユーザー名対応（UTF-8 完全統一）**:
  - `src/app.manifest`（`<activeCodePage>UTF-8</activeCodePage>`）を追加して `src/app.rc` に埋め込み。
  - `SHGetKnownFolderPath(FOLDERID_RoamingAppData)` による Unicode 安全な `%APPDATA%` 取得（`winshell::appDataDir` / `ports::appDataDir`）と、`std::u8string` 経由の `pathFromUtf8()` / `pathToUtf8()` へ統一し、日本語等のユーザー名環境での起動時クラッシュと文字化けを解消。
  - クラッシュハンドラ（`crash.cpp`）のファイル・モジュール操作を Unicode (`*W`) API 化。
  - `migrateDataFolder()` をステージングフォルダ経由のアトミック移行に変更し、旧フォルダの `crash/`（`pending.txt` 含む）を移行対象から除外。
- **セーブ・設計データ保護**:
  - `session.cpp` の重複 `writeAtomic()`（`std::error_code` 再利用により保存失敗時に `true` を返すバグ）を削除し、`storage.cpp` の `writeFileAtomic()` へ統一。
  - 設計ファイル（`.sfd` / `.sfc`）、設定ファイル（`prefs.txt`）、キー割り当て（`keys.txt`）をすべてアトミック書き込み化。`loadPrefs()` も行単位パースにして 1 行の破損で以降の設定が失われないように修正。
  - バックグラウンド保存の失敗時に未保存状態（dirty）を維持するように修正し、`listMaps()` 実行前に `finishSaves()` を呼んでファイル共有違反競合を防止。
  - `mapDirty()` が基地増減・射場/設定変更・マップ名変更も検知するよう補強。
  - `renameMap()` にサイズ上限（256 MB）と所有者パス検証を追加、`deleteMap()` のパス検証を正規化パス＋ファイル名規則で厳密化し、プレイ中マップ削除時のプレゼンテーション通知漏れを修正。
  - `World::deserialize()` の行数上限を 400,000 行から 4,000,000 行へ引き上げ、大規模セーブの読み込み不能を解消。`v.spool` と `dockHoldUntil` の保存・復元、重複/非正 ID・ゼロベクトル・過大座標の検証、`nextId` の既存最大 ID 以上の保証を追加。

### クラッシュ・セキュリティ・数値安定性の修正
- **`trf()` 無限ループ修正**: 1 パス走査に書き換え、マップ名や機体名に `{0}` 等が含まれても無限ループや二重展開が起きないように修正。
- **空力・形状・軌道計算の防御強化**:
  - `craftFromText()` で非円錐形状の `dims.x / dims.y == 0` を防止し、`AeroTable::lookup()` で `NaN` 入力・負インデックスによる範囲外アクセスを防止。空サンプル時の `AeroTable::length` 異常値とキャッシュ重複登録も修正。
  - `orbit.cpp` の放物線・近放物線・動径軌道でのゼロ除算／オーバーフローを修正（バーカーの方程式対応）、`keplerPropagate()` で位置・速度の全 3 成分の有限性を検査。
  - `designFromText()` で寸法の `NaN` / 非有限値を拒否。
  - `fpsOverlay()` の `snprintf` 切り詰め時のバッファ残量アンダーフローを防止。
  - `formatTime()` / `formatDate()` の過大浮動小数点数キャストによる未定義動作を防止。

### 描画・GPU リソース・システム基盤の修正
- **OpenGL 3.3 Core 準拠・GPU リーク解消**:
  - シャドウマップの深度専用 FBO 作成時に `glDrawBuffer(GL_NONE)` / `glReadBuffer(GL_NONE)` を設定。
  - `trussCache` に上限（64 件）を追加し、`trimCaches(true)` および `gfx::shutdown()` / `flightShutdown()` で全メッシュキャッシュ・基地テクスチャ・大気透過 LUT・天の川・シャドウマップ・PBR/暗視シェーダー＆FBO を確実に解放。
  - PBR シェーダー一部コンパイル失敗時のリークを修正し、`sunTintAt()` が散乱係数 `bM` ではなく消散係数 `bMe` を使用するように修正（火星・タイタン等の太陽光色修正）。
  - 環メッシュを天体別キャッシュ化し、複数環天体での内外径比使い回しを解消。
  - 温室モジュールを不透明基地モジュールの後に描画するよう 2 パス化し、半透明ガラスによる背後パーツの深度クリッピングを軽減。シャドウパス中の背面カリング途中復帰も修正。
  - 暗視モードの 0×0 リサイズ/FBO 失敗ガードを追加、毎フレームの `GenTextureMipmaps` による `TRACELOG` スパムを抑制、タイトル画面復帰時の `g_nightLift` 残留を解消。
- **マルチスレッド・診断基盤の堅牢化**:
  - `terrain.cpp` に `terrain::shutdown()` を追加し、`main.cpp` の終了処理で明示停止。ウィンドウ移動・破棄時に旧世代ジョブを即時破棄・スキップし、中央 3×3 高さタイルの二重デコードを解消。
  - `screenshot.cpp` を予約済みファイル名管理付きの常駐キューワーカーへ変更し、同一秒連打時の TOCTOU 競合と `detach()` 終了時クラッシュリスクを解消。
  - `crash.cpp` から非推奨の `IsBadReadPtr` を撤去して `VirtualQuery` 化、同時クラッシュ時のレポート欠損防止、リングバッファの符号なし添字化、`g_state` の同期を追加。
  - `gputime.cpp` で `wglGetProcAddress` のエラーセンチネル値検証、`reset()` 後の旧フレーム混入修正、符号なしラップ防止、`gputime::shutdown()` によるクエリ解放を追加。
  - `winshell.cpp` に COM STA 初期化を追加、`audio.cpp` の `shutdown()` リセットと `play()` 有限性検証、`gpumon.cpp` の RAII 停止ガードとサーマルゾーン名 UTF-8 変換を追加。

### シミュレーション・UI・CAD の修正
- **シミュレーション**:
  - 着陸・無推力状態でもバッテリー充放電・冷却・テレメトリ更新が継続するように修正。
  - アンドック・ペイロード放出での `vessels.capacity()` 固定制限による恒久失敗を解消し、電力の消失・複製・太陽電池強制収納・角運動量未更新・分離側命名の不整合を修正。
  - `World::rewind()` で `dockHoldUntil` を復元し、巻き戻し・新規ゲーム・ロード時に `mission::invalidate()` / `navg::invalidate()` を呼び出すよう修正（終了時は `navg::shutdown()` で安全待機）。
  - 基地モジュール上限付近での発射台欠落偽装を修正、極付近の基地からの機体展開姿勢を修正、`analyzeCraft()` の同ステージ複数タンク残量計算バグを修正、`unloadComponents()` の収納脚無視・レール進行時の `starFlux` 時刻ずれ・`groundRoll` ステップ幅ずれを修正。
- **UI / CAD / 多言語**:
  - `vesselsModal` で機体削除時に破壊済み機体へ誤切替されるバグを修正。
  - `numberField` / `slider` のドラッグ中コントロール消失による操作ロックアウトを解消。
  - `factory.cpp` の文字化けリテラル `"Â°"` を `"°"` に修正、Duplicate の 60 部品上限追加、統計領域の負の高さガード、設計読込/新規作成時の `multiSel` クリアを追加。
  - `cad.cpp` の形状選択に `Truss` チップを追加。
  - `ui::fitRaw()` / `ui::measureRaw()` を追加し、ユーザー入力名・マップ名・機体名・クラッシュレポートの誤翻訳を防止。UTF-8 文字境界での切り詰めと `ensureGlyphs()` 呼び出しを補強。
  - チュートリアル一覧モーダルを画面高さに合わせてスクロール対応化、Flight 画面の `bodyInfoPanel` / `layOrbit` / `layNavball` の重複解消、設定画面の Esc 競合・`nav::viewCube` Home ボタンの背面入力貫通・リネーム中マップ削除時の状態残留・クイックセーブ失敗トースト色を修正。
  - タイトル画面の太陽系天体数表記を実数（30 天体）に更新。

### リポジトリ・ビルド・開発ツールの修正
- **リポジトリ整理**: ルート直下に誤配置されていた 5 つのマップアセットを `assets/maps/` へ移動・追跡し、`.gitignore` をコミット対象に追加。
- **ビルド構成（`CMakeLists.txt`）**:
  - `-DSF_NOSTRIP=1` 指定時に `-g` が `windres` へ渡されてビルド失敗する問題を修正。
  - `Debug` ビルド時の `-O2` / `-s` 強制を解除し、アセット GLOB に `CONFIGURE_DEPENDS` を追加。
  - リリース時にテストコードと開発用 CLI（`--autotest` / `--sfxdump` / `--crashtest`）を除外できる `ASTRAUM_ENABLE_TESTS` オプションを追加。
  - 自動テスト実行時にログの `FAIL` に応じて終了コード `1` を返す `autotestExitCode()` を実装。
- **開発ツール（`tools/`）**:
  - `resolve_crash.py`: 空白を含むデマングル済み C++ シンボルと `__image_base__` を正しく解析するよう修正。
  - `make_icon.py`: 共有 `%TEMP%` の固定一時ファイル作成を廃止してインメモリ化し、HTML 属性をエスケープ。
  - `make_detail_maps.py`: `draft()` 呼び出し順序を修正し、`make_maps.py` / `make_body_maps.py` / `make_sky.py` / `make_notices.py` の NaN・境界・表示バグを修正。

---

## v1.2.0 (2026-11-27)
- 目標機体への打ち上げウィンドウ計算と SAS 上昇誘導を追加。
- マップ画面での暗視モード対応。
- 物理ベースシェーディング時のリアルな影（シャドウマップ＋PCSS）を追加。

## v1.1.3 (2026-11-26)
- ランデブー目標機体が見えないところで軌道を変えてしまう不具合を修正。

## v1.1.2 (2026-11-25)
- 高倍率タイムワープでの地球脱出軌道が予測軌道と一致するように修正（普遍変数ケプラー方程式の区間付き解法）。

## v1.1.1 (2026-11-24)
- 工場画面の表示／非表示ボタンを追加、地球外の開始天体には基地建設を必要とするよう変更。

## v1.1.0 (2026-11-23)
- ミッション支援、GUI サイズ設定、マップ描画の高速化、探査状況の判定修正。
