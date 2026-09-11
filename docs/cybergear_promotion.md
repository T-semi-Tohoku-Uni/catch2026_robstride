# CyberGearの試験結果を本番へ取り込む手順

## ブランチの役割

| ブランチ | 用途 |
|---|---|
| `test/cybergear-swing-60-5` | パラメーター調整、単体往復試験、UARTコンソール、ホスト試験 |
| `control/cybergear-production` | 確認済みのパラメーター・制御実装とその説明。試験コードを含めない |
| `20_cybergear_fix` | 本番ブランチをマージする通常4軸運転の統合先 |

流れは「試験ブランチで検証 → 必要な制御差分を本番ブランチへ反映 → 20へマージ」。
試験ブランチの丸ごとマージや、試験変更が混在したコミットの丸ごとcherry-pickは行わない。
`main.c` や `cybergear_config.h` もファイル全体を試験版で置き換えず、必要な差分だけを編集する。

初回は `20_cybergear_fix` の `30e630115cc3b58dad66a19fab14cd3b8707b02d` を基点とし、
試験コミット `1760d43e008fe0dfa589e8733b10e2f285bf8d82` の実効設定と制御修正を取り込んだ。
基点にあった `tests/`、単体試験の実装・設定・ビルド切替も削除した。
試験コードと手順は試験ブランチに残し、過去コミットの履歴は書き換えない。

## 1. 試験結果と比較元を確定する

元の作業ディレクトリは `test/cybergear-swing-60-5` のまま使用する。
変更後にホスト試験・実機試験を行い、試験条件と結果をコミットする。
UARTの `set` はRAM内だけの変更なので、採用値をヘッダーへ保存し、再ビルド・確認してから確定する。
取込み対象と前回の出典を固定する。以下は同じシェルで実行する。

```sh
source_test_commit=$(git rev-parse test/cybergear-swing-60-5)
previous_test_commit=$(git log control/cybergear-production --first-parent --grep='^Source-Test-Commit:' -1 --format='%(trailers:key=Source-Test-Commit,valueonly)')
git merge-base --is-ancestor "$previous_test_commit" "$source_test_commit"
git diff --stat "$previous_test_commit" "$source_test_commit"
```

出典が空、または祖先確認が失敗する場合は、試験履歴の変更理由と比較元を確認してから進める。
抽出対象は「前回取り込んだ試験コミット→今回の試験コミット」の差分とする。

## 2. 本番worktreeで必要な差分を反映する

本番ブランチは独立したworktreeで開く。初回に作成した `tmp/cybergear-production` があれば再利用する。
`git worktree list` で使用場所を確認し、本番と試験の作業ディレクトリを取り違えない。

```sh
git diff "$previous_test_commit" "$source_test_commit" -- Core/Inc/cybergear_config.h Core/Inc/cybergear.h Core/Src/cybergear.c Core/Inc/cybergear_controller.h Core/Src/cybergear_controller.c Core/Inc/cybergear_dynamics.h Core/Src/cybergear_dynamics.c Core/Inc/cybergear_trajectory.h Core/Src/cybergear_trajectory.c Core/Src/main.c
```

差分を読み、本番worktreeでパラメーターと制御処理だけを反映する。
上記以外の制御変更や依存ファイルも全体差分から確認する。
汎用制御ライブラリの単体既定値と `cybergear_config_defaults()` の実機設定を混同しない。

| 試験の実効値 | 本番で設定する項目 | 初回採用値 |
|---|---|---:|
| `min(CYBERGEAR_CURRENT_LIMIT_A, CG_TEST_CURRENT_LIMIT_A)` | `CYBERGEAR_CURRENT_LIMIT_A` | 5 A |
| `min(CYBERGEAR_TRAJECTORY_SPEED_RAD_S, CG_TEST_SPEED_RAD_S)` | `CYBERGEAR_TRAJECTORY_SPEED_RAD_S` | 0.8 rad/s |
| `min(CYBERGEAR_OBSERVER_RAD_S, CG_TEST_OBSERVER_RAD_S)` | `CYBERGEAR_OBSERVER_RAD_S` | 6 rad/s |
| 実効電流上限 × `CG_TEST_DISTURBANCE_CURRENT_FRACTION` | `CYBERGEAR_DISTURBANCE_LIMIT_A` | 2.5 A |
| `CG_TEST_RESERVE_CURRENT_FRACTION` | `CYBERGEAR_DYNAMICS_RESERVE_CURRENT_FRACTION` | 0.5（2.5 A） |
| `CG_TEST_LEAK_MODE` | `CYBERGEAR_LEAK_MODE` | `CYBERGEAR_LEAK_FIXED` |

帯域・減衰比・b0・軌道上限・電流変化率・復帰設定などの共通マクロも確認する。
振幅・到達判定・待機時間・区間タイムアウトは単体試験の条件なので本番へ移さない。
本番の実効値と復帰動作は `docs/controller_parameters.md` に記録する。

## 3. 検証し、出典付きでコミットする

本番worktreeで通常4軸のDebug/Releaseをビルドし、差分とビルド対象を点検する。
`tests/`、単体往復試験、UART調整コンソール、試験モード切替が再混入していないことを確認する。
ホスト試験・Flash配置検証は試験ブランチのスクリプトを使い、本番のソースやELFを対象にする。
試験コードを本番へコミットしない。実機確認の範囲と未確認の範囲は分けて記録する。
コミット本文の末尾に、今回確認した試験コミットの完全なハッシュを残す。

```text
Source-Test-Branch: test/cybergear-swing-60-5
Source-Test-Commit: <今回のsource_test_commitの完全なハッシュ>
```

この記録は試験コミット全体の取込みを意味しない。
出典を進める前に、そのコミットまでの制御変更を採用・対象外に整理し、対象外の理由も本文に残す。

## 4. `20_cybergear_fix` へマージする

統合先の作業ディレクトリがクリーンな状態で実行する。
本番worktreeを一時的に20へ切り替えてマージする場合は、完了後に本番ブランチへ戻す。

```sh
git switch 20_cybergear_fix
git merge --no-ff control/cybergear-production
git switch control/cybergear-production
```

20が別worktreeで使用中なら、そのworktreeでマージする。
競合は通常4軸運転の統合内容を保って解消し、マージ結果をビルド・確認する。
次回も同じ本番ブランチへ、前回の `Source-Test-Commit` 以降の差分を取り込む。
リモートへの反映が必要な場合は、検証済みの抽出用ブランチと統合先ブランチをpushする。
