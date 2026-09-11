# CyberGear の変更と取り込み

- 制御・パラメーターの変更は、まず `test/cybergear-swing-60-5` で実装して試験する。
- 試験済みのパラメーターと制御だけを `control/cybergear-production` に抽出し、検証後に `20_cybergear_fix` へマージする。
- 試験ブランチ全体を抽出用ブランチや `20_cybergear_fix` にマージしない。`tests/`、単体往復、UART調整コンソール、試験用モード分岐は試験ブランチに置く。
- `CG_TEST_*` による上書きも含めた実効値を通常運転用の `CYBERGEAR_*` 設定へ反映する。UARTだけで変更した値は、先に試験ブランチのヘッダーへ保存する。
- 抽出コミットに `Source-Test-Commit: <試験済みコミットの完全SHA>` を記録し、次回はそのコミット以降の差分を確認する。
- 詳細は `docs/cybergear_promotion.md` を参照する。
