"""Regenerate the Japanese summary from an actually executed simulation CSV."""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    source = root / "docs" / "offline_simulation_results.csv"
    rows = list(csv.DictReader(source.open(encoding="utf-8-sig", newline="")))
    if len(rows) != 128:
        raise ValueError(f"Expected 128 executed cases, found {len(rows)}")
    grouped: dict[tuple[str, str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row["schedule"] == "0":
            grouped[(row["plant"], row["hz"], row["one_way_delay_ms"])].append(row)
    lines = [
        "# CyberGear オフライン数値試験の実行結果",
        "",
        "2026-09-09、Windows ホスト GCC 15.2.0 / C11 / `-O2 -Wall -Wextra -Wpedantic -Werror -fno-fast-math` で実行。実機、CAN、デバッガ、書込み器へはアクセスしていない。",
        "",
        "**合成モデル 128 条件のうち、54 条件で報告用しきい値を超える追従崩壊・残留運動を確認した。100 Hz から 200 Hz への変更だけでは解消しない。** この試験は実機で使えるゲイン・速度・電流・遅延の保証範囲を決めるものではない。数値を満たした 74 条件も実機合格ではない。",
        "",
        "コントローラ単体の感度試験のため、ファームウェア側の可動域・速度・追従誤差・停止状態機械はここでは意図的に動作させていない。発散条件の大きい角度は停止保護を外した合成方程式の出力であり、実機にそのまま許す値ではない。プロトコルの ±12.5 rad 範囲によるクリップも加えていない。保護経路は別のドライバ試験で扱う。",
        "",
        "## 実行した条件",
        "",
        "- 通常設定：`wc=4 rad/s, zeta=2, wo=10 rad/s, b0=10 rad/s²/A`、誤差依存リーク 0.5→0.03 /s。電流 ±10 A、正負スルー 5 A/s、外乱補償 ±0.5 A / 1 A/s、100 ms 待機後 500 ms で gamma を 1 へ上げる。これは `cybergear_controller_default_config()` の合成試験設定で、実機許容値ではない。",
        "- 制御周期 100 / 200 Hz、基準入力ゲイン比 `b/b0=0.25, 0.5, 1, 2, 4`、**指令と測定それぞれ**の遅延 0 / 5 / 10 / 20 ms。例えば表の 20 ms は往復の輸送遅延だけで 40 ms。制御サンプル保持の遅れがさらにある。",
        "- 測定は 25/65535 ≈0.0003815 rad の量子化と、一様雑音 ±0.0002 rad。乱数は固定 seed `0x431279af` の LCG。受信タイムスタンプは到着時刻で、古い物理状態を現在時刻へ近似配置する現行設計の限界を含む。",
        "- 参照は停止端点の五次式：0→1.2 rad を 2 s、1 s 保持、1.2→−0.8 rad を 2.5 s、1 s 保持、−0.8→0.3 rad を 2 s、3.5 s 保持。全長 12 s。プラント積分は 1 ms。",
        "- 剛体：正規化トルク定数 Ktau=1、J=0.1/ratio、摩擦トルク `0.01*tanh(v/0.02)+0.005*v`。これは機体から同定した物理値ではない。",
        "- 二慣性：Jmotor:Jload=1:4、総 J=0.1/ratio、相対モードの角周波数 15 rad/s、モード減衰比 0.04。ばね・ダンパはこの条件から設定。制御器はモーター位置だけを観測し、負荷位置は評価にのみ使用する。",
        "- 可変慣性：`J(t)=(0.1/ratio)*(1+0.55*sin(0.7*t))`。`J*qdd + Jdot*qd = iq - friction` を解き、単なる入力ゲイン切替ではなく Jdot 項を含む滑らかな物理遷移とする。",
        "- スケジュール比較は可変慣性・ratio=1 の追加 8 条件。正規化姿勢 `s=sin(0.7*t)` と `b=10/(1+0.55*s)` の合成校正表 8 点を使用し、b0 変化率を 10 [rad/s²/A/s] に制限。実機の姿勢変換やリンク寸法の推測ではない。",
        "",
        "## 全条件で通った不変条件",
        "",
        "制御計算・プラント状態は有限、送信要求は ±10 A 以内、1 制御周期の電流変化は 5 A/s 以内、外乱補償は ±0.5 A 以内だった。これらはプログラム中の `assert` で検査する。**電流制限を守っていても振動・追従崩壊は発生する。**",
        "",
        "合成系の良否はテスト成功/失敗と分けて記録する。`poor` は最大負荷位置誤差 >0.6 rad、終端 1 秒の負荷位置誤差 RMS >0.1 rad、または終端負荷速度 RMS >0.2 rad/s のいずれか。この緩いしきい値は大きい崩壊を見つける報告用で、機体の上位完了条件・可動域・精度設定ではない。",
        "",
        "## 比ごとの全データを集約した結果",
        "",
        "各行は同じプラント・周期・遅延の 5 ゲイン比を集約。最大誤差と終端速度 RMS はそれぞれ別条件の最大となる場合がある。詳細は [128 条件の CSV](offline_simulation_results.csv)、再現実装は [controller_simulation.c](../tests/controller_simulation.c) にある。",
        "",
        "| 合成プラント | Hz | 片道遅延 ms | 最大負荷誤差 rad | 終端負荷速度 RMS 最大 rad/s | poor / 5 | poor の基準ゲイン比 |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for (plant, hz, delay), group in grouped.items():
        bad = [row for row in group if row["result"] == "poor"]
        max_error = max(float(row["max_load_error_rad"]) for row in group)
        max_velocity = max(float(row["tail_load_velocity_rms_rad_s"]) for row in group)
        ratios = ", ".join(row["nominal_ratio"] for row in bad) or "—"
        lines.append(f"| {plant} | {hz} | {delay} | {max_error:.4f} | {max_velocity:.4f} | {len(bad)} | {ratios} |")
    lines += [
        "",
        "剛体の poor は 8/40 条件。基準比 4 では片道 5 ms 以上、基準比 2 では 20 ms で発生した。二慣性は 36/40、固定 b0 の可変慣性は 10/40 条件だった。低い構造共振とモデルずれへの余裕を、単一姿勢の剛体試験から推定できないことが分かる。",
        "",
        "## 可変慣性に対する校正表の比較",
        "",
        "全行 ratio=1。表は最大負荷位置誤差で、小さいほどよい。この合成校正表は誤差を減らしたが、干渉トルクや遅延を除去したわけではない。",
        "",
        "| Hz | 片道遅延 ms | 固定 b0 の最大誤差 rad | スケジュールの最大誤差 rad |",
        "|---:|---:|---:|---:|",
    ]
    for hz in ("100", "200"):
        for delay in ("0", "5", "10", "20"):
            pair = [row for row in rows if row["plant"] == "varying_inertia" and
                    row["nominal_ratio"] == "1.00" and row["hz"] == hz and
                    row["one_way_delay_ms"] == delay]
            fixed = next(row for row in pair if row["schedule"] == "0")
            scheduled = next(row for row in pair if row["schedule"] == "1")
            lines.append(f"| {hz} | {delay} | {float(fixed['max_load_error_rad']):.6f} | {float(scheduled['max_load_error_rad']):.6f} |")
    lines += [
        "",
        "## 再現",
        "",
        "通常のホスト試験には 128 条件の実行も含まれる。CSV を更新するときはホスト GCC で以下を実行し、その実行出力から要約を再生成する。Windows では gcc.exe のディレクトリを PATH に追加して使用する。",
        "",
        "```text",
        "gcc -std=c11 -O2 -Wall -Wextra -Wpedantic -Werror -fno-fast-math -ICore/Inc -DCYBERGEAR_SIMULATION_STANDALONE Core/Src/cybergear_controller.c Core/Src/cybergear_dynamics.c tests/controller_simulation.c -lm -o build/controller-simulation.exe",
        "build/controller-simulation.exe > docs/offline_simulation_results.csv",
        "python tests/summarize_controller_simulation.py",
        "```",
        "",
        "この CSV には停止回路、モーター内蔵電流ループのダイナミクス、CAN 仲裁・bus-off、電源回生、接触・バックラッシュ・荷物把持の衝撃は含まれない。実機の b0 範囲・実電流・遅延分布・最小構造共振が分かるまでは、既定 100 Hz と校正モデル無効を維持する判断を支持する結果である。",
        "",
    ]
    (root / "docs" / "offline_simulation_results.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
