"""Exercise real adapter CSV export through the analysis CLI, using fake CAN only."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: check_calibration_log_pipeline.py <calibration_app_test executable>")
    executable = Path(sys.argv[1]).resolve(strict=True)
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="cybergear-synthetic-csv-") as directory:
        capture = Path(directory) / "synthetic.csv"
        output = Path(directory) / "observations.json"
        subprocess.run([str(executable), str(capture)], check=True, capture_output=True, text=True)
        original = capture.read_bytes()
        assert original.count(b"timestamp_ms,feedback_timestamp_ms,") == 2, "expected two real UART exports"
        subprocess.run([sys.executable, str(root / "tools/analyze_cybergear_calibration.py"),
                        str(capture), "--json", str(output)], check=True, capture_output=True, text=True)
        report = json.loads(output.read_text(encoding="utf-8"))
        assert capture.read_bytes() == original, "analysis must not modify the capture"
        assert report["status"] == "unverified_observations" and not report["config_written"]
        assert report["row_count"] > 80 and report["quality_issue_rows"] == {}
        assert report["relative_excursion_observed_rad"]["anchor_source"] == "initial_position_rad"
        assert report["feedback_interval_ms"]["max"] == 10, "manual pause must not count as RX loss"
        assert len(report["windows"]) == 4
        assert all(window["accepted"] for window in report["windows"]), report["windows"]
        pairs = report["input_gain_pairs"]
        assert [pair["trial_id"] for pair in pairs] == ["1", "2"]
        for pair, direction in zip(pairs, (1, -1)):
            assert pair["accepted"] and pair["positive_coordinate_direction"], pair
            assert pair["delta_current_a"] * direction > 0
            # The C trace has a synthetic q acceleration of +/-0.25 rad/s^2
            # for +/-0.05 A, followed by the real 16-bit protocol quantization.
            assert abs(pair["signed_b_rad_s2_per_a"] - 5.0) < 3 * pair["b_fit_standard_error_rad_s2_per_a"], pair
        brakes = [row for row in report["stop_observations"] if row["phase"] == "BRAKE"]
        assert len(brakes) == 2
        assert all(row["stationary_observed"] and not row["guaranteed"] for row in brakes)
        print("Adapter UART CSV -> analysis CLI: two signed synthetic trials accepted; no hardware/config access")


if __name__ == "__main__":
    main()
