"""Synthetic-only regression tests for calibration log analysis (no motor I/O)."""
from __future__ import annotations

import csv
import contextlib
from dataclasses import replace
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
SPEC = importlib.util.spec_from_file_location("calibration_analysis", ROOT / "tools/analyze_cybergear_calibration.py")
analysis = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analysis
SPEC.loader.exec_module(analysis)


def records(gain=10.0, current=0.2, gravity=0.0, quantize=False):
    result = []
    resolution = 25 / 65535
    for tick in range(0, 410, 10):
        t = tick / 1000
        pulse_t = max(0.0, t - 0.2)
        q = 0.5 * gravity * t * t + 0.5 * gain * current * pulse_t * pulse_t
        if quantize:
            q = round(q / resolution) * resolution
        result.append({"timestamp_ms": tick, "feedback_timestamp_ms": tick,
                       "rx_sequence": tick // 10 + 1, "trial_id": "1",
                       "phase": "BASELINE" if tick < 200 else "PULSE",
                       "position_rad": q, "velocity_rad_s": gravity * t + gain * current * pulse_t,
                       "command_current_a": 0 if tick < 200 else current,
                       "fault": 0, "saturated": 0, "dropped": 0, "tx_failed": 0})
    return result


class CalibrationAnalysisTests(unittest.TestCase):
    def analyse(self, raw=None, options=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(raw[0] if raw else records()[0]))
                writer.writeheader()
                writer.writerows(records() if raw is None else raw)
            return analysis.analyse(analysis.load_csv(path), options)

    def test_exact_known_gain_and_json_are_finite(self):
        result = self.analyse()
        pair = result["input_gain_pairs"][0]
        self.assertTrue(pair["accepted"], pair)
        self.assertAlmostEqual(pair["signed_b_rad_s2_per_a"], 10, places=8)
        self.assertGreater(pair["b_fit_standard_error_rad_s2_per_a"], 0)
        self.assertEqual(result["status"], "unverified_observations")
        self.assertFalse(result["config_written"])
        json.dumps(result, allow_nan=False)

    def test_quantization_is_not_erased_and_estimate_remains_near_model(self):
        pair = self.analyse(records(quantize=True))["input_gain_pairs"][0]
        self.assertTrue(pair["accepted"], pair)
        self.assertAlmostEqual(pair["signed_b_rad_s2_per_a"], 10, delta=0.5)
        self.assertGreater(pair["b_fit_standard_error_rad_s2_per_a"], 0)

    def test_constant_gravity_acceleration_is_cancelled_by_baseline(self):
        result = self.analyse(records(gravity=0.15), replace(analysis.Options(), max_pair_velocity_rad_s=0.4))
        pair = result["input_gain_pairs"][0]
        self.assertTrue(pair["accepted"], pair)
        self.assertAlmostEqual(pair["signed_b_rad_s2_per_a"], 10, places=8)

    def test_negative_coordinate_gain_is_preserved(self):
        report = self.analyse(records(gain=-10))
        pair = report["input_gain_pairs"][0]
        self.assertTrue(pair["accepted"], pair)
        self.assertAlmostEqual(pair["signed_b_rad_s2_per_a"], -10, places=8)
        self.assertFalse(pair["positive_coordinate_direction"])
        self.assertIn("do not take absolute value", analysis.format_summary(report))

    def test_negative_pulse_current_is_identified(self):
        pair = self.analyse(records(current=-0.2))["input_gain_pairs"][0]
        self.assertTrue(pair["accepted"], pair)
        self.assertAlmostEqual(pair["signed_b_rad_s2_per_a"], 10, places=8)

    def test_low_excitation_and_zero_acceleration_are_rejected(self):
        for raw, reason in ((records(current=0.001), "insufficient_current_excitation"),
                            (records(gain=0), "insufficient_acceleration_signal")):
            with self.subTest(reason=reason):
                pair = self.analyse(raw)["input_gain_pairs"][0]
                self.assertFalse(pair["accepted"])
                self.assertIn(reason, pair["reasons"])

    def test_quality_faults_reject_whole_plateau(self):
        cases = (("fault", 2, "fault"), ("saturated", 1, "saturated"),
                 ("dropped", 1, "dropped_changed"), ("tx_failed", 1, "tx_failed_changed"),
                 ("rx_sequence", 25, "duplicate_feedback"),
                 ("feedback_timestamp_ms", 200, "stale_feedback"))
        for field, value, reason in cases:
            with self.subTest(field=field):
                raw = records()
                raw[25][field] = value
                result = self.analyse(raw)
                self.assertFalse(result["input_gain_pairs"][0]["accepted"])
                self.assertIn(reason, result["windows"][1]["reasons"])

    def test_feedback_sequence_gap_is_not_silently_bridged(self):
        raw = records()
        del raw[25]
        result = self.analyse(raw)
        self.assertFalse(result["input_gain_pairs"][0]["accepted"])
        self.assertIn("feedback_sequence_gap", result["windows"][1]["reasons"])

    def test_variable_current_and_bad_quadratic_are_rejected(self):
        for alteration, expected in (("current", "nonconstant_current"), ("position", "poor_quadratic_fit")):
            with self.subTest(alteration=alteration):
                raw = records()
                if alteration == "current":
                    raw[25]["command_current_a"] += 0.03
                else:
                    raw[25]["position_rad"] += 0.1
                result = self.analyse(raw)
                self.assertIn(expected, result["windows"][1]["reasons"])

    def test_insufficient_short_window_is_rejected(self):
        result = self.analyse(records()[:25])
        self.assertIn("insufficient_settled_samples", result["windows"][1]["reasons"])

    def test_different_trials_and_pose_mismatch_are_rejected(self):
        raw = records()
        for row in raw[20:]:
            row["trial_id"] = "2"
        pair = self.analyse(raw)["input_gain_pairs"][0]
        self.assertIn("missing_preceding_trial_baseline", pair["reasons"])
        pair = self.analyse(options=replace(analysis.Options(), max_pair_velocity_rad_s=0.01))["input_gain_pairs"][0]
        self.assertIn("velocity_mismatch", pair["reasons"])

    def test_tick_and_sequence_wrap_remain_monotonic(self):
        raw = records()
        for row in raw:
            row["timestamp_ms"] = (row["timestamp_ms"] + (1 << 32) - 210) % (1 << 32)
            row["feedback_timestamp_ms"] = row["timestamp_ms"]
            row["rx_sequence"] = (row["rx_sequence"] + (1 << 32) - 25) % (1 << 32)
        pair = self.analyse(raw)["input_gain_pairs"][0]
        self.assertTrue(pair["accepted"], pair)
        self.assertAlmostEqual(pair["signed_b_rad_s2_per_a"], 10, places=6)

    def test_nonfinite_and_reordered_log_are_not_accepted(self):
        for field, value in (("position_rad", "nan"), ("timestamp_ms", 5), ("fault", -1)):
            with self.subTest(field=field):
                raw = records()
                raw[10][field] = value
                with self.assertRaises(ValueError):
                    self.analyse(raw)

    def stop_records(self, complete=True, phase="BRAKE"):
        raw = records()
        q0 = raw[-1]["position_rad"]
        for tick in range(410, 810 if complete else 550, 10):
            t = (tick - 410) / 1000
            motion_t = min(t, 0.2)
            raw.append({**raw[-1], "timestamp_ms": tick, "feedback_timestamp_ms": tick,
                        "rx_sequence": raw[-1]["rx_sequence"] + 1,
                        "phase": phase, "position_rad": q0 + 0.4 * motion_t - motion_t * motion_t,
                        "velocity_rad_s": max(0, 0.4 - 2 * t), "command_current_a": -0.2})
        return raw

    def test_braking_observation_needs_stationary_dwell(self):
        stop = self.analyse(self.stop_records())["stop_observations"][0]
        self.assertTrue(stop["accepted"])
        self.assertTrue(stop["stationary_observed"])
        self.assertFalse(stop["guaranteed"])
        self.assertAlmostEqual(stop["observed_mean_deceleration_rad_s2"], 2, places=6)
        self.assertAlmostEqual(stop["observed_forward_distance_rad"], 0.04, delta=0.0002)
        incomplete = self.analyse(self.stop_records(complete=False))["stop_observations"][0]
        self.assertFalse(incomplete["stationary_observed"])
        self.assertIn("stationary_dwell_not_observed_distance_is_incomplete", incomplete["reasons"])

    def test_coast_is_separately_labelled_and_never_guaranteed(self):
        stop = self.analyse(self.stop_records(phase="COAST"))["stop_observations"][0]
        self.assertEqual(stop["phase"], "COAST")
        self.assertFalse(stop["guaranteed"])

    def test_brake_and_coast_are_combined_but_both_are_reported(self):
        raw = self.stop_records()
        for row in raw:
            if row["timestamp_ms"] >= 610:
                row["phase"] = "COAST"
        stop = self.analyse(raw)["stop_observations"][0]
        self.assertTrue(stop["stationary_observed"])
        self.assertEqual(stop["phases_included"], ["BRAKE", "COAST"])

    def test_intervening_fault_segment_does_not_match_old_baseline(self):
        raw = records()
        raw[19]["phase"] = "FAULT"
        raw[19]["fault"] = 1
        pair = self.analyse(raw)["input_gain_pairs"][0]
        self.assertFalse(pair["accepted"])
        self.assertIn("missing_preceding_trial_baseline", pair["reasons"])

    def test_serial_comments_repeated_headers_and_nonzero_initial_anchor(self):
        raw = records()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "serial.csv"
            columns = list(raw[0]) + ["initial_position_rad"]
            with path.open("w", newline="", encoding="utf-8") as stream:
                stream.write("# metadata: synthetic only\n")
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader()
                for index, row in enumerate(raw):
                    if index == 20:
                        stream.write("# next batch\n")
                        writer.writeheader()
                    writer.writerow(dict(row, initial_position_rad=-0.1))
            report = analysis.analyse(analysis.load_csv(path))
        self.assertEqual(report["row_count"], len(raw))
        self.assertTrue(report["input_gain_pairs"][0]["accepted"])
        self.assertAlmostEqual(report["relative_excursion_observed_rad"]["min"], 0.1)

    def test_steady_stop_speed_without_initial_motion_is_not_a_decel_measurement(self):
        raw = self.stop_records()
        for row in raw:
            if row["phase"] == "BRAKE":
                row["velocity_rad_s"] = 0
        stop = self.analyse(raw)["stop_observations"][0]
        self.assertFalse(stop["accepted"])
        self.assertIn("insufficient_initial_speed", stop["reasons"])

    def test_aborted_trial_never_accepts_earlier_gain_observation(self):
        raw = self.stop_records()
        raw[-1]["fault"] = 1
        pair = self.analyse(raw)["input_gain_pairs"][0]
        self.assertFalse(pair["accepted"])
        self.assertIn("trial_fault", pair["reasons"])

    def test_initial_anchor_change_rejected_and_blank_precapture_anchor_allowed(self):
        raw = records()
        for row in raw:
            row["initial_position_rad"] = "" if row["timestamp_ms"] < 100 else -0.1
        report = self.analyse(raw)
        self.assertAlmostEqual(report["relative_excursion_observed_rad"]["min"], 0.1)
        raw[-1]["initial_position_rad"] = 0.2
        with self.assertRaisesRegex(ValueError, "initial_position_rad changed"):
            self.analyse(raw)

    def test_cli_json_export_and_config_path_refusal(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "capture.csv"
            destination = Path(directory) / "observation.json"
            config = Path(directory) / "config.h"
            with source.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=analysis.REQUIRED_COLUMNS)
                writer.writeheader()
                writer.writerows(records())
            original = source.read_bytes()
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(analysis.main([str(source), "--json", str(destination)]), 0)
                self.assertEqual(analysis.main([str(source), "--json", str(config)]), 2)
            self.assertEqual(source.read_bytes(), original)
            self.assertFalse(config.exists())
            report = json.loads(destination.read_text(encoding="utf-8"))
            self.assertTrue(report["input_gain_pairs"][0]["accepted"])
            self.assertFalse(report["config_written"])

    def test_intentional_pause_between_manual_trials_is_not_a_dropped_sample(self):
        raw = records()
        second = records(current=-0.2)
        for row in second:
            row["timestamp_ms"] += 2000
            row["feedback_timestamp_ms"] += 2000
            row["rx_sequence"] += 200
            row["trial_id"] = "2"
        report = self.analyse(raw + second)
        self.assertEqual(len(report["input_gain_pairs"]), 2)
        self.assertTrue(all(pair["accepted"] for pair in report["input_gain_pairs"]))
        self.assertEqual(report["quality_issue_rows"], {})
        self.assertEqual(report["feedback_interval_ms"]["max"], 10)


if __name__ == "__main__":
    unittest.main()
