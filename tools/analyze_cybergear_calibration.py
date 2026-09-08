"""Analyse captured CyberGear calibration CSV; never connect to or configure hardware.

Only the Python standard library is required. Run with --help for the CSV contract
and analysis tolerances. All results are observations, not certified motor limits.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Iterable


REQUIRED_COLUMNS = (
    "timestamp_ms", "feedback_timestamp_ms", "rx_sequence", "trial_id", "phase",
    "position_rad", "velocity_rad_s", "command_current_a", "fault", "saturated",
    "dropped", "tx_failed",
)
PHASE_ALIASES: dict[str, str] = {}
UINT32 = 1 << 32


@dataclass(frozen=True)
class Options:
    """Offline quality gates; these are not safe movement settings for the robot."""

    min_samples: int = 6
    settle_ms: float = 20.0
    max_window_ms: float = 250.0
    max_rx_age_ms: float = 30.0
    max_gap_ms: float = 30.0
    max_current_variation_a: float = 0.001
    min_current_step_a: float = 0.02
    max_fit_rmse_rad: float = 0.001
    position_resolution_rad: float = 25.0 / 65535.0
    min_accel_snr: float = 3.0
    max_pair_gap_ms: float = 1000.0
    max_pair_position_rad: float = 0.1
    max_pair_velocity_rad_s: float = 0.25
    stationary_speed_rad_s: float = 0.03
    stationary_dwell_ms: float = 100.0

    def validate(self) -> None:
        if self.min_samples < 4:
            raise ValueError("min_samples must be at least 4")
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        for name in ("max_window_ms", "max_gap_ms", "min_current_step_a",
                     "position_resolution_rad", "min_accel_snr", "stationary_dwell_ms"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")


def _finite(value: str, name: str, line: int) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"line {line}: {name} must be finite")
    return number


def _integer(value: str, name: str, line: int) -> int:
    number = _finite(value, name, line)
    if int(number) != number or not 0 <= number < UINT32:
        raise ValueError(f"line {line}: {name} must be an unsigned 32-bit integer")
    return int(number)


def load_csv(path: Path) -> list[dict]:
    """Keep recorded order, unwrap HAL ticks, and retain duplicate/drop evidence."""
    rows: list[dict] = []
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(line for line in stream if not line.startswith("#"))
        missing = set(REQUIRED_COLUMNS) - set(reader.fieldnames or ())
        if missing:
            raise ValueError("missing CSV columns: " + ", ".join(sorted(missing)))
        elapsed_ms = 0
        previous_tick = None
        initial_position = None
        for line, raw in enumerate(reader, 2):
            # Serial exports from several manually started trials can repeat a header.
            if all(raw.get(name) == name for name in REQUIRED_COLUMNS):
                continue
            if None in raw or any(raw[name] is None for name in REQUIRED_COLUMNS):
                raise ValueError(f"line {line}: malformed CSV row")
            row: dict = {"line": line, "trial_id": raw["trial_id"].strip(),
                         "phase": raw["phase"].strip().upper(), "issues": []}
            row["phase"] = PHASE_ALIASES.get(row["phase"], row["phase"])
            for name in ("timestamp_ms", "feedback_timestamp_ms", "rx_sequence",
                         "fault", "saturated", "dropped", "tx_failed"):
                row[name] = _integer(raw[name], name, line)
            for name in ("position_rad", "velocity_rad_s", "command_current_a"):
                row[name] = _finite(raw[name], name, line)
            if raw.get("initial_position_rad", ""):
                row["initial_position_rad"] = _finite(raw["initial_position_rad"], "initial_position_rad", line)
                if initial_position is None:
                    initial_position = row["initial_position_rad"]
                elif row["initial_position_rad"] != initial_position:
                    raise ValueError(f"line {line}: initial_position_rad changed; use one boot/session per file")
            row["feedback_valid"] = _integer(raw.get("feedback_valid", "1"),
                                             "feedback_valid", line)
            age = (row["timestamp_ms"] - row["feedback_timestamp_ms"]) % UINT32
            row["rx_age_ms"] = age
            if raw.get("rx_age_ms", ""):
                reported_age = _integer(raw["rx_age_ms"], "rx_age_ms", line)
                if reported_age != age:
                    row["issues"].append("rx_age_mismatch")
            if previous_tick is None:
                elapsed_ms = row["timestamp_ms"]
            else:
                delta = (row["timestamp_ms"] - previous_tick) % UINT32
                if delta == 0 or delta >= UINT32 // 2:
                    raise ValueError(f"line {line}: log timestamps are not increasing")
                elapsed_ms += delta
            row["log_time_ms"] = elapsed_ms
            row["feedback_time_ms"] = elapsed_ms - age
            previous_tick = row["timestamp_ms"]
            rows.append(row)
    if not rows:
        raise ValueError("CSV contains no data rows")
    return rows


def _inverse(matrix: list[list[float]]) -> list[list[float]]:
    n = len(matrix)
    work = [list(row) + [float(i == j) for j in range(n)]
            for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda i: abs(work[i][col]))
        if abs(work[pivot][col]) < 1e-12:
            raise ValueError("singular position fit")
        work[col], work[pivot] = work[pivot], work[col]
        divisor = work[col][col]
        work[col] = [value / divisor for value in work[col]]
        for i in range(n):
            if i != col:
                scale = work[i][col]
                work[i] = [a - scale * b for a, b in zip(work[i], work[col])]
    return [row[n:] for row in work]


def quadratic_fit(rows: list[dict], resolution_rad: float) -> dict:
    """Fit q=c0+c1*x+c2*x^2 at centred/scaled arrival times, not differences."""
    times = [row["feedback_time_ms"] / 1000.0 for row in rows]
    center = statistics.mean(times)
    scale = max(abs(t - center) for t in times)
    if len(rows) < 4 or scale <= 0:
        raise ValueError("insufficient independent timestamps")
    design = [[1.0, (t - center) / scale, ((t - center) / scale) ** 2] for t in times]
    inverse = _inverse([[sum(x[i] * x[j] for x in design) for j in range(3)]
                        for i in range(3)])
    # Subtract a position offset so encoder origin does not impair conditioning.
    offset = rows[0]["position_rad"]
    rhs = [sum(x[i] * (row["position_rad"] - offset)
               for x, row in zip(design, rows)) for i in range(3)]
    coeff = [sum(a * b for a, b in zip(row, rhs)) for row in inverse]
    residuals = [row["position_rad"] - offset - sum(a * b for a, b in zip(x, coeff))
                 for row, x in zip(rows, design)]
    sse = sum(value * value for value in residuals)
    variance = max(sse / (len(rows) - 3), resolution_rad ** 2 / 12.0)
    return {
        "samples": len(rows), "center_time_ms": center * 1000.0,
        "span_ms": (times[-1] - times[0]) * 1000.0,
        "position_rad": coeff[0] + offset, "velocity_rad_s": coeff[1] / scale,
        "feedback_velocity_rad_s": statistics.mean(row["velocity_rad_s"] for row in rows),
        "acceleration_rad_s2": 2.0 * coeff[2] / scale ** 2,
        "acceleration_standard_error_rad_s2": 2.0 * math.sqrt(variance * inverse[2][2]) / scale ** 2,
        "fit_rmse_rad": math.sqrt(sse / len(rows)),
        "command_current_a": statistics.mean(row["command_current_a"] for row in rows),
    }


def _quantiles(values: Iterable[float]) -> dict:
    ordered = sorted(values)
    if not ordered:
        return {"count": 0}
    def percentile(p: float) -> float:
        index = (len(ordered) - 1) * p
        lower = int(index)
        upper = min(lower + 1, len(ordered) - 1)
        return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)
    return {"count": len(ordered), "min": ordered[0], "median": percentile(0.5),
            "p95": percentile(0.95), "max": ordered[-1]}


def _mark_quality(rows: list[dict], options: Options) -> None:
    previous = None
    for row in rows:
        issues = row["issues"]
        if not row["feedback_valid"]:
            issues.append("invalid_feedback")
        if row["rx_age_ms"] > options.max_rx_age_ms:
            issues.append("stale_feedback")
        if row["fault"]:
            issues.append("fault")
        if row["saturated"]:
            issues.append("saturated")
        if previous and row["trial_id"] == previous["trial_id"]:
            seq_delta = (row["rx_sequence"] - previous["rx_sequence"]) % UINT32
            if seq_delta == 0:
                issues.append("duplicate_feedback")
            elif seq_delta >= UINT32 // 2:
                issues.append("reversed_feedback_sequence")
            elif seq_delta > 1:
                issues.append("feedback_sequence_gap")
            dt = row["feedback_time_ms"] - previous["feedback_time_ms"]
            if dt <= 0:
                issues.append("nonincreasing_feedback_time")
            elif dt > options.max_gap_ms:
                issues.append("feedback_time_gap")
            for name in ("dropped", "tx_failed"):
                if row[name] != previous[name]:
                    issues.append(name + "_changed")
        elif row["dropped"] or row["tx_failed"]:
            issues.append("nonzero_initial_loss_counters")
        previous = row


def _segments(rows: list[dict]) -> list[list[dict]]:
    segments: list[list[dict]] = []
    for row in rows:
        if not segments or (row["trial_id"], row["phase"]) != (
                segments[-1][-1]["trial_id"], segments[-1][-1]["phase"]):
            segments.append([])
        segments[-1].append(row)
    return segments


def _fit_segment(rows: list[dict], options: Options) -> dict:
    result = {"trial_id": rows[0]["trial_id"], "phase": rows[0]["phase"],
              "first_line": rows[0]["line"], "last_line": rows[-1]["line"],
              "accepted": False, "reasons": sorted({i for row in rows for i in row["issues"]})}
    # Baseline closest to the pulse; pulse after its leading command transition.
    low = rows[0]["log_time_ms"] + options.settle_ms
    high = rows[-1]["log_time_ms"] - options.settle_ms
    if result["phase"] == "BASELINE":
        low = max(low, high - options.max_window_ms)
    else:
        high = min(high, low + options.max_window_ms)
    selected = [row for row in rows if low <= row["feedback_time_ms"] <= high]
    if len(selected) < options.min_samples:
        result["reasons"].append("insufficient_settled_samples")
    if result["reasons"]:
        return result
    currents = [row["command_current_a"] for row in selected]
    if max(currents) - min(currents) > options.max_current_variation_a:
        result["reasons"].append("nonconstant_current")
        return result
    try:
        result.update(quadratic_fit(selected, options.position_resolution_rad))
    except ValueError as error:
        result["reasons"].append(str(error))
        return result
    if result["fit_rmse_rad"] > options.max_fit_rmse_rad:
        result["reasons"].append("poor_quadratic_fit")
    result["accepted"] = not result["reasons"]
    return result


def _stop_observation(rows: list[dict], options: Options) -> dict:
    reasons = sorted({i for row in rows for i in row["issues"]})
    result = {"trial_id": rows[0]["trial_id"], "phase": rows[0]["phase"],
              "accepted": False, "reasons": reasons, "guaranteed": False,
              "stationary_observed": False,
              "phases_included": list(dict.fromkeys(row["phase"] for row in rows))}
    if reasons:
        return result
    first = rows[0]
    v0 = first["velocity_rad_s"]
    if abs(v0) <= options.stationary_speed_rad_s:
        reasons.append("insufficient_initial_speed")
        return result
    direction = math.copysign(1.0, v0)
    stationary_start = None
    stop_row = None
    for row in rows:
        if abs(row["velocity_rad_s"]) <= options.stationary_speed_rad_s:
            if stationary_start is None:
                stationary_start = row
            if row["feedback_time_ms"] - stationary_start["feedback_time_ms"] >= options.stationary_dwell_ms:
                stop_row = stationary_start
                break
        else:
            stationary_start = None
    end = stop_row or rows[-1]
    selected = [row for row in rows if row["feedback_time_ms"] <= end["feedback_time_ms"]]
    dt = (end["feedback_time_ms"] - first["feedback_time_ms"]) / 1000.0
    distance = max(direction * (row["position_rad"] - first["position_rad"]) for row in selected)
    result.update({"initial_speed_rad_s": v0, "observation_duration_ms": dt * 1000.0,
                   "observed_forward_distance_rad": distance,
                   "observed_absolute_travel_rad": sum(abs(b["position_rad"] - a["position_rad"])
                                                       for a, b in zip(selected, selected[1:])),
                   "stationary_observed": stop_row is not None,
                   "observed_mean_deceleration_rad_s2":
                       direction * (v0 - end["velocity_rad_s"]) / dt if dt > 0 else None,
                   "accepted": dt > 0})
    if stop_row is None:
        reasons.append("stationary_dwell_not_observed_distance_is_incomplete")
    return result


def _stops(segments: list[list[dict]], options: Options) -> list[dict]:
    results = []
    for index, segment in enumerate(segments):
        if segment[0]["phase"] not in ("BRAKE", "COAST"):
            continue
        rows = list(segment)
        # Retain fresh feedback after command STOP to confirm stationary dwell.
        # The output names every phase, so mixed braking/coasting is not presented
        # as a measurement of active-brake performance alone.
        for following in segments[index + 1:]:
            if (following[0]["trial_id"] != segment[0]["trial_id"] or
                    following[0]["phase"] not in ("COAST", "STOPPING", "DONE")):
                break
            rows.extend(following)
        results.append(_stop_observation(rows, options))
    return results


def analyse(rows: list[dict], options: Options | None = None) -> dict:
    options = options or Options()
    options.validate()
    # Caller can reuse records for different offline tolerance choices.
    rows = [dict(row, issues=list(row["issues"])) for row in rows]
    _mark_quality(rows, options)
    segments = _segments(rows)
    fits = [dict(_fit_segment(segment, options), segment_index=index)
            for index, segment in enumerate(segments)
            if segment[0]["phase"] in ("BASELINE", "PULSE")]
    trial_veto: dict[str, set[str]] = {}
    for row in rows:
        for issue in row["issues"]:
            if issue in ("fault", "dropped_changed", "tx_failed_changed", "nonzero_initial_loss_counters"):
                trial_veto.setdefault(row["trial_id"], set()).add("trial_" + issue)
    pairs = []
    for index, pulse in enumerate(fits):
        if pulse["phase"] != "PULSE":
            continue
        result = {"trial_id": pulse["trial_id"], "pulse_first_line": pulse["first_line"],
                  "accepted": False, "reasons": sorted(trial_veto.get(pulse["trial_id"], set()))}
        pairs.append(result)
        # Require immediately preceding analysed segment, never average different trials.
        baseline = fits[index - 1] if index > 0 else None
        if (baseline is None or baseline["phase"] != "BASELINE" or
                baseline["trial_id"] != pulse["trial_id"] or
                baseline["segment_index"] + 1 != pulse["segment_index"]):
            result["reasons"].append("missing_preceding_trial_baseline")
            continue
        if not baseline["accepted"] or not pulse["accepted"]:
            result["reasons"].append("rejected_baseline_or_pulse_window")
            continue
        dq = abs(pulse["position_rad"] - baseline["position_rad"])
        dv = abs(pulse["velocity_rad_s"] - baseline["velocity_rad_s"])
        dt = pulse["center_time_ms"] - baseline["center_time_ms"]
        di = pulse["command_current_a"] - baseline["command_current_a"]
        da = pulse["acceleration_rad_s2"] - baseline["acceleration_rad_s2"]
        sigma = math.hypot(pulse["acceleration_standard_error_rad_s2"],
                           baseline["acceleration_standard_error_rad_s2"])
        result.update({"pair_position_difference_rad": dq, "pair_velocity_difference_rad_s": dv,
                       "pair_time_difference_ms": dt, "delta_current_a": di,
                       "delta_acceleration_rad_s2": da, "acceleration_difference_snr": abs(da) / sigma})
        for invalid, reason in (
                (dq > options.max_pair_position_rad, "position_mismatch"),
                (dv > options.max_pair_velocity_rad_s, "velocity_mismatch"),
                (not 0 < dt <= options.max_pair_gap_ms, "time_mismatch"),
                (abs(di) < options.min_current_step_a, "insufficient_current_excitation"),
                (abs(da) < options.min_accel_snr * sigma, "insufficient_acceleration_signal")):
            if invalid:
                result["reasons"].append(reason)
        if abs(di) >= options.min_current_step_a:
            result["signed_b_rad_s2_per_a"] = da / di
            result["b_fit_standard_error_rad_s2_per_a"] = sigma / abs(di)
        if not result["reasons"]:
            result["accepted"] = True
            result["positive_coordinate_direction"] = da / di > 0
    accepted = [pair["signed_b_rad_s2_per_a"] for pair in pairs if pair["accepted"]]
    errors = {issue: sum(issue in row["issues"] for row in rows)
              for issue in sorted({issue for row in rows for issue in row["issues"]})}
    anchors = [row["initial_position_rad"] for row in rows if "initial_position_rad" in row]
    anchor = anchors[0] if anchors else rows[0]["position_rad"]
    return {
        "schema_version": 1, "status": "unverified_observations",
        "config_written": False, "options": asdict(options), "row_count": len(rows),
        "quality_issue_rows": errors, "rx_age_ms": _quantiles(row["rx_age_ms"] for row in rows),
        "feedback_interval_ms": _quantiles(b["feedback_time_ms"] - a["feedback_time_ms"]
                                           for a, b in zip(rows, rows[1:])
                                           if b["trial_id"] == a["trial_id"] and
                                           b["feedback_time_ms"] > a["feedback_time_ms"]),
        "position_observed_rad": _quantiles(row["position_rad"] for row in rows),
        "relative_excursion_observed_rad": {
            "anchor_source": "initial_position_rad" if anchors else "first_log_position",
            "min": min(row["position_rad"] - anchor for row in rows),
            "max": max(row["position_rad"] - anchor for row in rows)},
        "windows": fits, "input_gain_pairs": pairs,
        "signed_b_observed_rad_s2_per_a": _quantiles(accepted),
        "stop_observations": _stops(segments, options),
        "limitations": [
            "Current is a queued command, not measured iq or an arrival acknowledgement.",
            "Feedback timestamps are CAN arrival times, not motor sampling times.",
            "Sequence gaps can reflect faster feedback than logging; they are conservatively rejected, not identified as CAN loss.",
            "Fit standard errors include a quantization floor but exclude current, delay, friction and load bias.",
            "Matched position/velocity gates reduce but do not remove gravity, friction and posture confounding.",
            "Observed extrema do not establish B0_MIN/MAX, permitted current, mechanical limits or guaranteed braking.",
            "Stopping records start at the first logged phase sample; earlier command/transport delay is omitted.",
            "COAST is not active braking. BRAKE is an observed phase label, not a brake performance guarantee.",
        ],
    }


def format_summary(report: dict) -> str:
    lines = ["CyberGear calibration: UNVERIFIED OBSERVATIONS (no configuration written)",
             f"Rows: {report['row_count']}; quality issues: {report['quality_issue_rows']}",
             f"RX age [ms]: {report['rx_age_ms']}",
             f"Feedback interval [ms]: {report['feedback_interval_ms']}"]
    for pair in report["input_gain_pairs"]:
        tag = "OBSERVED" if pair["accepted"] else "REJECTED"
        value = pair.get("signed_b_rad_s2_per_a")
        estimate = "none" if value is None else f"{value:.6g} rad/s^2/A"
        lines.append(f"Trial {pair['trial_id']} {tag}: signed b={estimate}; {', '.join(pair['reasons'])}")
        if pair["accepted"] and not pair["positive_coordinate_direction"]:
            lines.append("  Negative sign: incompatible with positive-b normal controller; do not take absolute value.")
    if not report["input_gain_pairs"]:
        lines.append("No BASELINE/PULSE gain pairs were recorded.")
    for stop in report["stop_observations"]:
        lines.append(f"Trial {stop['trial_id']} {stop['phase']}: " + json.dumps(stop, ensure_ascii=False))
    lines.extend("- " + item for item in report["limitations"])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, epilog="CSV columns: " + ", ".join(REQUIRED_COLUMNS))
    parser.add_argument("csv", type=Path, help="Captured CSV file (read only)")
    parser.add_argument("--json", type=Path, help="Write observations and quality reasons as JSON; never a config")
    defaults = Options()
    for name, value in asdict(defaults).items():
        parser.add_argument("--" + name.replace("_", "-"), type=int if name == "min_samples" else float,
                            default=value, help=f"Offline analysis criterion; default {value:g}")
    args = parser.parse_args(argv)
    try:
        report = analyse(load_csv(args.csv), Options(**{name: getattr(args, name) for name in asdict(defaults)}))
        report["source"] = str(args.csv.resolve())
        print(format_summary(report))
        if args.json:
            if args.json.resolve() == args.csv.resolve():
                raise ValueError("JSON destination must differ from input CSV")
            if args.json.suffix.lower() != ".json":
                raise ValueError("observation output must have a .json suffix")
            args.json.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
