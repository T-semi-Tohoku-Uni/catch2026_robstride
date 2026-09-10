"""Change the central header in an isolated host fixture and run real configuration code."""
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
from compile_host import compile_host


OVERRIDES = {
    "APP_CYBERGEAR_STANDALONE_TEST": "0",
    "CYBERGEAR_USE_200_HZ": "1",
    "CYBERGEAR_CURRENT_LIMIT_A": "4.5f",
    "CYBERGEAR_B0_FIXED": "1.25f",
    "CYBERGEAR_B0_MIN": "0.4f",
    "CYBERGEAR_B0_MAX": "4.0f",
    "CYBERGEAR_SOFT_MIN_RAD": "-2.5f",
    "CYBERGEAR_SOFT_MAX_RAD": "2.5f",
    "CYBERGEAR_HARD_MIN_RAD": "-3.0f",
    "CYBERGEAR_HARD_MAX_RAD": "3.0f",
    "CYBERGEAR_TRAJECTORY_SPEED_RAD_S": "0.7f",
    "CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2": "0.8f",
    "CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2": "0.9f",
    "CYBERGEAR_TRAJECTORY_JERK_RAD_S3": "3.0f",
    "CYBERGEAR_TIMING_TOLERANCE_MS": "0U",
    "CYBERGEAR_FEEDBACK_TIMEOUT_MS": "80U",
    "CYBERGEAR_CONTROL_BANDWIDTH_RAD_S": "3.5f",
    "CYBERGEAR_CONTROL_DAMPING_RATIO": "1.7f",
    "CYBERGEAR_OBSERVER_RAD_S": "8.0f",
    "CYBERGEAR_CURRENT_RISE_A_S": "6.0f",
    "CYBERGEAR_CURRENT_FALL_A_S": "4.0f",
    "CYBERGEAR_DISTURBANCE_LIMIT_A": "0.6f",
    "CYBERGEAR_DISTURBANCE_SLEW_A_S": "0.8f",
    "CYBERGEAR_COMPENSATION_GAIN": "0.9f",
    "CYBERGEAR_COMPENSATION_DELAY_MS": "150U",
    "CYBERGEAR_COMPENSATION_RAMP_MS": "650U",
    "CYBERGEAR_LEAK_MODE": "CYBERGEAR_LEAK_FIXED",
    "CYBERGEAR_LEAK_FIXED_S": "0.04f",
    "CYBERGEAR_LEAK_NEAR_S": "0.35f",
    "CYBERGEAR_LEAK_FAR_S": "0.02f",
    "CYBERGEAR_LEAK_NEAR_RAD": "0.004f",
    "CYBERGEAR_LEAK_FAR_RAD": "0.016f",
    "CYBERGEAR_TRACKING_ERROR_RAD": "0.12f",
    "CYBERGEAR_TRACKING_TIMEOUT_MS": "600U",
    "CYBERGEAR_SATURATION_TIMEOUT_MS": "1100U",
    "CYBERGEAR_STALL_CURRENT_FRACTION": "0.7f",
    "CYBERGEAR_STATIONARY_SPEED_RAD_S": "0.025f",
    "CYBERGEAR_PLANNER_LEAD_MS": "60U",
    "CYBERGEAR_PLANNER_TIMEOUT_MS": "550U",
    "CYBERGEAR_DYNAMICS_RESERVE_CURRENT_FRACTION": "0.3f",
    "CYBERGEAR_DYNAMICS_RESERVE_SLEW_A_S": "2.0f",
    "CG_TEST_AMPLITUDE_DEG": "45.0f",
    "CG_TEST_SMALL_AMPLITUDE_DEG": "3.5f",
    "CG_TEST_SPEED_RAD_S": "0.22f",
    "CG_TEST_CURRENT_LIMIT_A": "2.5f",
    "CG_TEST_OBSERVER_RAD_S": "5.0f",
    "CG_TEST_DWELL_MS": "1200U",
    "CG_TEST_LEG_TIMEOUT_MS": "24000U",
    "CG_TEST_REACHED_DEG": "0.4f",
    "CG_TEST_REACHED_SPEED_RAD_S": "0.02f",
    "CG_TEST_TRAVEL_GUARD_DEG": "2.0f",
    "CG_TEST_DISTURBANCE_CURRENT_FRACTION": "0.4f",
    "CG_TEST_RESERVE_CURRENT_FRACTION": "0.45f",
    "CG_TEST_LEAK_MODE": "CYBERGEAR_LEAK_NONE",
}

FIELDS = {
    "controller.current_limit_a": "CYBERGEAR_CURRENT_LIMIT_A",
    "controller.b0_initial": "CYBERGEAR_B0_FIXED",
    "controller.b0_min": "CYBERGEAR_B0_MIN",
    "controller.b0_max": "CYBERGEAR_B0_MAX",
    "dynamics.fixed_b0": "CYBERGEAR_B0_FIXED",
    "dynamics.b0_min": "CYBERGEAR_B0_MIN",
    "dynamics.b0_max": "CYBERGEAR_B0_MAX",
    "trajectory.position_min_rad": "CYBERGEAR_SOFT_MIN_RAD",
    "trajectory.position_max_rad": "CYBERGEAR_SOFT_MAX_RAD",
    "hard_min_rad": "CYBERGEAR_HARD_MIN_RAD",
    "hard_max_rad": "CYBERGEAR_HARD_MAX_RAD",
    "trajectory.velocity_max_rad_s": "CYBERGEAR_TRAJECTORY_SPEED_RAD_S",
    "trajectory.acceleration_max_rad_s2": "CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2",
    "trajectory.braking_max_rad_s2": "CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2",
    "trajectory.jerk_max_rad_s3": "CYBERGEAR_TRAJECTORY_JERK_RAD_S3",
    "dynamics.acceleration_cap_rad_s2": "CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2",
    "dynamics.braking_cap_rad_s2": "CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2",
    "dynamics.jerk_cap_rad_s3": "CYBERGEAR_TRAJECTORY_JERK_RAD_S3",
    "controller.timing_tolerance_ms": "CYBERGEAR_TIMING_TOLERANCE_MS",
    "controller.feedback_timeout_ms": "CYBERGEAR_FEEDBACK_TIMEOUT_MS",
    "controller.bandwidth_rad_s": "CYBERGEAR_CONTROL_BANDWIDTH_RAD_S",
    "controller.damping_ratio": "CYBERGEAR_CONTROL_DAMPING_RATIO",
    "controller.observer_rad_s": "CYBERGEAR_OBSERVER_RAD_S",
    "controller.current_rise_a_s": "CYBERGEAR_CURRENT_RISE_A_S",
    "controller.current_fall_a_s": "CYBERGEAR_CURRENT_FALL_A_S",
    "controller.disturbance_limit_a": "CYBERGEAR_DISTURBANCE_LIMIT_A",
    "controller.disturbance_slew_a_s": "CYBERGEAR_DISTURBANCE_SLEW_A_S",
    "controller.compensation_gain": "CYBERGEAR_COMPENSATION_GAIN",
    "controller.compensation_delay_ms": "CYBERGEAR_COMPENSATION_DELAY_MS",
    "controller.compensation_ramp_ms": "CYBERGEAR_COMPENSATION_RAMP_MS",
    "controller.leak_mode": "CYBERGEAR_LEAK_MODE",
    "controller.leak_fixed_s": "CYBERGEAR_LEAK_FIXED_S",
    "controller.leak_near_s": "CYBERGEAR_LEAK_NEAR_S",
    "controller.leak_far_s": "CYBERGEAR_LEAK_FAR_S",
    "controller.leak_near_rad": "CYBERGEAR_LEAK_NEAR_RAD",
    "controller.leak_far_rad": "CYBERGEAR_LEAK_FAR_RAD",
    "tracking_error_rad": "CYBERGEAR_TRACKING_ERROR_RAD",
    "tracking_timeout_ms": "CYBERGEAR_TRACKING_TIMEOUT_MS",
    "saturation_timeout_ms": "CYBERGEAR_SATURATION_TIMEOUT_MS",
    "stationary_speed_rad_s": "CYBERGEAR_STATIONARY_SPEED_RAD_S",
    "planner_lead_ms": "CYBERGEAR_PLANNER_LEAD_MS",
    "planner_timeout_ms": "CYBERGEAR_PLANNER_TIMEOUT_MS",
    "dynamics.reserve_slew_a_s": "CYBERGEAR_DYNAMICS_RESERVE_SLEW_A_S",
}


def replace_define(source: str, name: str, value: str) -> str:
    result, count = re.subn(r"(?m)^([ \t]*#define[ \t]+" + re.escape(name)
                            + r"[ \t]+)[^\n]+$", lambda match: match[1] + value, source)
    if count != 1:
        raise ValueError(f"Expected one central definition of {name}; found {count}")
    return result


def main() -> None:
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo = Path(repo_arg).resolve()
    output = Path(output_arg).resolve() / "cybergear-config-header"
    output.mkdir(parents=True, exist_ok=True)
    source = (repo / "Core/Inc/cybergear_config.h").read_text(encoding="utf-8")
    for name, value in OVERRIDES.items():
        source = replace_define(source, name, value)
    config_header = output / "cybergear_config_altered.h"
    config_header.write_text(source, encoding="utf-8")
    generated = output / "config_header_generated.c"
    generated.write_text((repo / "tests/config_header_test.c").read_text(encoding="utf-8"),
                         encoding="utf-8")
    assertions = [f"check_close(config.{field}, {OVERRIDES[macro]});"
                  for field, macro in FIELDS.items()]
    assertions += ["assert(config.controller.period_ms == 5U);",
                   "assert(cybergear_control_phase_due(5U));",
                   "assert(!cybergear_control_phase_due(4U));"]
    sources = [repo / "Core/Src" / name for name in (
        "cybergear.c", "cybergear_controller.c", "cybergear_dynamics.c",
        "cybergear_trajectory.c", "cybergear_test_motion.c")]
    for explicit_override in (False, True):
        expected_mode = int(explicit_override)
        (output / "config_header_assertions.h").write_text(
            "\n".join(assertions + [f"assert(APP_CYBERGEAR_STANDALONE_TEST == {expected_mode});"]),
            encoding="utf-8")
        binary = output / f"config_header_mode_{expected_mode}.exe"
        defines = ["APP_CYBERGEAR_STANDALONE_TEST=1"] if explicit_override else []
        compile_host(compiler, repo, generated, binary, defines,
                     extra_sources=sources, force_include=config_header)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
