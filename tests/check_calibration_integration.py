"""Check real calibration startup/interrupt isolation without contacting hardware.

Preprocess main.c for both build modes, then compile and run its calibration
TIM6 and FDCAN1 callbacks against in-memory stubs. Normal drive APIs and target
state are intentionally absent: an accidental call/reference is a compile error.
"""
from pathlib import Path
import re
import subprocess
import sys


def function(source: str, name: str) -> str:
    match = re.search(r"\b(?:void|int)\s+" + re.escape(name)
                      + r"\s*\([^;{}]*\)\s*\{", source)
    if match is None:
        raise AssertionError(f"Function definition not found: {name}")
    opening = source.index("{", match.start())
    depth = 1
    # The preprocessor removed comments; erase string contents before counting
    # braces so diagnostic text cannot change the function extent.
    structural = re.sub(r'"(?:\\.|[^"\\])*"',
                        lambda item: " " * len(item.group(0)), source)
    for index in range(opening + 1, len(source)):
        depth += (structural[index] == "{") - (structural[index] == "}")
        if depth == 0:
            return source[match.start():index + 1]
    raise AssertionError(f"Unbalanced braces: {name}")


def preprocess(compiler: str, source: Path, output: Path, enabled: int) -> str:
    # Only main.c's conditional code is under test. Avoid including target HAL
    # headers when preprocessing on a native compiler; declarations needed by
    # the selected callbacks are supplied below when compiling the result.
    text = re.sub(r"^\s*#\s*include[^\n]*", "",
                  source.read_text(encoding="utf-8"), flags=re.MULTILINE)
    path = output / f"calibration_main_preprocess_{enabled}.c"
    path.write_text(text, encoding="utf-8")
    result = subprocess.run(
        [compiler, "-E", "-P", "-x", "c",
         f"-DCYBERGEAR_CALIBRATION_BUILD={enabled}", str(path)],
        check=True, capture_output=True, text=True, encoding="utf-8")
    return result.stdout


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32g4xx_hal.h"
#include "cybergear_calibration_config.h"
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 1U
#define FDCAN_RX_FIFO1 1U
static int peripheral1, peripheral3;
#define FDCAN1 ((void *)&peripheral1)
#define FDCAN3 ((void *)&peripheral3)
typedef struct { int unused; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim6, other_timer;
static int cybergear_calibration_app;
static uint32_t now_ms, cal_calls, last_cal_ms;
static unsigned int next_frame, frame_count;
static FDCAN_RxHeaderTypeDef frames[3];
uint32_t HAL_GetTick(void) { return now_ms; }
static void cg_cal_app_tick(int *app, uint32_t tick)
{
    assert(app == &cybergear_calibration_app);
    assert(tick == now_ms);
    if (cal_calls != 0U) assert(tick - last_cal_ms == 10U);
    last_cal_ms = tick;
    ++cal_calls;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *can, uint32_t fifo)
{
    assert(can->Instance == FDCAN1);
    assert(fifo == FDCAN_RX_FIFO1);
    return frame_count - next_frame;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    assert(can->Instance == FDCAN1);
    assert(fifo == FDCAN_RX_FIFO1);
    assert(next_frame < frame_count);
    *header = frames[next_frame++];
    memset(data, 0x7f, 16U);
    return HAL_OK;
}
'''


SUFFIX = r'''
int main(void)
{
    for (unsigned int i = 0U; i < 1000U; ++i) {
        ++now_ms;
        HAL_TIM_PeriodElapsedCallback(&other_timer);
    }
    assert(cal_calls == 0U);
    for (unsigned int i = 0U; i < 1000U; ++i) {
        ++now_ms;
        HAL_TIM_PeriodElapsedCallback(&htim6);
    }
    assert(cal_calls == 100U);

    frames[0].Identifier = 0x500U; /* Normal motor-init/return request. */
    frames[1].Identifier = 0x200U; /* Normal position target frame. */
    frames[2].Identifier = 0x777U; /* Unknown/irrelevant frame. */
    frame_count = 3U;
    FDCAN_HandleTypeDef can1 = {FDCAN1, 0U};
    FDCAN_HandleTypeDef other_can = {FDCAN3, 0U};
    HAL_FDCAN_RxFifo1Callback(&other_can, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    HAL_FDCAN_RxFifo1Callback(&can1, 0U);
    assert(next_frame == 0U);
    HAL_FDCAN_RxFifo1Callback(&can1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    assert(next_frame == frame_count);
    assert(cal_calls == 100U);
    puts("Calibration real callbacks: 100 Hz, no normal drive, board commands drained.");
    return 0;
}
'''


def main() -> None:
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = repo / "Core/Src/main.c"
    normal = preprocess(compiler, source, output, 0)
    calibration = preprocess(compiler, source, output, 1)
    normal_entry = function(normal, "main")
    calibration_entry = function(calibration, "main")
    assert "cybergear_homing(" in normal_entry
    assert "robstride_init(" in normal_entry
    assert "cg_cal_app_init(" in calibration_entry
    assert "cg_cal_app_request_trial(" in calibration_entry
    for forbidden in ("cybergear_base_init", "cybergear_homing", "robstride_init",
                      "robstride_start_position_pp_mode", "cybergear_set_zero",
                      "cybergear_start_position_adrc", "cybergear_service",
                      "motor_check_feedback", "motor_return_update", "NVIC_SystemReset"):
        assert not re.search(r"\b" + forbidden + r"\s*\(", calibration_entry), forbidden

    generated = output / "calibration_callbacks_generated.c"
    generated.write_text(
        PREFIX + function(calibration, "HAL_TIM_PeriodElapsedCallback")
        + function(calibration, "HAL_FDCAN_RxFifo1Callback") + SUFFIX,
        encoding="utf-8")
    binary = output / "calibration_callbacks.exe"
    subprocess.run(
        [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-UNDEBUG",
         "-I" + str(repo / "tests/stubs"), "-I" + str(repo / "Core/Inc"),
         str(generated), "-o", str(binary)],
        check=True)
    subprocess.run([str(binary)], check=True)
    print("Calibration main startup excludes homing, zeroing, other-axis enables and reset loops.")


if __name__ == "__main__":
    main()
