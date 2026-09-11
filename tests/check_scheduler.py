"""Compile the real TIM6 callback with counting stubs; never contact hardware."""
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode = True
from compile_host import compile_host


def callback(source: str, name: str = "HAL_TIM_PeriodElapsedCallback") -> str:
    start = source.index("void " + name + "(")
    opening = source.index("{", start)
    depth = 1
    for index in range(opening + 1, len(source)):
        depth += (source[index] == "{") - (source[index] == "}")
        if depth == 0:
            return source[start:index + 1]
    raise ValueError("Unbalanced callback braces")


PREFIX = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "cybergear.h"
#include "app_mode.h"
#include "board_protocol.h"
#define FDCAN_DLC_BYTES_16 0x000a0000U
#define RIGHT_RS03_INDEX 0
#define LEFT_RS03_INDEX 1
#define EL05_INDEX 2
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { struct { float position_rad; } feedback; } MockRobstride;
static TIM_HandleTypeDef htim6, other_timer;
static CyberGearMotor cybergear_base;
static MockRobstride robstride_handler[3];
static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_TxHeaderTypeDef inter_board_txheader;
static bool motors_running;
#if APP_CYBERGEAR_STANDALONE_TEST
static bool cybergear_test_timer_active = true;
static uint8_t cybergear_test_control_phase;
#endif
static float target_angle[4] = {0.3f, 0.5f, -0.1f, 0.8f};
static unsigned int cg_calls, rs_calls[3], board_calls;
static float cg_target, rs_target[3], reported[4];
bool cybergear_control_position_adrc(CyberGearMotor *m, float target)
{
    assert(m == &cybergear_base);
    ++cg_calls;
    cg_target = target;
    return m->state != CG_STATE_FAULT;
}
static bool robstride_set_position(MockRobstride *m, float target)
{
    const unsigned int index = (unsigned int)(m - robstride_handler);
    assert(index < 3U);
    ++rs_calls[index];
    rs_target[index] = target;
    return true;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *m,
    FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    assert(m == &hfdcan1);
    assert(header->Identifier == 0x210U);
    assert(header->DataLength == FDCAN_DLC_BYTES_16);
    board_decode_angles(data, reported);
    ++board_calls;
    return HAL_OK;
}
'''

SUFFIX = r'''
int main(void)
{
    cybergear_base.feedback.position_rad = 0.7f;
    robstride_handler[0].feedback.position_rad = 0.2f;
    robstride_handler[1].feedback.position_rad = 0.4f;
    robstride_handler[2].feedback.position_rad = -0.6f;
    for (unsigned int i = 0; i < 1000U; ++i) HAL_TIM_PeriodElapsedCallback(&other_timer);
    assert(cg_calls == 0U && board_calls == 0U);
    for (unsigned int i = 0; i < 1000U; ++i) HAL_TIM_PeriodElapsedCallback(&htim6);
    assert(cg_calls == (APP_CYBERGEAR_STANDALONE_TEST ?
        (CYBERGEAR_USE_200_HZ ? 200U : 100U) : 0U));
    assert(rs_calls[0] == 0U && rs_calls[1] == 0U && rs_calls[2] == 0U);
    assert(board_calls == 0U);
    cg_calls = 0U;
    motors_running = true;
    for (unsigned int i = 0; i < 1000U; ++i) HAL_TIM_PeriodElapsedCallback(&htim6);
    assert(cg_calls == (CYBERGEAR_USE_200_HZ ? 200U : 100U));
    for (unsigned int i = 0; i < 3U; ++i)
        assert(rs_calls[i] == (APP_CYBERGEAR_STANDALONE_TEST ? 0U : 100U));
    assert(board_calls == (APP_CYBERGEAR_STANDALONE_TEST ? 0U : 100U));
    assert(cg_target == target_angle[0]);
    if (!APP_CYBERGEAR_STANDALONE_TEST) {
    assert(fabsf(rs_target[0] - (target_angle[2] - 1.884f)) < 1e-6f);
    assert(fabsf(rs_target[1] - (-target_angle[1] - 1.0f)) < 1e-6f);
    assert(fabsf(rs_target[2] - (-target_angle[3] - 2.963f)) < 1e-6f);
    assert(fabsf(reported[0] - 0.7f) < 1e-6f);
    assert(fabsf(reported[1] + 1.4f) < 1e-6f);
    assert(fabsf(reported[2] - 2.084f) < 1e-6f);
    assert(fabsf(reported[3] + 2.363f) < 1e-6f);
    }
    cybergear_base.state = CG_STATE_FAULT;
    for (unsigned int i = 0; i < 1000U; ++i) HAL_TIM_PeriodElapsedCallback(&htim6);
    assert(cg_calls == (CYBERGEAR_USE_200_HZ ? 400U : 200U));
    if (APP_CYBERGEAR_STANDALONE_TEST) {
        assert(rs_calls[0] == 0U && rs_calls[1] == 0U && rs_calls[2] == 0U);
        assert(board_calls == 0U);
    } else {
        assert(rs_calls[0] == 200U && rs_calls[1] == 200U && rs_calls[2] == 200U);
        assert(board_calls == 200U);
    }
    motors_running = false;
    cg_calls = 0U;
    board_calls = 0U;
    for (unsigned int index = 0U; index < 3U; ++index) rs_calls[index] = 0U;
    for (unsigned int index = 0U; index < 1000U; ++index) HAL_TIM_PeriodElapsedCallback(&htim6);
    assert(cg_calls == (APP_CYBERGEAR_STANDALONE_TEST ?
        (CYBERGEAR_USE_200_HZ ? 200U : 100U) : 0U));
    assert(rs_calls[0] == 0U && rs_calls[1] == 0U && rs_calls[2] == 0U);
    assert(board_calls == 0U);
    puts("Real TIM6 callback: startup/stop gates, cadence, offsets, and isolation passed.");
    return 0;
}
'''


def main() -> None:
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = callback((repo / "Core/Src/main.c").read_text(encoding="utf-8"))
    generated = output / "scheduler_test_generated.c"
    protocol = (repo / "Core/Src/board_protocol.c").read_text(encoding="utf-8")
    generated.write_text(PREFIX + protocol + source + SUFFIX, encoding="utf-8")
    for high_rate in (0, 1):
        for standalone in (0, 1):
            binary = output / f"scheduler_{high_rate}_{standalone}.exe"
            compile_host(compiler, repo, generated, binary,
                         [f"CYBERGEAR_USE_200_HZ={high_rate}",
                          f"APP_CYBERGEAR_STANDALONE_TEST={standalone}"])
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
