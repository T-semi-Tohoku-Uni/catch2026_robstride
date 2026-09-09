#include "cybergear_homing.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static CyberGearMotor motor;
static uint32_t tick;
static uint32_t mask;
static float velocity;
static float switch_position;
static float zero_position;
static uint32_t zero_count;
static uint32_t enable_count;
static uint32_t stop_count;
static uint32_t watchdog_count;
static bool enabled;
static bool no_feedback;
static bool fail_velocity;
static bool reversed_fast;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1U; }
void __set_PRIMASK(uint32_t value) { mask = value; }

void HAL_Delay(uint32_t milliseconds)
{
    tick += milliseconds;
    if (enabled)
        motor.feedback.position_rad += velocity * (float)milliseconds / 1000.0f;
    if (!no_feedback)
    {
        motor.feedback.online = true;
        motor.feedback.last_received_ms = tick;
        motor.feedback.velocity_rad_s = enabled ? velocity : 0.0f;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    CHECK(port == limit_GPIO_Port && pin == limit_Pin);
    return motor.feedback.position_rad >= switch_position ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

bool cybergear_set_run_mode(CyberGearMotor *instance, CyberGearRunMode mode)
{
    CHECK(instance == &motor);
    instance->run_mode = mode;
    return true;
}

bool cybergear_set_velocity(CyberGearMotor *instance, float value)
{
    CHECK(instance == &motor);
    if (fail_velocity) return false;
    velocity = value;
    if (value == -1.0f) reversed_fast = true;
    return true;
}

bool cybergear_enable(CyberGearMotor *instance)
{
    CHECK(instance == &motor);
    enabled = true;
    ++enable_count;
    return true;
}

bool cybergear_stop(CyberGearMotor *instance)
{
    CHECK(instance == &motor);
    enabled = false;
    velocity = 0;
    ++stop_count;
    return true;
}

bool cybergear_set_zero(CyberGearMotor *instance)
{
    CHECK(instance == &motor && !enabled);
    zero_position = instance->feedback.position_rad;
    instance->feedback.position_rad = 0;
    ++zero_count;
    return true;
}

static void check_feedback(void) { ++watchdog_count; }
static void print_can_status(void) { }

static void reset_simulation(void)
{
    memset(&motor, 0, sizeof(motor));
    tick = 100U;
    mask = 0U;
    velocity = 0;
    switch_position = 0.2f;
    zero_position = 0;
    zero_count = enable_count = stop_count = watchdog_count = 0;
    enabled = no_feedback = fail_velocity = reversed_fast = false;
}

int main(void)
{
    const CyberGearHomingContext context = {&motor, check_feedback, print_can_status};
    reset_simulation();
    CHECK(cybergear_homing_run(&context));
    CHECK(zero_count == 1 && stop_count > 0 && !enabled);
    CHECK(fabsf(zero_position - switch_position) < 0.01f);
    CHECK(!reversed_fast && watchdog_count > 0 && mask == 0);

    reset_simulation();
    switch_position = -0.2f;
    CHECK(cybergear_homing_run(&context));
    CHECK(reversed_fast && zero_count == 1 && !enabled);
    CHECK(fabsf(zero_position - switch_position) < 0.01f);

    reset_simulation();
    switch_position = 100.0f;
    CHECK(!cybergear_homing_run(&context));
    CHECK(reversed_fast && tick >= 15100U && zero_count == 0 && !enabled);

    reset_simulation();
    no_feedback = true;
    CHECK(!cybergear_homing_run(&context));
    CHECK(tick >= 3100U && enable_count > 1 && stop_count > 0 && zero_count == 0);

    reset_simulation();
    motor.feedback.fault_flags = 1;
    CHECK(!cybergear_homing_run(&context));
    CHECK(stop_count > 0 && zero_count == 0 && !enabled);

    reset_simulation();
    fail_velocity = true;
    CHECK(!cybergear_homing_run(&context));
    CHECK(stop_count > 0 && zero_count == 0 && !enabled);

    /* A stale sample may satisfy the startup check but must not permit motion. */
    reset_simulation();
    no_feedback = true;
    motor.feedback.online = true;
    motor.feedback.last_received_ms = tick + 10U;
    CHECK(!cybergear_homing_run(&context));
    CHECK(tick < 1000U && stop_count > 0 && zero_count == 0 && !enabled);
    CHECK(mask == 0);

    /* Preserve unsigned elapsed-time handling across the HAL tick wrap. */
    reset_simulation();
    tick = UINT32_MAX - 50U;
    CHECK(cybergear_homing_run(&context));
    CHECK(zero_count == 1 && !enabled && mask == 0);

    puts("homing: both directions, timeout, missing/stale feedback, fault, TX failure and tick wrap passed");
    return 0;
}
