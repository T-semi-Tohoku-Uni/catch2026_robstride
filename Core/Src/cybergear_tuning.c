#include "cybergear_tuning.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    TUNING_CURRENT, TUNING_SPEED, TUNING_BANDWIDTH, TUNING_DAMPING, TUNING_OBSERVER,
    TUNING_B0, TUNING_ACCELERATION, TUNING_BRAKING, TUNING_JERK, TUNING_RISE,
    TUNING_FALL, TUNING_DISTURBANCE_FRACTION, TUNING_RESERVE_FRACTION,
    TUNING_DISTURBANCE_SLEW, TUNING_RESERVE_SLEW, TUNING_GAIN, TUNING_LEAK,
    TUNING_AMPLITUDE, TUNING_SMALL_AMPLITUDE, TUNING_DWELL, TUNING_LEG_TIMEOUT
} TuningField;

typedef struct {
    const char *key;
    const char *help;
    TuningField field;
} TuningParameter;

static const TuningParameter parameters[] = {
    {"current", "A; >0, <= compiled current ceiling", TUNING_CURRENT},
    {"speed", "rad/s; >0, <= compiled trajectory speed", TUNING_SPEED},
    {"wc", "rad/s; >0, <= compiled bandwidth ceiling", TUNING_BANDWIDTH},
    {"zeta", "ratio; >0, <= compiled damping ceiling", TUNING_DAMPING},
    {"wo", "rad/s; >0, <= compiled observer bandwidth", TUNING_OBSERVER},
    {"b0", "rad/s^2/A; compiled minimum..maximum", TUNING_B0},
    {"accel", "rad/s^2; >0, <= compiled acceleration", TUNING_ACCELERATION},
    {"brake", "rad/s^2; >0, <= compiled braking", TUNING_BRAKING},
    {"jerk", "rad/s^3; >0, <= compiled jerk", TUNING_JERK},
    {"rise", "A/s; >reserve_slew, <= compiled rise", TUNING_RISE},
    {"fall", "A/s; >reserve_slew, <= compiled fall", TUNING_FALL},
    {"dist_fraction", "ratio; 0..reserve_fraction", TUNING_DISTURBANCE_FRACTION},
    {"reserve_fraction", "ratio; >=dist_fraction, <1", TUNING_RESERVE_FRACTION},
    {"dist_slew", "A/s; >0, <= compiled current rise", TUNING_DISTURBANCE_SLEW},
    {"reserve_slew", "A/s; >=0, <min(rise,fall)", TUNING_RESERVE_SLEW},
    {"gain", "ratio; 0..1", TUNING_GAIN},
    {"leak", "1/s; 0..compiled near leak", TUNING_LEAK},
    {"amp", "deg; >=small, >0, <= compiled test amplitude", TUNING_AMPLITUDE},
    {"small", "deg; >0, <=amp", TUNING_SMALL_AMPLITUDE},
    {"dwell_ms", "integer ms; >0, <leg_ms", TUNING_DWELL},
    {"leg_ms", "integer ms; >dwell_ms, < compiled timeout ceiling", TUNING_LEG_TIMEOUT}
};

size_t cybergear_tuning_count(void)
{
    return sizeof(parameters) / sizeof(parameters[0]);
}

const char *cybergear_tuning_key(size_t index)
{
    return index < cybergear_tuning_count() ? parameters[index].key : NULL;
}

const char *cybergear_tuning_help(size_t index)
{
    return index < cybergear_tuning_count() ? parameters[index].help : NULL;
}

static void refresh_derived(CyberGearTuning *tuning)
{
    CyberGearConfig *motor = &tuning->motor;
    motor->dynamics.fixed_b0 = motor->controller.b0_initial;
    motor->dynamics.b0_min = motor->controller.b0_min;
    motor->dynamics.b0_max = motor->controller.b0_max;
    motor->dynamics.acceleration_current_a = motor->controller.current_limit_a;
    motor->dynamics.braking_current_a = motor->controller.current_limit_a;
    motor->dynamics.current_slew_a_s = fminf(motor->controller.current_rise_a_s,
        motor->controller.current_fall_a_s);
    motor->dynamics.acceleration_cap_rad_s2 = motor->trajectory.acceleration_max_rad_s2;
    motor->dynamics.braking_cap_rad_s2 = motor->trajectory.braking_max_rad_s2;
    motor->dynamics.jerk_cap_rad_s3 = motor->trajectory.jerk_max_rad_s3;
    motor->stall_current_a = CYBERGEAR_STALL_CURRENT_FRACTION * motor->controller.current_limit_a;
}

static bool positive_bounded(float value, float maximum)
{
    return isfinite(value) && value > 0.0f && value <= maximum;
}

bool cybergear_tuning_valid(const CyberGearTuning *tuning)
{
    if (tuning == NULL || !cybergear_config_valid(&tuning->motor)) return false;
    const CyberGearConfig *motor = &tuning->motor;
    const CyberGearControllerConfig *controller = &motor->controller;
    const float amplitude_rad = CYBERGEAR_DEG_TO_RAD(tuning->amplitude_deg);
    return positive_bounded(controller->current_limit_a, CYBERGEAR_CURRENT_LIMIT_A) &&
        positive_bounded(motor->trajectory.velocity_max_rad_s, CYBERGEAR_TRAJECTORY_SPEED_RAD_S) &&
        positive_bounded(controller->bandwidth_rad_s, CG_TUNING_MAX_BANDWIDTH_RAD_S) &&
        positive_bounded(controller->damping_ratio, CG_TUNING_MAX_DAMPING_RATIO) &&
        positive_bounded(controller->observer_rad_s, CYBERGEAR_OBSERVER_RAD_S) &&
        controller->b0_min == (float)CYBERGEAR_B0_MIN &&
        controller->b0_max == (float)CYBERGEAR_B0_MAX &&
        positive_bounded(motor->trajectory.acceleration_max_rad_s2, CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2) &&
        positive_bounded(motor->trajectory.braking_max_rad_s2, CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2) &&
        positive_bounded(motor->trajectory.jerk_max_rad_s3, CYBERGEAR_TRAJECTORY_JERK_RAD_S3) &&
        positive_bounded(controller->current_rise_a_s, CYBERGEAR_CURRENT_RISE_A_S) &&
        positive_bounded(controller->current_fall_a_s, CYBERGEAR_CURRENT_FALL_A_S) &&
        positive_bounded(controller->disturbance_slew_a_s, CYBERGEAR_CURRENT_RISE_A_S) &&
        controller->leak_fixed_s <= CYBERGEAR_LEAK_NEAR_S &&
        motor->dynamics.acceleration_current_a == controller->current_limit_a &&
        motor->dynamics.braking_current_a == controller->current_limit_a &&
        motor->dynamics.current_slew_a_s == fminf(controller->current_rise_a_s, controller->current_fall_a_s) &&
        motor->dynamics.acceleration_cap_rad_s2 == motor->trajectory.acceleration_max_rad_s2 &&
        motor->dynamics.braking_cap_rad_s2 == motor->trajectory.braking_max_rad_s2 &&
        motor->dynamics.jerk_cap_rad_s3 == motor->trajectory.jerk_max_rad_s3 &&
        motor->stall_current_a == (float)(CYBERGEAR_STALL_CURRENT_FRACTION * controller->current_limit_a) &&
        positive_bounded(tuning->amplitude_deg, CG_TEST_AMPLITUDE_DEG) &&
        positive_bounded(tuning->small_amplitude_deg, tuning->amplitude_deg) &&
        -amplitude_rad > motor->trajectory.position_min_rad &&
        amplitude_rad < motor->trajectory.position_max_rad &&
        tuning->dwell_ms > 0U && tuning->dwell_ms < tuning->leg_timeout_ms &&
        tuning->leg_timeout_ms < CG_TUNING_LEG_TIMEOUT_CEILING_MS;
}

bool cybergear_tuning_defaults(CyberGearTuning *tuning)
{
    if (tuning == NULL) return false;
    memset(tuning, 0, sizeof(*tuning));
    cybergear_config_defaults(&tuning->motor);
    CyberGearConfig *motor = &tuning->motor;
    motor->controller.current_limit_a = fminf(motor->controller.current_limit_a, CG_TEST_CURRENT_LIMIT_A);
    motor->controller.disturbance_limit_a = CG_TEST_DISTURBANCE_CURRENT_FRACTION * motor->controller.current_limit_a;
    motor->controller.leak_mode = CG_TEST_LEAK_MODE;
    motor->controller.observer_rad_s = fminf(motor->controller.observer_rad_s, CG_TEST_OBSERVER_RAD_S);
    motor->trajectory.velocity_max_rad_s = fminf(motor->trajectory.velocity_max_rad_s, CG_TEST_SPEED_RAD_S);
    motor->dynamics.reserve_current_a = CG_TEST_RESERVE_CURRENT_FRACTION * motor->controller.current_limit_a;
    tuning->amplitude_deg = CG_TEST_AMPLITUDE_DEG;
    tuning->small_amplitude_deg = CG_TEST_SMALL_AMPLITUDE_DEG;
    tuning->dwell_ms = CG_TEST_DWELL_MS;
    tuning->leg_timeout_ms = CG_TEST_LEG_TIMEOUT_MS;
    refresh_derived(tuning);
    return cybergear_tuning_valid(tuning);
}

double cybergear_tuning_value(const CyberGearTuning *tuning, size_t index)
{
    if (tuning == NULL || index >= cybergear_tuning_count()) return NAN;
    const CyberGearConfig *motor = &tuning->motor;
    switch (parameters[index].field) {
    case TUNING_CURRENT: return motor->controller.current_limit_a;
    case TUNING_SPEED: return motor->trajectory.velocity_max_rad_s;
    case TUNING_BANDWIDTH: return motor->controller.bandwidth_rad_s;
    case TUNING_DAMPING: return motor->controller.damping_ratio;
    case TUNING_OBSERVER: return motor->controller.observer_rad_s;
    case TUNING_B0: return motor->controller.b0_initial;
    case TUNING_ACCELERATION: return motor->trajectory.acceleration_max_rad_s2;
    case TUNING_BRAKING: return motor->trajectory.braking_max_rad_s2;
    case TUNING_JERK: return motor->trajectory.jerk_max_rad_s3;
    case TUNING_RISE: return motor->controller.current_rise_a_s;
    case TUNING_FALL: return motor->controller.current_fall_a_s;
    case TUNING_DISTURBANCE_FRACTION: return motor->controller.disturbance_limit_a / motor->controller.current_limit_a;
    case TUNING_RESERVE_FRACTION: return motor->dynamics.reserve_current_a / motor->controller.current_limit_a;
    case TUNING_DISTURBANCE_SLEW: return motor->controller.disturbance_slew_a_s;
    case TUNING_RESERVE_SLEW: return motor->dynamics.reserve_slew_a_s;
    case TUNING_GAIN: return motor->controller.compensation_gain;
    case TUNING_LEAK: return motor->controller.leak_fixed_s;
    case TUNING_AMPLITUDE: return tuning->amplitude_deg;
    case TUNING_SMALL_AMPLITUDE: return tuning->small_amplitude_deg;
    case TUNING_DWELL: return tuning->dwell_ms;
    case TUNING_LEG_TIMEOUT: return tuning->leg_timeout_ms;
    }
    return NAN;
}

static bool parse_value(const char *text, TuningField field, float *scalar, uint32_t *integer)
{
    if (text == NULL || text[0] == '\0') return false;
    char *end = NULL;
    errno = 0;
    if (field == TUNING_DWELL || field == TUNING_LEG_TIMEOUT) {
        for (const char *digit = text; *digit != '\0'; ++digit)
            if (*digit < '0' || *digit > '9') return false;
        const unsigned long parsed = strtoul(text, &end, 10);
        if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) return false;
        *integer = (uint32_t)parsed;
    } else {
        if ((text[0] < '0' || text[0] > '9') && text[0] != '+' && text[0] != '-' && text[0] != '.') return false;
        *scalar = strtof(text, &end);
        if (errno != 0 || end == text || *end != '\0' || !isfinite(*scalar)) return false;
    }
    return true;
}

bool cybergear_tuning_set(CyberGearTuning *candidate, const char *key, const char *value)
{
    if (!cybergear_tuning_valid(candidate) || key == NULL) return false;
    size_t index = 0U;
    while (index < cybergear_tuning_count() && strcmp(key, parameters[index].key) != 0) ++index;
    if (index == cybergear_tuning_count()) return false;
    const TuningField field = parameters[index].field;
    float scalar = 0.0f;
    uint32_t integer = 0U;
    if (!parse_value(value, field, &scalar, &integer)) return false;
    CyberGearTuning next = *candidate;
    CyberGearConfig *motor = &next.motor;
    switch (field) {
    case TUNING_CURRENT: {
        const float previous_current = motor->controller.current_limit_a;
        const float disturbance_fraction = motor->controller.disturbance_limit_a / previous_current;
        const float reserve_fraction = motor->dynamics.reserve_current_a / previous_current;
        motor->controller.current_limit_a = scalar;
        motor->controller.disturbance_limit_a = disturbance_fraction * scalar;
        motor->dynamics.reserve_current_a = reserve_fraction * scalar;
        break;
    }
    case TUNING_SPEED: motor->trajectory.velocity_max_rad_s = scalar; break;
    case TUNING_BANDWIDTH: motor->controller.bandwidth_rad_s = scalar; break;
    case TUNING_DAMPING: motor->controller.damping_ratio = scalar; break;
    case TUNING_OBSERVER: motor->controller.observer_rad_s = scalar; break;
    case TUNING_B0: motor->controller.b0_initial = scalar; break;
    case TUNING_ACCELERATION: motor->trajectory.acceleration_max_rad_s2 = scalar; break;
    case TUNING_BRAKING: motor->trajectory.braking_max_rad_s2 = scalar; break;
    case TUNING_JERK: motor->trajectory.jerk_max_rad_s3 = scalar; break;
    case TUNING_RISE: motor->controller.current_rise_a_s = scalar; break;
    case TUNING_FALL: motor->controller.current_fall_a_s = scalar; break;
    case TUNING_DISTURBANCE_FRACTION: motor->controller.disturbance_limit_a = scalar * motor->controller.current_limit_a; break;
    case TUNING_RESERVE_FRACTION: motor->dynamics.reserve_current_a = scalar * motor->controller.current_limit_a; break;
    case TUNING_DISTURBANCE_SLEW: motor->controller.disturbance_slew_a_s = scalar; break;
    case TUNING_RESERVE_SLEW: motor->dynamics.reserve_slew_a_s = scalar; break;
    case TUNING_GAIN: motor->controller.compensation_gain = scalar; break;
    case TUNING_LEAK: motor->controller.leak_fixed_s = scalar; break;
    case TUNING_AMPLITUDE: next.amplitude_deg = scalar; break;
    case TUNING_SMALL_AMPLITUDE: next.small_amplitude_deg = scalar; break;
    case TUNING_DWELL: next.dwell_ms = integer; break;
    case TUNING_LEG_TIMEOUT: next.leg_timeout_ms = integer; break;
    }
    refresh_derived(&next);
    if (!cybergear_tuning_valid(&next)) return false;
    *candidate = next;
    return true;
}
