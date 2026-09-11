#include "cybergear_tuning.h"
#include "cybergear_test_motion.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static size_t parameter_index(const char *key)
{
    for (size_t index = 0U; index < cybergear_tuning_count(); ++index)
        if (strcmp(key, cybergear_tuning_key(index)) == 0) return index;
    assert(false);
    return 0U;
}

static void assert_rejected(CyberGearTuning *candidate, const char *key, const char *value)
{
    const CyberGearTuning previous = *candidate;
    assert(!cybergear_tuning_set(candidate, key, value));
    assert(memcmp(candidate, &previous, sizeof(*candidate)) == 0);
}

static void defaults_and_metadata(void)
{
    CyberGearTuning tuning;
    assert(cybergear_tuning_defaults(&tuning));
    assert(cybergear_tuning_valid(&tuning));
    CyberGearMotor reference;
    memset(&reference, 0, sizeof(reference));
    cybergear_config_defaults(&reference.config);
    assert(cybergear_test_configure(&reference));
    assert(memcmp(&tuning.motor.controller, &reference.config.controller,
        sizeof(tuning.motor.controller)) == 0);
    assert(memcmp(&tuning.motor.trajectory, &reference.config.trajectory,
        sizeof(tuning.motor.trajectory)) == 0);
    assert(tuning.motor.dynamics.acceleration_current_a == reference.config.dynamics.acceleration_current_a);
    assert(tuning.motor.dynamics.braking_current_a == reference.config.dynamics.braking_current_a);
    assert(tuning.motor.dynamics.reserve_current_a == reference.config.dynamics.reserve_current_a);
    assert(tuning.motor.dynamics.current_slew_a_s == reference.config.dynamics.current_slew_a_s);
    assert(tuning.motor.stall_current_a == reference.config.stall_current_a);
    assert(tuning.amplitude_deg == CG_TEST_AMPLITUDE_DEG);
    assert(tuning.small_amplitude_deg == CG_TEST_SMALL_AMPLITUDE_DEG);
    assert(tuning.dwell_ms == CG_TEST_DWELL_MS);
    assert(tuning.leg_timeout_ms == CG_TEST_LEG_TIMEOUT_MS);
    assert(cybergear_tuning_count() == 21U);
    for (size_t index = 0U; index < cybergear_tuning_count(); ++index) {
        assert(cybergear_tuning_key(index) != NULL);
        assert(cybergear_tuning_help(index) != NULL);
        assert(isfinite(cybergear_tuning_value(&tuning, index)));
        for (size_t previous = 0U; previous < index; ++previous)
            assert(strcmp(cybergear_tuning_key(index), cybergear_tuning_key(previous)) != 0);
    }
    assert(cybergear_tuning_key(cybergear_tuning_count()) == NULL);
    assert(cybergear_tuning_help(cybergear_tuning_count()) == NULL);
    assert(isnan(cybergear_tuning_value(&tuning, cybergear_tuning_count())));
    assert(isnan(cybergear_tuning_value(NULL, 0U)));
    assert(!cybergear_tuning_defaults(NULL));
    assert(!cybergear_tuning_valid(NULL));
    assert(!cybergear_tuning_set(NULL, "current", "2"));
}

static void accepted_parameters(void)
{
    static const struct {
        const char *key;
        const char *text;
        double expected;
    } cases[] = {
        {"current", "2", 2.0}, {"speed", "0.2", 0.2}, {"wc", "3.5", 3.5},
        {"zeta", "1.5", 1.5}, {"wo", "5", 5.0}, {"b0", "1.5", 1.5},
        {"accel", "0.6", 0.6}, {"brake", "0.7", 0.7}, {"jerk", "2", 2.0},
        {"rise", "4", 4.0}, {"fall", "4.5", 4.5}, {"dist_fraction", "0.4", 0.4},
        {"reserve_fraction", "0.6", 0.6}, {"dist_slew", "0.8", 0.8},
        {"reserve_slew", "1.5", 1.5}, {"gain", "0.7", 0.7}, {"leak", "0.1", 0.1},
        {"amp", "45", 45.0}, {"small", "3", 3.0}, {"dwell_ms", "1500", 1500.0},
        {"leg_ms", "30000", 30000.0}, {"current", "6", 6.0}, {"speed", "0.4", 0.4},
        {"wc", "20", 20.0}, {"zeta", "10", 10.0}, {"wo", "10", 10.0},
        {"b0", "0.35", 0.35}, {"b0", "3", 3.0}, {"dist_slew", "5", 5.0},
        {"gain", "1", 1.0}, {"leak", "0.5", 0.5}, {"small", "60", 60.0},
        {"dwell_ms", "1", 1.0}, {"leg_ms", "59999", 59999.0}
    };
    for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index) {
        CyberGearTuning tuning;
        assert(cybergear_tuning_defaults(&tuning));
        assert(cybergear_tuning_set(&tuning, cases[index].key, cases[index].text));
        assert(cybergear_tuning_valid(&tuning));
        assert(fabs(cybergear_tuning_value(&tuning, parameter_index(cases[index].key)) -
            cases[index].expected) <= 1e-6);
    }
}

static void parsing_and_bounds(void)
{
    CyberGearTuning tuning;
    assert(cybergear_tuning_defaults(&tuning));
    const char *invalid_values[] = {"", " ", "1 ", " 1", "1junk", "nan", "NaN", "inf",
        "-inf", "1e100", "1e-100", "+", ".", "1 2", "1\n", "1; s"};
    for (size_t index = 0U; index < sizeof(invalid_values) / sizeof(invalid_values[0]); ++index)
        assert_rejected(&tuning, "wc", invalid_values[index]);
    const char *invalid_integers[] = {"1.5", "1e3", "-1", "+1", "0x10", "4294967296",
        "18446744073709551616", "1000x", "0", "60000"};
    for (size_t index = 0U; index < sizeof(invalid_integers) / sizeof(invalid_integers[0]); ++index)
        assert_rejected(&tuning, "leg_ms", invalid_integers[index]);
    assert_rejected(&tuning, NULL, "2");
    assert_rejected(&tuning, "current", NULL);
    assert_rejected(&tuning, "unknown", "2");
    assert_rejected(&tuning, "CURRENT", "2");
    static const struct { const char *key; const char *text; } invalid_ranges[] = {
        {"current", "0"}, {"current", "6.1"}, {"speed", "0.41"}, {"wc", "20.1"},
        {"zeta", "10.1"}, {"wo", "10.1"}, {"b0", "0.34"}, {"b0", "3.1"},
        {"accel", "1.1"}, {"brake", "1.1"}, {"jerk", "5.1"},
        {"rise", "5.1"}, {"fall", "5.1"}, {"rise", "2.5"}, {"fall", "2.5"},
        {"dist_fraction", "0.6"}, {"dist_fraction", "-0.1"},
        {"reserve_fraction", "0.4"}, {"reserve_fraction", "1"},
        {"dist_slew", "5.1"}, {"dist_slew", "0"}, {"reserve_slew", "5"},
        {"gain", "1.1"}, {"gain", "-0.1"}, {"leak", "0.51"}, {"leak", "-0.01"},
        {"amp", "61"}, {"amp", "4"}, {"amp", "0"}, {"small", "61"},
        {"small", "0"}, {"dwell_ms", "20000"}, {"leg_ms", "1000"}
    };
    for (size_t index = 0U; index < sizeof(invalid_ranges) / sizeof(invalid_ranges[0]); ++index)
        assert_rejected(&tuning, invalid_ranges[index].key, invalid_ranges[index].text);
    assert(cybergear_tuning_set(&tuning, "gain", "0"));
    assert(cybergear_tuning_set(&tuning, "leak", "0"));
    assert(cybergear_tuning_set(&tuning, "reserve_slew", "0"));
    assert(cybergear_tuning_set(&tuning, "dist_fraction", "0"));
    assert(cybergear_tuning_set(&tuning, "reserve_fraction", "0"));
    assert(cybergear_tuning_valid(&tuning));
}

static void derived_budgets(void)
{
    CyberGearTuning tuning;
    assert(cybergear_tuning_defaults(&tuning));
    assert(cybergear_tuning_set(&tuning, "dist_fraction", "0.4"));
    assert(cybergear_tuning_set(&tuning, "reserve_fraction", "0.6"));
    assert(cybergear_tuning_set(&tuning, "current", "2"));
    assert(fabsf(tuning.motor.controller.disturbance_limit_a - 0.8f) < 1e-6f);
    assert(fabsf(tuning.motor.dynamics.reserve_current_a - 1.2f) < 1e-6f);
    assert(tuning.motor.dynamics.acceleration_current_a == 2.0f);
    assert(tuning.motor.dynamics.braking_current_a == 2.0f);
    assert(fabsf(tuning.motor.stall_current_a - 1.6f) < 1e-6f);
    assert(cybergear_tuning_set(&tuning, "b0", "1.5"));
    assert(tuning.motor.dynamics.fixed_b0 == 1.5f);
    assert(tuning.motor.dynamics.b0_min == (float)CYBERGEAR_B0_MIN);
    assert(tuning.motor.dynamics.b0_max == (float)CYBERGEAR_B0_MAX);
    assert(cybergear_tuning_set(&tuning, "accel", "0.6"));
    assert(cybergear_tuning_set(&tuning, "brake", "0.7"));
    assert(cybergear_tuning_set(&tuning, "jerk", "2"));
    assert(tuning.motor.dynamics.acceleration_cap_rad_s2 == 0.6f);
    assert(tuning.motor.dynamics.braking_cap_rad_s2 == 0.7f);
    assert(tuning.motor.dynamics.jerk_cap_rad_s3 == 2.0f);
    assert(cybergear_tuning_set(&tuning, "rise", "4"));
    assert(cybergear_tuning_set(&tuning, "fall", "3"));
    assert(tuning.motor.dynamics.current_slew_a_s == 3.0f);
    const CyberGearConfig before_rejection = tuning.motor;
    assert_rejected(&tuning, "reserve_slew", "3");
    assert(memcmp(&tuning.motor, &before_rejection, sizeof(tuning.motor)) == 0);
    tuning.motor.dynamics.fixed_b0 = 1.0f;
    assert(!cybergear_tuning_valid(&tuning));
    assert_rejected(&tuning, "wc", "3");
}

void tuning_tests(void)
{
    defaults_and_metadata();
    accepted_parameters();
    parsing_and_bounds();
    derived_budgets();
    puts("Runtime tuning: strict parsing, transactional changes, bounds and derived budgets passed.");
}
