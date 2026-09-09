#include "cybergear_calibration.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static CgCalConfig fixture_config(void)
{
    CgCalConfig c;
    cg_cal_config_defaults(&c);
    c.armed = true;
    c.pulse_current_a = 0.1f;
    c.brake_current_a = 0.1f;
    c.current_limit_a = 0.2f;
    c.temp_trip_c = 50.0f;
    c.speed_trip_rad_s = 0.5f;
    assert(cg_cal_config_valid(&c));
    return c;
}

static CgCalFeedback fixture_feedback(float origin, uint32_t now)
{
    const CgCalFeedback f = {origin, 0.0f, 25.0f, now, 1U, true, false};
    return f;
}

static void start_fixture(CgCal *cal, CgCalFeedback *f, uint32_t now)
{
    const CgCalConfig c = fixture_config();
    *f = fixture_feedback(0.0f, now);
    cg_cal_init(cal);
    assert(cg_cal_start(cal, &c, 0.0f, f, now));
}

static CgCalOutput next_step(CgCal *cal, CgCalFeedback *f, uint32_t *now)
{
    *now += 10U;
    f->rx_ms = *now;
    ++f->rx_sequence;
    return cg_cal_step(cal, f, *now);
}

static void advance_to_pulse(CgCal *cal, CgCalFeedback *f, uint32_t *now)
{
    for (unsigned int i = 0U; i < 100U; ++i) {
        CgCalOutput out = next_step(cal, f, now);
        assert(!out.stop && out.write_current);
        assert(cg_cal_commit(cal, &out, true));
        if (cal->phase == CG_CAL_PHASE_PULSE) return;
    }
    assert(!"pulse not reached");
}

static void test_configuration_and_origin(void)
{
    CgCalConfig c;
    CgCal cal;
    CgCalFeedback f = fixture_feedback(1.0f, 0U);
    cg_cal_config_defaults(&c);
    assert(!c.armed && isnan(c.pulse_current_a));
    assert(!cg_cal_config_valid(&c));
    c = fixture_config();
    c.travel_hard_rad = CG_CAL_ABSOLUTE_TRAVEL_RAD;
    assert(!cg_cal_config_valid(&c));
    c.travel_hard_rad += 0.01f;
    assert(!cg_cal_config_valid(&c));
    c = fixture_config();
    c.pulse_current_a = c.current_limit_a + 0.01f;
    assert(!cg_cal_config_valid(&c));
    c = fixture_config();
    c.brake_current_a = -0.1f;
    assert(!cg_cal_config_valid(&c));
    c = fixture_config();
    c.pulse_ms = 10000U;
    assert(!cg_cal_config_valid(&c));
    c = fixture_config();
    c.speed_trip_rad_s = INFINITY;
    assert(!cg_cal_config_valid(&c));
    c = fixture_config();
    cg_cal_init(&cal);
    assert(cg_cal_start(&cal, &c, 1.0f, &f, 0U));
    assert(!cg_cal_start(&cal, &c, 0.0f, &f, 0U));
    assert(cal.initial_position_rad == 1.0f);
    cg_cal_abort(&cal, CG_CAL_FAULT_REQUESTED_STOP);
    cg_cal_confirm_stopped(&cal);
    assert(cal.phase == CG_CAL_PHASE_FAULT);
    assert(cal.initial_position_rad == 1.0f);
    assert(!cg_cal_start(&cal, &c, 0.0f, &f, 0U));
}

static void test_synthetic_trial(float input_sign, float plant_polarity, uint32_t initial_ms)
{
    CgCal cal;
    CgCalConfig c = fixture_config();
    c.pulse_current_a *= input_sign;
    CgCalFeedback f = fixture_feedback(0.7f, initial_ms);
    uint32_t now = initial_ms;
    cg_cal_init(&cal);
    assert(cg_cal_start(&cal, &c, 0.7f, &f, now));
    bool saw_pulse = false, saw_brake = false, saw_coast = false;
    unsigned int pulse_starts = 0U;
    CgCalPhase previous_phase = cal.phase;
    for (unsigned int i = 0U; i < 200U; ++i) {
        /* Synthetic inertia only: alpha = signed b0 * last QUEUED current.
         * This validates sequencing/envelopes, not installed robot physics. */
        f.v_rad_s += plant_polarity * 10.0f * cal.last_current_a * 0.01f;
        f.q_rad += f.v_rad_s * 0.01f;
        CgCalOutput out = next_step(&cal, &f, &now);
        assert(fabsf(f.q_rad - cal.initial_position_rad) < CG_CAL_ABSOLUTE_TRAVEL_RAD);
        assert(cal.initial_position_rad == 0.7f);
        if (cal.phase == CG_CAL_PHASE_PULSE && previous_phase != cal.phase) ++pulse_starts;
        previous_phase = cal.phase;
        if (out.stop) {
            assert(cal.fault == CG_CAL_FAULT_NONE);
            assert(cal.phase == CG_CAL_PHASE_STOPPING);
            cg_cal_confirm_stopped(&cal);
            break;
        }
        assert(out.write_current && !out.stop);
        assert(fabsf(out.current_a) <= c.current_limit_a);
        if (cal.phase == CG_CAL_PHASE_PULSE) {
            saw_pulse = true;
            assert(out.current_a * input_sign > 0.0f);
        } else if (cal.phase == CG_CAL_PHASE_BRAKE) {
            saw_brake = true;
            assert(out.current_a * input_sign < 0.0f);
        } else {
            assert(out.current_a == 0.0f);
            if (cal.phase == CG_CAL_PHASE_COAST) saw_coast = true;
        }
        assert(cg_cal_commit(&cal, &out, true));
    }
    assert(cal.phase == CG_CAL_PHASE_DONE && saw_pulse && saw_brake && saw_coast);
    assert(pulse_starts == 1U);
    const CgCalOutput final = cg_cal_step(&cal, &f, now + 10000U);
    assert(!final.write_current && !final.stop);
}

static void test_feedback_faults(void)
{
    CgCal cal;
    CgCalFeedback f;
    for (unsigned int case_no = 0U; case_no < 12U; ++case_no) {
        start_fixture(&cal, &f, 0U);
        f.rx_ms = 10U;
        f.rx_sequence = 2U;
        CgCalFault expected = CG_CAL_FAULT_FEEDBACK;
        switch (case_no) {
        case 0: f.rx_ms = UINT32_MAX - 100U; break; /* stale */
        case 1: f.rx_ms = 11U; break;              /* future */
        case 2: f.rx_sequence = 1U; break;       /* replay with new timestamp */
        case 3: f.rx_sequence = 0U; break;       /* sequence went backward */
        case 4: f.rx_ms = UINT32_MAX; break;     /* RX timestamp went backward */
        case 5: f.q_rad = NAN; break;
        case 6: f.online = false; break;
        case 7: f.motor_fault = true; expected = CG_CAL_FAULT_MOTOR; break;
        case 8: f.v_rad_s = 0.5f; expected = CG_CAL_FAULT_SPEED; break;
        case 9: f.temp_c = 50.0f; expected = CG_CAL_FAULT_TEMPERATURE; break;
        case 10: f.q_rad = 0.2f; expected = CG_CAL_FAULT_POSITION; break;
        case 11: f.q_rad = 0.002f; f.v_rad_s = 0.03f; expected = CG_CAL_FAULT_BASELINE_MOTION; break;
        default: assert(0); break;
        }
        const CgCalOutput out = cg_cal_step(&cal, &f, 10U);
        assert(out.stop && !out.write_current && cal.fault == expected);
        assert(cal.last_current_a == 0.0f);
    }
    start_fixture(&cal, &f, 0U);
    f.v_rad_s = 0.01f; /* Unchanged sequence cannot carry a new velocity. */
    CgCalOutput out = cg_cal_step(&cal, &f, 10U);
    assert(out.stop && cal.fault == CG_CAL_FAULT_FEEDBACK);

    start_fixture(&cal, &f, 0U);
    f.rx_sequence = 2U;
    f.rx_ms = 21U;
    out = cg_cal_step(&cal, &f, 21U);
    assert(out.stop && cal.fault == CG_CAL_FAULT_TIMING);

    /* A truly identical sample remains usable only until the original RX
     * deadline; repeated step calls never refresh its age. */
    start_fixture(&cal, &f, 0U);
    for (uint32_t now = 10U; now <= 40U; now += 10U) {
        out = cg_cal_step(&cal, &f, now);
        if (now <= 30U) assert(cg_cal_commit(&cal, &out, true));
        else assert(out.stop && cal.fault == CG_CAL_FAULT_FEEDBACK);
    }
}

static void test_travel_guards(void)
{
    const CgCalConfig c = fixture_config();
    for (int sign = -1; sign <= 1; sign += 2) {
        CgCal cal;
        CgCalFeedback f = fixture_feedback((float)sign * 0.12f, 0U);
        cg_cal_init(&cal);
        assert(cg_cal_start(&cal, &c, 0.0f, &f, 0U));
        f.q_rad = (float)sign * 0.125f;
        f.v_rad_s = (float)sign * 0.2f;
        uint32_t now = 0U;
        CgCalOutput out = next_step(&cal, &f, &now);
        assert(out.stop && !out.write_current && cal.fault == CG_CAL_FAULT_POSITION);
        assert(fabsf(f.q_rad) < c.travel_soft_rad); /* Predictive trip, before soft boundary. */

        start_fixture(&cal, &f, 0U);
        f.q_rad = (float)sign * CG_CAL_ABSOLUTE_TRAVEL_RAD;
        now = 0U;
        out = next_step(&cal, &f, &now);
        assert(out.stop && !out.write_current && cal.fault == CG_CAL_FAULT_POSITION);
        const CgCalOutput repeat = cg_cal_step(&cal, &f, now + 10U);
        assert(repeat.stop && !repeat.write_current);
    }
}

static void test_command_failures_and_latches(void)
{
    CgCal cal;
    CgCalFeedback f;
    uint32_t now = 0U;
    start_fixture(&cal, &f, now);
    advance_to_pulse(&cal, &f, &now);
    const float previously_queued = cal.last_current_a;
    CgCalOutput out = next_step(&cal, &f, &now);
    assert(!cg_cal_commit(&cal, &out, false));
    assert(cal.fault == CG_CAL_FAULT_TX && cal.last_current_a == previously_queued);
    out = next_step(&cal, &f, &now);
    assert(out.stop && !out.write_current);
    cg_cal_abort(&cal, CG_CAL_FAULT_REQUESTED_STOP);
    assert(cal.fault == CG_CAL_FAULT_TX); /* first fault remains visible */

    now = 0U;
    start_fixture(&cal, &f, now);
    out = next_step(&cal, &f, &now);
    assert(out.write_current);
    out = next_step(&cal, &f, &now); /* missing commit */
    assert(out.stop && cal.fault == CG_CAL_FAULT_TX);

    now = 0U;
    start_fixture(&cal, &f, now);
    out = next_step(&cal, &f, &now);
    ++out.generation;
    assert(!cg_cal_commit(&cal, &out, true));
    assert(cal.fault == CG_CAL_FAULT_TX);

    now = 0U;
    start_fixture(&cal, &f, now);
    cg_cal_abort(&cal, CG_CAL_FAULT_LOG_OVERFLOW);
    out = next_step(&cal, &f, &now);
    assert(out.stop && !out.write_current && cal.fault == CG_CAL_FAULT_LOG_OVERFLOW);
}

static void test_brake_timeout_and_reversal(void)
{
    for (unsigned int reversal = 0U; reversal < 2U; ++reversal) {
        CgCal cal;
        CgCalFeedback f;
        uint32_t now = 0U;
        start_fixture(&cal, &f, now);
        advance_to_pulse(&cal, &f, &now);
        f.v_rad_s = 0.1f;
        for (unsigned int i = 0U; i < 60U; ++i) {
            CgCalOutput out = next_step(&cal, &f, &now);
            if (cal.phase == CG_CAL_PHASE_BRAKE && reversal) {
                assert(cg_cal_commit(&cal, &out, true));
                f.v_rad_s = -0.01f;
                out = next_step(&cal, &f, &now);
                assert(cal.phase == CG_CAL_PHASE_COAST && out.current_a == 0.0f);
                assert(cg_cal_commit(&cal, &out, true));
                break;
            }
            if (out.stop) {
                assert(!reversal && cal.fault == CG_CAL_FAULT_BRAKE_TIMEOUT);
                break;
            }
            assert(cg_cal_commit(&cal, &out, true));
        }
        if (!reversal) assert(cal.fault == CG_CAL_FAULT_BRAKE_TIMEOUT);
    }
}

static void test_baseline_noise_retry(void)
{
    CgCal cal;
    CgCalFeedback f;
    uint32_t now = UINT32_MAX - 50U;
    start_fixture(&cal, &f, now);
    cal.config.stationary_speed_rad_s = 0.1f;
    f.v_rad_s = 0.1203938f;
    f.q_rad = 0.0003815f;
    CgCalOutput out = next_step(&cal, &f, &now);
    assert(!out.stop && out.write_current && out.current_a == 0.0f);
    assert(cg_cal_commit(&cal, &out, true));
    f.v_rad_s = 0.0f;
    for (unsigned i=0; i<19; ++i) {
        out = next_step(&cal, &f, &now);
        assert(!out.stop && cal.phase == CG_CAL_PHASE_BASELINE && out.current_a == 0.0f);
        assert(cg_cal_commit(&cal, &out, true));
    }
    /* An unchanged measurement must not authorize the pulse at the boundary. */
    now += 10U;
    out = cg_cal_step(&cal, &f, now);
    assert(!out.stop && cal.phase == CG_CAL_PHASE_BASELINE && out.current_a == 0.0f);
    assert(cg_cal_commit(&cal, &out, true));
    out = next_step(&cal, &f, &now);
    assert(!out.stop && cal.phase == CG_CAL_PHASE_PULSE);
    assert(cg_cal_commit(&cal, &out, true));

    /* Repeated velocity spikes cannot extend the wait indefinitely. */
    now = 0U;
    start_fixture(&cal, &f, now);
    f.v_rad_s = 0.03f;
    for (unsigned i=0; i<120; ++i) {
        out = next_step(&cal, &f, &now);
        if (out.stop) break;
        assert(cal.phase == CG_CAL_PHASE_BASELINE && out.current_a == 0.0f);
        assert(cg_cal_commit(&cal, &out, true));
    }
    assert(out.stop && cal.fault == CG_CAL_FAULT_BASELINE_MOTION);

    /* Position drift is checked even with reported speed zero. */
    now = 0U;
    start_fixture(&cal, &f, now);
    f.q_rad = 0.0011f;
    out = next_step(&cal, &f, &now);
    assert(out.stop && cal.fault == CG_CAL_FAULT_BASELINE_MOTION);
}

static void test_coast_noise_retry(void)
{
    CgCal cal;
    CgCalFeedback f;
    uint32_t now=0U;
    start_fixture(&cal,&f,now);
    cal.phase=CG_CAL_PHASE_COAST;
    cal.phase_ms=now;
    cal.coast_started_ms=now;
    f.v_rad_s=0.03f;
    CgCalOutput out=next_step(&cal,&f,&now);
    assert(!out.stop && out.write_current && out.current_a == 0.0f);
    assert(cg_cal_commit(&cal,&out,true));
    f.v_rad_s=0.0f;
    for (unsigned i=0;i<20;++i) {
        out=next_step(&cal,&f,&now);
        if (out.stop) break;
        assert(out.current_a == 0.0f);
        assert(cg_cal_commit(&cal,&out,true));
    }
    assert(out.stop && cal.fault == CG_CAL_FAULT_NONE && cal.phase == CG_CAL_PHASE_STOPPING);
}

void test_calibration(void)
{
    test_configuration_and_origin();
    test_baseline_noise_retry();
    test_coast_noise_retry();
    test_synthetic_trial(1.0f, 1.0f, 0U);
    test_synthetic_trial(-1.0f, 1.0f, 0U);
    test_synthetic_trial(1.0f, -1.0f, UINT32_MAX - 50U);
    test_synthetic_trial(-1.0f, -1.0f, UINT32_MAX - 50U);
    test_feedback_faults();
    test_travel_guards();
    test_command_failures_and_latches();
    test_brake_timeout_and_reversal();
    assert(strcmp(cg_cal_phase_name(CG_CAL_PHASE_BRAKE), "BRAKE") == 0);
    assert(strcmp(cg_cal_fault_name(CG_CAL_FAULT_TX), "TX") == 0);
    puts("calibration: limited one-shot trials and fault guards passed");
}
