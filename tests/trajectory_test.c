#include "cybergear_trajectory.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static CgTrajectoryLimits standard_limits(void)
{
    const CgTrajectoryLimits l = {
        .position_min_rad = -3.0f, .position_max_rad = 3.0f,
        .velocity_max_rad_s = 1.5f, .acceleration_max_rad_s2 = 3.0f,
        .braking_max_rad_s2 = 3.0f, .jerk_max_rad_s3 = 15.0f,
        .duration_min_s = 0.01f, .duration_max_s = 30.0f,
        .target_tolerance_rad = 1.0e-5f, .search_iterations = 96u
    };
    return l;
}

static uint32_t random_state = 0x8aee45d1u;
static float random_unit(void)
{
    random_state = random_state * 1664525u + 1013904223u;
    return (float)(random_state >> 8u) / 16777216.0f;
}

static void assert_dense_bounds(const CgTrajectory *t, const CgTrajectoryLimits *l)
{
    assert(cg_trajectory_validate(t, l));
    const float acceleration = fminf(l->acceleration_max_rad_s2, l->braking_max_rad_s2);
    for (unsigned i = 0u; i <= 2048u; ++i) {
        CgTrajectoryPoint p;
        assert(cg_trajectory_evaluate(t, t->duration_s * i / 2048.0, &p));
        assert(p.q_rad >= l->position_min_rad - 2.0e-6f && p.q_rad <= l->position_max_rad + 2.0e-6f);
        assert(fabsf(p.v_rad_s) <= l->velocity_max_rad_s + 2.0e-6f);
        assert(fabsf(p.a_rad_s2) <= acceleration + 2.0e-6f);
        assert(fabsf(p.jerk_rad_s3) <= l->jerk_max_rad_s3 + 2.0e-5f);
    }
    CgTrajectoryPoint end;
    assert(cg_trajectory_evaluate(t, t->duration_s + 1.0, &end));
    assert(end.q_rad == t->target_rad && end.v_rad_s == 0.0f && end.a_rad_s2 == 0.0f && end.jerk_rad_s3 == 0.0f);
}

int test_trajectory(void)
{
    CgTrajectoryLimits limits = standard_limits();
    CgTrajectory trajectory;
    CgTrajectoryPoint point = {0};
    assert(cg_trajectory_reset(&trajectory, &point, &limits));
    assert(cg_trajectory_plan(&trajectory, 1.57079632679f, &limits));
    /* Independent analytic rest-to-rest duration; guards an overly conservative
     * or incorrect derivative implementation as well as the maximum bounds. */
    const double distance = trajectory.target_rad;
    const double expected = fmax(1.875 * distance / 1.5,
                                fmax(sqrt((10.0 * sqrt(3.0) / 3.0) * distance / 3.0), cbrt(60.0 * distance / 15.0)));
    assert(fabs(trajectory.duration_s - expected) < 1.0e-8);
    assert_dense_bounds(&trajectory, &limits);
    assert(cg_trajectory_advance(&trajectory, 0.2f, &point));
    CgTrajectory old = trajectory;
    for (unsigned i = 0u; i < 100u; ++i)
        assert(cg_trajectory_plan(&trajectory, trajectory.target_rad, &limits));
    assert(memcmp(&trajectory, &old, sizeof(old)) == 0);

    /* Reversal starts from the generated q/v/a, never the measured position. */
    const CgTrajectoryPoint before_reversal = point;
    assert(cg_trajectory_plan(&trajectory, -0.7f, &limits));
    assert(cg_trajectory_evaluate(&trajectory, 0.0, &point));
    assert(point.q_rad == before_reversal.q_rad && point.v_rad_s == before_reversal.v_rad_s && point.a_rad_s2 == before_reversal.a_rad_s2);
    assert_dense_bounds(&trajectory, &limits);

    /* Tightening constraints on an unchanged destination must replan. */
    assert(cg_trajectory_advance(&trajectory, 0.1f, &point));
    const CgTrajectoryPoint before_limits = point;
    limits.velocity_max_rad_s = 1.0f;
    assert(cg_trajectory_plan(&trajectory, trajectory.target_rad, &limits));
    assert(trajectory.elapsed_s == 0.0);
    assert(cg_trajectory_evaluate(&trajectory, 0.0, &point));
    assert(point.q_rad == before_limits.q_rad && point.v_rad_s == before_limits.v_rad_s && point.a_rad_s2 == before_limits.a_rad_s2);
    assert_dense_bounds(&trajectory, &limits);

    /* Strictly invalid input/failure must be transactional. */
    old = trajectory;
    assert(!cg_trajectory_plan(&trajectory, NAN, &limits));
    assert(!cg_trajectory_plan(&trajectory, 4.0f, &limits));
    assert(!cg_trajectory_advance(&trajectory, NAN, &point));
    assert(!cg_trajectory_advance(&trajectory, -0.1f, &point));
    assert(memcmp(&trajectory, &old, sizeof(old)) == 0);
    CgTrajectoryLimits invalid = limits;
    invalid.search_iterations = 257u;
    assert(!cg_trajectory_plan(&trajectory, 0.0f, &invalid));
    invalid = limits; invalid.duration_max_s = INFINITY;
    assert(!cg_trajectory_plan(&trajectory, 0.0f, &invalid));
    assert(memcmp(&trajectory, &old, sizeof(old)) == 0);

    /* A boundary-directed initial velocity cannot be made safe by an arbitrarily
     * long duration. Reject without clipping or silently dropping velocity. */
    limits = standard_limits();
    point = (CgTrajectoryPoint){.q_rad = 3.0f, .v_rad_s = 0.3f};
    assert(cg_trajectory_reset(&trajectory, &point, &limits));
    old = trajectory;
    assert(!cg_trajectory_plan(&trajectory, 2.0f, &limits));
    assert(memcmp(&trajectory, &old, sizeof(old)) == 0);
    point = (CgTrajectoryPoint){0};
    assert(cg_trajectory_reset(&trajectory, &point, &limits));
    limits.duration_max_s = 0.011f;
    old = trajectory;
    assert(!cg_trajectory_plan(&trajectory, 2.0f, &limits));
    assert(memcmp(&trajectory, &old, sizeof(old)) == 0);

    /* Analytic extrema must see an interior overshoot with legal endpoints:
     * q=2.9+2*s*(1-s), max=3.4. This is intentionally not a valid quintic
     * endpoint profile but must still be rejected by the safety validator. */
    limits = standard_limits();
    point = (CgTrajectoryPoint){.q_rad = 2.9f};
    assert(cg_trajectory_reset(&trajectory, &point, &limits));
    trajectory.duration_s = 10.0;
    trajectory.coefficients[1] = 2.0;
    trajectory.coefficients[2] = -2.0;
    assert(!cg_trajectory_validate(&trajectory, &limits));

    /* A genuine fifth-degree curve with four distinct internal velocity roots.
     * Its local maxima exceed both endpoints. Test a limit just below the
     * independently sampled maximum, then a generous containing limit. */
    limits.position_min_rad = -100.0f;
    limits.position_max_rad = 100.0f;
    limits.velocity_max_rad_s = 100.0f;
    limits.acceleration_max_rad_s2 = 100.0f;
    limits.braking_max_rad_s2 = 100.0f;
    limits.jerk_max_rad_s3 = 100.0f;
    point = (CgTrajectoryPoint){0};
    assert(cg_trajectory_reset(&trajectory, &point, &limits));
    trajectory.duration_s = 10.0;
    /* q'(s) = 100*(s-.1)*(s-.3)*(s-.6)*(s-.9). */
    trajectory.coefficients[1] = 1.62;
    trajectory.coefficients[2] = -13.05;
    trajectory.coefficients[3] = 39.0;
    trajectory.coefficients[4] = -47.5;
    trajectory.coefficients[5] = 20.0;
    trajectory.target_rad = (float)(1.62 - 13.05 + 39.0 - 47.5 + 20.0);
    float sample_max = -100.0f;
    for (unsigned i = 0u; i < 20000u; ++i) {
        assert(cg_trajectory_evaluate(&trajectory, trajectory.duration_s * i / 20000.0, &point));
        sample_max = fmaxf(sample_max, point.q_rad);
    }
    assert(sample_max > trajectory.target_rad + 0.01f);
    assert(cg_trajectory_validate(&trajectory, &limits));
    limits.position_max_rad = sample_max - 1.0e-4f;
    assert(!cg_trajectory_validate(&trajectory, &limits));

    /* Random coefficients provide an independent check of all derivative
     * orders, separate from the quintic endpoint coefficient generator. */
    for (unsigned trial = 0u; trial < 200u; ++trial) {
        limits = standard_limits();
        point = (CgTrajectoryPoint){0};
        assert(cg_trajectory_reset(&trajectory, &point, &limits));
        trajectory.duration_s = 1.0 + random_unit() * 3.0;
        double endpoint = 0.0;
        for (unsigned i = 1u; i < 6u; ++i) {
            trajectory.coefficients[i] = (random_unit() * 2.0 - 1.0) * 2.0;
            endpoint += trajectory.coefficients[i];
        }
        trajectory.target_rad = (float)endpoint;
        if (cg_trajectory_validate(&trajectory, &limits)) {
            for (unsigned i = 0u; i < 4096u; ++i) {
                assert(cg_trajectory_evaluate(&trajectory, trajectory.duration_s * i / 4096.0, &point));
                assert(point.q_rad >= limits.position_min_rad - 2.0e-6f && point.q_rad <= limits.position_max_rad + 2.0e-6f);
                assert(fabsf(point.v_rad_s) <= limits.velocity_max_rad_s + 2.0e-6f);
                assert(fabsf(point.a_rad_s2) <= limits.acceleration_max_rad_s2 + 2.0e-6f);
                assert(fabsf(point.jerk_rad_s3) <= limits.jerk_max_rad_s3 + 2.0e-5f);
            }
        }
    }

    /* Independent dense comparison for different starting velocities,
     * accelerations, signs, distances, and asymmetric braking capability. */
    unsigned planned = 0u, failed = 0u;
    for (unsigned trial = 0u; trial < 400u; ++trial) {
        limits = standard_limits();
        limits.braking_max_rad_s2 = 1.5f + random_unit() * 1.5f;
        point = (CgTrajectoryPoint){
            .q_rad = (random_unit() * 2.0f - 1.0f) * 2.5f,
            .v_rad_s = (random_unit() * 2.0f - 1.0f) * 1.2f,
            .a_rad_s2 = (random_unit() * 2.0f - 1.0f) * 1.4f
        };
        const float target = (random_unit() * 2.0f - 1.0f) * 2.5f;
        assert(cg_trajectory_reset(&trajectory, &point, &limits));
        old = trajectory;
        if (cg_trajectory_plan(&trajectory, target, &limits)) {
            ++planned;
            CgTrajectoryPoint start;
            assert(cg_trajectory_evaluate(&trajectory, 0.0, &start));
            assert(start.q_rad == point.q_rad && start.v_rad_s == point.v_rad_s && start.a_rad_s2 == point.a_rad_s2);
            assert(trajectory.plan_iterations <= limits.search_iterations);
            assert_dense_bounds(&trajectory, &limits);
            /* Move and replan at an off-grid instant, then check C2 continuity. */
            assert(cg_trajectory_advance(&trajectory, (float)(0.137 * trajectory.duration_s), &point));
            old = trajectory;
            if (cg_trajectory_plan(&trajectory, -target * 0.5f, &limits)) {
                assert(cg_trajectory_evaluate(&trajectory, 0.0, &start));
                assert(start.q_rad == point.q_rad && start.v_rad_s == point.v_rad_s && start.a_rad_s2 == point.a_rad_s2);
                assert_dense_bounds(&trajectory, &limits);
            } else assert(memcmp(&trajectory, &old, sizeof(old)) == 0);
        } else {
            ++failed;
            assert(memcmp(&trajectory, &old, sizeof(old)) == 0);
        }
    }
    assert(planned > 300u);
    assert(planned + failed == 400u);
    return 0;
}
