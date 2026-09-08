#include "cybergear_trajectory.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Degree is at most four. Recursively partitioning at derivative roots gives
 * monotone intervals; each sign-changing interval contains exactly one root.
 * This checks continuous-time extrema, not a sample grid. All loops are bounded. */
#define CG_ROOT_BISECTIONS 56u
#define CG_MAX_PLAN_ITERATIONS 256u

static double min_double(double a, double b) { return a < b ? a : b; }
static double max_double(double a, double b) { return a > b ? a : b; }

static bool limits_valid(const CgTrajectoryLimits *l)
{
    return l != NULL && isfinite(l->position_min_rad) && isfinite(l->position_max_rad) &&
           l->position_min_rad < l->position_max_rad &&
           isfinite(l->velocity_max_rad_s) && l->velocity_max_rad_s > 0.0f &&
           isfinite(l->acceleration_max_rad_s2) && l->acceleration_max_rad_s2 > 0.0f &&
           isfinite(l->braking_max_rad_s2) && l->braking_max_rad_s2 > 0.0f &&
           isfinite(l->jerk_max_rad_s3) && l->jerk_max_rad_s3 > 0.0f &&
           isfinite(l->duration_min_s) && l->duration_min_s >= 1.0e-6f &&
           isfinite(l->duration_max_s) && l->duration_max_s >= l->duration_min_s &&
           isfinite(l->target_tolerance_rad) && l->target_tolerance_rad >= 0.0f &&
           l->search_iterations >= 2u && l->search_iterations <= CG_MAX_PLAN_ITERATIONS;
}

static bool point_valid(const CgTrajectoryPoint *p, const CgTrajectoryLimits *l)
{
    return p != NULL && isfinite(p->q_rad) && isfinite(p->v_rad_s) &&
           isfinite(p->a_rad_s2) && isfinite(p->jerk_rad_s3) &&
           p->q_rad >= l->position_min_rad && p->q_rad <= l->position_max_rad &&
           fabs((double)p->v_rad_s) <= l->velocity_max_rad_s &&
           fabs((double)p->a_rad_s2) <= min_double(l->acceleration_max_rad_s2, l->braking_max_rad_s2);
}

static bool same_limits(const CgTrajectoryLimits *a, const CgTrajectoryLimits *b)
{
    return a->position_min_rad == b->position_min_rad && a->position_max_rad == b->position_max_rad &&
           a->velocity_max_rad_s == b->velocity_max_rad_s &&
           a->acceleration_max_rad_s2 == b->acceleration_max_rad_s2 &&
           a->braking_max_rad_s2 == b->braking_max_rad_s2 && a->jerk_max_rad_s3 == b->jerk_max_rad_s3 &&
           a->duration_min_s == b->duration_min_s && a->duration_max_s == b->duration_max_s &&
           a->target_tolerance_rad == b->target_tolerance_rad && a->search_iterations == b->search_iterations;
}

static double polynomial(const double *c, unsigned degree, double x)
{
    double value = c[degree];
    while (degree > 0u) value = value * x + c[--degree];
    return value;
}

static unsigned derivative(const double *c, unsigned degree, double *d)
{
    if (degree == 0u) { d[0] = 0.0; return 0u; }
    for (unsigned i = 1u; i <= degree; ++i) d[i - 1u] = (double)i * c[i];
    return degree - 1u;
}

static void add_root(double *roots, unsigned *count, unsigned capacity, double root)
{
    if (root < 0.0 || root > 1.0 || *count >= capacity) return;
    /* Called in ascending order. Do not merge nearby distinct roots. */
    if (*count == 0u || root > roots[*count - 1u]) roots[(*count)++] = root;
}

static unsigned roots_unit_interval(const double *c, unsigned degree, double *roots)
{
    while (degree > 0u && c[degree] == 0.0) --degree;
    if (degree == 0u) return 0u;
    if (degree == 1u) {
        const double root = -c[0] / c[1];
        if (root >= 0.0 && root <= 1.0 && isfinite(root)) { roots[0] = root; return 1u; }
        return 0u;
    }
    double d[4] = {0.0}, critical[3] = {0.0}, bounds[5] = {0.0};
    const unsigned d_degree = derivative(c, degree, d);
    const unsigned critical_count = roots_unit_interval(d, d_degree, critical);
    unsigned bound_count = 1u;
    for (unsigned i = 0u; i < critical_count; ++i)
        if (critical[i] > 0.0 && critical[i] < 1.0) bounds[bound_count++] = critical[i];
    bounds[bound_count++] = 1.0;
    unsigned count = 0u;
    for (unsigned i = 0u; i + 1u < bound_count; ++i) {
        double left = bounds[i], right = bounds[i + 1u];
        double fl = polynomial(c, degree, left), fr = polynomial(c, degree, right);
        if (fl == 0.0) add_root(roots, &count, degree, left);
        if ((fl < 0.0 && fr > 0.0) || (fl > 0.0 && fr < 0.0)) {
            for (unsigned step = 0u; step < CG_ROOT_BISECTIONS; ++step) {
                const double middle = 0.5 * (left + right);
                const double fm = polynomial(c, degree, middle);
                if (fm == 0.0) { left = middle; right = middle; break; }
                if ((fl < 0.0 && fm < 0.0) || (fl > 0.0 && fm > 0.0)) { left = middle; fl = fm; }
                else right = middle;
            }
            add_root(roots, &count, degree, 0.5 * (left + right));
        }
    }
    if (polynomial(c, degree, 1.0) == 0.0) add_root(roots, &count, degree, 1.0);
    /* An even-multiplicity root is not a sign change and may be rounded away.
     * Such a root cannot be a strict extremum of the polynomial being checked
     * by its parent. To cover near multiple roots conservatively, the bounds
     * validator below also evaluates every derivative critical point. */
    return count;
}

static bool within(double value, double lower, double upper, double roundoff)
{
    return isfinite(value) && value >= lower - roundoff && value <= upper + roundoff;
}

static bool polynomial_bounded(const double *c, unsigned degree, double lower, double upper)
{
    double scale = 0.0;
    for (unsigned i = 0u; i <= degree; ++i) {
        if (!isfinite(c[i])) return false;
        scale += fabs(c[i]);
    }
    /* Reject badly conditioned ranges rather than conceal physical violations
     * behind a tolerance proportional to huge cancelling coefficients. */
    const double tolerance = 64.0 * DBL_EPSILON * max_double(1.0, scale);
    if (!isfinite(scale) || tolerance > 1.0e-8 * max_double(1.0, max_double(fabs(lower), fabs(upper)))) return false;
    if (!within(polynomial(c, degree, 0.0), lower, upper, tolerance) ||
        !within(polynomial(c, degree, 1.0), lower, upper, tolerance)) return false;
    double d[5] = {0.0}, roots[4] = {0.0};
    unsigned d_degree = derivative(c, degree, d);
    unsigned count = roots_unit_interval(d, d_degree, roots);
    for (unsigned i = 0u; i < count; ++i)
        if (!within(polynomial(c, degree, roots[i]), lower, upper, tolerance)) return false;
    /* Near-multiple derivative roots are covered at their critical locations.
     * These extra evaluation points cannot create false acceptance. */
    while (d_degree > 0u) {
        double next[5] = {0.0};
        d_degree = derivative(d, d_degree, next);
        memcpy(d, next, sizeof(d));
        count = roots_unit_interval(d, d_degree, roots);
        for (unsigned i = 0u; i < count; ++i)
            if (!within(polynomial(c, degree, roots[i]), lower, upper, tolerance)) return false;
    }
    return true;
}

bool cg_trajectory_reset(CgTrajectory *t, const CgTrajectoryPoint *initial, const CgTrajectoryLimits *l)
{
    if (t == NULL || !limits_valid(l) || !point_valid(initial, l)) return false;
    CgTrajectory next;
    memset(&next, 0, sizeof(next));
    next.initial = *initial;
    next.initial.jerk_rad_s3 = 0.0f;
    next.point = next.initial;
    next.target_rad = initial->q_rad;
    next.coefficients[0] = initial->q_rad;
    next.limits = *l;
    next.initialized = true;
    *t = next;
    return true;
}

bool cg_trajectory_validate(const CgTrajectory *t, const CgTrajectoryLimits *l)
{
    if (t == NULL || !t->initialized || !limits_valid(l) || !point_valid(&t->initial, l) ||
        !isfinite(t->target_rad) || t->target_rad < l->position_min_rad || t->target_rad > l->position_max_rad ||
        !isfinite(t->duration_s) || t->duration_s < 0.0) return false;
    if (t->duration_s == 0.0)
        return t->initial.q_rad == t->target_rad && t->initial.v_rad_s == 0.0f && t->initial.a_rad_s2 == 0.0f;
    if (t->duration_s < l->duration_min_s || t->duration_s > l->duration_max_s) return false;
    double c[6];
    memcpy(c, t->coefficients, sizeof(c));
    unsigned degree = 5u;
    const double a_max = min_double(l->acceleration_max_rad_s2, l->braking_max_rad_s2);
    const double lower[4] = {l->position_min_rad, -l->velocity_max_rad_s, -a_max, -l->jerk_max_rad_s3};
    const double upper[4] = {l->position_max_rad, l->velocity_max_rad_s, a_max, l->jerk_max_rad_s3};
    for (unsigned order = 0u; order < 4u; ++order) {
        if (!polynomial_bounded(c, degree, lower[order], upper[order])) return false;
        if (order != 3u) {
            double d[6] = {0.0};
            degree = derivative(c, degree, d);
            for (unsigned i = 0u; i <= degree; ++i) c[i] = d[i] / t->duration_s;
        }
    }
    return true;
}

static void make_candidate(CgTrajectory *t, const CgTrajectoryPoint *p, float target, double duration,
                           const CgTrajectoryLimits *limits, uint16_t iterations)
{
    memset(t, 0, sizeof(*t));
    const double distance = (double)target - p->q_rad;
    const double v = (double)p->v_rad_s * duration;
    const double a = (double)p->a_rad_s2 * duration * duration;
    t->coefficients[0] = p->q_rad;
    t->coefficients[1] = v;
    t->coefficients[2] = 0.5 * a;
    t->coefficients[3] = 10.0 * distance - 6.0 * v - 1.5 * a;
    t->coefficients[4] = -15.0 * distance + 8.0 * v + 1.5 * a;
    t->coefficients[5] = 6.0 * distance - 3.0 * v - 0.5 * a;
    t->duration_s = duration;
    t->initial = *p;
    t->point = *p;
    t->limits = *limits;
    t->target_rad = target;
    t->plan_iterations = iterations;
    t->initialized = true;
    t->active = true;
}

bool cg_trajectory_plan(CgTrajectory *t, float target, const CgTrajectoryLimits *l)
{
    if (t == NULL || !t->initialized || !limits_valid(l) || !point_valid(&t->point, l) ||
        !isfinite(target) || target < l->position_min_rad || target > l->position_max_rad) return false;
    const bool rest = t->point.v_rad_s == 0.0f && t->point.a_rad_s2 == 0.0f;
    if ((t->duration_s > 0.0 || rest) && same_limits(l, &t->limits) &&
        fabs((double)target - t->target_rad) <= l->target_tolerance_rad) return true;
    if (rest && target == t->point.q_rad) {
        CgTrajectoryPoint hold = t->point;
        hold.jerk_rad_s3 = 0.0f;
        return cg_trajectory_reset(t, &hold, l);
    }
    const double distance = fabs((double)target - t->point.q_rad);
    const double a_max = min_double(l->acceleration_max_rad_s2, l->braking_max_rad_s2);
    double minimum = max_double(l->duration_min_s, distance / l->velocity_max_rad_s);
    minimum = max_double(minimum, fabs((double)t->point.v_rad_s) / a_max);
    minimum = max_double(minimum, fabs((double)t->point.a_rad_s2) / l->jerk_max_rad_s3);
    if (rest) {
        minimum = max_double(minimum, 1.875 * distance / l->velocity_max_rad_s);
        minimum = max_double(minimum, sqrt((10.0 * sqrt(3.0) / 3.0) * distance / a_max));
        minimum = max_double(minimum, cbrt(60.0 * distance / l->jerk_max_rad_s3));
    }
    if (!isfinite(minimum) || minimum > l->duration_max_s) return false;
    const double logarithmic_span = log((double)l->duration_max_s / minimum);
    for (uint16_t i = 0u; i < l->search_iterations; ++i) {
        const double fraction = (double)i / (double)(l->search_iterations - 1u);
        const double duration = i + 1u == l->search_iterations ? l->duration_max_s : minimum * exp(logarithmic_span * fraction);
        CgTrajectory candidate;
        make_candidate(&candidate, &t->point, target, duration, l, (uint16_t)(i + 1u));
        if (cg_trajectory_validate(&candidate, l)) {
            CgTrajectoryPoint output;
            if (!cg_trajectory_evaluate(&candidate, 0.0, &output)) continue;
            candidate.point = output;
            *t = candidate;
            return true;
        }
    }
    return false;
}

bool cg_trajectory_evaluate(const CgTrajectory *t, double time_s, CgTrajectoryPoint *point)
{
    if (t == NULL || point == NULL || !t->initialized || !isfinite(time_s) || time_s < 0.0 ||
        !isfinite(t->duration_s) || t->duration_s < 0.0) return false;
    CgTrajectoryPoint output = {0};
    if (t->duration_s == 0.0) {
        if (time_s > 0.0 && (t->initial.v_rad_s != 0.0f || t->initial.a_rad_s2 != 0.0f)) return false;
        output = t->initial;
        if (time_s > 0.0) output.jerk_rad_s3 = 0.0f;
    } else if (time_s >= t->duration_s) {
        output.q_rad = t->target_rad;
    } else {
        const double s = time_s / t->duration_s;
        const double *c = t->coefficients;
        const double inv_t = 1.0 / t->duration_s;
        output.q_rad = (float)polynomial(c, 5u, s);
        output.v_rad_s = (float)(((((5.0*c[5]*s + 4.0*c[4])*s + 3.0*c[3])*s + 2.0*c[2])*s + c[1]) * inv_t);
        output.a_rad_s2 = (float)((((20.0*c[5]*s + 12.0*c[4])*s + 6.0*c[3])*s + 2.0*c[2]) * inv_t * inv_t);
        output.jerk_rad_s3 = (float)(((60.0*c[5]*s + 24.0*c[4])*s + 6.0*c[3]) * inv_t * inv_t * inv_t);
        if (time_s == 0.0) {
            output.q_rad = t->initial.q_rad;
            output.v_rad_s = t->initial.v_rad_s;
            output.a_rad_s2 = t->initial.a_rad_s2;
        }
    }
    if (!isfinite(output.q_rad) || !isfinite(output.v_rad_s) ||
        !isfinite(output.a_rad_s2) || !isfinite(output.jerk_rad_s3)) return false;
    *point = output;
    return true;
}

bool cg_trajectory_advance(CgTrajectory *t, float dt_s, CgTrajectoryPoint *point)
{
    if (t == NULL || point == NULL || !isfinite(dt_s) || dt_s <= 0.0f ||
        !isfinite(t->elapsed_s) || t->elapsed_s < 0.0) return false;
    const double next_time = t->elapsed_s + dt_s;
    CgTrajectoryPoint output;
    if (!cg_trajectory_evaluate(t, next_time, &output)) return false;
    t->elapsed_s = min_double(next_time, t->duration_s);
    t->point = output;
    t->active = t->elapsed_s < t->duration_s;
    *point = output;
    return true;
}
