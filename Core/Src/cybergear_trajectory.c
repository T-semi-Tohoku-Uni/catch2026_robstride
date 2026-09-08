#include "cybergear_trajectory.h"
#include <math.h>
#include <string.h>

bool cg_limits_valid(const CGLimits *l)
{
    return l && isfinite(l->q_min) && isfinite(l->q_max) && l->q_min < l->q_max &&
        isfinite(l->velocity) && l->velocity > 0 &&
        isfinite(l->acceleration) && l->acceleration > 0 &&
        isfinite(l->jerk) && l->jerk > 0;
}

static double evaluate(const double *c, int n, double x)
{
    double y = c[n];
    while (n > 0) y = y*x + c[--n];
    return y;
}

/* Isolate roots on monotone intervals separated by derivative roots.
 * Degree <= 4, recursion depth <= 4, exactly 32 bisections per sign change.
 * Tangencies are included as interval boundaries in extrema checks. */
static int roots(const double *c, int n, double *out, CGBudgetAvailable available, void *context)
{
    if (available && !available(context)) return -1;
    while (n > 0 && fabs(c[n]) < 1e-14) --n;
    if (n == 0) return 0;
    if (n == 1) {
        double x = -c[0]/c[1];
        if (x > 0 && x < 1) { out[0] = x; return 1; }
        return 0;
    }
    double d[5], bounds[6] = {0}, critical[4];
    for (int i=1; i<=n; ++i) d[i-1] = i*c[i];
    int k = roots(d, n-1, critical, available, context), count = 0;
    if (k<0) return -1;
    for (int i=0; i<k; ++i) bounds[i+1] = critical[i];
    bounds[k+1] = 1;
    for (int i=0; i<=k; ++i) {
        double lo=bounds[i], hi=bounds[i+1], f=evaluate(c,n,lo);
        if (i > 0 && fabs(f) < 1e-12 && count < n) out[count++]=lo;
        if (f*evaluate(c,n,hi) >= 0) continue;
        for (int j=0; j<32; ++j) {
            if (available && !available(context)) return -1;
            double mid=(lo+hi)*0.5, fm=evaluate(c,n,mid);
            if ((f < 0) == (fm < 0)) {lo=mid; f=fm;} else hi=mid;
        }
        if (count < n) out[count++]=(lo+hi)*0.5;
    }
    return count;
}

static bool feasible(const CGTrajectory *t, const CGLimits *l, CGBudgetAvailable available, void *context)
{
    double c[6], d[5], r[4];
    for (int i=0;i<6;++i) { c[i]=t->c[i]; if (!isfinite(c[i])) return false; }
    for (int order=0;order<4;++order) {
        int n=5-order;
        for (int i=1;i<=n;++i) d[i-1]=i*c[i];
        int nr=roots(d,n-1,r,available,context);
        if (nr<0) return false;
        double low=order == 0 ? l->q_min : -(order == 1 ? l->velocity : order == 2 ? l->acceleration : l->jerk);
        double high=order == 0 ? l->q_max : -low;
        for (int i=-2;i<nr;++i) {
            double y=evaluate(c,n,i == -2 ? 0 : i == -1 ? 1 : r[i]);
            if (!isfinite(y) || y < low-1e-6 || y > high+1e-6) return false;
        }
        for (int i=0;i<n;++i) c[i]=d[i]/t->duration;
    }
    return true;
}

void cg_trajectory_hold(CGTrajectory *t, float q)
{
    memset(t,0,sizeof(*t)); t->c[0]=q; t->target=q;
}

CGReference cg_trajectory_sample(const CGTrajectory *t, float seconds)
{
    CGReference r={t->target,0,0};
    if (t->duration <= 0 || seconds >= t->duration) return r;
    float s=fmaxf(seconds,0)/t->duration;
    r.q=t->c[5]; r.v=5*t->c[5]; r.a=20*t->c[5];
    for (int i=4;i>=0;--i) r.q=r.q*s+t->c[i];
    for (int i=4;i>=1;--i) r.v=r.v*s+i*t->c[i];
    for (int i=4;i>=2;--i) r.a=r.a*s+i*(i-1)*t->c[i];
    r.v/=t->duration; r.a/=t->duration*t->duration;
    return r;
}

bool cg_trajectory_plan_bounded(CGTrajectory *t, CGReference s, float target,
    const CGLimits *l, CGBudgetAvailable available, void *context)
{
    if (!t || !cg_limits_valid(l) || !isfinite(target) || !isfinite(s.q) ||
        !isfinite(s.v) || !isfinite(s.a) || target<l->q_min || target>l->q_max ||
        s.q<l->q_min || s.q>l->q_max || fabsf(s.v)>l->velocity || fabsf(s.a)>l->acceleration) return false;
    float distance=fabsf(target-s.q);
    float duration=fmaxf(0.01f,fmaxf(1.875f*distance/l->velocity,
        fmaxf(sqrtf(5.773502692f*distance/l->acceleration),cbrtf(60*distance/l->jerk))));
    for (int attempt=0;attempt<12 && duration<=120; ++attempt, duration*=1.3f) {
        if (available && !available(context)) return false;
        CGTrajectory candidate={.duration=duration,.target=target};
        float v=s.v*duration, a=s.a*duration*duration, delta=target-s.q;
        candidate.c[0]=s.q; candidate.c[1]=v; candidate.c[2]=a/2;
        candidate.c[3]=10*delta-6*v-1.5f*a;
        candidate.c[4]=-15*delta+8*v+1.5f*a;
        candidate.c[5]=6*delta-3*v-0.5f*a;
        if (feasible(&candidate,l,available,context)) { *t=candidate; return true; }
    }
    return false;
}

bool cg_trajectory_plan(CGTrajectory *t, CGReference s, float target, const CGLimits *l)
{
    return cg_trajectory_plan_bounded(t,s,target,l,0,0);
}
