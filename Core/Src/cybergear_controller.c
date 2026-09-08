#include "cybergear_controller.h"
#include <math.h>

static float clip(float v,float limit) { return fminf(limit,fmaxf(-limit,v)); }
bool cg_config_valid(const CGConfig *c)
{
    if (!c || !c->approved || !c->b0_verified || !cg_limits_valid(&c->motion)) return false;
    const float positive[]={c->current_limit,c->current_slew,c->disturbance_slew,
        c->temperature_limit,c->speed_trip,c->position_jump,c->stall_error,c->stall_progress};
    for (unsigned i=0;i<sizeof(positive)/sizeof(positive[0]);++i)
        if (!isfinite(positive[i]) || positive[i]<=0) return false;
    return isfinite(c->b0) && fabsf(c->b0)>1e-6f && isfinite(c->disturbance_limit) &&
        c->disturbance_limit>=0 && c->disturbance_limit<=c->current_limit && c->stall_ms>0 &&
        c->motion.q_min>=-12.5f && c->motion.q_max<=12.5f && c->speed_trip<=30 &&
        isfinite(c->guard_min) && isfinite(c->guard_max) &&
        c->guard_min<=c->motion.q_min && c->guard_max>=c->motion.q_max &&
        c->guard_min>=-12.5f && c->guard_max<=12.5f;
}
void cg_observer_gains(float dt,float *l1,float *l2,float *l3)
{
    float p=expf(-10*dt), d=1-p;
    *l1=1-p*p*p; *l2=1.5f*d*d*(1+p)/dt; *l3=d*d*d/(dt*dt);
}
bool cg_controller_step(CGController *s,const CGConfig *c,CGReference r,
    float measurement,bool fresh,float queued_current,float dt)
{
    if (!s || !cg_config_valid(c) || !isfinite(dt) || dt<=0 || dt>0.05f ||
        !isfinite(measurement) || !isfinite(queued_current) || !isfinite(r.q) ||
        !isfinite(r.v) || !isfinite(r.a)) return false;
    float acc=s->f+c->b0*queued_current;
    s->q+=dt*s->v+0.5f*dt*dt*acc; s->v+=dt*acc;
    if (fresh) {
        float l1,l2,l3; cg_observer_gains(dt,&l1,&l2,&l3);
        float error=measurement-s->q;
        s->q+=l1*error; s->v+=l2*error; s->f+=l3*error;
    }
    if (c->legacy_leak) {
        float blend=fminf(1,fmaxf(0,(fabsf(r.q-measurement)-0.003f)/0.012f));
        s->f*=expf(-(0.5f+blend*(0.03f-0.5f))*dt);
    }
    s->gamma=fminf(1,s->gamma+dt); /* one-second compensation ramp */
    float desired=clip(-s->gamma*s->f/c->b0,c->disturbance_limit);
    s->dist+=clip(desired-s->dist,c->disturbance_slew*dt);
    s->track=(r.a+16*(r.q-s->q)+16*(r.v-s->v))/c->b0;
    s->requested=s->track+s->dist;
    float limited=clip(s->requested,c->current_limit);
    s->command=queued_current+clip(limited-queued_current,c->current_slew*dt);
    s->amplitude_limited=limited!=s->requested; s->slew_limited=s->command!=limited;
    return isfinite(s->q) && isfinite(s->v) && isfinite(s->f) && isfinite(s->command) &&
        isfinite(s->requested) && isfinite(s->dist) && isfinite(s->gamma);
}
