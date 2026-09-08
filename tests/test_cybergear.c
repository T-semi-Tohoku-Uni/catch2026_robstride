#include "cybergear.h"
#include "cybergear_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

TestDWT test_dwt;
uint32_t SystemCoreClock=80000000;
static uint32_t tick, tx_id, tx_count, normal_count, busoff;
static int tx_fail;
static uint8_t tx_bytes[8];
static FDCAN_HandleTypeDef hfdcan1;
static unsigned upstream_count;
uint32_t HAL_GetTick(void) {return tick;}
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,FDCAN_TxHeaderTypeDef *head,uint8_t *data)
{
    if(h==&hfdcan1) {
        float angles[4]; memcpy(angles,data,sizeof(angles));
        assert(head->Identifier==0x210 && head->DataLength==0x100000);
        assert(fabsf(angles[1]+1)<1e-6f && fabsf(angles[2]-1.884f)<1e-6f);
        assert(fabsf(angles[3]+2.963f)<1e-6f);
        ++upstream_count; return 0;
    }
    tx_id=head->Identifier; memcpy(tx_bytes,data,8); ++tx_count;
    if ((tx_id>>24)!=4) ++normal_count;
    return tx_fail;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h) {(void)h; return tx_fail?0:3;}
int HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *s)
{ (void)h; s->BusOff=busoff; return 0; }
int HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *s)
{ (void)h; memset(s,0,sizeof(*s)); return 0; }
static CGConfig config(void)
{
    CGConfig c={0};
    c.motion=(CGLimits){-3,3,1.5f,3,15};
    c.guard_min=-3; c.guard_max=3;
    c.current_limit=2; c.current_slew=5; c.disturbance_limit=0.5f; c.disturbance_slew=1;
    c.b0=10; c.temperature_limit=60; c.speed_trip=5; c.position_jump=0.5f;
    c.stall_error=0.1f; c.stall_progress=0.01f; c.stall_ms=1000;
    c.approved=true; c.b0_verified=true;
    return c;
}
static void near(float a,float b,float tolerance) { assert(fabsf(a-b)<tolerance); }
static bool no_budget(void *context) {(void)context; return false;}
static void trajectory_tests(void)
{
    CGConfig c=config(); CGTrajectory t; cg_trajectory_hold(&t,0);
    const float goals[]={1.570796327f,-1.570796327f,0,0.0001f,2.99f};
    for (unsigned k=0;k<sizeof(goals)/sizeof(goals[0]);++k) {
        assert(cg_trajectory_plan(&t,(CGReference){0,0,0},goals[k],&c.motion));
        if (k==0) near(t.duration,1.9634954f,0.0001f);
        for (int j=0;j<=1000;++j) {
            float time=t.duration*j/1000;
            CGReference r=cg_trajectory_sample(&t,time);
            assert(r.q>=-3.00001f && r.q<=3.00001f);
            assert(fabsf(r.v)<=1.50001f && fabsf(r.a)<=3.00001f);
            float s=time/t.duration;
            float jerk=(6*t.c[3]+24*t.c[4]*s+60*t.c[5]*s*s)/(t.duration*t.duration*t.duration);
            assert(fabsf(jerk)<=15.001f);
        }
        CGReference end=cg_trajectory_sample(&t,t.duration);
        near(end.q,goals[k],1e-6f); near(end.v,0,1e-6f); near(end.a,0,1e-6f);
    }
    assert(cg_trajectory_plan(&t,(CGReference){0,0,0},1,&c.motion));
    CGReference moving=cg_trajectory_sample(&t,t.duration*0.3f);
    assert(cg_trajectory_plan(&t,moving,-0.5f,&c.motion));
    CGReference start=cg_trajectory_sample(&t,0);
    near(start.q,moving.q,1e-6f); near(start.v,moving.v,1e-6f); near(start.a,moving.a,1e-6f);
    CGTrajectory saved=t;
    assert(!cg_trajectory_plan(&t,(CGReference){2.999f,1.5f,3},-3,&c.motion));
    assert(memcmp(&saved,&t,sizeof(t))==0);
    assert(!cg_trajectory_plan(&t,moving,NAN,&c.motion));
    assert(!cg_trajectory_plan(&t,moving,INFINITY,&c.motion));
    assert(!cg_trajectory_plan_bounded(&t,moving,1,&c.motion,no_budget,0));
    assert(memcmp(&saved,&t,sizeof(t))==0);
}
static void controller_tests(void)
{
    CGConfig c=config(); CGController s={0}; float l1,l2,l3;
    cg_observer_gains(0.01f,&l1,&l2,&l3);
    near(l1,0.2591818f,1e-6f); near(l2,2.58753f,0.0001f); near(l3,8.61784f,0.001f);
    assert(cg_controller_step(&s,&c,(CGReference){0,1,2},0,false,0,0.01f));
    near(s.track,1.8f,1e-6f); near(s.command,0.05f,1e-6f);
    assert(s.slew_limited);
    memset(&s,0,sizeof(s)); c.b0=-10;
    assert(cg_controller_step(&s,&c,(CGReference){0,1,2},0,false,0,0.01f));
    assert(s.command<0);
    c=config(); memset(&s,0,sizeof(s));
    assert(cg_controller_step(&s,&c,(CGReference){3,0,0},0,false,0,0.01f));
    assert(s.amplitude_limited && s.slew_limited);
    float previous=0;
    for (int k=0;k<300;++k) {
        assert(cg_controller_step(&s,&c,(CGReference){k<100?3.0f:-3.0f,0,0},0,true,previous,0.01f));
        assert(fabsf(s.command)<=2.00001f && fabsf(s.command-previous)<=0.05001f);
        assert(fabsf(s.dist)<=0.50001f); previous=s.command;
    }
    memset(&s,0,sizeof(s));
    assert(cg_controller_step(&s,&c,(CGReference){0,0,0},1,true,0,0.01f));
    float disturbance=s.f;
    assert(cg_controller_step(&s,&c,(CGReference){0,0,0},1,false,0,0.01f));
    near(s.f,disturbance,1e-6f); /* no duplicate measurement correction */
    assert(!cg_controller_step(&s,&c,(CGReference){0,0,0},NAN,true,0,0.01f));
    assert(!cg_controller_step(&s,&c,(CGReference){0,0,0},0,true,0,0.051f));
    c.current_limit=INFINITY; assert(!cg_config_valid(&c));
    assert(!cg_config_valid(&cybergear_config));
}
static FDCAN_HandleTypeDef handle;
static void setup(CyberGearMotor *m)
{
    tick=0; tx_fail=0; tx_count=normal_count=busoff=0;
    assert(cybergear_init(m,&handle,0x7f,0xfe)); m->config=config();
}
static void feedback(CyberGearMotor *m,uint8_t mode)
{
    m->feedback=(CyberGearFeedback){.position_rad=0,.velocity_rad_s=0,.temperature_c=25,
        .mode=mode,.online=true,.rx_sequence=m->feedback.rx_sequence+1,.last_received_ms=tick};
}
static bool step(CyberGearMotor *m,float target)
{ tick+=1000/CYBERGEAR_CONTROL_HZ; return cybergear_control_position_adrc(m,target); }
static void arm(CyberGearMotor *m)
{
    assert(cybergear_start_position_adrc(m));
    assert(step(m,0)); feedback(m,0); assert(step(m,0));
    assert(m->diagnostics.state==CG_MODE_WRITE);
    assert(step(m,0)); assert(m->diagnostics.state==CG_MODE_READBACK);
    assert(step(m,0));
    uint8_t data[8]={5,0x70,0,0,3,0,0,0};
    FDCAN_RxHeaderTypeDef h={.Identifier=0x11007ffe,.IdType=FDCAN_EXTENDED_ID,
        .RxFrameType=FDCAN_DATA_FRAME,.DataLength=FDCAN_DLC_BYTES_8};
    assert(cybergear_dispatch(m,&h,data)); assert(step(m,0));
    assert(step(m,0)); assert(step(m,0)); feedback(m,2); assert(step(m,0));
    for (int i=0;i<5;++i) {feedback(m,2); assert(step(m,0));}
    assert(m->diagnostics.state==CG_RUN);
}
static void protocol_state_tests(void)
{
    CyberGearMotor m; setup(&m);
    const uint8_t mode_bytes[8]={5,0x70,0,0,3,0,0,0};
    assert(cybergear_set_run_mode(&m,CYBERGEAR_RUN_MODE_CURRENT));
    assert(tx_id==0x1200fe7f && memcmp(tx_bytes,mode_bytes,8)==0);
    assert(cybergear_set_current(&m,1));
    const uint8_t current_bytes[8]={6,0x70,0,0,0,0,0x80,0x3f};
    assert(tx_id==0x1200fe7f && memcmp(tx_bytes,current_bytes,8)==0);
    assert(cybergear_stop(&m)); assert(tx_id==0x0400fe7f);
    assert(!cybergear_set_current(&m,NAN));
    assert(!cybergear_control(&m,NAN,0,0,0,0));
    FDCAN_RxHeaderTypeDef h={.Identifier=0x02807ffe,.IdType=FDCAN_EXTENDED_ID,
        .RxFrameType=FDCAN_DATA_FRAME,.DataLength=FDCAN_DLC_BYTES_8};
    uint8_t data[8]={0x80,0,0x80,0,0x80,0,0,250};
    assert(cybergear_dispatch(&m,&h,data)); assert(cybergear_dispatch(&m,&h,data));
    assert(m.feedback.rx_sequence==2 && m.feedback.last_received_ms==0);
    assert(m.feedback.mode==2 && m.mode_sequence==0);
    h.DataLength=0; assert(!cybergear_dispatch(&m,&h,data)); h.DataLength=FDCAN_DLC_BYTES_8;
    h.Identifier=0x02807ffe^1; assert(!cybergear_dispatch(&m,&h,data));
    h.Identifier=0x02807efe; assert(!cybergear_dispatch(&m,&h,data));
    h.Identifier=0x03807ffe; assert(!cybergear_dispatch(&m,&h,data));
    setup(&m); arm(&m);
    assert(!cybergear_set_velocity(&m,1)); assert(!cybergear_enable(&m));
    feedback(&m,2); assert(step(&m,1)); float duration=m.trajectory.duration;
    feedback(&m,2); assert(step(&m,1)); assert(m.trajectory.duration==duration && m.trajectory.elapsed>0);
    float queued=m.diagnostics.last_queued_current;
    tx_fail=1; feedback(&m,2); assert(!step(&m,1));
    assert(m.diagnostics.fault==CG_FAULT_TX && m.diagnostics.last_queued_current==queued);
    uint32_t normal=normal_count;
    for(int i=0;i<120;++i) step(&m,1);
    assert(normal_count==normal && m.diagnostics.stop_attempts==5);
    assert(!cybergear_start_position_adrc(&m)); assert(!cybergear_enable(&m));
    assert(!m.diagnostics.reset_confirmed && !m.diagnostics.applied_current_valid);
    setup(&m); arm(&m); feedback(&m,2); assert(!step(&m,NAN)); assert(m.diagnostics.fault==CG_FAULT_INPUT);
    setup(&m); arm(&m); tick+=100; assert(!step(&m,0)); assert(m.diagnostics.fault==CG_FAULT_DT);
    setup(&m); arm(&m); busoff=1; assert(!step(&m,0)); assert(m.diagnostics.fault==CG_FAULT_BUS);
    setup(&m); arm(&m);
    tick=UINT32_MAX-4; m.diagnostics.timestamp_ms=tick; feedback(&m,2);
    assert(step(&m,0)); /* modular tick subtraction */
    setup(&m); arm(&m);
    h.Identifier=0x15007ffe; data[0]=1;
    assert(cybergear_dispatch(&m,&h,data)); feedback(&m,2);
    assert(!step(&m,0)); assert(m.diagnostics.fault==CG_FAULT_DEVICE);
    CGLog log; int logs=0; while(cybergear_log_pop(&m,&log)) ++logs;
    assert(logs>0 && logs<=16);
    setup(&m); m.config.approved=false;
    assert(!cybergear_start_position_adrc(&m)); assert(m.diagnostics.fault==CG_FAULT_CONFIG);
    setup(&m); arm(&m); feedback(&m,2); m.feedback.temperature_c=61;
    assert(!step(&m,0) && m.diagnostics.fault==CG_FAULT_TEMPERATURE);
    setup(&m); arm(&m); feedback(&m,2); m.feedback.velocity_rad_s=6;
    assert(!step(&m,0) && m.diagnostics.fault==CG_FAULT_SPEED);
    setup(&m); arm(&m); feedback(&m,2); m.feedback.position_rad=0.6f;
    assert(!step(&m,0) && m.diagnostics.fault==CG_FAULT_JUMP);
    setup(&m); arm(&m); feedback(&m,2); m.feedback.position_rad=3.1f;
    assert(!step(&m,0) && m.diagnostics.fault==CG_FAULT_RANGE);
    setup(&m); arm(&m); feedback(&m,2);
    for (int i=0;i<4;++i) step(&m,0);
    assert(m.diagnostics.fault==CG_FAULT_TIMEOUT);
    setup(&m); assert(cybergear_start_position_adrc(&m));
    for (int i=0;i<120;++i) step(&m,0);
    assert(m.diagnostics.fault==CG_FAULT_TIMEOUT && normal_count==0);
    setup(&m); m.diagnostics.state=CG_MODE_READBACK; m.state_command_queued=true;
    h.Identifier=0x11007ffe; memcpy(data,mode_bytes,8); data[4]=2;
    assert(cybergear_dispatch(&m,&h,data));
    assert(!step(&m,0) && m.diagnostics.fault==CG_FAULT_MODE);
    setup(&m); m.diagnostics.state=CG_ENABLE_WAIT; tx_fail=1;
    assert(!step(&m,0) && m.diagnostics.fault==CG_FAULT_TX);
    setup(&m); arm(&m);
    for (int i=0;i<400 && !m.diagnostics.fault;++i) {feedback(&m,2); step(&m,1);}
    assert(m.diagnostics.fault==CG_FAULT_STALL);
    setup(&m); arm(&m);
    h.Identifier=0x02817ffe; memset(data,0,8);
    assert(cybergear_dispatch(&m,&h,data));
    h.Identifier=0x02807ffe; assert(cybergear_dispatch(&m,&h,data));
    assert(m.diagnostics.fault==CG_FAULT_DEVICE); /* transient flags stay latched */
}
typedef struct {int unused;} TIM_HandleTypeDef;
static TIM_HandleTypeDef htim6;
static CyberGearMotor cybergear_base;
static float target_angle[4];
static bool el05_initializing;
typedef struct { struct {float position_rad;} feedback;} TestRobstride;
static TestRobstride robstride_handler[3];
enum {RIGHT_RS03_INDEX,LEFT_RS03_INDEX,EL05_INDEX};
static FDCAN_TxHeaderTypeDef inter_board_txheader;
static unsigned rs_count[3];
static void robstride_set_position(TestRobstride *motor,float target)
{
    int index=(int)(motor-robstride_handler);
    near(target,index==0?-1.884f:index==1?-1.0f:-2.963f,1e-6f);
    ++rs_count[index];
}
static void float_to_u8(float *values,uint8_t *bytes,int count)
{memcpy(bytes,values,(size_t)count*sizeof(float));}
#define FDCAN_DLC_BYTES_16 0x100000
#ifdef _MSC_VER
#pragma warning(push)
/* Existing RobStride feedback offsets use double literals; preserve firmware. */
#pragma warning(disable:4244)
#endif
#include "scheduler.inc"
#ifdef _MSC_VER
#pragma warning(pop)
#endif
static void schedule_tests(void)
{
    setup(&cybergear_base); arm(&cybergear_base);
    unsigned initial=cybergear_base.diagnostics.control_cycles;
    for (unsigned t=0;t<1000;++t) {
        /* First phase starts one full control period after the startup helper. */
        tick+=(t==0 ? 1000/CYBERGEAR_CONTROL_HZ : 1);
        feedback(&cybergear_base,2);
        HAL_TIM_PeriodElapsedCallback(&htim6);
    }
    assert(cybergear_base.diagnostics.state==CG_RUN);
    assert(cybergear_base.diagnostics.control_cycles-initial==CYBERGEAR_CONTROL_HZ);
    assert(rs_count[0]==100 && rs_count[1]==100 && rs_count[2]==100 && upstream_count==100);
    assert(cybergear_base.diagnostics.log_dropped>0);
    target_angle[0]=NAN;
    for (unsigned t=0;t<1000;++t) {++tick; HAL_TIM_PeriodElapsedCallback(&htim6);}
    assert(cybergear_base.diagnostics.state==CG_FAULT_LATCHED);
    assert(rs_count[0]==200 && rs_count[1]==200 && rs_count[2]==200 && upstream_count==200);
}
static void stress_tests(void)
{
    const float ratios[]={0.25f,0.5f,1,2,4};
    const int delays[]={0,5,10,20};
    puts("rate,b_ratio,delay_ms,flexible,friction,passed,max_q,final_error,late_rms,previous_rms,saturation_fraction");
    for (int bi=0;bi<5;++bi) for (int di=0;di<4;++di)
    for (int flex=0;flex<2;++flex) for (int friction=0;friction<2;++friction) {
        CGConfig c=config(); CGController s={0}; CGTrajectory trajectory;
        assert(cg_trajectory_plan(&trajectory,(CGReference){0,0,0},1.57f,&c.motion));
        float q=0,v=0,ql=0,vl=0,command=0,measurements[64]={0},commands[64]={0};
        float max_q=0,late=0,previous=0; unsigned saturated=0,controls=0;
        uint32_t seed=20260909; bool valid=true;
        for (int ms=0;ms<12000;++ms) {
            int at=ms%64, delayed=(ms+64-delays[di])%64;
            measurements[at]=q; commands[at]=command;
            if (ms%(1000/CYBERGEAR_CONTROL_HZ)==0) {
                seed=1664525U*seed+1013904223U;
                float noise=((float)(seed>>16)/65535-0.5f)*0.0003815f;
                CGReference r=cg_trajectory_sample(&trajectory,ms*0.001f);
                if (!cg_controller_step(&s,&c,r,measurements[delayed]+noise,true,command,
                    1.0f/CYBERGEAR_CONTROL_HZ)) {valid=false; break;}
                command=s.command; ++controls;
                if(s.amplitude_limited || s.slew_limited) ++saturated;
                commands[at]=command;
            }
            float spring=flex ? 25*(q-ql)+0.2f*(v-vl) : 0;
            float drag=friction ? 0.4f*tanhf(v/0.01f) : 0;
            float acc=10*ratios[bi]*commands[delayed]-spring-drag-(ms>=4000?0.4f:0);
            v+=acc*0.001f; q+=v*0.001f;
            if (flex) {vl+=spring*0.001f; ql+=vl*0.001f;} else ql=q;
            max_q=fmaxf(max_q,fabsf(q));
            if (!isfinite(q) || fabsf(q)>3 || fabsf(ql)>3) {valid=false; break;}
            float e=1.57f-ql;
            if(ms>=10000) late+=e*e/2000;
            else if(ms>=8000) previous+=e*e/2000;
        }
        float error=fabsf(1.57f-ql);
        bool passed=valid && error<0.05f && sqrtf(late)<=fmaxf(0.01f,1.1f*sqrtf(previous));
        printf("%d,%.2f,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            CYBERGEAR_CONTROL_HZ,(double)ratios[bi],delays[di],flex,friction,(int)passed,
            (double)max_q,(double)error,(double)sqrtf(late),(double)sqrtf(previous),
            controls?(double)saturated/controls:0);
    }
}
int main(int argc,char **argv)
{
    if (argc>1 && strcmp(argv[1],"--stress")==0) {stress_tests(); return 0;}
    trajectory_tests(); controller_tests(); protocol_state_tests(); schedule_tests();
    printf("CyberGear %d Hz host tests passed\n",CYBERGEAR_CONTROL_HZ);
    return 0;
}
