#ifndef CYBERGEAR_HOMING_H
#define CYBERGEAR_HOMING_H

#include "cybergear.h"

typedef struct
{
    CyberGearMotor *motor;
    void (*check_feedback)(void);
    void (*print_can_status)(void);
} CyberGearHomingContext;

/* Main-context only. All context members must be valid; success leaves STOP set. */
bool cybergear_homing_run(const CyberGearHomingContext *context);

#endif
