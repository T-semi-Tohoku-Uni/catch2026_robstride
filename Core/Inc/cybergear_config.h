#ifndef CYBERGEAR_CONFIG_H
#define CYBERGEAR_CONFIG_H
#include "cybergear_controller.h"
/* Populate only with reviewed hardware limits. Zero configuration cannot arm.
 * Units: rad, rad/s, rad/s^2, rad/s^3, A, A/s, Celsius, milliseconds.
 * Keep legacy_leak=true for the first trajectory/FF comparison. */
static const CGConfig cybergear_config = { .b0=10.0f, .legacy_leak=true };
#ifndef CYBERGEAR_CONTROL_HZ
#define CYBERGEAR_CONTROL_HZ 100
#endif
#if CYBERGEAR_CONTROL_HZ != 100 && CYBERGEAR_CONTROL_HZ != 200
#error CyberGear control rate must be 100 or 200 Hz
#endif
static inline bool cybergear_control_phase(unsigned phase)
{
    return phase==0 || (CYBERGEAR_CONTROL_HZ==200 && phase==5);
}
#endif
