#ifndef CYBERGEAR_TUNING_H
#define CYBERGEAR_TUNING_H

#include "cybergear.h"
#include <stddef.h>

typedef struct {
    CyberGearConfig motor;
    float amplitude_deg;
    float small_amplitude_deg;
    uint32_t dwell_ms;
    uint32_t leg_timeout_ms;
} CyberGearTuning;

bool cybergear_tuning_defaults(CyberGearTuning *tuning);
bool cybergear_tuning_valid(const CyberGearTuning *tuning);
bool cybergear_tuning_set(CyberGearTuning *candidate, const char *key, const char *value);
size_t cybergear_tuning_count(void);
const char *cybergear_tuning_key(size_t index);
const char *cybergear_tuning_help(size_t index);
double cybergear_tuning_value(const CyberGearTuning *tuning, size_t index);

#endif
