#pragma once

#include "main.h"

#define TB6612_PWM_MAX_DUTY 4999u
#define TB6612_MAX_RPM      282.0f
#define TB6612_RPM_DIR      1.0f

typedef enum
{
    TB6612_DIR_STOP = 0,
    TB6612_DIR_FORWARD,
    TB6612_DIR_REVERSE,
    TB6612_DIR_BRAKE
} TB6612_Dir_t;

void TB6612_SetRPM(float rpm);
void TB6612_SetControlOutput(float output);
float TB6612_GetTargetRPM(void);
void TB6612_Init(void);
