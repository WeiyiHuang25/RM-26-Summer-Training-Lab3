#pragma once

#include "main.h"

/* Motor-side encoder: 11 pulses per motor revolution. */
#define ENCODER_PULSE_PER_MOTOR_REV 11u
#define ENCODER_QUADRATURE_MULTIPLE 4u
#define ENCODER_GEAR_RATIO          21.3f
#define ENCODER_DIRECTION           -1

void Encoder_Init(void);
int32_t Encoder_GetCount(void);
float Encoder_GetRPM(void);
