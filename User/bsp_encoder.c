#include "bsp_encoder.h"
#include "tim.h"

static int16_t encoder_last_count = 0;
static uint32_t encoder_last_tick = 0;

#define ENCODER_COUNTS_PER_MOTOR_REV  (ENCODER_PULSE_PER_MOTOR_REV * ENCODER_QUADRATURE_MULTIPLE)

void Encoder_Init(void);
void Encoder_Reset(void);
int32_t Encoder_GetCount(void);
void Encoder_Reset(void);
float Encoder_GetMotorRPM(void);
float Encoder_GetRPM(void);


void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    Encoder_Reset();
}

int32_t Encoder_GetCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
}

void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    encoder_last_count = 0;
    encoder_last_tick = HAL_GetTick();
}

float Encoder_GetMotorRPM(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t dt_ms = now_tick - encoder_last_tick;

    if(dt_ms == 0)
    {
        return 0.0f;
    }

    int16_t now_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t delta_count = (int16_t)(now_count - encoder_last_count);

    encoder_last_count = now_count;
    encoder_last_tick = now_tick;

    return ((float)delta_count * (float)ENCODER_DIRECTION * 60000.0f) /
           ((float)ENCODER_COUNTS_PER_MOTOR_REV * (float)dt_ms);
}

float Encoder_GetRPM(void)
{
    return Encoder_GetMotorRPM() / ENCODER_GEAR_RATIO;
}
