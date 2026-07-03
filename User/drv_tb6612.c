#include "drv_tb6612.h"
#include "bsp_pwm.h"
#include "tim.h"

void TB6612_Init(void);
void TB6612_SetDirection(TB6612_Dir_t dir);
void TB6612_SetDuty(uint32_t duty);
void TB6612_Stop(void);
void TB6612_Brake(void);
void TB6612_SetRPM(float rpm);
void TB6612_SetControlOutput(float output);
float TB6612_GetTargetRPM(void);


static float tb6612_target_rpm = 0.0f;

static float TB6612_AbsFloat(float value)
{
    if(value < 0.0f)
    {
        return -value;
    }

    return value;
}

void TB6612_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    TB6612_Stop();
}

void TB6612_SetDirection(TB6612_Dir_t dir)
{
    switch(dir)
    {
        case TB6612_DIR_FORWARD:
            HAL_GPIO_WritePin(MOTOR_OUT1_GPIO_Port, MOTOR_OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_OUT2_GPIO_Port, MOTOR_OUT2_Pin, GPIO_PIN_RESET);
            break;

        case TB6612_DIR_REVERSE:
            HAL_GPIO_WritePin(MOTOR_OUT1_GPIO_Port, MOTOR_OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_OUT2_GPIO_Port, MOTOR_OUT2_Pin, GPIO_PIN_SET);
            break;

        case TB6612_DIR_BRAKE:
            HAL_GPIO_WritePin(MOTOR_OUT1_GPIO_Port, MOTOR_OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_OUT2_GPIO_Port, MOTOR_OUT2_Pin, GPIO_PIN_SET);
            break;

        case TB6612_DIR_STOP:
        default:
            HAL_GPIO_WritePin(MOTOR_OUT1_GPIO_Port, MOTOR_OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_OUT2_GPIO_Port, MOTOR_OUT2_Pin, GPIO_PIN_RESET);
            break;
    }
}

void TB6612_SetDuty(uint32_t duty)
{
    if(duty > TB6612_PWM_MAX_DUTY)
    {
        duty = TB6612_PWM_MAX_DUTY;
    }

    setPwmDuty(duty);
}

void TB6612_Stop(void)
{
    tb6612_target_rpm = 0.0f;
    TB6612_SetDuty(0);
    TB6612_SetDirection(TB6612_DIR_STOP);
}

void TB6612_Brake(void)
{
    tb6612_target_rpm = 0.0f;
    TB6612_SetDuty(0);
    TB6612_SetDirection(TB6612_DIR_BRAKE);
}

void TB6612_SetRPM(float rpm)
{
    if(rpm > TB6612_MAX_RPM)
    {
        rpm = TB6612_MAX_RPM;
    }

    if(rpm < -TB6612_MAX_RPM)
    {
        rpm = -TB6612_MAX_RPM;
    }

    tb6612_target_rpm = rpm;

    if(rpm == 0.0f)
    {
        TB6612_Stop();
    }
}

void TB6612_SetControlOutput(float output)
{
    float motor_output = output * TB6612_RPM_DIR;
    float abs_output = TB6612_AbsFloat(motor_output);
    uint32_t duty = 0;

    if(abs_output <= 0.0f)
    {
        TB6612_SetDuty(0);
        TB6612_SetDirection(TB6612_DIR_STOP);
        return;
    }

    if(abs_output > (float)TB6612_PWM_MAX_DUTY)
    {
        abs_output = (float)TB6612_PWM_MAX_DUTY;
    }

    duty = (uint32_t)abs_output;

    if(motor_output > 0.0f)
    {
        TB6612_SetDirection(TB6612_DIR_FORWARD);
    }
    else
    {
        TB6612_SetDirection(TB6612_DIR_REVERSE);
    }

    TB6612_SetDuty(duty);
}

float TB6612_GetTargetRPM(void)
{
    return tb6612_target_rpm;
}
