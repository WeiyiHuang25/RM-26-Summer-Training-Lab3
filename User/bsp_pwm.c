#include "bsp_pwm.h"
#include "tim.h"


/**
 *  @brief Set PWM duty
 * 
 *  @param duty must between 0-4999
 */
void setPwmDuty(uint32_t duty)
{
    if(duty >4999)
        return;
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty);
}

