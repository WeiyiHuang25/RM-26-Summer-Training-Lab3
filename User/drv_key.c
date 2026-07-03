#include "drv_key.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case KEY_Pin:
        {
            HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        }break;
        default: break;
    }
}