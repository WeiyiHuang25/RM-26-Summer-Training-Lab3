#include "app_task.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "drv_tb6612.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "dsp/controller_functions.h"
#include <string.h>

#define SPEED_CONTROL_HZ               100.0f
#define SPEED_SAMPLE_QUEUE_LENGTH      1u
#define SPEED_SAMPLE_TASK_STACK_WORDS  256u
#define SPEED_PID_TASK_STACK_WORDS     256u
#define SPEED_SAMPLE_TASK_PRIORITY     (tskIDLE_PRIORITY + 3u)
#define SPEED_PID_TASK_PRIORITY        (tskIDLE_PRIORITY + 2u)

#define SPEED_PID_KP                   25.0f
#define SPEED_PID_KI                   10.0f
#define SPEED_PID_KD                   0.0f
#define SPEED_PID_OUTPUT_LIMIT         ((float)TB6612_PWM_MAX_DUTY)

typedef struct
{
    float target_rpm;
    float measured_rpm;
} SpeedSample_t;

static SemaphoreHandle_t speed_tick_sem = NULL;
static QueueHandle_t speed_sample_queue = NULL;
static arm_pid_instance_f32 speed_pid;
static float last_target_rpm = 0.0f;

static void SpeedSampleTask(void *argument);
static void SpeedPidTask(void *argument);
static float App_ClampFloat(float value, float min_value, float max_value);

static volatile SpeedSample_t sample;

void app_task(void)
{
}

void App_Timer100HzISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if((speed_tick_sem != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        xSemaphoreGiveFromISR(speed_tick_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void app_startMotor(void)
{
    TB6612_SetRPM(100.0f);
}

void User_init(void)
{
    Encoder_Init();
    TB6612_Init();
    UART_Init();
}

void App_FREERTOS_Init(void)
{
    speed_tick_sem = xSemaphoreCreateBinary();
    speed_sample_queue = xQueueCreate(SPEED_SAMPLE_QUEUE_LENGTH, sizeof(SpeedSample_t));

    speed_pid.Kp = SPEED_PID_KP;
    speed_pid.Ki = SPEED_PID_KI / SPEED_CONTROL_HZ;
    speed_pid.Kd = SPEED_PID_KD * SPEED_CONTROL_HZ;
    arm_pid_init_f32(&speed_pid, 1);

    xTaskCreate(SpeedSampleTask,
                "spd_sample",
                SPEED_SAMPLE_TASK_STACK_WORDS,
                NULL,
                SPEED_SAMPLE_TASK_PRIORITY,
                NULL);
    xTaskCreate(SpeedPidTask,
                "spd_pid",
                SPEED_PID_TASK_STACK_WORDS,
                NULL,
                SPEED_PID_TASK_PRIORITY,
                NULL);
}

static void SpeedSampleTask(void *argument)
{
    (void)argument;

    for(;;)
    {
        if(xSemaphoreTake(speed_tick_sem, portMAX_DELAY) == pdTRUE)
        {
            SpeedSample_t sample;
            uint8_t telemetry[8];

            sample.target_rpm = TB6612_GetTargetRPM();
            sample.measured_rpm = Encoder_GetRPM();

            xQueueOverwrite(speed_sample_queue, &sample);

            memcpy(&telemetry[0], &sample.measured_rpm, sizeof(sample.measured_rpm));
            memcpy(&telemetry[4], &sample.target_rpm, sizeof(sample.target_rpm));
            (void)UART_SendData(telemetry, sizeof(telemetry));
        }
    }
}

static void SpeedPidTask(void *argument)
{
    (void)argument;

    for(;;)
    {
        

        if(xQueueReceive(speed_sample_queue, (void*) &sample, portMAX_DELAY) == pdTRUE)
        {
            float error = sample.target_rpm - sample.measured_rpm;
            float output;

            if(sample.target_rpm == 0.0f)
            {
                arm_pid_reset_f32(&speed_pid);
                last_target_rpm = 0.0f;
                TB6612_SetControlOutput(0.0f);
                continue;
            }

            if((last_target_rpm >= 0.0f && sample.target_rpm < 0.0f) ||
               (last_target_rpm <= 0.0f && sample.target_rpm > 0.0f))
            {
                arm_pid_reset_f32(&speed_pid);
            }

            output = arm_pid_f32(&speed_pid, error);
            output = App_ClampFloat(output, -SPEED_PID_OUTPUT_LIMIT, SPEED_PID_OUTPUT_LIMIT);
            TB6612_SetControlOutput(output);
            last_target_rpm = sample.target_rpm;
        }
    }
}

static float App_ClampFloat(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }

    if(value < min_value)
    {
        return min_value;
    }

    return value;
}
