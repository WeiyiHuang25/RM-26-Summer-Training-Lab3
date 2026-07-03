# 从基础工程到 100Hz 速度闭环

本文面向刚完成 `bsp`、`drv` 和基本 FreeRTOS 框架的同学，目标是把一个 TB6612 PWM 开环电机项目，逐步改成：

- FreeRTOS 任务式结构
- 100Hz 速度闭环
- CMSIS-DSP PID 控制
- UART DMA + ReceiveToIdle 非阻塞通信
- 上位机显示实际转速和目标转速

你可以把本文当成实验步骤，而不只是代码说明。

## 0. 开始前你应该已有的内容

假设你当前工程已经有这些文件和功能：

```text
User/
  bsp_encoder.c / bsp_encoder.h
  bsp_pwm.c     / bsp_pwm.h
  bsp_uart.c    / bsp_uart.h
  drv_tb6612.c  / drv_tb6612.h
  app_task.c    / app_task.h
```

其中基础功能应满足：

- `Encoder_Init()` 可以启动编码器定时器。
- `Encoder_GetRPM()` 可以返回当前输出轴 RPM。
- `setPwmDuty(duty)` 可以设置 PWM 占空比。
- `TB6612_Init()` 可以启动 PWM 并初始化 TB6612。
- `TB6612_SetDirection()` 可以设置正转、反转、刹车或停止。
- FreeRTOS 已经能正常创建默认任务。
- `TIM6` 已配置成 10ms 周期中断，也就是 100Hz。

如果你还没有这些基础驱动，应先完成 `bsp` 和 `drv`，再继续本文。

## 1. 为什么不能继续用主循环 flag

最开始的做法通常是：

```c
volatile uint32_t flag100hz = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim6)
    {
        flag100hz = 1;
    }
}

void app_task(void)
{
    if(flag100hz)
    {
        float rpm = Encoder_GetRPM();
        UART_SendData((uint8_t *)&rpm, sizeof(rpm));
        flag100hz = 0;
    }
}
```

这种方式能跑，但有几个问题：

- 主循环轮询，不适合复杂任务。
- UART 如果阻塞发送，会拖慢控制逻辑。
- 采样、控制、通信混在一起，代码越来越难维护。
- 没有真正的闭环，只是在读速度和发速度。

所以我们改成 FreeRTOS 任务模型。

## 2. 设计新的任务模型

我们采用“生产者消费者模型”。

生产者任务负责采样：

```text
SpeedSampleTask
  等待 100Hz 信号量
  读取目标 RPM
  读取实际 RPM
  把数据放入队列
  用 UART DMA 发给上位机
```

消费者任务负责控制：

```text
SpeedPidTask
  等待队列里的速度样本
  计算 error = target - measured
  调用 CMSIS-DSP PID
  把 PID 输出交给 TB6612
```

中断只负责释放信号量：

```text
TIM6_IRQHandler
  -> HAL_TIM_PeriodElapsedCallback
  -> App_Timer100HzISR
  -> xSemaphoreGiveFromISR
```

最终结构如下：

```text
TIM6 100Hz 中断
    |
    v
speed_tick_sem
    |
    v
SpeedSampleTask 生产速度样本
    |
    v
speed_sample_queue
    |
    v
SpeedPidTask 消费速度样本并计算 PID
    |
    v
TB6612_SetControlOutput()
```

## 3. 第一步：准备 app_task.h

先在 `User/app_task.h` 中声明应用层接口：

```c
#pragma once

#include "main.h"
#include "tim.h"

void app_task(void);
void app_startMotor(void);
void User_init(void);
void App_FREERTOS_Init(void);
void App_Timer100HzISR(void);
```

这里最重要的是两个函数：

- `App_FREERTOS_Init()`：创建闭环任务、信号量和队列。
- `App_Timer100HzISR()`：给 TIM6 中断调用，用来释放 100Hz 信号量。

`app_task()` 可以保留为空，兼容原来的工程结构。

## 4. 第二步：在 app_task.c 中包含 FreeRTOS 和 PID 头文件

在 `User/app_task.c` 中加入：

```c
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
```

说明：

- `queue.h` 用于速度样本队列。
- `semphr.h` 用于 100Hz 信号量。
- `task.h` 用于创建任务。
- `dsp/controller_functions.h` 提供 `arm_pid_f32()`。

## 5. 第三步：定义控制周期、PID 参数和样本结构体

在 `app_task.c` 中加入：

```c
#define SPEED_CONTROL_HZ               100.0f
#define SPEED_SAMPLE_QUEUE_LENGTH      1u
#define SPEED_SAMPLE_TASK_STACK_WORDS  256u
#define SPEED_PID_TASK_STACK_WORDS     256u
#define SPEED_SAMPLE_TASK_PRIORITY     (tskIDLE_PRIORITY + 3u)
#define SPEED_PID_TASK_PRIORITY        (tskIDLE_PRIORITY + 2u)

#define SPEED_PID_KP                   18.0f
#define SPEED_PID_KI                   1.2f
#define SPEED_PID_KD                   0.0f
#define SPEED_PID_OUTPUT_LIMIT         ((float)TB6612_PWM_MAX_DUTY)
```

然后定义速度样本：

```c
typedef struct
{
    float target_rpm;
    float measured_rpm;
} SpeedSample_t;
```

这个结构体就是生产者和消费者之间传递的数据。

队列长度设置为 `1` 是故意的，因为速度闭环只关心最新速度。旧数据没有意义。

## 6. 第四步：创建全局对象

继续在 `app_task.c` 中加入：

```c
static SemaphoreHandle_t speed_tick_sem = NULL;
static QueueHandle_t speed_sample_queue = NULL;
static arm_pid_instance_f32 speed_pid;
static float last_target_rpm = 0.0f;
```

含义：

- `speed_tick_sem`：100Hz 控制节拍。
- `speed_sample_queue`：速度样本队列。
- `speed_pid`：CMSIS-DSP 的 PID 对象。
- `last_target_rpm`：用于检测目标方向是否改变，方向改变时重置 PID。

## 7. 第五步：写 FreeRTOS 初始化函数

实现 `App_FREERTOS_Init()`：

```c
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
```

注意 PID 参数这里做了采样周期换算：

- `Ki / SPEED_CONTROL_HZ`：积分项按 100Hz 周期缩放。
- `Kd * SPEED_CONTROL_HZ`：微分项按 100Hz 周期缩放。

如果你还没声明任务函数，需要在文件前面加：

```c
static void SpeedSampleTask(void *argument);
static void SpeedPidTask(void *argument);
static float App_ClampFloat(float value, float min_value, float max_value);
```

## 8. 第六步：把 FreeRTOS 初始化函数接入工程

打开 `Core/Src/freertos.c`。

在用户 include 区加入：

```c
/* USER CODE BEGIN Includes */
#include "app_task.h"
/* USER CODE END Includes */
```

然后在 `MX_FREERTOS_Init()` 的线程创建区域调用：

```c
/* USER CODE BEGIN RTOS_THREADS */
App_FREERTOS_Init();
/* USER CODE END RTOS_THREADS */
```

这样 FreeRTOS 启动时，会创建我们的速度采样任务和 PID 任务。

## 9. 第七步：让 TIM6 产生 100Hz 信号量

在 `app_task.c` 中实现：

```c
void App_Timer100HzISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if((speed_tick_sem != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        xSemaphoreGiveFromISR(speed_tick_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
```

然后打开 `Core/Src/main.c`，在 `HAL_TIM_PeriodElapsedCallback()` 中加入：

```c
if (htim->Instance == TIM6)
{
    App_Timer100HzISR();
}
```

为什么要检查 `xTaskGetSchedulerState()`？

因为 `TIM6` 可能在调度器启动前就开始中断。如果此时调用 FreeRTOS ISR API，可能导致异常。这个检查可以避免启动阶段的问题。

## 10. 第八步：写生产者任务 SpeedSampleTask

生产者任务代码：

```c
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
```

这段代码做了三件事：

1. 等待 TIM6 给出的 100Hz 信号量。
2. 读取实际速度和目标速度。
3. 把速度样本写入队列，同时发给上位机。

UART 协议是：

```text
8 字节一帧
第 0~3 字节：float measured_rpm，小端
第 4~7 字节：float target_rpm，小端
```

## 11. 第九步：写消费者任务 SpeedPidTask

消费者任务代码：

```c
static void SpeedPidTask(void *argument)
{
    (void)argument;

    for(;;)
    {
        SpeedSample_t sample;

        if(xQueueReceive(speed_sample_queue, &sample, portMAX_DELAY) == pdTRUE)
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
```

这里的控制逻辑是：

```text
误差 = 目标速度 - 实际速度
PID 输出 = arm_pid_f32(误差)
电机控制 = TB6612_SetControlOutput(PID 输出)
```

当目标速度变成 0 时，重置 PID 并停止电机。

当目标速度从正变负，或从负变正时，也重置 PID，避免积分项残留导致反向时冲击太大。

限幅函数如下：

```c
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
```

## 12. 第十步：改造 TB6612 驱动

闭环控制时，`TB6612_SetRPM()` 不应该直接换算 PWM。

建议把 TB6612 驱动拆成两个层次：

- 目标速度层：保存用户想要的 RPM。
- 控制输出层：由 PID 直接控制 PWM 和方向。

在 `drv_tb6612.h` 中加入：

```c
void TB6612_SetRPM(float rpm);
void TB6612_SetControlOutput(float output);
float TB6612_GetTargetRPM(void);
void TB6612_Init(void);
```

`TB6612_SetRPM()` 只保存目标值：

```c
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
```

`TB6612_SetControlOutput()` 接收 PID 输出：

```c
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
```

`TB6612_GetTargetRPM()` 返回目标值：

```c
float TB6612_GetTargetRPM(void)
{
    return tb6612_target_rpm;
}
```

## 13. 第十一步：初始化用户模块并设置目标速度

在 `User_init()` 中启动编码器、电机和 UART：

```c
void User_init(void)
{
    Encoder_Init();
    TB6612_Init();
    UART_Init();
}
```

设置一个测试目标速度：

```c
void app_startMotor(void)
{
    TB6612_SetRPM(100.0f);
}
```

在 `main.c` 中，外设初始化完成后调用：

```c
HAL_TIM_Base_Start_IT(&htim6);
User_init();
app_startMotor();
```

注意：真正的 PID 任务会在 FreeRTOS 调度器启动后运行。

## 14. 第十二步：把 UART 改成 DMA 非阻塞

原来的阻塞发送通常是：

```c
HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
```

闭环控制中不建议这样做，因为它会让任务卡在串口发送上。

改成 DMA 发送：

```c
HAL_UART_Transmit_DMA(&huart1, uart_tx_data, len);
```

`bsp_uart.h` 建议改成：

```c
#define UART_RX_MAX_LEN 32u
#define UART_TX_MAX_LEN 64u

void UART_Init(void);
HAL_StatusTypeDef UART_SendByte(uint8_t data);
HAL_StatusTypeDef UART_SendData(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef UART_SendString(const char *str);

void UART_RxCallback(uint8_t *data, uint16_t len);
```

`UART_Init()` 中启动 ToIdle DMA 接收：

```c
void UART_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_data, UART_RX_MAX_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}
```

发送函数使用内部缓冲区，避免用户传入的局部变量在 DMA 完成前失效：

```c
HAL_StatusTypeDef UART_SendData(const uint8_t *data, uint16_t len)
{
    if((data == NULL) || (len == 0u) || (len > UART_TX_MAX_LEN))
    {
        return HAL_ERROR;
    }

    if(uart_tx_busy)
    {
        return HAL_BUSY;
    }

    memcpy(uart_tx_data, data, len);
    uart_tx_busy = 1u;

    if(HAL_UART_Transmit_DMA(&huart1, uart_tx_data, len) != HAL_OK)
    {
        uart_tx_busy = 0u;
        return HAL_ERROR;
    }

    return HAL_OK;
}
```

发送完成回调：

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uart_tx_busy = 0u;
    }
}
```

接收事件回调：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        UART_RxCallback(uart_rx_data, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_data, UART_RX_MAX_LEN);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
```

## 15. 第十三步：配置 USART1 DMA

如果 CubeMX 已经帮你生成 DMA，可以跳过本节。

如果没有，需要在 `Core/Src/usart.c` 中添加 DMA handle：

```c
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
```

USART1 常用 DMA 映射：

```text
USART1_RX -> DMA2_Stream2, Channel 4
USART1_TX -> DMA2_Stream7, Channel 4
```

初始化后要链接到 UART：

```c
__HAL_LINKDMA(uartHandle, hdmarx, hdma_usart1_rx);
__HAL_LINKDMA(uartHandle, hdmatx, hdma_usart1_tx);
```

并开启中断：

```c
HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
```

在 `Core/Src/stm32f4xx_it.c` 中添加：

```c
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}
```

## 16. 第十四步：把 CMSIS-DSP 加入构建

本项目只需要 CMSIS-DSP 的 PID 初始化和复位源文件：

```text
Libs/CMSIS-DSP-1.17.0/Source/arm_pid_init_f32.c
Libs/CMSIS-DSP-1.17.0/Source/arm_pid_reset_f32.c
```

还需要 include 路径：

```text
Libs/CMSIS-DSP-1.17.0/Include
```

如果使用 Makefile，需要加入：

```makefile
C_SOURCES += \
Libs/CMSIS-DSP-1.17.0/Source/arm_pid_init_f32.c \
Libs/CMSIS-DSP-1.17.0/Source/arm_pid_reset_f32.c

C_INCLUDES += \
-ILibs/CMSIS-DSP-1.17.0/Include
```

如果使用 EIDE，需要在 `.eide/eide.yml` 中加入 `Libs/CMSIS-DSP-1.17.0/Source` 和 include 路径。

## 17. 第十五步：上位机协议

现在 STM32 每 10ms 发送一帧：

```text
float measured_rpm
float target_rpm
```

也就是：

```text
8 字节一帧，小端 Float32
```

网页上位机应按 8 字节解析：

- 前 4 字节画实际速度曲线。
- 后 4 字节画目标速度曲线。

如果网页仍按 4 字节解析，就会把目标速度误认为下一帧实际速度，曲线会一跳一跳。

## 18. 第十六步：调试顺序

建议按这个顺序调试，不要一上来就调 PID：

1. 只启动 PWM，确认 TB6612 能正反转。
2. 手转电机，确认 `Encoder_GetRPM()` 正负方向正确。
3. 开启 UART DMA，确认上位机能收到实际 RPM 和目标 RPM。
4. 设置 `Ki = 0`、`Kd = 0`，只调 `Kp`。
5. 让电机能接近目标速度后，再慢慢增加 `Ki`。
6. 如果超调明显，再考虑加一点 `Kd`。

方向非常重要。

如果目标 RPM 为正，但测到的 RPM 为负，PID 会越调越大，电机会失控。此时检查：

```c
#define ENCODER_DIRECTION 1
#define TB6612_RPM_DIR 1.0f
```

根据实际情况改成 `-1` 或 `-1.0f`。

## 19. 常见问题

### 19.1 为什么队列长度是 1？

速度闭环只关心最新速度。旧速度样本已经过时，所以队列长度为 1，并使用 `xQueueOverwrite()`。

### 19.2 为什么不在中断里直接算 PID？

中断里应该尽量短。PID、UART、队列处理都放到任务里，可以让系统更稳定，也更容易扩展。

### 19.3 为什么 UART 发送可能返回 HAL_BUSY？

DMA 还没发完上一帧时，新的发送会返回 `HAL_BUSY`。当前 100Hz 下每帧 8 字节，115200 波特率通常足够，不会长期 busy。

### 19.4 为什么目标为 0 时要 reset PID？

如果不 reset，积分项可能残留。下次启动时电机会突然给出较大的 PWM。

## 20. 构建验证

本工程已使用以下命令编译通过：

```powershell
mingw32-make -j4 GCC_PATH="C:/Users/burim/.eide/tools/gcc_arm/bin"
```

生成文件：

```text
build/Project.elf
build/Project.hex
build/Project.bin
```

