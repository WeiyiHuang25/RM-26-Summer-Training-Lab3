# TB6612 PWM 项目改造说明

本文档说明本项目如何从原来的开环 PWM/串口阻塞发送结构，改造成现在的 FreeRTOS + CMSIS-DSP PID + UART DMA ToIdle 的 100Hz 速度闭环项目。

## 1. 原有项目结构

原项目已经具备以下基础模块：

- `TIM8_CH1` 输出 PWM，用于控制 TB6612 的 PWMA/PWMB。
- `TIM1` 工作在 Encoder Interface 模式，用于读取电机编码器。
- `TIM6` 配置为 10ms 周期中断，即 100Hz。
- `drv_tb6612.c` 封装 TB6612 方向控制和占空比控制。
- `bsp_encoder.c` 根据编码器计数差计算 RPM。
- `bsp_uart.c` 使用阻塞式 `HAL_UART_Transmit()` 发送数据。
- `app_task.c` 中通过 TIM6 中断置位 `flag100hz`，主循环轮询该标志后读取 RPM 并通过 UART 发出。

原来的控制方式本质上是开环：

```c
TB6612_SetRPM(100.0f);
```

这个函数会根据目标 RPM 与最大 RPM 的比例直接换算 PWM 占空比，但不会根据实际编码器速度修正误差。

## 2. 改造目标

本次改造实现了四个目标：

1. 使用 `Libs/CMSIS-DSP-1.17.0` 中的 PID 库实现速度闭环。
2. 使用 FreeRTOS 信号量产生 100Hz 控制节拍。
3. 使用生产者消费者模型组织速度采样和 PID 控制。
4. 将 UART 改成 DMA + ReceiveToIdle 的非阻塞方式。

改造后的整体数据流为：

```text
TIM6 100Hz 中断
    |
    v
释放 speed_tick_sem 信号量
    |
    v
SpeedSampleTask 读取 Encoder_GetRPM()
    |
    v
写入 speed_sample_queue
    |
    v
SpeedPidTask 取出速度样本
    |
    v
CMSIS-DSP arm_pid_f32()
    |
    v
TB6612_SetControlOutput()
    |
    v
PWM + 方向控制电机
```

## 3. TB6612 驱动的改造

涉及文件：

- `User/drv_tb6612.c`
- `User/drv_tb6612.h`

原来的 `TB6612_SetRPM()` 同时承担两个职责：

- 保存目标 RPM
- 直接把 RPM 映射为 PWM 占空比

闭环控制中，目标 RPM 和实际 PWM 输出应该分离。因此现在改成：

- `TB6612_SetRPM(float rpm)`：只设置目标速度。
- `TB6612_GetTargetRPM(void)`：获取当前目标速度。
- `TB6612_SetControlOutput(float output)`：由 PID 输出直接控制电机方向和 PWM 占空比。

其中 `output` 是 PID 控制量，范围被限制到：

```c
-TB6612_PWM_MAX_DUTY ~ TB6612_PWM_MAX_DUTY
```

正数表示正转，负数表示反转，绝对值表示 PWM 占空比。

## 4. 100Hz FreeRTOS 闭环任务

涉及文件：

- `User/app_task.c`
- `User/app_task.h`
- `Core/Src/freertos.c`
- `Core/Src/main.c`

### 4.1 TIM6 中断只释放信号量

原来 TIM6 中断中会设置全局变量 `flag100hz`。现在改为调用：

```c
App_Timer100HzISR();
```

该函数内部使用：

```c
xSemaphoreGiveFromISR(speed_tick_sem, &higher_priority_task_woken);
```

这样 TIM6 只负责产生 100Hz 节拍，不在中断里做耗时计算。

### 4.2 生产者任务：SpeedSampleTask

`SpeedSampleTask` 是生产者。

它等待 100Hz 信号量：

```c
xSemaphoreTake(speed_tick_sem, portMAX_DELAY)
```

每次被唤醒后：

1. 读取目标速度 `TB6612_GetTargetRPM()`。
2. 读取实际速度 `Encoder_GetRPM()`。
3. 将二者组成 `SpeedSample_t`。
4. 写入 `speed_sample_queue`。
5. 通过 DMA UART 发出遥测数据。

当前遥测数据为 8 字节：

```text
float measured_rpm
float target_rpm
```

均为小端 Float32。

### 4.3 消费者任务：SpeedPidTask

`SpeedPidTask` 是消费者。

它从队列中取出最新速度样本：

```c
xQueueReceive(speed_sample_queue, &sample, portMAX_DELAY)
```

然后计算误差：

```c
error = target_rpm - measured_rpm;
```

再调用 CMSIS-DSP PID：

```c
output = arm_pid_f32(&speed_pid, error);
```

最后将 PID 输出限制在 PWM 范围内，并传给 TB6612：

```c
TB6612_SetControlOutput(output);
```

## 5. CMSIS-DSP PID 的接入

涉及文件：

- `User/app_task.c`
- `Makefile`
- `.eide/eide.yml`

项目使用的 PID 类型是：

```c
arm_pid_instance_f32
```

初始化代码在 `App_FREERTOS_Init()` 中：

```c
speed_pid.Kp = SPEED_PID_KP;
speed_pid.Ki = SPEED_PID_KI / SPEED_CONTROL_HZ;
speed_pid.Kd = SPEED_PID_KD * SPEED_CONTROL_HZ;
arm_pid_init_f32(&speed_pid, 1);
```

当前 PID 初始参数为：

```c
#define SPEED_PID_KP 18.0f
#define SPEED_PID_KI 1.2f
#define SPEED_PID_KD 0.0f
```

注意这里做了采样周期换算：

- `Ki / 100Hz`：将积分系数换算到每个采样周期。
- `Kd * 100Hz`：将微分系数换算到每个采样周期。

为了让工程能够编译，构建配置中加入了：

```text
Libs/CMSIS-DSP-1.17.0/Include
Libs/CMSIS-DSP-1.17.0/Source/arm_pid_init_f32.c
Libs/CMSIS-DSP-1.17.0/Source/arm_pid_reset_f32.c
```

## 6. UART DMA ToIdle 改造

涉及文件：

- `User/bsp_uart.c`
- `User/bsp_uart.h`
- `Core/Src/usart.c`
- `Core/Src/stm32f4xx_it.c`
- `Core/Inc/stm32f4xx_it.h`

原来的 UART 发送使用：

```c
HAL_UART_Transmit()
```

这是阻塞式发送，会占用 CPU 等待串口发送完成。

现在改成：

```c
HAL_UART_Transmit_DMA()
```

发送完成后通过：

```c
HAL_UART_TxCpltCallback()
```

清除发送忙标志。

接收部分使用：

```c
HAL_UARTEx_ReceiveToIdle_DMA()
```

当 UART 收到数据并检测到 IDLE，或 DMA 缓冲区满时，HAL 会调用：

```c
HAL_UARTEx_RxEventCallback()
```

回调中执行用户接收回调，并重新启动 ToIdle DMA 接收。

USART1 使用的 DMA 配置为：

```text
USART1_RX -> DMA2_Stream2, Channel 4
USART1_TX -> DMA2_Stream7, Channel 4
```

对应中断函数：

```c
DMA2_Stream2_IRQHandler()
DMA2_Stream7_IRQHandler()
```

## 7. FreeRTOS 初始化接入

在 `Core/Src/freertos.c` 中，`MX_FREERTOS_Init()` 会调用：

```c
App_FREERTOS_Init();
```

该函数完成：

- 创建 `speed_tick_sem`
- 创建 `speed_sample_queue`
- 初始化 CMSIS-DSP PID
- 创建 `SpeedSampleTask`
- 创建 `SpeedPidTask`

TIM6 在 `main.c` 中启动：

```c
HAL_TIM_Base_Start_IT(&htim6);
```

为了避免调度器启动前 TIM6 中断访问 FreeRTOS API，`App_Timer100HzISR()` 内部会检查：

```c
xTaskGetSchedulerState() == taskSCHEDULER_RUNNING
```

只有调度器真正运行后才释放信号量。

## 8. 当前需要注意的点

### 8.1 PID 参数需要上板调试


实际电机、供电、电机负载、编码器方向都会影响闭环效果，需要根据波形调整。

建议调参顺序：

1. 先令 `Ki = 0`，只调 `Kp`，让速度能快速接近目标但不过度振荡。
2. 再逐渐增加 `Ki`，减小稳态误差。
3. 如果超调或振荡明显，再考虑加入少量 `Kd`。

### 8.2 编码器方向和电机方向要一致

如果目标是正转，但测得 RPM 为负，PID 会越调越大，导致失控。

此时需要检查：

```c
#define ENCODER_DIRECTION 1
#define TB6612_RPM_DIR 1.0f
```

根据实际方向改成 `-1` 或 `-1.0f`。


## 9. 构建验证

改造后已使用以下命令编译通过：

```powershell
mingw32-make -j4 GCC_PATH="C:/Users/burim/.eide/tools/gcc_arm/bin"
```

生成文件：

```text
build/Project.elf
build/Project.hex
build/Project.bin
```

