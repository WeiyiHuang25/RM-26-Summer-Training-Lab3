#include "bsp_uart.h"
#include "usart.h"
#include <string.h>

static uint8_t uart_rx_data[UART_RX_MAX_LEN];
static uint8_t uart_tx_data[UART_TX_MAX_LEN];
static volatile uint8_t uart_tx_busy = 0u;

void UART_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_data, UART_RX_MAX_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

HAL_StatusTypeDef UART_SendByte(uint8_t data)
{
    return UART_SendData(&data, 1);
}

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

HAL_StatusTypeDef UART_SendString(const char *str)
{
    return UART_SendData((const uint8_t *)str, (uint16_t)strlen(str));
}

__weak void UART_RxCallback(uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        UART_RxCallback(uart_rx_data, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_data, UART_RX_MAX_LEN);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uart_tx_busy = 0u;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uart_tx_busy = 0u;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_data, UART_RX_MAX_LEN);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
