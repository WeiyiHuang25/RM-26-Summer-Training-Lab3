#pragma once

#include "main.h"

#define UART_RX_MAX_LEN 32u
#define UART_TX_MAX_LEN 64u


void UART_Init(void);
HAL_StatusTypeDef UART_SendByte(uint8_t data);
HAL_StatusTypeDef UART_SendData(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef UART_SendString(const char *str);

void UART_RxCallback(uint8_t *data, uint16_t len);
