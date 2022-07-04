#ifndef _UART_H
#define _UART_H

#include "string.h"
#include "main.h"

#define UART_LEN 5

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/**
 * Todo: Fix send_uart to be able to transmit over any UART.
 */


/**
 * @brief Transmits string over UART2
 * @param string is the string to be transmitted.
 */
void send_uart(char *string, UART_HandleTypeDef huart);

uint32_t bigEndian(uint32_t target);

#endif
