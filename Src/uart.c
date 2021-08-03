#include "uart.h"


/**
 *	@brief Transmits a string over UART2
 *	@param string is the string to be transmitted.
 *	@param huart The UART Handle to be transmitted through
 */
void send_uart(char *string, UART_HandleTypeDef huart)
{
	//uint8_t len = strlen(string);
	if(huart.Instance == USART2){
		HAL_UART_Transmit(&huart2, (uint8_t *) string, strlen(string), 2000);
	}else if(huart.Instance == USART3){
		HAL_UART_Transmit(&huart3, (uint8_t *) string, strlen(string), 2000);
	}
}


uint32_t bigEndian(uint32_t target)
{
    unsigned int tmp = 1;
    if (*(unsigned char *)&tmp) // returns 1 if little-endian
        return target << 24 & 0xff000000 | target << 8 & 0x00ff0000 | target >> 8 & 0x0000ff00 | target >> 24 & 0x000000ff;
    else
        return target;
}

