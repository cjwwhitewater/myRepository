/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HAL_UART_H
#define HAL_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include "stdlib.h"

#include "dji_platform.h"

//#ifdef __cplusplus
//extern "C" {
//#endif

/* Exported constants --------------------------------------------------------*/
//User can config dev based on there environmental conditions
#define LINUX_UART_DEV1    "/dev/ttyUSB0"
#define LINUX_UART_DEV2    "/dev/ttyACM0"

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
T_DjiReturnCode HalUart_Init(E_DjiHalUartNum uartNum, uint32_t baudRate,
                             T_DjiUartHandle *uartHandle);
T_DjiReturnCode HalUart_DeInit(T_DjiUartHandle uartHandle);
T_DjiReturnCode HalUart_WriteData(T_DjiUartHandle uartHandle, const uint8_t *buf,
                                  uint32_t len, uint32_t *realLen);
T_DjiReturnCode HalUart_ReadData(T_DjiUartHandle uartHandle, uint8_t *buf,
                                 uint32_t len, uint32_t *realLen);
T_DjiReturnCode HalUart_GetStatus(E_DjiHalUartNum uartNum, T_DjiUartStatus *status);

//#ifdef __cplusplus
//}
//#endif

#endif // HAL_UART_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
