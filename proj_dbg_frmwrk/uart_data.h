/**
 * @file    uart_data.h
 * @brief   UART data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_DATA_H
#define __UART_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "uart_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Retrieve UART data for this board
 *
 * This function allows retrieval of UART data for this particular board which
 * allows the driver to be same for different boards
 *
 * @return  None
 */
void UART_getData(
        UartData_t** pUartData  /**< [in,out] pointer to retrieve UART data array */
);

#ifdef __cplusplus
}
#endif

#endif                                                       /* __UART_DATA_H */

