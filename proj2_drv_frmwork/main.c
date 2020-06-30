/**
 * @file    main.c
 * @brief   Main for UART LL example
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "uart.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
int main(void)
{
    /* Initialize our board and all the drivers */
    BSP_init();
    const uint8_t buffer[25] = "Hello World\r\n\0";
    while(1) {
        uint8_t counter = 0;
        while (1) {

            if (ERR_NONE != UART_sendDma(UART_DBG, sizeof(buffer), buffer)) {

            }


        }
    }
}

/* Private functions ---------------------------------------------------------*/

