/**
 * @file    main.c
 * @brief   Main for UART HAL example
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_dma_ex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
//#include "stm32h7xx_nucleo_144.h"
#include "led.h"
#include "uart.h"
#include "bsp.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USE_FULL_HAL_DRIVER

/* Uncomment to try with cache enabled */
//#define CACHE_ENABLED

/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

UART_HandleTypeDef UartHandle = {0};       /**< UART handle that we are using */

__IO ITStatus uartStatus = RESET;/**< Flag to let us know if UART is finished */

static uint16_t rxBytes = 0;          /**< Number of bytes received over UART */
static uint16_t txBytes = 0;           /**< Number of bytes to send over UART */

static bool uartTxBusy = false;
static bool uartRxBusy = false;

/* These buffers are aligned to 32 byte boundaries since DMA requires it. */

/* Buffer for sending DMA data over UART */
__attribute__((aligned(32))) static uint8_t txBuffer[64]  = {0};

/* Buffer for receiving DMA data over UART */
__attribute__((aligned(32))) static uint8_t rxBuffer[64]  = {0};

/* Buffer for copying received data in rxBuffer after finishing so it
 * can be used. This probably doesn't need to be aligned but you could
 * in theory use DMA to transfer data out of the rxBuffer to this one
 * so we'll leave it aligned */
__attribute__((aligned(32))) static uint8_t rxOutBuffer[64]  = {0};

/* Private function prototypes -----------------------------------------------*/

static void UART_rxCallback(Uart_t uart, const uint8_t* const pData, uint16_t len);
static void UART_txCallback(Uart_t uart, const uint8_t* const pData, uint16_t len);

static void     Error_Handler(void);

/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
int main(void)
{
    Error_t status = ERR_NONE;

    /* We have to handle caches here before we make any function calls or we'll
     * get a Hard Fault on data cache enable/disable */
    SCB_EnableICache();

#ifdef CACHE_ENABLED
    SCB_EnableDCache();
#else
    SCB_DisableDCache();
#endif

    /* Initialize our board and all the drivers */
    BSP_init();

    /* Communication done with success : Turn the GREEN LED on */
    LED_on(LED1);

    uint8_t counter = 0;                  /* Just for printing out our cycles */

    /* Set up our callbacks */
    UART_regCallbackDataSent(UART_DBG, UART_txCallback);
    UART_regCallbackDataRcvd(UART_DBG, UART_rxCallback);

    /* Start all of our drivers and board support drivers */
    BSP_start();

    txBytes = snprintf((char *)txBuffer, sizeof(txBuffer), "Type something to see it echoed back\n");
    UART_sendDma(UART_DBG, txBytes, txBuffer);
    uartTxBusy = true;
    txBytes = 0;

    /* Infinite loop */
    while (1) {

        LED_toggle(LED2);
        if (!uartRxBusy) {
            if (ERR_NONE != (status = UART_recvDma(UART_DBG, sizeof(rxBuffer), rxBuffer))) {
                LED_on(LED3);
            }
            uartRxBusy = true;
            LED_off(LED3);
        }

        if (0 != rxBytes && !uartTxBusy) {
            txBytes = snprintf((char *)txBuffer, sizeof(txBuffer), "Loop %03d: rcvd %d bytes: %s\n", counter++, rxBytes, rxBuffer);

            while (!uartTxBusy) {
                UART_sendDma(UART_DBG, txBytes, txBuffer);
                uartTxBusy = true;
                rxBytes = 0;
            }
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
static void UART_rxCallback(Uart_t uart, const uint8_t* const pData, uint16_t len)
{
    uartRxBusy = false;
    rxBytes = len;
}

/******************************************************************************/
static void UART_txCallback(Uart_t uart, const uint8_t* const pData, uint16_t len)
{
    uartTxBusy = false;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
    /* Turn LED3 on */
//    BSP_LED_Off(LED1);

    LED_set(LED1, GPIO_PIN_RESET);
    while(1)
    {
    }
}

