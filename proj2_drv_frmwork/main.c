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
#include <stdio.h>
#include <string.h>

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
/* Buffer for sending DMA data over UART */
__attribute__((aligned(32))) static uint8_t txBuffer[64]  = {0};

/* Buffer for receiving DMA data over UART */
__attribute__((aligned(32))) static uint8_t rxBuffer[64]  = {0};

static Buffer_t bufRx = {
        .len = 0,
        .maxLen = sizeof(rxBuffer),
        .pData = rxBuffer,
};

static Buffer_t bufTx = {
        .len = 0,
        .maxLen = sizeof(txBuffer),
        .pData = txBuffer,
};

static bool isRxDataRcvd = false;
static bool isTxDataSent = true;

/* Private function prototypes -----------------------------------------------*/

static void UART_dataRcvd(Uart_t port, Error_t status, Buffer_t* pBuffer);
static void UART_dataSent(Uart_t port, Error_t status, Buffer_t* pBuffer);

/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
int main(void)
{
    BSP_init();                   /* Initialize our board and all the drivers */

    const uint8_t buffer[25] = "Hello World\r\n\0";

    /* Register callbacks with any drivers we are interested in */
    UART_regCallback(UART_DBG, UartEvtDataRcvd, UART_dataRcvd);
    UART_regCallback(UART_DBG, UartEvtDataSent, UART_dataSent);

    BSP_start();     /* After setting all our callback, start all the drivers */

    if (ERR_NONE != UART_sendDma(UART_DBG, strlen(buffer), buffer)) {
        /* Not much we can do here for error handling/printing */
    }


    if (ERR_NONE != UART_recvDma(UART_DBG, bufRx.maxLen, bufRx.pData)) {
        /* Not much we can do here for error handling/printing */
    }

    while(1) {
        uint8_t counter = 0;
        while (1) {

            if (isRxDataRcvd) {

                isRxDataRcvd = false;                     /* Clear local flag */

                /* Copy from reception to transmission buffer to print out */
                bufTx.len = snprintf(bufTx.pData, bufTx.maxLen,
                        "Rcvd %d: %s\n", bufRx.len, bufRx.pData);
                bufRx.len = 0;                  /* Clear the reception buffer */

                /* Start a new reception */
                if (ERR_NONE != UART_recvDma(UART_DBG, bufRx.maxLen, bufRx.pData)) {
                    /* Not much we can do here for error handling/printing */
                }

                if (ERR_NONE != UART_sendDma(UART_DBG, bufTx.len, bufTx.pData)) {
                    /* Not much we can do here for error handling/printing */
                }
            }
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
static void UART_dataRcvd(Uart_t port, Error_t status, Buffer_t* pBuffer)
{
    bufRx.len = pBuffer->len;
    isRxDataRcvd = true;
}

/******************************************************************************/
static void UART_dataSent(Uart_t port, Error_t status, Buffer_t* pBuffer)
{
    bufTx.len = 0;                                            /* Clear buffer */
    isTxDataSent = true;                                        /* Clear flag */
}
