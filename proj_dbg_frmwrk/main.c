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
static void     Error_Handler(void);

/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
int main(void)
{

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

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */

    UartHandle.Instance          = USART3;

    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /* Is this really necessary? */
    if (HAL_OK != HAL_UART_DeInit(&UartHandle)) {
        Error_Handler();
    }

    if (HAL_OK != HAL_UART_Init(&UartHandle)) {
        Error_Handler();
    }

    uint8_t counter = 0;                  /* Just for printing out our cycles */

    /* Infinite loop */
    while (1) {

//        BSP_LED_On(LED2);
        LED_set(LED2, GPIO_PIN_SET);
        HAL_StatusTypeDef status = HAL_OK;

#ifdef CACHE_ENABLED
        /* Invalidate cache prior to access by CPU */
        SCB_CleanDCache_by_Addr ((uint32_t *)txBuffer, sizeof(txBuffer));
#endif
//#ifdef CACHE_ENABLED
//        /* This is not working like it should. Perhaps rxBytes is being caught up in the invalidation?*/
////        SCB_CleanInvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
//        SCB_InvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
//#endif
        snprintf((char *)txBuffer, sizeof(txBuffer), "Loop %03d: rcvd %d bytes: %s\n", counter++, rxBytes, rxOutBuffer);
        uartStatus = RESET; /* Reset flag before transmitting */
        while (HAL_OK != (status = HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)txBuffer, strlen((const char *)txBuffer)))) {
            /* We might have to wait for a bit before being able to send if an error occurred during
             * last operation. */
//            BSP_LED_On(LED3);

            LED_set(LED3, GPIO_PIN_SET);
        }
//        BSP_LED_Off(LED3);

        LED_set(LED3, GPIO_PIN_RESET);
        while (SET != uartStatus ) {}    /* Wait for DMA to complete with dump polling for now */

        uartStatus = RESET; /* Reset flag before transmitting */

        /* Clear out our "command" buffer before we go to receive more data */
        memset(rxOutBuffer, 0, sizeof(rxOutBuffer));

//#ifdef CACHE_ENABLED
//        /* This is not working like it should. Perhaps rxBytes is being caught up in the invalidation?*/
//        SCB_CleanInvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
//#endif
        rxBytes = 0;
        if (HAL_OK != (status = HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)rxBuffer, sizeof(rxBuffer)))) {
            if (HAL_ERROR == status) {
                Error_Handler();
            } else {
//                BSP_LED_Off(LED1);
                LED_set(LED1, GPIO_PIN_RESET);
            }
        }

        while (SET != uartStatus ) {}    /* Wait for DMA to complete with dump polling for now */
    }
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of DMA Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    /* Set transmission flag: transfer complete */
    uartStatus = SET;

    /* Turn LED2 off: Transfer in transmission process is correct */
//    BSP_LED_Off(LED2);

    LED_set(LED2, GPIO_PIN_RESET);

}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    /* Some error checking */
    if (NULL == UartHandle) {
        return;
    }

    if (USART3 == UartHandle->Instance) {

        if (NULL == UartHandle->hdmarx) {
            return;
        }

        /* Determine how many items of data have been received */
        rxBytes = UartHandle->RxXferSize - __HAL_DMA_GET_COUNTER(UartHandle->hdmarx);

        HAL_DMA_Abort(UartHandle->hdmarx);

        UartHandle->RxXferCount = 0;
        /* Check if a transmit process is ongoing or not */
        if(UartHandle->gState == HAL_UART_STATE_BUSY_TX_RX) {
            UartHandle->gState = HAL_UART_STATE_BUSY_TX;
        } else {
            UartHandle->gState = HAL_UART_STATE_READY;
        }
#ifdef CACHE_ENABLED
        /* This is not working like it should. Perhaps rxBytes is being caught up in the invalidation?*/
        //        SCB_CleanInvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
        SCB_InvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
#endif
        memcpy( rxOutBuffer, rxBuffer, rxBytes);

        uartStatus = SET;

        /* Turn LED2 off: Transfer in transmission process is correct */
//        BSP_LED_Off(LED2);

        LED_set(LED2, GPIO_PIN_RESET);
    }

}

/**
 * @brief  UART receive process idle callback for short RX DMA transfers.
 *
 * @note   This function was added as part of a modification to stm32h7xx_hal_uart.c/h
 *         that add handling of the IDLE interrupt. People on forums have been
 *         begging STM to add this functionality since it's the only rational
 *         way to deal with high speed uart rx of variable length data but STM
 *         is gonna STM...
 *         https://community.st.com/s/question/0D50X00009XkhGfSAJ/cubemx-feature-request-add-usart-rx-idle-handling
 *         While it might be possible (or perhaps even better) to use the Receiver
 *         Timeout interrupt, STM HAL drivers treat it as an error so that's not
 *         a great path despite it being more configurable.
 *
 * @param  UartHandle: Pointer to a UART_HandleTypeDef structure that contains
 *         the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxIdleCallback(UART_HandleTypeDef* UartHandle)
{
    /* Some error checking */
    if (NULL == UartHandle) {
        return;
    }

    if (USART3 == UartHandle->Instance) {

        if (NULL == UartHandle->hdmarx) {
            return;
        }

        /* Overrun error means our DMA buffer ran out of space before IDLE went off */
        if (UartHandle->ErrorCode & HAL_UART_ERROR_ORE) {
            rxBytes = UartHandle->RxXferSize;
        } else {
            /* Determine how many items of data have been received */
            rxBytes = UartHandle->RxXferSize - __HAL_DMA_GET_COUNTER(UartHandle->hdmarx);
        }

        HAL_DMA_Abort(UartHandle->hdmarx);

        UartHandle->RxXferCount = 0;
        /* Check if a transmit process is ongoing or not */
        if(UartHandle->gState == HAL_UART_STATE_BUSY_TX_RX) {
            UartHandle->gState = HAL_UART_STATE_BUSY_TX;
        } else {
            UartHandle->gState = HAL_UART_STATE_READY;
        }
#ifdef CACHE_ENABLED
        /* This is not working like it should. Perhaps rxBytes is being caught up in the invalidation?*/
//        SCB_CleanInvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
        SCB_InvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
#endif
        memcpy( rxOutBuffer, rxBuffer, rxBytes);

        uartStatus = SET;
    }
}

/**
 * @brief  UART error callbacks
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    /* Set transmission flag: transfer complete */
    uartStatus = SET;
    /* Turn LED2 off: Transfer in transmission process is correct */
//    BSP_LED_On(LED3);

    LED_set(LED3, GPIO_PIN_SET);
    HAL_UART_RxIdleCallback(UartHandle);
//    BSP_LED_Off(LED3);

    LED_set(LED3, GPIO_PIN_RESET);

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

