/**
 * @file    bsp.c
 * @brief   Board support package
 *
 * This file allows separation of anything board related to be in its own
 * module. This allows better encapsulation of hardware
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
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_dma_ex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_nucleo_144.h"

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
/**
 * @brief
 */
static void     SystemClock_Config(void);
static void     Error_Handler(void);

/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
int main(void)
{
    /* STM32H7xx HAL library initialization:
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Set NVIC Group Priority to 4
         - Low Level Initialization
     */
    HAL_Init();

    SCB_EnableICache();

#ifdef CACHE_ENABLED
    SCB_EnableDCache();
#else
    SCB_DisableDCache();
#endif
    /* Configure the system clock to 400 MHz */
    SystemClock_Config();

    /* Initialize LEDs mounted on STM32H743ZI-NUCLEO board */
    BSP_LED_Init(LED1);
    BSP_LED_Init(LED2);
    BSP_LED_Init(LED3);

    /* Communication done with success : Turn the GREEN LED on */
    BSP_LED_On(LED1);

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

        BSP_LED_On(LED2);
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
            BSP_LED_On(LED3);
        }
        BSP_LED_Off(LED3);
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
                BSP_LED_Off(LED1);
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
    BSP_LED_Off(LED2);

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
        BSP_LED_Off(LED2);
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
    BSP_LED_On(LED3);
    HAL_UART_RxIdleCallback(UartHandle);
    BSP_LED_Off(LED3);

}
/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL1 (HSE BYPASS)
 *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
 *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
 *            AHB Prescaler                  = 2
 *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
 *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
 *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
 *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 4
 *            PLL_N                          = 400
 *            PLL_P                          = 2
 *            PLL_Q                          = 4
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Flash Latency(WS)              = 4
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    HAL_StatusTypeDef ret = HAL_OK;

    /*!< Supply configuration update enable */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /* The voltage scaling allows optimizing the power consumption when the device is
	     clocked below the maximum system frequency, to update the voltage scaling value
	     regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 400;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
    ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if(ret != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure  bus clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
            RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
    if(ret != HAL_OK)
    {
        Error_Handler();
    }

    /*activate CSI clock mondatory for I/O Compensation Cell*/
    __HAL_RCC_CSI_ENABLE() ;

    /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
    __HAL_RCC_SYSCFG_CLK_ENABLE() ;

    /* Enables the I/O Compensation Cell */
    HAL_EnableCompensationCell();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
    /* Turn LED3 on */
    BSP_LED_Off(LED1);
    while(1)
    {
    }
}

