/*------------------------------------------------------------------------------
 -------------------------------------------------------------------------------
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 --                       THIS FILE IS UNCLASSIFIED                           --
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 -------------------------------------------------------------------------------
 -----------------------------------------------------------------------------*/

/**
 * @file    main.c
 * @brief   Main for UART HAL example
 *
 * Copyright 2020, Northrop Grumman Innovation Systems, Inc.
 * All other rights reserved.
 *
 * NORTHROP GRUMMAN PROPRIETARY LEVEL I
 * This information contains proprietary data and should not be released or
 * distributed without the express written approval of Northrop Grumman
 * Innovation Systems, Inc. This document contains private or privileged
 * information and/or trade secrets, which is/are disclosed in confidence.
 * This information may be used, duplicated, or disclosed only to the extent as
 * specifically authorized in writing by Northrop Grumman Innovation Systems,
 * Inc.
 */

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_nucleo_144.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USE_FULL_HAL_DRIVER

/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
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

    /* Configure the system clock to 400 MHz */
    SystemClock_Config();

    /* Initialize LEDs mounted on STM32H743ZI-NUCLEO board */
    BSP_LED_Init(LED1);

    /* Communication done with success : Turn the GREEN LED on */
    BSP_LED_On(LED1);

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    UART_HandleTypeDef UartHandle = {0};
    UartHandle.Instance          = USART3;

    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_OK != HAL_UART_DeInit(&UartHandle)) {
        Error_Handler();
    }

    if (HAL_OK != HAL_UART_Init(&UartHandle)) {
        Error_Handler();
    }

    const char buffer[25] = "Hello World\r\n";
    /* Infinite loop */
    while (1) {
        if(HAL_OK != HAL_UART_Transmit(&UartHandle, (uint8_t*)buffer, strlen(buffer), 5000))  {
            Error_Handler();
        }
    }
}

/* Private functions ---------------------------------------------------------*/
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

