/**
 ******************************************************************************
 * @file    SPI_FullDuplex_ComIT/Src/main.c
 * @author  MCD Application Team
 * @brief   Main program body through the LL API
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#define USE_FULL_HAL_DRIVER

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_usart.h"
#include "stm32h7xx_nucleo_144.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void     SystemClock_Config(void);
static void     Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
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

	/* -1- Initialize LEDs mounted on STM32H743ZI-NUCLEO board */
	BSP_LED_Init(LED1);



    /* Communication done with success : Turn the GREEN LED on */
    BSP_LED_On(LED1);

//    /* Configure USART3 for debug output. Configure pins and uart peripheral */
//
//    /* Enable PORTD clock for GPIO port D */
//    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
//
//    /* Configure PD8 as Tx Pin : Alternate function 7, High Speed, Push pull, Pull up */
//    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
//    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_8, LL_GPIO_AF_7);
//    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
//    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
//    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
//
//    /* Configure PD9 as Rx Pin : Alternate function 7, High Speed, Push pull, Pull up */
//    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
//    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_9, LL_GPIO_AF_7);
//    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
//    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
//    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
//
//    /* Enable the peripheral clock of UART and set source */
////    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
////    LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE);
//
//    LL_USART_ClockInitTypeDef usartClkInit = {0};
//    LL_USART_ClockStructInit(&usartClkInit);
//    usartClkInit.ClockOutput = LL_USART_CLOCK_ENABLE;
//    usartClkInit.ClockPhase  = LL_USART_PHASE_1EDGE;
//    usartClkInit.ClockPolarity = LL_USART_POLARITY_HIGH;
//    usartClkInit.LastBitClockPulse = LL_USART_LASTCLKPULSE_OUTPUT;
//    LL_USART_ClockInit(USART3, &usartClkInit);
//    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
//    LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);
//
//    uint32_t Periphclk = LL_RCC_GetUSARTClockFreq(USART3);
//
//    LL_USART_InitTypeDef usartInit = {0};
//    usartInit.BaudRate            = 115200U;
//	usartInit.DataWidth           = LL_USART_DATAWIDTH_8B;
//    usartInit.StopBits	          = LL_USART_STOPBITS_1;
//    usartInit.Parity              = LL_USART_PARITY_NONE;
//    usartInit.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//    usartInit.TransferDirection	  = LL_USART_DIRECTION_TX_RX;
//    usartInit.OverSampling        = LL_USART_OVERSAMPLING_16;
//    usartInit.PrescalerValue      = LL_USART_PRESCALER_DIV1;
//    LL_USART_Init( USART3, &usartInit);
//    LL_USART_Enable( USART3 );

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
          - Word Length = 8 Bits
          - Stop Bit = One Stop bit
          - Parity = None
          - BaudRate = 115200 baud
          - Hardware flow control disabled (RTS and CTS signals) */

//    UART_HandleTypeDef UartHandle;
//    UartHandle.Instance        = USART3;
//
//    UartHandle.Init.BaudRate     = 115200;
//    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
//    UartHandle.Init.StopBits     = UART_STOPBITS_1;
//    UartHandle.Init.Parity       = UART_PARITY_NONE;
//    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
//    UartHandle.Init.Mode         = UART_MODE_TX_RX;
//    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
//    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//
//    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
//    {
//    	Error_Handler();
//    }
//    if(HAL_UART_Init(&UartHandle) != HAL_OK)
//    {
//    	Error_Handler();
//    }

    const uint8_t buffer[25] = "Hello World\r\n";
    /* Infinite loop */
    while (1) {
#if 0
    	for( uint8_t i = 0; i < 13; i++ ) {
			/* Wait for TXE flag to be raised */
			while (!LL_USART_IsActiveFlag_TXE(USART3))
			{

			}

			/* Write character in Transmit Data register.
			   TXE flag is cleared by writing data in TDR register */
			LL_USART_TransmitData8(USART3, buffer[i]);
    	}
#else
    	for(uint8_t i = 0; i < 0xff; i++) {
//			while (!LL_USART_IsActiveFlag_TXE(USART3)) {}
//    		LL_USART_TransmitData8(USART3, i);
    	}
#endif
    }
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

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
