/**
 * @file    main.c
 * @brief   Main for UART LL example
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_usart.h"


/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

//#define USE_FULL_LL_DRIVER

/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

uint32_t timeout = 0;

/* GPIO Init Structure */
LL_GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void     SystemClock_Config(void);
//static void     Error_Handler(void);
static void     LED1_Init(void);
static void     LED3_Init(void);

/* Public and Exported functions ---------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
    timeout = 0xFF;

    /* Configure the system clock to 400 MHz */
    SystemClock_Config();

    /* Initialize LED1 */
    LED1_Init();
    LED3_Init();

    /* Communication done with success : Turn the GREEN LED on */
    LL_GPIO_SetOutputPin(LED1_GPIO_PORT, LED1_PIN);

    /* Configure USART3 for debug output. Configure pins and uart peripheral */

    /* Enable PORTD clock for GPIO port D */
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

    /* Configure PD8 as Tx Pin : Alternate function 7, High Speed, Push pull, Pull up */
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_8, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

    /* Configure PD9 as Rx Pin : Alternate function 7, High Speed, Push pull, Pull up */
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_9, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

    /* Enable the peripheral clock of UART */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);


#if 0
    /* The function LL_USART_Init is horribly broken or is not meant to be used
     * with this chip. It sets the baudrate incorrectly. Even the LL examples in
     * manually configure the UARTs. Thanks again, STM. */
    LL_USART_InitTypeDef usartInit = {0};
    usartInit.BaudRate            = 115200U;
	usartInit.DataWidth           = LL_USART_DATAWIDTH_8B;
    usartInit.StopBits	          = LL_USART_STOPBITS_1;
    usartInit.Parity              = LL_USART_PARITY_NONE;
    usartInit.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usartInit.TransferDirection	  = LL_USART_DIRECTION_TX_RX;
    usartInit.OverSampling        = LL_USART_OVERSAMPLING_16;
    usartInit.PrescalerValue      = LL_USART_PRESCALER_DIV1;
    LL_USART_Init( USART3, &usartInit);
#endif

    /* TX/RX direction */
    LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_TX_RX);

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    LL_USART_ConfigCharacter(USART3, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

    LL_USART_SetBaudRate(USART3, SystemCoreClock/4, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, 115200);

    LL_USART_Enable( USART3 );

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
    	for(uint8_t i = 0; i < sizeof(buffer); i++) {
			while (!LL_USART_IsActiveFlag_TXE(USART3)) {}
    		LL_USART_TransmitData8(USART3, buffer[i]);
    	}
#endif
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
    /* Power Configuration */
    LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (LL_PWR_IsActiveFlag_VOS() == 0)
    {
    }

    /* Enable HSE oscillator */
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();
    while(LL_RCC_HSE_IsReady() != 1)
    {
    }

    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

    /* Main PLL configuration and activation */
    LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
    LL_RCC_PLL1P_Enable();
    LL_RCC_PLL1Q_Enable();
    LL_RCC_PLL1R_Enable();
    LL_RCC_PLL1FRACN_Disable();
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
    LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL1_SetM(4);
    LL_RCC_PLL1_SetN(400);
    LL_RCC_PLL1_SetP(2);
    LL_RCC_PLL1_SetQ(4);
    LL_RCC_PLL1_SetR(2);
    LL_RCC_PLL1_Enable();
    while(LL_RCC_PLL1_IsReady() != 1)
    {
    }

    /* Set Sys & AHB & APB1 & APB2 & APB4  prescaler */
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    /* Set PLL1 as System Clock Source */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
    {
    }

    /* Set systick to 1ms */
    SysTick_Config(400000000 / 4000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    SystemCoreClock = 400000000;
}

/**
 * @brief  Initialize LED1 (Green LED).
 * @param  None
 * @retval None
 */
void LED1_Init(void)
{
    /* Enable the LED1 Clock */
    LED1_GPIO_CLK_ENABLE();

    /* Configure IO in output push-pull mode to drive external LED1 */
    LL_GPIO_SetPinMode(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
}

/**
 * @brief  Initialize LED3 (Red LED).
 * @param  None
 * @retval None
 */
void LED3_Init(void)
{
    /* Enable the LED3 Clock */
    LED3_GPIO_CLK_ENABLE();

    /* Configure IO in output push-pull mode to drive external LED3 */
    LL_GPIO_SetPinMode(LED3_GPIO_PORT, LED3_PIN, LL_GPIO_MODE_OUTPUT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
//static void Error_Handler(void)
//{
//    /* Turn LED3 on */
//    LL_GPIO_SetOutputPin(LED3_GPIO_PORT, LED3_PIN);
//    while(1)
//    {
//    }
//}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
