/**
 * @file    bsp.c
 * @brief   Board Support Package
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

#include "btn.h"
#include "stm32h7xx.h"
#include "stm32h743xx.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"

#include "led.h"
#include "leds.h"
#include "uart.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
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
 *
 * @return  Error_t code that specifies success or failure
 * @retval  ERR_NONE: success
 */
Error_t BSP_configSystemClock(void);

/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
Error_t BSP_init(void)
{
    Error_t status = ERR_NONE;

    SCB_EnableICache();                           /* Enable Instruction Cache */

    NVIC_SetPriorityGrouping(0U);                 /* No subpriority groupings */

    /* Enable all the clocks that we are going to use here. While it might make
     * sense to put these clock enables right into the drivers, some of these
     * clocks are shared (DMA and GPIO, for example), it's better to do it up
     * front so they can be enabled all in one place */
    /* Configure the system clock to 400 MHz */
    if (ERR_NONE != (status = BSP_configSystemClock())) {
        goto END;
    }

    /* While it might make sense to enable these clocks right in the driver, it
     * is not possible to disable them in the driver since multiple devices
     * might be sharing the same clock and it would be unsafe to disable that
     * clock. We could probably make a complex way of keeping track of which
     * devices use which clocks and have some automated way of knowing when
     * it's safe to disable the clocks... Another day. */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    /* Now that the clocks are enabled, we can initialize our hardware by
     * calling into the drivers */
//    BTN_init(BTN_USER1);
//    LED_init(LED1);
//    LED_init(LED2);
//    LED_init(LED3);

    /* Configure the UARTs */
    if (ERR_NONE != (status = UART_init(UART_DBG))) {
        goto END;
    }

//    if (ERR_NONE != (status = UART_start(UART_DBG))) {
//        goto END;
//    }


END:                                     /* Tag to jump to in case of failure */
    return status;
}

/******************************************************************************/
Error_t BSP_start(void)
{
    Error_t status = ERR_NONE;

    /* If we were using an RTOS, we'd want to finish up our RTOS init before
     * starting drivers but since we are bare-metal, we can go ahead and start
     * them now. */
//    BTN_start(BTN_USER1);

    if (ERR_NONE != (status = UART_start(UART_DBG))) {
        goto END;
    }

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
Error_t BSP_configSystemClock(void)
{
    Error_t status = ERR_NONE;                        /* Keep track of errors */

    /* Power Configuration */
    LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (0 == LL_PWR_IsActiveFlag_VOS()) {}     /* Wait for voltage scaling */

    /* Enable HSE oscillator */
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();

    while (1 != LL_RCC_HSE_IsReady()) {}          /* Wait for HSE to be ready */

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

    while (1 != LL_RCC_PLL1_IsReady()) {}        /* Wait for PLL1 to be ready */

    /* Set Sys & AHB & APB1 & APB2 & APB4  prescaler */
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    /* Set PLL1 as System Clock Source */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1) {}

    /* Set systick to 1ms */
    SysTick_Config(400000000 / 4000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    SystemCoreClock = 400000000;

END:                                     /* Tag to jump to in case of failure */
    return status;
}
