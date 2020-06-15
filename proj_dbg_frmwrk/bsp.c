/**
 * @file    bsp.c
 * @brief   Board Support Package
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_pwr_ex.h"

#include "button.h"
#include "led.h"
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

    /* STM32H7xx HAL library initialization:
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Set NVIC Group Priority to 4
         - Low Level Initialization
     */
    if (HAL_OK != HAL_Init()) {
        status = ERR_HW_INIT_FAILURE; goto END;
    }




    /* Enable all the clocks that we are going to use here. While it might make
     * sense to put these clock enables right into the drivers, some of these
     * clocks are shared (DMA and GPIO, for example), it's better to do it up
     * front so they can be enabled all in one place */

    /* Configure the system clock to 400 MHz */
    if (ERR_NONE != (status = BSP_configSystemClock())) {
        goto END;
    }

    /* Enable caches. Instruction cache is pretty safe to enable but data cache
     * is tricky, especially if DMA is used. This is why the DCache is behind a
     * compile flag. It's really not worth the trouble in most cases. The
     * performance gains are minimal if on-die RAM is used. */
//    SCB_EnableICache();
//
//#ifdef CACHE_ENABLED
//    SCB_EnableDCache();
//#else
////    SCB_DisableDCache();
//#endif

    /* While it might make sense to enable these clocks right in the driver, it
     * is not possible to disable them in the driver since multiple devices
     * might be sharing the same clock and it would be unsafe to disable that
     * clock. We could probably make a complex way of keeping track of which
     * devices use which clocks and have some automated way of knowing when
     * it's safe to disable the clocks... Another day. */
    __HAL_RCC_DMA2_CLK_ENABLE();                          /* Enable DMA clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();                       /* Enable GPIOB clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();                       /* Enable GPIOC clock */
    __HAL_RCC_GPIOD_CLK_ENABLE();                       /* Enable GPIOD clock */
    __HAL_RCC_USART3_CLK_ENABLE();                   /* Enable the UART clock */

    /* Now that the clocks are enabled, we can initialize our hardware by
     * calling into the drivers */
    BUT_init(BUTTON_USER1);
    LED_init(LED1);
    LED_init(LED2);
    LED_init(LED3);

    /* If we were using an RTOS, we'd want to finish up our RTOS init before
     * starting drivers but since we are bare-metal, we can go ahead and start
     * them now. */
    BUT_start(BUTTON_USER1);

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
Error_t BSP_configSystemClock(void)
{
    Error_t status = ERR_NONE;                        /* Keep track of errors */
    HAL_StatusTypeDef ret = HAL_OK;               /* Keep track of HAL errors */


    /*!< Supply configuration update enable */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /* The voltage scaling allows optimizing the power consumption when the device is
         clocked below the maximum system frequency, to update the voltage scaling value
         regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitTypeDef RCC_OscInitStruct;
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

    if (HAL_OK != (ret = HAL_RCC_OscConfig(&RCC_OscInitStruct))) {
        status = ERR_HW_INIT_FAILURE; goto END;
    }

    /* Select PLL as system clock source and configure  bus clocks dividers */
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  |
                                   RCC_CLOCKTYPE_HCLK    |
                                   RCC_CLOCKTYPE_D1PCLK1 |
                                   RCC_CLOCKTYPE_PCLK1   |
                                   RCC_CLOCKTYPE_PCLK2   |
                                   RCC_CLOCKTYPE_D3PCLK1);

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_OK != (ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4))) {
        status = ERR_HW_INIT_FAILURE; goto END;
    }

    /*activate CSI clock mondatory for I/O Compensation Cell*/
    __HAL_RCC_CSI_ENABLE() ;

    /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
    __HAL_RCC_SYSCFG_CLK_ENABLE() ;

    /* Enables the I/O Compensation Cell */
    HAL_EnableCompensationCell();

END:                                     /* Tag to jump to in case of failure */
    return status;
}
