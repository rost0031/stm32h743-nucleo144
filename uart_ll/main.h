/**
 * @file    main.h
 * @brief   Main for UART LL example
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
/* LL drivers common to all LL examples */
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_utils.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
//#define STM32H743xx
//#define USE_FULL_LL_DRIVER

/* Definition LED1-GREEN*/
#define LED1_PIN                           LL_GPIO_PIN_0
#define LED1_GPIO_PORT                     GPIOB
#define LED1_GPIO_CLK_ENABLE()             LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB)

/* Definition LED3-RED*/
#define LED3_PIN                           LL_GPIO_PIN_14
#define LED3_GPIO_PORT                     GPIOB
#define LED3_GPIO_CLK_ENABLE()             LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB)

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif                                                            /* __MAIN_H */

