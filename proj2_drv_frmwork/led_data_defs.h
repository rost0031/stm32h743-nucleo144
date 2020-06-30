/**
 * @file    led_data.h
 * @brief   LED data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_DATA_DEFS_H
#define __LED_DATA_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "gpio_pins.h"
#include "gpio_data_defs.h"

/* Exported types ------------------------------------------------------------*/

/**
 *  @brief  LED state
 *
 *  LED state is technically the same as that of GPIO pins so we can just use
 *  that type
 */
typedef GpioPinState_t LEDState_t;

/**
 * @brief LED data structure definition
 */
typedef struct {
    const GpioPin_t     systemGpioPin;  /**< Which GPIO pin controls this LED */
} LedData_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                       /* __LED_DATA_H */

