/**
 * @file    led.h
 * @brief   LED driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "errors.h"
#include "board_defs.h"
#include "leds.h"
#include "led_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief   Initialize given LED
 *
 * This function initializes all hardware needed to operate an LED
 *
 * @return  None
 */
void LED_init(
        LED_t led                                  /**< [in] which system LED */
);

/**
 * @brief   DeInitialize given LED
 *
 * This function de-initializes all hardware needed to operate an LED
 *
 * @return  None
 */
void LED_deinit(
        LED_t led                                  /**< [in] which system LED */
);

/**
 * @brief   Turn on given LED
 * @return  None
 */
void LED_on(
        LED_t led                                  /**< [in] which system LED */
);

/**
 * @brief   Turn off given LED
 * @return  None
 */
void LED_off(
        LED_t led                                  /**< [in] which system LED */
);

/**
 * @brief   Set a given LED to a desired state
 * @return  None
 */
void LED_set(
        LED_t led,                                 /**< [in] which system LED */
        LEDState_t state                    /**< [in] new state to set LED to */
);

/**
 * @brief   Get given LED's current state
 * @return  None
 */
void LED_get(
        LED_t led,                                 /**< [in] which system LED */
        LEDState_t* const pState     /**< [out] storage for current LED state */
);

/**
 * @brief   Toggle a given LED to a opposite of current state
 * @return  None
 */
void LED_toggle(
        LED_t led                                  /**< [in] which system LED */
);

#ifdef __cplusplus
}
#endif

#endif                                                             /* __LED_H */

