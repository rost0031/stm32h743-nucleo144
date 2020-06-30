/**
 * @file    button_data_defs.h
 * @brief   Button data definitions specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_DATA_DEFS_H
#define __BUTTON_DATA_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "gpio_pins.h"
#include "gpio_data_defs.h"
#include "btns.h"

/* Exported types ------------------------------------------------------------*/

/**
 *  @brief  Button state
 *
 *  Button state is technically the same as that of GPIO pins so we can just use
 *  that type
 */
typedef GpioPinState_t ButtonState_t;

/**
 * @brief   Button callback function pointer type with button and state as arguments
 */
typedef void (*ButtonCallback_t)(Button_t,ButtonState_t);

/**
 * @brief   Button callback data
 *
 * This structure type allows specification of a user callback to activate upon
 * a button press if desired by the user
 */
typedef struct {
    ButtonCallback_t  callback;            /**< Callback to call on interrupt */
} ButtonIrqData_t;

/**
 * @brief Button data structure definition
 */
typedef struct {
    const GpioPin_t     systemGpioPin;  /**< Which GPIO pin controls this LED */
    ButtonIrqData_t* const pInt;              /**< Button user interrupt data */
} ButtonData_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                /* __BUTTON_DATA_DEFS_H */

