/**
 * @file    gpio.h
 * @brief   GPIO driver
 *
 * This GPIO driver is basically just a wrapper around whatever underlying
 * HW GPIO driver exists. The header should be the same for all implementations
 * which will allow compilation of application code for any target as long as
 * an implementation of the driver is provided that matches the interface.
 *
 * By making a wrapper around the underlying driver, the application code can
 * use common types that are not specific to any HW or simulated drivers.
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "errors.h"
#include "gpio_pins.h"
#include "gpio_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Initialize given GPIO pin
 *
 * This function will initialize a given GPIO pin.
 *
 * @return  None
 */
void GPIO_init(
        GpioPin_t pin                         /**< [in] which system gpio pin */
);

/**
 * @brief   Start given GPIO pin
 *
 * This function is really only necessary for input pins since it starts the
 * interrupt for the given pin if specified by user data file
 *
 * @return  None
 */
void GPIO_start(
        GpioPin_t pin                         /**< [in] which system gpio pin */
);

/**
 * @brief   Register a new callback
 *
 * This function only applies to input pins and will have no effect on output
 * or alt function pins
 *
 * @return  None
 */
void GPIO_regCallback(
        GpioPin_t pin,                        /**< [in] which system gpio pin */
        GPIOCallback_t callback               /**< [in] new callback to add */
);

/**
 * @brief   Clear a set callback
 *
 * This function only has an effect on input pins that have a set callback
 *
 * @return  None
 */
void GPIO_clrCallback(
        GpioPin_t pin                         /**< [in] which system gpio pin */
);

/**
 * @brief   Set a pin to a desired state
 *
 * This function only applies to pins configured as output. It will have no
 * effect on input or alt function configured pins.
 *
 * @return  None
 */
void GPIO_set(
        GpioPin_t pin,                        /**< [in] which system gpio pin */
        GpioPinState_t state                   /**< [in] new pin state to set */
);

/**
 * @brief   Get a pin's current state
 *
 * @note:   This function will probably not work for alt function pins but should
 * work for both input and output pins, depending on underlying HW.
 *
 * @return  None
 */
void GPIO_get(
        GpioPin_t pin,                        /**< [in] which system gpio pin */
        GpioPinState_t* const pState  /**< [out] read-out pin state to return */
);

/**
 * @brief   Toggle a given pin's value
 *
 * @return  None
 */
void GPIO_toggle(
        GpioPin_t pin                         /**< [in] which system gpio pin */
);

/**
 * @brief   Deinitialize a pin
 *
 * This function will first stop the pin (if input and interrupts enabled for it)
 * and then deinitialize the pin.
 *
 * @note:   This function does not disable the clock since it could be used by
 * other devices
 *
 * @return  None
 */
void GPIO_deinit(
        GpioPin_t pin                         /**< [in] which system gpio pin */
);

/**
 * @brief   Stop a pin
 *
 * This disables the interrupt if enabled on a given pin. Has no effect on
 * non-input configured pins.
 *
 * @return  None
 */
void GPIO_stop(
        GpioPin_t pin                         /**< [in] which system gpio pin */
);

#ifdef __cplusplus
}
#endif

#endif                                                            /* __GPIO_H */

