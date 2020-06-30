/**
 * @file    gpio_pins.h
 * @brief   GPIO pins used by this project
 *
 * Having the pin definitions be in a separate header file allows the same
 * project use same pin definitions while allowing it to be compiled for
 * several different targets (windows simulations, real hardware, devkit, etc)
 * without having to copy and paste same pin definitions.
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_PINS_H
#define __GPIO_PINS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief   GPIO Output Pins
 */
typedef enum {
    GPIO_PIN_START          = 0,                       /**< Start of all pins */
    GPIO_OUT_PIN_START      = GPIO_PIN_START,       /**< Start of output pins */

    GPIO_OUT_PIN_LED1       = GPIO_PIN_START,           /**< out pin for LED1 */
    GPIO_OUT_PIN_LED2,                                  /**< out pin for LED2 */
    GPIO_OUT_PIN_LED3,                                  /**< out pin for LED3 */

    GPIO_OUT_PIN_END,                                 /**< End of output pins */

    GPIO_IN_PIN_START       = GPIO_OUT_PIN_END,      /**< Start of input pins */

    GPIO_IN_PIN_USER_BUTTON = GPIO_IN_PIN_START,  /**< in pin for user button */

    GPIO_IN_PIN_END,                                   /**< End of input pins */

    GPIO_ALT_PIN_START      = GPIO_IN_PIN_END,    /**< Start of Alt Func pins */
    GPIO_ALT_PIN_UART3_TX   = GPIO_ALT_PIN_START, /**< Alt Func pin for UART3 TX */
    GPIO_ALT_PIN_UART3_RX,                     /**< Alt Func pin for UART3 RX */

    GPIO_ALT_PIN_END,                               /**< End of Alt Func pins */

    GPIO_PIN_MAX            = GPIO_ALT_PIN_END              /**< GPIO pin max */
} GpioPin_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                       /* __GPIO_PINS_H */

