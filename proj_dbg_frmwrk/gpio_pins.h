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
    GPIO_OUT_PIN_LED1               = 0, /**< GPIO pin for LED1 */
    GPIO_OUT_PIN_LED2               = 1, /**< GPIO pin for LED2 */
    GPIO_OUT_PIN_LED3               = 2, /**< GPIO pin for LED3 */

    GPIO_IN_PIN_USER_BUTTON         = 3, /**< GPIO pin for user button */

    GPIO_ALT_PIN_UART3_TX           = 4, /**< GPIO pin for UART3 TX */
    GPIO_ALT_PIN_UART3_RX           = 5, /**< GPIO pin for UART3 RX */

    GPIO_PIN_MAX                     /**< GPIO pin max */
} GpioPin_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                       /* __GPIO_PINS_H */

