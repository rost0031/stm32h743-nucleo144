/**
 * @file    leds.h
 * @brief   LEDs used by this project
 *
 * Having the LED definitions be in a separate header file allows the same
 * project use same LED definitions while allowing it to be compiled for
 * several different targets (windows simulations, real hardware, devkit, etc)
 * without having to copy and paste same pin definitions.
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LEDS_H
#define __LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief   LEDs available in the system
 *
 * These enumerations allow user to specify either LED number or color
 */
typedef enum {
  LED1      = 0,    /**< First LED */
  LED_GREEN = LED1, /**< Green LED */
  LED2      = 1,    /**< Second LED */
  LED_BLUE  = LED2, /**< Blue LED */
  LED3      = 2,    /**< Third LED */
  LED_RED   = LED3, /**< Red LED */
  LED_MAX           /**< LED max */
} LED_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                            /* __LEDS_H */

