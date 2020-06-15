/**
 * @file    buttons.h
 * @brief   Buttons used by this project
 *
 * Having the Button definitions be in a separate header file allows the same
 * project use same button definitions while allowing it to be compiled for
 * several different targets (windows simulations, real hardware, devkit, etc)
 * without having to copy and paste same pin definitions.
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTONS_H
#define __BUTTONS_H

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
  BUTTON_USER1  = 0,    /**< User button 1 */
  BUTTON_MAX            /**< Buttons max */
} Button_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                         /* __BUTTONS_H */

