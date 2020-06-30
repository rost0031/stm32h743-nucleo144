/**
 * @file    gpio_data.h
 * @brief   GPIO data definitions specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_DATA_DEFS_H
#define __GPIO_DATA_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gpio_pins.h"
#include "board_defs.h"
#include "stm32h7xx_ll_gpio.h"

/* Exported types ------------------------------------------------------------*/

/**
 *  @brief  GPIO pin state
 *
 *  This exists so main code doesn't have to reference STM HAL drivers directly.
 */
typedef enum {
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET,
} GpioPinState_t;

/**
 * @brief   GPIO callback function pointer type
 */
typedef void (*GPIOCallback_t)(
        GpioPin_t,                            /**< [in] Which system GPIO pin */
        GpioPinState_t                         /**< [in] current state of pin */
);

/**
 * @brief   GPIO interrupt data
 *
 * This structure type allows specification of all data needed to control and
 * configure GPIO interrupt.
 */
typedef struct {
    const IRQn_Type irq;                                /**< Interrupt number */
    const IRQPrio_t prio;                             /**< Interrupt priority */
    GPIOCallback_t  callback;              /**< Callback to call on interrupt */
} GpioIrqData_t;

/**
 * @brief GPIO data structure definition
 */
typedef struct {
    const LL_GPIO_InitTypeDef   init;            /**< GPIO pin init structure */
    const GPIO_TypeDef*         port;                          /**< GPIO port */
    GpioIrqData_t* const        pInt;      /**< GPIO interrupt data if needed */
} GpioData_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                  /* __GPIO_DATA_DEFS_H */

