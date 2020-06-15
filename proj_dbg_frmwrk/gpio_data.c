/**
 * @file    gpio.c
 * @brief   GPIO data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio_data_defs.h"
#include "gpio_data.h"
#include "gpio_pins.h"
#include "stm32h743xx.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

/**
 * @brief   Interrupt data for the User button
 */
GpioIrqData_t buttonInterruptData = {
        .irq = EXTI15_10_IRQn,
        .prio = IRQ_PRIO_BUTTON_EXTI,
        .callback = NULL,
};

/**
 * @brief   Array of pin data
 *
 * This array contains data about how to initialize the various GPIO pins used
 * by the system.
 * @note: the array is const so it will be placed into flash to save RAM.
 */
const GpioData_t pins[GPIO_PIN_MAX] = {
        [GPIO_OUT_PIN_LED1] = {
                .init = {
                        .Pin   = GPIO_PIN_0,
                        .Mode  = GPIO_MODE_OUTPUT_PP,
                        .Pull  = GPIO_NOPULL,
                        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                },
                .port = GPIOB,
                .pInt = NULL,   /**< No interrupt for output */
        },
        [GPIO_OUT_PIN_LED2] = {
                .init = {
                        .Pin   = GPIO_PIN_7,
                        .Mode  = GPIO_MODE_OUTPUT_PP,
                        .Pull  = GPIO_NOPULL,
                        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                },
                .port = GPIOB,
                .pInt = NULL,   /**< No interrupt for output */
        },
        [GPIO_OUT_PIN_LED3] = {
                .init = {
                        .Pin   = GPIO_PIN_14,
                        .Mode  = GPIO_MODE_OUTPUT_PP,
                        .Pull  = GPIO_NOPULL,
                        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                },
                .port = GPIOB,
                .pInt = NULL,   /**< No interrupt for output */
        },
        [GPIO_IN_PIN_USER_BUTTON] = {
                .init = {
                        .Pin   = GPIO_PIN_13,
                        .Mode  = GPIO_MODE_IT_FALLING,
                        .Pull  = GPIO_PULLDOWN,
                        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                },
                .port = GPIOC,
                .pInt = &buttonInterruptData,   /**< Interrupt data for user button */
        },
        [GPIO_ALT_PIN_UART3_TX] = {
                .init = {
                        .Pin   = GPIO_PIN_8,
                        .Mode  = GPIO_MODE_AF_PP,
                        .Pull  = GPIO_PULLUP,
                        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                        .Alternate = GPIO_AF7_USART3,
                },
                .port = GPIOD,
                .pInt = NULL,   /**< No interrupt for alt function */
        },
        [GPIO_ALT_PIN_UART3_RX] = {
                .init = {
                        .Pin   = GPIO_PIN_9,
                        .Mode  = GPIO_MODE_AF_PP,
                        .Pull  = GPIO_PULLUP,
                        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                        .Alternate = GPIO_AF7_USART3,
                },
                .port = GPIOD,
                .pInt = NULL,   /**< No interrupt for alt function */
        },
};

/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/

/* This disables the "discarded-qualifiers" warning. Having a const pointer
 * seems to make this warning appear but the code is being placed into correct
 * sections as can be seen by the *.map file so this just disables the warning
 * only for the following explicit sections */
#pragma GCC diagnostic push    /* Start ignoring -Wdiscard-qualifiers warning */
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

/******************************************************************************/
void GPIO_getData(GpioData_t** pGpioData)
{
    *pGpioData = pins;
}

#pragma GCC diagnostic pop      /* Stop ignoring -Wdiscard-qualifiers warning */

/* Private functions ---------------------------------------------------------*/

