/**
 * @file    led_data.c
 * @brief   LED data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "led_data.h"
#include "led_data_defs.h"
#include "leds.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

/**
 * @brief   Array of pin data
 *
 * This array contains data about how to initialize the various LEDs used by the
 * system.
 *
 * @note: the array is const so it will be placed into flash to save RAM.
 */
const LedData_t leds[LED_MAX] = {
        [LED1] = {
                .systemGpioPin = GPIO_OUT_PIN_LED1,
        },
        [LED2] = {
                .systemGpioPin = GPIO_OUT_PIN_LED2,
        },
        [LED3] = {
                .systemGpioPin = GPIO_OUT_PIN_LED3,
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
void LED_getData(LedData_t** pLedData)
{
    *pLedData = leds;
}

#pragma GCC diagnostic pop      /* Stop ignoring -Wdiscard-qualifiers warning */

/* Private functions ---------------------------------------------------------*/

