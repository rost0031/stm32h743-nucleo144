/**
 * @file    led.c
 * @brief   LED driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "led_data.h"
#include <stddef.h>
#include "gpio.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

static LedData_t *pLeds = NULL;                      /**< pointer to LED data */

/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
void LED_init(LED_t led)
{

    /* First time this is called, retrieve the LED data array from the user
     * LED data that is specific to this board */
    if (NULL == pLeds) {
        LED_getData(&pLeds);
    }

    GPIO_init(pLeds[led].systemGpioPin);
    GPIO_set(pLeds[led].systemGpioPin, GPIO_PIN_RESET);
}

/******************************************************************************/
void LED_deinit(LED_t led)
{
    if (NULL == pLeds) {
        LED_getData(&pLeds);
    }
    GPIO_deinit(pLeds[led].systemGpioPin);
}

/******************************************************************************/
void LED_on(LED_t led)
{
    GPIO_set(pLeds[led].systemGpioPin, GPIO_PIN_SET);
}

/******************************************************************************/
void LED_off(LED_t led)
{
    GPIO_set(pLeds[led].systemGpioPin, GPIO_PIN_RESET);
}

/******************************************************************************/
void LED_set(LED_t led, LEDState_t state)
{
    GPIO_set(pLeds[led].systemGpioPin, state);
}

/******************************************************************************/
void LED_get(LED_t led, LEDState_t* const pState)
{
    GPIO_get(pLeds[led].systemGpioPin, pState);
}

/******************************************************************************/
void LED_toggle(LED_t led)
{
    GPIO_toggle(pLeds[led].systemGpioPin);
}

/* Private functions ---------------------------------------------------------*/

