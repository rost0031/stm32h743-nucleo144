/**
 * @file    button.c
 * @brief   Button driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "button.h"
#include "button_data.h"
#include <stddef.h>
#include "gpio.h"
#include "gpio_data_defs.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

static ButtonData_t *pButtons = NULL;             /**< pointer to Button data */

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief   A local button callback
 *
 * This is a local private callback function to pass to GPIO driver so the
 * button driver is notified when a button is pressed. It can then call a
 * user set button callback if set.
 *
 * @return  None
 */
static void BUT_gpioCallback(
        GpioPin_t pin,                               /**< [in] which GPIO pin */
        GpioPinState_t state                     /**< [in] current GPIO state */
);

/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
void BUT_init(Button_t button)
{
    /* First time this is called, retrieve the LED data array from the user
     * LED data that is specific to this board */
    if (NULL == pButtons) {
        BUT_getData(&pButtons);
    }

    GPIO_init(pButtons[button].systemGpioPin);
    GPIO_regCallback(pButtons[button].systemGpioPin, BUT_gpioCallback);
}

/******************************************************************************/
void BUT_deinit(Button_t button)
{
    if (NULL == pButtons) {
        BUT_getData(&pButtons);
    }
    GPIO_deinit(pButtons[button].systemGpioPin);
}

/******************************************************************************/
void BUT_start(Button_t button)
{
    GPIO_start(pButtons[button].systemGpioPin);
}

/******************************************************************************/
void BUT_stop(Button_t button)
{
    GPIO_stop(pButtons[button].systemGpioPin);
}


/******************************************************************************/
void BUT_get(Button_t button, ButtonState_t* const pState)
{
    GPIO_get(pButtons[button].systemGpioPin, pState);
}

/******************************************************************************/
void BUT_regCallback(Button_t button, ButtonCallback_t callback)
{
    pButtons[button].pInt->callback = callback;
}

/******************************************************************************/
void BUT_clrCallback(Button_t button)
{
    pButtons[button].pInt->callback = NULL;
}

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
static void BUT_gpioCallback(GpioPin_t pin, GpioPinState_t state)
{
    for (Button_t button = BUTTON_USER1; button < BUTTON_MAX; button++) {
        if (NULL != pButtons[button].pInt->callback) {
            pButtons[button].pInt->callback(button, state);
        }
    }
}

