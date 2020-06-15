/**
 * @file    gpio.c
 * @brief   GPIO driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "gpio_data.h"
#include <stddef.h>

#include "stm32h7xx_hal_cortex.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

static GpioData_t *pGpios = NULL;                   /**< pointer to GPIO data */

/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
void GPIO_init(GpioPin_t pin)
{
    /* First time this is called, retrieve the LED data array from the user
     * LED data that is specific to this board */
    if (NULL == pGpios ) {
        GPIO_getData(&pGpios);
    }

    /* Casts are there because HAL functions don't expect const pointers despite
     * the fact they aren't changing any underlying data */
    HAL_GPIO_Init((GPIO_TypeDef *)pGpios[pin].port, (GPIO_InitTypeDef *)&(pGpios[pin].init));
}

/******************************************************************************/
void GPIO_start(GpioPin_t pin)
{
    if (pGpios[pin].pInt) {
        /* Enable and set Button EXTI Interrupt to the lowest priority */
        HAL_NVIC_SetPriority(pGpios[pin].pInt->irq, pGpios[pin].pInt->prio, 0x00);
        HAL_NVIC_EnableIRQ(pGpios[pin].pInt->irq);
    }
}

/******************************************************************************/
void GPIO_regCallback(GpioPin_t pin, GPIOCallback_t callback)
{
    pGpios[pin].pInt->callback = callback;
}

/******************************************************************************/
void GPIO_clrCallback(GpioPin_t pin)
{
    pGpios[pin].pInt->callback = NULL;
}

/******************************************************************************/
void GPIO_set(GpioPin_t pin, GpioPinState_t state)
{
    HAL_GPIO_WritePin((GPIO_TypeDef *)pGpios[pin].port, pGpios[pin].init.Pin, state);
}

/******************************************************************************/
void GPIO_get(GpioPin_t pin, GpioPinState_t* const pState)
{
    *pState = HAL_GPIO_ReadPin((GPIO_TypeDef *)pGpios[pin].port, pGpios[pin].init.Pin);
}

/******************************************************************************/
void GPIO_toggle(GpioPin_t pin)
{
    HAL_GPIO_TogglePin((GPIO_TypeDef *)pGpios[pin].port, pGpios[pin].init.Pin);
}

/******************************************************************************/
void GPIO_deinit(GpioPin_t pin)
{
    GPIO_stop(pin);
    HAL_GPIO_DeInit((GPIO_TypeDef *)pGpios[pin].port, pGpios[pin].init.Pin);
}

/******************************************************************************/
void GPIO_stop(GpioPin_t pin)
{
    if (pGpios[pin].pInt) {
        HAL_NVIC_DisableIRQ(pGpios[pin].pInt->irq);
    }
}

/******************** Direct calls from ISR to our driver *********************/

/**
 * @brief   EXTI line detection callback
 *
 * This function is weakly defined in the STM HAL so by providing an implementation
 * here, the compiler will automatically link it to that call. This function will
 * get called by the HAL driver once it has finished handling the hardware aspects
 * of an EXTI interrupt.
 *
 * @return  None
 */
void HAL_GPIO_EXTI_Callback(
        uint16_t gpioPin    /**< [in] which STM HAL gpio pin number this is */
)
{
    /* Start the loop from the first input pin since there's no reason to check
     * output pins for interrupts */
    for (GpioPin_t pin = GPIO_IN_PIN_USER_BUTTON; pin < GPIO_PIN_MAX; pin++) {
        if (pGpios[pin].init.Pin == gpioPin) {
            /* Found the first matching pin number, check for interrupt data */
            if (NULL != pGpios[pin].pInt) {
                if (NULL != pGpios[pin].pInt->callback) {
                    /* Callback exists. Get the current pin state and call it.
                     * We'll call the HAL directly instead of our function to
                     * avoid an extra layer of redirection while in the ISR */
                    pGpios[pin].pInt->callback(pin,
                            HAL_GPIO_ReadPin((GPIO_TypeDef *)pGpios[pin].port,
                                    pGpios[pin].init.Pin));
                    /* We can't have multiple pins on different ports get
                     * interrupts so it's safe to return once the callback has
                     * completed */
                    return;
                }
            }
        }
    }
}


/* Private functions ---------------------------------------------------------*/

