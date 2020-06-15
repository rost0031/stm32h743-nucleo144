/**
 * @file    button_data.c
 * @brief   Button data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "button_data.h"
#include "button_data_defs.h"
#include "buttons.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
/**
 * @brief   Interrupt data for the User button
 */
ButtonIrqData_t buttonInterruptData[BUTTON_MAX] = {
        [BUTTON_USER1] = {
                .callback = NULL,
        }
};

/**
 * @brief   Array of pin data
 *
 * This array contains data about how to initialize the various LEDs used by the
 * system.
 *
 * @note: the array is const so it will be placed into flash to save RAM.
 */
const ButtonData_t buttons[BUTTON_MAX] = {
        [BUTTON_USER1] = {
                .systemGpioPin = GPIO_IN_PIN_USER_BUTTON,
                .pInt = &buttonInterruptData[BUTTON_USER1],
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
void BUT_getData(ButtonData_t** pButtonData)
{
    *pButtonData = buttons;
}

#pragma GCC diagnostic pop      /* Stop ignoring -Wdiscard-qualifiers warning */

/* Private functions ---------------------------------------------------------*/

