/**
 * @file    button.h
 * @brief   Button driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "errors.h"
#include "board_defs.h"
#include "buttons.h"
#include "button_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief   Initialize given Button
 *
 * This function initializes all hardware needed to operate a Button
 *
 * @return  None
 */
void BUT_init(
        Button_t button                         /**< [in] which system Button */
);

/**
 * @brief   DeInitialize given Button
 *
 * This function de-initializes all hardware needed to operate an Button
 *
 * @return  None
 */
void BUT_deinit(
        Button_t button                         /**< [in] which system Button */
);

/**
 * @brief   Start given Button driver
 *
 * @return  None
 */
void BUT_start(
        Button_t button                         /**< [in] which system Button */
);

/**
 * @brief   Stop a given Button driver
 *
 * @return  None
 */
void BUT_stop(
        Button_t button                         /**< [in] which system Button */
);

/**
 * @brief   Get given Button's current state
 * @return  None
 */
void BUT_get(
        Button_t led,                           /**< [in] which system Button */
        ButtonState_t* const pState           /**< [out] current Button state */
);

/**
 * @brief   Register a new callback
 *
 * Allows user to add a callback to call when/if button gets pressed
 *
 * @return  None
 */
void BUT_regCallback(
        Button_t button,                        /**< [in] which system Button */
        ButtonCallback_t callback
);

/**
 * @brief   Clear a set callback
 * @return  None
 */
void BUT_clrCallback(
        Button_t button                        /**< [in] which system Button */
);

#ifdef __cplusplus
}
#endif

#endif                                                          /* __BUTTON_H */

