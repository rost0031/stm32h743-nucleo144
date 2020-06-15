/**
 * @file    button_data.h
 * @brief   Button data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_DATA_H
#define __BUTTON_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "button_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Retrieve button data for this board
 *
 * This function allows retrieval of button data for this particular board which
 * allows the driver to be same for different boards
 *
 * @return  None
 */
void BUT_getData(
        ButtonData_t** pButtonData  /**< [in,out] pointer to retrieve button data array */
);

#ifdef __cplusplus
}
#endif

#endif                                                     /* __BUTTON_DATA_H */

