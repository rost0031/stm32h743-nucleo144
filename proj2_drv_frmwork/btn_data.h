/**
 * @file    btn_data.h
 * @brief   Button data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BTN_DATA_H
#define __BTN_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "btn_data_defs.h"

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
void BTN_getData(
        ButtonData_t** pButtonData  /**< [in,out] ptr to retrieve button data */
);

#ifdef __cplusplus
}
#endif

#endif                                                        /* __BTN_DATA_H */

