/**
 * @file    led_data.h
 * @brief   LED data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_DATA_H
#define __LED_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "led_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Retrieve LED data for this board
 *
 * This function allows retrieval of LED data for this particular board which
 * allows the driver to be same for different boards
 *
 * @return  None
 */
void LED_getData(
        LedData_t** pLedData  /**< [in,out] pointer to retrieve LED data array */
);

#ifdef __cplusplus
}
#endif

#endif                                                        /* __LED_DATA_H */

