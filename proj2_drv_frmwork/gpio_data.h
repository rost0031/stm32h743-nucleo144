/**
 * @file    gpio_data.h
 * @brief   GPIO data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_DATA_H
#define __GPIO_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gpio_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief   Retrieve GPIO data for this board
 *
 * This function allows retrieval of GPIO data for this particular board which
 * allows the driver to be same for different boards
 *
 * @return  None
 */
void GPIO_getData(
        GpioData_t** pGpioData  /**< [in,out] pointer to retrieve GPIO data array */
);

#ifdef __cplusplus
}
#endif

#endif                                                       /* __GPIO_DATA_H */

