/**
 * @file    bps.h
 * @brief   Board Support Package
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "errors.h"
#include "buffers.h"
#include "board_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Initialize the board support package
 *
 * This function initializes all hardware in the system
 *
 * @return  Error_t code that specifies success or failure
 */
Error_t BSP_init(void);

#ifdef __cplusplus
}
#endif

#endif                                                             /* __BSP_H */

