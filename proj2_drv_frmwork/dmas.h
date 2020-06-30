/**
 * @file    dmas.h
 * @brief   DMAs used by this project
 *
 * Having the DMA definitions be in a separate header file allows the same
 * project use same button definitions while allowing it to be compiled for
 * several different targets (windows simulations, real hardware, devkit, etc)
 * without having to copy and paste same definitions.
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMAS_H
#define __DMAS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief   DMAs available in the system
 *
 * These enumerations allow user to specify a system DMA to use
 */
typedef enum {
    DMA_START     = 0,                                     /**< Start of DMAs */
    DMA_UART3_TX  = DMA_START,                          /**< DMA for UART3 TX */
    DMA_UART3_RX,                                       /**< DMA for UART3 RX */

    DMA_END,
    DMA_MAX       = DMA_END                                      /**< DMA max */
} Dma_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                            /* __DMAS_H */

