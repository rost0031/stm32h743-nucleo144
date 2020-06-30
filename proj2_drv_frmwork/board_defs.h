/**
 * @file    board_defs.h
 * @brief   Board definitions
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_DEFS_H
#define __BOARD_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief   IRQ priorities.
 */
typedef enum {
   IRQ_PRIO_UART = 0,                                 /**< UART IRQ priority */
   IRQ_PRIO_DMA_UART_TX,                       /**< DMA UART TX IRQ priority */
   IRQ_PRIO_DMA_UART_RX,                       /**< DMA UART RX IRQ priority */
   IRQ_PRIO_BUTTON_EXTI,                       /**< Button EXTI IRQ priority */

   /* Insert priority enumerations here... */
   IRQ_PRIO_MAX         /**< Maximum priority */
} IRQPrio_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                      /* __BOARD_DEFS_H */

