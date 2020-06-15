/**
 * @file    uart_data_defs.h
 * @brief   UART data definitions specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_DATA_DEFS_H
#define __UART_DATA_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "uarts.h"
#include "buffers.h"
#include "board_defs.h"
#include "gpio_pins.h"

#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief   UART callback function pointer type
 */
typedef void (*UartCallback_t)(
        Uart_t,                                   /**< [in] Which system UART */
        const uint8_t* const,                       /**< [in] pointer to data */
        uint16_t                           /**< [in] length of data in buffer */
);

/**
 * @brief   UART buffer management
 */
typedef struct {
    Buffer_t bufferTx;                                     /**< Buffer for TX */
    Buffer_t bufferRx;                                     /**< Buffer for RX */
} UartBuf_t;

typedef struct {
    DMA_HandleTypeDef   handle;                               /**< DMA handle */
    const IRQn_Type     irq;                              /**< DMA IRQ number */
    const IRQPrio_t     prio;                           /**< DMA IRQ priority */
} DmaDevData_t;

/**
 * @brief   UART interrupt/callback data
 *
 * This structure type allows specification of all data needed to control and
 * configure UART interrupts.
 */
typedef struct {
    UART_HandleTypeDef  handle;                   /**< Configurable UART data */
    const IRQn_Type     irq;                            /**< Interrupt number */
    const IRQPrio_t     prio;                         /**< Interrupt priority */
    UartCallback_t      callbackDataSent;  /**< Callback to call on data sent */
    UartCallback_t      callbackDataRcvd;  /**< Callback to call on data rcvd */
} UartDevData_t;

/**
 * @brief   UART Device Structure
 *
 * This is a constant top-level structure that contains data that pertains to
 * setting up and running a UART driver for a given port. Because this structure
 * is constant, it should only contain data that can't change during runtime
 * such as clock, base port, pins, etc. For dynamic data, a pointer in this
 * structure should be used to point to dynamic structures that are allowed to
 * have dynamic data such as speed, buffer data, message interface, etc.
 */
typedef struct {
    /* UART GPIO pin configurations */
    const GpioPin_t         gpioPinTx;       /**< System GPIO pin for UART TX */
    const GpioPin_t         gpioPinRx;       /**< System GPIO pin for UART RX */

    /* General UART settings */
    UartDevData_t* const    pUart;                      /**< UART device data */

    /* DMA */
    DmaDevData_t*  const    pDmaTx;              /**< UART DMA TX device data */
    DmaDevData_t*  const    pDmaRx;              /**< UART DMA RX device data */

    /* Dynamic data */
    UartBuf_t*     const    pBufMgr;              /**< UART buffer management */
} UartData_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                  /* __UART_DATA_DEFS_H */

