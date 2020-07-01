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
#include "dma.h"
#include "buffers.h"
#include "board_defs.h"
#include "gpio_pins.h"

#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_ll_usart.h"

/* Exported types ------------------------------------------------------------*/


/**
 * @brief   UART interrupts
 *
 * This set of enums is a complete list of possible interrupts for this driver.
 * Its primary purpose to allow user to control which interrupts should be
 * listened and acted upon. They also allow for registration of callbacks which
 * is how the driver will know which interrupts to enable
 */
typedef enum {
    UartIntStart = 0,                               /**< For bounds checking */

    UartIntParityError = UartIntStart,       /**< UART parity error interrupt */
    UartIntFramingError,                    /**< UART framing error interrupt */
    UartIntNoiseError,                        /**< UART noise error interrupt */
    UartIntOverrunError,                    /**< UART overrun error interrupt */
    UartIntIdle,                                     /**< UART IDLE interrupt */
    UartIntRxneRxfne,/**< UART read data register or fifo not empty interrupt */
    UartIntTransferComplete,            /**< UART Transfer Complete interrupt */
    UartIntTxeTxfnf,     /**< UART TX reg empty or TX FIFO not full interrupt */
    UartIntLineBreak,                          /**< UART Line Break interrupt */
    UartIntNCTS,                                     /**< UART NCTS interrupt */
    UartIntCTS,                                       /**< UART CTS interrupt */
    UartIntReceiverTimeout,             /**< UART Receiver Time Out interrupt */
    UartIntEndOfBlock,          /**< UART End Of Block interrupt (smart card) */
    UartIntUnderrunError,                  /**< UART Underrun Error interrupt */

    UartIntEnd                                       /**< For bounds checking */
} UartInterrupt_t;


/**
 * @brief   UART callback function pointer type
 */
typedef void (*UartCallback_t)(
        Uart_t,                                   /**< [in] Which system UART */
        const uint8_t* const,                       /**< [in] pointer to data */
        uint16_t                           /**< [in] length of data in buffer */
);

/**
 * @brief   DMA structure for use with UART
 * The only reason this is a separate structure is to allow quick checking for
 * whether a pointer to this is NULL so the driver knows whether to do anything
 * with DMA or not. This structure can still be const and live in flash.
 */
typedef struct {
    const Dma_t         channel;                              /**< DMA channel*/
} UartDmaData_t;

/**
 * @brief   UART device data
 *
 * This structure type allows specification of all data needed to control and
 * configure UART.
 */
typedef struct {
    USART_TypeDef*       const base;            /**< UART memory base address */
    LL_USART_InitTypeDef       init;            /**< UART initialization data */
    bool                   isTxBusy;       /**< Is the UART TX currently busy */
    bool                   isRxBusy;       /**< Is the UART RX currently busy */
    UartCallback_t callbackDataSent;       /**< Callback to call on data sent */
    UartCallback_t callbackDataRcvd;       /**< Callback to call on data rcvd */
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
    const GpioPin_t   gpioPinTx;             /**< System GPIO pin for UART TX */
    const GpioPin_t   gpioPinRx;             /**< System GPIO pin for UART RX */

    /* General UART settings */
    UartDevData_t* const pUart;                         /**< UART device data */
    const uint32_t    clk; /**< BUS_LL_EC_APB1_GRP1_PERIPH group clock select */
    const IRQn_Type   irq;                              /**< Interrupt number */
    const IRQPrio_t   prio;                           /**< Interrupt priority */

    /* DMA */
    const UartDmaData_t* const pDmaTx;            /**< UART DMA TX device data */
    const UartDmaData_t* const pDmaRx;            /**< UART DMA RX device data */

} UartData_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                  /* __UART_DATA_DEFS_H */

