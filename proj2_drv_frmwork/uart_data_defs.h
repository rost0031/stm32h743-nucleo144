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
 * @brief   UART events
 *
 * This set of enums is a list of possible events for this driver.
 * Its primary purpose to allow user to assign callbacks for these events
 *
 * This list started by listing all interrupts on UART which is a long list.
 * Really, the user doesn't need to be aware of all of these error and action
 * interrupts. Instead, the user only cares if:
 *
 * 1. Data sent successfully:
 * tells caller driver is available to send more data
 *
 * 2. Data received successfully:
 * tells caller driver has data available and that the caller needs to
 * handle it and initiate a new receive with a new buffer for storage.
 *
 * 3. Error during receive:
 * tells caller to retry or somehow handle this information
 *
 * 4. Error during send:
 * tells caller to resend or somehow handle this information.
 */
typedef enum {
    UartEvtStart = 0,                                /**< For bounds checking */

    UartEvtDataRcvd = UartEvtStart,                       /**< Data received */
    UartEvtDataSent,                             /**< Data finished sending  */
    UartEvtDataRecvErr,                     /**< Error during receiving data */
    UartEvtDataSendErr,                       /**< Error during sending data */

#if 0

    UartIntErrParity,                                       /**< parity error */
    UartIntIDLE,                                   /**< RX line idle detected */
    UartIntErrFraming,                                     /**< framing error */
    UartIntErrNoise,                                         /**< noise error */
    UartIntErrOverrun,                                     /**< overrun error */
    UartIntTXE,                                   /**< TX data register empty */
    UartIntTXFNF,                                       /**< TX FIFO not full */
    UartIntTXTF,                               /**< TX FIFO threshold reached */
    UartIntTC,                                         /**< Transfer Complete */
    UartIntRXNE,                              /**< RX data register not empty */
    UartIntRXFNE,                                      /**< RX FIFO not empty */
    UartIntRXFF,                                            /**< RX FIFO full */
    UartIntRXTF,                               /**< RX FIFO threshold reached */
    UartIntCMF,                                 /**< Character match detected */
    UartIntRTOF,                                              /**< RX timeout */

    /* Not commonly used - START */
    UartIntErrLBDF,                            /**< Line Break detected error */
    UartIntCTS,                                            /**< CTS interrupt */
    UartIntEOB,                                /**< End Of Block (smart card) */
    UartIntWUF,                               /**< Wakeup from low-power mode */
    UartIntUDR,                                 /**< SPI slave underrun error */
    UartIntTCBGT,                    /**< Transfer Complete before guard time */
    /* Not commonly used - END */
#endif

    UartEvtEnd                                       /**< For bounds checking */
} UartInterrupt_t;


/**
 * @brief   UART callback function pointer type
 */
typedef void (*UartCallback_t)(
        Uart_t,                                   /**< [in] Which system UART */
        Error_t,                                /**< [in] status of operation */
        Buffer_t*                     /**< [in] pointer to buffer information */
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
    UartCallback_t callbacks[UartEvtEnd];             /**< Array of callbacks */
    Buffer_t    bufferTx;                          /**< TX Buffer information */
    Buffer_t    bufferRx;                          /**< RX Buffer information */
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

