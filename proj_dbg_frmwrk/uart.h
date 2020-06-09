/**
 * @file    uart.h
 * @brief   UART driver that encapsulates all UART work to one spot
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "errors.h"
#include "buffers.h"
#include "board_defs.h"

#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
/* Exported types ------------------------------------------------------------*/

/**
 * @brief   UART ports available on the system.
 *
 * This type allows abstraction from UART instances, handles, etc. and lets the
 * user deal with just the UART by its functionality using this name. It is the
 * primary interface to the UART driver.
 * Using enumerations allows the compiler to do most of the checks at compile
 * time instead of having to check at runtime
 */
typedef enum {
   UART_DBG = 0,   /**< UART port for debug output/input */

   /* Insert more serial port enumerations here... */
   UART_MAX         /**< Maximum number of available UART ports on the system */
} UartPort_t;


/**
 * @brief   UART buffer management
 */
typedef struct {
    Buffer_t txBuffer;  /**< Buffer for TX */
    Buffer_t rxBuffer;  /**< Buffer for RX */
} UartBuf_t;

typedef struct {
    DMA_HandleTypeDef* const handle;
    const IRQn_Type         irqNumber;             /**< DMA TX IRQ number */
    const IRQPrio_t         irqPriority;         /**< DMA TX IRQ priority */
} DmaDev_t;

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
    /* General UART settings */
    UART_HandleTypeDef      handleUart;           /**< Configurable UART data */
    const IRQn_Type         irqUart;                   /**< IRQ for this UART */
    const IRQPrio_t         irqPrioUart;                    /**< IRQ priority */

    /* UART GPIO pin configurations */
    const GPIO_InitTypeDef  gpioTx;                      /**< GPIO pin for TX */
    GPIO_TypeDef*           gpioTxPort;                 /**< GPIO port for TX */
    const GPIO_InitTypeDef  gpioRx;                      /**< GPIO pin for RX */
    GPIO_TypeDef*           gpioRxPort;                 /**< GPIO port for RX */

    /* DMA */
    DmaDev_t                dmaTx;
    DmaDev_t                dmaRx;
    /* Dynamic data */
    UartBuf_t* const      pBufMgr;                /**< Uart buffer management */

} UartDev_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Initialize a given UART
 *
 * @return  Error_t code that specifies success or failure
 */
Error_t UART_init(
        UartPort_t port                    /**< [in] which UART to initialize */
);

/**
 * @brief   Start a given UART
 *
 * This function enables the interrupts that allow the device to function after
 * initialization. This is to allow RTOSes to set up their systems and buffers
 * and gives user some control over the HW to prevent interrupts before the
 * system is ready to correctly handle them.
 *
 * @return  Error_t code that specifies success or failure
 */
Error_t UART_start(
        UartPort_t port                         /**< [in] which UART to start */
);

#ifdef __cplusplus
}
#endif

#endif                                                            /* __UART_H */

