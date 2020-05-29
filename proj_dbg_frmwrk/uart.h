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
#include "errors.h"
#include "buffers.h"
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

/**
 * @brief   User configuration structure for the UART driver.
 *
 * This structure should hold any data that can be changed during runtime as
 * opposed to the constant data such as clock and pin configurations
 */
typedef struct {
    uint32_t                baudRate;      /**< UART baud rate */
    lpuart_parity_mode_t    parityMode;    /**< parity mode, disabled, even, odd */
    lpuart_stop_bit_count_t stopBits;      /**< number of stop bits */
    lpuart_data_bits_t      dataBitsCount; /**< number of data bits */
    MsgIf_t                 msgIf;         /**< Expected message types */
} UartUserConfig_t;

/**
 * @brief   UART attributes
 *
 * This is a constant top-level structure that contains data that pertains to
 * setting up and running a UART driver for a given port. Because this structure
 * is constant, it should only contain data that can't change during runtime
 * such as clock, base port, pins, etc. For dynamic data, a pointer in this
 * structure should be used to point to dynamic structures that are allowed to
 * have dynamic data such as speed, buffer data, message interface, etc.
 */
typedef struct UartAttributes {
    /* General UART settings */
    UART_HandleTypeDef      handle;           /**< Uart Memory map pointer */
    const clock_ip_name_t    clk;            /**< Clock enumeration */
    const IRQn_Type          irq;            /**< IRQ for this UART */

    /* UART GPIO pin configurations */
    const uint32_t           txPin[GPIO_PIN_DEF_LEN];        /**< UART TX pin */
    const uint32_t           rxPin[GPIO_PIN_DEF_LEN];        /**< UART RX pin */

    /* DMA */
    DmaChnlConf_t* const     pTxDma;         /**< DMA information for TX */
    DmaChnlConf_t* const     pRxDma;         /**< DMA information for RX */

    /* Dynamic data */
    UartBufMgr_t* const      pBufMgr;        /**< Uart buffer management */
    UartUserConfig_t* const  pUserConfig;    /**< User configurable uart settings */

} UartAttr_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Initialize a given UART
 *
 * @return  Error_t code that specifies success or failure
 */
Error_t UART_init(
        UartPort_t uart     /**< [in] which UART to initialize */
);

#ifdef __cplusplus
}
#endif

#endif                                                            /* __UART_H */

