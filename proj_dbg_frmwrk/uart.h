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

#include "uart_data.h"
#include "uarts.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Initialize a given UART
 *
 * @return  Error_t code that specifies success or failure
 * @retval  ERR_NONE: success
 */
Error_t UART_init(
        Uart_t port                    /**< [in] which UART to initialize */
);

/**
 * @brief   Initialize a given UART
 *
 * @return  Error_t code that specifies success or failure
 * @retval  ERR_NONE: success
 */
Error_t UART_deInit(
        Uart_t port                  /**< [in] which UART to deinitialize */
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
 * @retval  ERR_NONE: success
 */
Error_t UART_start(
        Uart_t port                               /**< [in] which system UART */
);

/**
 * @brief   Set a Data Sent callback
 *
 * Set a function to call upon UART completing a data send
 *
 * @return  None
 */
void UART_regCallbackDataSent(
        Uart_t port,                              /**< [in] which system UART */
        UartCallback_t callback
);

/**
 * @brief   Set a Data Received callback
 *
 * Set a function to call upon UART completing a data receive. This will be
 * called either when there is an IDLE detected or when the receive buffer
 * is full.
 *
 * @return  None
 */
void UART_regCallbackDataRcvd(
        Uart_t port,                              /**< [in] which system UART */
        UartCallback_t callback
);

/**
 * @brief   Clear a Data Sent callback
 * @return  None
 */
void UART_clrCallbackDataSent(
        Uart_t port                               /**< [in] which system UART */
);

/**
 * @brief   Clear a Data Received callback
 * @return  None
 */
void UART_clrCallbackDataRcvd(
        Uart_t port                               /**< [in] which system UART */
);

/**
 * @brief   Send data over UART using DMA
 *
 * This function does some basic error checking on inputs and initiates a DMA
 * transfer of passed in data and returns instantly. If the user registered a
 * callback for transfer complete, this driver will call the user callback
 * when DMA transfer is complete.
 *
 * @warning The caller of this function is responsible for maintaining the
 * buffer that was passed in until DMA transfer is complete. No data is
 * copied to any local buffers and the DMA transfer is done directly out of
 * the passed in buffer.
 *
 * @note:   STM HAL drivers have some strange behavior if trying to send
 * immediately after a send was just finished and the next call to their driver
 * returns a BUSY error. This error clears quickly but you need to attempt a
 * transfer again before it clears. The user is spared this by this driver by
 * looping until the BUSY error is cleared. All other HAL errors will result in
 * return of an error.
 *
 * @return  Error_t code that specifies success or failure
 * @retval  ERR_NONE: success
 */
Error_t UART_sendDma(
        Uart_t port,                              /**< [in] which system UART */
        uint16_t dataLen,                   /**< [in] length of data in pData */
        uint8_t* const pData             /**< [in] ptr to data buffer to send */
);

/**
 * @brief   Receive data over UART using DMA
 *
 * This function does some basic error checking on inputs and initiates a DMA
 * receive of data into passed in buffer and returns instantly. The driver will
 * write received data into the passed in buffer until 1 of 3 conditions occur:
 *
 * 1. DMA/UART error is detected.
 * 2. Passed in buffer becomes full.
 * 3. An IDLE is detected on the UART at the configured baud rate.
 *
 * If the caller registered a callback for receive complete, this driver will
 * call the user callback when any of the conditions above occur.
 *
 * @warning The caller of this function is responsible for maintaining the
 * buffer that was passed in until DMA transfer is complete. No data is
 * copied to any local buffers and the DMA transfer is done directly into the
 * passed in buffer.
 *
 * @note:   STM HAL drivers have some strange behavior if trying to send
 * immediately after a send was just finished and the next call to their driver
 * returns a BUSY error. This error clears quickly but you need to attempt a
 * transfer again before it clears. The user is spared this by this driver by
 * looping until the BUSY error is cleared. All other HAL errors will result in
 * return of an error.
 *
 * @return  Error_t code that specifies success or failure
 * @retval  ERR_NONE: success
 */
Error_t UART_recvDma(
        Uart_t port,                              /**< [in] which system UART */
        uint16_t dataLenMax,         /**< [in] max length of the pData buffer */
        uint8_t* const pData             /**< [in,out] buffer to receive data */
);

/**
 * @brief   Redirect ISR to HAL
 *
 * This function intercepts the ISR call that would normally go directly to
 * STM's HAL driver and redirects it there. The reason for doing this is that
 * STM HAL drivers all take the handle as a parameter instead of finding the
 * handle by instance, which in turn necessitates using a shared handle and
 * global variables. Instead, we find the handle by instance and pass that to
 * the HAL drivers.
 *
 * The better way of doing this would be to run LL drivers because STM's HAL
 * implementation is a travesty.
 *
 * @return  None
 */
void UART_redirectIsrToHAL(
        USART_TypeDef* instance
);

void UART_redirectDmaIsrToHAL(
        DMA_Stream_TypeDef* instance
);

#ifdef __cplusplus
}
#endif

#endif                                                            /* __UART_H */

