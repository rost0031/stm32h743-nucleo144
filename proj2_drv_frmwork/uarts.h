/**
 * @file    uarts.h
 * @brief   UARTs used by this project
 *
 * Having the UART definitions be in a separate header file allows the same
 * project use same button definitions while allowing it to be compiled for
 * several different targets (windows simulations, real hardware, devkit, etc)
 * without having to copy and paste same UART definitions.
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UARTS_H
#define __UARTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
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
    UART_START = 0,
    UART_DBG = UART_START,   /**< UART port for debug output/input */

    /* Insert more serial port enumerations here... */
    UART_MAX         /**< Maximum number of available UART ports on the system */
} Uart_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                           /* __UARTS_H */

