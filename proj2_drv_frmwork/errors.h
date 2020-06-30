/**
 * @file    errors.h
 * @brief   Error codes used throughout the system
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERRORS_H
#define __ERRORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief   Error codes and statuses
 *
 * These are the various error codes and status that can occur in the system
 */
typedef enum {
    ERR_NONE                = 0x00, /**< No error, success */
    ERR_MEM_NULL            = 0x01, /**< NULL memory or buffer */
    ERR_LEN_INVALID         = 0x02, /**< Invalid length of something */
    ERR_ARG_INVALID         = 0x03, /**< Invalid argument passed in */
    ERR_HW_INIT_FAILURE     = 0x04, /**< HW initialization failure */
    ERR_HW_BUSY             = 0x05, /**< HW busy */
    ERR_HW_TIMEOUT          = 0x06, /**< HW timeout */
    ERR_HW_CONFIG           = 0x07, /**< HW not or misconfigured */

    ERR_UNIMPLEMENTED       = 0xFE, /**< Unimplemented feature */
    ERR_UNKNOWN             = 0xFF  /**< Max Error code*/
} Error_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif                                                          /* __ERRORS_H */

