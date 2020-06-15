/**
 * @file    buffers.h
 * @brief   Buffer types to use that are safer than passing around simple pointers
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUFFERS_H
#define __BUFFERS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief   Buffer structure
 */
typedef struct {
    uint16_t maxLen;                        /**< Max length of the buffer */
    uint16_t len;                           /**< Length of data in the buffer */
    uint8_t* pData;                         /**< Pointer to the actual data */
} Buffer_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif                                                         /* __BUFFERS_H */

