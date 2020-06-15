/**
 * @file    uart.c
 * @brief   UART driver that encapsulates all UART work to one spot
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "uart_data.h"
#include "gpio.h"
#include "stm32h7xx_hal_uart.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

static UartData_t *pUarts = NULL;                   /**< pointer to UART data */

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief   Find device by handle
 *
 */
static Uart_t UART_findDeviceByHandle(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
);

/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
Error_t UART_init(Uart_t port)
{
    Error_t status = ERR_NONE;

    /* First time this is called, retrieve the LED data array from the user
     * LED data that is specific to this board */
    if (NULL == pUarts) {
        UART_getData(&pUarts);
    }

    /* The cast is just for STM HAL driver call since it expects a non-const
     * despite the fact it makes no changes */
    if (HAL_OK != HAL_UART_Init((UART_HandleTypeDef *)&(pUarts[port].pUart->handle))) {
        status = ERR_HW_INIT_FAILURE; goto END;
    }

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/******************************************************************************/
Error_t UART_deInit(Uart_t port)
{
    Error_t status = ERR_NONE;

    /* The cast is just for STM HAL driver call since it expects a non-const
     * despite the fact it makes no changes */
    if (HAL_OK != HAL_UART_DeInit((UART_HandleTypeDef *)&(pUarts[port].pUart->handle))) {
        status = ERR_HW_INIT_FAILURE; goto END;
    }

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/******************************************************************************/
Error_t UART_start(Uart_t port)
{
    /* Enable some extra interrupts that we care about */
    __HAL_UART_ENABLE_IT(&pUarts[port].pUart->handle, UART_IT_IDLE);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(pUarts[port].pDmaTx->irq , pUarts[port].pDmaTx->prio, 0);
    HAL_NVIC_EnableIRQ(pUarts[port].pDmaTx->irq);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(pUarts[port].pDmaRx->irq, pUarts[port].pDmaRx->prio, 0);
    HAL_NVIC_EnableIRQ(pUarts[port].pDmaRx->irq);

    /* NVIC for USART, to catch the TX complete and RX Idle */
    HAL_NVIC_SetPriority(pUarts[port].pUart->irq, pUarts[port].pUart->prio, 0);
    HAL_NVIC_EnableIRQ(pUarts[port].pUart->irq);

    /* There's just no error checking that can be done in this function but
     * we'll return success just so we can keep a consistent interface to the
     * driver */
    return ERR_NONE;
}

/******************************************************************************/
Error_t UART_stop(Uart_t port)
{
    /* Enable some extra interrupts that we care about */
    __HAL_UART_DISABLE_IT(&pUarts[port].pUart->handle, UART_IT_IDLE);

    /* Take down interrupts for DMA RX/TX and UART */
    HAL_NVIC_DisableIRQ(pUarts[port].pDmaTx->irq);
    HAL_NVIC_DisableIRQ(pUarts[port].pDmaRx->irq);
    HAL_NVIC_DisableIRQ(pUarts[port].pUart->irq);

    /* There's just no error checking that can be done in this function but
     * we'll return success just so we can keep a consistent interface to the
     * driver */
    return ERR_NONE;
}

/******************************************************************************/
void UART_regCallbackDataSent(Uart_t port, UartCallback_t callback)
{
    pUarts[port].pUart->callbackDataSent = callback;
}

/******************************************************************************/
void UART_regCallbackDataRcvd(Uart_t port, UartCallback_t callback)
{
    pUarts[port].pUart->callbackDataRcvd = callback;
}

/******************************************************************************/
void UART_clrCallbackDataSent(Uart_t port)
{
    pUarts[port].pUart->callbackDataSent = NULL;
}

/******************************************************************************/
void UART_clrCallbackDataRcvd(Uart_t port)
{
    pUarts[port].pUart->callbackDataRcvd = NULL;
}

/******************************************************************************/
Error_t UART_sendDma(Uart_t port, uint16_t dataLen, uint8_t* const pData)
{
    Error_t status = ERR_NONE;
    if (NULL == pData) {
        status = ERR_MEM_NULL; goto END;
    }

    if (0 == dataLen) {
        status = ERR_LEN_INVALID; goto END;
    }

    /* Normally, this would be a simple if statement instead of a while loop
     * but STM HAL drivers are goofy and they sometimes stay BUSY longer than
     * they should and you just have to try again real quick. Instead of making
     * the caller worry about this, we handle it here. In experiments, a max of
     * 2 loops were required to get this but it could in theory be longer. */
    HAL_StatusTypeDef halStatus = HAL_OK;
    while (HAL_OK != (halStatus = HAL_UART_Transmit_DMA(
            &(pUarts[port].pUart->handle), pData, dataLen))) {
        switch (halStatus) {
            case HAL_TIMEOUT: status = ERR_HW_HAL_TIMEOUT; goto END; break;
            case HAL_ERROR:   status = ERR_HW_HAL_ERROR;   goto END; break;
            default: break;
        }
    }

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/******************************************************************************/
Error_t UART_recvDma(Uart_t port, uint16_t dataLen, uint8_t* const pData)
{
    Error_t status = ERR_NONE;
    if (NULL == pData) {
        status = ERR_MEM_NULL; goto END;
    }

    if (0 == dataLen) {
        status = ERR_LEN_INVALID; goto END;
    }

    /* Normally, this would be a simple if statement instead of a while loop
     * but STM HAL drivers are goofy and they sometimes stay BUSY longer than
     * they should and you just have to try again real quick. Instead of making
     * the caller worry about this, we handle it here. In experiments, a max of
     * 2 loops were required to get this but it could in theory be longer. */
    HAL_StatusTypeDef halStatus = HAL_OK;
    while (HAL_OK != (halStatus = HAL_UART_Transmit_DMA(
            &(pUarts[port].pUart->handle), pData, dataLen))) {
        switch (halStatus) {
            case HAL_TIMEOUT: status = ERR_HW_HAL_TIMEOUT; goto END; break;
            case HAL_ERROR:   status = ERR_HW_HAL_ERROR;   goto END; break;
            default: break;
        }
    }

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/* STM HAL callbacks ---------------------------------------------------------*/

/**
 * @brief UART MSP Initialization
 *
 * This function is a callback to the STM HAL functions that enables the UARTs.
 * When calling UART_init"()", STM HAL code will call this function as a
 * callback. This function then:
 *
 * 1. Calls HAL functions to enable the GPIO pins
 * 2. Calls HAL functions to disable the UART clocks
 * 3.
 * 4. Calls HAL functions to disable the DMA used.
 *
 * @note: This function is weakly defined in the STM HAL driver and by placing a
 * definition here, the compiler override the weak linking with this definition.
 * @return None
 */
void HAL_UART_MspInit(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{
    if (NULL == huart) {
        return;
    }

    Uart_t port = UART_MAX;
    if (UART_MAX == (port = UART_findDeviceByHandle(huart))) {
        return;
    }
    /* The cast is just for STM HAL driver call since it expects a non-const
     * despite the fact it makes no changes */
    GPIO_init(pUarts[port].gpioPinRx);
    GPIO_init(pUarts[port].gpioPinTx);

    /* Initialize the DMA we are going to use for this UART */
    HAL_DMA_Init(&(pUarts[port].pDmaTx->handle));
    /* Associate the initialized DMA handle to the the UART handle. STM HAL macros
     *
     * __HAL_LINKDMA(huart, hdmatx, (DMA_HandleTypeDef *)&(pUarts[port].handleDmaTx));
     * __HAL_LINKDMA(huart, hdmarx, (DMA_HandleTypeDef *)&(pUarts[port].handleDmaRx));
     *
     * for linking DMA to UART won't work here since we did a goofy thing with our
     * DMA handler by making it a pointer in a larger structure so we'll just do it
     * manually */
    huart->hdmatx = (DMA_HandleTypeDef *)&(pUarts[port].pDmaTx->handle);
    pUarts[port].pDmaTx->handle.Parent = huart;

    HAL_DMA_Init((DMA_HandleTypeDef *)&(pUarts[port].pDmaRx->handle));
    huart->hdmarx = (DMA_HandleTypeDef *)&(pUarts[port].pDmaRx->handle);
    pUarts[port].pDmaRx->handle.Parent = huart;

    /* Normally, you would enable interrupts here but we'll do that in UART_start()
     * This is useful if the driver is being used by a system running an RTOS and
     * gives caller a chance to wrap up that initialization if needed. */
}

/**
 * @brief UART MSP De-Initialization
 *
 * This function is a callback to the STM HAL functions that disable the UARTs.
 * When calling UART_deInit"()", STM HAL code will call this function as a
 * callback. This function then:
 *
 * 1. Stops the driver by calling UART_stop"()"
 * 2. Calls HAL functions to disable the UART clocks
 * 3. Calls HAL functions to disable the GPIO pins
 * 4. Calls HAL functions to disable the DMA used.
 *
 * @note: This function is weakly defined in the STM HAL driver and by placing a
 * definition here, the compiler override the weak linking with this definition.
 * @return None
 */
void HAL_UART_MspDeInit(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{
    if (NULL == huart) {
        return;
    }

    Uart_t port = UART_MAX;
    if (UART_MAX == (port = UART_findDeviceByHandle(huart))) {
        return;
    }

    /* Stop the UART before taking down hardware just in case the interrupts are
     * still enabled so we don't end up getting an interrupt while taking down
     * hardware. */
    if (ERR_NONE != UART_stop(port)) {
        return;
    }

    /* Take down the pins */
    GPIO_deinit(pUarts[port].gpioPinTx);
    GPIO_deinit(pUarts[port].gpioPinRx);

    /* Take down the DMA */
    if (NULL != huart->hdmatx) {
        HAL_DMA_DeInit(huart->hdmatx);
    }

    if (NULL != huart->hdmarx) {
        HAL_DMA_DeInit(huart->hdmarx);
    }
}

/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of DMA Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{
    /* Some error checking */
    if (NULL == huart) {
        return;
    }

    Uart_t port = UART_MAX;
    if (UART_MAX == (port = UART_findDeviceByHandle(huart))) {
        return;
    }

    if (NULL != pUarts[port].pUart->callbackDataSent) {
        pUarts[port].pUart->callbackDataSent(port,
                pUarts[port].pBufMgr->bufferTx.pData,
                pUarts[port].pBufMgr->bufferTx.len);
    }
}

/**
 * @brief   RX Transfer completed callback
 * @note    This example shows a simple way to report end of DMA Rx transfer, and
 *          you can add your own implementation.
 * @return  None
 */
void HAL_UART_RxCpltCallback(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{
    /* Some error checking */
    if (NULL == huart) {
        return;
    }

    Uart_t port = UART_MAX;
    if (UART_MAX == (port = UART_findDeviceByHandle(huart))) {
        return;
    }

    /* Get the current length of data recieved by UART RX DMA */
    pUarts[port].pBufMgr->bufferRx.len = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    /* Stop the UART RX DMA transfer for now */
    HAL_DMA_Abort(huart->hdmarx);

    /* Clear out the buffer */
    huart->RxXferCount = 0;

    /* Call user defined callback */
    if (NULL != pUarts[port].pUart->callbackDataRcvd) {
        pUarts[port].pUart->callbackDataRcvd(port,
                pUarts[port].pBufMgr->bufferRx.pData,
                pUarts[port].pBufMgr->bufferRx.len);
    }

    /* Check if a transmit process is ongoing and if it is, leave the TX flag set */
    if(huart->gState == HAL_UART_STATE_BUSY_TX_RX) {
        huart->gState = HAL_UART_STATE_BUSY_TX;
    } else {
        huart->gState = HAL_UART_STATE_READY;
    }
}

/**
 * @brief   UART receive process idle callback for short RX DMA transfers.
 *
 * @note    This function was added as part of a modification to stm32h7xx_hal_uart.c/h
 *          that add handling of the IDLE interrupt. People on forums have been
 *          begging STM to add this functionality since it's the only rational
 *          way to deal with high speed UART RX of variable length data but STM
 *          is gonna STM...
 *          https://community.st.com/s/question/0D50X00009XkhGfSAJ/cubemx-feature-request-add-usart-rx-idle-handling
 *          While it might be possible (or perhaps even better) to use the Receiver
 *          Timeout interrupt, STM HAL drivers treat it as an error so that's not
 *          a great path despite it being more configurable.
 *
 * @return  None
 */
void HAL_UART_RxIdleCallback(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{
    /* Some error checking */
    if (NULL == huart) {
        return;
    }

    Uart_t port = UART_MAX;
    if (UART_MAX == (port = UART_findDeviceByHandle(huart))) {
        return;
    }


    /* Overrun error means our DMA buffer ran out of space before IDLE went off */
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        pUarts[port].pBufMgr->bufferRx.len = huart->RxXferSize;
    } else {
        /* Determine how many items of data have been received */
        pUarts[port].pBufMgr->bufferRx.len = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    }

    HAL_DMA_Abort(huart->hdmarx);

    huart->RxXferCount = 0;
    /* Check if a transmit process is ongoing or not */
    if(huart->gState == HAL_UART_STATE_BUSY_TX_RX) {
        huart->gState = HAL_UART_STATE_BUSY_TX;
    } else {
        huart->gState = HAL_UART_STATE_READY;
    }

#if 0
    if (USART3 == huart->Instance) {

        if (NULL == huart->hdmarx) {
            return;
        }

        /* Overrun error means our DMA buffer ran out of space before IDLE went off */
        if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
            rxBytes = huart->RxXferSize;
        } else {
            /* Determine how many items of data have been received */
            rxBytes = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        }

        HAL_DMA_Abort(huart->hdmarx);

        huart->RxXferCount = 0;
        /* Check if a transmit process is ongoing or not */
        if(huart->gState == HAL_UART_STATE_BUSY_TX_RX) {
            huart->gState = HAL_UART_STATE_BUSY_TX;
        } else {
            huart->gState = HAL_UART_STATE_READY;
        }
#ifdef CACHE_ENABLED
        /* This is not working like it should. Perhaps rxBytes is being caught up in the invalidation?*/
//        SCB_CleanInvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
        SCB_InvalidateDCache_by_Addr ((uint32_t *)rxBuffer, sizeof(rxBuffer));
#endif
        memcpy( rxOutBuffer, rxBuffer, rxBytes);

        uartStatus = SET;
    }
#endif
}

/**
 * @brief   UART error callback
 *
 * @note    This example shows a simple way to report transfer error, and you can
 *          add your own implementation.
 *
 * @return  None
 */
void HAL_UART_ErrorCallback(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{
    /* Set transmission flag: transfer complete */
//    uartStatus = SET;
    /* Turn LED2 off: Transfer in transmission process is correct */
//    BSP_LED_On(LED3);

//    LED_set(LED3, GPIO_PIN_SET);
    HAL_UART_RxIdleCallback(huart);
//    BSP_LED_Off(LED3);huart

//    LED_set(LED3, GPIO_PIN_RESET);

}


/* Private functions ---------------------------------------------------------*/
static Uart_t UART_findDeviceByHandle(UART_HandleTypeDef* huart)
{
    for (Uart_t port = UART_DBG; port < UART_MAX; port++) {
        if (huart->Instance == pUarts[port].pUart->handle.Instance) {
            return port;
        }
    }
    return UART_MAX;
}
