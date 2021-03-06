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
#include "dma.h"
#include "gpio.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_usart.h"

#include <stddef.h>

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

static UartData_t *pUarts = NULL;                   /**< pointer to UART data */

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief   Get UART port from DMA channel.
 * @return  Uart_t port which has the passed in channel
 */
static Uart_t UART_getPortFromDmaChannel(
        Dma_t channel
);

/**
 * @brief   Callback for DMA to call when finished.
 *
 * @warning: This function runs in the ISR context so be concise and speedy
 *
 * @return  None
 */
static void UART_callbackDmaTxDone(
        Dma_t channel,
        Error_t status,
        Buffer_t *pBuffer
);

/**
 * @brief   Callback for DMA to call when finished.
 *
 * @warning: This function runs in the ISR context so be concise and speedy
 *
 * @return  None
 */
static void UART_callbackDmaRxDone(
        Dma_t channel,
        Error_t status,
        Buffer_t *pBuffer
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

    /* Initialize GPIO pins use by the UART */
    GPIO_init(pUarts[port].gpioPinTx);
    GPIO_init(pUarts[port].gpioPinRx);

    /* All UARTs use the same clock bus for this chip, thankfully */
    LL_APB1_GRP1_EnableClock(pUarts[port].clk);

    LL_USART_SetTransferDirection(pUarts[port].pUart->base,
            pUarts[port].pUart->init.TransferDirection);

    LL_USART_ConfigCharacter(pUarts[port].pUart->base,
            pUarts[port].pUart->init.DataWidth,
            pUarts[port].pUart->init.Parity,
            pUarts[port].pUart->init.StopBits);

    /* SystemCoreClock is a global var from STM32 libraries. The division by 4
     * is needed to make most baudrates work
     * TODO: there should be a better way to get the clock and divisor (/4) than
     * hard coding. Tackle this later */
    LL_USART_SetBaudRate(pUarts[port].pUart->base,
            SystemCoreClock/4,
            pUarts[port].pUart->init.PrescalerValue,
            pUarts[port].pUart->init.OverSampling,
            pUarts[port].pUart->init.BaudRate);

    LL_USART_SetHWFlowCtrl(pUarts[port].pUart->base,
            pUarts[port].pUart->init.HardwareFlowControl);

//    LL_USART_SetTXFIFOThreshold(pUarts[port].pUart->base, LL_USART_FIFOTHRESHOLD_7_8);
//    LL_USART_SetRXFIFOThreshold(pUarts[port].pUart->base, LL_USART_FIFOTHRESHOLD_7_8);
//    LL_USART_EnableFIFO(pUarts[port].pUart->base);
//    LL_USART_ConfigAsyncMode(pUarts[port].pUart->base);

    /* If DMA has been configured, initialize it. */
    if (NULL != pUarts[port].pDmaTx) {
        DMA_init(pUarts[port].pDmaTx->channel);
        LL_USART_EnableDMAReq_TX(pUarts[port].pUart->base);
    }
    if (NULL != pUarts[port].pDmaRx) {
        DMA_init(pUarts[port].pDmaRx->channel);
        LL_USART_EnableDMAReq_RX(pUarts[port].pUart->base);
    }

//    LL_USART_EnableIT_IDLE(pUarts[port].pUart->base);/* Enable IDLE interrupt */

    LL_USART_Enable( pUarts[port].pUart->base );               /* Enable UART */

    return status;
}

/******************************************************************************/
Error_t UART_deInit(Uart_t port)
{
    Error_t status = ERR_NONE;

    if (NULL != pUarts[port].pDmaTx) {
        DMA_deinit( pUarts[port].pDmaTx->channel);
        LL_USART_DisableDMAReq_TX(pUarts[port].pUart->base);
    }

    if (NULL != pUarts[port].pDmaRx) {
        DMA_deinit( pUarts[port].pDmaRx->channel);
        LL_USART_DisableDMAReq_RX(pUarts[port].pUart->base);
    }

    LL_USART_DeInit(pUarts[port].pUart->base);

    return status;
}

/******************************************************************************/
Error_t UART_start(Uart_t port)
{
    /* TODO: this driver currently only enables IDLE interrupt because it's set
     * up to run with DMA and IDLE detection for RX. If there is ever a desire
     * or need to make this driver more general, there should be a better way to
     * handle which interrupts we care about and which we don't based on some
     * small set of factors such as either configuration or which UART_sendXXX
     * functions user calls. Problem for another day. */

    /* Enable the interrupts we care about */
//    LL_USART_EnableIT_IDLE(pUarts[port].pUart->base);

    /* Enable error interrupts */
    LL_USART_EnableIT_ERROR(pUarts[port].pUart->base);
    LL_USART_EnableIT_PE(pUarts[port].pUart->base);

    if (NULL != pUarts[port].pDmaTx) {
        DMA_start(pUarts[port].pDmaTx->channel);
    }

    if (NULL != pUarts[port].pDmaRx) {
        DMA_start(pUarts[port].pDmaRx->channel);
    }

    /* NVIC for USART, to catch the TX complete and RX Idle */
    NVIC_SetPriority(pUarts[port].irq, pUarts[port].prio);
    NVIC_EnableIRQ(pUarts[port].irq);

    pUarts[port].pUart->isTxBusy = false;
    pUarts[port].pUart->isRxBusy = false;

    /* There's just no error checking that can be done in this function but
     * we'll return success just so we can keep a consistent interface to the
     * driver */
    return ERR_NONE;
}

/******************************************************************************/
Error_t UART_stop(Uart_t port)
{
    LL_USART_DisableIT_IDLE(pUarts[port].pUart->base);

    /* Enable error interrupts */
    LL_USART_DisableIT_ERROR(pUarts[port].pUart->base);
    LL_USART_DisableIT_PE(pUarts[port].pUart->base);

    /* Stop DMA driver if configured */
    if (NULL != pUarts[port].pDmaTx) {
        DMA_stop(pUarts[port].pDmaTx->channel);
    }

    if (NULL != pUarts[port].pDmaRx) {
        DMA_stop(pUarts[port].pDmaRx->channel);
    }

    NVIC_DisableIRQ(pUarts[port].irq);

    pUarts[port].pUart->isTxBusy = true;
    pUarts[port].pUart->isRxBusy = true;

    /* There's just no error checking that can be done in this function but
     * we'll return success just so we can keep a consistent interface to the
     * driver */
    return ERR_NONE;
}

/******************************************************************************/
void UART_regCallback(Uart_t port, UartInterrupt_t uartInt, UartCallback_t callback)
{
    pUarts[port].pUart->callbacks[uartInt] = callback;
}

/******************************************************************************/
void UART_clrCallback(Uart_t port, UartInterrupt_t uartInt)
{
    pUarts[port].pUart->callbacks[uartInt] = NULL;
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

    if (true == pUarts[port].pUart->isTxBusy) {
        status = ERR_HW_BUSY; goto END;
    }

    if (NULL == pUarts[port].pDmaTx) {
        status = ERR_HW_CONFIG; goto END;
    }

    /* Save buffer information */
    pUarts[port].pUart->bufferTx.pData  = pData;
    pUarts[port].pUart->bufferTx.maxLen = dataLen;
    pUarts[port].pUart->bufferTx.len    = 0;

    pUarts[port].pUart->isTxBusy = true;

    /* Register a private UART callback with DMA channel to notify UART driver
     * when DMA completes the transfer */
    DMA_registerCallback(pUarts[port].pDmaTx->channel,
            DmaTransferCompleteInt, UART_callbackDmaTxDone);

    if (ERR_NONE != (status = DMA_startTransfer(pUarts[port].pDmaTx->channel, pData, dataLen))) {
        goto END;
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

    if (true == pUarts[port].pUart->isRxBusy) {
        status = ERR_HW_BUSY; goto END;
    }

    if (NULL == pUarts[port].pDmaRx) {
        status = ERR_HW_CONFIG; goto END;
    }

    /* Save buffer information */
    pUarts[port].pUart->bufferRx.pData  = pData;
    pUarts[port].pUart->bufferRx.maxLen = dataLen;
    pUarts[port].pUart->bufferRx.len    = 0;

    pUarts[port].pUart->isRxBusy = true;

    /* Enable IDLE interrupt for RX only */
    LL_USART_EnableIT_IDLE(pUarts[port].pUart->base);

    /* Register a private UART callback with DMA channel to notify UART driver
     * when DMA completes the transfer */
    DMA_registerCallback(pUarts[port].pDmaRx->channel,
            DmaTransferCompleteInt, UART_callbackDmaRxDone);

    if (ERR_NONE != (status = DMA_startTransfer(pUarts[port].pDmaRx->channel, pData, dataLen))) {
        goto END;
    }

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/******************************************************************************/
void UART_isr(Uart_t port)
{
    if (LL_USART_IsActiveFlag_ORE(pUarts[port].pUart->base)) {
        /* Overrun error. This occurs when either we ran the DMA stream out of
         * space to write data into when receiving or the stream was enabled,
         * no receive was initiated but data came in anyway and there was no
         * buffer to write into. */

        if (pUarts[port].pDmaRx) {
            DMA_abortTransfer(pUarts[port].pDmaRx->channel);
        }

        LL_USART_ClearFlag_ORE(pUarts[port].pUart->base);
    }

    if (LL_USART_IsEnabledIT_IDLE(pUarts[port].pUart->base) &&
        LL_USART_IsActiveFlag_IDLE(pUarts[port].pUart->base)) {

        /* IDLE interrupt was enabled and occurred */
        DMA_abortTransfer(pUarts[port].pDmaRx->channel);

        /* Don't call any callbacks here. When we disable the DMA stream, it
         * will automatically post a Transfer Complete DMA interrupt and we'll
         * let the DMA ISR handle it as if it normally completed. */

        LL_USART_ClearFlag_IDLE(pUarts[port].pUart->base);
        LL_USART_DisableIT_IDLE(pUarts[port].pUart->base);
    }

    pUarts[port].pUart->isRxBusy = false;
}

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
static Uart_t UART_getPortFromDmaChannel(Dma_t channel)
{
    Uart_t port = UART_MAX;
    for (port = UART_START; port < UART_MAX; port++) {
        if (pUarts[port].pDmaRx->channel == channel ||
            pUarts[port].pDmaTx->channel == channel) {
            return port;
        }
    }
    return UART_MAX;
}

/* Interrupt callback functions ----------------------------------------------*/
/* Functions below are handled in the ISR context including anything they call
 * so be concise and speedy */
/******************************************************************************/
static void UART_callbackDmaTxDone(
        Dma_t channel,
        Error_t status,
        Buffer_t *pBuffer
)
{
    Uart_t port = UART_getPortFromDmaChannel(channel);
    if (UART_MAX == port) {
        goto END;
    }

    if (pUarts[port].pUart->callbacks[UartEvtDataSent]) {
        pUarts[port].pUart->callbacks[UartEvtDataSent](port, status, pBuffer);
    }

END:                                     /* Tag to jump to in case of failure */

    /* We always want to clear this flag since whether we had an error or not,
     * we want to free up the driver to try again */
    pUarts[port].pUart->isTxBusy = false;
}

/******************************************************************************/
static void UART_callbackDmaRxDone(Dma_t channel, Error_t status, Buffer_t *pBuffer)
{
    Uart_t port = UART_getPortFromDmaChannel(channel);
    if (UART_MAX == port) {
        goto END;
    }

    if (pUarts[port].pUart->callbacks[UartEvtDataRcvd]) {
        pUarts[port].pUart->callbacks[UartEvtDataRcvd](port, status, pBuffer);
    }

END:                                     /* Tag to jump to in case of failure */

    /* We always want to clear this flag since whether we had an error or not,
     * we want to free up the driver to try again */
    pUarts[port].pUart->isRxBusy = false;
}

