/**
 * @file    uart.c
 * @brief   UART driver that encapsulates all UART work to one spot
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include <stdbool.h>

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

static DMA_HandleTypeDef handleDmaTx[UART_MAX] = {
        [UART_DBG] = {
                .Instance                 = DMA2_Stream7,
                .Init = {
                        .Direction           = DMA_MEMORY_TO_PERIPH,
                        .PeriphInc           = DMA_PINC_DISABLE,
                        .MemInc              = DMA_MINC_ENABLE,
                        .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                        .MemDataAlignment    = DMA_MDATAALIGN_BYTE,
                        .Mode                = DMA_NORMAL,
                        .Priority            = DMA_PRIORITY_LOW,
                        .FIFOMode            = DMA_FIFOMODE_DISABLE,
                        .FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL,
                        .MemBurst            = DMA_MBURST_SINGLE,
                        .PeriphBurst         = DMA_MBURST_SINGLE,
                        .Request             = DMA_REQUEST_USART3_TX,
                },
        },
};

static DMA_HandleTypeDef handleDmaRx[UART_MAX] = {
        [UART_DBG] = {
                .Instance                 = DMA2_Stream1,
                .Init = {
                        .Direction           = DMA_PERIPH_TO_MEMORY,
                        .PeriphInc           = DMA_PINC_DISABLE,
                        .MemInc              = DMA_MINC_ENABLE,
                        .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                        .MemDataAlignment    = DMA_MDATAALIGN_BYTE,
                        .Mode                = DMA_NORMAL,
                        .Priority            = DMA_PRIORITY_LOW,
                        .FIFOMode            = DMA_FIFOMODE_DISABLE,
                        .FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL,
                        .MemBurst            = DMA_MBURST_SINGLE,
                        .PeriphBurst         = DMA_MBURST_SINGLE,
                        .Request             = DMA_REQUEST_USART3_RX,
                },
        },
};

/**
 * @brief   Array of all the UART devices used by the system
 */
static const UartDev_t uarts[UART_MAX] = {
        [UART_DBG] = {
                /* General UART settings */
                .handleUart.Instance          = USART3,
                .handleUart.Init.BaudRate     = 115200,
                .handleUart.Init.WordLength   = UART_WORDLENGTH_8B,
                .handleUart.Init.StopBits     = UART_STOPBITS_1,
                .handleUart.Init.Parity       = UART_PARITY_NONE,
                .handleUart.Init.HwFlowCtl    = UART_HWCONTROL_NONE,
                .handleUart.Init.Mode         = UART_MODE_TX_RX,
                .handleUart.Init.OverSampling = UART_OVERSAMPLING_16,
                .handleUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT,
                .irqUart                      = USART3_IRQn,
                .irqPrioUart                  = IRQ_PRIO_UART,

                /* UART GPIO pin configurations */
                .gpioTx.Mode                  = GPIO_MODE_AF_PP,
                .gpioTx.Pull                  = GPIO_PULLUP,
                .gpioTx.Speed                 = GPIO_SPEED_FREQ_VERY_HIGH,
                .gpioTx.Alternate             = GPIO_AF7_USART3,
                .gpioTx.Pin                   = GPIO_PIN_8,
                .gpioTxPort                   = GPIOD,
                .gpioRx.Mode                  = GPIO_MODE_AF_PP,
                .gpioRx.Pull                  = GPIO_PULLUP,
                .gpioRx.Speed                 = GPIO_SPEED_FREQ_VERY_HIGH,
                .gpioRx.Alternate             = GPIO_AF7_USART3,
                .gpioRx.Pin                   = GPIO_PIN_9,
                .gpioRxPort                   = GPIOD,

                /* DMA */
                .dmaTx = {
                        .handle = &handleDmaTx[UART_DBG],
                        .irqNumber      = DMA2_Stream7_IRQn,
                        .irqPriority    = IRQ_PRIO_DMA_UART_TX,
                },
                .dmaRx = {
                        .handle = &handleDmaRx[UART_DBG],
                        .irqNumber      = DMA2_Stream1_IRQn,
                        .irqPriority    = IRQ_PRIO_DMA_UART_RX,
                },
        },
};
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief   Find device by handle
 *
 */
static UartPort_t UART_findDeviceByHandle(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
);

/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
Error_t UART_init(UartPort_t port)
{
    Error_t status = ERR_NONE;

    /* The cast is just for STM HAL driver call since it expects a non-const
     * despite the fact it makes no changes */
    if (HAL_OK != HAL_UART_Init((UART_HandleTypeDef *)&(uarts[port].handleUart))) {
        status = ERR_HW_INIT_FAILURE; goto END;
    }



END:                                     /* Tag to jump to in case of failure */
    return status;
}

/******************************************************************************/
Error_t UART_start(UartPort_t port)
{
    Error_t status = ERR_NONE;

    /* Enable some extra interrupts that we care about */
    __HAL_UART_ENABLE_IT(&uarts[port].handleUart, UART_IT_IDLE);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(uarts[port].dmaTx.irqNumber, uarts[port].dmaTx.irqPriority, 0);
    HAL_NVIC_EnableIRQ(uarts[port].dmaTx.irqNumber);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(uarts[port].dmaRx.irqNumber, uarts[port].dmaRx.irqPriority, 0);
    HAL_NVIC_EnableIRQ(uarts[port].dmaRx.irqNumber);

    /* NVIC for USART, to catch the TX complete and RX Idle */
    HAL_NVIC_SetPriority(uarts[port].irqUart, uarts[port].irqPrioUart, 1);
    HAL_NVIC_EnableIRQ(uarts[port].irqUart);

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/* STM HAL callbacks ---------------------------------------------------------*/

/**
 * @brief UART MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 * @return None
 */
void HAL_UART_MspInit(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{
    if (NULL == huart) {
        return;
    }

    UartPort_t port = UART_MAX;
    if (UART_MAX == (port = UART_findDeviceByHandle(huart))) {
        return;
    }
    /* The cast is just for STM HAL driver call since it expects a non-const
     * despite the fact it makes no changes */
    HAL_GPIO_Init(uarts[port].gpioTxPort, (GPIO_InitTypeDef *)&(uarts[port].gpioTx));
    HAL_GPIO_Init(uarts[port].gpioRxPort, (GPIO_InitTypeDef *)&(uarts[port].gpioRx));

    /* Initialize the DMA we are going to use for this UART */
    HAL_DMA_Init((DMA_HandleTypeDef *)(uarts[port].dmaTx.handle));
    /* Associate the initialized DMA handle to the the UART handle. STM HAL macro
     * for linking DMA to UART won't work here since we did a goofy thing with our
     * DMA handler by making it a pointer in a larger structure so we'll just do it
     * manually */
//    __HAL_LINKDMA(huart, hdmatx, (DMA_HandleTypeDef *)&(uarts[port].handleDmaTx));
    huart->hdmatx = (DMA_HandleTypeDef *)(uarts[port].dmaTx.handle);
    uarts[port].dmaTx.handle->Parent = huart;

    HAL_DMA_Init((DMA_HandleTypeDef *)(uarts[port].dmaRx.handle));
//    __HAL_LINKDMA(huart, hdmarx, (DMA_HandleTypeDef *)&(uarts[port].handleDmaRx));
    huart->hdmarx = (DMA_HandleTypeDef *)(uarts[port].dmaRx.handle);
    uarts[port].dmaRx.handle->Parent = huart;

    /* Normally, you would enable interrupts here but we'll do that in UART_start() */

}

/**
 * @brief UART MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO configuration to their default state
 * @return None
 */
void HAL_UART_MspDeInit(
        UART_HandleTypeDef* huart           /**< [in,out] UART handle pointer */
)
{

}

/* Private functions ---------------------------------------------------------*/
static UartPort_t UART_findDeviceByHandle(UART_HandleTypeDef* huart)
{
    for (UartPort_t port = UART_DBG; port < UART_MAX; port++) {
        if (huart->Instance == uarts[port].handleUart.Instance) {
            return port;
        }
    }
    return UART_MAX;
}
