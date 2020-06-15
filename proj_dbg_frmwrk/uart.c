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
#include "stm32h7xx_hal_uart.h"

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
Error_t UART_deInit(UartPort_t port)
{
    Error_t status = ERR_NONE;

    /* The cast is just for STM HAL driver call since it expects a non-const
     * despite the fact it makes no changes */
    if (HAL_OK != HAL_UART_DeInit((UART_HandleTypeDef *)&(uarts[port].handleUart))) {
        status = ERR_HW_INIT_FAILURE; goto END;
    }

END:                                     /* Tag to jump to in case of failure */
    return status;
}

/******************************************************************************/
Error_t UART_start(UartPort_t port)
{
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

    /* There's just no error checking that can be done in this function but
     * we'll return success just so we can keep a consistent interface to the
     * driver */
    return ERR_NONE;
}

/******************************************************************************/
Error_t UART_stop(UartPort_t port)
{
    /* Enable some extra interrupts that we care about */
    __HAL_UART_DISABLE_IT(&uarts[port].handleUart, UART_IT_IDLE);

    /* Take down interrupts for DMA RX/TX and UART */
    HAL_NVIC_DisableIRQ(uarts[port].dmaTx.irqNumber);
    HAL_NVIC_DisableIRQ(uarts[port].dmaRx.irqNumber);
    HAL_NVIC_DisableIRQ(uarts[port].irqUart);

    /* There's just no error checking that can be done in this function but
     * we'll return success just so we can keep a consistent interface to the
     * driver */
    return ERR_NONE;
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
    /* Associate the initialized DMA handle to the the UART handle. STM HAL macros
     *
     * __HAL_LINKDMA(huart, hdmatx, (DMA_HandleTypeDef *)&(uarts[port].handleDmaTx));
     * __HAL_LINKDMA(huart, hdmarx, (DMA_HandleTypeDef *)&(uarts[port].handleDmaRx));
     *
     * for linking DMA to UART won't work here since we did a goofy thing with our
     * DMA handler by making it a pointer in a larger structure so we'll just do it
     * manually */
    huart->hdmatx = (DMA_HandleTypeDef *)(uarts[port].dmaTx.handle);
    uarts[port].dmaTx.handle->Parent = huart;

    HAL_DMA_Init((DMA_HandleTypeDef *)(uarts[port].dmaRx.handle));
    huart->hdmarx = (DMA_HandleTypeDef *)(uarts[port].dmaRx.handle);
    uarts[port].dmaRx.handle->Parent = huart;

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

    UartPort_t port = UART_MAX;
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
    HAL_GPIO_DeInit(uarts[port].gpioTxPort, uarts[port].gpioTx.Pin);
    HAL_GPIO_DeInit(uarts[port].gpioRxPort, uarts[port].gpioRx.Pin);

    /* Take down the DMA */
    if (NULL != huart->hdmatx) {
        HAL_DMA_DeInit(huart->hdmatx);
    }

    if (NULL != huart->hdmarx) {
        HAL_DMA_DeInit(huart->hdmarx);
    }
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
