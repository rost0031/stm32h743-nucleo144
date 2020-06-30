/**
 * @file    dma.c
 * @brief   DMA driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"
#include "dma_data.h"
#include <stddef.h>
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/



///**
//  * @brief  Helper macro to calculate stream index from DMA base address
//  * @note   This is used to get the stream index
//  * @param  __DMA_INSTANCE__ DMAx
//  * @return uint32_t stream number
//  */
//#define LL_DMA_CALC_STREAM_NUMBER(__DMA_INSTANCE__) \
//        ((uint32_t)((((__DMA_INSTANCE__) & 0xFFU) - 16U) / 24U))
//
///**
//  * @brief  Calculate stream index from DMA base address
//  * @note   This is used to get the stream index
//  * @param  DMAx
//  * @return uint32_t stream number
//  */
//__STATIC_INLINE uint32_t LL_DMA_calcStreamNumber(DMA_TypeDef *DMAx)
//{
//    return (((uint32_t)(((DMAx) & 0xFFU) - 16U) / 24U));
//}

/**
  * @brief  Helper macro to calculate stream index from DMA base address
  * @note   This is used to get the stream index
  * @param  __DMA_INSTANCE__ DMAx
  * @return uint32_t stream number
  */
//#define LL_DMA_CALC_STREAM_NUMBER(_DMA_STRM_INST_) \
//        ((uint32_t)(((((uint32_t *)(_DMA_STRM_INST_)) & 0xFFU) - 16U) / 24U))

/**
  * @brief  Helper macro to calculate stream index from DMA base address
  * @param  __DMA_INSTANCE__ DMAx
  * @return uint32_t stream index
  */
//#define LL_DMA_CALC_STREAM_INDEX(__DMA_INSTANCE__)                             \
//        do {                                                                   \
//            uint32_t streamNum = LL_DMA_CALC_STREAM_NUMBER(__DMA_INSTANCE__);  \
//            static const bitShifts[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U}; \
//            uint32_t streamIdx = bitShifts[streamNum & 0x7U];                  \
//        } while(0);

/**
  * @brief  Helper macro to calculate stream index from DMA base address
  * @param  __DMA_INSTANCE__ DMAx
  * @return uint32_t stream index
  */
//#define LL_DMA_CALC_STREAM_INDEX(_DMA_STRM_INST_)                              \
//        do {                                                                   \
//            uint32_t streamNum = LL_DMA_CALC_STREAM_NUMBER(_DMA_STRM_INST_);   \
//            static const bitShifts[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U}; \
//            uint32_t streamIdx = bitShifts[streamNum & 0x7U];                  \
//        } while(0);

//static inline uint32_t LL_DMA_calcStreamIndex(DMA_Stream_TypeDef* const streamAddr)
//{
////    uint32_t streamNum = LL_DMA_CALC_STREAM_NUMBER(streamAddr);
//    static const bitShifts[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
//    uint32_t streamIdx = bitShifts[streamNum & 0x7U];
//    return streamIdx;
//}

static inline uint32_t LL_DMA_calcStreamBaseAddr(DMA_Stream_TypeDef *DMAx)
{
//    uint32_t streamNum = LL_DMA_CALC_STREAM_NUMBER(DMAx);
    uint32_t streamNum = (((uint32_t)((uint32_t*)(DMAx)) & 0xFFU) - 16U) / 24U;
//    uint32_t streamIdx = LL_DMA_calcStreamIndex(DMAx);
    static const bitShifts[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
    uint32_t streamIdx = bitShifts[streamNum & 0x7U];
    uint32_t streamBaseAddr = 0;
    if (streamNum > 3) {
        streamBaseAddr = (((uint32_t)((uint32_t*)DMAx) & (uint32_t)(~0x3FFU)) + 4U);
    } else {
        streamBaseAddr = (((uint32_t)((uint32_t*)DMAx) & (uint32_t)(~0x3FFU)));
    }
    return streamBaseAddr;
}


/* Private variables and Local objects ---------------------------------------*/

static DmaData_t *pDMAs = NULL;                      /**< pointer to DMA data */

/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
void DMA_init(Dma_t channel)
{
    /* First time this is called, retrieve the DMA data array from the user
     * DMA data that is specific to this board */
    if (NULL == pDMAs) {
        DMA_getData(&pDMAs);
        /* First time, enable both DMA clocks. We could be a little more careful
         * if we cared about power but for this purpose, we don't care. Ideally,
         * we would have a driver module for clocks that would keep track of all
         * the peripherals using clocks and allow disabling only if no other
         * devices are using the same clock */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    }

    LL_DMA_SetPeriphRequest(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphRequest);

    LL_DMA_SetDataTransferDirection(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.Direction);

    LL_DMA_SetStreamPriorityLevel(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.Priority);

    LL_DMA_SetMode(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.Mode);

    LL_DMA_SetPeriphIncMode(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphOrM2MSrcIncMode);

    LL_DMA_SetMemoryIncMode(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.MemoryOrM2MDstIncMode);

    LL_DMA_SetPeriphSize(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphOrM2MSrcDataSize);

    LL_DMA_SetMemorySize(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.MemoryOrM2MDstDataSize);

    LL_DMA_DisableFifoMode(pDMAs[channel].base, pDMAs[channel].stream);

    LL_DMA_SetPeriphAddress(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphOrM2MSrcAddress);

    /* Errata workaround for UART/DMA lockup */
    LL_DMA_EnableBufferableTransfer(pDMAs[channel].base, pDMAs[channel].stream);

    /* Make sure to clear this internal state variable so the driver is
     * available for work. */
    pDMAs[channel].pDynData->isBusy = false;

}

/******************************************************************************/
void DMA_deinit(Dma_t channel)
{
    /* This function shouldn't need to retrieve DMA data since it should already
     * exist but some users are paranoid and like to call deinit before calling
     * init so this allows that silly behavior without trying to dereference a
     * NULL pointer */
    if (NULL == pDMAs) {
        DMA_getData(&pDMAs);
    }

    /* Set the busy flag so that if someone tries to use this after it's been
     * deinitialized and happen to check for errors, at least we'll let them
     * know that a problem exists. */
    pDMAs[channel].pDynData->isBusy = true;
}

/******************************************************************************/
void DMA_start(Dma_t channel)
{
    LL_DMA_EnableIT_TC(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_EnableIT_TE(pDMAs[channel].base, pDMAs[channel].stream);
    NVIC_SetPriority(pDMAs[channel].irq, pDMAs[channel].prio);
    NVIC_EnableIRQ(pDMAs[channel].irq);

    /* Make sure to clear this internal state variable so the driver is
     * available for work. */
    pDMAs[channel].pDynData->isBusy = false;
}

/******************************************************************************/
void DMA_stop(Dma_t channel)
{
    LL_DMA_DisableIT_TC(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_TE(pDMAs[channel].base, pDMAs[channel].stream);
    NVIC_DisableIRQ(pDMAs[channel].irq);

    /* Set the busy flag so that if someone tries to use this after it's been
     * stopped and happen to check for errors, at least we'll let them
     * know that a problem exists. */
    pDMAs[channel].pDynData->isBusy = true;
}

/******************************************************************************/
void DMA_regCallback(Dma_t channel, DmaCallback_t callback)
{
    pDMAs[channel].pDynData->callback = callback;
}

/******************************************************************************/
void DMA_clrCallback(Dma_t channel)
{
    pDMAs[channel].pDynData->callback = NULL;
}

/******************************************************************************/
Error_t DMA_transfer(Dma_t channel, const uint8_t* const pData, uint16_t len)
{
    Error_t status = ERR_NONE;

    if (NULL == pData) {
        status = ERR_MEM_NULL; goto END;
    }

    if (0 == len) {
        status = ERR_LEN_INVALID; goto END;
    }

    if (true == pDMAs[channel].pDynData->isBusy) {
        status = ERR_HW_BUSY; goto END;
    }

    /* We don't want to start a new DMA transfer while this one is going */
    pDMAs[channel].pDynData->isBusy = true;

    /* The only configuration needed to start a transfer is to set number of
     * bytes and the buffer location. Then we can enable the stream to go. */
    LL_DMA_SetMemoryAddress(pDMAs[channel].base, pDMAs[channel].stream, (uint32_t)pData);
    LL_DMA_SetDataLength(pDMAs[channel].base, pDMAs[channel].stream, len);
    LL_DMA_EnableStream(pDMAs[channel].base, pDMAs[channel].stream);

END:                                       /* Tag to jump to in case of error */
    return status;
}

/******************************************************************************/
void DMA_isr(Dma_t channel)
{
//    uint32_t streamBaseAddr = LL_DMA_calcStreamBaseAddr(pDMAs[channel].streamBase);
    /* TODO: Make this more general after getitng the basics working. */

    if (LL_DMA_IsActiveFlag_TC7(pDMAs[channel].base) == 1) {
        if(NULL != pDMAs[channel].pDynData->callback) {
            pDMAs[channel].pDynData->callback(channel);
        }
        LL_DMA_ClearFlag_TC7(pDMAs[channel].base);
        pDMAs[channel].pDynData->isBusy = false;
    } else if(LL_DMA_IsActiveFlag_TE7(pDMAs[channel].base) == 1) {
        if(NULL != pDMAs[channel].pDynData->callback) {
            pDMAs[channel].pDynData->callback(channel);
        }
        pDMAs[channel].pDynData->isBusy = false;
        LL_DMA_ClearFlag_TE7(pDMAs[channel].base);
    }

    if (LL_DMA_IsActiveFlag_TC1(pDMAs[channel].base) == 1) {
        if(NULL != pDMAs[channel].pDynData->callback) {
            pDMAs[channel].pDynData->callback(channel);
        }
        LL_DMA_ClearFlag_TC1(pDMAs[channel].base);
        pDMAs[channel].pDynData->isBusy = false;
    } else if(LL_DMA_IsActiveFlag_TE1(pDMAs[channel].base) == 1) {
        if(NULL != pDMAs[channel].pDynData->callback) {
            pDMAs[channel].pDynData->callback(channel);
        }
        pDMAs[channel].pDynData->isBusy = false;
        LL_DMA_ClearFlag_TE1(pDMAs[channel].base);
    }
}

/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
//static uint32_t DMA_calcStreamNumber(Dma_t channel)
//{
//    uint32_t streamNum = (((uint32_t)((uint32_t*)pDMAs[channel].base) & 0xFFU) - 16U) / 24U;
//    return streamNum;
//}
//
///******************************************************************************/
//static uint32_t DMA_calcStreamIndex(Dma_t channel)
//{
//    uint32_t streamNum = DMA_calcStreamNumber(channel);
//
//    /* lookup table for necessary bitshift of flags within status registers */
//    static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
//    uint32_t streamIndex = flagBitshiftOffset[streamNum & 0x7U];
//    return streamIndex;
//}
//
///******************************************************************************/
//static uint32_t DMA_calcStreamBaseAddr(Dma_t channel)
//{
//    uint32_t streamNumber = DMA_calcStreamNumber(channel);
//    uint32_t streamIndex = DMA_calcStreamIndex(channel);
//
//
//    if(IS_DMA_STREAM_INSTANCE(hdma->Instance) != 0U) /* DMA1 or DMA2 instance */
//    {
//        uint32_t stream_number = (((uint32_t)((uint32_t*)hdma->Instance) & 0xFFU) - 16U) / 24U;
//
//        /* lookup table for necessary bitshift of flags within status registers */
//        static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
//        hdma->StreamIndex = flagBitshiftOffset[stream_number & 0x7U];
//
//        if (stream_number > 3U)
//        {
//            /* return pointer to HISR and HIFCR */
//            hdma->StreamBaseAddress = (((uint32_t)((uint32_t*)hdma->Instance) & (uint32_t)(~0x3FFU)) + 4U);
//        }
//        else
//        {
//            /* return pointer to LISR and LIFCR */
//            hdma->StreamBaseAddress = ((uint32_t)((uint32_t*)hdma->Instance) & (uint32_t)(~0x3FFU));
//        }
//    }
//
//    return hdma->StreamBaseAddress;
//}
