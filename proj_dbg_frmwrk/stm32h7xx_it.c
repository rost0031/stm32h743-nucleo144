/**
 * @file    stm32h7xx_it.c
 * @brief   STM32 interrupts for UART HAL example
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_it.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "uart.h"

/* Private typedefs ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef UartHandle;
/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

/******************************************************************************/
void NMI_Handler(void)
{

}

/******************************************************************************/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/******************************************************************************/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/******************************************************************************/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/******************************************************************************/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/******************************************************************************/
void SVC_Handler(void)
{
}


/******************************************************************************/
void DebugMon_Handler(void)
{
}

/******************************************************************************/
void PendSV_Handler(void)
{
}

/******************************************************************************/
void SysTick_Handler(void)
{
    HAL_IncTick();
}


/******************************************************************************/
/*                 STM32H7xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32h7xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream1_IRQHandler(void)
{
//    HAL_DMA_IRQHandler(UartHandle.hdmarx);
    UART_redirectDmaIsrToHAL(DMA2_Stream1);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
//  HAL_DMA_IRQHandler(UartHandle.hdmatx);
    UART_redirectDmaIsrToHAL(DMA2_Stream7);
}

/**
  * @brief  This function handles external lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
    if (0x00 != __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10)) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    }
    if (0x00 != __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11)) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    }
    if (0x00 != __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12)) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    }
    if (0x00 != __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13)) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    }
    if (0x00 != __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14)) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    }
    if (0x00 != __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15)) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    }
}

/**
  * @brief  This function handles UART3 interrupts
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
//    HAL_UART_IRQHandler(&UartHandle);
    UART_redirectIsrToHAL(USART3);
}

/* Private functions ---------------------------------------------------------*/


