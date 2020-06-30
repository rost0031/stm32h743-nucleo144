/**
 * @file    stm32h7xx_it.h
 * @brief   Interrupt vector function declarations
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief   This function handles NMI exception.
 * @return None
 */
void NMI_Handler(void);

/**
 * @brief  This function handles Hard Fault exception.
 * @return None
 */
void HardFault_Handler(void);

/**
 * @brief  This function handles Memory Manage exception.
 * @return None
 */
void MemManage_Handler(void);
/**
 * @brief  This function handles Bus Fault exception.
 * @return None
 */
void BusFault_Handler(void);

/**
 * @brief  This function handles Usage Fault exception.
 * @return None
 */
void UsageFault_Handler(void);

/**
 * @brief  This function handles SVCall exception.
 * @return None
 */
void SVC_Handler(void);

/**
 * @brief  This function handles Debug Monitor exception.
 * @return None
 */
void DebugMon_Handler(void);

/**
 * @brief  This function handles PendSVC exception.
 * @return None
 */
void PendSV_Handler(void);

/**
 * @brief  This function handles SysTick Handler.
 * @return None
 */
void SysTick_Handler(void);


/**
 * @brief  This function handles DMA2 stream1 interrupt request.
 * @return None
 */
void DMA2_Stream1_IRQHandler(void);

/**
 * @brief  This function handles DMA2 stream7 interrupt request.
 * @return None
 */
void DMA2_Stream7_IRQHandler(void);

/**
 * @brief  This function handles UART3 interrupts
 * @return None
 */
void USART3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif                                                    /* __STM32H7xx_IT_H */

