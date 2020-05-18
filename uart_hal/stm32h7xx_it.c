/*------------------------------------------------------------------------------
 -------------------------------------------------------------------------------
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 --                       THIS FILE IS UNCLASSIFIED                           --
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 -------------------------------------------------------------------------------
 -----------------------------------------------------------------------------*/

/**
 * @file    stm32h7xx_it.c
 * @brief   STM32 interrupts for UART HAL example
 *
 * Copyright 2020, Northrop Grumman Innovation Systems, Inc.
 * All other rights reserved.
 *
 * NORTHROP GRUMMAN PROPRIETARY LEVEL I
 * This information contains proprietary data and should not be released or
 * distributed without the express written approval of Northrop Grumman
 * Innovation Systems, Inc. This document contains private or privileged
 * information and/or trade secrets, which is/are disclosed in confidence.
 * This information may be used, duplicated, or disclosed only to the extent as
 * specifically authorized in writing by Northrop Grumman Innovation Systems,
 * Inc.
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_it.h"
#include "stm32h7xx_hal.h"

/* Private typedefs ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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


/* Private functions ---------------------------------------------------------*/


