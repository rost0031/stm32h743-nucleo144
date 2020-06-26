/**
 * @file    main.c
 * @brief   Main for UART LL example
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_usart.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

//#define USE_FULL_LL_DRIVER

/* Definition LED1-GREEN*/
#define LED1_PIN                           LL_GPIO_PIN_0
#define LED1_GPIO_PORT                     GPIOB
#define LED1_GPIO_CLK_ENABLE()             LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB)

/* Definition LED3-RED*/
#define LED3_PIN                           LL_GPIO_PIN_14
#define LED3_GPIO_PORT                     GPIOB
#define LED3_GPIO_CLK_ENABLE()             LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB)

#define BUFFER_SIZE                        (64)

/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

uint32_t timeout = 0;

/* GPIO Init Structure */
LL_GPIO_InitTypeDef  GPIO_InitStruct;
static uint8_t txBuffer[BUFFER_SIZE] = {0xA5};
static uint8_t rxBuffer[BUFFER_SIZE] = {0xA5};
static uint16_t rxBytes = 0;
static bool txBusy = false;
static bool rxBusy = false;
static bool rxDataRcvd = false;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL1 (HSE BYPASS)
 *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
 *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
 *            AHB Prescaler                  = 2
 *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
 *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
 *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
 *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 4
 *            PLL_N                          = 400
 *            PLL_P                          = 2
 *            PLL_Q                          = 4
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Flash Latency(WS)              = 4
 * @return None
 */
static void     SystemClock_Config(void);

/**
 * @brief  Initialize LED1 (Green LED).
 * @return None
 */
static void     LED1_Init(void);

/**
 * @brief  Initialize LED3 (Red LED).
 * @return None
 */
static void     LED3_Init(void);
static void DMA_init(void);
static void DMA_configTx(const uint8_t* const pData, uint16_t len);
static void DMA_configRx(uint8_t* const pData, uint16_t len);
/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
int main(void)
{
    timeout = 0xFF;

    SystemClock_Config();            /* Configure the system clock to 400 MHz */

    /* Initialize LED1 */
    LED1_Init();
    LED3_Init();

    LL_GPIO_SetOutputPin(LED1_GPIO_PORT, LED1_PIN);

    /* Configure USART3 for debug input/output. Configure pins and uart peripheral */

    /* Enable PORTD clock for GPIO port D */
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

    /* Configure PD8 as Tx Pin : Alternate function 7, High Speed, Push pull, Pull up */
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_8, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

    /* Configure PD9 as Rx Pin : Alternate function 7, High Speed, Push pull, Pull up */
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_9, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

    /* Enable the peripheral clock of UART */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

#if 0
    /* The function LL_USART_Init is horribly broken or is not meant to be used
     * with this chip. It sets the baudrate incorrectly. Even the LL examples in
     * manually configure the UARTs. Thanks again, STM. */
    LL_USART_InitTypeDef usartInit = {0};
    usartInit.BaudRate            = 115200U;
    usartInit.DataWidth           = LL_USART_DATAWIDTH_8B;
    usartInit.StopBits	          = LL_USART_STOPBITS_1;
    usartInit.Parity              = LL_USART_PARITY_NONE;
    usartInit.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usartInit.TransferDirection	  = LL_USART_DIRECTION_TX_RX;
    usartInit.OverSampling        = LL_USART_OVERSAMPLING_16;
    usartInit.PrescalerValue      = LL_USART_PRESCALER_DIV1;
    LL_USART_Init( USART3, &usartInit);
#endif

    DMA_init();

    /* TX/RX direction */
    LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_TX_RX);

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    LL_USART_ConfigCharacter(USART3, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate(USART3, SystemCoreClock/4, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_SetHWFlowCtrl(USART3, LL_USART_HWCONTROL_NONE);
    LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_EnableFIFO(USART3);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
    LL_USART_EnableDMAReq_TX(USART3);
    LL_USART_EnableIT_IDLE(USART3);
    LL_USART_Enable( USART3 );

    NVIC_SetPriority(USART3_IRQn, 0);
    NVIC_EnableIRQ(USART3_IRQn);

    /* Polling USART3 initialization */
    while (!LL_USART_IsActiveFlag_TEACK(USART3) || !LL_USART_IsActiveFlag_REACK(USART3)) {}

    const uint8_t buffer[25] = "Hello World\r\n\0";
    /* Infinite loop */
    if( !txBusy) {
        DMA_configTx(buffer, strlen(buffer));
    }

    uint8_t counter = 0;
    while (1) {

        if (!txBusy && rxDataRcvd) {
            uint16_t bytes = snprintf((char *)txBuffer, sizeof(txBuffer), "Loop %03d: rcvd %d bytes: %s\n", counter++, rxBytes, rxBuffer);
            DMA_configTx(txBuffer, bytes);
            rxDataRcvd = false;
        }

        if (!rxBusy) {
            DMA_configRx(rxBuffer, sizeof(rxBuffer));
        }

    }
}


/******************************************************************************/
void LED1_Init(void)
{
    /* Enable the LED1 Clock */
    LED1_GPIO_CLK_ENABLE();

    /* Configure IO in output push-pull mode to drive external LED1 */
    LL_GPIO_SetPinMode(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
}

/******************************************************************************/
void LED3_Init(void)
{
    /* Enable the LED3 Clock */
    LED3_GPIO_CLK_ENABLE();

    /* Configure IO in output push-pull mode to drive external LED3 */
    LL_GPIO_SetPinMode(LED3_GPIO_PORT, LED3_PIN, LL_GPIO_MODE_OUTPUT);
}

/******************************************************************************/
void UART_isr(void)
{
    /* Check for interrupt flags and errors */
    if (LL_USART_IsActiveFlag_ORE(USART3)) {                /* Overrun error? */
        LL_USART_ClearFlag_ORE(USART3);                 /* Clear Overrun flag */
        rxBytes = sizeof(rxBuffer);
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
    }


    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
        LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
        /* Calculate current position in buffer */
        rxBytes = sizeof(rxBuffer) - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
        rxDataRcvd = true;
    }

    rxBusy = false;
}

/******************************************************************************/
void DMA_txDone(void)
{
    if (LL_DMA_IsActiveFlag_TC7(DMA2) == 1) {
        LL_DMA_ClearFlag_TC7(DMA2);
    } else if(LL_DMA_IsActiveFlag_TE7(DMA2) == 1) {
        LL_DMA_ClearFlag_TE7(DMA2);
    }

    txBusy = false;
}

/******************************************************************************/
void DMA_rxDone(void)
{
    if (LL_DMA_IsActiveFlag_TC1(DMA2) == 1) {
        LL_DMA_ClearFlag_TC1(DMA2);
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
    } else if(LL_DMA_IsActiveFlag_TE1(DMA2) == 1) {
        LL_DMA_ClearFlag_TE1(DMA2);
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
    }

    rxBusy = false;
}

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
static void DMA_init(void)
{
    /* (1) Enable the clock of DMA2 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /* Configuration of the DMA parameters can be done using unitary functions or using the specific configure function */
    /* Unitary Functions */

    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_7, LL_DMAMUX1_REQ_USART3_TX);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT));
    /* Errata workaround for UART/DMA lockup */
    LL_DMA_EnableBufferableTransfer(DMA2, LL_DMA_STREAM_7);

    /* Configuration of the DMA parameters can be done using unitary functions or using the specific configure function */
    /* Unitary Functions */
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_USART3_RX);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE));
    /* Errata workaround for UART/DMA lockup */
    LL_DMA_EnableBufferableTransfer(DMA2, LL_DMA_STREAM_1);

    /* (3) Configure NVIC for DMA transfer complete/error interrupts */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_7);
    NVIC_SetPriority(DMA2_Stream7_IRQn, 0);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    /* (3) Configure NVIC for DMA transfer complete/error interrupts */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_1);
    NVIC_SetPriority(DMA2_Stream1_IRQn, 0);
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/******************************************************************************/
static void DMA_configTx(const uint8_t* const pData, uint16_t len)
{
//    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)pData, USART3->TDR, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)pData);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, len);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
    txBusy = true;
}

/******************************************************************************/
static void DMA_configRx(uint8_t* const pData, uint16_t len)
{
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)pData);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, len);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
    rxBusy = true;
}

/******************************************************************************/
static void SystemClock_Config(void)
{
    /* Power Configuration */
    LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (LL_PWR_IsActiveFlag_VOS() == 0)
    {
    }

    /* Enable HSE oscillator */
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();
    while(LL_RCC_HSE_IsReady() != 1)
    {
    }

    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

    /* Main PLL configuration and activation */
    LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
    LL_RCC_PLL1P_Enable();
    LL_RCC_PLL1Q_Enable();
    LL_RCC_PLL1R_Enable();
    LL_RCC_PLL1FRACN_Disable();
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
    LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL1_SetM(4);
    LL_RCC_PLL1_SetN(400);
    LL_RCC_PLL1_SetP(2);
    LL_RCC_PLL1_SetQ(4);
    LL_RCC_PLL1_SetR(2);
    LL_RCC_PLL1_Enable();
    while(LL_RCC_PLL1_IsReady() != 1)
    {
    }

    /* Set Sys & AHB & APB1 & APB2 & APB4  prescaler */
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    /* Set PLL1 as System Clock Source */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
    {
    }

    /* Set systick to 1ms */
    SysTick_Config(400000000 / 4000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    SystemCoreClock = 400000000;
}

