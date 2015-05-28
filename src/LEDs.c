/*
 * LEDs.c
 *
 *  Created on: 11.08.2014
 *      Author: sagok
 */

#include "stm32f2xx_hal.h"
#include "LEDs.h"


/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "\nUART Debuger ready\n";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED0_GPIO_PORT,
								 LED1_GPIO_PORT,
                                 LED2_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED4_GPIO_PORT,
                                 LED5_GPIO_PORT,
                                 LED6_GPIO_PORT,
                                 LED7_GPIO_PORT,
                                 LED8_GPIO_PORT,
                                 LED9_GPIO_PORT,
                                 LED10_GPIO_PORT,
                                 LED11_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED0_PIN,
								 LED1_PIN,
                                 LED2_PIN,
                                 LED3_PIN,
                                 LED4_PIN,
                                 LED5_PIN,
                                 LED6_PIN,
                                 LED7_PIN,
                                 LED8_PIN,
                                 LED9_PIN,
                                 LED10_PIN,
                                 LED11_PIN};

GPIO_TypeDef* TST_PORT[TSTn] = {TST1_GPIO_PORT,
								TST3_GPIO_PORT,
								TST4_GPIO_PORT,
								TST5_GPIO_PORT,
								TST6_GPIO_PORT};

const uint16_t TST_PIN[TSTn] = {TST1_PIN,
								TST3_PIN,
                                TST4_PIN,
								TST5_PIN,
								TST6_PIN};
/***********************************************************
 *
 ***********************************************************/
static void Error_Handler(uint16_t	Nerror){
	TST_High(TST4);
//	OutToAllLED(Nerror);
}
/***********************************************************
 *
 ***********************************************************/
void Uart1_Init(void){


	UartHandle.Instance        = USARTx;
	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;


	if(HAL_UART_Init(&UartHandle) != HAL_OK){	  Error_Handler(1);	}

	  /*##-2- Start the transmission process #####################################*/
	  /* User start transmission data through "TxBuffer" buffer */
	  if (HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)aTxBuffer, TXBUFFERSIZE) != HAL_OK)
	  {
	    /* Transfer error in transmission process */
	    Error_Handler(2);
	  }

	  /*##-3- Put UART peripheral in reception process ###########################*/
	  /* Any data received will be stored in "RxBuffer" buffer : the number max of
	     data received is 10 */
//	  if (HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
//	  {
	    /* Transfer error in reception process */
//	    Error_Handler(3);
//	  }

	  /*##-4- Wait for the end of the transfer ###################################*/
	  /*  Before starting a new communication transfer, you need to check the current
	      state of the peripheral; if it’s busy you need to wait for the end of current
	      transfer before starting a new one.
	      For simplicity reasons, this example is just waiting till the end of the
	      transfer, but application may perform other tasks while transfer operation
	      is ongoing. */
//	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
//	  {
//	  }

	  /*##-5- Send the received Buffer ###########################################*/
	//  if (HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	//  {
	    /* Transfer error in transmission process */
	//    Error_Handler(4);
	//  }

	  /*##-6- Wait for the end of the transfer ###################################*/
	  /*  Before starting a new communication transfer, you need to check the current
	      state of the peripheral; if it’s busy you need to wait for the end of current
	      transfer before starting a new one.
	      For simplicity reasons, this example is just waiting till the end of the
	      transfer, but application may perform other tasks while transfer operation
	      is ongoing. */
//	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
//	  {
//	  }

}

/***********************************************************
 *
 ***********************************************************/
void AllLEDs_Init(void){
static unsigned int i;

	for(i=LED0;i<=LED11;i++){
		LED_Init(i,GPIO_MODE_OUTPUT_OD);
		LED_High(i);
	}
}
/***********************************************************
 *
 ***********************************************************/
void OutToAllLED(uint16_t DataToOut){
static unsigned int i;

	for(i=LED0;i<=LED11;i++){
		if   (DataToOut & 0x800>>i) LED_Low(i);
		else LED_High(i);
	}
}
/***********************************************************
 *
 ***********************************************************/
void AllTSTs_Init(void){
static unsigned int i;

	for(i=TST1;i<=TST6;i++){
		TST_Init(i);
	}
}
/***********************************************************
 *
 ***********************************************************/
void LED_Init(Led_TypeDef Led,uint32_t Mode)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */

	  LEDx_GPIO_CLK_ENABLE(Led);

	  /* Configure the GPIO_LED pin */
	  GPIO_InitStruct.Pin = GPIO_PIN[Led];
	  GPIO_InitStruct.Mode = Mode;//GPIO_MODE_OUTPUT_OD;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

	  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}
/***********************************************************
 *
 ***********************************************************/
void TST_Init(Led_TypeDef TST)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */
  TSTx_GPIO_CLK_ENABLE(TST);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = TST_PIN[TST];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(TST_PORT[TST], &GPIO_InitStruct);
}
/************************************************************************************
 * ReadInputDataBit
*************************************************************************************/
uint8_t ReadInputDataBit(Led_TypeDef Led)
{
 uint8_t bitstatus = 0x00;

 /* Check the parameters */
 assert_param(IS_GPIO_ALL_PERIPH(GPIO_PORT[Led]));
 assert_param(IS_GET_GPIO_PIN(GPIO_PIN[Led]));

 if ((GPIO_PORT[Led]->IDR & GPIO_PIN[Led]) != (uint32_t)Bit_RESET)
 {
   bitstatus = (uint8_t)Bit_SET;
 }
 else
 {
   bitstatus = (uint8_t)Bit_RESET;
 }
 return bitstatus;
}
/***********************************************************
 *
 ***********************************************************/
void LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}
/***********************************************************
 *
 ***********************************************************/
void LED_High(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led],GPIO_PIN_SET);
}
/***********************************************************
 *
 ***********************************************************/
void LED_Low(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led],GPIO_PIN_RESET);
}
/***********************************************************
 *
 ***********************************************************/
void TST_Toggle(Led_TypeDef TST)
{
  HAL_GPIO_TogglePin(TST_PORT[TST], TST_PIN[TST]);
}
/***********************************************************
 *
 ***********************************************************/
void TST_High(Led_TypeDef TST)
{
	HAL_GPIO_WritePin(TST_PORT[TST], TST_PIN[TST],GPIO_PIN_SET);
}
/***********************************************************
 *
 ***********************************************************/
void TST_Low(Led_TypeDef TST)
{
	HAL_GPIO_WritePin(TST_PORT[TST], TST_PIN[TST],GPIO_PIN_RESET);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  //HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
//	OutToAllLED(ch);

	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)&ch, 1);

  return ch;
}
/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;



  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  /* Enable USARTx clock */
  USARTx_CLK_ENABLE();
  /* Enable DMA2 clock */
  DMAx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
   GPIO_InitStruct.Pin = USARTx_RX_PIN;
   GPIO_InitStruct.Alternate = USARTx_RX_AF;

   HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;

  hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;//DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;//DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init(&hdma_tx);

  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(huart, hdmatx, hdma_tx);

  /* Configure the DMA handler for reception process */
  hdma_rx.Instance                 = USARTx_RX_DMA_STREAM;

  hdma_rx.Init.Channel             = USARTx_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init(&hdma_rx);

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(huart, hdmarx, hdma_rx);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
  HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 0, 1);	//0,1
  HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
   HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);

}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{

	  static DMA_HandleTypeDef hdma_tx;
	  static DMA_HandleTypeDef hdma_rx;

	  /*##-1- Reset peripherals ##################################################*/
	  USARTx_FORCE_RESET();
	  USARTx_RELEASE_RESET();

	  /*##-2- Disable peripherals and GPIO Clocks #################################*/
	  /* Configure UART Tx as alternate function  */
	  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
	  /* Configure UART Rx as alternate function  */
	  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

	  /*##-3- Disable the DMA Streams ############################################*/
	  /* De-Initialize the DMA Stream associate to transmission process */
	  HAL_DMA_DeInit(&hdma_tx);
	  /* De-Initialize the DMA Stream associate to reception process */
	  HAL_DMA_DeInit(&hdma_rx);

	  /*##-4- Disable the NVIC for DMA ###########################################*/
	  HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);
	  HAL_NVIC_DisableIRQ(USARTx_DMA_RX_IRQn);
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED1 on: Transfer in transmission process is correct */
	TST_High(TST1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED2 on: Transfer in reception process is correct */
//	TST_High(TST3);
	TST_Toggle(TST3);
}
