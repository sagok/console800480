/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/stm32f2xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-March-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "main.h"
#include "LEDs.h"
//#include "sd_diskio.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/** @addtogroup STM32F2xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd;
extern HCD_HandleTypeDef hhcd;

/* UART handler declared in "usbd_cdc_interface.c" file */
extern UART_HandleTypeDef 	UartHandle;
extern SD_HandleTypeDef 	uSdHandle;
//extern SPI_HandleTypeDef 	SpiHandle;
extern ADC_HandleTypeDef    AdcHandle;


extern xSemaphoreHandle 	xSemaphore_USBInt;			//

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
/**
  * @bri
  * @param  None
  * @retval None
  */


/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles USB-On-The-Go FS or HS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)

{
//portBASE_TYPE xTaskWoken= pdFALSE;

//	TST_High(TST6);

	LED_Toggle(LED5);


	HAL_PCD_IRQHandler(&hpcd);
//	HAL_HCD_IRQHandler(&hhcd);

//	    xSemaphoreGiveFromISR( xSemaphore_USBInt, &xTaskWoken );
//	    if( xTaskWoken == pdTRUE) {
//	        taskYIELD();
//	    }
//	TST_Low(TST6);

}
void OTG_HS_IRQHandler(void)
{
    LED_Toggle(LED4);

	TST_High(TST4);
	HAL_HCD_IRQHandler(&hhcd);
	TST_Low(TST4);

}
/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA stream
  *         used for USART data reception
  */
void USARTx_DMA_TX_IRQHandler(void)
{
  LED_Toggle(LED3);
  TST_High(TST1);
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
  TST_Low(TST1);

}

void USARTx_DMA_RX_IRQHandler(void)
{
//	TST_High(TST3);
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}


/**
  * @brief  This function handles DMA Rx interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_DMA_RX_IRQHandler(void)
{

//  HAL_DMA_IRQHandler(SpiHandle.hdmarx);
//	HAL_DMA_IRQHandler(uSdHandle.hdmarx);
}

/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_DMA_TX_IRQHandler(void)
{
//  HAL_DMA_IRQHandler(SpiHandle.hdmatx);
//	HAL_DMA_IRQHandler(uSdHandle.hdmatx);

}

/**
  * @brief  This function handles ADC interrupt request.
  * @param  None
  * @retval None
  */
void ADCTS_IRQHandler(void)
{
//  HAL_ADC_IRQHandler(&AdcHandle);
//  printf("HAL_ADC_IRQHandler\n");

}
/**
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
void ADCTS_DMA_IRQHandler(void)
{
//  HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
 // printf("ADCTS_DMA_IRQHandler\n");
}


/**
  * @brief  This function handles DMA2 Stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)
{
  LED_High(LED0);
//  HAL_DMA_IRQHandler(uSdHandle.hdmarx);
  BSP_SD_DMA_Rx_IRQHandler();
}

/**
  * @brief  This function handles DMA2 Stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream6_IRQHandler(void)
{
   LED_High(LED1);
//   HAL_DMA_IRQHandler(uSdHandle.hdmatx);
  BSP_SD_DMA_Tx_IRQHandler();
}

/**
  * @brief  This function handles SDIO interrupt request.
  * @param  None
  * @retval None
  */
void SDIO_IRQHandler(void)
{
  LED_High(LED2);
//  HAL_SD_IRQHandler(&uSdHandle);		//&uSdHandle
  BSP_SD_IRQHandler();
 // SD_ProcessIRQSrc ();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
