/**
  ******************************************************************************
  * @file    Internal.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file provides a set of functions needed to manage the Internal
  *          IO Expander devices.
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
#include "ADS7843_TS.h"
#include "LEDs.h"
#include "Display.h"

#include "stm32f2xx_hal.h"
//#include "stm32f2xx_hal.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define ADS7843TS_MAX_INSTANCE         2


/* Private variables ---------------------------------------------------------*/

int16_t ax, bx, ay, by;

/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandleX,AdcHandleY;

/* TIM handler declaration */
//static TIM_HandleTypeDef  htim;

/* Variable used to get converted value */
__IO uint32_t uhADCxConvertedValue = 0;

uint32_t ADCTSConvertedValue = 0;

/* Touch screen driver structure initialization */  
TS_DrvTypeDef ADS7843_ts_drv =
{
		ADS7843_Init,
		ADS7843_TS_Start,
		ADS7843_TS_DetectTouch,
		ADS7843_TS_GetXY
};

/* IO driver structure initialization */ 
IO_DrvTypeDef ADS7843_io_drv =
{
		ADS7843_Init,
		ADS7843_IO_Start,
		ADS7843_IO_Config
};


/* Internal instances by address */
uint8_t ADS7843TS[ADS7843TS_MAX_INSTANCE] = {0};
/**
  * @}
  */ 
    
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static uint8_t ADS7843_GetInstance(uint16_t DeviceAddr);
static void Error_Handler(void);

void TP_SPI_Config(void);

void touchGetSense(int16_t * x, int16_t * y);
int16_t touchGetX(void);
int16_t touchGetY(void);
void ADS7843_SendData(uint16_t	Data);
uint16_t ADS7843_ReceiveData(void);
uint16_t touchVerifyCoef(void);

/**
  * @brief  Initialize the Internal and configure the needed hardware resources
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void ADS7843_Init(uint16_t DeviceAddr)
{
  uint8_t instance;
  uint8_t empty;
  
  /* Check if device instance already exists */
  instance = ADS7843_GetInstance(DeviceAddr);

  printf("[TS] I:DeviceAddr: %d , instance: %d \n",DeviceAddr,instance);

  /* To prevent double initialization */
  if(instance == 0xFF)
  {
    /* Look for empty instance */
    empty = ADS7843_GetInstance(0);
    
    if(empty < ADS7843TS_MAX_INSTANCE)
    {
      /* Register the current device instance */
    	ADS7843TS[empty] = DeviceAddr;
      /* Initialize IO  */
    	TP_SPI_Config();
       printf("[TS] I: TP_SPI_Config..\n");
    }
  }
}


void ADS7843_IO_Start(uint16_t DeviceAddr, uint16_t IO_Pin)
{

}

/**
  */
void ADS7843_IO_Config(uint16_t DeviceAddr, uint16_t IO_Pin, IO_ModeTypedef IO_Mode)
{


}

/**
  */
void ADS7843_TS_Start(uint16_t DeviceAddr)
{

}
/************************************************************
 *
 */
uint8_t ADS7843_TS_DetectTouch(uint16_t DeviceAddr)
{
uint8_t ret = 0;

return ret;
}
/**
  * @brief  Get the touch screen X and Y positions values
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value   
  * @retval None.
  */
void ADS7843_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{

}
/**
  * @brief  Check if the device instance of the selected address is already registered
  *         and return its index
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Index of the device instance if registered, 0xFF if not.
  */
static uint8_t ADS7843_GetInstance(uint16_t DeviceAddr)
{
  uint8_t idx = 0;

  /* Check all the registered instances */
  for(idx = 0; idx < ADS7843TS_MAX_INSTANCE ; idx ++)
  {
    if(ADS7843TS[idx] == DeviceAddr)
    {
      return idx;
    }
  }

  return 0xFF;
}

/*****************************************************************
* @brief  This function is executed in case of error occurrence.
******************************************************************/
static void Error_Handler(void)
{
  TST_High(TST4);
  printf("ERROR: [ADS7843_TS] Error_Handler\n");
  while(1)  {  }
}
/******************************************************************
 *
******************************************************************/
void TP_SPI_Config(void)
{
	//NVIC_InitTypeDef nvic_InitStruct;
	GPIO_InitTypeDef  GPIO_InitStructure;

	__GPIOE_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();


	TP_Deselect();

	// spi out
	GPIO_InitStructure.Pin 		= Pin_TP_SCK | Pin_TP_MOSI;
	GPIO_InitStructure.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed 	= GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull 	= GPIO_PULLDOWN;
	HAL_GPIO_Init(PORT_TP_SPI, &GPIO_InitStructure);

	// spi in
	GPIO_InitStructure.Pin 		=  Pin_TP_MISO;
	GPIO_InitStructure.Mode 	= GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed 	= GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull 	= GPIO_PULLDOWN;
	HAL_GPIO_Init(PORT_TP_SPI, &GPIO_InitStructure);

	// spi cs
	GPIO_InitStructure.Pin 		= Pin_TP_CS;
	GPIO_InitStructure.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed 	= GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull 	= GPIO_PULLUP;
	HAL_GPIO_Init(PORT_TP_SPI, &GPIO_InitStructure);


	// TP interrupt
	GPIO_InitStructure.Pin 		= Pin_TP_IRQ;					//Pen Interrupt. Open anode output (requires 10kΩ to 100kΩ pull-up resistor externally).
	GPIO_InitStructure.Mode 	= GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed 	= GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull 	= GPIO_PULLUP;
	HAL_GPIO_Init(PORT_TP_IRQ, &GPIO_InitStructure);

	// TP BUSY
	GPIO_InitStructure.Pin 		= Pin_TP_BUSY;					// Busy Output. This output is high impedance when CS is HIGH.
	GPIO_InitStructure.Mode 	= GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed 	= GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull 	= GPIO_PULLUP;
	HAL_GPIO_Init(PORT_TP_BUSY, &GPIO_InitStructure);

	TP_Deselect();

	HAL_GPIO_WritePin(PORT_TP_CS, Pin_TP_SCK,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_TP_MOSI, Pin_TP_MOSI,GPIO_PIN_RESET);

}
/******************************************************************
 *
******************************************************************/
uint16_t getTouchState(void)
{
	uint16_t ret = 0;

	if (HAL_GPIO_ReadPin(PORT_TP_IRQ, Pin_TP_IRQ) == 0)
		ret = 1;
	return ret;
}
/******************************************************************
 *
******************************************************************/
void touchGetSense(int16_t * x, int16_t * y)
{
	*x = (touchGetX()/ax)+bx;
	*y = (touchGetY()/ay)+by;
}
/******************************************************************
 *
******************************************************************/
int16_t touchGetX(void)
{
	uint16_t LSB, MSB;
	int16_t ret = 4095;

	if (HAL_GPIO_ReadPin(PORT_TP_IRQ, Pin_TP_IRQ) == 0)
	{
		TP_Select();
//		ret = 0x2F;
//		while(--ret);

		ADS7843_SendData(0x94);
		MSB=ADS7843_ReceiveData();
		LSB=ADS7843_ReceiveData();

		TP_Deselect();

		ret =  ((MSB<<4) & 0x0FF0) | ((LSB>>4) & 0x000F);
	}
	return ret;
}
/******************************************************************
 *
******************************************************************/
int16_t touchGetY(void)
{
	uint16_t LSB, MSB;
	int16_t ret = 4095;

	if (HAL_GPIO_ReadPin(PORT_TP_IRQ, Pin_TP_IRQ) == 0)
	{
		TP_Select();
//		ret = 0x2F;
//		while(--ret);

		ADS7843_SendData(0xD4);
		MSB=ADS7843_ReceiveData();
		LSB=ADS7843_ReceiveData();

		TP_Deselect();

		ret =  ((MSB<<4) & 0x0FF0) | ((LSB>>4) & 0x000F);
	}
	return ret;
}
/****************************************************************
 * @param Data
******************************************************************/
void ADS7843_SendData(uint16_t	Data){
uint8_t i;

	for(i=0;i<8;i++){
		HAL_GPIO_WritePin(PORT_TP_CS, Pin_TP_SCK,GPIO_PIN_SET);

		if(Data & (1<<(7-i)))
			HAL_GPIO_WritePin(PORT_TP_MOSI, Pin_TP_MOSI,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(PORT_TP_MOSI, Pin_TP_MOSI,GPIO_PIN_RESET);

		HAL_GPIO_WritePin(PORT_TP_CS, Pin_TP_SCK,GPIO_PIN_RESET);

	}
	HAL_GPIO_WritePin(PORT_TP_MOSI, Pin_TP_MOSI,GPIO_PIN_RESET);
}

/****************************************************************
 *
******************************************************************/
uint16_t ADS7843_ReceiveData(void){

uint16_t ret = 0;
uint8_t i;

	for(i=0;i<8;i++){
		HAL_GPIO_WritePin(PORT_TP_CS, Pin_TP_SCK,GPIO_PIN_SET);

		if (HAL_GPIO_ReadPin(PORT_TP_MISO, Pin_TP_MISO) == 1)
		{
			ret=ret<<1; ret|=1;
		}else{
			ret=ret<<1;
		}

		HAL_GPIO_WritePin(PORT_TP_CS, Pin_TP_SCK,GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(PORT_TP_MOSI, Pin_TP_MOSI,GPIO_PIN_RESET);

	return ret;
}
/****************************************************************
 *
******************************************************************/
void touchSetCoef(int16_t _ax, int16_t _bx, int16_t _ay, int16_t _by)
{
	ax = _ax;
	bx = _bx;
	ay = _ay;
	by = _by;
}
/****************************************************************
 *
******************************************************************/
void touchGetCoef(int16_t *_ax, int16_t *_bx, int16_t *_ay, int16_t *_by)
{
	*_ax = ax;
	*_bx = bx;
	*_ay = ay;
	*_by = by;
}
/****************************************************************
 *
******************************************************************/
uint16_t touchVerifyCoef(void)
{
	uint16_t ret = 0;
	if (ax == 0 || ax == 0xFFFF
		|| bx == 0xFFFF
		|| ay == 0 || ay == 0xFFFF
		|| by == 0xFFFF)
	ret = 1;
	return ret;
}
