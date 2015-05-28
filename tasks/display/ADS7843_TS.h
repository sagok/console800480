/**
  ******************************************************************************
  * @file    stmpe811.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file contains all the functions prototypes for the
  *          stmpe811.c IO expander driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INTERNALTS_H
#define __INTERNALTS_H

#ifdef __cplusplus
 extern "C" {
#endif   
   
/* Includes ------------------------------------------------------------------*/
#include "ts.h"
#include "io.h"
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
  
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#define Pin_TP_CS                       GPIO_PIN_3						// PE3
#define PORT_TP_CS                  	GPIOE
#define TP_CS_GPIO_CLK_ENABLE()         __GPIOE_CLK_ENABLE()
#define TP_CS_GPIO_CLK_DISABLE()        __GPIOE_CLK_DISABLE()

#define Pin_TP_SCK                      GPIO_PIN_6
#define PORT_TP_SCK                  	GPIOF
#define TP_SCK_GPIO_CLK_ENABLE()         __GPIOF_CLK_ENABLE()
#define TP_SCK_GPIO_CLK_DISABLE()        __GPIOF_CLK_DISABLE()

#define Pin_TP_MOSI                     GPIO_PIN_7
#define PORT_TP_MOSI                  	GPIOF
#define TP_MOSI_GPIO_CLK_ENABLE()       __GPIOF_CLK_ENABLE()
#define TP_MOSI_GPIO_CLK_DISABLE()      __GPIOF_CLK_DISABLE()

#define Pin_TP_MISO                     GPIO_PIN_8
#define PORT_TP_MISO                  	GPIOF
#define TP_MISO_GPIO_CLK_ENABLE()       __GPIOF_CLK_ENABLE()
#define TP_MISO_GPIO_CLK_DISABLE()      __GPIOF_CLK_DISABLE()

#define PORT_TP_SPI                  	GPIOF

#define Pin_TP_IRQ                      GPIO_PIN_5
#define PORT_TP_IRQ                  	GPIOE
#define TP_IRQ_GPIO_CLK_ENABLE()        __GPIOE_CLK_ENABLE()
#define TP_IRQ_GPIO_CLK_DISABLE()       __GPIOE_CLK_DISABLE()

#define Pin_TP_BUSY                      GPIO_PIN_4
#define PORT_TP_BUSY                 	GPIOE
#define TP_BUSY_GPIO_CLK_ENABLE()       __GPIOE_CLK_ENABLE()
#define TP_BUSY_GPIO_CLK_DISABLE()      __GPIOE_CLK_DISABLE()

#define TP_Select()			HAL_GPIO_WritePin(PORT_TP_CS, Pin_TP_CS,GPIO_PIN_RESET);   //GPIO_ResetBits(PORT_TP_CS, GPIO_Pin_TP_CS)
#define TP_Deselect()		HAL_GPIO_WritePin(PORT_TP_CS, Pin_TP_CS,GPIO_PIN_SET);   //GPIO_SetBits(PORT_TP_CS, GPIO_Pin_TP_CS)


void     ADS7843_Init(uint16_t DeviceAddr);
//void     Internal_Reset(uint16_t DeviceAddr);
uint16_t ADS7843_ReadID(uint16_t DeviceAddr);
//void     Internal_EnableGlobalIT(uint16_t DeviceAddr);
//void     Internal_DisableGlobalIT(uint16_t DeviceAddr);
//void     Internal_EnableITSource(uint16_t DeviceAddr, uint8_t Source);
//void     Internal_DisableITSource(uint16_t DeviceAddr, uint8_t Source);
//void     Internal_SetITPolarity(uint16_t DeviceAddr, uint8_t Polarity);
//void     Internal_SetITType(uint16_t DeviceAddr, uint8_t Type);
//uint8_t  Internal_GlobalITStatus(uint16_t DeviceAddr, uint8_t Source);
//uint8_t  Internal_ReadGITStatus(uint16_t DeviceAddr, uint8_t Source);
//void     Internal_ClearGlobalIT(uint16_t DeviceAddr, uint8_t Source);


/** 
  * @brief STMPE811 IO functionalities functions
  */
void     ADS7843_IO_Start(uint16_t DeviceAddr, uint16_t IO_Pin);
void     ADS7843_IO_Config(uint16_t DeviceAddr, uint16_t IO_Pin, IO_ModeTypedef IO_Mode);
//void     Internal_IO_InitPin(uint16_t DeviceAddr, uint16_t IO_Pin, uint8_t Direction);

/** 
  * @brief STMPE811 Touch screen functionalities functions
  */
void     ADS7843_TS_Start(uint16_t DeviceAddr);
void     ADS7843_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);
uint8_t  ADS7843_TS_DetectTouch(uint16_t DeviceAddr);
void     ADS7843_TS_EnableIT(uint16_t DeviceAddr);

uint16_t getTouchState(void);
void touchSetCoef(int16_t _ax, int16_t _bx, int16_t _ay, int16_t _by);
void touchGetCoef(int16_t *_ax, int16_t *_bx, int16_t *_ay, int16_t *_by);

void     IOE_Init(void);
void     IOE_ITConfig (void);
void     IOE_Delay(uint32_t delay);
void     IOE_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t  IOE_Read(uint8_t addr, uint8_t reg);
uint16_t IOE_ReadMultiple(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);

/* Touch screen driver structure */
extern TS_DrvTypeDef 	ADS7843_ts_drv;

/* IO driver structure */
extern IO_DrvTypeDef 	ADS7843_io_drv;

#ifdef __cplusplus
}
#endif
#endif /* __Internal_H */


/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */       
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
