/**
  ******************************************************************************
  * @file    ili9320.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2014
  * @brief   This file contains all the functions prototypes for the ili9320.c
  *          driver.
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
#ifndef __ILI9320_H
#define __ILI9320_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "lcd.h"

 #define LCD_WIDTH			800//800
 #define LCD_HEIGHT			480//480


 //Horizontal Cycle = Horizontal Pulse Width + Horizontal Back Porch + Horizontal Display Period + Horizontal Front Porch
 //Horizontal Start Position = Horizontal Pulse Width + Horizontal Back Porch

 #define LCD_HORI_FRONT_PORCH		120//((80)&0x7f)//120			120
 #define LCD_HORI_BACK_PORCH		0//((5)&0xff)//0				0
 #define LCD_HORI_PULSE_WIDTH		20//((30)&0xff)//				20

 //Vertical Cycle = Vertical Pulse Width + Vertical Back Porch + Vertical Display Period + Vertical Front Porch
 //Vertical Start Position = Vertical Pulse Width + Vertical Back Porch

 #define LCD_VERT_FRONT_PORCH		5//((30)&0xff)//				5
 #define LCD_VERT_BACK_PORCH		0//((30)&0xff)//				0
 #define LCD_VERT_PULSE_WIDTH		20//((15)&0x3f)//20				20

 #define REFRESH_RATE				70	//70 Hz

 #define LCD_HORI_CYCLE				(LCD_HORI_PULSE_WIDTH + LCD_HORI_BACK_PORCH + LCD_WIDTH + LCD_HORI_FRONT_PORCH)
 #define LCD_VERT_CYCLE				(LCD_VERT_PULSE_WIDTH + LCD_VERT_BACK_PORCH + LCD_HEIGHT + LCD_VERT_FRONT_PORCH)

 #define LCD_HORI_START				(LCD_HORI_PULSE_WIDTH + LCD_HORI_BACK_PORCH)
 #define LCD_VERT_START				(LCD_VERT_PULSE_WIDTH + LCD_VERT_BACK_PORCH)

 #define PIXEL_CLOCK					(unsigned int)(LCD_HORI_CYCLE * LCD_VERT_CYCLE * REFRESH_RATE)

 #define OSC_FREQ					4000000			// 4000000Hz
 #define MULTIPLIER_N				115				// 115
 #define DIVIDER_M					4				// 4

 #define VCO_FREQ					(unsigned int)(OSC_FREQ * (MULTIPLIER_N + 1))			//Hz, VCO Frequency is required to be larger than 250MHz
 #define PLL_FREQ					(unsigned int)(VCO_FREQ / (DIVIDER_M + 1))			//Hz
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 //  128					   8192
 #define CAL						(unsigned int)(((float)((float)PIXEL_CLOCK * 128) / (float)PLL_FREQ) * 8192)

 #define LCDC_FPR					(unsigned int)(CAL - 1)



#define  ILI9320_ID    0xFFFF//0x9320
   
/** 
  * @brief  ILI9320 Size  
  */  
#define  ILI9320_LCD_PIXEL_WIDTH    ((uint16_t)LCD_WIDTH)//800
#define  ILI9320_LCD_PIXEL_HEIGHT   ((uint16_t)LCD_HEIGHT)//480
   

/**
  * @brief  GFTM070 Registers
  */
//       Command									 Parameters
#define NoOperatoin             	0x00			//None
#define SoftwareReset             	0x01			//None
#define GetPowerMode             	0x0A			//1
#define GetAddressMode             	0x0B			//1
#define GetPixeFormat             	0x0C			//1
#define GetDisplayMode             	0x0D			//1
#define GetSignalMode             	0x0E			//1
#define EnterSleepMode             	0x10			//None
#define ExitSleepMode             	0x11			//None
#define EnterPartialMode           	0x12			//None
#define EnterNormalMode           	0x13			//None
#define ExitInvertMode           	0x20			//None
#define EnterInvertMode           	0x21			//None
#define SetGammaCurve           	0x26			//1
#define SetDisplayOff           	0x28			//None
#define SetDisplayOn	           	0x29			//None
#define SETColumnAddress           	0x2A			//4
#define SETPageAddress           	0x2B			//4
#define WriteMemoryStart           	0x2C			//None
#define ReadMemoryStart           	0x2E			//None
#define SetPartialArea           	0x30			//4
#define SetScrollArea           	0x33			//6
#define SetTearOff		           	0x34			//None
#define SetTearOn		           	0x35			//1
#define SetAddressMode	           	0x36			//1
#define SetScrollStart	           	0x37			//2
#define ExitIdleMode	           	0x38			//None
#define EnterIdleMode	           	0x39			//None
#define SetPixelFormat	           	0x3A			//1
#define WriteMemoryContinue        	0x3C			//None
#define ReadMemoryContinue        	0x3E			//None
#define SetTearScanline	        	0x44			//2
#define GetTearScanline	        	0x45			//2
#define ReadDDB			        	0xA1			//5
#define SetLCDMode		        	0xB0			//7
#define GetLCDMode		        	0xB1			//7
#define SetHorizontalPeriod        	0xB4			//8
#define GetHorizontalPeriod        	0xB5			//8
#define SetVerticalPeriod        	0xB6			//7
#define GetVerticalPeriod        	0xB7			//7


#define _AddressMode	           	0x02			// 0x01 mirrir Vert  0x02 mirror Hor
#define _AddressModeForWindowsChar 	0x03			// 0x01 mirrir Vert  0x02 mirror Hor


/** 
  * @brief  ILI9320 Registers  
  */ 
#define LCD_REG_0             0x00
#define LCD_REG_1             0x01
#define LCD_REG_2             0x02
#define LCD_REG_3             0x03
#define LCD_REG_4             0x04
#define LCD_REG_5             0x05
#define LCD_REG_6             0x06
#define LCD_REG_7             0x07
#define LCD_REG_8             0x08
#define LCD_REG_9             0x09
#define LCD_REG_10            0x0A
#define LCD_REG_12            0x0C
#define LCD_REG_13            0x0D
#define LCD_REG_14            0x0E
#define LCD_REG_15            0x0F
#define LCD_REG_16            0x10
#define LCD_REG_17            0x11
#define LCD_REG_18            0x12
#define LCD_REG_19            0x13
#define LCD_REG_20            0x14
#define LCD_REG_21            0x15
#define LCD_REG_22            0x16
#define LCD_REG_23            0x17
#define LCD_REG_24            0x18
#define LCD_REG_25            0x19
#define LCD_REG_26            0x1A
#define LCD_REG_27            0x1B
#define LCD_REG_28            0x1C
#define LCD_REG_29            0x1D
#define LCD_REG_30            0x1E
#define LCD_REG_31            0x1F
#define LCD_REG_32            0x20
#define LCD_REG_33            0x21
#define LCD_REG_34            0x22
#define LCD_REG_36            0x24
#define LCD_REG_37            0x25
#define LCD_REG_40            0x28
#define LCD_REG_41            0x29
#define LCD_REG_43            0x2B
#define LCD_REG_45            0x2D
#define LCD_REG_48            0x30
#define LCD_REG_49            0x31
#define LCD_REG_50            0x32
#define LCD_REG_51            0x33
#define LCD_REG_52            0x34
#define LCD_REG_53            0x35
#define LCD_REG_54            0x36
#define LCD_REG_55            0x37
#define LCD_REG_56            0x38
#define LCD_REG_57            0x39
#define LCD_REG_58            0x3A
#define LCD_REG_59            0x3B
#define LCD_REG_60            0x3C
#define LCD_REG_61            0x3D
#define LCD_REG_62            0x3E
#define LCD_REG_63            0x3F
#define LCD_REG_64            0x40
#define LCD_REG_65            0x41
#define LCD_REG_66            0x42
#define LCD_REG_67            0x43
#define LCD_REG_68            0x44
#define LCD_REG_69            0x45
#define LCD_REG_70            0x46
#define LCD_REG_71            0x47
#define LCD_REG_72            0x48
#define LCD_REG_73            0x49
#define LCD_REG_74            0x4A
#define LCD_REG_75            0x4B
#define LCD_REG_76            0x4C
#define LCD_REG_77            0x4D
#define LCD_REG_78            0x4E
#define LCD_REG_79            0x4F
#define LCD_REG_80            0x50
#define LCD_REG_81            0x51
#define LCD_REG_82            0x52
#define LCD_REG_83            0x53
#define LCD_REG_96            0x60
#define LCD_REG_97            0x61
#define LCD_REG_106           0x6A
#define LCD_REG_118           0x76
#define LCD_REG_128           0x80
#define LCD_REG_129           0x81
#define LCD_REG_130           0x82
#define LCD_REG_131           0x83
#define LCD_REG_132           0x84
#define LCD_REG_133           0x85
#define LCD_REG_134           0x86
#define LCD_REG_135           0x87
#define LCD_REG_136           0x88
#define LCD_REG_137           0x89
#define LCD_REG_139           0x8B
#define LCD_REG_140           0x8C
#define LCD_REG_141           0x8D
#define LCD_REG_143           0x8F
#define LCD_REG_144           0x90
#define LCD_REG_145           0x91
#define LCD_REG_146           0x92
#define LCD_REG_147           0x93
#define LCD_REG_148           0x94
#define LCD_REG_149           0x95
#define LCD_REG_150           0x96
#define LCD_REG_151           0x97
#define LCD_REG_152           0x98
#define LCD_REG_153           0x99
#define LCD_REG_154           0x9A
#define LCD_REG_157           0x9D
#define LCD_REG_192           0xC0
#define LCD_REG_193           0xC1
#define LCD_REG_229           0xE5
/**
  * @}
  */
  
 /** @defgroup GFTM070_Exported_Functions
   * @{
   */
 void     	GFTM070_Init(void);
 void 		GFTM070_WriteCommand(uint16_t LCD_Cmd);
 void 		GFTM070_WriteData(uint16_t LCD_Data);
uint16_t 	GFTM070_ReadData(void);

 void 		GFTM070_DisplayOn(void);
 void 		GFTM070_SetCursor(uint16_t Xpos, uint16_t Ypos);
 void     	GFTM070_DrawHLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
 void 		GFTM070_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code);
 void 		GFTM070_WriteNextWindowPixel(uint16_t RGB_Code);
 void 		GFTM070_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
 void 		GFTM070_DrawVLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
 void 		GFTM070_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);
 void 		GFTM070_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);
 void     	GFTM070_DrawHDataLine(uint16_t RGB_Code, uint16_t RGB_CodeBG , uint16_t Xpos, uint16_t Ypos,uint8_t *pdata, uint16_t Length);

/** @defgroup ILI9320_Exported_Functions
  * @{
  */ 
void     ili9320_Init(void);
uint16_t ili9320_ReadID(void);
void     ili9320_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
uint16_t ili9320_ReadReg(uint8_t LCD_Reg);

void     ili9320_DisplayOn(void);
void     ili9320_DisplayOff(void);
void     ili9320_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     ili9320_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code);
uint16_t ili9320_ReadPixel(uint16_t Xpos, uint16_t Ypos);

void     ili9320_DrawHLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9320_DrawVLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9320_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);
void     ili9320_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);

void     ili9320_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);


uint16_t ili9320_GetLcdPixelWidth(void);
uint16_t ili9320_GetLcdPixelHeight(void);

/* LCD driver structure */
extern LCD_DrvTypeDef   ili9320_drv;

/* TFT driver structure */
extern TFT_DrvTypeDef   GFTM070SoftMode_drv;

/* LCD IO functions */
void     LCD_IO_Init(void);
void     LCD_IO_WriteData(uint16_t RegValue);
void     LCD_IO_WriteReg(uint16_t Reg);
uint16_t LCD_IO_ReadData(void);

/* TFT IO functions */
void     GFTM070_IO_Init(void);


/**
  * @}
  */ 
      
#ifdef __cplusplus
}
#endif

#endif /* __ILI9320_H */

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
