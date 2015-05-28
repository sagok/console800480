/**
  ******************************************************************************
  * @file    ili9320.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2014
  * @brief   This file includes the LCD driver for ILI9320 LCD.
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
#include "ili9320.h"
#include "stm32f2xx_hal.h"
#include "Display.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup ili9320
  * @brief     This file provides a set of functions needed to drive the 
  *            ILI9320 LCD.
  * @{
  */

/** @defgroup ILI9320_Private_TypesDefinitions
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup ILI9320_Private_Defines
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup ILI9320_Private_Macros
  * @{
  */
     
/**
  * @}
  */  

/** @defgroup ILI9320_Private_Variables
  * @{
  */ 
LCD_DrvTypeDef   ili9320_drv = 
{
  GFTM070_Init,//ili9320_Init,
  ili9320_ReadID,
  GFTM070_DisplayOn,
  ili9320_DisplayOff,
  GFTM070_SetCursor,
  GFTM070_WritePixel,
  GFTM070_WriteNextWindowPixel,
  ili9320_ReadPixel,
  GFTM070_SetDisplayWindow,
  GFTM070_DrawHLine,
  GFTM070_DrawVLine,
  ili9320_GetLcdPixelWidth,
  ili9320_GetLcdPixelHeight,
  GFTM070_DrawBitmap,
  GFTM070_DrawRGBImage,
  GFTM070_DrawHDataLine,
};

TFT_DrvTypeDef   GFTM070SoftMode_drv =
{
GFTM070_Init,
GFTM070_WriteCommand,
};

static uint8_t Is_ili9320_Initialized = 0;
static uint8_t Is_GFTM070_Initialized = 0;

/**
  * @}
  */ 
  
/** @defgroup ILI9320_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup ILI9320_Private_Functions
  * @{
  */   
/**
  * @brief  Initialise the GFTM070 LCD Component.
  * @param  None
  * @retval None
  */
void GFTM070_Init(void){
	uint32_t	i;

	if(Is_GFTM070_Initialized == 0)
	{

		Is_GFTM070_Initialized = 1;
	    /* Initialise low level bus layer -----------------------------------*/
		GFTM070_IO_Init();

//		HAL_GPIO_WritePin(IOTFT_NRESET_GPIO_PORT, IOTFT_NRESET_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(IOTFT_NRESET_GPIO_PORT, IOTFT_NRESET_PIN,GPIO_PIN_RESET);
		vTaskDelay(10);
		HAL_GPIO_WritePin(IOTFT_NRESET_GPIO_PORT, IOTFT_NRESET_PIN,GPIO_PIN_SET);
		vTaskDelay(10);

		/* Start Initial Sequence -------------------------------------------*/

		GFTM070_WriteCommand(SoftwareReset);				// Command: Soft reset

		GFTM070_WriteCommand(0xE2);							// Set PLL: VCO (> 250MHz) = OSC x (N + 1)
		GFTM070_WriteData(MULTIPLIER_N);					// Multiplier N, VCO = 360MHz
		GFTM070_WriteData(DIVIDER_M);						// Divider M, PLL = 120MHz (Standard)
		GFTM070_WriteData(0x54);							// dummy write, no meaning. PLL = VCO / (M + 1)

		GFTM070_WriteCommand(0xE0);							// Command: Set PLL
		GFTM070_WriteData(0x01);							// Turn on PLL
		GFTM070_WriteData(0x00);							// Turn on PLL

		GFTM070_WriteCommand(0xE0);							// Command: Set PLL
		GFTM070_WriteData(0x03);							// Switch the clock source to PLL

		GFTM070_WriteCommand(SoftwareReset);				// Command: Soft reset
		// Delay 3000
		vTaskDelay(100);

		GFTM070_WriteCommand(0xE6);							// pclk = pll freq * (setting + 1) / 0x100000		//Set LSHIFT frequency
		GFTM070_WriteData((LCDC_FPR&0x000F0000)>>16); 		// Remark: 0x100000 must be divided by (setting+1), otherwise, pclk may oscilate
		GFTM070_WriteData((LCDC_FPR&0x0000FF00)>>8); 		// pclk = 5MHz
		GFTM070_WriteData((LCDC_FPR&0x000000FF)); 			// refresh rate = 5MHz / (336 * 244) = 60.98Hz

		GFTM070_WriteCommand(SetLCDMode); 					// display period		//Set LCD mode / pad strength
		GFTM070_WriteData(0x0B); 							// 7.0:0B  5.7:0C		// [B5:0] - 18Bit [B2:1] LSHIFT data latch falling edge [B1:0] LLINE active low [B0:0] - LFRAME active low
		GFTM070_WriteData(0x00);
		GFTM070_WriteData(((LCD_WIDTH-1)&0xFF00)>>8); 		// 0x13F = 320 - 1 (Horizontal)
		GFTM070_WriteData((LCD_WIDTH-1)&0x00FF);
		GFTM070_WriteData(((LCD_HEIGHT-1)&0xFF00)>>8); 		// 0x0ef = 240 - 1 (Vertical)
		GFTM070_WriteData((LCD_HEIGHT-1)&0x00FF);

		GFTM070_WriteCommand(SetHorizontalPeriod); 			// hsync
		GFTM070_WriteData(((LCD_HORI_CYCLE-1)&0xFF00)>>8); 	// ht LCD_TOTAL_WIDTH - 1
		GFTM070_WriteData((LCD_HORI_CYCLE-1)&0x00FF);
		GFTM070_WriteData((LCD_HORI_START&0x0700)>>8);
		GFTM070_WriteData((LCD_HORI_START)&0x00FF); 		// Horizontal Start 8 pclks
		GFTM070_WriteData(LCD_HORI_PULSE_WIDTH - 1); 		// pulse width = setting + 1 clock = 2 clock
		GFTM070_WriteData(0x00);
		GFTM070_WriteData(0x00);

		GFTM070_WriteCommand(SetVerticalPeriod); 			// vsync
		GFTM070_WriteData(((LCD_VERT_CYCLE-1)&0xFF00)>>8); 	// vt 244-1
		GFTM070_WriteData((LCD_VERT_CYCLE-1)&0x00FF);
		GFTM070_WriteData((LCD_VERT_START&0x0700)>>8);
		GFTM070_WriteData((LCD_VERT_START)&0x00FF); 		// Vertical Start Position: 2 line
		GFTM070_WriteData(LCD_VERT_PULSE_WIDTH - 1); 		// pulse width = setting + 1 line = 2 line
		GFTM070_WriteData(0x00);
		GFTM070_WriteData(0x00);

		GFTM070_WriteCommand(0xba); 						// set GPIO value to all high first.
		GFTM070_WriteData(0x0f);

		GFTM070_WriteCommand(0xb8); 						// config gpio
		GFTM070_WriteData(0x0f);
		GFTM070_WriteData(0x01);

		for(i = 0 ; i < 200000; i++);
		vTaskDelay(10);

		GFTM070_WriteCommand(SetDisplayOn); 				// display on

		for(i = 0 ; i < 200000; i++);
		vTaskDelay(10);

		GFTM070_WriteCommand(ExitSleepMode);

		//Set Column Address
		GFTM070_WriteCommand(SETColumnAddress);
		GFTM070_WriteData(0x00);
		GFTM070_WriteData(0x00);
		GFTM070_WriteData(((LCD_WIDTH-1) & 0xFF00)>>8);
		GFTM070_WriteData(((LCD_WIDTH-1) & 0x00FF));

		//Set Page Address
		GFTM070_WriteCommand(SETPageAddress);
		GFTM070_WriteData(0x00);
		GFTM070_WriteData(0x00);
		GFTM070_WriteData(((LCD_HEIGHT-1) & 0xFF00)>>8);
		GFTM070_WriteData(((LCD_HEIGHT-1) & 0x00FF)); //239

		//Set the read order from frame buffer to the display panel
		GFTM070_WriteCommand(SetAddressMode);
		GFTM070_WriteData(_AddressMode);				//0x00

		GFTM070_WriteCommand(0xbe);		// pwm config ;
		GFTM070_WriteData(0x08);		// pwmclk divide;
		GFTM070_WriteData(0xff);		// duty cycle;
		GFTM070_WriteData(0x01);		// 0x9=dbc control, 0x1=host control;

		GFTM070_WriteCommand(0xf0);
		GFTM070_WriteData(0x03); 		// 0x03 - 16bit 565 mode

//		GFTM070_WriteCommand(EnterInvertMode);


	}
}

/************************************************************************************
 * GFTM070_WriteCommand
*************************************************************************************/
void GFTM070_WriteCommand(uint16_t LCD_Cmd)
{
  LCD_IO_WriteReg(LCD_Cmd);
}
/************************************************************************************
 * GFTM070_WriteCommand
*************************************************************************************/
void GFTM070_ReadCommand(uint16_t LCD_Cmd)
{
  LCD_IO_WriteReg(LCD_Cmd);
}
/************************************************************************************
 * GFTM070_WriteData
*************************************************************************************/
void GFTM070_WriteData(uint16_t LCD_Data)
{
  LCD_IO_WriteData(LCD_Data);
}
/************************************************************************************
 * GFTM070_ReadData
*************************************************************************************/
uint16_t GFTM070_ReadData(void)
{
	return LCD_IO_ReadData();
}

/************************************************************************************
*************************************************************************************
*************************************************************************************
*************************************************************************************
*************************************************************************************
*************************************************************************************
*************************************************************************************
*************************************************************************************/
/**
  * @brief  Initialise the ILI9320 LCD Component.
  * @param  None
  * @retval None
  */
void ili9320_Init(void)
{  

}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void GFTM070_DisplayOn(void)
{
uint32_t	i;

  /* Power On sequence ---------------------------------------------------------*/
 
	/* Display On */
	GFTM070_WriteCommand(SetDisplayOn); /* 262K color and display ON */
	for(i = 0 ; i < 200000; i++);

	GFTM070_WriteCommand(ExitSleepMode);

}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9320_DisplayOff(void)
{

}

/**
  * @brief  Get the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t ili9320_GetLcdPixelWidth(void)
{
 return (uint16_t)ILI9320_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Get the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t ili9320_GetLcdPixelHeight(void)
{
 return (uint16_t)ILI9320_LCD_PIXEL_HEIGHT;
}

/**
  * @brief  Get the ILI9320 ID.
  * @param  None
  * @retval The ILI9320 ID 
  */
uint16_t ili9320_ReadID(void)
{
  if(Is_ili9320_Initialized == 0)
  {
    ili9320_Init();  
  }
  return (ili9320_ReadReg(0x00));
}

/**
  * @brief  Set Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void ili9320_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  ili9320_WriteReg(LCD_REG_32, Ypos);
  ili9320_WriteReg(LCD_REG_33, (ILI9320_LCD_PIXEL_WIDTH - 1 - Xpos));      
}

void GFTM070_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	//Set Column Address
	GFTM070_WriteCommand(SETColumnAddress);
	GFTM070_WriteData(((Xpos) & 0xFF00)>>8);
	GFTM070_WriteData(((Xpos) & 0x00FF));
	GFTM070_WriteData(((Xpos) & 0xFF00)>>8);
	GFTM070_WriteData(((Xpos) & 0x00FF));

	//Set Page Address
	GFTM070_WriteCommand(SETPageAddress);
	GFTM070_WriteData(((Ypos) & 0xFF00)>>8);
	GFTM070_WriteData(((Ypos) & 0x00FF));
	GFTM070_WriteData(((Ypos) & 0xFF00)>>8);
	GFTM070_WriteData(((Ypos) & 0x00FF));

	//Set the read order from frame buffer to the display panel
	GFTM070_WriteCommand(SetAddressMode);
	GFTM070_WriteData(_AddressMode);				//0x00

	GFTM070_WriteCommand(WriteMemoryStart);

}
/**
  * @brief  Write pixel.   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGB_Code: the RGB pixel color
  * @retval None
  */

void ili9320_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
  /* Set Cursor */
  ili9320_SetCursor(Xpos, Ypos);
  
  /* Prepare to write GRAM */
  LCD_IO_WriteReg(LCD_REG_34);

  /* Write 16-bit GRAM Reg */
  LCD_IO_WriteData(RGB_Code);
}

void GFTM070_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
  /* Set Cursor */
  GFTM070_SetCursor(Xpos, Ypos);
  /* Write 16-bit GRAM Reg */
  GFTM070_WriteData(RGB_Code);
 // LCD_IO_WriteData(RGB_Code);
}
/**
  * @brief  WriteNextWindowPixel.
  * @param  None
  * @retval the RGB pixel color
  */
void 		GFTM070_WriteNextWindowPixel(uint16_t RGB_Code)
{
	  GFTM070_WriteData(RGB_Code);
}
/**
  * @brief  Read pixel.
  * @param  None
  * @retval the RGB pixel color
  */
uint16_t ili9320_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  /* Set Cursor */
  ili9320_SetCursor(Xpos, Ypos);
  
  /* Prepare to write GRAM */
  LCD_IO_WriteReg(LCD_REG_34);
  
  /* Dummy read */
  LCD_IO_ReadData();
  
  /* Read 16-bit Reg */
  return (LCD_IO_ReadData());
}

/**
  * @brief  Writes to the selected LCD register.
  * @param  LCD_Reg:      address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
void ili9320_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
  LCD_IO_WriteReg(LCD_Reg);
  
  /* Write 16-bit GRAM Reg */
  LCD_IO_WriteData(LCD_RegValue);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  LCD_Reg: address of the selected register.
  * @retval LCD Register Value.
  */
uint16_t ili9320_ReadReg(uint8_t LCD_Reg)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD_IO_WriteReg(LCD_Reg);
  
  /* Read 16-bit Reg */
  return (LCD_IO_ReadData());
}


/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void ili9320_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{

  /* Horizontal GRAM Start Address */
  ili9320_WriteReg(LCD_REG_80, (Ypos));
  /* Horizontal GRAM End Address */
  ili9320_WriteReg(LCD_REG_81, (Ypos + Height - 1));
  
  /* Vertical GRAM Start Address */
  ili9320_WriteReg(LCD_REG_82, ILI9320_LCD_PIXEL_WIDTH - Xpos - Width);
  /* Vertical GRAM End Address */
  ili9320_WriteReg(LCD_REG_83, ILI9320_LCD_PIXEL_WIDTH - Xpos - 1);  
}

void GFTM070_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{

	//Set Column Address
	GFTM070_WriteCommand(SETColumnAddress);
	GFTM070_WriteData(((Xpos) & 0xFF00)>>8);
	GFTM070_WriteData(((Xpos) & 0x00FF));
	GFTM070_WriteData(((Xpos + Width -1) & 0xFF00)>>8);
	GFTM070_WriteData(((Xpos + Width -1) & 0x00FF));

	//Set Page Address
	GFTM070_WriteCommand(SETPageAddress);
	GFTM070_WriteData(((Ypos) & 0xFF00)>>8);
	GFTM070_WriteData(((Ypos) & 0x00FF));
	GFTM070_WriteData(((Ypos + Height - 1) & 0xFF00)>>8);
	GFTM070_WriteData(((Ypos + Height - 1) & 0x00FF));

	//Set the read order from frame buffer to the display panel
	GFTM070_WriteCommand(SetAddressMode);
	GFTM070_WriteData(_AddressModeForWindowsChar);				//0x00

	GFTM070_WriteCommand(WriteMemoryStart);//WriteMemoryStart

}

/**
  * @brief  Draw vertical line.
  * @param  RGB_Code: Specifies the RGB color   
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.  
  * @retval None
  */
void ili9320_DrawHLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t i = 0;
  
  /* Set Cursor */
  ili9320_SetCursor(Xpos, Ypos);
  /* Prepare to write GRAM */
  LCD_IO_WriteReg(LCD_REG_34);

  for(i = 0; i < Length; i++)
  {
    /* Write 16-bit GRAM Reg */
    LCD_IO_WriteData(RGB_Code);
  }  
}

void GFTM070_DrawHLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t i = 0;

  /* Set Cursor */
  GFTM070_SetDisplayWindow(Xpos, Ypos ,Xpos+Length,Ypos);

  for(i = 0; i < Length; i++)
  {
    /* Write 16-bit GRAM Reg */
    LCD_IO_WriteData(RGB_Code);
  }
}

/**
  * @brief  Draw horizontal line from data raw.
  * @param  RGB_Code: Specifies the RGB color
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param *pdata:	  pointer to data block for draw
  * @param  Length:   specifies the Line length.
  * @retval None
  */
void     	GFTM070_DrawHDataLine(uint16_t RGB_Code, uint16_t RGB_CodeBG , uint16_t Xpos, uint16_t Ypos,uint8_t *pdata, uint16_t Length)
{
	  uint16_t i = 0;
	  uint8_t dataByte, cntByte=0, line;

	  dataByte = (uint8_t)(Length>>8);		//число целых байт
	  line =  *pdata;

	  /* Set Cursor */
	  GFTM070_SetDisplayWindow(Xpos, Ypos ,Xpos+Length,Ypos);

	  for(i = 0; i < Length; i++)
	  {
		cntByte++; if (cntByte > 7) {					// следующий байт
			cntByte = 0;
			pdata ++;
	    	line =  *pdata;
		}

//	    LCD_IO_WriteData(*(volatile uint16_t *)pdata);

	    /* Write 16-bit GRAM Reg */
		if(line & (1 << (7-cntByte)))  	LCD_IO_WriteData(RGB_Code);
	    else  							LCD_IO_WriteData(RGB_CodeBG);
	  }
}
/**
  * @brief  Draw vertical line.
  * @param  RGB_Code: Specifies the RGB color    
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.  
  * @retval None
  */
void ili9320_DrawVLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t i = 0;

  /* set GRAM write direction and BGR = 1 */
  /* I/D=00 (Horizontal : increment, Vertical : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
  ili9320_WriteReg(LCD_REG_3, 0x1010);
  
  /* Set Cursor */
  ili9320_SetCursor(Xpos, Ypos);
  
  /* Prepare to write GRAM */
  LCD_IO_WriteReg(LCD_REG_34);

  for(i = 0; i < Length; i++)
  {
    /* Write 16-bit GRAM Reg */
    LCD_IO_WriteData(RGB_Code);
  }
  
  /* set GRAM write direction and BGR = 1 */
  /* I/D=00 (Horizontal : increment, Vertical : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
  ili9320_WriteReg(LCD_REG_3, 0x1018);  
}

void GFTM070_DrawVLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t i = 0;

 /*		// не работает
  // Set Cursor
  GFTM070_SetDisplayWindow(Xpos, Ypos ,Xpos,Ypos+Length);
  for(i = 0; i < Length; i++)
  {
    // Write 16-bit GRAM Reg
    LCD_IO_WriteData(RGB_Code);
  }
*/

 // вариант очень медленный.
   for(i = Ypos; i < (Ypos+Length); i++)
  {
  GFTM070_WritePixel(Xpos,i,RGB_Code);
  }

}
/**
  * @brief  Displays a bitmap picture..
  * @param  BmpAddress: Bmp picture address.
  * @param  Xpos:  Bmp X position in the LCD
  * @param  Ypos:  Bmp Y position in the LCD    
  * @retval None
  */
void ili9320_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, size = 0;
  /* Read bitmap size */
  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  size = (size - index)/2;
  pbmp += index;
  /* Set GRAM write direction and BGR = 1 */
  /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
  ili9320_WriteReg(LCD_REG_3, 0x1008);

  /* Set Cursor */
  ili9320_SetCursor(Xpos, Ypos);  
  
  /* Prepare to write GRAM */
  LCD_IO_WriteReg(LCD_REG_34);
 
  for(index = 0; index < size; index++)
  {
    /* Write 16-bit GRAM Reg */
    LCD_IO_WriteData(*(volatile uint16_t *)pbmp);
    pbmp += 2;
  }

}

void GFTM070_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
 {
    uint32_t index = 0, size = 0;
    /* Read bitmap size */
    size = *(volatile uint16_t *) (pbmp + 2);
    size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
    /* Get bitmap data address offset */
    index = *(volatile uint16_t *) (pbmp + 10);
    index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
    size = (size - index)/2;
    pbmp += index;
    /* Set GRAM write direction and BGR = 1 */
    /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
//    ili9320_WriteReg(LCD_REG_3, 0x1008);

    /* Set Cursor */
  //  GFTM070_SetCursor(Xpos, Ypos);
//    GFTM070_SetDisplayWindow(Xpos, Ypos,);
    /* Prepare to write GRAM */
 //   LCD_IO_WriteReg(LCD_REG_34);

    for(index = 0; index < size; index++)
    {
      /* Write 16-bit GRAM Reg */
      LCD_IO_WriteData(*(volatile uint16_t *)pbmp);
      pbmp += 2;
    }
  /* Set GRAM write direction and BGR = 1 */
  /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
  /* AM = 1 (address is updated in vertical writing direction) */
//  ili9320_WriteReg(LCD_REG_3, 0x1018);
}

/**
  * @brief  Displays picture..
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the LCD
  * @param  Ypos:  Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  */
void ili9320_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  uint32_t index = 0, size = 0;

  size = (Xsize * Ysize);

  /* Set Cursor */
  ili9320_SetCursor(Xpos, Ypos);  
  
  /* Prepare to write GRAM */
  LCD_IO_WriteReg(LCD_REG_34);
 
  for(index = 0; index < size; index++)
  {
    /* Write 16-bit GRAM Reg */
    LCD_IO_WriteData(*(volatile uint16_t *)pdata);
    pdata += 2;
  }
}

void GFTM070_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  uint32_t index = 0, size = 0;

  size = (Xsize * Ysize);

  /* Set Cursor */
//  GFTM070_SetCursor(Xpos, Ypos);
  GFTM070_SetDisplayWindow(Xpos, Ypos, Xsize,Ysize);

  /* Prepare to write GRAM */
//  LCD_IO_WriteReg(LCD_REG_34);

  for(index = 0; index < size; index++)
  {
    /* Write 16-bit GRAM Reg */
    LCD_IO_WriteData(*(volatile uint16_t *)pdata);
    pdata += 2;
  }
}
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
