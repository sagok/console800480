/**
  ******************************************************************************
  * @file    ts_calibration.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-March-2014
  * @brief   This example code shows how to calibrate the touchscreen.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Board_lcd.h"
#include "Board_ts.h"
#include "ADS7843_TS.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "DiscBrowser.h"

//#include "stm32f2xx_hal.h"
/** @addtogroup STM32F2xx_HAL_Applications
  * @{
  */

/** @addtogroup LCD_Paint
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TS_StateTypeDef  TS_State;
static uint8_t Calibration_Done = 0;
static int16_t  A1, A2, B1, B2;
static int16_t aPhysX[2], aPhysY[2], aLogX[2], aLogY[2];

uint16_t tcState = 0;

int16_t xPos[5], yPos[5], axc[2], ayc[2], bxc[2], byc[2];
/* Private function prototypes -----------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define CONVERTED_FRAME_BUFFER                   0x64000000
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//void Touchscreen_Calibration (void);
uint16_t Calibration_GetX(uint16_t x);
uint16_t Calibration_GetY(uint16_t y);
uint8_t  IsCalibrationDone(void);

static void TouchscreenCalibration_SetHint(void);
static void GetPhysValues(int16_t LogX, int16_t LogY, int16_t * pPhysX, int16_t * pPhysY) ;
static void WaitForPressedState(uint8_t Pressed) ;
/* Private functions ---------------------------------------------------------*/

uint16_t touchCalibration(void)
{
	static uint16_t flag, counter = 0;
	static FIL 	FFile;
	extern FIL LogFileOnSD;
	sFONT Font;
	char		byteread[255];
	uint8_t	error;

	counter++;
	if (counter > 6000)
	{
		touchSetCoef(11, -17, -15, 256);
		tcState = 9;
	}
	switch (tcState)
	{
	case 0:	// left top corner draw

		/* Clear the LCD */
		BSP_LCD_Clear(LCD_COLOR_WHITE);

		/* Set Touchscreen Demo description */
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

		scan_files("fonts");
		f_chdir("/fonts");											// сменим директорию
		   if(f_open(&FFile, "cour36b.FFN", FA_READ) != FR_OK)		//"Arial36B.FFN"
			{
				printf("[Dp] :not open the cour36b.FFN file\n");
				f_chdir("/");											// сменим директорию
				Write_to_logFile(&LogFileOnSD, (char *)"[Dp] :not open the cour36b.FFN file\n",sizeof("[Dp] :not open the cour36b.FFN file\n")-1);	// добавим в лог.
			} else {
				printf("[Dp] :open the cour36b.FFN file\n");
				error = f_gets(&byteread[0],255,&FFile);					// читаем строку
				Font.Width = (uint16_t)byteread[0]<<8 | (uint16_t)byteread[1];
				Font.Height = (uint16_t)byteread[2]<<8 | (uint16_t)byteread[3];
				Font.table =(uint8_t *)"cour36b.fbn";
				printf("Width: %u \n",Font.Width);
				printf("Height: %u \n",Font.Height);
				f_close(&FFile);
			}
		   f_open(&FFile, "cour36b.FBN", FA_READ);

		   BSP_LCD_DisplayStringAt(&FFile,BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2,  "Калибровка сенсорного экрана", LEFT_MODE);
		   BSP_LCD_DisplayStringAt(&FFile,BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2 + BSP_LCD_GetTextHeight(),(void *)"Нажмите в центр круга", CENTER_MODE);
		   BSP_LCD_DrawCircle(10,10,5);
			flag = 0;
		   if (getTouchState() == 0)
			tcState++;

		    f_close(&FFile);
		break;
	}

	return tcState;
}

/**
  * @brief  Performs the TS calibration
  * @param  None
  * @retval None
  */
void Touchscreen_Calibration (void)
{ 
  uint8_t status = 0;
  uint8_t i = 0;
  FIL FFile;
  sFONT Font;

  TouchscreenCalibration_SetHint();
  printf("TouchscreenCalibration_SetHint\n");

  status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  printf("BSP_TS_Init: %d\n",status);
  
  if (status != TS_OK)
  {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    OpenFontFile(&FFile,&Font);
    BSP_LCD_DisplayStringAt(&FFile,0, BSP_LCD_GetYSize()- 95,(void *)"ERROR", CENTER_MODE);
    BSP_LCD_DisplayStringAt(&FFile,0, BSP_LCD_GetYSize()- 80,(void *)"Touchscreen cannot be initialized", CENTER_MODE);
    CloseFontFile(&FFile);
  }
  
  while (1)
  {
    if (status == TS_OK)
    {
      aLogX[0] = 30;
      aLogY[0] = 30;
      aLogX[1] = BSP_LCD_GetXSize() - 30;
      aLogY[1] = BSP_LCD_GetYSize() - 30;

      printf("status = TS_OK,%d,%d,%d,%d\n",aLogX[0],aLogY[0],aLogX[1],aLogY[1]);

      for (i = 0; i < 2; i++) 
      {
        GetPhysValues(aLogX[i], aLogY[i], &aPhysX[i], &aPhysY[i]);
        printf("GetPhysValues,%d\n",i);

      }
      A1 = (1000 * ( aLogX[1] - aLogX[0]))/ ( aPhysX[1] - aPhysX[0]); 
      B1 = (1000 * aLogX[0]) - A1 * aPhysX[0]; 
      
      A2 = (1000 * ( aLogY[1] - aLogY[0]))/ ( aPhysY[1] - aPhysY[0]); 
      B2 = (1000 * aLogY[0]) - A2 * aPhysY[0]; 
      
      Calibration_Done = 1;
      printf("Calibration_Done\n");

      return;
    }
   
    HAL_Delay(5);
  }
}

/**
  * @brief  Display calibration hint
  * @param  None
  * @retval None
  */
static void TouchscreenCalibration_SetHint(void)
{
  FIL	Fontfile;
  sFONT Font;

  /* Clear the LCD */ 
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  
  /* Set Touchscreen Demo description */
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  OpenFontFile(&Fontfile,&Font);
//  BSP_LCD_SetFont((uint16_t *)&Font.table);		//DrawProp.FileName

  BSP_LCD_DisplayStringAt(&Fontfile, BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2-BSP_LCD_GetTextHeight(),(void *)"Калибровка", CENTER_MODE);

//  BSP_LCD_DisplayStringAt(&Fontfile, BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2-BSP_LCD_GetTextHeight(),(void *)"Перед использованием Touchscreen", CENTER_MODE);
//  BSP_LCD_DisplayStringAt(&Fontfile, BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2,(void *)"you need to calibrate it.", CENTER_MODE);
//  BSP_LCD_DisplayStringAt(&Fontfile, BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2+BSP_LCD_GetTextHeight(),(void *)"Press on the black circles", CENTER_MODE);
  CloseFontFile(&Fontfile);

}


/**
  * @brief  Get Physical position
  * @param  LogX : logical X position
  * @param  LogY : logical Y position
  * @param  pPhysX : Physical X position
  * @param  pPhysY : Physical Y position
  * @retval None
  */
static void GetPhysValues(int16_t LogX, int16_t LogY, int16_t * pPhysX, int16_t * pPhysY) 
{
  /* Draw the ring */
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillCircle(LogX, LogY, 5);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(LogX, LogY, 2);

  printf("[TS] I:GetPhysValues - WaitForPressedState.......\n");

  /* Wait until touch is pressed */
  WaitForPressedState(1);
  printf("[TS] I:GetPhysValues - WaitForPressedState(1). X=%d, Y=%d\n",TS_State.x,TS_State.y);

  BSP_TS_GetState(&TS_State);
  *pPhysX = TS_State.x;
  *pPhysY = TS_State.y; 
  
  printf("[TS] I:GetPhysValues - BSP_TS_GetState X=%d, Y=%d\n",TS_State.x,TS_State.y);

  /* Wait until touch is released */
  WaitForPressedState(0);
  printf("[TS] I:GetPhysValues - WaitForPressedState(0)\n");

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(LogX, LogY, 5);
}

/**
  * @brief  wait for touch detected
  * @param  Pressed: touch pressed. 
  * @retval None
  */
static void WaitForPressedState(uint8_t Pressed) 
{
  TS_StateTypeDef  State;
  
  do 
  {
    BSP_TS_GetState(&State);
    if (State.TouchDetected) printf("[TS] I: WaitForPressedState->BSP_TS_GetState, %d \n",State.TouchDetected);

    HAL_Delay(10);
    if (State.TouchDetected == Pressed) 
    {
      uint16_t TimeStart = HAL_GetTick();
      do {
        BSP_TS_GetState(&State);      
        HAL_Delay(10);
        if (State.TouchDetected != Pressed) 
        {
          break;
        } else if ((HAL_GetTick() - 100) > TimeStart) 
        {
          return;
        }
      } while (1);
    }
  } while (1);
}

/**
  * @brief  Calibrate X position
  * @param  x : X position
  * @retval calibrated x
  */
uint16_t Calibration_GetX(uint16_t x)
{
  return (((A1 * x) + B1)/1000);
}

/**
  * @brief  Calibrate Y position
  * @param  y : Y position
  * @retval calibrated y
  */
uint16_t Calibration_GetY(uint16_t y)
{
  return (((A2 * y) + B2)/1000);
}

/**check if the TS is calibrated
  * @param  None
* @retval calibration state (1 : calibrated / 0: no)
  */
uint8_t IsCalibrationDone(void)
{
  return (Calibration_Done);
}

/**
  * @}
  */ 
  
/**
  * @}
  */
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
