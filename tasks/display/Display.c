/* Standard includes. */
#include <stdint.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Tasks


#include "main.h"
#include "board.h"
#include "Display.h"
#include "Board_lcd.h"
#include "Board_ts.h"
#include "LEDs.h"


#include "color.h"
#include "save.h"

/* FatFs includes component */
#include "ff_gen_drv.h"

#include "DiscBrowser.h"

#include "menu.h"


//#include "logo.h"

portTickType 			xLastWakeTime;


#define  BL_PERIOD_VALUE       1000-1  		/* Period Value  */
uint16_t  BL_PULSE4 = 500;

static uint32_t Radius = 2;
static uint32_t x = 0, y = 0;

TS_StateTypeDef  TS_State;

/* Timer handler declaration */
TIM_HandleTypeDef    	TimHandle;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef 		sConfig;
/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

//extern uint8_t 		DisplayBuf[64];

DIR fontsDir;


sFONT Font;

/************************************************************************************
 * глобальные переменные
*************************************************************************************/
extern uint8_t aShowTime[50];
extern uint8_t aShowDate[50];

uint8_t		byteForwrite[1];
char		byteread[255];

extern FIL LogFileOnSD;

extern	char SDPath[4];
FIL 	FontfntFile, FontpngFile;     					// файл на карточке
FIL 	FontBinFile;     					// файл на карточке

FIL		FFile;

//TCHAR 	BuffFntRead[255];


void IOTFT_Init(IOLCD_TypeDef IOLCDPin);
void IOTFT_AllPortsInit(void);
int8_t TestAllLED(void);

void IOTFT_TS_Init(IOLCD_TypeDef IOLCDPin,uint32_t Mode);

void BLTFT_Init(IOLCD_TypeDef IOLCDPin,uint32_t AlternateFunction);
void LCD_BLInit(void);
void Delay_uS(uint16_t Delayus);
uint8_t SetBLBrightnessPercent(uint8_t brightnessPercent);

static void FSMC_BANK1_MspInit(void);
static void FSMC_BANK1_Init(void);
static void FSMC_BANK1_WriteData(uint16_t Data);
static void FSMC_BANK1_WriteReg(uint16_t Reg);
static uint16_t FSMC_BANK1_ReadData(void);

void LCD_IO_WriteData(uint16_t Data);
void LCD_IO_WriteReg(uint16_t Reg);
uint16_t LCD_IO_ReadData(void);

static void Draw_Menu(void);
static void GetPosition(void);
static void Update_Size(uint8_t size);
static void Update_Color(void);
static void Error_Handler(void);

uint8_t ReadFontFile(fnt_struct FNTFileName);
uint8_t ReadPNGtFile(fnt_struct FNTFileName);
uint16_t GetCharFromFontFile(FIL* fp);

uint32_t png_read_chunk_header(HDRCHUNK* buff);
void png_check_chunk_name(uint32_t chunk_name);
uint8_t Str_to_8Hex(uint8_t in1,uint8_t in2);

fnt_struct	fontFile;

/** @defgroup STM322xG_EVAL_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */
typedef struct
{
  __IO uint16_t REG;
  __IO uint16_t RAM;

}LCD_CONTROLLER_TypeDef;

// NE1 - BANK1 (0x60000000 - 0x64000000), A0 - адрес 0x00000001
#define FMC_BANK1_BASE  ((uint32_t)(0x60000000 | 0x00000000))
#define FMC_BANK1       ((LCD_CONTROLLER_TypeDef *) FMC_BANK1_BASE)


GPIO_TypeDef* GPIOLCD_PORT[IOTFTn] ={	IOTFT_NRESET_GPIO_PORT,
										IOTFT_BL_GPIO_PORT,
										IOTFT_XL_GPIO_PORT,
										IOTFT_XR_GPIO_PORT,
										IOTFT_YU_GPIO_PORT,
										IOTFT_YD_GPIO_PORT};

const uint16_t GPIOLCD_PIN[IOTFTn] ={	IOTFT_NRESET_PIN,
										IOTFT_BL_PIN,
										IOTFT_XL_PIN,
										IOTFT_XR_PIN,
										IOTFT_YU_PIN,
										IOTFT_YD_PIN};


/*****************************************************************
* @brief  This function is executed in case of error occurrence.
******************************************************************/
static void Error_Handler(void)
{
  TST_High(TST4);
  printf("ERROR: [Display Task] Error_Handler\n");
  while(1)  {  }
}
/************************************************************************************
 * vTaskDisplay
*************************************************************************************/
void vTaskDisplay(void *pvParameters)
{
	TCHAR 		BuffpngRead[255];
	uint32_t	lineposetmp=0;
	uint8_t	error;
	uint8_t	radiusDemo=15,StepDemo=1;
	uint8_t	returnErrorFromLcd;
	uint8_t	returnErrorFromLED;
	static uint16_t i=0, j=0, colorR=0;
	static uint16_t Xx=40, Yy=200,Xxinc=1,Yyinc=1,XxPre,YyPre,inkCon=0;
	uint16_t	sizeSYMB,stepSYMB;
	static uint8_t 	Nline[2];


    vTaskDelay(1000);

	IOTFT_AllPortsInit();				// конфиг портов

	/* ## -1- тестируем неисправность светодиодов ##############################*/
	printf("[DP] I:Start LED Test...\n");

	returnErrorFromLED = TestAllLED();
	AllLEDs_Init();									// конфиг светодиодов

	if (returnErrorFromLED == LED_OK)
		{
		OutToAllLED(0);
		printf("[DP] I:LED OK\n");
		}else 	{
			OutToAllLED(0xFFF);
			printf("[DP] E:LED Error\n");
		}

	LCD_BLInit();						// конфиг подсветки
	SetBLBrightnessPercent(70);
	printf("[DP] I:BL Init\n");

	/*##-2- LCD инит #################################################*/
	/* Initialize the LCD */
//	GFTM070_IO_Init();
	returnErrorFromLcd = BSP_LCD_Init();
	returnErrorFromLcd = LCD_OK;
	if (returnErrorFromLcd == LCD_OK)
	{
		printf("[DP] I:Initialize the LCD\n");

//		 GFTM070_WriteCommand(WriteMemoryStart);	//

		/* Clear the LCD Background layer */
		BSP_LCD_Clear(LCD_COLOR_BLACK);
		printf("[DP] I:Clear the LCD Background layer\n");


//		Init_RTC();							//инит часов.


	/*##-2- Touch screen initialization ########################################*/
		Touchscreen_Calibration();
//		BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

	/*##-5- Draw the menu ######################################################*/
		BSP_LCD_Clear(LCD_COLOR_BLACK);
//		Draw_Menu();

//  		BSP_LCD_DrawBitmap(0, 0, (uint8_t *)logo);

		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);

		printf("[DP] I:LCD Init complete\n");
	}

	scan_files("fonts");
	f_chdir("/fonts");											// сменим директорию
	   if(f_open(&FontfntFile, "cour36b.FFN", FA_READ) != FR_OK)		//"Arial36B.FFN"
		{
			printf("[Dp] :not open the cour36b.FFN file\n");
			Write_to_logFile(&LogFileOnSD, (char *)"[Dp] :not open the font.fnt file\n",sizeof("[Dp] :not open the font.fnt file\n")-1);	// добавим в лог.
		} else {
			printf("[Dp] :open the cour36b.FFN file\n");

			error = f_gets(&byteread[0],255,&FontfntFile);					// читаем строку
			sizeSYMB = (uint16_t)byteread[0]<<8 | (uint16_t)byteread[1];
			stepSYMB = (uint16_t)byteread[2]<<8 | (uint16_t)byteread[3];
			Font.table =(uint8_t *)"cour36b.fbn";
			Font.Width = sizeSYMB;
			Font.Height = stepSYMB;

			printf("Width: %u \n",Font.Width);
			printf("Height: %u \n",Font.Height);

			f_close(&FontfntFile);

		}

	   // генератор хекса из .c ------------------------------------------------
	   if(f_open(&FontfntFile, "font.c", FA_READ) != FR_OK)		//"Arial36B.FFN"
		{
			printf("[Dp] :not open the font.c file\n");
		} else {
			printf("[Dp] :open the fnt.fbn file\n");
	   			f_open(&FontBinFile, "fnt.fbn", FA_CREATE_ALWAYS | FA_WRITE);
	   			for(;;)
	   			//while(error)
	   			{
	   			for(i=0;i<255;i++){byteread[i]=0;};
	   			error = f_gets(&byteread[0],255,&FontfntFile);					// читаем строку
	   			if (!error) break;

	   				for(i=1;i<255;i++){
	   					if ((byteread[i-1] == '0') && (byteread[i] == 'x')){	// нашли код.....
	   						byteForwrite[0] = Str_to_8Hex(byteread[i+1],byteread[i+2]);
	   						Write_to_HexFile(&FontBinFile,(TCHAR *)byteForwrite,1);
	   						i += 2;

	   					}
	   				}
	   			}
	   			printf("close font.fbn (%u)\n",error);
	   		    f_close(&FontBinFile);
	   		    f_close(&FontfntFile);
		}
	   			//! генератор хекса из .c ------------------------------------------------

	   f_open(&FFile, "cour36b.FBN", FA_READ);

    	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    	BSP_LCD_SetFont((uint16_t *)&Font.table);		//DrawProp.FileName

 //   	BSP_LCD_DrawRGBImage(500,300,300,180,);

    	BSP_LCD_DisplayStringAt(&FFile, BSP_LCD_GetTextWidth(), BSP_LCD_GetTextHeight(),  "јЅ¬√ƒ≈∆«»… ЋћЌќѕ–—“”‘’÷„ЎўЏџ№Ёёяабвгдежзийклмнопрстуфхцчшщъыьэю€", LEFT_MODE);


    	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
   	 for (i=0;i<(BSP_LCD_GetYSize()/BSP_LCD_GetTextHeight());i++)
   	 {
		    sprintf((uint8_t *)Nline,"%d",i);
			BSP_LCD_DisplayStringAt(&FFile, 0,i*BSP_LCD_GetTextHeight(), (uint8_t *)Nline, LEFT_MODE);
   	 }


	 for (;;)
	  {
		 switch (returnErrorFromLcd){
		 case LCD_OK:

//		    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
//		    sprintf((uint8_t *)aShowTime,"“аск:%u",i++);
//			BSP_LCD_DisplayStringAt(BSP_LCD_GetTextWidth(),8*BSP_LCD_GetTextHeight(), (uint8_t *)aShowTime, LEFT_MODE);
//			BSP_LCD_DisplayStringAt(BSP_LCD_GetTextWidth(),BSP_LCD_GetYSize()-BSP_LCD_GetTextHeight(), (uint8_t *)aShowTime, LEFT_MODE);


//			RTC_CalendarShow(aShowTime, aShowDate);					// получим врем€

//	    	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
//	    	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 0, (uint8_t *)aShowTime, CENTER_MODE);
/*	    	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	    	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, BSP_LCD_GetTextHeight(), (uint8_t *)aShowTime, CENTER_MODE);
		    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	    	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 2*BSP_LCD_GetTextHeight(), (uint8_t *)aShowTime, CENTER_MODE);
*/	    	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	    	BSP_LCD_DisplayStringAt(&FFile, BSP_LCD_GetXSize()/2,0, (uint8_t *)aShowTime, CENTER_MODE);

//			vTaskDelay(100);

//			GetPosition();

			taskYIELD();									// все что нужно выполнили
			 break;
		 case LCD_ERROR:
			 	 TST_Toggle(TST6);
			 	 vTaskDelay(50);
			 break;
		 case LCD_TIMEOUT:
			 	 TST_Toggle(TST6);
			 	 vTaskDelay(250);
			 break;
		 }

	  }

	  vTaskDelete(NULL);				//
}
/************************************************************************************
 * TestAllLED
 * провер€ем исправность светодиодов(закоротки)
 * подаем в 1 порт остальные переключаем на вход и смотрим всЄ ли в норме.
*************************************************************************************/
int8_t TestAllLED(void)
{
static unsigned int i,tstLed=0;

	for (tstLed=LED0;tstLed<=LED11;tstLed++){						// погнали по одному.
		for(i=LED0;i<=LED11;i++)	LED_Init(i,GPIO_MODE_INPUT);	// все на вход, должны быть "1"
		LED_Init(LED11-tstLed,GPIO_MODE_OUTPUT_OD);
		LED_Low(LED11-tstLed);
	 	vTaskDelay(50);
		for(i=LED0;i<=LED10;i++) {	if (i==(LED11-tstLed)) i++;
			if (!ReadInputDataBit(i)) { return LED_ERROR;	}		// все на вход, должны быть "1"
		}
	}
 return	LED_OK;
}
/************************************************************************************
 * Delay_uS
*************************************************************************************/
void Delay_uS(uint16_t Delayus)
{
	while (Delayus--) {
	}
}

/************************************************************************************
 * GPIO_Write
*************************************************************************************/
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  GPIOx->ODR = PortVal;
}

/************************************************************************************
 * GPIO_ReadInputData
*************************************************************************************/
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  return ((uint16_t)GPIOx->IDR);
}
/************************************************************************************
 * GPIO_ReadOutputData
*************************************************************************************/
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  return ((uint16_t)GPIOx->ODR);
}

/************************************************************************************
 * IOLCD_AllPortsInit
*************************************************************************************/
void LCD_BLInit(void){

	/* Compute the prescaler value to have TIM4 counter clock equal to 100  Hz */
	uhPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 100000) - 1;

	TimHandle.Instance = TIM4;
	TimHandle.Init.Prescaler = uhPrescalerValue;
	TimHandle.Init.Period = BL_PERIOD_VALUE;
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;			//TIM_CLOCKDIVISION_DIV1
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)										TST_High(TST4);

	sConfig.OCMode 		= TIM_OCMODE_PWM1;
	sConfig.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode 	= TIM_OCFAST_DISABLE;
	sConfig.Pulse 		= BL_PULSE4;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)	TST_High(TST5);
	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)						TST_High(TST6);

}

/************************************************************************************
 * IOLCD_AllPortsInit
*************************************************************************************/
void IOTFT_AllPortsInit(void){
	BLTFT_Init(IOTFT_BL,GPIO_AF2_TIM4);
	IOTFT_Init(IOTFT_NRESET);
}
/************************************************************************************
*************************************************************************************/
void BLTFT_Init(IOLCD_TypeDef IOLCDPin,uint32_t AlternateFunction)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */
 // IOLCDx_GPIO_CLK_ENABLE(IOLCDPin);
  IOTFT_BL_TIM_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIOLCD_PIN[IOLCDPin];
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;//GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = AlternateFunction;
  HAL_GPIO_Init(GPIOLCD_PORT[IOLCDPin], &GPIO_InitStruct);
}
/************************************************************************************
 * IO_Init
*************************************************************************************/
void LCD_IO_Init(void)
{
  FSMC_BANK1_Init();
}
/************************************************************************************
 * IO_Init дл€ индикатора
*************************************************************************************/
void GFTM070_IO_Init(void)
{
 FSMC_BANK1_Init();
}
/************************************************************************************
 * IO_Init
*************************************************************************************/
void IOTFT_Init(IOLCD_TypeDef IOLCDPin)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */
  IOTFTx_GPIO_CLK_ENABLE(IOLCDPin);

//  IOLCD_BL_TIM_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIOLCD_PIN[IOLCDPin];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;		//GPIO_MODE_OUTPUT_OD //GPIO_MODE_OUTPUT_PP
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//  GPIO_InitStruct.Alternate = AlternateFunction;

  HAL_GPIO_Init(GPIOLCD_PORT[IOLCDPin], &GPIO_InitStruct);
}
/************************************************************************************
 * IO_Init
*************************************************************************************/
void IOTFT_TS_Init(IOLCD_TypeDef IOLCDPin,uint32_t Mode)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */
  IOTFTx_GPIO_CLK_ENABLE(IOLCDPin);


  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIOLCD_PIN[IOLCDPin];
  GPIO_InitStruct.Mode = Mode;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOLCD_PORT[IOLCDPin], &GPIO_InitStruct);
}
/************************************************************************************
 * IOLCD_High
*************************************************************************************/

void IOLCD_High(IOLCD_TypeDef IOLCDPin)
{
	HAL_GPIO_WritePin(GPIOLCD_PORT[IOLCDPin], GPIOLCD_PIN[IOLCDPin],GPIO_PIN_SET);
}
/************************************************************************************
 * IOLCD_Low
*************************************************************************************/

void IOLCD_Low(IOLCD_TypeDef IOLCDPin)
{
	HAL_GPIO_WritePin(GPIOLCD_PORT[IOLCDPin], GPIOLCD_PIN[IOLCDPin],GPIO_PIN_RESET);
}
/*******************************************************************************
*  SetBLBrightnessPercent
*******************************************************************************/
uint8_t SetBLBrightnessPercent(uint8_t brightnessPercent){
	sConfig.Pulse = ((uint32_t)brightnessPercent * (BL_PERIOD_VALUE+1))/100;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {return HAL_ERROR;}
	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {return HAL_ERROR;}
	return HAL_OK;
}

/*************************** FSMC Routines ************************************/
/**
  * @brief  Initializes FSMC_BANK1 MSP.
  * @param  None
  * @retval None
  */
static void FSMC_BANK1_MspInit(void)
{
  GPIO_InitTypeDef GPIO_Init_Structure;

  /* Enable FSMC clock */
  __FSMC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();

  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FSMC;

  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8     |\
                              GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 ;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_7  | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);

  /* GPIOF configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);

}
/**
  * @brief  Initializes LCD IO.
  * @param  None
  * @retval None
  */
static void FSMC_BANK1_Init(void)
{
  SRAM_HandleTypeDef hsram;
  FSMC_NORSRAM_TimingTypeDef SRAM_Timing;

  /*** Configure the SRAM Bank 1 ***/

  /* Configure IPs */
  hsram.Instance  = FMC_NORSRAM_DEVICE;
  hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

  SRAM_Timing.AddressSetupTime      = 0;	//1
  SRAM_Timing.AddressHoldTime       = 0;	//0
  SRAM_Timing.DataSetupTime         = 0;	//1								// длительность RS
  SRAM_Timing.BusTurnAroundDuration = 0;	//0
  SRAM_Timing.CLKDivision           = 0;	//0
  SRAM_Timing.DataLatency           = 0;	//0
  SRAM_Timing.AccessMode            = FSMC_ACCESS_MODE_A;					//FSMC_ACCESS_MODE_B

  hsram.Init.NSBank             = FSMC_NORSRAM_BANK1;
  hsram.Init.DataAddressMux     = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram.Init.MemoryType         = FSMC_MEMORY_TYPE_SRAM;					//FSMC_MEMORY_TYPE_SRAM;
  hsram.Init.MemoryDataWidth    = FSMC_NORSRAM_MEM_BUS_WIDTH_16;			// 16 бит шина
  hsram.Init.BurstAccessMode    = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;			//FSMC_WAIT_SIGNAL_POLARITY_LOW
  hsram.Init.WrapMode           = FSMC_WRAP_MODE_DISABLE;
  hsram.Init.WaitSignalActive   = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram.Init.WriteOperation     = FSMC_WRITE_OPERATION_ENABLE;
  hsram.Init.WaitSignal         = FSMC_WAIT_SIGNAL_DISABLE;
  hsram.Init.ExtendedMode       = FSMC_EXTENDED_MODE_DISABLE;
  hsram.Init.AsynchronousWait   = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram.Init.WriteBurst         = FSMC_WRITE_BURST_DISABLE;

  /* Initialize the SRAM controller */
  FSMC_BANK1_MspInit();
  HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing);
}
/**
  * @brief  Writes register value.
  * @param  Data:
  * @retval None
  */
static void FSMC_BANK1_WriteData(uint16_t Data)
{
  /* Write 16-bit Reg */
  FMC_BANK1->RAM = Data;
}

/**
  * @brief  Writes register address.
  * @param  Reg:
  * @retval None
  */
static void FSMC_BANK1_WriteReg(uint16_t Reg)
{
  /* Write 16-bit Index, then write register */
  FMC_BANK1->REG = Reg;
}

/**
  * @brief  Reads register value.
  * @param  None
  * @retval Read value
  */
static uint16_t FSMC_BANK1_ReadData(void)
{
  return FMC_BANK1->RAM;
}

/**
  * @brief  Writes data on LCD data register.
  * @param  Data: Data to be written
  * @retval None
  */
void LCD_IO_WriteData(uint16_t Data)
{
  /* Write 16-bit Reg */
  FSMC_BANK1_WriteData(Data);
}
/**
  * @brief  Writes register on LCD register.
  * @param  Reg: Register to be written
  * @retval None
  */
void LCD_IO_WriteReg(uint16_t Reg)
{
  /* Write 16-bit Index, then Write Reg */
  FSMC_BANK1_WriteReg(Reg);
}
/**
  * @brief  Reads data from LCD data register.
  * @param  None
  * @retval Read data.
  */
uint16_t LCD_IO_ReadData(void)
{
  return FSMC_BANK1_ReadData();
}

/**
  * @brief  Draws the menu.
  * @param  None
  * @retval None
  */
static void Draw_Menu(void)
{
	uint16_t buttonWidth,buttonHeight;
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  /* Draw color image */
//  BSP_LCD_DrawBitmap(0, 0, (uint8_t *)color);

  /* Draw save image */
//  BSP_LCD_DrawBitmap(0, 0, (uint8_t *)logo);
  buttonWidth  = BSP_LCD_GetXSize()/4-20;
  buttonHeight = 60;

  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_DrawRect(6 ,  1, buttonWidth , buttonHeight);
  BSP_LCD_DrawRect(206, 1, buttonWidth, buttonHeight);
  BSP_LCD_DrawRect(406, 1, buttonWidth, buttonHeight);
  BSP_LCD_DrawRect(606, 1, buttonWidth, buttonHeight);

  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_DrawRect(6, 70, 780, 400);
  BSP_LCD_DrawRect(7, 71, 778, 398);
  BSP_LCD_DrawRect(8, 72, 776, 396);
  BSP_LCD_DrawRect(9, 73, 774, 394);


//  BSP_LCD_DrawRect(63, 30, (BSP_LCD_GetXSize()-66), (BSP_LCD_GetYSize()-66));
//  BSP_LCD_DrawRect(65, 32, (BSP_LCD_GetXSize()-70), (BSP_LCD_GetYSize()-70));
 // BSP_LCD_DrawRect(67, 34, (BSP_LCD_GetXSize()-74), (BSP_LCD_GetYSize()-74));

  /* Draw size icons */
/*  BSP_LCD_FillRect(60, (BSP_LCD_GetYSize() - 48), 90, 48);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(80, (BSP_LCD_GetYSize() - 24), 10);
  BSP_LCD_FillCircle(110, (BSP_LCD_GetYSize() - 24), 5);
  BSP_LCD_FillCircle(135, (BSP_LCD_GetYSize() - 24), 2);

  BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(210, (BSP_LCD_GetYSize() - 55), (uint8_t *)"Selected Color  Size", LEFT_MODE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(220, (BSP_LCD_GetYSize() - 40), 30, 30);
  BSP_LCD_FillCircle(290, (BSP_LCD_GetYSize() - 24), Radius);
  */
}

/*************************************************************************
 * GetPosition
 * дл€ тачскрина.
 *************************************************************************/
static void GetPosition(void)
{
  static uint32_t color_width;
  static uint32_t color;

  color_width = 18;

  /* Get Touch screen position */
  BSP_TS_GetState(&TS_State);

  if(TS_State.TouchDetected)
  {
    /* Read the coordinate */
    x = Calibration_GetX(TS_State.x);
    y = Calibration_GetX(TS_State.y);

//    if((x > (67 + Radius)) & (y > (7 + Radius) ) & ( x < (BSP_LCD_GetXSize() - (7 + Radius ))) & (y < (BSP_LCD_GetYSize() - (67 + Radius ))))
//    {
      BSP_LCD_FillCircle((x), (y), Radius);
//    }
/*    else if((x > 0 ) & ( x < 50 ))
    {
      if((y > 0 ) & (y < color_width ))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        Update_Size(Radius);
      }
      else if((y > color_width ) & (y < (2 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
        Update_Size(Radius);
      }
      else if((y > (2 * color_width)) & (y < (3 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
        Update_Size(Radius);
      }
      else if((y > (3 * color_width)) & (y < (4 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
        Update_Size(Radius);
      }
      else if((y > (4 * color_width)) & (y < (5 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
        Update_Size(Radius);
      }
      else if((y > (5 * color_width)) &(y < (6 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        Update_Size(Radius);
      }
      else if((y > (6 * color_width)) &(y < (7 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
        Update_Size(Radius);
      }
      else if((y > (7 * color_width)) & (y < (8 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        Update_Size(Radius);
      }
      else if((y > (8 * color_width)) & (y < (9 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
        Update_Size(Radius);
      }
      else if((y > (9 * color_width)) & (y < (10 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
        Update_Size(Radius);
      }
      else if((y > (10 * color_width)) & (y < (11 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
        Update_Size(Radius);
      }
      else if((y > (11 * color_width)) & (y < (12 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        Update_Size(Radius);
      }
      else if((y > (12 * color_width)) & (y < (13 * color_width)))
      {
        // Get the current text color
        color = BSP_LCD_GetTextColor();
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        // Clear the working window
        BSP_LCD_FillRect(68, 8, 245, 164);
        BSP_LCD_SetTextColor(color);
      }
      else
      {
        x = 0;
        y = 0;
      }
      Update_Color();
    }
    else if((x > 70) & (y > (BSP_LCD_GetYSize() - 50)) & (y < (BSP_LCD_GetYSize()) ) & ( x < 100))
    {
      Radius = 10;
      Update_Size(Radius);
    }
    else if((x > 100) & (y > (BSP_LCD_GetYSize() - 50)) & (y < (BSP_LCD_GetYSize()) ) & ( x < 130))
    {
      Radius = 5;
      Update_Size(Radius);
    }
    else if((x > 130) & (y > (BSP_LCD_GetYSize() - 50)) & (y < (BSP_LCD_GetYSize()) ) & ( x < 160))
    {
      Radius = 2;
      Update_Size(Radius);
    }
    else if((x > (BSP_LCD_GetXSize()-5) ) & (y > (12 * color_width)) & (y < (13 * color_width)) | ( x < 55) & (y < 5))
    {
      TS_State.x = 0;
      TS_State.y = 0;
    }
    else if((x > 160) & (y > (BSP_LCD_GetYSize() - 50)) & (x < 210) & (y < BSP_LCD_GetYSize()))
    {
      Save_Picture();
    }
    */
  }
}
/*************************************************************************
 * Update_Size
 *
 *************************************************************************/
static void Update_Size(uint8_t size)
{
  static uint32_t color;

  /* Get the current text color */
  color = BSP_LCD_GetTextColor();

  /* Update the selected size icon */
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(290, (BSP_LCD_GetYSize() - 24), 20);
  BSP_LCD_SetTextColor(color);
  BSP_LCD_FillCircle(290, (BSP_LCD_GetYSize() - 24), size);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DrawCircle(290, (BSP_LCD_GetYSize() - 24), size);
  BSP_LCD_SetTextColor(color);
}
/*************************************************************************
 * Update_Color
 *
 *************************************************************************/
static void Update_Color(void)
{
  static uint32_t color;

  /* Get the current text color */
  color = BSP_LCD_GetTextColor();

  /* Update the selected color icon */
  BSP_LCD_FillRect(220, (BSP_LCD_GetYSize() - 40), 30, 30);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DrawRect(220, (BSP_LCD_GetYSize() - 40), 30, 30);
  BSP_LCD_SetTextColor(color);
}
/*************************************************************************
 * Save_Picture
 *
 *************************************************************************/
void Save_Picture(void)
{
}

/*************************************************************************
 * Prepare_Picture
 *
 *************************************************************************/
static void Prepare_Picture(void)
{

}

/*************************************************************************
 * IOE_Init
 * инит ј÷ѕ дл€ тачскрина.
 *************************************************************************/
void IOE_Init(void)
{
	/*
	IOTFT_TS_Init(IOTFT_XR,GPIO_MODE_ANALOG);
	IOTFT_TS_Init(IOTFT_XL,GPIO_MODE_ANALOG);
	IOTFT_TS_Init(IOTFT_YU,GPIO_MODE_ANALOG);
	IOTFT_TS_Init(IOTFT_YD,GPIO_MODE_ANALOG);
	*/
}
/*************************************************************************
 *
 *************************************************************************/
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
 // I2Cx_Write(Addr, Reg, Value);
}
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
 // return I2Cx_Read(Addr, Reg);
	return	0;
}
void IOE_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
void IOE_ITConfig(void)
{
 // I2Cx_ITConfig();
}
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
// return I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
	return 0;
}
/*************************************************************************
 *
 *************************************************************************/
uint8_t ReadFontFile(fnt_struct FNTFileName){

	TCHAR 		BuffFntRead[255], SymbBuff[6];
	uint8_t		i,j,nm,startpos=0,currpos=0;
	uint16_t	symb,tmp;
	uint16_t	n;
	uint32_t	*t;

       f_gets(&BuffFntRead[0],255,&FontfntFile);			// textures file name
       for (i=0;i<255;i++){
    	   FNTFileName.PngFileName[i] = BuffFntRead[10+i];
    	   if (BuffFntRead[i] == ' ') break;
       }

       f_gets(&BuffFntRead[0],255,&FontfntFile);			// font name  and size
       for (i=0;i<255;i++){
    	   FNTFileName.FntName[i] = BuffFntRead[i];
    	   if (BuffFntRead[i] == ' ') {startpos = i+1; break;}
       }
       for (i=0;i<255;i++){
    	   FNTFileName.FntSize[i] = BuffFntRead[i+startpos];
    	   if (BuffFntRead[i+startpos] == ' ') {break;}
       }

       // ----- вынести в отдельную поисковую функцию

       f_gets(&BuffFntRead[0],255,&FontfntFile);			// textures file name
       printf(BuffFntRead); //printf("\n");

       t = &(FNTFileName.UNICODENump);

        for (nm=0;nm<9;nm++){
		   startpos = currpos;

		   for (i=startpos;i<255;i++){
			   SymbBuff[i-startpos] = BuffFntRead[i];
			   if (BuffFntRead[i] == '	' || BuffFntRead[i] == ' ' || BuffFntRead[i] == 0x0A) {currpos = i;break;}
		   }
		   symb=0;
		   for (i=startpos;i<currpos;i++){
			   tmp = 1; for (j=0;j<(currpos-i-1);j++){tmp = tmp*10;}
			   symb += (SymbBuff[i-startpos]-'0')*tmp;
		   }
		   currpos++;
		   *t++ = (unsigned int)symb;

       }

 //       n = GetCharFromFontFile(&FontfntFile);
//       printf("%u",n);
//       printf("\n");

//       f_gets(&BuffFntRead[0],255,&FontfntFile);			// symb position on texture
//       printf(BuffFntRead);

       return 0;
}/*************************************************************************
 *
 *************************************************************************/
/*************************************************************************
 *
 *************************************************************************/
uint8_t ReadPNGtFile(fnt_struct FNTFileName){

	TCHAR 			BuffFntRead[255], SymbBuff[6];
	BYTE 			signature[8];
	BYTE 			IDAT[0x2000];
	HDRCHUNK		HDR;
	IHDRCHUNK		IHDR;
	PHYSCHUNKENTRY	PHYS;
	PLTECHUNKENTRY 	PLTE;
	UINT 			rc;
	uint32_t		lengthchunk;
	uint32_t		namechunk;
	BYTE			PNGmode=0;

	uint16_t		i;


	   f_read(&FontpngFile, &signature, 8, &rc);
	   printf("--------------------------------------------\n");

for (;;){
//-----------------------------------------------------------------
// цикл
//-----------------------------------------------------------------
	   lengthchunk = png_read_chunk_header(&HDR);
	   namechunk = HDR.Name;
//	   printf("namechunk: %d ",(uint8_t)(HDR.Name>>24));
//	   printf("%d ",(uint8_t)(HDR.Name>>16));
//	   printf("%d ",(uint8_t)(HDR.Name>>8));
//	   printf("%d \n",(uint8_t)HDR.Name);

//	   printf("lengthchunk: %u : ",(uint16_t)lengthchunk);

	   if (namechunk == png_IHDR) {									//png_IHDR
	 	   f_read(&FontpngFile, &IHDR, lengthchunk, &rc);
	 	   printf("IHDR: %d \n",rc);
	 	   f_read(&FontpngFile, &IDAT, 4, &rc);
	   }else
	   if (namechunk == png_IDAT){
	 	   f_read(&FontpngFile, &IDAT, lengthchunk, &rc);
	 	   printf("IDAT: %d \n",rc);
	 	   for (i=0;i<lengthchunk;i++){
	 		  printf("%d ",IDAT[i]);
	 	   }
	 	  printf("\n");
	 	   f_read(&FontpngFile, &IDAT, 4, &rc);
	   } else
 	   if (namechunk == png_IEND){
	 	   printf("IEND \n");
 		   break;
 	   }else
 	   if (namechunk == png_pHYs){
			f_read(&FontpngFile, &PHYS, lengthchunk, &rc);
			printf("pHYs: %d \n",rc);
		 	f_read(&FontpngFile, &IDAT, 4, &rc);
 	   }else
 	   {
 	 	   f_read(&FontpngFile, &IDAT, lengthchunk, &rc);
 		   printf("f_read: %u \n",(uint16_t)rc);
 	 	   f_read(&FontpngFile, &IDAT, 4, &rc);
 	   }

}



	   printf("--------------------------------------------\n");
	   printf("Width: %u ",(uint16_t)(IHDR.Width>>16));
	   printf("%u ",(uint16_t)IHDR.Width);
	   printf("\n");

	   printf("Height: %u ",(uint16_t)(IHDR.Height>>16));
	   printf("%u ",(uint16_t)IHDR.Height);
	   printf("\n");

	   printf("BitDepth: %d ",(uint16_t)IHDR.BitDepth);
	   printf("\n");

	   printf("ColorType: %d ",(uint16_t)IHDR.ColorType);
	   printf("\n");

	   printf("Compression: %d ",(uint16_t)IHDR.Compression);
	   printf("\n");

	   printf("Filter: %d ",(uint16_t)IHDR.Filter);
	   printf("\n");

	   printf("Interlace: %d ",(uint16_t)IHDR.Interlace);
	   printf("\n");
	   printf("--------------------------------------------\n");


 //      f_gets(&BuffFntRead[0],255,&FontpngFile);			// textures file name
 //      printf(BuffFntRead);printf(":\n");
//       f_gets(&BuffFntRead[0],255,&FontpngFile);			// textures file name
//       printf(BuffFntRead);printf("\n");
//       f_gets(&BuffFntRead[0],255,&FontpngFile);			// textures file name
//       printf(BuffFntRead);printf("\n");

}


/*************************************************************************
 *
 *************************************************************************/
uint16_t GetCharFromFontFile(FIL* fp){

	TCHAR 		BuffFntRead[255], SymbBuff[6];
	uint8_t		i,j,startpos=0,currpos=0;
	uint16_t	Size,symb,tmp;

//	while (!f_eof(&FontfntFile)){
       f_gets(&BuffFntRead[0],255,&FontfntFile);			// textures file name

       printf(BuffFntRead); printf("\n");

       startpos = currpos;
       for (i=startpos;i<255;i++){
           SymbBuff[i] = BuffFntRead[i];
    	   if (BuffFntRead[i] == '	' || BuffFntRead[i] == ' ') {currpos = i;break;}
       }
       symb=0;
       for (i=startpos;i<currpos;i++){
    	   tmp = 1; for (j=0;j<(currpos-i-1);j++){tmp = tmp*10;}
    	   symb += (SymbBuff[i]-'0')*tmp;
       }

//	}

      return symb;
}

/*************************************************************************
 *
 *************************************************************************/
uint32_t png_read_chunk_header(HDRCHUNK* buff)
{
   BYTE 		buf[8];
   uint32_t 	chunk_name;
   UINT 		rc;
/*
	1. читаем в буфер 8 байт
	2. из буфера забираем первые 4 байта (uint32_t) length  - размер
	3. из буфера забираем след. 4 байта  name[4]    chunk_name - им€
	4. считаем CRC
	5. провер€ем на подлинность им€ куска png_check_chunk_name(0);
	6. возвращаем длину.
*/
   f_read(&FontpngFile, &buf, 8, &rc);
   buff->Length = (uint32_t)buf[0]<<24 | (uint32_t)buf[1]<<16 | (uint32_t)buf[2]<<8 | (uint32_t)buf[3];
   chunk_name = (uint32_t)buf[4]<<24 | (uint32_t)buf[5]<<16 | (uint32_t)buf[6]<<8 | (uint32_t)buf[7];
   buff->Name = chunk_name;
   png_check_chunk_name(chunk_name);

   return buff->Length;
}

/*************************************************************************
 * Bit hacking: the test for an invalid byte in the 4 byte chunk name is:
 * ((c) < 65 || (c) > 122 || ((c) > 90 && (c) < 97))
 *************************************************************************/

void png_check_chunk_name(uint32_t chunk_name)
{
   int i;

//   printf("in png_check_chunk_name\n");

   for (i=1; i<=4; ++i)
   {
      int c = chunk_name & 0xff;

      if (c < 65 || c > 122 || (c > 90 && c < 97))
    	  printf("png_check_chunk_name invalid chunk type\n");

      chunk_name >>= 8;
   }
}
/*************************************************************************
 *
 *************************************************************************/

/***********************************************************************
 *
 ***********************************************************************/
uint8_t Str_to_8Hex(uint8_t in1,uint8_t in2)
{
	uint8_t	ret=0;

	if ((in1>='0' && in1<='9') || (in1>='A' && in1<='F')){
		switch (in1)
		{
		case '1':	ret = 0x10;	break;
		case '2':	ret = 0x20;	break;
		case '3':	ret = 0x30;	break;
		case '4':	ret = 0x40;	break;
		case '5':	ret = 0x50;	break;
		case '6':	ret = 0x60;	break;
		case '7':	ret = 0x70;	break;
		case '8':	ret = 0x80;	break;
		case '9':	ret = 0x90;	break;
		case 'A':	ret = 0xA0;	break;
		case 'B':	ret = 0xB0;	break;
		case 'C':	ret = 0xC0;	break;
		case 'D':	ret = 0xD0;	break;
		case 'E':	ret = 0xE0;	break;
		case 'F':	ret = 0xF0;	break;
		}
	}
	if ((in2>='0' && in2<='9') || (in2>='A' && in2<='F')){
	switch (in2)
		{
		case '1':	ret |= 0x01;	break;
		case '2':	ret |= 0x02;	break;
		case '3':	ret |= 0x03;	break;
		case '4':	ret |= 0x04;	break;
		case '5':	ret |= 0x05;	break;
		case '6':	ret |= 0x06;	break;
		case '7':	ret |= 0x07;	break;
		case '8':	ret |= 0x08;	break;
		case '9':	ret |= 0x09;	break;
		case 'A':	ret |= 0x0A;	break;
		case 'B':	ret |= 0x0B;	break;
		case 'C':	ret |= 0x0C;	break;
		case 'D':	ret |= 0x0D;	break;
		case 'E':	ret |= 0x0E;	break;
		case 'F':	ret |= 0x0F;	break;
		}
	}
	return ret;
}
