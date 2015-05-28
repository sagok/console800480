
#include "stm32f2xx_hal.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "main.h"
#include "Display.h"
#include "LEDs.h"

// ----- tmp ------
#include "logo.h"
#include "Fonts\fonts.h"
#include "Board_lcd.h"
// ----- USB ------
// Tasks
#include "usb_task.h"
// Device
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

#include "usbd_msc.h"
#include "usbd_msc_storage.h"
// HOST
#include "usbh_core.h"
#include "usbh_msc.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "usbh_diskio.h"


/************************************************************************************
 * глобальные переменные
*************************************************************************************/
//extern uint8_t 		Terminal_Numb_Line;				// позиция курсора в окне терминала

//extern xSemaphoreHandle 	xSemaphore_USBInt;			// хранилище семафоров
//extern PCD_HandleTypeDef 	hpcd;

//USER_STANDARD_REQUESTS 	User_Standard_Requests;
__IO uint8_t PrevXferComplete = 1;

/************************************************************************************
 * локальные переменные
*************************************************************************************/
USBD_HandleTypeDef hUSBDDevice;

USBD_HandleTypeDef USBD_Device;
USBH_HandleTypeDef hUSB_Host; 				/* USB Host handle */

//LCD_DrawPropTypeDef DrawProp;
FileSD_DrawPropTypeDef DrawProp;

FATFS	USBDISKFatFs;           		/* File system object for USB disk logical drive */
FIL MyFile;                   		/* File object */
char USBDISKPath[4];          	/* USB Host logical drive path */


typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_START,
  APPLICATION_RUNNING,
}MSC_ApplicationTypeDef;

MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;
/************************************************************************************
 * локальные функции
*************************************************************************************/
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static void MSC_Application(void);
static void Error_Handler(void);
//void USB_Interrupts_Config(void);
//void Set_USBClock(void);
//void USART_Config_Default(void);

/************************************************************************************
 * vTaskUSB
*************************************************************************************/
void vTaskUSB(void *pvParameters)
{
//	char _desc[50];

//	  vSemaphoreCreateBinary(xSemaphore_USBInt);			// создаем двоичный семафор для запуска обработчика USB

	  vTaskDelay(3000);

/*		// MSC не доделано, виснет при чтении из SD
	   USBD_Init(&hUSBDDevice, &MSC_Desc, 0);
	   USBD_RegisterClass(&hUSBDDevice, &USBD_MSC);
	   USBD_MSC_RegisterStorage(&hUSBDDevice, &USBD_MSC_fops);
	   USBD_Start(&hUSBDDevice);
*/
//	   BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
//	   BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
//	   BSP_LCD_SetFont(&Font24);
 //      BSP_LCD_DisplayStringAt(25, BSP_LCD_GetYSize()-TerminalIncNumbLine() * DrawProp.pFont->Height, (uint8_t *)"USB dev.", LEFT_MODE);


/*
	  USBD_Init(&USBD_Device, &APDKC_Desc, 0);						// Init Device Library
 	  USBD_RegisterClass(&USBD_Device, &USBD_CDC);					// Add Supported Class
	  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);		// Add CDC Interface Class
	  USBD_Start(&USBD_Device);										// Start Device Process
	  printf("[USB] I:USBD_Start\n");
*/


	  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)	  		//##-1- Link the USB Host disk I/O driver
	  {
	    USBH_Init(&hUSB_Host, USBH_UserProcess, 0);	    			//##-2- Init Host Library
	    USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);	    		//##-3- Add Supported Class
	    USBH_Start(&hUSB_Host);	    								//##-4- Start Host Process
		printf("[USB] I:USBH_Start\n");


//		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
//		BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
//		BSP_LCD_SetFont(&Font20);
//		BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize() - TerminalIncNumbLine()-20, (uint8_t *)"Please insert usb drive with the logo.bmp", LEFT_MODE);
//		BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize() - TerminalIncNumbLine()-20, (uint8_t *)"АБВГДЕЁЖЗ_абвгдеёжз", LEFT_MODE);

//		printf("~: %d,}: %d \n",'~','}');
//        printf("А: %d,Б: %d \n",'А','Б');
//       printf("а: %d,б: %d \n",'а','б');

	  }



	 for (;;)
	  {
//		TST_Toggle(TST1);
		/* USB Host Background task */
    	USBH_Process(&hUSB_Host);
//		printf("[USB] I:USBH_Process(&hUSB_Host);\n");

 		/* Mass Storage Application State Machine */
			  switch(Appli_state)
			  {
			  case APPLICATION_START:
				/*
				BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
				BSP_LCD_SetFont(&Font24);
				BSP_LCD_DisplayStringAt(700, 30, (uint8_t *)"USBMSD", CENTER_MODE);
				BSP_LCD_DisplayStringAt(700, 5, (uint8_t *)"inserted", CENTER_MODE);
				 */

				MSC_Application();
				Appli_state = APPLICATION_IDLE;
				break;

			  case APPLICATION_IDLE:

			  default:
				break;
			  }

	  }
	  vTaskDelete(NULL);				// удаляем таск если вдруг вывалился из цикла таска
}

/*****************************************************************
* @brief  User Process
* @param  phost: Host handle
* @param  id: Host Library user message ID
* @retval None
******************************************************************/
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
//  char desc[50];
//  MSC_HandleTypeDef *MSC_Handle =  phost->pActiveClass->pData;

  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
    break;

  case HOST_USER_DISCONNECTION:

    f_mount(NULL, (TCHAR const*)"", 0);

	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
//	BSP_LCD_DisplayStringAt(700, 30, (uint8_t *)"USBMSD", CENTER_MODE);
//	BSP_LCD_DisplayStringAt(700, 5, (uint8_t *)"inserted", CENTER_MODE);
    Appli_state = APPLICATION_IDLE;
    break;

  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_START;
    break;

  default:
    break;
  }
}
/*****************************************************************
* @brief  Main routine for Mass Storage Class
* @param  None
* @retval None
******************************************************************/
static void MSC_Application(void)
{
  uint32_t byteswritten;                     			/* File write/read counts */
  uint8_t wtext[] = "FatFs is ready. лог файл создан."; /* File write buffer */

  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    Error_Handler();    						/* FatFs Initialization Error */
  }
  else
  {
	  // запись файла
	  if (f_open(&MyFile, "0:log.txt", FA_CREATE_ALWAYS | FA_WRITE) !=FR_OK){
	  					printf("[USB] I:f_open log.txt for write .. error \n");
	  			        Error_Handler();
	  				}else{
	  					printf("[USB] I:f_open log.txt for write .. ok \n");
	  					if (f_write(&MyFile, wtext, 32, (void *)&byteswritten) == FR_OK){
	  						 f_close(&MyFile);
	  					}
	  				}
	  //! запись файла
  }
  FATFS_UnLinkDriver(USBDISKPath);								  /* Unlink the USB disk I/O driver */
}
/*****************************************************************
* @brief  This function is executed in case of error occurrence.
******************************************************************/
static void Error_Handler(void)
{
  TST_High(TST4);
  printf("ERROR: [USBTask] Error_Handler\n");
  while(1)  {  }
}




