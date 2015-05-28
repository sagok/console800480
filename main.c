/*
 * main.c
 *
 *  Created on: 08.08.2014
 *      Author: sagok
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "stm32f2xx_hal.h"
#include "board.h"
#include "board_lcd.h"

#include "LEDs.h"
#include "RTC.h"

#include "usb_task.h"
#include "SD_task.h"
#include "Display.h"


//#include  "usbd_hid_core.h"
//#include  "usbd_usr.h"
//#include  "usbd_desc.h"

/* Private variables ---------------------------------------------------------*/
//FATFS 	SDFatFs;  /* File system object for SD card logical drive */
//FIL 	MyFile;     /* File object */
//char 	SDPath[4]; /* SD card logical drive path */


//#include "usbd_def.h"

/* Private funtcion ---------------------------------------------------------*/


/*************************************************************************
 * приоритеты тасков. 1 - самый низкий
 *************************************************************************/
#define vTaskDisplay_PRIORITY               (tskIDLE_PRIORITY + 2)		//2
#define vTaskUSB_PRIORITY               	(tskIDLE_PRIORITY + 2)		//1
#define vTaskSDMMC_PRIORITY               	(tskIDLE_PRIORITY + 2)		//1

#define	vTaskUSB_STACK_SIZE					configMINIMAL_STACK_SIZE*4
#define	vTaskSDMMC_STACK_SIZE				configMINIMAL_STACK_SIZE*4
/* Private function prototypes -----------------------------------------------*/

/* Global variables ---------------------------------------------------------*/

//LCD_DrawPropTypeDef DrawProp;

RTC_HandleTypeDef RtcHandle;
uint8_t aShowTime[50] = {0};					// текущее время
uint8_t aShowDate[50] = {0};					// текущее дата

xSemaphoreHandle 	xSemaphore_USBInt;			// семафор обработчика USB запросов

uint8_t 		DisplayBuf[64];					// глобальный буфер экрана.

uint16_t 		Terminal_Numb_Line = 1;				// позиция курсора в окне терминала

/*---------------------------------------------------------------------------*/

int main(void) {

	HAL_Init();										// initialize the HAL Library

	SystemClock_Config();							// конфиг осциллятора
	NVIC_Configuration();							// конфиг прерываний

//	Init_RTC();


	AllLEDs_Init();									// конфиг светодиодов
	AllTSTs_Init();									// конфиг сервисных светодиодов
    Uart1_Init();									// конфиг усарта для дебагера

//    RTC_CalendarConfig();

	xTaskCreate( vTaskDisplay,  (char*)"DISP",   configMINIMAL_STACK_SIZE*8/*8*/,   NULL, vTaskDisplay_PRIORITY,( xTaskHandle * ) NULL);	//configMAX_PRIORITIES-1
	xTaskCreate( vTaskUSB,      (char*)"USB",    vTaskUSB_STACK_SIZE,        NULL, vTaskUSB_PRIORITY,     ( xTaskHandle * ) NULL);
	xTaskCreate( vTaskSDMMC,    (char*)"SDMC",   vTaskSDMMC_STACK_SIZE,      NULL, vTaskSDMMC_PRIORITY,     ( xTaskHandle * ) NULL);



	vTaskStartScheduler();							// Запускаем планировщик


	while (1) {

	}
	return 0;
}

/*************************************************************************
 * vApplicationStackOverflowHook   при переполнении стека
 *************************************************************************/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pxTask;
	( void ) pcTaskName;
	for( ;; );
}
/*******************************************************
 *
 ******************************************************/
/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{

}

/*
 int *x;	// указатель 			переменная *x хранит адрес чего либо в памяти.
 int y;		// обычная переменная
 int d;		// обычная переменная

 x=&y;		// теперь в указателе x лежит адрес y;
 y = 15;	// после этого z = *x; вернёт значение 15.
 *x = 23;	// по адресу *x записали новое значение, после этого в переменной у лежит новое значение 23;

 x=&d;		// теперь указатель указывает на новую переменную d;
 *x = 23;	// теперь изменили cjlth;bvjt переменной d;


чтобы записать или прочитать значение по адресу указателя используется разименовывание y = *x;

у любой переменной можно взять адрес: &y;

*/

uint16_t TerminalIncNumbLine(void){
//	Terminal_Numb_Line += DrawProp.pFont->Height;
//	if (Terminal_Numb_Line >= BSP_LCD_GetYSize()) Terminal_Numb_Line = DrawProp.pFont->Height;
//	return Terminal_Numb_Line;
}

/*****************************************************************
* @brief  This function is executed in case of error occurrence.
******************************************************************/
void Error_Handler_Main(void)
{
  TST_High(TST4);
  printf("ERROR: [Global] Error_Handler\n");
  while(1)  {  }
}
