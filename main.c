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
 * ���������� ������. 1 - ����� ������
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
uint8_t aShowTime[50] = {0};					// ������� �����
uint8_t aShowDate[50] = {0};					// ������� ����

xSemaphoreHandle 	xSemaphore_USBInt;			// ������� ����������� USB ��������

uint8_t 		DisplayBuf[64];					// ���������� ����� ������.

uint16_t 		Terminal_Numb_Line = 1;				// ������� ������� � ���� ���������

/*---------------------------------------------------------------------------*/

int main(void) {

	HAL_Init();										// initialize the HAL Library

	SystemClock_Config();							// ������ �����������
	NVIC_Configuration();							// ������ ����������

//	Init_RTC();


	AllLEDs_Init();									// ������ �����������
	AllTSTs_Init();									// ������ ��������� �����������
    Uart1_Init();									// ������ ������ ��� ��������

//    RTC_CalendarConfig();

	xTaskCreate( vTaskDisplay,  (char*)"DISP",   configMINIMAL_STACK_SIZE*8/*8*/,   NULL, vTaskDisplay_PRIORITY,( xTaskHandle * ) NULL);	//configMAX_PRIORITIES-1
	xTaskCreate( vTaskUSB,      (char*)"USB",    vTaskUSB_STACK_SIZE,        NULL, vTaskUSB_PRIORITY,     ( xTaskHandle * ) NULL);
	xTaskCreate( vTaskSDMMC,    (char*)"SDMC",   vTaskSDMMC_STACK_SIZE,      NULL, vTaskSDMMC_PRIORITY,     ( xTaskHandle * ) NULL);



	vTaskStartScheduler();							// ��������� �����������


	while (1) {

	}
	return 0;
}

/*************************************************************************
 * vApplicationStackOverflowHook   ��� ������������ �����
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
 int *x;	// ��������� 			���������� *x ������ ����� ���� ���� � ������.
 int y;		// ������� ����������
 int d;		// ������� ����������

 x=&y;		// ������ � ��������� x ����� ����� y;
 y = 15;	// ����� ����� z = *x; ����� �������� 15.
 *x = 23;	// �� ������ *x �������� ����� ��������, ����� ����� � ���������� � ����� ����� �������� 23;

 x=&d;		// ������ ��������� ��������� �� ����� ���������� d;
 *x = 23;	// ������ �������� cjlth;bvjt ���������� d;


����� �������� ��� ��������� �������� �� ������ ��������� ������������ ��������������� y = *x;

� ����� ���������� ����� ����� �����: &y;

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
