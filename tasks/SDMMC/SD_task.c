/*
 * SD_task.c
 *
 *  Created on: 15.09.2014
 *      Author: sagok
 */

/*		SPISD_MISO	-	PB4	(JTRST)
 * 		SPISD_MOSI	-	PB5
 * 		SPISD_SCK	-	PB3	(JTDO)
 * 	SPISD_MEM_NCS	-	PA4
 *
 *
 */
#ifndef SD_TASK_C_
#define SD_TASK_C_

/* Standard includes. */
#include <stdint.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "main.h"
#include "SD_task.h"
#include "LEDs.h"
#include "RTC.h"
#include "Board_lcd.h"

/* FatFs includes component */
#include "ff_gen_drv.h"

#include "DiscBrowser.h"
//#include "sd_diskio.h"


/************************************************************************************
 * ���������� ����������
*************************************************************************************/
extern uint8_t aShowTime[50];
extern uint8_t aShowDate[50];
/************************************************************************************
 * ��������� �������
*************************************************************************************/

/************************************************************************************
 * ��������� ����������
*************************************************************************************/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFileSD;     /* File object */
FIL LogFileOnSD;     					// ������� �� ��������

FIL FileFont;
char SDPath[4]; 						/* SD card logical drive path */

extern Diskio_drvTypeDef  SD_Driver;

/*****************************************************************
* @brief  This function is executed in case of error occurrence.
******************************************************************/
static void Error_Handler(void)
{
  TST_High(TST4);
  printf("ERROR: [SDTask] Error_Handler\n");
  while(1)  {  }
}
/************************************************************************************
 * vTaskDisplay
*************************************************************************************/
void vTaskSDMMC(void *pvParameters)
{
	FRESULT		res;
	uint32_t 	byteswritten;
	uint8_t  	MessageLog[255];
	int 		Hours, Minutes, Seconds, Date, Month, Year;


	Init_RTC();																			//���� �����.

	 RTC_CalendarShow(aShowTime, aShowDate);

	  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)										// ��������� ������� SD �����
	  {
			printf("[SD] I:vTaskSDMMC - Link the SD disk I/O driver .. ok\n");
		    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)						// ��������� ��������� ��� �� �����
		    {
			  printf("[SD] E:vTaskSDMMC - FatFs Initialization Error\n");
		      Error_Handler();
		    }
		    else
		     {
		    	Write_to_logFile(&LogFileOnSD, (char *)"Power on.\n",sizeof("Power on.\n")-1);	// ������� � ���.

//				RTC_GetTime(Hours,Minutes,Seconds);
//				RTC_GetDate(Date,Month,Year);
//				set_timestamp("log.txt",2015,Month,Date,Hours,Minutes,Seconds);
		     }

	  }
//	  FATFS_UnLinkDriver(SDPath);				// ��������� ����

	  for (;;)
	  {
	 	 TST_Toggle(TST5);
		 RTC_CalendarShow(aShowTime, aShowDate);
		 vTaskDelay(1000);
		 taskYIELD();									// ��� ��� ����� ���������
//		 vTaskDelete(NULL);				// ������� ���� ���� ����� ��������� �� ����� �����
	  }
	  vTaskDelete(NULL);				// ������� ���� ���� ����� ��������� �� ����� �����
}

#endif /* SD_TASK_C_ */
