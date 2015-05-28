/*
 * RTC.c
 *
 *  Created on: 04.03.2015
 *      Author: sagok
 */


/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
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
#include "RTC.h"
#include "LEDs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Buffers used for displaying Time and Date */

static void Error_Handler(void);

/* RTC handler declaration */
extern RTC_HandleTypeDef RtcHandle;


void Init_RTC(void){

	  /*##-1- Configure the RTC peripheral #######################################*/
	  /* Configure RTC prescaler and RTC data registers */
	  /* RTC configured as follow:
	      - Hour Format    = Format 24
	      - Asynch Prediv  = Value according to source clock
	      - Synch Prediv   = Value according to source clock
	      - OutPut         = Output Disable
	      - OutPutPolarity = High Polarity
	      - OutPutType     = Open Drain */
	  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
	  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
	  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	  RtcHandle.Instance = RTC;

	  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }
//	  else   printf("[RTC_Calendar] Initialization ok...\n");


	  /*##-2- Check if Data stored in BackUp register0: No Need to reconfigure RTC#*/
	  /* Read the Back Up Register 0 Data */
	  if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0) != 0x32F2)
	  {
	    /* Configure RTC Calendar */
	    RTC_CalendarConfig();
	  }
	  else
	  {
	    /* Check if the Power On Reset flag is set */
	    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
	    {
	      /* Turn on LED2: Power on reset occured */
//	      BSP_LED_On(LED2);
	    }
	    /* Check if Pin Reset flag is set */
	    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
	    {
	      /* Turn on LED4: External reset occured */
//	      BSP_LED_On(LED4);
	    }
	    /* Clear source Reset Flag */
	    __HAL_RCC_CLEAR_RESET_FLAGS();
	  }

}


void RTC_CalendarConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = 0x15;
  sdatestructure.Month = RTC_MONTH_MARCH;
  sdatestructure.Date = 0x05;
  sdatestructure.WeekDay = RTC_WEEKDAY_SATURDAY;

  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:00:00 */
  stimestructure.Hours = 0x09;
  stimestructure.Minutes = 0x24;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-3- Writes a data in a RTC Backup data Register0 #######################*/
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, 0x32F2);
}

/**
  * @brief  Display the current time and date.
  * @param  showtime : pointer to buffer
  * @param  showdate : pointer to buffer
  * @retval None
  */
void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

  /* Display time Format : hh:mm:ss */
//  printf(" %0.2d:%0.2d:%0.2d ", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
//  printf(" %0.2d-%0.2d-%0.2d \n", sdatestructureget.Date, sdatestructureget.Month,  2000 + sdatestructureget.Year);

 sprintf((char *)showtime, "%0.2d:%0.2d:%0.2d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  /* Display date Format : mm-dd-yy */
 sprintf((char *)showdate, "%0.2d-%0.2d-%0.2d", sdatestructureget.Date, sdatestructureget.Month, 2000 + sdatestructureget.Year);
}

/*****************************************************************
* RTC_GetTime получаем время от часов в uint8_t
******************************************************************/
void RTC_GetTime(uint8_t *Hours, uint8_t *Minutes, uint8_t *Seconds)
{
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);

  Hours   = stimestructureget.Hours;
  Minutes = stimestructureget.Minutes;
  Seconds = stimestructureget.Seconds;

    printf("RTC_GetTime: %0.2d:%0.2d:%0.2d \n", Hours, Minutes, Seconds);

}
/*****************************************************************
* RTC_GetDate получаем время от часов в uint8_t
******************************************************************/
void RTC_GetDate(uint8_t *Date, uint8_t *Month, uint8_t *Year )
{
  RTC_DateTypeDef sdatestructureget;

  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

  Date = sdatestructureget.Date; if (!Date) Date=1;
  Month = sdatestructureget.Month; if (!Month) Month=1;
  Year = 2000 + sdatestructureget.Year;
}

/*****************************************************************
* @brief  This function is executed in case of error occurrence.
******************************************************************/
static void Error_Handler(void)
{
  TST_High(TST4);
  printf("ERROR: [RTC_Calendar] Error_Handler\n");
  while(1)  {  }
}
