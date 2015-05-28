/*
 * main.h
 *
 *  Created on: 08.08.2014
 *      Author: sagok
 */



#ifndef MAIN_H_
#define MAIN_H_


typedef enum {
	FALSE   = 0,
	TRUE
}Error_StatusTypeDef;


/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

SD_HandleTypeDef 		uSdHandle;
HAL_SD_CardInfoTypedef SD_CardInfo;


void Error_Handler_Main(void);

#endif /* MAIN_H_ */
