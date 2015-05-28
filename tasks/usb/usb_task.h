/*
 * usb_task.h
 *
 *  Created on: 12.04.2013
 *      Author: sagok
 */

#ifndef USB_TASK_H_
#define USB_TASK_H_

enum M_STATES{
  ST_WAIT_PACKET_HDR
, ST_WAIT_PACKET_DATA
, ST_SEND_TO_PC
  };

enum COMMANDS{
  Line485CMD_CHECK
, Line485CMD_SEND
, Line485CMD_SEND_AND_WAIT_RESP
, Line485CMD_LAST
  };


enum {
  HDR_SIZE 	= 8
, MARKER 	= 'P'
, ACK 		= 'A'
, NAK 		= 'N'
};

uint16_t 		data_size;
uint16_t 		cnt_back;
uint8_t 		timeout;
uint8_t 		comd;

void vTaskUSB(void *pvParameters )  __attribute__((naked));


#endif /* USB_TASK_H_ */
