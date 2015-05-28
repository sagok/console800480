/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-March-2014
  * @brief   Source file for USBD CDC interface
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

#include "usb_task.h"
#include "LEDs.h"
#include "Display.h"
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_DATA_SIZE  64//2048
#define APP_TX_DATA_SIZE  32768

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t 		state = ST_WAIT_PACKET_HDR;		// состояние USB запроса
int8_t 			From_USBpacket = FALSE;
int8_t 			HEAD = FALSE;

uint8_t USBRxBuffer[APP_RX_DATA_SIZE];			/* Received Data over USB are stored in this buffer */
uint8_t USBTxBuffer[APP_TX_DATA_SIZE];			/* Received Data over UART (CDC interface) are stored in this buffer */
uint32_t BuffLength;
uint32_t USB_Tx_Buffer_Size_For_Out;

/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint8_t* buf, uint32_t *Len);
static int8_t CDC_Itf_Transmit (uint8_t* buf, uint16_t Len);

static int8_t decode_hdr(uint8_t* USB_Rx_Buffer,uint8_t USB_Rx_Cnt);
void send_ack(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops = 
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : void send_ack(void).
* Description    : подготовка данных для USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void send_ack(void)
{
	USBTxBuffer[0] = ACK;
	USB_Tx_Buffer_Size_For_Out = 1;
}
/******************************************************************************
 * Function:        void decode_hdr(void)
 * Формат заголовка:
 *   Маркер     команда     hi_size     lo_size     wt_hsize   wt_lsize    timeout       CRC
 * |--------|  |--------|  |--------|  |--------|  |--------| |--------|  |--------|  |--------|
 *     0           1           2           3           4           5          6           7
 *
 *****************************************************************************/
static int8_t decode_hdr(uint8_t* USB_Rx_Buffer,uint8_t USB_Rx_Cnt)
{
  uint8_t 	i,sum;

  if(USB_Rx_Cnt < HDR_SIZE) return FALSE;
  if(USB_Rx_Buffer[0] != MARKER) return FALSE;
  if(USB_Rx_Buffer[1] >= Line485CMD_LAST) return FALSE;
  // ----------- расчет CRC для игнора ошибочных данных -----------------------
  sum = 0;
  for(i=0; i<(HDR_SIZE-1); i++) sum += USB_Rx_Buffer[i];
	  if(sum != USB_Rx_Buffer[HDR_SIZE-1]) {
		  return FALSE;
	  }
  // ----------- ..расчет CRC для игнора ошибочных данных ---------------------
  comd            	= USB_Rx_Buffer[1];
  data_size 		= (uint16_t)(USB_Rx_Buffer[2] << 7) | USB_Rx_Buffer[3];
  cnt_back       	= (uint16_t)(USB_Rx_Buffer[4] << 7) | USB_Rx_Buffer[5];
  timeout        	= USB_Rx_Buffer[6];

  return TRUE;

}
/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
	  USBD_CDC_SetTxBuffer(&USBD_Device, USBTxBuffer, 0);
	  USBD_CDC_SetRxBuffer(&USBD_Device, USBRxBuffer);

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  return (USBD_OK);
}


/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
uint8_t			result,resultInternalTransfer;
int8_t			HeaderStatus;
uint8_t			offsetlocal = 0;
uint16_t		countlocal;

	if (state != ST_SEND_TO_PC)
		HeaderStatus = decode_hdr(Buf,*Len);				// парсим пакет смотрим кто он, что он....

	if (state == ST_WAIT_PACKET_HDR){
		  if (!HeaderStatus){								// ожидаем хедер а пришеол мусор
			  From_USBpacket = FALSE;
		  }else
		  if (HeaderStatus){								// ожидаем хедер он и пришел
			  if(comd == Line485CMD_CHECK)							// команда проверки
			  {
				  send_ack();
				  result = CDC_Itf_Transmit(USBTxBuffer, USB_Tx_Buffer_Size_For_Out);

				  From_USBpacket = FALSE;					// после выхода ненадо работать по запросуUSB
			  } else
			  if(comd != Line485CMD_CHECK)							// команда проверки
			  {												// все остальные команды
				  HEAD = TRUE;								// указываем что это хедер
				  offsetlocal = HDR_SIZE;
				  countlocal = *Len - HDR_SIZE;
				  From_USBpacket = TRUE;
			  }
		  }

	}else	//(state == ST_SEND_TO_PC
	{
		HEAD = FALSE;
		countlocal = *Len;
		From_USBpacket = TRUE;
	}

	if (From_USBpacket){																			// если был запрос из USB то нужно его запров отослать
																									// в шину с очередным сеансом мониторинга и
																									// по окончанию получить ответ и вернуть в шину
	if (state == ST_WAIT_PACKET_HDR){
		resultInternalTransfer = TRUE;//strela_ext_ctrl((uint8_t*)USB_Rx_Buffer+offsetlocal, countlocal, TRUE);
		  if (!resultInternalTransfer) {															// если не дописан или недочитан файл.
				  state = ST_SEND_TO_PC;															// на длинные пакеты ответ не даем. Дадим по окончанию всего пакета
			   } else {
				  result = CDC_Itf_Transmit(USBTxBuffer, USB_Tx_Buffer_Size_For_Out);
			   }

	}else
//		if (state == ST_SEND_TO_PC)
			{																						// если не хэдер то дописываем данные но уже без оффсета
				resultInternalTransfer = TRUE;//strela_ext_ctrl((uint8_t*)USB_Rx_Buffer, countlocal, FALSE);
				if (resultInternalTransfer) {
					state = ST_WAIT_PACKET_HDR;														// если передали весь фай то указываем хедер
					 result = CDC_Itf_Transmit(USBTxBuffer, USB_Tx_Buffer_Size_For_Out);			// посути нужно давать один ответ на весь пакет.

				}
			}

	 From_USBpacket	= FALSE;
	}


///		for (i=0;i<*Len;i++)	WH1604A_print_hex_xx(i/16,(i % 16)*2,Buf[i]);

	USBD_CDC_ReceivePacket(&USBD_Device);

	return result;
}

static int8_t CDC_Itf_Transmit(uint8_t* Buf, uint16_t Len)
{

	  memcpy(USBTxBuffer,Buf,Len);

	  USBD_CDC_SetTxBuffer(&USBD_Device, USBTxBuffer, Len);
	  return (USBD_CDC_TransmitPacket(&USBD_Device));

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

