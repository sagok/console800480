/**
 ******************************************************************************
 * @file    USB_Device/CDC_Standalone/Src/usbd_desc.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    13-March-2014
 * @brief   This file provides the USBD descriptors and string formating method.
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
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USBD_VID                      0x0483
#define USBD_PID                      0xFFF0
#define USBD_LANGID_STRING            0x0409
#define USBD_MANUFACTURER_STRING      "BEMN"
#define USBD_PRODUCT_HS_STRING        "STRELA-M (APDKC-01) HS"		//STRELA-M (APDKC-01)
#define USBD_SERIALNUMBER_HS_STRING   "Rev.03"			//STM3210
#define USBD_PRODUCT_FS_STRING        "STRELA-M (APDKC-01) FS"
#define USBD_SERIALNUMBER_FS_STRING   "Rev.03"
#define USBD_CONFIGURATION_HS_STRING  "APDKC Config HS"
#define USBD_INTERFACE_HS_STRING      "APDKC Interface HS"
#define USBD_CONFIGURATION_FS_STRING  "APDKC Config FS"
#define USBD_INTERFACE_FS_STRING      "APDKC Interface FS"

#define USBD_VID                      0x0483
#define USBD_PID                      0x5730
#define USBD_MSCPRODUCT_HS_STRING      "STM32 AUDIO Streaming in HS Mode"
//#define USBD_SERIALNUMBER_HS_STRING   "00000000034E"
#define USBD_MSCPRODUCT_FS_STRING     "STM32 AUDIO Streaming in FS Mode"
//#define USBD_SERIALNUMBER_FS_STRING   "00000000034F"
#define USBD_MSCCONFIGURATION_HS_STRING  "AUDIO Config"
#define USBD_MSCINTERFACE_HS_STRING      "AUDIO Interface"
#define USBD_MSCCONFIGURATION_FS_STRING  "AUDIO Config"
#define USBD_MSCINTERFACE_FS_STRING      "AUDIO Interface"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
uint8_t *USBD_APDKC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_APDKC_LangIDStrDescriptor(USBD_SpeedTypeDef speed,uint16_t *length);
uint8_t *USBD_APDKC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed,uint16_t *length);
uint8_t *USBD_APDKC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_APDKC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_APDKC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_APDKC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed,	uint16_t *length);
#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *USBD_APDKC_USRStringDesc (USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);
#endif /* USB_SUPPORT_USER_STRING_DESC */  


uint8_t *USBD_MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *USBD_MSC_USRStringDesc(USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);
#endif /* USB_SUPPORT_USER_STRING_DESC */

/* Private variables ---------------------------------------------------------*/
USBD_DescriptorsTypeDef APDKC_Desc = {
		USBD_APDKC_DeviceDescriptor,
		USBD_APDKC_LangIDStrDescriptor,
		USBD_APDKC_ManufacturerStrDescriptor,
		USBD_APDKC_ProductStrDescriptor,
		USBD_APDKC_SerialStrDescriptor,
		USBD_APDKC_ConfigStrDescriptor,
		USBD_APDKC_InterfaceStrDescriptor,
};

USBD_DescriptorsTypeDef MSC_Desc = {
  USBD_MSC_DeviceDescriptor,
  USBD_MSC_LangIDStrDescriptor,
  USBD_MSC_ManufacturerStrDescriptor,
  USBD_MSC_ProductStrDescriptor,
  USBD_MSC_SerialStrDescriptor,
  USBD_MSC_ConfigStrDescriptor,
  USBD_MSC_InterfaceStrDescriptor,
};


/* USB Standard Device Descriptor */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t hUSBDDeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = { 0x12, /* bLength */
USB_DESC_TYPE_DEVICE, /* bDescriptorType */
0x00, /* bcdUSB */
0x02, 0x00, /* bDeviceClass */
0x00, /* bDeviceSubClass */
0x00, /* bDeviceProtocol */
USB_MAX_EP0_SIZE, /* bMaxPacketSize */
LOBYTE(USBD_VID), /* idVendor */
HIBYTE(USBD_VID), /* idVendor */
LOBYTE(USBD_PID), /* idVendor */
HIBYTE(USBD_PID), /* idVendor */
0x00, /* bcdDevice rel. 2.00 */
0x01, USBD_IDX_MFC_STR, /* Index of manufacturer string */
USBD_IDX_PRODUCT_STR, /* Index of product string */
USBD_IDX_SERIAL_STR, /* Index of serial number string */
USBD_MAX_NUM_CONFIGURATION /* bNumConfigurations */
}; /* USB_DeviceDescriptor */

__ALIGN_BEGIN uint8_t hUSBDMSCDeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = { 0x12, /* bLength */
USB_DESC_TYPE_DEVICE, /* bDescriptorType */
0x00, /* bcdUSB */
0x02, 0x00, /* bDeviceClass */
0x00, /* bDeviceSubClass */
0x00, /* bDeviceProtocol */
USB_MAX_EP0_SIZE, /* bMaxPacketSize */
LOBYTE(USBD_VID), /* idVendor */
HIBYTE(USBD_VID), /* idVendor */
LOBYTE(USBD_PID), /* idVendor */
HIBYTE(USBD_PID), /* idVendor */
0x00, /* bcdDevice rel. 2.00 */
0x01, USBD_IDX_MFC_STR, /* Index of manufacturer string */
USBD_IDX_PRODUCT_STR, /* Index of product string */
USBD_IDX_SERIAL_STR, /* Index of serial number string */
USBD_MAX_NUM_CONFIGURATION /* bNumConfigurations */
}; /* USB_DeviceDescriptor */

/* USB Standard Device Descriptor */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
		USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
		LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING), };

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;
__ALIGN_BEGIN uint8_t USBD_MSCStrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Returns the device descriptor.
 * @param  speed: Current device speed
 * @param  length: Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t *USBD_APDKC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	*length = sizeof(hUSBDDeviceDesc);
	return hUSBDDeviceDesc;
}

uint8_t *USBD_MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = sizeof(hUSBDMSCDeviceDesc);
  return hUSBDMSCDeviceDesc;
}


/**
 * @brief  Returns the LangID string descriptor.
 * @param  speed: Current device speed
 * @param  length: Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t *USBD_APDKC_LangIDStrDescriptor(USBD_SpeedTypeDef speed,
		uint16_t *length) {
	*length = sizeof(USBD_LangIDDesc);
	return USBD_LangIDDesc;
}

uint8_t *USBD_MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}


/**
 * @brief  Returns the product string descriptor.
 * @param  speed: Current device speed
 * @param  length: Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t *USBD_APDKC_ProductStrDescriptor(USBD_SpeedTypeDef speed,
		uint16_t *length) {
	if (speed == 0) {
		USBD_GetString((uint8_t *) USBD_PRODUCT_HS_STRING, USBD_StrDesc,
				length);
	} else {
		USBD_GetString((uint8_t *) USBD_PRODUCT_FS_STRING, USBD_StrDesc,
				length);
	}
	return USBD_StrDesc;
}

uint8_t *USBD_MSC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_MSCPRODUCT_HS_STRING, USBD_MSCStrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_MSCPRODUCT_FS_STRING, USBD_MSCStrDesc, length);
  }
  return USBD_StrDesc;
}


/**
 * @brief  Returns the manufacturer string descriptor.
 * @param  speed: Current device speed
 * @param  length: Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t *USBD_APDKC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed,
		uint16_t *length) {
	USBD_GetString((uint8_t *) USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}
uint8_t *USBD_MSC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}
/**
 * @brief  Returns the serial number string descriptor.
 * @param  speed: Current device speed
 * @param  length: Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t *USBD_APDKC_SerialStrDescriptor(USBD_SpeedTypeDef speed,
		uint16_t *length) {
	if (speed == USBD_SPEED_HIGH) {
		USBD_GetString((uint8_t *) USBD_SERIALNUMBER_HS_STRING, USBD_StrDesc,
				length);
	} else {
		USBD_GetString((uint8_t *) USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc,
				length);
	}
	return USBD_StrDesc;
}

uint8_t *USBD_MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_SERIALNUMBER_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}
/**
 * @brief  Returns the configuration string descriptor.
 * @param  speed: Current device speed
 * @param  length: Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t *USBD_APDKC_ConfigStrDescriptor(USBD_SpeedTypeDef speed,
		uint16_t *length) {
	if (speed == USBD_SPEED_HIGH) {
		USBD_GetString((uint8_t *) USBD_CONFIGURATION_HS_STRING, USBD_StrDesc,
				length);
	} else {
		USBD_GetString((uint8_t *) USBD_CONFIGURATION_FS_STRING, USBD_StrDesc,
				length);
	}
	return USBD_StrDesc;
}

uint8_t *USBD_MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_MSCCONFIGURATION_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_MSCCONFIGURATION_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
 * @brief  Returns the interface string descriptor.
 * @param  speed: Current device speed
 * @param  length: Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t *USBD_APDKC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed,
		uint16_t *length) {
	if (speed == 0) {
		USBD_GetString((uint8_t *) USBD_INTERFACE_HS_STRING, USBD_StrDesc,
				length);
	} else {
		USBD_GetString((uint8_t *) USBD_INTERFACE_FS_STRING, USBD_StrDesc,
				length);
	}
	return USBD_StrDesc;
}

uint8_t *USBD_MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_MSCINTERFACE_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_MSCINTERFACE_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

