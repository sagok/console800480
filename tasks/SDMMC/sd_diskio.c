/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @author  sagok
  * @version V1.0.0
  * @date    26-February-2015
  * @brief   SD Disk I/O driver
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <integer.h>
#include "ff_gen_drv.h"

#include "sd_diskio.h"
#include "stm32f2xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SD_DMAx_Tx_STREAM	  DMA2_Stream6
#define SD_DMAx_Rx_STREAM	  DMA2_Stream3
#define SD_DMAx_Tx_CHANNEL    DMA_CHANNEL_4
#define SD_DMAx_Rx_CHANNEL    DMA_CHANNEL_4

#define SD_DMAx_Tx_IRQn		  DMA2_Stream6_IRQn
#define SD_DMAx_Rx_IRQn		  DMA2_Stream3_IRQn
#define DMA_IRQHANDLERRx      DMA2_Stream3_IRQHandler
#define DMA_IRQHANDLERTx      DMA2_Stream6_IRQHandler

#define SD_SDIO_DMA_FLAG_FEIF         ((uint32_t)0x10400000)		//DMA_FLAG_FEIF3
#define SD_SDIO_DMA_FLAG_TCIF         ((uint32_t)0x18000000)		//DMA_FLAG_TCIF3

/* Block Size in Bytes */
#define BLOCK_SIZE                512

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile 		DSTATUS Stat = STA_NOINIT;
static uint32_t 		IsCardType =  SDIO_STD_CAPACITY_SD_CARD_V1_1;
static uint32_t 		CSD_Tab[4], CID_Tab[4], RCA = 0;

SDIO_InitTypeDef		SDIO_InitStructure;
SDIO_CmdInitTypeDef 	SDIO_CmdInitStructure;
SDIO_DataInitTypeDef 	SDIO_DataInitStructure;

extern SD_HandleTypeDef 		uSdHandle;
extern HAL_SD_CardInfoTypedef SD_CardInfo;

static uint32_t DeviceMode = SD_DMA_MODE;						 // Изменить для старта с этого значения

/* Private function prototypes -----------------------------------------------*/
//extern void Error_Handler(void);

FlagStatus 	SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void 		SDIO_ClearFlag(uint32_t SDIO_FLAG);

static HAL_SD_ErrorTypedef CmdError(void);
static HAL_SD_ErrorTypedef CmdResp1Error(uint8_t cmd);
static HAL_SD_ErrorTypedef CmdResp2Error(void);
static HAL_SD_ErrorTypedef CmdResp3Error(void);
static HAL_SD_ErrorTypedef CmdResp6Error(uint8_t cmd, uint16_t *prca);
static HAL_SD_ErrorTypedef CmdResp7Error(void);
static HAL_SD_ErrorTypedef SDEnWideBus(FunctionalState NewState);
static HAL_SD_ErrorTypedef FindSCR(uint16_t rca, uint32_t *pscr);

//HAL_SD_ErrorTypedef SD_Init(void);
HAL_SD_ErrorTypedef SDIO_PowerON(void);
HAL_SD_ErrorTypedef SD_InitializeCards(void);
HAL_SD_ErrorTypedef SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo);
HAL_SD_ErrorTypedef SD_SelectDeselect(uint32_t addr);
HAL_SD_ErrorTypedef SD_EnableWideBusOperation(uint32_t WideMode);
HAL_SD_ErrorTypedef SD_SendStatus(uint32_t *pcardstatus);

uint8_t 			SD_Detect(void);
//void 				SD_LowLevel_DeInit(void);
void 				SD_LowLevel_Init(void);
HAL_SD_ErrorTypedef SD_SetDeviceMode(uint32_t Mode);

void 						BSP_SD_GetCardInfo(HAL_SD_CardInfoTypedef *CardInfo);
uint8_t 					BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t 					BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void);
uint8_t 					BSP_SD_Init(void);

void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
void SDIO_DMACmd(FunctionalState NewState);
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);
/*********************************************************************************/
DSTATUS SD_initialize (void);
DSTATUS SD_status (void);
DRESULT SD_read (BYTE*, DWORD, BYTE);
#if _USE_WRITE == 1
  DRESULT SD_write (const BYTE*, DWORD, BYTE);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SD_ioctl (BYTE, void*);
#endif  /* _USE_IOCTL == 1 */
  
  /*********************************************************************************/
Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/****************************************************
* @brief  Initializes a Drive
* @param  None
* @retval DSTATUS: Operation status
*****************************************************/
DSTATUS SD_initialize(void)
{
	Stat = STA_NOINIT;

	/* Configure the uSD device */
	if(BSP_SD_Init() == MSD_OK)
	{
	Stat &= ~STA_NOINIT;
	}
	return Stat;
}

/****************************************************
* @brief  Gets Disk Status
* @param  None
* @retval DSTATUS: Operation status
*****************************************************/
DSTATUS SD_status(void)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetStatus() == 0)
  {
    Stat &= ~STA_NOINIT;
  } else printf("[SD] I:SD_GetStatus = SD_TRANSFER NOT OK\n");
  
  return Stat;
}

/****************************************************
* @brief  Reads Sector(s)
* @param  *buff: Data buffer to store read data
* @param  sector: Sector address (LBA)
* @param  count: Number of sectors to read (1..128)
* @retval DRESULT: Operation result
*****************************************************/
DRESULT SD_read(BYTE *buff, DWORD sector, BYTE count)
{
//	printf("[SD] I:SD_read: %d, %d, %d \n",*buff,(int)(sector),count);

	  uint32_t timeout = 100000;
	  DWORD scratch [BLOCK_SIZE / 4];  /* Alignment ensured, need enough stack */
	  uint8_t SD_state = MSD_OK;

	  if ((DWORD)buff & 3) /* DMA Alignment issue, do single up to aligned buffer */
	  {
	    while (count--)
	    {
	      SD_state = BSP_SD_ReadBlocks_DMA((uint32_t*)scratch, (uint64_t) ((sector + count) * BLOCK_SIZE), BLOCK_SIZE, 1);
//	  	printf("[SD] I:SD_read SD_state:%d \n",SD_state);

	      while(BSP_SD_GetStatus() != SD_TRANSFER_OK)
	      {
	        if (timeout-- == 0)
	        {
	          return RES_ERROR;
	        }
	      }
	      memcpy (&buff[count * BLOCK_SIZE] ,scratch, BLOCK_SIZE);
	    }
	  }
	  else
	  {
	    SD_state = BSP_SD_ReadBlocks_DMA((uint32_t*)buff, (uint64_t) (sector * BLOCK_SIZE), BLOCK_SIZE, count);

	    while(BSP_SD_GetStatus() != SD_TRANSFER_OK)
	    {
	      if (timeout-- == 0)
	      {
	        return RES_ERROR;
	      }
	    }
	  }
	  if (SD_state == MSD_OK)
	  {
	    return RES_OK;
	  }

	  return RES_ERROR;
}

/****************************************************
* @brief  Writes Sector(s)
* @param  *buff: Data to be written
* @param  sector: Sector address (LBA)
* @param  count: Number of sectors to write (1..128)
* @retval DRESULT: Operation result
*****************************************************/
#if _USE_WRITE == 1
DRESULT SD_write(const BYTE *buff, DWORD sector, BYTE count)
{
	 uint32_t timeout = 100000;
	  DWORD scratch [BLOCK_SIZE / 4];  /* Alignment ensured, need enough stack */
	  uint8_t SD_state = MSD_OK;

	  if ((DWORD)buff & 3) /* DMA Alignment issue, do single up to aligned buffer */
	  {
	    while (count--)
	    {
	      memcpy (scratch, &buff[count * BLOCK_SIZE], BLOCK_SIZE);
	      SD_state = BSP_SD_WriteBlocks_DMA((uint32_t*)scratch, (uint64_t)((sector + count) * BLOCK_SIZE), BLOCK_SIZE, 1);
	      while(BSP_SD_GetStatus() != SD_TRANSFER_OK)
	      {
	        if (timeout-- == 0)
	        {
	          return RES_ERROR;
	        }
	      }
	    }
	  }
	  else
	  {
	    SD_state = BSP_SD_WriteBlocks_DMA((uint32_t*)buff, (uint64_t)(sector * BLOCK_SIZE), BLOCK_SIZE, count);
	    while(BSP_SD_GetStatus() != SD_TRANSFER_OK)
	    {
	      if (timeout-- == 0)
	      {
	        return RES_ERROR;
	      }
	    }
	  }
	  if (SD_state == MSD_OK)
	  {
	    return RES_OK;
	  }
	  return RES_ERROR;
}
#endif /* _USE_WRITE == 1 */

/****************************************************
* @brief  I/O control operation
* @param  cmd: Control code
* @param  *buff: Buffer to send/receive control data
* @retval DRESULT: Operation result
*****************************************************/
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  HAL_SD_CardInfoTypedef CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;
  
  switch (cmd)
  {
  /* Make sure that no pending write process */
  /* по этой команде устройство должно записать данные, хранящиеся в кэше и завершить запись.
   * Мы работаем с картой напрямую без кэша, данные записываем и всегда дожидаемся окончания
   * записи, поэтому по этой команде можно ничего не делать.*/
  case CTRL_SYNC :
    res = RES_OK;
    break;
  
  /* Get number of sectors on the disk (DWORD) */
  /* команда должна возвратить количество секторов на карте памяти. Берём SDCardInfo.CardCapacity
   * и делим на размер сектора SDCardInfo.CardBlockSize (всегда 512).*/
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff =  CardInfo.CardCapacity / BLOCK_SIZE;//CardInfo.CardCapacity / CardInfo.CardBlockSize;// = CardInfo.CardCapacity / CardInfo.CardBlockSize;		// BLOCK_SIZE

    res = RES_OK;
    break;
  
  /* Get R/W sector size (WORD) */
  /* требуется возвратить размер сектора. Для режима _MAX_SS=512 (этот параметр всегда указывается
   * при работе с картой памяти) команда GET_SECTOR_SIZE не используется и реализовывать её также не требуется.*/
  case GET_SECTOR_SIZE :
    *(WORD*)buff = BLOCK_SIZE;//BLOCK_SIZE;
    res = RES_OK;
    break;
  
  /* Get erase block size in unit of sector (DWORD) */
  /* запрос количества секторов, объединяемых в один блок, который можно стереть.
   * Параметр используется в функции f_mkfs для правильной разметки диска.*/
  case GET_BLOCK_SIZE :
	  BSP_SD_GetCardInfo(&CardInfo);
	  *(DWORD*)buff = BLOCK_SIZE;//BLOCK_SIZE; //CardInfo.CardBlockSize;//BLOCK_SIZE;
	  break;

  case CTRL_ERASE_SECTOR:
	    res = RES_OK;
	break;

  default:
    res = RES_PARERR;
  }
  
  return res;
}
#endif /* _USE_IOCTL == 1 */
  
/****************************************************************
* @brief  Initializes the SD Card and put it into StandBy State
* 			(Ready for data transfer).
* @param  None
* @retval None
*****************************************************************/
void SD_LowLevel_Init(void)
{
 static DMA_HandleTypeDef dmaRxHandle;
static DMA_HandleTypeDef dmaTxHandle;
GPIO_InitTypeDef  GPIO_InitStructure;
SD_HandleTypeDef *hsd = &uSdHandle;


/* Enable the SDIO Clock */
__SDIO_CLK_ENABLE();
__SDIO_ENABLE();

/* Enable the DMA2 Clock */
__DMA2_CLK_ENABLE();

/* Enable GPIOs clock */
__GPIOB_CLK_ENABLE();
__GPIOC_CLK_ENABLE();
__GPIOD_CLK_ENABLE();
//__SD_DETECT_GPIO_CLK_ENABLE();


/* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins PC.12 pin: CLK pin */
GPIO_InitStructure.Pin 			= GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 ;//| GPIO_PIN_12;
GPIO_InitStructure.Speed 		= GPIO_SPEED_HIGH;
GPIO_InitStructure.Mode 		= GPIO_MODE_AF_PP;//GPIO_MODE_AF_PP;//GPIO_Mode_AF;GPIO_OType_PP
GPIO_InitStructure.Pull 		= GPIO_PULLUP;
GPIO_InitStructure.Alternate 	= GPIO_AF12_SDIO;
HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

/* Configure PC.12 pin: CLK pin */
GPIO_InitStructure.Pin = GPIO_PIN_12;
GPIO_InitStructure.Pull = GPIO_NOPULL;
GPIO_InitStructure.Alternate 	= GPIO_AF12_SDIO;
HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);


/* Configure PD.02 CMD line */
GPIO_InitStructure.Pin 			= GPIO_PIN_2;
GPIO_InitStructure.Alternate 	= GPIO_AF12_SDIO;
HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);


/*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
GPIO_InitStructure.Pin = SD_DETECT_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
GPIO_InitStructure.Pull = GPIO_PULLUP;
//GPIO_InitStructure.Alternate = ;
HAL_GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// добавим ДМА на чтение и запись
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* NVIC configuration for SDIO interrupts */
HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);		//5, 0)
HAL_NVIC_EnableIRQ(SDIO_IRQn);

/* Configure DMA Rx parameters */
dmaRxHandle.Init.Channel             = SD_DMAx_Rx_CHANNEL;
dmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
dmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
dmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
dmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
dmaRxHandle.Init.Mode                = DMA_PFCTRL;
dmaRxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
dmaRxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
dmaRxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
dmaRxHandle.Init.MemBurst            = DMA_MBURST_INC4;
dmaRxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

dmaRxHandle.Instance = SD_DMAx_Rx_STREAM;

/* Associate the DMA handle */
__HAL_LINKDMA(hsd, hdmarx, dmaRxHandle);

/* Deinitialize the stream for new transfer */
HAL_DMA_DeInit(&dmaRxHandle);

/* Configure the DMA stream */
HAL_DMA_Init(&dmaRxHandle);

/* Configure DMA Tx parameters */
dmaTxHandle.Init.Channel             = SD_DMAx_Tx_CHANNEL;
dmaTxHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
dmaTxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
dmaTxHandle.Init.MemInc              = DMA_MINC_ENABLE;
dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
dmaTxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
dmaTxHandle.Init.Mode                = DMA_PFCTRL;
dmaTxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
dmaTxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
dmaTxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
dmaTxHandle.Init.MemBurst            = DMA_MBURST_INC4;
dmaTxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

dmaTxHandle.Instance = SD_DMAx_Tx_STREAM;

/* Associate the DMA handle */
__HAL_LINKDMA(hsd, hdmatx, dmaTxHandle);

/* Deinitialize the stream for new transfer */
HAL_DMA_DeInit(&dmaTxHandle);

/* Configure the DMA stream */
HAL_DMA_Init(&dmaTxHandle);

/* NVIC configuration for DMA transfer complete interrupt */
HAL_NVIC_SetPriority(SD_DMAx_Rx_IRQn, 6, 0);		// 6, 0);
HAL_NVIC_EnableIRQ(SD_DMAx_Rx_IRQn);

/* NVIC configuration for DMA transfer complete interrupt */
HAL_NVIC_SetPriority(SD_DMAx_Tx_IRQn, 6, 0);		// 6, 0);
HAL_NVIC_EnableIRQ(SD_DMAx_Tx_IRQn);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Enable the SDIO Clock */
__SDIO_CLK_ENABLE();
__SDIO_ENABLE();

}

/****************************************************************
* @brief
* @param
* @param
* @retval
*****************************************************************/
void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypedef *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_Get_CardInfo(&uSdHandle, CardInfo);		// этот не читает паразит

/*
  printf("-------------------- BSP_SD_GetCardInfo --------------------------\n");
  printf("[SD] I:ManufacturerID: %d \n",CardInfo->SD_cid.ManufacturerID);
  printf("[SD] I:OEM/Application ID: %d \n",CardInfo->SD_cid.OEM_AppliID);
//  printf("[SD] I:Product Name part1: %d \n",CardInfo->SD_cid.ProdName1);
  printf("[SD] I:Product Name part2: %d \n",CardInfo->SD_cid.ProdName2);
  printf("[SD] I:Product Revision: %d \n",CardInfo->SD_cid.ProdRev);
//  printf("[SD] I:Product Serial Number: %d \n",CardInfo->SD_cid.ProdSN);
  printf("[SD] I:Manufacturing Date: %d \n",CardInfo->SD_cid.ManufactDate);
  printf("[SD] I:Device Size: %d.%d.%d.%d \n",(uint8_t)(CardInfo->SD_csd.DeviceSize>>24),(uint8_t)(CardInfo->SD_csd.DeviceSize>>16),(uint8_t)(CardInfo->SD_csd.DeviceSize>>8),(uint8_t)(CardInfo->SD_csd.DeviceSize));
  printf("[SD] I:CardType: %d \n",CardInfo->CardType);
  printf("[SD] I:CardBlockSize: %d \n",(uint16_t)CardInfo->CardBlockSize);
  printf("[SD] I:CardCapacity: %d.%d.%d.%d.%d.%d \n",(uint8_t)(CardInfo->CardCapacity>>40),(uint8_t)(CardInfo->CardCapacity>>32),(uint8_t)(CardInfo->CardCapacity>>24),(uint8_t)(CardInfo->CardCapacity>>16),(uint8_t)(CardInfo->CardCapacity>>8),(uint8_t)CardInfo->CardCapacity);
  printf("------------------------------------------------------------------\n");
*/
}

/**
  * @brief  Enquires cards about their operating voltage and configures
  *   clock controls.
  * @param  None
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
HAL_SD_ErrorTypedef SDIO_PowerON(void)
{

  __IO HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t response = 0, count = 0, validvoltage = 0;
  uint32_t SDType = SD_STD_CAPACITY;//SD_STD_CAPACITY;

  /*!< Power ON Sequence -----------------------------------------------------*/
  /*!< Configure the SDIO peripheral */
  /*!< SDIO_CK = SDIOCLK / (SDIO_INIT_CLK_DIV + 2) */
  /*!< on STM32F2xx devices, SDIOCLK is fixed to 48MHz */
  /*!< SDIO_CK for initialization should not exceed 400 KHz */
  SDIO_InitStructure.ClockDiv = SDIO_INIT_CLK_DIV;
  SDIO_InitStructure.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  SDIO_InitStructure.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  SDIO_InitStructure.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;//SDIO_CLOCK_POWER_SAVE_DISABLE;
  SDIO_InitStructure.BusWide = SDIO_BUS_WIDE_1B;
  SDIO_InitStructure.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;//SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  SDIO_Init(SDIO, SDIO_InitStructure);

  /*!< Set Power State to ON */
  SDIO_PowerState_ON(SDIO);

  /*!< Enable SDIO Clock */
  __SDIO_CLK_ENABLE();
  __SDIO_ENABLE();

  /*!< CMD0: GO_IDLE_STATE ---------------------------------------------------*/
  /*!< No CMD response required */
  SDIO_CmdInitStructure.Argument = 0x0;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_GO_IDLE_STATE;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_NO;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdError();

  if (errorstatus != SD_OK)
  {
    /*!< CMD Response TimeOut (wait for CMDSENT flag) */

    return(errorstatus);
  }

  /*!< CMD8: SEND_IF_COND ----------------------------------------------------*/
  /*!< Send CMD8 to verify SD card interface operating condition */
  /*!< Argument: - [31:12]: Reserved (shall be set to '0')
               - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
               - [7:0]: Check Pattern (recommended 0xAA) */
  /*!< CMD Response: R7 */
  SDIO_CmdInitStructure.Argument = SD_CHECK_PATTERN;
  SDIO_CmdInitStructure.CmdIndex = SDIO_SEND_IF_COND;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp7Error();

  if (errorstatus == SD_OK)
  {
	IsCardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /*!< SD Card 2.0 */
    SDType = SD_HIGH_CAPACITY;
  }
  else
  {
    /*!< CMD55 */
    SDIO_CmdInitStructure.Argument = 0x00;
    SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
    SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
  }
  /*!< CMD55 */
  SDIO_CmdInitStructure.Argument = 0x00;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_CMD;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

  /*!< If errorstatus is Command TimeOut, it is a MMC card */
  /*!< If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
     or SD card 1.x */
  if (errorstatus == SD_OK)
  {
    /*!< SD CARD */
    /*!< Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
    while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
    {

      /*!< SEND CMD55 APP_CMD with RCA as 0 */
      SDIO_CmdInitStructure.Argument = 0x00;
      SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
      SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
      SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
      SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      SDIO_CmdInitStructure.Argument = SD_VOLTAGE_WINDOW_SD | SDType;
      SDIO_CmdInitStructure.CmdIndex = SD_CMD_SD_APP_OP_COND;
      SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
      SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
      SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
      SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

      errorstatus = CmdResp3Error();
      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      response = SDIO_GetResponse(SDIO_RESP1);
      validvoltage = (((response >> 31) == 1) ? 1 : 0);
      count++;
    }
    if (count >= SD_MAX_VOLT_TRIAL)
    {
      errorstatus = SD_INVALID_VOLTRANGE;
      return(errorstatus);
    }

    if (response &= SD_HIGH_CAPACITY)
    {
    	IsCardType = SDIO_HIGH_CAPACITY_SD_CARD;
    }

  }/*!< else MMC Card */

  return(errorstatus);
}
/**
  * @brief  Checks for error conditions for CMD0.
  * @param  None
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef CmdError(void)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t timeout;

  timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */

  while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))
  {
    timeout--;
  }

  if (timeout == 0)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/**
  * @brief  Checks whether the specified SDIO flag is set or not.
  * @param  SDIO_FLAG: specifies the flag to check.
  * @retval The new state of SDIO_FLAG (SET or RESET).
  */
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_SDIO_FLAG(SDIO_FLAG));

  if ((SDIO->STA & SDIO_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}
/**
  * @brief  Clears the SDIO's pending flags.
  * @param  SDIO_FLAG: specifies the flag to clear.
  * @retval None
  */
void SDIO_ClearFlag(uint32_t SDIO_FLAG)
{
  /* Check the parameters */
  assert_param(IS_SDIO_CLEAR_FLAG(SDIO_FLAG));

  SDIO->ICR = SDIO_FLAG;
}

/**
  * @brief  Checks for error conditions for R1 response.
  * @param  cmd: The sent command index.
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef CmdResp1Error(uint8_t cmd)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Check response received is of desired command */
  if (SDIO_GetCommandResponse(SDIO) != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /*!< We have received response, retrieve it for analysis  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

  if ((response_r1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
  {
    return(errorstatus);
  }

  if (response_r1 & SD_OCR_ADDR_OUT_OF_RANGE)
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (response_r1 & SD_OCR_ADDR_MISALIGNED)
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (response_r1 & SD_OCR_BLOCK_LEN_ERR)
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (response_r1 & SD_OCR_ERASE_SEQ_ERR)
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (response_r1 & SD_OCR_BAD_ERASE_PARAM)
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (response_r1 & SD_OCR_WRITE_PROT_VIOLATION)
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (response_r1 & SD_OCR_LOCK_UNLOCK_FAILED)
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (response_r1 & SD_OCR_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  if (response_r1 & SD_OCR_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_OCR_CARD_ECC_FAILED)
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (response_r1 & SD_OCR_CC_ERROR)
  {
    return(SD_CC_ERROR);
  }

  if (response_r1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_OCR_STREAM_READ_UNDERRUN)
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (response_r1 & SD_OCR_STREAM_WRITE_OVERRUN)
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (response_r1 & SD_OCR_CID_CSD_OVERWRIETE)
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (response_r1 & SD_OCR_WP_ERASE_SKIP)
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (response_r1 & SD_OCR_CARD_ECC_DISABLED)
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (response_r1 & SD_OCR_ERASE_RESET)
  {
    return(SD_ERASE_RESET);
  }

  if (response_r1 & SD_OCR_AKE_SEQ_ERROR)
  {
    return(SD_AKE_SEQ_ERROR);
  }
  return(errorstatus);
}
/**
  * @brief  Checks for error conditions for R3 (OCR) response.
  * @param  None
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef CmdResp3Error(void)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  return(errorstatus);
}
/**
  * @brief  Intialises all cards or single card as the case may be Card(s) come
  *         into standby state.
  * @param  None
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_InitializeCards(void)
{
	HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint16_t rca = 0x01;

  if (!SDIO_GetPowerState(SDIO) )
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }

  if (SDIO_SECURE_DIGITAL_IO_CARD != IsCardType)
  {
    /*!< Send CMD2 ALL_SEND_CID */
    SDIO_CmdInitStructure.Argument = 0x0;
    SDIO_CmdInitStructure.CmdIndex = SD_CMD_ALL_SEND_CID;
    SDIO_CmdInitStructure.Response = SDIO_RESPONSE_LONG;
    SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == IsCardType) ||  (SDIO_STD_CAPACITY_SD_CARD_V2_0 == IsCardType) ||  (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == IsCardType)
      ||  (SDIO_HIGH_CAPACITY_SD_CARD == IsCardType))
  {
    /*!< Send CMD3 SET_REL_ADDR with argument 0 */
    /*!< SD Card publishes its RCA. */
    SDIO_CmdInitStructure.Argument = 0x00;
    SDIO_CmdInitStructure.CmdIndex = SD_CMD_SET_REL_ADDR;
    SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
    SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

    errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }

  if (SDIO_SECURE_DIGITAL_IO_CARD != IsCardType)
  {
    RCA = rca;

    /*!< Send CMD9 SEND_CSD with argument as card's RCA */
    SDIO_CmdInitStructure.Argument = (uint32_t)(rca << 16);
    SDIO_CmdInitStructure.CmdIndex = SD_CMD_SEND_CSD;
    SDIO_CmdInitStructure.Response = SDIO_RESPONSE_LONG;
    SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
    SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }

  errorstatus = SD_OK; /*!< All cards get intialized */

  return(errorstatus);
}
/**
  * @brief  Checks for error conditions for R6 (RCA) response.
  * @param  cmd: The sent command index.
  * @param  prca: pointer to the variable that will contain the SD card relative
  *         address RCA.
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef CmdResp6Error(uint8_t cmd, uint16_t *prca)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Check response received is of desired command */
  if (SDIO_GetCommandResponse(SDIO) != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /*!< We have received response, retrieve it.  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

  if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
  {
    *prca = (uint16_t) (response_r1 >> 16);
    return(errorstatus);
  }

  if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_R6_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_R6_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  return(errorstatus);
}
/**
  * @brief  Checks for error conditions for R7 response.
  * @param  None
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef CmdResp7Error(void)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t status;
  uint32_t timeout = SDIO_CMD0TIMEOUT;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
  {
    timeout--;
    status = SDIO->STA;
  }

  if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
  {
    /*!< Card is not V2.0 complient or card does not support the set voltage range */
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }

  if (status & SDIO_FLAG_CMDREND)
  {
    /*!< Card is SD V2.0 compliant */
    errorstatus = SD_OK;
    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    return(errorstatus);
  }
  return(errorstatus);
}
/**
  * @brief  Returns information about specific card.
  * @param  cardinfo: pointer to a SD_CardInfo structure that contains all SD card
  *         information.
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint8_t tmp = 0;

  cardinfo->CardType = (uint8_t)IsCardType;
  cardinfo->RCA = (uint16_t)RCA;

  /*!< Byte 0 */
  tmp = (uint8_t)((CSD_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
  cardinfo->SD_csd.Reserved1 = tmp & 0x03;

  /*!< Byte 1 */
  tmp = (uint8_t)((CSD_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.TAAC = tmp;

  /*!< Byte 2 */
  tmp = (uint8_t)((CSD_Tab[0] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.NSAC = tmp;

  /*!< Byte 3 */
  tmp = (uint8_t)(CSD_Tab[0] & 0x000000FF);
  cardinfo->SD_csd.MaxBusClkFrec = tmp;

  /*!< Byte 4 */
  tmp = (uint8_t)((CSD_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CardComdClasses = tmp << 4;

  /*!< Byte 5 */
  tmp = (uint8_t)((CSD_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
  cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

  /*!< Byte 6 */
  tmp = (uint8_t)((CSD_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.Reserved2 = 0; /*!< Reserved */

  if ((IsCardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (IsCardType == SDIO_STD_CAPACITY_SD_CARD_V2_0))
  {
    cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

    /*!< Byte 7 */
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

    /*!< Byte 8 */
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

    cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
    cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

    /*!< Byte 9 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
    cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
    /*!< Byte 10 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
    cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
    cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
    cardinfo->CardCapacity *= cardinfo->CardBlockSize;
  }
  else if (IsCardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    /*!< Byte 7 */
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

    /*!< Byte 8 */
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);

    cardinfo->SD_csd.DeviceSize |= (tmp << 8);

    /*!< Byte 9 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);

    cardinfo->SD_csd.DeviceSize |= (tmp);

    /*!< Byte 10 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);

    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
    cardinfo->CardBlockSize = 512;
  }


  cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

  /*!< Byte 11 */
  tmp = (uint8_t)(CSD_Tab[2] & 0x000000FF);
  cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

  /*!< Byte 12 */
  tmp = (uint8_t)((CSD_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
  cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
  cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

  /*!< Byte 13 */
  tmp = (uint8_t)((CSD_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.Reserved3 = 0;
  cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

  /*!< Byte 14 */
  tmp = (uint8_t)((CSD_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
  cardinfo->SD_csd.ECC = (tmp & 0x03);

  /*!< Byte 15 */
  tmp = (uint8_t)(CSD_Tab[3] & 0x000000FF);
  cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_csd.Reserved4 = 1;


  /*!< Byte 0 */
  tmp = (uint8_t)((CID_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ManufacturerID = tmp;

  /*!< Byte 1 */
  tmp = (uint8_t)((CID_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.OEM_AppliID = tmp << 8;

  /*!< Byte 2 */
  tmp = (uint8_t)((CID_Tab[0] & 0x000000FF00) >> 8);
  cardinfo->SD_cid.OEM_AppliID |= tmp;

  /*!< Byte 3 */
  tmp = (uint8_t)(CID_Tab[0] & 0x000000FF);
  cardinfo->SD_cid.ProdName1 = tmp << 24;

  /*!< Byte 4 */
  tmp = (uint8_t)((CID_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdName1 |= tmp << 16;

  /*!< Byte 5 */
  tmp = (uint8_t)((CID_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdName1 |= tmp << 8;

  /*!< Byte 6 */
  tmp = (uint8_t)((CID_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdName1 |= tmp;

  /*!< Byte 7 */
  tmp = (uint8_t)(CID_Tab[1] & 0x000000FF);
  cardinfo->SD_cid.ProdName2 = tmp;

  /*!< Byte 8 */
  tmp = (uint8_t)((CID_Tab[2] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdRev = tmp;

  /*!< Byte 9 */
  tmp = (uint8_t)((CID_Tab[2] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdSN = tmp << 24;

  /*!< Byte 10 */
  tmp = (uint8_t)((CID_Tab[2] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdSN |= tmp << 16;

  /*!< Byte 11 */
  tmp = (uint8_t)(CID_Tab[2] & 0x000000FF);
  cardinfo->SD_cid.ProdSN |= tmp << 8;

  /*!< Byte 12 */
  tmp = (uint8_t)((CID_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdSN |= tmp;

  /*!< Byte 13 */
  tmp = (uint8_t)((CID_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
  cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

  /*!< Byte 14 */
  tmp = (uint8_t)((CID_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ManufactDate |= tmp;

  /*!< Byte 15 */
  tmp = (uint8_t)(CID_Tab[3] & 0x000000FF);
  cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_cid.Reserved2 = 1;

  return(errorstatus);
}

/**
  * @brief  Selects od Deselects the corresponding card.
  * @param  addr: Address of the Card to be selected.
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_SelectDeselect(uint32_t addr)
{
	HAL_SD_ErrorTypedef errorstatus = SD_OK;

  /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
  SDIO_CmdInitStructure.Argument =  addr;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_SEL_DESEL_CARD;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);

  return(errorstatus);
}
/**
  * @brief  Enables wide bus opeartion for the requeseted card if supported by
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode.
  *   This parameter can be one of the following values:
  *     @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
  *     @arg SDIO_BusWide_4b: 4-bit data transfer
  *     @arg SDIO_BusWide_1b: 1-bit data transfer
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_EnableWideBusOperation(uint32_t WideMode)
{
	HAL_SD_ErrorTypedef errorstatus = SD_OK;

  /*!< MMC Card doesn't support this feature */
  if (SDIO_MULTIMEDIA_CARD == IsCardType)
  {
    errorstatus = SD_UNSUPPORTED_FEATURE;
    return(errorstatus);
  }
  else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == IsCardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == IsCardType) || (SDIO_HIGH_CAPACITY_SD_CARD == IsCardType))
  {
    if (SDIO_BUS_WIDE_8B == WideMode)
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }
    else if (SDIO_BUS_WIDE_4B == WideMode)
    {
      errorstatus = SDEnWideBus(ENABLE);

      if (SD_OK == errorstatus)
      {
        /*!< Configure the SDIO peripheral */
        SDIO_InitStructure.ClockDiv = SDIO_TRANSFER_CLK_DIV;
        SDIO_InitStructure.ClockEdge = SDIO_CLOCK_EDGE_RISING;
        SDIO_InitStructure.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
        SDIO_InitStructure.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
        SDIO_InitStructure.BusWide = SDIO_BUS_WIDE_4B;
        SDIO_InitStructure.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
        SDIO_Init(SDIO, SDIO_InitStructure);
      }
    }
    else
    {
      errorstatus = SDEnWideBus(DISABLE);

      if (SD_OK == errorstatus)
      {
          /*!< Configure the SDIO peripheral */
          SDIO_InitStructure.ClockDiv = SDIO_TRANSFER_CLK_DIV;
          SDIO_InitStructure.ClockEdge = SDIO_CLOCK_EDGE_RISING;
          SDIO_InitStructure.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
          SDIO_InitStructure.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
          SDIO_InitStructure.BusWide = SDIO_BUS_WIDE_1B;
          SDIO_InitStructure.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
          SDIO_Init(SDIO, SDIO_InitStructure);
      }
    }
  }

  return(errorstatus);
}
/**
  * @brief  Checks for error conditions for R2 (CID or CSD) response.
  * @param  None
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef CmdResp2Error(void)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}
/**
  * @brief  Enables or disables the SDIO wide bus mode.
  * @param  NewState: new state of the SDIO wide bus mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef SDEnWideBus(FunctionalState NewState)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;

  uint32_t scr[2] = {0, 0};

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /*!< Get SCR Register */
  errorstatus = FindSCR(RCA, scr);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /*!< If wide bus operation to be enabled */
  if (NewState == ENABLE)
  {
    /*!< If requested card supports wide bus operation */
    if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
      SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
      SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
      SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
      SDIO_CmdInitStructure.Argument = 0x2;
      SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
      SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
      SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
      SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }   /*!< If wide bus operation to be disabled */
  else
  {
    /*!< If requested card supports 1 bit mode operation */
    if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
      SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
      SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
      SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
      SDIO_CmdInitStructure.Argument = 0x00;
      SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
      SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
      SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
      SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }
}
/**
  * @brief  Find the SD card SCR register value.
  * @param  rca: selected card address.
  * @param  pscr: pointer to the buffer that will contain the SCR value.
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
static HAL_SD_ErrorTypedef FindSCR(uint16_t rca, uint32_t *pscr)
{
  uint32_t index = 0;
  HAL_SD_ErrorTypedef errorstatus = SD_OK;
  uint32_t tempscr[2] = {0, 0};

  /*!< Set Block Size To 8 Bytes */
  /*!< Send CMD55 APP_CMD with argument as card's RCA */
  SDIO_CmdInitStructure.Argument = (uint32_t)8;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /*!< Send CMD55 APP_CMD with argument as card's RCA */
  SDIO_CmdInitStructure.Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_APP_CMD;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  SDIO_DataInitStructure.DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.DataLength = 8;
  SDIO_DataInitStructure.DataBlockSize = SDIO_DATABLOCK_SIZE_8B;
  SDIO_DataInitStructure.TransferDir = SDIO_TRANSFER_DIR_TO_SDIO;
  SDIO_DataInitStructure.TransferMode = SDIO_TRANSFER_MODE_BLOCK;
  SDIO_DataInitStructure.DPSM = SDIO_DPSM_ENABLE;
  SDIO_DataConfig(SDIO, &SDIO_DataInitStructure);


  /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
  SDIO_CmdInitStructure.Argument = 0x0;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_SD_APP_SEND_SCR;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
  {
    if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
    {
      *(tempscr + index) = SDIO_ReadFIFO(SDIO);
      index++;
    }
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

  *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

  return(errorstatus);
}


/**
 * @brief  Detect if SD card is correctly plugged in the memory slot.
 * @param  None
 * @retval Return if SD is detected or not
 */
uint8_t SD_Detect(void)
{
  __IO uint8_t status = SD_PRESENT;

  /*!< Check GPIO to detect SD */
  if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET)
  {
//	LED_High(LED11);
    status = SD_NOT_PRESENT;
  }
  return status;
}
/**
  * @brief  Returns the current card's status.
  * @param  pcardstatus: pointer to the buffer that will contain the SD card
  *         status (Card Status register).
  * @retval HAL_SD_ErrorTypedef: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_SendStatus(uint32_t *pcardstatus)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;

  if (pcardstatus == NULL)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  SDIO_CmdInitStructure.Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_SEND_STATUS;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  *pcardstatus = SDIO_GetResponse(SDIO_RESP1);

  return(errorstatus);
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  uint8_t SD_state = MSD_OK;

//	printf("[SD] I:BSP_SD_ReadBlocks_DMA....\n");


  /* Read block(s) in DMA transfer mode */
  if(HAL_SD_ReadBlocks_DMA(&uSdHandle, pData, ReadAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
    SD_state = MSD_ERROR;
  }

  /* Wait until transfer is complete */
  if(SD_state == MSD_OK)
  {
    if(HAL_SD_CheckReadOperation(&uSdHandle, (uint32_t)SD_DATATIMEOUT) != SD_OK)
    {
      SD_state = MSD_ERROR;
    }
    else
    {
      SD_state = MSD_OK;
    }
  }

//	printf("[SD] I:BSP_SD_ReadBlocks_DMA: %d\n",SD_state);

  return SD_state;
}
/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  *            @arg  SD_TRANSFER_ERROR: Data transfer error
  */
HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void)
{
  return(HAL_SD_GetStatus(&uSdHandle));
}
/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  uint8_t SD_state = MSD_OK;

//  printf("[SD] I:BSP_SD_WriteBlocks_DMA -  go BSP_SD_WriteBlocks_DMA..CardType %d  \n", uSdHandle.CardType);

  /* Write block(s) in DMA transfer mode */
  if(HAL_SD_WriteBlocks_DMA(&uSdHandle, pData, WriteAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
		printf("[SD] E:Write block(s) in DMA transfer mode ... error \n");

    SD_state = MSD_ERROR;
  } else{
//		printf("[SD] I:Write block(s) in DMA transfer mode ... ok \n");
  }

  /* Wait until transfer is complete */
  if(SD_state == MSD_OK)
  {
    if(HAL_SD_CheckWriteOperation(&uSdHandle, (uint32_t)SD_DATATIMEOUT) != SD_OK)
    {
		printf("[SD] E:Wait until transfer is complete ... error \n");
      SD_state = MSD_ERROR;
    }
    else
    {
//	 printf("[SD] I:Wait until transfer is complete ... ok \n");

      SD_state = MSD_OK;
    }
  }

  return SD_state;
}

/**
  * @brief  Sets device mode whether to operate in Polling, Interrupt or DMA mode.
  * @param  Mode: Specifies the Data Transfer mode.
  *   This parameter can be one of the following values:
  *     @arg SD_DMA_MODE: Data transfer using DMA.
  *     @arg SD_INTERRUPT_MODE: Data transfer using interrupts.
  *     @arg SD_POLLING_MODE: Data transfer using flags.
  * @retval SD_Error: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_SetDeviceMode(uint32_t Mode)
{
	HAL_SD_ErrorTypedef errorstatus = SD_OK;

  if ((Mode == SD_DMA_MODE) || (Mode == SD_INTERRUPT_MODE) || (Mode == SD_POLLING_MODE))
  {
    DeviceMode = Mode;
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
  }
  return(errorstatus);

}
/**
  * @brief  Handles SD card interrupt request.
  * @param  None
  * @retval None
  */
void BSP_SD_IRQHandler(void)
{
// LED_Toggle(LED0);
  HAL_SD_IRQHandler(&uSdHandle);
}

/**
  * @brief  Handles SD DMA Tx transfer interrupt request.
  * @param  None
  * @retval None
  */
void BSP_SD_DMA_Tx_IRQHandler(void)
{
// LED_Toggle(LED1);
  HAL_DMA_IRQHandler(uSdHandle.hdmatx);
}

/**
  * @brief  Handles SD DMA Rx transfer interrupt request.
  * @param  None
  * @retval None
  */
void BSP_SD_DMA_Rx_IRQHandler(void)
{
//  LED_Toggle(LED2);
  HAL_DMA_IRQHandler(uSdHandle.hdmarx);
}


/**
  * @brief  Initializes the SD card device.
  * @param  None
  * @retval SD status.
  */
uint8_t BSP_SD_Init(void)
{
  uint8_t SD_state = MSD_OK;

  /* uSD device interface configuration */
  uSdHandle.Instance = SDIO;

  uSdHandle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  uSdHandle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
  uSdHandle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
  uSdHandle.Init.BusWide             = SDIO_BUS_WIDE_1B;
  uSdHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  uSdHandle.Init.ClockDiv            = SDIO_TRANSFER_CLK_DIV;

  /* Check if the SD card is plugged in the slot */
  if(SD_Detect() != SD_PRESENT)
  {
    return MSD_ERROR;
  }

  /* HAL SD initialization */
  SD_LowLevel_Init();	//SD_MspInit();

  if(HAL_SD_Init(&uSdHandle, &SD_CardInfo) != SD_OK)
  {
    SD_state = MSD_ERROR;
  }

  /* Configure SD Bus width */
  if(SD_state == MSD_OK)
  {
    /* Enable wide operation */
    if(HAL_SD_WideBusOperation_Config(&uSdHandle, SDIO_BUS_WIDE_4B) != SD_OK)
    {
      SD_state = MSD_ERROR;
    }
    else
    {
      SD_state = MSD_OK;
    }
  }

  return  SD_state;
}


/**
  * @brief  Allows to read blocks from a specified address  in a card.  The Data
  *         transfer can be managed by DMA mode or Polling mode.
  * @note   This operation should be followed by two functions to check if the
  *         DMA Controller and SD Card status.
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controller has finished all data transfer.
  *          - SD_GetStatus(): to check that the SD Card has finished the
  *            data transfer and it is ready for data.
  * @param  readbuff: pointer to the buffer that will contain the received data.
  * @param  ReadAddr: Address from where data are to be read.
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @param  NumberOfBlocks: number of blocks to be read.
  * @retval SD_Error: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_ReadMultiBlocks(uint8_t *readbuff, uint64_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  HAL_SD_ErrorTypedef errorstatus = SD_OK;


  SDIO->DCTRL = 0x0;

  SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
  SD_LowLevel_DMA_RxConfig((uint32_t *)readbuff, (NumberOfBlocks * BlockSize));				// нужна настройка чтения блока через DMA
  SDIO_DMACmd(ENABLE);

  if (IsCardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    ReadAddr /= 512;
  }

  /*!< Set Block Size for Card */
  SDIO_CmdInitStructure.Argument = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (SD_OK != errorstatus)
  {
	printf("[SD] E:SD_ReadMultiBlocks 1. %d \n",errorstatus);
    return(errorstatus);
  }

  SDIO_DataInitStructure.DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.DataLength = NumberOfBlocks * BlockSize;
  SDIO_DataInitStructure.DataBlockSize = (uint32_t) 9 << 4;
  SDIO_DataInitStructure.TransferDir = SDIO_TRANSFER_DIR_TO_SDIO;
  SDIO_DataInitStructure.TransferMode = SDIO_TRANSFER_MODE_BLOCK;
  SDIO_DataInitStructure.DPSM = SDIO_DPSM_ENABLE;
  SDIO_DataConfig(SDIO, &SDIO_DataInitStructure);

  /*!< Send CMD18 READ_MULT_BLOCK with argument data address */
  SDIO_CmdInitStructure.Argument = (uint32_t)ReadAddr;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_READ_MULT_BLOCK;
  SDIO_CmdInitStructure.Response = SDIO_RESPONSE_SHORT;
  SDIO_CmdInitStructure.WaitForInterrupt = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSM = SDIO_CPSM_ENABLE;
  SDIO_SendCommand(SDIO, &SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_READ_MULT_BLOCK);

  if (errorstatus != SD_OK)
  {
	printf("[SD] E:SD_ReadMultiBlocks 2. %d \n",errorstatus);
    return(errorstatus);
  }
  printf("[SD] I:SD_ReadMultiBlocks ok. %d \n",errorstatus);
  return(errorstatus);
}
/*************************************************************
**************************************************************/
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SDIO_IT(SDIO_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the SDIO interrupts */
    SDIO->MASK |= SDIO_IT;
  }
  else
  {
    /* Disable the SDIO interrupts */
    SDIO->MASK &= ~SDIO_IT;
  }
}
/*************************************************************
**************************************************************/
void SDIO_DMACmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(__IO uint32_t *) DCTRL_DMAEN_BB = (uint32_t)NewState;
}
/*************************************************************
**************************************************************/
/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  static DMA_HandleTypeDef dmaRxHandle;
  SD_HandleTypeDef *hsd = &uSdHandle;

  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dmaRxHandle);

  /* Configure DMA Rx parameters */
  dmaRxHandle.Init.Channel             = SD_DMAx_Rx_CHANNEL;
  dmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  dmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaRxHandle.Init.Mode                = DMA_NORMAL;
  dmaRxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dmaRxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dmaRxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaRxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  dmaRxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

  dmaRxHandle.Instance = SD_DMAx_Rx_STREAM;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmarx, dmaRxHandle);

  /* не нашел в CUBE
  SDDMA_InitStructure.PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.Memory0BaseAddr = (uint32_t)BufferDST;
  SDDMA_InitStructure.BufferSize = BufferSize;
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);
*/
  /* Configure the DMA stream */
  HAL_DMA_Init(&dmaRxHandle);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

