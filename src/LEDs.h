/*
 * LEDs.h
 *
 *  Created on: 11.08.2014
 *      Author: sagok
 */

#ifndef LEDS_H_
#define LEDS_H_

#define LEDn                             12								// индикатор на панели
#define TSTn                             5								// тестовые светодиоды на плате

typedef enum
{
  LED0 = 0,
  LED1 = 1,
  LED2 = 2,
  LED3 = 3,
  LED4 = 4,
  LED5 = 5,
  LED6 = 6,
  LED7 = 7,
  LED8 = 8,
  LED9 = 9,
  LED10 = 10,
  LED11 = 11,
  TST1 = 0,
  TST3 = 1,
  TST4 = 2,
  TST5 = 3,
  TST6 = 4
//  TST7 = 17	//RFU - reserved for future use
}Led_TypeDef;

/**
  * @brief  Bit_SET and Bit_RESET enumeration
  */

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;

#define LED0_PIN                         GPIO_PIN_9						// PF9
#define LED0_GPIO_PORT                   GPIOF
#define LED0_GPIO_CLK_ENABLE()           __GPIOF_CLK_ENABLE()
#define LED0_GPIO_CLK_DISABLE()          __GPIOF_CLK_DISABLE()

#define LED1_PIN                         GPIO_PIN_10					// PF10
#define LED1_GPIO_PORT                   GPIOF
#define LED1_GPIO_CLK_ENABLE()           __GPIOF_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __GPIOF_CLK_DISABLE()

#define LED2_PIN                         GPIO_PIN_0						// PC0
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_5						// PA5
#define LED3_GPIO_PORT                   GPIOA
#define LED3_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_6						// PA6
#define LED4_GPIO_PORT                   GPIOA
//#define LED4_GPIO_CLK                    RCC_AHB1Periph_GPIOB
#define LED4_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()

#define LED5_PIN                         GPIO_PIN_11					// PF11
#define LED5_GPIO_PORT                   GPIOF
#define LED5_GPIO_CLK_ENABLE()           __GPIOF_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE()          __GPIOF_CLK_DISABLE()

#define LED6_PIN                         GPIO_PIN_12					// PF12
#define LED6_GPIO_PORT                   GPIOF
#define LED6_GPIO_CLK_ENABLE()           __GPIOF_CLK_ENABLE()
#define LED6_GPIO_CLK_DISABLE()          __GPIOF_CLK_DISABLE()

#define LED7_PIN                         GPIO_PIN_6						// PG6
#define LED7_GPIO_PORT                   GPIOG
#define LED7_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()
#define LED7_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()

#define LED8_PIN                         GPIO_PIN_7						// PG7
#define LED8_GPIO_PORT                   GPIOG
#define LED8_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()
#define LED8_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()

#define LED9_PIN                         GPIO_PIN_3						// PD3
#define LED9_GPIO_PORT                   GPIOD
#define LED9_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED9_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED10_PIN                        GPIO_PIN_6						// PD6
#define LED10_GPIO_PORT                  GPIOD
#define LED10_GPIO_CLK_ENABLE()          __GPIOD_CLK_ENABLE()
#define LED10_GPIO_CLK_DISABLE()         __GPIOD_CLK_DISABLE()

#define LED11_PIN                        GPIO_PIN_10					// PG10
#define LED11_GPIO_PORT                  GPIOG
#define LED11_GPIO_CLK_ENABLE()          __GPIOG_CLK_ENABLE()
#define LED11_GPIO_CLK_DISABLE()         __GPIOG_CLK_DISABLE()

#define TST1_PIN                        GPIO_PIN_6						// PE6
#define TST1_GPIO_PORT                  GPIOE
#define TST1_GPIO_CLK_ENABLE()          __GPIOE_CLK_ENABLE()
#define TST1_GPIO_CLK_DISABLE()         __GPIOE_CLK_DISABLE()

#define TST3_PIN                        GPIO_PIN_1						// PG11
#define TST3_GPIO_PORT                  GPIOE
#define TST3_GPIO_CLK_ENABLE()          __GPIOE_CLK_ENABLE()
#define TST3_GPIO_CLK_DISABLE()         __GPIOE_CLK_DISABLE()

#define TST4_PIN                        GPIO_PIN_12						// PG12
#define TST4_GPIO_PORT                  GPIOG
#define TST4_GPIO_CLK_ENABLE()          __GPIOG_CLK_ENABLE()
#define TST4_GPIO_CLK_DISABLE()         __GPIOG_CLK_DISABLE()

#define TST5_PIN                        GPIO_PIN_11						// PG15
#define TST5_GPIO_PORT                  GPIOG
#define TST5_GPIO_CLK_ENABLE()          __GPIOG_CLK_ENABLE()
#define TST5_GPIO_CLK_DISABLE()         __GPIOG_CLK_DISABLE()

#define TST6_PIN                        GPIO_PIN_15						// PE1
#define TST6_GPIO_PORT                  GPIOG
#define TST6_GPIO_CLK_ENABLE()          __GPIOG_CLK_ENABLE()
#define TST6_GPIO_CLK_DISABLE()         __GPIOG_CLK_DISABLE()

/*
#define TST7_PIN                        GPIO_PIN_1						// RFU
#define TST7_GPIO_PORT                  GPIOE
#define TST7_GPIO_CLK_ENABLE()          __GPIOE_CLK_ENABLE()
#define TST7_GPIO_CLK_DISABLE()         __GPIOE_CLK_DISABLE()
*/


#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   (\
											((__INDEX__) == 0) ? LED0_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 1) ? LED1_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 2) ? LED2_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 3) ? LED3_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 4) ? LED4_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 5) ? LED5_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 6) ? LED6_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 7) ? LED7_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 8) ? LED8_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 9) ? LED9_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 10) ? LED10_GPIO_CLK_ENABLE() : LED11_GPIO_CLK_ENABLE())

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED0_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 1) ? LED1_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 2) ? LED2_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 3) ? LED3_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 4) ? LED4_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 5) ? LED5_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 6) ? LED6_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 7) ? LED7_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 8) ? LED8_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 9) ? LED9_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 10) ? LED10_GPIO_CLK_DISABLE() : LED11_GPIO_CLK_DISABLE())

#define TSTx_GPIO_CLK_ENABLE(__INDEX__)   (\
											((__INDEX__) == 0) ? TST1_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 1) ? TST3_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 2) ? TST4_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 3) ? TST5_GPIO_CLK_ENABLE() : TST6_GPIO_CLK_ENABLE())

#define TSTx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? TST1_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 1) ? TST3_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 2) ? TST4_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 3) ? TST5_GPIO_CLK_DISABLE() : TST6_GPIO_CLK_DISABLE())


/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE();
#define DMAx_CLK_ENABLE()                __DMA2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_4
#define USARTx_TX_DMA_STREAM              DMA2_Stream7
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_4
#define USARTx_RX_DMA_STREAM              DMA2_Stream5


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA2_Stream7_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Stream5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Stream7_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Stream5_IRQHandler


/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
#define RXBUFFERSIZE					  10
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Size of Trasmission buffer */

uint8_t ReadInputDataBit(Led_TypeDef Led);

void Uart1_Init(void);
void AllLEDs_Init(void);
void OutToAllLED(uint16_t DataToOut);

void LED_Init(Led_TypeDef Led,uint32_t Mode);
void LED_Toggle(Led_TypeDef Led);
void LED_High(Led_TypeDef Led);
void LED_Low(Led_TypeDef Led);

void AllTSTs_Init(void);
void TST_Init(Led_TypeDef TST);
void TST_Toggle(Led_TypeDef TST);
void TST_High(Led_TypeDef TST);
void TST_Low(Led_TypeDef TST);


#endif /* LEDS_H_ */
