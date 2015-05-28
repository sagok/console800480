/*
 * board.h
 *
 *  Created on: 08.08.2014
 *      Author: sagok
 */

#ifndef BOARD_H_
#define BOARD_H_

enum {
	VER_MAJOR 	= 5						// версия			(изменяем при несовместимости с предыдущим железом)
,	VER_MINOR 	= 0						// субвесия			(при небольших изменениях не повлекших изменения в других модулях)
,	VER_DAY		= 11
,	VER_MONTH	= 8
,	VER_YEAR	= 14
};

// порты Дисплея
#define GPIO_DISPLAY			GPIOE
#define LCD_DB4					GPIO_Pin_7
#define LCD_DB5					GPIO_Pin_8
#define LCD_DB6					GPIO_Pin_9
#define LCD_DB7					GPIO_Pin_10
#define LCD_E					GPIO_Pin_11
#define LCD_RW					GPIO_Pin_12
#define LCD_RS					GPIO_Pin_13


// порты реле
#define GPIO_RELEY				GPIOD
#define RELEY_1					GPIO_Pin_12
#define RELEY_2					GPIO_Pin_11
#define RELEY_3					GPIO_Pin_10
#define RELEY_4					GPIO_Pin_9
#define RELEY_5					GPIO_Pin_8

#define GPIO_RELEY_AMPL			GPIOE
#define RELEY_PWR_AMPLIFIRE		GPIO_Pin_15		// Питание УМ
#define ZUMMER					GPIO_Pin_14		// пищалка на лицевой панели

#define GPIO_GLOBAL_RESET		GPIOC
#define GLOBAL_RESET			GPIO_Pin_0		// сброс модулей



#define USB_RX_BufferSize   	64
#define USB_TX_BufferSize   	2048			//32768 уменшаю до 2048 из-за нехватки памяти

// бекап регистры
#define RTCRegister			BKP_DR1
#define EventBKUPRegister	BKP_DR2
#define ACCBKUPRegister		BKP_DR3

/* частота входа в прерывание в мкс.  Deviations from this are measured asthe jitter. */
#define timerINTERRUPT_FREQUENCY		( ( unsigned portSHORT ) configTICK_RATE_HZ )


void SystemClock_Config(void);
void NVIC_Configuration(void);

uint16_t TerminalIncNumbLine(void);

#endif /* BOARD_H_ */
