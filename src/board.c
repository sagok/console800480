/*
 * board.c
 *
 *  Created on: 08.08.2014
 *      Author: sagok
 */
#include "stm32f2xx_hal.h"
#include "stm32f2xx_hal_rcc.h"
#include "stm32f2xx_hal_cortex.h"
#include "board.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	HAL_RCC_DeInit();

	/* Enable HSE Oscillator and activate PLL with HSE as source */					// SYSCLK =120MHz
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;						// RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;										// RCC_HSE_ON;
//	RCC_OscInitStruct.HSIState = RCC_HSI_ON;										// RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;									// RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;							// RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;												// 25; 16-HSI
	RCC_OscInitStruct.PLL.PLLN = 240;												// 240;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;										// RCC_PLLP_DIV2;			// это ядро, USB не цепляет
	RCC_OscInitStruct.PLL.PLLQ = 5;													// 5;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);							//  (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK	| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;						// RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;								// RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;								// RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;								// RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);						// FLASH_LATENCY_3

}

/*************************************************************************
 * NVIC_Configuration
 *************************************************************************/
void NVIC_Configuration(void)
    {

	  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);			//NVIC_PRIORITYGROUP_1

	  HAL_NVIC_SetPriority(RCC_IRQn,(uint8_t)(configKERNEL_INTERRUPT_PRIORITY >> 4),0);
	  HAL_NVIC_SetPriority(PVD_IRQn,0,0);


/*
	  HAL_NVIC_SetPriority(SDIO_IRQn,0,0);
	  HAL_NVIC_EnableIRQ(SDIO_IRQn);
	  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn,0,0);
	  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn,0,0);
	  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
*/

	//  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn,0,0);
	//  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn,0,0);

    }



