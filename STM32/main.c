/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_BASE_ADDR ((uint32_t)0x8000000)
#define FLASH_USR_ADDR ((uint32_t)0x08005000)
#define CALLOC_SIZE 128
#define PCK_LEN 3
#define SWAP_INT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Change mgic number
#if defined ( __GNUC__ )
__IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#endif

uint8_t rxByte = 0; // Receives file data
uint8_t rxCplt = 0;
static uint16_t rxIndex = 0; // Receive buffer index
static uint32_t packetExpect = 0;
static uint16_t packetIndex = 0;
uint32_t rxPack[PCK_LEN] = {0}; // Receives total number of packets
char ready[] = "1";
char *ram; // Allocate SRAM to receive data from COM Port

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void VectorTableRelocate(void);
void PeripheralDeInit(void);
void BLJumpToUsrApp(void);
void EraseFlashApp(void);
static inline uint32_t PagesToErase(void);
static inline uint32_t TotalPack(void);
void WriteToFlash (uint16_t index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char ready[] = "1";
  char begin[] = "2";
  char tx[20] = {0};

  // Allocate RAM buffer
  ram = (char*) calloc(CALLOC_SIZE, sizeof(char));

  if (ram == NULL)
  	sprintf(tx, "Allocate Err");
  else
	sprintf(tx, "0x%lX \r\n", (uint32_t)ram);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  VectorTableRelocate();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 2. BOOTING MESSAGE
    //HAL_UART_Transmit(&huart1, (uint8_t*) bootMsg, sizeof(bootMsg), HAL_MAX_DELAY);

  // Turn on Blue LED to signal BL operation
  //HAL_UART_Transmit(&huart1, (uint8_t*)tx, sizeof(tx), HAL_MAX_DELAY);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1000);

  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_SET) // button not pressed
  {
	  // ----- Jump to frmwr_v1 -----
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

      // Reset peripherals to guarantee flawless start of user application
	  PeripheralDeInit();

	  // Jump to user application
	  BLJumpToUsrApp();
  }

  // Erase User App space
  EraseFlashApp();

  // Send Ready signal to PC to begin download
  HAL_Delay(500);
  HAL_UART_Transmit(&huart1, (uint8_t*)begin, sizeof(begin), HAL_MAX_DELAY);
  HAL_Delay(1000);

  // Receive number of packets from UART
  // PCK_LEN: 2 digits < 12800 B
  // PCK_LEN: 3 digits >= 12800 B
  for (uint8_t i = 0; i < PCK_LEN; i++)
	  HAL_UART_Receive(&huart1, (uint8_t *)&rxPack[i], 1, HAL_MAX_DELAY);

  // Process number of expected packets
  packetExpect = TotalPack();

  // Now host waits for ready bit to send data packets
  HAL_Delay(1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)ready, sizeof(ready), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, sizeof(rxByte));

	  if (rxCplt == 1) { // Write RAM buffer contents to Flash
		  rxCplt = 0;
		  *(ram + rxIndex) = rxByte;
		  rxIndex += 1;

		  if (rxIndex == CALLOC_SIZE) { // end of packet
		  	packetIndex++;
		  	WriteToFlash(rxIndex); // Write RAM buffer contents to Flash
		  	rxIndex = 0;
		  	// Process complete, send Ready flag
		  	HAL_Delay(500);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)ready, sizeof(ready), HAL_MAX_DELAY);
		  }

		  // Flush Receive Data Register
		  __HAL_UART_FLUSH_DRREGISTER(&huart1);
	  }

	  if (packetIndex >= packetExpect) {
		  // Soft reset once program flash is complete
		  NVIC_SystemReset();
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pi_Pin */
  GPIO_InitStruct.Pin = Pi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pi_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void VectorTableRelocate(void)
{
	uint32_t i = 0;

	/* Assign VT to address array in SRAM */
	for (i = 0; i < 48; i++)
		VectorTable[i] = *(__IO uint32_t*)(FLASH_USR_ADDR + (i<<2));

}

void PeripheralDeInit(void)
{
	// DeInitialize peripherals in reverse order they were initialized
	// 1. GPIO
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_14);

	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	//__HAL_RCC_GPIOD_CLK_DISABLE();

	// 2. ADC
	// 3. DAC1
	// 4. TIMX
	// 5. USARTX
	HAL_UART_DeInit(&huart1);
	// 6. Interrupt Handlers
	__disable_irq();

	// 7. Clocks
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__HAL_RCC_SYSCFG_CLK_DISABLE();

	HAL_RCC_DeInit();

	// 8. HAL
	HAL_DeInit();
}

void BLJumpToUsrApp(void)
{
	// function pointer to hold the user app reset handler address
	void (*AppResetHandler) (void);

	// Configure MSP by reading the value from the base address of sector 3
	uint32_t mspVal = *(volatile uint32_t *) FLASH_USR_ADDR;

	// Set new main stack pointer
	__set_MSP(mspVal); // From CMSIS

	// Relocate user app Vector Table (N/A M0)
	// SCB->VTOR = FLASH_USR_ADDR

	// Reset handler address is the next 32-bit address after the base (VT)
	uint32_t resetHandlerAddr = *(volatile uint32_t *) (FLASH_USR_ADDR+4);
	AppResetHandler = (void*) resetHandlerAddr;

	// Jump to the reset handler of the user application
	AppResetHandler();
}

static inline uint32_t PagesToErase(void)
{
	return ((FLASH_BASE_ADDR + (64*1024)) - FLASH_USR_ADDR)/1024;
}

void EraseFlashApp(void)
{
	// Unlock Flash
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	// Erase
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = (uint32_t)FLASH_USR_ADDR;
	f.NbPages = PagesToErase(); // Calculates usr app space based on page addr
	uint32_t pageError = 0;

	HAL_FLASHEx_Erase(&f, &pageError);

	// Lock Flash
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}
// void EraseFlashBL(void)

// Pass ram buffer index
void WriteToFlash (uint16_t index)//char *data, uint16_t size)
{
	uint8_t i;
	static uint32_t offset = 0;
	static uint32_t *flashAddr = NULL;
	flashAddr = (uint32_t *) FLASH_USR_ADDR;

	/* Unlock Flash CR access */
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	/* Burn to flash */
	uint32_t tempBuf = 0; // only for debug

	for (i = 0; i < index;)
	{
		//tempBuf = *((uint32_t*) ram + i++);
		//tempBuf = SWAP_INT32(tempBuf);

		tempBuf |= (*(ram + i++) << 0);
		tempBuf |= (*(ram + i++) << 8);
		tempBuf |= (*(ram + i++) << 16);
		tempBuf |= (*(ram + i++) << 24);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(flashAddr+offset), tempBuf);
		offset++;
		tempBuf = 0;
	}

	/* Lock Flash CR access */
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();

	// Clear RAM buffer
	for (i = 0; i < index; i++)
	{
		*(ram+i) &= 0;
	}
}

// rxPack array converts to single uint32_t
static inline uint32_t TotalPack(void)
{
	uint32_t packets = 0;
	static uint32_t pow10[4] = {1, 10, 100, 1000};

	for (uint8_t i = 0; i < PCK_LEN; i++)
	{
		if (*(rxPack+i) == 0)
			packets += (*(rxPack+i))*pow10[(PCK_LEN-1) - i];
		else
			packets += (*(rxPack+i) - 48)*pow10[(PCK_LEN-1) - i];
	}

	return packets;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

		if (packetIndex <= packetExpect)
			rxCplt = 1;

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
