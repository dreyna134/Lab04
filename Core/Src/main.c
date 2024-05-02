/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
volatile char command[2];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void transmitChar(char c);
void transmitCharArr(char* arr);
void AwaitUserInput();
void HandleUserInput();
void AwaitUserCommand();
void HandleUserCommand();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	RCC->AHBENR |= (1 << 19); // Enable clock for Port C
	
	/* PIN CONFIG - PC4 & PC5 to TX and RX respectively*/
	GPIOC->MODER = 0;
	GPIOC->MODER |= (1 << 11) | (1 << 9); // Set pins PC4 & PC5 to AF mode (10)
	
	GPIOC->AFR[0] |= (1 << 20) | (1 << 16); // Set pins 10 & 11 to AF1 (0001)
	GPIOC->AFR[0] &= ~((1 << 23) | (1 << 22) | (1 << 21) | (1 << 19) | (1 << 18) | ( 1 << 17));
	/* END PIN CONFIG */
	
	/* RCC CONFIG - USART3 */
	RCC->APB1ENR |= (1 << 18); // Enable RCC for USART 3
	/* END RCC CONFIG */
	
	/* USART 3 CONFIG */
	USART3->BRR = 69; // Target 115200 b/s, @ 8Mhz clk freq. BRR = 69.444
	USART3->CR1 |= (1 << 3) | (1 << 2); // Enable Transmitter and Receiver respectively
	USART3->CR1 |= (1 << 0); // Enable USART 3
	/* END USART 3 CONFIG */
	
	/* LED CONFIG */
	GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12); // output mode (01)
	GPIOC->OTYPER &= ~((1 << 9) | (1 << 8) | (1 << 7) | (1 << 6)); // push-pull (0)
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12)); // Low speed (x0)
	GPIOC->PUPDR = 0;
	
	GPIOC->ODR |= (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6);
	/* END LED CONFIG */
	
	
	// Main running loop
  while (1)
  {
		//AwaitUserInput();
		//HandleUserInput();
		AwaitUserCommand();
		HandleUserCommand();
  }
}

void HandleUserCommand()
{
	unsigned int r = 6;
	unsigned int o = 8;
	unsigned int g = 9;
	unsigned int b = 7;
	unsigned int color = 0;
	switch(command[0])
	{
		case 'r':
			color = r;
			break;
		case 'o':
			color = o;
			break;
		case 'g':
			color = g;
			break;
		case 'b':
			color = b;
			break;
		default:
			transmitCharArr("Error! Not a valid LED color character.\n");
	}
	switch(command[1])
	{
		case '0':
			GPIOC->ODR &= ~(1 << color);
			break;
		case '1':
			GPIOC->ODR |= (1 << color);
			break;
		case '2':
			GPIOC->ODR ^= (1 << color);
			break;
		default:
			transmitCharArr("Error! Not a valid LED command digit.\n");
	}
}

void AwaitUserCommand()
{
	transmitCharArr("Enter color character [r, g, b, o]\n");
	AwaitUserInput();
	command[0] = USART3->RDR;
	transmitCharArr("Enter LED command [0-off, 1-on, 2-toggle]\n");
	AwaitUserInput();
	command[1] = USART3->RDR;
}

void HandleUserInput()
{
	char c = USART3->RDR;
	if (c == 'r')
		GPIOC->ODR ^= (1 << 6);
	else if (c == 'o')
		GPIOC->ODR ^= (1 << 8);
	else if (c == 'g')
		GPIOC->ODR ^= (1 << 9);
	else if (c == 'b')
		GPIOC->ODR ^= (1 << 7);
	else
		transmitCharArr("Error! Enter a valid color character.");
}

void AwaitUserInput()
{
	int mask = (1 << 5);
	int isRXEmpty = USART3->ISR & mask;
	// While there is a 0 in bit 5
	while(isRXEmpty != mask)
	{
		// effectively do nothing until there is receive data to be handled
		isRXEmpty = USART3->ISR & mask;
	}
}

void transmitCharArr(char* arr)
{
	for(int i = 0; i < strlen(arr); i++)
	{
		transmitChar(arr[i]);
	}
}

void transmitChar(char c)
{
	unsigned int mask = (1 << 7);
	unsigned int isTransmitEmpty = USART3->ISR & mask;
	
	while(mask != isTransmitEmpty)
	{
		isTransmitEmpty = USART3->ISR & mask;
	}
	
	USART3->TDR = c;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
