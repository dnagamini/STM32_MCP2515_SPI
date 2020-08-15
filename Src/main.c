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
#include "CANSPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TAM_MAX 50
#define UART_TIMEOUT 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
// interrupt single byte receive
uint8_t uart_rx_data;
// buffer
uint8_t uart_buffer_rx_data[TAM_MAX];
uint8_t uart_buffer_pos = 0;
// flags
uint8_t receive_command_mode = 0;
uint8_t command_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(uart_rx_data == '*'){
		uart_buffer_rx_data[uart_buffer_pos] = '\0'; // end of string
		command_received = 1;// command received, ready to process
	}
	else{
		uart_buffer_rx_data[uart_buffer_pos] = uart_rx_data;
		if(uart_buffer_pos == TAM_MAX){
			uart_buffer_pos = 0;
		}
		else{
			uart_buffer_pos++;
		}
		HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1); // receive next character
	}
}

void process_bluetooth_command(uint8_t *buffer, uint8_t lenght){

	HAL_UART_Transmit(&huart1, buffer, lenght, UART_TIMEOUT); // echo to user
	HAL_Delay(10);

	if(memcmp(buffer, "#liga", strlen("#liga")) == 0){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		uint8_t user_message[20] = "ligado";
		HAL_UART_Transmit(&huart1, user_message, 20, 100);
	}

	if(memcmp(buffer, "#mask", strlen("#mask")) == 0){
		uint8_t aux_num = (uint8_t)buffer[5] - '0';
		uint8_t aux_ext = (uint8_t)buffer[6] - '0';
		char aux_hex[8] = "";
		strncpy(aux_hex, (char*)buffer + strlen("#maskXX"), 8);
		uint32_t aux_ulData = strtol(aux_hex, NULL, 16);

		if(CANSPI_Init_Mask(aux_num, aux_ext, aux_ulData)){
			uint8_t user_message[20] = "";
			sprintf((char*)user_message, "setmask%lX", aux_ulData);
			HAL_UART_Transmit(&huart1, user_message, 20, 100);
		}
		else{
			uint8_t user_message[20] = "failed to set mask";
			HAL_UART_Transmit(&huart1, user_message, 20, 100);
		}
	}

	HAL_Delay(5000);
}

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if(CANSPI_Initialize()){
	  uint8_t user_message[20] = "init CAN SPI ok";
	  HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
  }
  else{
	  uint8_t user_message[20] = "init CAN SPI failed";
	  HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  if(command_received == 1){
		  command_received = 0;
		  process_bluetooth_command(uart_buffer_rx_data, uart_buffer_pos);
		  uart_buffer_pos = 0;
		  uart_buffer_rx_data[0] = '\0';
		  receive_command_mode = 0;
	  }

	  if(receive_command_mode == 0){
		  // check for CAN BUS receive
		  if(CANSPI_Receive(&rxMessage)){
			  uint8_t can_receive_buffer_id[20] = "";
			  uint8_t can_receive_buffer_data[20] = "";
			  uint8_t aux = 0;
			  snprintf((char*)can_receive_buffer_id, 10, "ID%lX", rxMessage.frame.id);
			  HAL_UART_Transmit(&huart1, can_receive_buffer_id, sizeof(can_receive_buffer_id), UART_TIMEOUT);
			  HAL_Delay(10);
			  aux = sprintf((char*)can_receive_buffer_data, "DATA%02X", rxMessage.frame.data0);
			  aux = aux + sprintf((char*)can_receive_buffer_data + aux, "%02X", rxMessage.frame.data1);
			  aux = aux + sprintf((char*)can_receive_buffer_data + aux, "%02X", rxMessage.frame.data2);
			  aux = aux + sprintf((char*)can_receive_buffer_data + aux, "%02X", rxMessage.frame.data3);
			  aux = aux + sprintf((char*)can_receive_buffer_data + aux, "%02X", rxMessage.frame.data4);
			  aux = aux + sprintf((char*)can_receive_buffer_data + aux, "%02X", rxMessage.frame.data5);
			  aux = aux + sprintf((char*)can_receive_buffer_data + aux, "%02X", rxMessage.frame.data6);
			  aux = aux + sprintf((char*)can_receive_buffer_data + aux, "%02X", rxMessage.frame.data7);
			  HAL_UART_Transmit(&huart1, can_receive_buffer_data, sizeof(can_receive_buffer_data), UART_TIMEOUT);
			  HAL_Delay(10);
		  }
		  // check for uart bluetooth receive
		  if(USART1->SR & USART_SR_RXNE){ // if UART RX is not empty
			  uint8_t user_message[20] = "STM32 Listening ...";
			  HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
			  receive_command_mode = 1; // stop CAN
			  HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1); // call Interrupt
		  }
	  }

  } // end while
  /* USER CODE END 3 */
} // end main

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

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
