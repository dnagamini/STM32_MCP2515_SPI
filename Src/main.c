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
// interrupt single byte receive
uint8_t uart_rx_data;
// buffer
uint8_t uart_buffer_rx_data[TAM_MAX];
uint8_t uart_buffer_pos = 0;
// flags
uint8_t receive_command_mode = 0;
uint8_t command_received = 0;
uint8_t can_receive_mode = 1;
uint8_t can_send_mode = 0;
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

	if(memcmp(buffer, "#canrcv", strlen("#canrcv")) == 0){
		uint8_t aux_cond = (uint8_t)buffer[7] - '0';

		if(aux_cond == 0){
			can_receive_mode = 0;
			uint8_t user_message[20] = "can receive mode 0";
			HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
		}
		else if(aux_cond == 1){
			can_receive_mode = 1;
			uint8_t user_message[20] = "can receive mode 1";
			HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
		}

		return;
	}

	if(memcmp(buffer, "#send", strlen("#send")) == 0){
		uint8_t aux_ext = (uint8_t)buffer[5] - '0';

		uint8_t aux_hex_id[9] = "";

		strlcpy((char*)aux_hex_id, (char*)buffer + strlen("#sendX"), 9);

		uint32_t aux_id = strtol((char*)aux_hex_id, NULL, 16);

		uint8_t aux_hex_data0[3] = "";
		uint8_t aux_hex_data1[3] = "";
		uint8_t aux_hex_data2[3] = "";
		uint8_t aux_hex_data3[3] = "";
		uint8_t aux_hex_data4[3] = "";
		uint8_t aux_hex_data5[3] = "";
		uint8_t aux_hex_data6[3] = "";
		uint8_t aux_hex_data7[3] = "";

		strlcpy((char*)aux_hex_data0, (char*)buffer + strlen("#sendXYYYYYYYY"), 3);
		strlcpy((char*)aux_hex_data1, (char*)buffer + strlen("#sendXYYYYYYYYZZ"), 3);
		strlcpy((char*)aux_hex_data2, (char*)buffer + strlen("#sendXYYYYYYYYZZZZ"), 3);
		strlcpy((char*)aux_hex_data3, (char*)buffer + strlen("#sendXYYYYYYYYZZZZZZ"), 3);
		strlcpy((char*)aux_hex_data4, (char*)buffer + strlen("#sendXYYYYYYYYZZZZZZZZ"), 3);
		strlcpy((char*)aux_hex_data5, (char*)buffer + strlen("#sendXYYYYYYYYZZZZZZZZZZ"), 3);
		strlcpy((char*)aux_hex_data6, (char*)buffer + strlen("#sendXYYYYYYYYZZZZZZZZZZZZ"), 3);
		strlcpy((char*)aux_hex_data7, (char*)buffer + strlen("#sendXYYYYYYYYZZZZZZZZZZZZZZ"), 3);

		uint8_t aux_data0 = (uint8_t)strtol((char*)aux_hex_data0, NULL, 16);
		uint8_t aux_data1 = (uint8_t)strtol((char*)aux_hex_data1, NULL, 16);
		uint8_t aux_data2 = (uint8_t)strtol((char*)aux_hex_data2, NULL, 16);
		uint8_t aux_data3 = (uint8_t)strtol((char*)aux_hex_data3, NULL, 16);
		uint8_t aux_data4 = (uint8_t)strtol((char*)aux_hex_data4, NULL, 16);
		uint8_t aux_data5 = (uint8_t)strtol((char*)aux_hex_data5, NULL, 16);
		uint8_t aux_data6 = (uint8_t)strtol((char*)aux_hex_data6, NULL, 16);
		uint8_t aux_data7 = (uint8_t)strtol((char*)aux_hex_data7, NULL, 16);

		txMessage.frame.idType = aux_ext;
		txMessage.frame.id = aux_id;
		txMessage.frame.dlc = 8;
		txMessage.frame.data0 = aux_data0;
		txMessage.frame.data1 = aux_data1;
		txMessage.frame.data2 = aux_data2;
		txMessage.frame.data3 = aux_data3;
		txMessage.frame.data4 = aux_data4;
		txMessage.frame.data5 = aux_data5;
		txMessage.frame.data6 = aux_data6;
		txMessage.frame.data7 = aux_data7;

		can_send_mode = 1;

		return;
	}

	if(memcmp(buffer, "#liga", strlen("#liga")) == 0){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		uint8_t user_message[20] = "ligado";
		HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
		return;
	}

	if(memcmp(buffer, "#mask", strlen("#mask")) == 0){
		uint8_t aux_num = (uint8_t)buffer[5] - '0';
		uint8_t aux_ext = (uint8_t)buffer[6] - '0';
		uint8_t aux_hex[9] = "";
		strlcpy((char*)aux_hex, (char*)buffer + strlen("#maskXX"), 9);
		uint32_t aux_ulData = strtol((char*)aux_hex, NULL, 16);

		if(CANSPI_Init_Mask(aux_num, aux_ext, aux_ulData)){
			uint8_t user_message[21] = "";
			snprintf((char*)user_message, 21,"setmask%lX", aux_ulData);
			HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
		}
		else{
			uint8_t user_message[20] = "failed to set mask";
			HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
		}
		return;
	}

	if(memcmp(buffer, "#filt", strlen("#filt")) == 0){
		uint8_t aux_num = (uint8_t)buffer[5] - '0';
		uint8_t aux_ext = (uint8_t)buffer[6] - '0';
		uint8_t aux_hex[9] = "";
		strlcpy((char*)aux_hex, (char*)buffer + strlen("#filtXX"), 9);
		uint32_t aux_ulData = strtol((char*)aux_hex, NULL, 16);

		if(CANSPI_Init_Filter(aux_num, aux_ext, aux_ulData)){
			uint8_t user_message[21] = "";
			snprintf((char*)user_message, 21,"setfilt%lX", aux_ulData);
			HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
		}
		else{
			uint8_t user_message[20] = "failed to set filter";
			HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
		}
		return;
	}

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
  HAL_Delay(5000);
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
		  HAL_Delay(1000);
	  }

	  if(receive_command_mode == 0){
		  // check for uart bluetooth receive
		  if(USART1->SR & USART_SR_RXNE){ // if UART RX is not empty
			  uint8_t user_message[20] = "STM32 Listening ...";
			  HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
			  receive_command_mode = 1; // stop CAN
			  HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1); // call Interrupt
		  }

		  if(can_send_mode){
			  if(CANSPI_Transmit(&txMessage)){
			  }
			  else{
				  uint8_t user_message[20] = "CAN send failed";
				  HAL_UART_Transmit(&huart1, user_message, 20, UART_TIMEOUT);
			  }
			  can_send_mode = 0;
		  }

		  if(can_receive_mode){
			  uCAN_MSG rxMessage;
			  // check for CAN BUS receive
			  if(CANSPI_Receive(&rxMessage)){
			  	  uint8_t can_receive_buffer_id[21] = "";
			  	  uint8_t can_receive_buffer_data[21] = "";
			  	  uint8_t aux = 0;
			  	  snprintf((char*)can_receive_buffer_id, 21, "ID%lX", rxMessage.frame.id);
			  	  HAL_UART_Transmit(&huart1, can_receive_buffer_id, 20, UART_TIMEOUT);
			  	  HAL_Delay(10);
			  	  aux = snprintf((char*)can_receive_buffer_data, 7, "DATA%02X", rxMessage.frame.data0);
			  	  aux = aux + snprintf((char*)can_receive_buffer_data + aux, 3, "%02X", rxMessage.frame.data1);
			  	  aux = aux + snprintf((char*)can_receive_buffer_data + aux, 3, "%02X", rxMessage.frame.data2);
			  	  aux = aux + snprintf((char*)can_receive_buffer_data + aux, 3, "%02X", rxMessage.frame.data3);
			  	  aux = aux + snprintf((char*)can_receive_buffer_data + aux, 3, "%02X", rxMessage.frame.data4);
			  	  aux = aux + snprintf((char*)can_receive_buffer_data + aux, 3, "%02X", rxMessage.frame.data5);
			  	  aux = aux + snprintf((char*)can_receive_buffer_data + aux, 3, "%02X", rxMessage.frame.data6);
			  	  aux = aux + snprintf((char*)can_receive_buffer_data + aux, 3, "%02X", rxMessage.frame.data7);
			  	  HAL_UART_Transmit(&huart1, can_receive_buffer_data, 20, UART_TIMEOUT);
			  	  HAL_Delay(10);
		  	  }
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
