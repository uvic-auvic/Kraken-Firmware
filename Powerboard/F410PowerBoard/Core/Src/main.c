/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "powerboard_instr.h"
#include "pin_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint8_t bool;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
//rx buffer variables
uint8_t rx_byte_buffer;
uint8_t circle_rx_buffer[RX_CIRCULAR_BUFFER_SIZE];
int head_rx = 0;
int tail_rx = 0;
int messages_in_rx = 0;

//tx buffer variables
uint16_t tx_word_buffer;
uint16_t circle_tx_bufferHigh[TX_CIRCULAR_BUFFER_SIZE];
uint16_t circle_tx_bufferLow[TX_CIRCULAR_BUFFER_SIZE];
int head_txH = 0;
int tail_txH = 0;
int head_txL = 0;
int tail_txL = 0;
int messages_in_txH = 0;
int messages_in_txL = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void UART_TransmitData(uint16_t);
void UART_Print(uint8_t*);


int write_tx_buffer(uint16_t, int*, int, int);
int write_rx_buffer(uint8_t, int*, int);
int read_tx_buffer(uint16_t*, int, int*,int);
int read_rx_buffer(uint8_t*, int, int*);
void parse_rx(uint8_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*UART Transmission functions----------------------------------
 * Param: uint16_t tx_buffer
 * Returns: void
 * Splits a 16 bit int into 2 bytes and transmits using HAL function
 *Uses blocking polling method which may need to change. Interrupt method was losing
 *the second byte of data
 * */

void UART_TransmitData(uint16_t tx_buffer){
	//char message[] = "hello world\r\n\0";
	uint8_t message[2];
	message[1] = (uint8_t) tx_buffer;
	message[0] = (uint8_t) (tx_buffer >> 8);
	HAL_UART_Transmit(&huart6, (uint8_t*)message, 2, 200);
}

/*Placeholder to verify receipt via UART.
 * Transmits 1 byte at a time
 * */
void UART_Print(uint8_t* message) {
	HAL_UART_Transmit(&huart6, (uint8_t*) message, 1, 100);
}
//------------------------------------------------------------

/*Read/Write buffer functions-----------------------------------------------
* Param: 8 bit binary val
* Returns 0 if buffer full, 1 if write successful
Writes received 8bit binary into circular buffer
Updates global circle buffer at [head] and increments head

*/
int write_tx_buffer(uint16_t val, int* head, int tail, int mode) {

	if ((*head + 1) % TX_CIRCULAR_BUFFER_SIZE == tail) {
		return 0;
	}

	if (mode == TXH_MODE){
		circle_tx_bufferHigh[*head] = val;
		messages_in_txH++;
	}else if (mode == TXL_MODE){
		circle_tx_bufferLow[*head] = val;
		messages_in_txL++;
	}else{
		//invalid buffer mode, make error handler
	}
	*head = (*head + 1) % TX_CIRCULAR_BUFFER_SIZE;
	return 1;
}

//Same method as write_tx_buffer but to different buffer
int write_rx_buffer(uint8_t val, int* head, int tail) {

	if ((*head + 1) % RX_CIRCULAR_BUFFER_SIZE == tail) {
		return 0;
	}

	circle_rx_buffer[*head] = val;
	messages_in_rx++;

	*head = (*head + 1) % RX_CIRCULAR_BUFFER_SIZE;
	return 1;
}
/* Reads from circle buffer,then clears at tail
 * Param: pointer to 8 bit val
 * Returns: 0 if buffer empty, 1 if read successful
 * Increments tail
 */
int read_tx_buffer(uint16_t* val, int head, int* tail, int mode) {
	if (*tail == head) {
		return 0;
	}
	if (mode == TXH_MODE){
		*val = circle_tx_bufferHigh[*tail];
	}else if (mode == TXL_MODE){
		*val = circle_tx_bufferLow[*tail];
	}else{
		//invalid buffer mode, make error handler
	}
	*tail = (*tail + 1) % TX_CIRCULAR_BUFFER_SIZE;
	return 1;
}

//Same as read_tx_buffer
int read_rx_buffer(uint8_t* val, int head, int* tail) {
	if (*tail == head) {
		return 0;
	}

	*val = circle_rx_buffer[*tail];

	*tail = (*tail + 1) % RX_CIRCULAR_BUFFER_SIZE;
	return 1;
}

//-----------------------------------------------------------------
/*Analyze rx input
 * Param: uint8_t received byte
 * Returns: void
 * Takes rx byte and maps to instruction from powerboard_inst.h
 * Performs said instruction for GPIO Write, Analog and GPIO read
 * */
void parse_rx(uint8_t current_data){
	//gpio outputs
	if (current_data == E_PARALLEL_0)
		e_parallel_off();
	else if (current_data == E_PARALLEL_1)
		e_parallel_on();
	else if (current_data == MICROPOWER_0)
		micropower_off();
	else if (current_data == MICROPOWER_1)
		micropower_on();
	else if (current_data == E_VBATT_0)
		e_vbatt_off();
	else if (current_data == E_VBATT_1)
		e_vbatt_on();
	else if (current_data == E_5V_0)
		e_5v_off();
	else if (current_data == E_5V_1)
		e_5v_on();
	else if (current_data == E_12V_0)
		e_12v_off();
	else if (current_data == E_12V_1)
		e_12v_on();
	else if (current_data == E_16V_0)
		e_16v_off();
	else if (current_data == E_16V_1)
		e_16v_on();
	else if (current_data == RED_0)
		led_red_on();
	else if (current_data == RED_1)
		led_red_off();
	else if (current_data == GREEN_0)
		led_green_off();
	else if (current_data == GREEN_1)
		led_green_on();
	else if (current_data == BLUE_0)
		led_blue_off();
	else if (current_data == BLUE_1)
		led_blue_on();

	//analog data requests ADC
	else if (current_data == R_BAT0_1)
		r_bat0_1();
	else if (current_data == R_BAT0_2)
		r_bat0_2();
	else if (current_data == R_BAT0_3)
		r_bat0_3();
	else if (current_data == R_BAT0_4)
		r_bat0_4();
	else if (current_data == R_BAT0_5)
		r_bat0_5();
	else if (current_data == R_BAT0_6)
		r_bat0_6();
	else if (current_data == R_BAT1_1)
		r_bat1_1();
	else if (current_data == R_BAT1_2)
		r_bat1_2();
	else if (current_data == R_BAT1_3)
		r_bat1_3();
	else if (current_data == R_BAT1_4)
		r_bat1_4();
	else if (current_data == R_BAT1_5)
		r_bat1_5();
	else if (current_data == R_BAT1_6)
		r_bat1_6();
	else if (current_data == R_EXT_PRES)
		r_ext_pres();
	else if (current_data == R_WATER_SENSE)
		r_water_sense();

	// gpio data
	else if (current_data == R_REED_DET)
		r_reed_det();
	else if (current_data == R_AUXREED1)
		r_auxreed1();
	else if (current_data == R_AUXREED2)
		r_auxreed2();
	else if (current_data == R_AUXREED3)
		r_auxreed3();
	else if (current_data == R_PARALLEL_E)
		r_parallel_e();
	else if (current_data == R_MICROPOWER)
		r_micropower();
	else if (current_data == R_VBATT_E)
		r_vbatt_e();
	else if (current_data == R_5V_E)
		r_5v_e();
	else if (current_data == R_12V_E)
		r_12v_e();
	else if (current_data == R_16V_E)
		r_16v_e();
	else if (current_data == R_RED)
		r_red();
	else if (current_data == R_GREEN)
		r_green();
	else if (current_data == R_BLUE)
		r_blue();
	else if (current_data == R_ALL_IO)
		r_all_io();
	else
		bad_rx_request();
}

/*
 * When receipt via UART is detected
 * Writes rx_byte_buffer into circular buffer
 * Restarts UART receive interrupt enable
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6)
	    {
	    	if (write_rx_buffer(rx_byte_buffer, &head_rx, tail_rx)) {
	    		//UART_Print(&rx_byte_buffer);
	    	} else {
	    		rx_buffer_full();
	    	}
	    }
	HAL_UART_Receive_IT(&huart6, &rx_byte_buffer, 1);
}

//Shortcuts for write tx_buffer------------------------------------------------
void write_txL_buffer(uint16_t val){
	if(write_tx_buffer(val, &head_txL, tail_txL, TXL_MODE)){}
	else{
		txL_buffer_full();
	}
}
void write_txH_buffer(uint16_t val){
	write_tx_buffer(val, &head_txH, tail_txH, TXH_MODE);
}


//Gpio interrupts. Takes pin number writes corresponding message. /triggers on rise and fall for each alert
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == P_BATT0_CA){
		write_tx_buffer(CA_BAT0, &head_txH, tail_txH, TXH_MODE);
	}else if (GPIO_Pin == P_BATT1_CA){
		write_tx_buffer(CA_BAT1, &head_txH, tail_txH, TXH_MODE);
	}else if (GPIO_Pin == P_PARALLEL_CA){
		write_tx_buffer(CA_PARALLEL, &head_txH, tail_txH, TXH_MODE);
	}else if (GPIO_Pin == P_VBATT_CA){
		write_tx_buffer(CA_VBATT, &head_txH, tail_txH, TXH_MODE);
	}else if (GPIO_Pin == P_5V_CA){
		write_tx_buffer(CA_5V, &head_txH, tail_txH, TXH_MODE);
	}else if (GPIO_Pin == P_12V_CA){
		write_tx_buffer(CA_12V, &head_txH, tail_txH, TXH_MODE);
	}else if (GPIO_Pin == P_16V_CA){
		write_tx_buffer(CA_16V, &head_txH, tail_txH, TXH_MODE);
	}else if (GPIO_Pin == P_REEDSW_DET){
		reed_switch_flipped();
	}else{
		__NOP();
	}
}

//ADC Configurations------------------------------------------------------------
/*Param: uint16_t ADC CHannel - 0 is 0, 1 is 0x1, 2 is 0x2, 3 is 0x4 etc
 * Returns void
 * Selects which ADC channel to poll for results
 *
 * */
void adc_sel(uint16_t ADC_CHANNEL){
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

//Resets ADC rank to 0 to avoid conflicting measurements- this may be causing problems
void adc_reset_conf(){
	ADC_ChannelConfTypeDef sConfig = {0};

		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 0;
		sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		  Error_Handler();
		}
}
//Returns a 32 bit right justified adc value
uint32_t adc_read(){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    return HAL_ADC_GetValue(&hadc1);
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
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  adc_reset_conf();
  uint8_t current_rx_data;// current rx and tx integers
  uint16_t current_txL_data;
  uint16_t current_txH_data;

  set_enables_init();
  set_cont_lights();
  HAL_UART_Receive_IT(&huart6, &rx_byte_buffer, 1); // begin UART interrupt receive

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  /*
	  	   * All received bytes are read and removed from circular buffer into current data buffer
	  	   * terminated by \n\0 in current data
	  	   */
	  	  // High priority transmissions are completed first such as alerts, errors
	  	  while (messages_in_txH){
	  		  read_tx_buffer(&current_txH_data, head_txH, &tail_txH, TXH_MODE);
	  		  messages_in_txH--;
	  		  UART_TransmitData(current_txH_data);
	  	  }
	  	  /*If messages to receive or transmit, read and analyze or transmit*/
	  	  if (messages_in_rx) {
	  		  read_rx_buffer(&current_rx_data, head_rx, &tail_rx);
	  		  messages_in_rx--;
	  		  parse_rx(current_rx_data);
	  		  //UART_Print(&current_rx_data);
	  		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  	  }
	  	  if(messages_in_txL){
	  		  read_tx_buffer(&current_txL_data, head_txL, &tail_txL, TXL_MODE);
	  		  messages_in_txL--;
	  		  UART_TransmitData(current_txL_data);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0); //Enable UART Interrupts
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_11
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC10
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB14 PB11
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_11
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
