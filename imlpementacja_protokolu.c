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
#include "string.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t frequency = 1000 / 2; //zmienna globalna przechowująca częstotliwość mrugania

uint8_t device_addres = 'S';
//uint8_t device_addres[4] = "STM";

#define USART_TXBUF_LEN 1512 // dlugosc bufora Tx (transmisja danych)
#define USART_RXBUF_LEN 1024 // dlugosc bufora Rx (odbiór danych)
uint8_t USART_TxBuf[USART_TXBUF_LEN], // deklaracja bufora Tx o długości USART_TXBUF_LEN
		USART_RxBuf[USART_RXBUF_LEN]; // deklaracja bufora Rx o długości USART_TXBUF_LEN

volatile uint16_t USART_TX_Empty = 0, // pierwsze wolne miejsce w buforze Tx
		 USART_TX_Busy = 0, // ostatnie zajęte miejsce w buforze Tx
		 USART_RX_Empty = 0, // pierwsze wolne miejsce w buforze Rx
		 USART_RX_Busy = 0; // ostatnie zajęte miejsce w buforze Rx
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void blink(){
	// ta funkcja wykonuję się co 1 ms, ponieważ wywołaliśmy ją w pliku "stm32f4xx_it.c" w funkcji "SysTick_Handler"
	static uint32_t time = 0; // statyczna zmienna przechowującas czas który minął od ostatniej zmiany

	if(frequency){
		if(++time >= frequency){ // zwiększamy czas i sprawdzamy czy nie przekroczył częstotliwości
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // przełączamy diodę
			time = 0; // zerójemy czas
		}
	}
}

// przerwanie transmitujące UART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2 && USART_TX_Empty != USART_TX_Busy){//sprawdzamy czy to ten uarti coś jest w buforze
		uint8_t tmp = USART_TxBuf[USART_TX_Busy];
		if(++USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy = 0; // zwiększenie wskaźnika busy w buforze Tx
		HAL_UART_Transmit_IT(&huart2, &tmp, 1); // wysłanie znaku
	}
}

// przerwanie odbierające UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2) {
		if(++USART_RX_Empty >= USART_RXBUF_LEN) USART_RX_Empty = 0; //zwiększenie wskaźnika epty w buforze Rx
		HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1); // zapis do buforu;
	}
}

void frame_send(uint8_t addres, uint8_t command[]){
	uint8_t tmp[260]; //1+1+255+3
	uint16_t index = 0;

	tmp[index++] = device_addres;
	tmp[index++] = addres;

	uint8_t index_cmd = 0;
	while(command[index_cmd]){
		tmp[index++] = command[index_cmd++];
	}

	//liczenie sumy kontrolnej
	uint32_t crc = 0;
	for(uint16_t i=0; i<index; i++){
		crc += tmp[i];
	}
	crc %= 1000;

	//dodawanie sumy kontrolnej do ramki
	tmp[index++] = crc / 100 + '0'; crc %= 100;
	tmp[index++] = crc / 10 + '0'; crc %= 10;
	tmp[index++] = crc + '0';

	uint8_t result[524]; //1+2+2+510+6+1+2
	uint16_t lenght = 0;
	result[lenght++] = '{';
	for(uint16_t i=0; i<index; i++){
		switch(tmp[i]){
			case '\\':
				result[lenght++] = '\\';
				result[lenght++] = '\\';
			break;
			case '{':
				result[lenght++] = '\\';
				result[lenght++] = '[';
			break;
			case '}':
				result[lenght++] = '\\';
				result[lenght++] = ']';
			break;
			default:
				result[lenght++] = tmp[i];
		}
	}
	result[lenght++] = '}';
	result[lenght++] = '\r';
	result[lenght++] = '\n';

	volatile uint16_t idx = USART_TX_Empty; // idx przyjmuje wartość wskaźnika na pierwsze wolne miejsce w buforze Tx
	for(uint16_t i = 0; i < lenght; i++){ // kopoiowanie wyniku do bufora wysyłającego
		USART_TxBuf[idx] = result[i];
		if(++idx == USART_TXBUF_LEN) idx = 0;
	}
	__disable_irq();
	if(USART_TX_Empty == USART_TX_Busy && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE == SET)) { // jeżeli bufor Tx jest pusty i flaga TXE jest ustawiona
		USART_TX_Empty = idx; // ustaw idx jako pierwsze wolne miejsce w buforze Tx
		uint8_t tmp = USART_TxBuf[USART_TX_Busy]; // zmienna tymczasowa przyjmuje wartość pierwszego znaku w buforze Tx
		if(++USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy = 0; // jeśli przekroczono długość bufora, to wróć na początek
		HAL_UART_Transmit_IT(&huart2, &tmp, 1); // uruchomienie przerwania transmitującego (wysłanie znaku)
	}
	else USART_TX_Empty = idx; // jeżeli bufor Tx nie jest pusty to ustaw idx jako pierwsze wolne miejsce w buforze Tx
	__enable_irq();
}

uint8_t frame_get(uint8_t * addres, uint8_t command[]){
	static uint8_t tmp[260]; //1+1+1+255+3+1
	static uint16_t index = 0; //ostatni index ramki
	static uint8_t escape = 0; //0 - zwykły tryb, 1 - ostatni zank to znak ucieczki

	while(USART_RX_Busy != USART_RX_Empty){//póki w buforze Rx są dane
		tmp[index] = USART_RxBuf[USART_RX_Busy];// odczyt znaku z buforu
		if(++USART_RX_Busy >= USART_RXBUF_LEN) USART_RX_Busy = 0; // jeśli przekroczono długość bufora, to wróć na początek

		if(tmp[index] == '{'){
			tmp[0] = '{';
			index = 1;
			escape = 0;
			continue;
		}

		if(!index)continue;

		if(escape){
			if(tmp[index]=='\\')tmp[index++]='\\';
			else if(tmp[index]=='[')tmp[index++]='{';
			else if(tmp[index]==']')tmp[index++]='}';
			else index = 0; //jeśli znak po znaku ucieczki jest zły to reset
			escape = 0;
		}else if(tmp[index]== '\\'){
			escape = 1;//jeśli mamy znak ucieczki
		}else if(tmp[index] == '}'){
			uint16_t lenght = index + 1;
			index = 0;

			if(lenght < 8)continue;//jeśli zbyt krótka ramka
			if(tmp[2] != device_addres)continue;//jeśli ne jesteśmy adresatem
//			if(tmp[4] != device_addres[0] || tmp[5] != device_addres[1] || tmp[6] != device_addres[2])continue;
			if(tmp[lenght-4] < '0' || tmp[lenght-4] > '9' || tmp[lenght-3] < '0' || tmp[lenght-3] > '9' || tmp[lenght-2] < '0' || tmp[lenght-2] > '9')continue;

			uint32_t crc = 0;
			for(uint16_t i=1; i < lenght-4; i++)crc += tmp[i];
			crc %= 1000;
			uint16_t fcrc = ((tmp[lenght-4]-'0')*100) + ((tmp[lenght-3]-'0')*10) + (tmp[lenght-2]-'0');//pobieranie sumy kontrolnej
			if(crc != fcrc)continue;

			addres[0] = tmp[1];
//			addres[1] = tmp[2];
//			addres[2] = tmp[3];
			uint16_t cmd_lenght = 0;
			for(uint16_t i=3; i < lenght-4; i++){
				command[cmd_lenght++] = tmp[i];
			}

			command[cmd_lenght] = 0;
			return 1;

		}else if(++index >= 260)index = 0;
	}

	return 0;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty], 1);
  uint8_t addres, command[255], tmp[255];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(frame_get(&addres, command)){
		  frame_send(addres, command);

		  if(!strcmp(command, "ZAPAL")){
			  frequency = 0;
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
			  sprintf(tmp, "ZAPALONO");
			  frame_send(addres, tmp);
		  }else if(!strcmp(command, "ZGAS")){
			  frequency = 0;
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
			  sprintf(tmp, "ZGASZONO");
			  frame_send(addres, tmp);
		  }else if(!strncmp(command, "MRUGAJ ", 7)){

			  int time = 0, i = 7;

			  while(command[i]>='0' && command[i]<='9'){
				  time = time * 10 + (command[i] - '0');
				  i++;
			  }
			  frequency = time;

			  if(time){
				  sprintf(tmp, "MRUGAM CO %d ms", time);
				  frame_send(addres, tmp);
			  }else{
				  sprintf(tmp, "ZLY ARGUMENT");
				  frame_send(addres, tmp);
			  }
		  }else if(!strcmp(command, "ZWROC")){
			  sprintf(tmp, "MRUGAM CO %d ms", frequency);
			  frame_send(addres, tmp);
		  }else if(!strcmp(command, "STAN")){
			  if(HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin)){
				  sprintf(tmp, "STAN ZAPALONY");
				  frame_send(addres, tmp);
			  }else {
				  sprintf(tmp, "STAN ZGASZONY");
				  frame_send(addres, tmp);
			}
		  }else{
			  sprintf(tmp, "NIEZNANA KOMENDA");
			  frame_send(addres, tmp);
		  }
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
