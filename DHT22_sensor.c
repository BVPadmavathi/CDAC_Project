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
#include "string.h"
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
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
float temp,hum;
volatile uint32_t __us = 0;
float Humidity, Temperature;
uint8_t bits[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
uint32_t expectPulse(GPIO_PinState level);
void pinMode(uint16_t GPIO_Pin, uint32_t MODE);
void delay(uint16_t ms);
void delayMicroseconds(uint32_t pus);
void digitalWrite(uint16_t GPIO_Pin, GPIO_PinState state);
GPIO_PinState digitalRead(uint16_t GPIO_Pin);
uint32_t micros();
int DHT22(float *temperature, float *humidity);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define abs(a) (((a) < 0)?-(a):(a))

#define DHTLIB_OK				0
#define DHTLIB_ERROR_CHECKSUM	-1
#define DHTLIB_ERROR_TIMEOUT	-2

#define MAX_CYCLE	((1000) * (HAL_RCC_GetSysClockFreq()/1000000L))//1000//((uint32_t)(1000000)/(uint32_t)(uwTickFreq))
#define TIMEOUT		4294967295

#define DHT22_GPIO_PORT GPIOA
#define DHT22_GPIO_PIN GPIO_PIN_1
#define OUTPUT GPIO_MODE_OUTPUT_PP
#define INPUT GPIO_MODE_INPUT
#define HIGH	GPIO_PIN_SET
#define LOW		GPIO_PIN_RESET
uint8_t bit,count;
//uint8_t data[50];


// function to read data from DHT11
//void DHT11_ReadData(float *temperature, float *humidity)
//{
//    uint8_t i;
//
//    // send start signal
//    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_RESET);
//    HAL_Delay(30);
//    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_SET);
//
//    // wait for response
//    HAL_Delay(40);
//
//    // initialize data array
//    memset(data, 0, sizeof(data));
//
//    // read 40 bits of data
//    for(i=0; i<40; i++)
//    {
//        // wait for low pulse
//        while(!HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN));
//
//        // wait for high pulse
//        uint32_t t = 0;
//        while(HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN))
//        {
//            t++;
//            HAL_Delay(1);
//        }
//
//        // store bit value in data array
//        if(t > 30)
//            data[i/8] |= (1 << (7 - (i % 8)));
//    }
//
//    // verify checksum
//    if(data[4] == (data[0] + data[1] + data[2] + data[3]))
//    {
//        // convert temperature and humidity values
//        *humidity = (data[0] << 8 | data[1]) / 10.0;
//        *temperature = ((data[2] & 0x7F) << 8 | data[3]) / 10.0;
//        if (data[2] & 0x80) *temperature *= -1;
//    }
//}

uint32_t expectPulse(GPIO_PinState level){
	//uint32_t start = micros();
	uint32_t count = 0;

	while (digitalRead(DHT22_GPIO_PIN) == level) {
	    if (count++ >= MAX_CYCLE) {
	      return TIMEOUT; // Exceeded timeout, fail.
	    }
	}
	return count;
}

void pinMode(uint16_t GPIO_Pin, uint32_t MODE){
	//0 -> INPUT
	//1 -> OUTPUT
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	  //__HAL_RCC_GPIOA_CLK_ENABLE();

	  /*Configure GPIO pin : PA1 */
	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = MODE;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(DHT22_GPIO_PORT, &GPIO_InitStruct);
}

void delay(uint16_t ms) {
	HAL_Delay(ms);
}

void delayMicroseconds(uint32_t pus) {
	uint32_t lus = micros();
	while(abs(micros() - lus) < pus);
}

void digitalWrite(uint16_t GPIO_Pin, GPIO_PinState state) {
	HAL_GPIO_WritePin(DHT22_GPIO_PORT, GPIO_Pin, state);
}

GPIO_PinState digitalRead(uint16_t GPIO_Pin) {
	return HAL_GPIO_ReadPin(DHT22_GPIO_PORT, GPIO_Pin);
}

uint32_t micros() {
	return __us;
}

int DHT22(float *temperature, float *humidity) {
	// BUFFER TO RECEIVE
//	uint8_t bits[5];
	uint32_t cycles[80] = {0};
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
	// EMPTY BUFFER
	for (int i=0; i< 5; i++) bits[i] = 0;

	// REQUEST SAMPLE
	pinMode(DHT22_GPIO_PIN, INPUT);
	delay(1);
	pinMode(DHT22_GPIO_PIN, OUTPUT);
	digitalWrite(DHT22_GPIO_PIN, LOW);
	delayMicroseconds(1100);
	digitalWrite(DHT22_GPIO_PIN, HIGH);
	pinMode(DHT22_GPIO_PIN, INPUT);
	delayMicroseconds(55);//40+15

	__disable_irq();
	// ACKNOWLEDGE or TIMEOUT
	if (expectPulse(LOW) == TIMEOUT) {
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		__enable_irq();
		return DHTLIB_ERROR_TIMEOUT;
	}

	if (expectPulse(HIGH) == TIMEOUT) {
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		__enable_irq();
		return DHTLIB_ERROR_TIMEOUT;
	}

	for (int i = 0; i < 80; i += 2) {//Critical code
	      cycles[i] = expectPulse(LOW);
	      cycles[i + 1] = expectPulse(HIGH);
	}

	__enable_irq();

	// READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
	for (int i=0; i<40; i++)
	{
		uint32_t lowCycles = cycles[2 * i];
		uint32_t highCycles = cycles[2 * i + 1];

		if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) {
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
			return DHTLIB_ERROR_TIMEOUT;
		}
		bits[i / 8] <<= 1;

		if (highCycles > lowCycles) {
		      // High cycles are greater than 50us low cycle count, must be a 1.
		      bits[i / 8] |= 1;
		}
	}

	// WRITE TO RIGHT VARS
	*temperature    = (((unsigned int)bits[0]) << 8 | bits[1]) * 0.1;
	*humidity = ((((unsigned int)(bits[2] & 0x7F)) << 8 | bits[3]) * 0.1);

	uint8_t sum = (bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF;
	if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
	return DHTLIB_OK;
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
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  DHT22(&Humidity, &Temperature);
//	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	  delayMicroseconds(2200000);
//	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//	  HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM10) {
		__us++;
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
