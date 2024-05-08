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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
//#include "hcsr04_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Connections:-
  //  STM32F407				HC-SR04
  //  5V					Vcc
  //  PD13(GPIO_Output)		Trig
  //  PE9(TIM1_CH1)			Echo0
  //  PC6(TIM8_CH1)			Echo1
  //  GND					Gnd
  //  *PD15(GPIO_Output)	*Blue led; it shows Echo signal status

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
//hcsr04_data_t HCSR04_sensor;
//TaskHandle_t task1;

//DHT22 Sensor
float temp,hum;
volatile uint32_t __us = 0;
float Humidity, Temperature;
uint8_t bits[5];

uint8_t sdata[3] = {0};
//Ultrasonic
float dx_cm[2];
unsigned long previousMicros;

//wrapper
uint32_t expectPulse(GPIO_PinState level);
void pinMode(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pin, uint32_t MODE);
void delay(uint16_t ms);
void delayMicroseconds(uint32_t pus);
void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState state);
GPIO_PinState digitalRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint32_t micros();
int DHT22(float *temperature, float *humidity);

uint8_t __Read_Axis(uint8_t address);
void Read_3Axis(uint8_t *sdata);

int Ultrasonic(float *sensor);

uint8_t TxBuf[2];
uint8_t Return[2];
int16_t Rx_x, Rx_y, Rx_z;


#define pdUS_TO_TICKS(xTimeInUs) ( ( TickType_t ) ( ( ( uint64_t ) ( xTimeInUs ) * ( uint64_t ) (configTICK_RATE_HZ) ) / ( uint64_t ) 1000000U ) )

#define GREEN_LED_PIN GPIO_PIN_12
#define ORANGE_LED_PIN GPIO_PIN_13
#define RED_LED_PIN GPIO_PIN_14
#define BLUE_LED_PIN GPIO_PIN_15


#define abs(a) (((a) < 0)?-(a):(a))

#define DHTLIB_OK				0
#define DHTLIB_ERROR_CHECKSUM	-1
#define DHTLIB_ERROR_TIMEOUT	-2

#define MAX_CYCLE	((1000) * (HAL_RCC_GetSysClockFreq()/1000000L))//1000//((uint32_t)(1000000)/(uint32_t)(uwTickFreq))
#define TIMEOUT		4294967295
#define CYCLEtoUS(a)	((a)/HAL_RCC_GetSysClockFreq())

#define DHT22_GPIO_PORT GPIOA
#define DHT22_GPIO_PIN GPIO_PIN_1
#define OUTPUT GPIO_MODE_OUTPUT_PP
#define INPUT GPIO_MODE_INPUT
#define HIGH	GPIO_PIN_SET
#define LOW		GPIO_PIN_RESET
uint8_t bit,count;

//ultrasonic
#define CM 28.0f
#define INC 71

#define sonic_timeout	20000UL

#define Echo0_PORT	GPIOE
#define Echo1_PORT	GPIOC
#define trig_PORT	GPIOD

#define Echo0_Pin	GPIO_PIN_9
#define Echo1_Pin	GPIO_PIN_6
#define trig	GPIO_PIN_13
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
//wrappers
uint32_t expectPulse(GPIO_PinState level){
	uint32_t count = 0;

	while (digitalRead(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == level) {
	    if (count++ >= MAX_CYCLE) {
	      return TIMEOUT; // Exceeded timeout, fail.
	    }
	}
	return count;
}

void pinMode(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pin, uint32_t MODE){
	//0 -> INPUT
	//1 -> OUTPUT
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	  //__HAL_RCC_GPIOA_CLK_ENABLE();

	  /*Configure GPIO pin : PA1 */
	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = MODE;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay(uint16_t ms) {
	HAL_Delay(ms);
}

void delayMicroseconds(uint32_t pus) {
	uint32_t lus = micros();
	while(abs(micros() - lus) <= pus);
}

void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState state) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, state);
}

GPIO_PinState digitalRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

uint32_t micros() {
	return __us;
}

//DHT22
int DHT22(float *temperature, float *humidity) {
	// BUFFER TO RECEIVE
//	uint8_t bits[5];
	uint32_t cycles[80] = {0};
//	uint32_t total_cycle = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
	// EMPTY BUFFER
	for (int i=0; i< 5; i++) bits[i] = 0;

	// REQUEST SAMPLE
	pinMode(DHT22_GPIO_PORT, DHT22_GPIO_PIN, INPUT);
	delay(1);
	pinMode(DHT22_GPIO_PORT, DHT22_GPIO_PIN, OUTPUT);
	digitalWrite(DHT22_GPIO_PORT, DHT22_GPIO_PIN, LOW);
	delayMicroseconds(1100);
	digitalWrite(DHT22_GPIO_PORT, DHT22_GPIO_PIN, HIGH);
	pinMode(DHT22_GPIO_PORT, DHT22_GPIO_PIN, INPUT);
	delayMicroseconds(55);//40+15

//	__disable_irq();
//	taskDISABLE_INTERRUPTS();
	taskENTER_CRITICAL();//Disable Interrupt and preemption
	// ACKNOWLEDGE or TIMEOUT
	if (expectPulse(LOW) == TIMEOUT) {
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
//		__enable_irq();
//		taskENABLE_INTERRUPTS();
		taskEXIT_CRITICAL();
		return DHTLIB_ERROR_TIMEOUT;
	}

	if (expectPulse(HIGH) == TIMEOUT) {
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
//		__enable_irq();
//		taskENABLE_INTERRUPTS();
		taskEXIT_CRITICAL();
		return DHTLIB_ERROR_TIMEOUT;
	}

	for (int i = 0; i < 80; i += 2) {//Critical code
	      cycles[i] = expectPulse(LOW);
	      cycles[i + 1] = expectPulse(HIGH);
	}

//	__enable_irq();

//	taskENABLE_INTERRUPTS();
	taskEXIT_CRITICAL();

	// READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
	for (int i=0; i<40; i++)
	{
		uint32_t lowCycles = cycles[2 * i];
		uint32_t highCycles = cycles[2 * i + 1];

//		total_cycle += (lowCycles + highCycles);//restore waste us by CRITICAL
//		__us += CYCLEtoUS(total_cycle);
//		total_cycle = 0;
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

int Ultrasonic(float *sensor) {
	pinMode(trig_PORT, trig, OUTPUT);
	digitalWrite(trig_PORT, trig, LOW);
	delayMicroseconds(2);
	digitalWrite(trig_PORT, trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig_PORT, trig, LOW);
	pinMode(trig_PORT, trig, INPUT);
//	pinMode(Echo0_PORT, Echo0_Pin, INPUT);
	previousMicros = micros();
//	taskENTER_CRITICAL();
	while(!digitalRead(Echo0_PORT, Echo0_Pin) && (micros() - previousMicros) <= sonic_timeout); // wait for the echo pin HIGH or timeout
//	taskEXIT_CRITICAL();
	previousMicros = micros();
//	taskENTER_CRITICAL();
	while(digitalRead(Echo0_PORT, Echo0_Pin)  && (micros() - previousMicros) <= sonic_timeout); // wait for the echo pin LOW or timeout
//	taskEXIT_CRITICAL();

	sensor[0] = (((micros() - previousMicros)/2.0)*(float)0.0343) - 2.00; // duration

	pinMode(trig_PORT, trig, OUTPUT);
	digitalWrite(trig_PORT, trig, LOW);
	delayMicroseconds(2);
	digitalWrite(trig_PORT, trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig_PORT, trig, LOW);
	pinMode(trig_PORT, trig, INPUT);
//	pinMode(Echo1_PORT, Echo1_Pin, INPUT);
	previousMicros = micros();
//	taskENTER_CRITICAL();
	while(!digitalRead(Echo1_PORT, Echo1_Pin) && (micros() - previousMicros) <= sonic_timeout); // wait for the echo pin HIGH or timeout
//	taskEXIT_CRITICAL();
	previousMicros = micros();
//	taskENTER_CRITICAL();
	while(digitalRead(Echo1_PORT, Echo1_Pin)  && (micros() - previousMicros) <= sonic_timeout); // wait for the echo pin LOW or timeout
//	taskEXIT_CRITICAL();
	sensor[1] = (((micros() - previousMicros)/2.0)*(float)0.0343) - 2.00; // duration
	return 0;
}

uint8_t __Read_Axis(uint8_t address) {
	uint8_t Return[2];
	uint8_t TxBuf[2];

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);// cs pin low
	//TxBuf[0] = 0x29 | 0x80	//Read
	TxBuf[0] = address | 0x80; //Read
	HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
	HAL_SPI_Receive(&hspi1, Return, 2, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
	return ((Return[1] << 8) | Return[0]);
}

void Read_3Axis(uint8_t *sdata) {
	//Reading the value of the X-registers
	  //OUT-X address 28h - 29h
	sdata[0] = __Read_Axis(0x28);
	  //Reading the value of the Y-Registers
	  //OUT-Y address 2Ah - 2Bh
	sdata[1] = __Read_Axis(0x2A);
	  //Reading the values of Z-registers
	  //OUT-Z Address 2Ch - 2Dh
	sdata[2] = __Read_Axis(0x2C);
}
void Task1a(void *p)
{
	QueueHandle_t Q1 = *(QueueHandle_t *)p;
	TickType_t woke = xTaskGetTickCount();

	for(;;)
	{
		DHT22(&Humidity, &Temperature);
		//HCSR04_GetInfo(&HCSR04_sensor);

		xQueueSend(Q1, sdata, portMAX_DELAY);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//		vTaskDelayUntil(&tip,12);
		vTaskDelayUntil(&woke, pdMS_TO_TICKS(2000));
//		taskYIELD();
	}
}
void Task1b(void *p) {
	TickType_t woke = xTaskGetTickCount();
	for(;;)
		{
			Read_3Axis(sdata);
			Ultrasonic(dx_cm);
			vTaskDelayUntil(&woke, pdMS_TO_TICKS(800));
		}
}
void Task2(void *p)
{
	QueueHandle_t *Q2[2] = {(QueueHandle_t *)p, ((QueueHandle_t *)p + 1)};
	uint8_t sdata[3] = {0, 0, 0};
	char buf[100] = {0};
	for(;;)
	{
		xQueueReceive(Q2[0][0], sdata, portMAX_DELAY);

		//vTaskDelayUntil(&tip,13);
		//format it to json (key:value)
		sprintf(buf, "\n""{X:%hu Y:%hu Z:%hu}", sdata[0], sdata[1], sdata[2]);
		xQueueSend(Q2[0][1], buf, portMAX_DELAY);
//		taskYIELD();
	}
}

void Task3(void *p) {
	QueueHandle_t Q3 = *(QueueHandle_t *)p;
	char buf[100] = {0};
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		xQueueReceive(Q3, buf, portMAX_DELAY);
		//send to CAN

//		taskYIELD();
	}
}
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //  STM32F407				DHT22
  //  	D					PA1

  //  STM32F407				HC-SR04
  //  5V					Vcc
  //  PD13(GPIO_Output)		Trig
  //  PE9(TIM1_CH1)			Echo0
  //  PC6(TIM8_CH1)			Echo1
  //  GND					Gnd
  //  *TIM2	*it is used for 10us trigger delay
  //  *PD15(GPIO_Output)	*Blue led; it shows Echo signal status
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
//  HAL_TIM_Base_Start_IT(&htim6);
  //HCSR04_Init();
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//PULL chip select to low
  TxBuf[0] = 0x20; //Address of registers
  TxBuf[1] = 0x37; //Data to be filled
  HAL_SPI_Transmit(&hspi1, TxBuf, 2, 50);
  //(handle of SPI, name of buffer, no. of bytes, timeout(ms))
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
  static QueueHandle_t T1T2;
  	static QueueHandle_t T2T3;
  	static QueueHandle_t *Q[2] = {&T1T2, &T2T3};

  	T1T2 = xQueueCreate(1, sizeof(uint8_t)*3);
  	T2T3 = xQueueCreate(1, sizeof(uint8_t)*100);
  	    // Create tasks
  	    xTaskCreate(Task1a, "Task1a", 200, &T1T2, 2, NULL);
  	    xTaskCreate(Task1b, "Task1b", 200, &T1T2, 2, NULL);
  	    xTaskCreate(Task2, "Task2", 200, *Q, 1, NULL);
  	    xTaskCreate(Task3, "Task3", 200, &T2T3, 1, NULL);

  	    // Start the FreeRTOS scheduler
  	    vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim10.Init.Prescaler = 41;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 3;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT22_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : echo0_Pin */
  GPIO_InitStruct.Pin = echo0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : trig_Pin */
  GPIO_InitStruct.Pin = trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : echo1_Pin */
  GPIO_InitStruct.Pin = echo1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(htim->Instance == TIM10) {//TIM10 ==> Prescale --> 41 ==> Counter Period --> 3
		if(__us % 3 == 0) {
			__us++;
		}
		__us++;//1495069 ==> 2000000
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  //HCSR04_TIM_PEC(htim);
  //portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//NOTE:- Caused isuue in critical section
  /* USER CODE END Callback 1 */
}

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
