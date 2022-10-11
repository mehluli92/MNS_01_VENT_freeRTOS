/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"

/////////////////////////////////////////////Include library for OLED////////////////////////////////////////////////////////////
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
////////////////////////////////////////////MAX30102 INCLUDES////////////////////////////////////////////////////////////////////////
#include "stdio.h"
#include "algorithm.h"
#include "max30102_lib.h"

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
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
TaskHandle_t uart_task_handle;
TaskHandle_t motor_task_handle;
TaskHandle_t max_sample_task_handle;
TaskHandle_t max_calc_task_handle;
TaskHandle_t pressure_task_handle;
TaskHandle_t oled_task_handle;
TaskHandle_t adc_task_handle;
TaskHandle_t indicator_task_handle;

BaseType_t status;
QueueHandle_t xQueueReadings;

max30102 max;  //MAX30102 object instantiation

int8_t i = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
//static void uart_task_handler(void* parameters); //uart send data after 5 seconds
static void motor_task_handler(void* parameters); //motor control ventilator
static void max_sample_task_handler(void* parameters); //sample values from the max30102
static void max_calc_task_handler(void* parameters);//calculate sampled values to bring an spo2 and hr reading
static void pressure_task_handler(void* parameters);//measure pressure from
static void oled_task_handler(void* parameters); //OLED tasks
static void adc_task_handler(void* parameters); //takes readings from the knobs
static void indicator_task_handler(void* parameters);//shows status of the system to the user through LEDs

//declare ADC functions for controls
static void setIE(uint8_t);
static uint16_t map(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);
//declare MAX30102 functions
void config_max30102_sensor(void);
void initiate_max30102_temp_measurement(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//////////////////////////////////////////////declare ADC functions for controls/////////////////////////////////////////////////////////
uint16_t rawIE;
uint16_t rawVpb;
uint16_t rawBpm;
uint16_t volumePerBreath = 367; //any value can be set from 0 - 367 corresponding to volume
uint16_t breathsPerMinute = 10;
uint8_t inspirationRatio = 1; // inspiration ratio value
uint8_t expirationRatio = 1; //expiration ratio value

ADC_ChannelConfTypeDef sConfig; //enable it to be available in all functions

/////////////////////////////////////////////////Declare values for MAX30102////////////////////////////////////////////////////////////
float t = 0; //temperature value

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
uint8_t bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

int32_t spo2 = 93; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate = 40; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
uint8_t initialReading = 0;
//////////////////////////////////////////////OLED DISPLAY ///////////////////////////////////////////////////////////////////
int8_t initialDisplay = 0;
/////////////////////////////////////////////// UART ///////////////////////////////////////////////////////////////////////////////////
//int uartValue = 1;
//char ox[8] = "&oxygen=";
//char expi[16] = "&expiratory=";
//char he[8] = "&heart=";
////////////////////////////////////////////// PRESSURE SENSOR ////////////////////////////////////////////////////////////////////////
uint8_t airwayPressure = 30; //depends on pressure
uint8_t rawPressure =0;


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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  //max30102 code
  config_max30102_sensor();
  initiate_max30102_temp_measurement();
 ///////////////////////////////////////////Initialize OLED START//////////////////////////////////////////////////////////
  SSD1306_Init (); // initialize the display
 //put constant parameters on the screen
  SSD1306_GotoXY (1,1); // goto 10, 10
  SSD1306_Puts ("MNS_01_VENT", &Font_7x10, 1); // print name of vent

  SSD1306_GotoXY (1, 10);
  SSD1306_Puts ("Temp:", &Font_7x10, 1); //temp

  SSD1306_GotoXY (1, 20);
  SSD1306_Puts ("SPO2:", &Font_7x10, 1);//spo2

  SSD1306_GotoXY (1, 30);
  SSD1306_Puts ("HR:", &Font_7x10, 1); //HR

  SSD1306_GotoXY (1, 40);
  SSD1306_Puts ("VPB:", &Font_7x10, 1);//VPB

  SSD1306_GotoXY (1, 50);
  SSD1306_Puts ("BPM:", &Font_7x10, 1); //BPM

  SSD1306_GotoXY (1, 60);
  SSD1306_Puts ("I:E:", &Font_7x10, 1);

  taskENTER_CRITICAL();
  SSD1306_UpdateScreen(); // update screen
  taskEXIT_CRITICAL();
 ///////////////////////////////////////////Initialize OLED START///////////////////////////////////////////////////////
  //create queue for holding spo2 and hr
  xQueueReadings = xQueueCreate( 1, sizeof( int8_t ) );

//  status = xTaskCreate(uart_task_handler, "uart", 120, "uart", 0, &uart_task_handle);
//  configASSERT(status == pdPASS);
//
  status = xTaskCreate(motor_task_handler, "motor", 160, "motor", 3, &motor_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(max_sample_task_handler, "max_sample", 200, "sampling", 2, &max_sample_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(max_calc_task_handler, "max_calc", 200, "calculating", 3, &max_calc_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(pressure_task_handler, "pressure_task", 100, "pressure calc", 1, &pressure_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(oled_task_handler, "oled_task", 200, "Hello", 1, &oled_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(adc_task_handler, "adc_task", 180, "adc", 1, &adc_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(indicator_task_handler, "indicator_task", 80, "led-indicators", 1, &indicator_task_handle);
  configASSERT(status == pdPASS);



  //start the freeRTOS scheduler
  vTaskStartScheduler();
  //if control comes here its because the scheduler has failed due to insufficient memory
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 840;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
///////////////////////////////////////////////MAX30102 CODE START//////////////////////////////////////////////////////////////////////
void config_max30102_sensor(void)
{
	  //Initiate the MAX30102 code
	   max30102_init(&max, &hi2c3);
	   max30102_reset(&max);
	   max30102_clear_fifo(&max);

	   //set mode for sensor
	     max30102_set_mode(&max);
	     //enable interrupts
	     max30102_interrupt_config(&max);

	     //spo2 config
	     max30102_spo2_config(&max);

	     //fifo config
	     max30102_fifo_config(&max);

	     //led1 current setting
	      max30102_led1_settings(&max);

	     //led2 current setting
	     max30102_led2_settings(&max);
}

void initiate_max30102_temp_measurement(void)
{
    //initiate 1 temperature measurement
    max30102_set_die_temp_en(&max);
    max30102_set_die_temp_rdy(&max);
}
//////////////////////////////////////////////MAX30102 CODE END//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////ADC code start ///////////////////////////////////////////////////////////////////////////
void setIE(uint8_t x)
{
	  if(x >= 0 && x <= 63)
	  {
	     expirationRatio = 1;
	  }
	  if(x >= 64 && x <= 127)
	  	  {
	  	     expirationRatio = 2;
	  	  }
	  if(x >= 128 && x <= 191)
	  	  {
	  	     expirationRatio = 3;
	  	  }
	  if(x >= 192 && x <= 255)
	 	  {
	 	  	expirationRatio = 4;
	 	  }
}

uint16_t map(uint16_t value, uint16_t A, uint16_t B, uint16_t a, uint16_t b)
{
	uint16_t v = value;
	return (v - A)*(b-a)/(B-A) + a;
}
///////////////////////////////////////////////ADC code end//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////RTOS TASKS BEGIN////////////////////////////////////////////////////////////////
//static void uart_task_handler(void* parameters)
//{
//	while(1)
//	{
//change values from integers to characters
//		  char o[10];
//		  //converting num to string
//		  sprintf(o, "%d", (int16_t) spo2);
//
//		  char e[10];
//		  sprintf(e, "%d", expirationRatio);
//
//		  char h[10];
//		  sprintf(h, "%d", (int16_t)heartRate);
//
//		  //concatenate values for sending over uart
//		  strcat(ox,o);
//		  strcat(ox,expi);
//		  strcat(ox, e);
//		  strcat(ox, he);
//		  strcat(ox, h);
//
//		  taskENTER_CRITICAL();
//		    HAL_UART_Transmit(&huart3, (uint8_t *) ox, strlen(ox), 5);
//		  taskEXIT_CRITICAL();

//	}
//}
static void motor_task_handler(void* parameters)
{
	while(1)
	{
		 uint32_t breathLength = 60000/breathsPerMinute; // get total lenght of inspiration and expiration
		 uint32_t lengthOfInspiration = (inspirationRatio * breathLength)/(inspirationRatio + expirationRatio);//get duration for inspiration
		 uint32_t lengthOfExpiration = breathLength - lengthOfInspiration; //get inspiration length

		 uint32_t steps = volumePerBreath /breathsPerMinute; //steps taken in the for loop
		 uint32_t stepSize = volumePerBreath / steps; // size of each step in the inspiration cycle
		 uint32_t secDelayPerInsp = lengthOfInspiration/steps; //delay per step

		//Inspiration phase
		for(uint32_t i = 100; i<=volumePerBreath; i += stepSize)
		{
		 htim1.Instance -> CCR1 = i;
		 vTaskDelay(pdMS_TO_TICKS(secDelayPerInsp));
		}

		vTaskDelay(pdMS_TO_TICKS(lengthOfExpiration));
	}
}
static void max_sample_task_handler(void* parameters)
{
	TickType_t xLastWakeTime;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
	  max30102_interrupt_handler(&max); //get max30102 sample
	  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 40 ) );
	}
}
static void max_calc_task_handler(void* parameters)
{
	int8_t valueToSend = 32;
	BaseType_t xStatus;
	TickType_t xLastWakeTime;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	int8_t k = 0;

	while(1)
	{
		t = max.temperature;
		if(initialReading == 0)
		{
			if(k == 0)
			{
				//store first 25 samples and store in buffer
				 for(uint8_t x = 0; x<=24; x++)
				{
				irBuffer[x] = max._ir_samples[x];
				redBuffer[x] = max._red_samples[x];
				}
			}

			if(k == 1)
			{
				//store second 25 samples in buffer position 25-49
				for(uint8_t x = 25; x<=49; x++)
				{
				 irBuffer[x] = max._ir_samples[x-25];
				 redBuffer[x] = max._red_samples[x-25];
				}
			}

			if(k == 2)
			{
				//store samples in buffer position 50-74
			    for(uint8_t x = 50; x<=74; x++)
				{
				 irBuffer[x] = max._ir_samples[x-50];
				 redBuffer[x] = max._red_samples[x-50];
				}
			}

			if(k == 3)
			{
				 //store samples in buffer position 74-99
				for(uint8_t x=74; x<=99; x++)
				{
				  irBuffer[x] = max._ir_samples[x-74];
				  redBuffer[x] = max._red_samples[x-74];
				}
				initialReading = 1;
			}

			k = k + 1;

			if(max._ir_samples[0] == 0)
			{
				k = 0;
			}


		} else
		{
			//re-order the samples in preparation of new calculations
			for(int8_t i = 75; i<100; i++)
			{
			 irBuffer[i] = max._ir_samples[i - 75];
			 redBuffer[i] = max._red_samples[i - 75];
			}

			//After gathering 25 new samples recalculate HR and SP02
			maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

			xStatus = xQueueSendToBack( xQueueReadings, &valueToSend, 100 );

			if( xStatus != pdPASS )
			{
				/* The send operation could not complete because the queue was full */

			}
		}

         vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

	}
}
static void pressure_task_handler(void* parameters)
{
	while(1)
	{
		//config channel 14 of ADC1
		sConfig.Channel = ADC_CHANNEL_14;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		HAL_ADC_Start(&hadc1);
		//get raw value for Breaths per minute
	    if(HAL_ADC_PollForConversion(&hadc1, 3) == HAL_OK)
		{
		rawPressure = HAL_ADC_GetValue(&hadc1);
		}

	    airwayPressure = map(rawPressure, 0, 255, 0, 5);
	}
}
static void oled_task_handler(void* parameters)
{
	int8_t recievedValue;
	BaseType_t xStatus;
//	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );

	while(1)
	{
		/* This call should always find the queue empty because this task will
		 immediately remove any data that is written to the queue. */
		 if( uxQueueMessagesWaiting( xQueueReadings ) != 0 )
		 {

		 }

		 xStatus = xQueueReceive( xQueueReadings, &recievedValue, 0 );

		 if( xStatus == pdPASS )
		  {
		  /* Data was successfully received from the queue */
			   i = recievedValue;

		  	  //temperature value
		  	    SSD1306_GotoXY (40, 10);
		  	    char temp[10];
		  	    taskENTER_CRITICAL();
		  	    sprintf(temp, "%i", (uint16_t)t);
		  	    taskEXIT_CRITICAL();
		  	    SSD1306_Puts ( temp, &Font_7x10, 1);

		  	    //spo2 value
		  	    SSD1306_GotoXY (40, 20);
		  	    if(validSPO2 == 1)
		  	    {
		  	    	char oxygen[10];
//		  	    	taskENTER_CRITICAL();
//		  	        sprintf(oxygen, "%i", (uint16_t)spo2);
		  	    	 sprintf(oxygen, "%i", (uint16_t)spo2);
//		  	        taskEXIT_CRITICAL();
		  	        SSD1306_Puts ( oxygen, &Font_7x10, 1);
		  	    }

		  	    //Heart rate value
		  	   SSD1306_GotoXY (40, 30);
		  	    if(validHeartRate == 1)
		  	    {
		  	    	 char heart[10];
//		  	    	 taskENTER_CRITICAL();
		  	         sprintf(heart, "%i", (uint16_t)heartRate);
//		  	         taskEXIT_CRITICAL();
		  	         SSD1306_Puts ( heart, &Font_7x10, 1);
		  	    }

		  	    //VPB value
		  	    SSD1306_GotoXY (40, 40);
		  	    char vpb[10];
		  	    sprintf(vpb, "%i", volumePerBreath);
		  	    SSD1306_Puts (vpb, &Font_7x10, 1);

		  	    //BPM value
		  	    SSD1306_GotoXY (40, 50);
		  	    char bpm[10];
		  	    sprintf(bpm, "%i", breathsPerMinute);
		  	    SSD1306_Puts (bpm, &Font_7x10, 1);

		  	    //I:E value
		  	    char ie[10];
		  	    sprintf(ie, "%i", expirationRatio);
		  	    SSD1306_GotoXY (40, 50);
		  	    SSD1306_Puts (ie, &Font_7x10, 1);

		  	    taskENTER_CRITICAL();
		  	    SSD1306_UpdateScreen(); // update screen
		  	    taskEXIT_CRITICAL();

		  }

	}
}
static void adc_task_handler(void* parameters)
{
	TickType_t xLastWakeTime;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		 sConfig.Channel = ADC_CHANNEL_0;
		 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		 //get raw value for I:E
		 HAL_ADC_Start(&hadc1);
		 if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
		 {
			rawIE = HAL_ADC_GetValue(&hadc1);
		 }
		 HAL_ADC_Stop(&hadc1);

		 setIE(rawIE);

		 sConfig.Channel = ADC_CHANNEL_1;
		 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		 //get raw value for Volume per breath
		 HAL_ADC_Start(&hadc1);
		 if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
		 {
		  rawVpb = HAL_ADC_GetValue(&hadc1);
		 }
		 volumePerBreath = map(rawVpb, 0, 255, 100, 367);

		 sConfig.Channel = ADC_CHANNEL_2;
		 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		 HAL_ADC_Start(&hadc1);
		 //get raw value for Breaths per minute
		 if(HAL_ADC_PollForConversion(&hadc1, 3) == HAL_OK)
		 {
		   rawBpm = HAL_ADC_GetValue(&hadc1);
		 }

		breathsPerMinute = map(rawVpb, 0, 255, 10, 40);

	    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 2000 ) );
	}
}
static void indicator_task_handler(void* parameters)
{
	while(1)
	{
		 if(airwayPressure > 35)
		   {
			   //pressure is too high
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		   }else{
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
		   }
		 //wrong heart rate values, spo2 & airway pressure
		   if(validSPO2 == 0 && validHeartRate ==0 && airwayPressure >35)
		   {
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
		   }
	}
}
//////////////////////////////////////////////////////RTOS TASKS END////////////////////////////////////////////////////////////////

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
