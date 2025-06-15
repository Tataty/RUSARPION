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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#include "spider_funcs.h"
#include "servo_param.h"
#include "Spider.h"

#define DATA_SIZE 	4
#define CRC_DATA_SIZE 	(DATA_SIZE + 2)
#define COBS_DATA_SIZE 	(CRC_DATA_SIZE + 2)

uint8_t command_uart[COBS_DATA_SIZE];

pca9685_handle_t pcaR, pcaL;

SpiderLeg spl_r1({45,  75}, Servo(&pcaR, RC1, 270), Servo(&pcaR, RF1, 270), Servo(&pcaR, RT1, 180));
SpiderLeg spl_r2({90,   0}, Servo(&pcaR, RC2, 180), Servo(&pcaR, RF2, 180), Servo(&pcaR, RT2, 180));
SpiderLeg spl_r3({45, -80}, Servo(&pcaR, RC3, 270), Servo(&pcaR, RF3, 270), Servo(&pcaR, RT3, 180));

SpiderLeg spl_l1({-45,  75}, Servo(&pcaL, LC1, 270), Servo(&pcaL, LF1, 270), Servo(&pcaL, LT1, 180));
SpiderLeg spl_l2({-90,   0}, Servo(&pcaL, LC2, 180), Servo(&pcaL, LF2, 180), Servo(&pcaL, LT2, 180));
SpiderLeg spl_l3({-45, -80}, Servo(&pcaL, LC3, 270), Servo(&pcaL, LF3, 270), Servo(&pcaL, LT3, 180));

Spider spider(&spl_r1, &spl_r2, &spl_r3,
	      &spl_l1, &spl_l2, &spl_l3);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /*leg_r1.SetProgression({{DELTA_X13, ground_y, DELTA_Z13 - step_len	   }, {DELTA_X13, ground_y, DELTA_Z13 + step_len    }, air_y, {DELTA_X13, ground_y, DELTA_Z13 - step_len	   }, {DELTA_X13 - step_len * 1.5, ground_y, DELTA_Z13 + step_len   }});
  leg_r2.SetProgression({{DELTA_X2 , ground_y,           - step_len/1.5}, {DELTA_X2 , ground_y,             step_len/1.5}, air_y, {DELTA_X2 , ground_y,           - step_len}, {DELTA_X2 , ground_y,             step_len}});
  leg_r3.SetProgression({{DELTA_X13, ground_y,-DELTA_Z13 - step_len	   }, {DELTA_X13, ground_y,-DELTA_Z13 + step_len    }, air_y, {DELTA_X13 - step_len * 1.5, ground_y,-DELTA_Z13 - step_len    }, {DELTA_X13, ground_y,-DELTA_Z13 + step_len    }});

  leg_l1.SetProgression({{-DELTA_X13, ground_y, DELTA_Z13 - step_len    }, {-DELTA_X13, ground_y, DELTA_Z13 + step_len    }, air_y, {-DELTA_X13, ground_y, DELTA_Z13 - step_len    }, {-DELTA_X13 + step_len * 1.5, ground_y, DELTA_Z13 + step_len    }});
  leg_l2.SetProgression({{-DELTA_X2 , ground_y,           - step_len/1.5}, {-DELTA_X2 , ground_y,             step_len/1.5}, air_y, {-DELTA_X2 , ground_y,           - step_len}, {-DELTA_X2 , ground_y,             step_len}});
  leg_l3.SetProgression({{-DELTA_X13, ground_y,-DELTA_Z13 - step_len    }, {-DELTA_X13, ground_y,-DELTA_Z13 + step_len    }, air_y, {-DELTA_X13 + step_len * 1.5, ground_y, -DELTA_Z13 - step_len    }, {-DELTA_X13, ground_y,-DELTA_Z13 + step_len    }});
*/
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*__HAL_RCC_I2C1_CLK_ENABLE();
    HAL_Delay(100);
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(100);
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_Delay(100);*/
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

 // HAL_TIM_Base_Start(&htim3);
  HAL_UART_Receive_DMA(&huart1, command_uart, COBS_DATA_SIZE);

  //___INITIALIZE_PCA9685___
  pcaR.i2c_handle = &hi2c1;
  pcaR.device_address = 0x80;
  pcaR.inverted = false;

  pcaL.i2c_handle = &hi2c1;
  pcaL.device_address = 0x82;
  pcaL.inverted = false;

  pca9685_init(&pcaR); // Initialise driver (performs basic setup).
  pca9685_set_pwm_frequency(&pcaR, 50.0f); // Set PWM frequency.

  pca9685_init(&pcaL); // Initialise driver (performs basic setup).
  pca9685_set_pwm_frequency(&pcaL, 50.0f); // Set PWM frequency.
  //________________________
  //___INITIALIZE_SERVOS____
  spl_r1.servo_t.SetLimit(0, 125);
  spl_r2.servo_t.SetLimit(0, 125);
  spl_r3.servo_t.SetLimit(0, 125);

  spl_l1.servo_t.SetLimit(0, 125);
  spl_l2.servo_t.SetLimit(0, 125);
  spl_l3.servo_t.SetLimit(0, 125);

  spl_r1.delta_c += 5;
  spl_r3.delta_c -= 10;

  spl_r3.delta_f -= 3;
  spl_r3.delta_t -= 3;

  spl_l1.delta_t += 3;

  //spl_l2.delta_t += 5;
  //spl_r2.delta_t += 5;

  //spl_l2.delta_f -= 5;
  spl_r2.delta_f += 4;

/*
  spl_r1.MoveToDELTA();
  spl_r2.MoveToDELTA();
  spl_r3.MoveToDELTA();
  spl_l1.MoveToDELTA();
  spl_l2.MoveToDELTA();
  spl_l3.MoveToDELTA();
*/

  spider.SetMaxSpeed(0.03);

  //spider.SetMovementVector({0, 1});
  //spider.SetRotatePower(0);
  //spider.SetSpeedCommand(120);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    spider.Update();// TODO: add deltaTime
   // if (__HAL_TIM_GET_COUNTER(&htim3) > 2) {
   //   __HAL_TIM_SET_COUNTER(&htim3, 0x0000);
  // }
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#include "cobs.h"


uint8_t crc_data[CRC_DATA_SIZE];
uint8_t cobs_data[COBS_DATA_SIZE];

uint8_t index_data = 0;
uint8_t receive_data[COBS_DATA_SIZE];

void ReceivingData(uint8_t data[DATA_SIZE]){

  spider.SetMovementVector({((float)data[0] - 127) / 127, ((float)data[1] - 127) / 127});
  spider.SetRotatePower(((float)data[2] - 127) / 127);
  spider.SetSpeedCommand(data[3]);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // USART1 завершил прием данных

    //cobs_data = command_uart;

    if (CobsDecode(command_uart, crc_data, COBS_DATA_SIZE) > 0){

      uint16_t crc_calc = CalculateCrc(crc_data, DATA_SIZE);

      uint16_t crc = crc_data[DATA_SIZE];
      crc = (crc << 8) | crc_data[DATA_SIZE + 1];

      if (crc_calc == crc) {
	  ReceivingData(crc_data);
      }

      index_data++;
      if(index_data >= COBS_DATA_SIZE){
	index_data = 0;
      }
    }

    HAL_UART_Receive_DMA(&huart1, command_uart, COBS_DATA_SIZE);
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
