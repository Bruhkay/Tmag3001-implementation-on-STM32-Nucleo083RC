/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
#include "main.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define TMAG3001_I2C_ADDR_7BIT (0x34)
#define TMAG3001_I2C_ADDR_8BIT (TMAG3001_I2C_ADDR_7BIT << 1)

// TMAG3001 Register Addresses (from datasheet Table 7-1)
#define TMAG3001_REG_DEVICE_CONFIG_2    0x01
#define TMAG3001_REG_SENSOR_CONFIG_1    0x02

volatile uint8_t timer_elapsed_flag = 0;

/**
 * @brief Prints the current time and date from the RTC.
 * @retval None
 */
void print_timestamp(void) {
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    // Get the RTC current Time and Date
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Print the timestamp
    printf("Timestamp: %02d:%02d:%02d - %02d/%02d/%02d\r\n",
           sTime.Hours, sTime.Minutes, sTime.Seconds,
           sDate.Date, sDate.Month, (2025 + sDate.Year));
}

void read_tmag_data(void) {
    uint8_t raw_data[14];
    // Read registers 0x10-0x17
    if (HAL_I2C_Mem_Read(&hi2c1, TMAG3001_I2C_ADDR_8BIT, 0x0D, 1, raw_data, 14, 100) == HAL_OK) {

        int8_t device_ID = (raw_data[0]); // starts from 0x0D and decodes all the raw registers into variables
        int16_t manufacturer_id = ( raw_data[2]<< 8 | raw_data[1] ) ;
        int16_t temp_raw = (raw_data[3] << 8) | raw_data[4];
        int16_t x_raw = (raw_data[5] << 8) | raw_data[6];
        int16_t y_raw = (raw_data[7] << 8) | raw_data[8];
        int16_t z_raw = (raw_data[9] << 8) | raw_data[10];
        int16_t conv = raw_data[11];
        int16_t angle_raw = (raw_data[12] << 8) | raw_data[13];

        float temp_sensitivity = 58.2 ;  //taken from datasheet
		float axis_sensitivity = 446.0f; // LSB/mT for +/-80mT range (TMAG3001A1)
									// Change to 885.0f for +/-40mT (TMAG3001A1)
									// Change to 273.0f for +/-120mT (TMAG3001A2)
									// Change to 137.0f for +/-240mT (TMAG3001A2)

		float temp_celcius = 25 + (float)(temp_raw -17512)/ temp_sensitivity; //17512 is the offset, at 25 celcius the value hits 17512
		float x_field = (float)x_raw / axis_sensitivity;
		float y_field = (float)y_raw / axis_sensitivity;
		float z_field = (float)z_raw / axis_sensitivity;

		float angle_deg = ((float)angle_raw) * 360.0f / 65536.0f;

		printf("Device_ID=0x%04X, Manufacturer ID=0x%04X\r\n", device_ID, manufacturer_id);
		printf("Magnetic Field (mT): X=%.3f, Y=%.3f, Z=%.3f, Temp=%.3f, Angle=%.3f, Conv=%.3d\r\n", x_field, y_field, z_field, temp_celcius, angle_deg,conv);

    } else {
        printf("Data read failed\r\n");
    }

}
void TMAG3001_ConfigureSensor(void) {
    uint8_t reg_val;

    printf("Configuring TMAG3001 sensor...\r\n");
    // 1. Device_Config_2 (0x01): Set Operating_Mode to Continuous Measure Mode (2h = 0b10)
    //    Operating_Mode[1:0] (bits 1-0): 2h (Continuous Measure Mode)
    //    Trigger_Mode[1:0] (bits 3-2): 0h (Conversion Start at I2C Command Bits)
    //    Other bits (Reserved): 0
    //    Value: 0b00000010 = 0x02
    reg_val = 0x02; // Set Operating_Mode to Continuous Measure Mode
    if (HAL_I2C_Mem_Write(&hi2c1, TMAG3001_I2C_ADDR_8BIT, TMAG3001_REG_DEVICE_CONFIG_2, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) != HAL_OK) {
        printf("Failed to write Device_Config_2!\r\n");
    }
    // 2. Sensor_Config_1 (0x02): Enable X, Y, Z channels (MAG_CH_EN = 7h = 0b0111)
    //    MAG_CH_EN[3:0] (bits 7-4): 7h (Enable X, Y, Z channels)
    //    SLEEPTIME[3:0] (bits 3-0): 0h (1ms, not relevant for Continuous Mode but good practice)
    //    Value: 0b01110000 = 0x70
    reg_val = 0x70; // Enable X, Y, Z channels
    if (HAL_I2C_Mem_Write(&hi2c1, TMAG3001_I2C_ADDR_8BIT, TMAG3001_REG_SENSOR_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) != HAL_OK) {
        printf("Failed to write Sensor_Config_1!\r\n");
    }

    HAL_Delay(50);
    printf("TMAG3001 configuration complete.\r\n\r\n");
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	printf("_________________\r\n");
    	print_timestamp();
		printf("Sleep ended. \r\n");
        HAL_TIM_Base_Stop_IT(&htim2); //Stop timer
        BSP_LED_On(LED_GREEN);    //Led lights up when board waking up
        timer_elapsed_flag = 1;       //Flag for interrupt
    }
}

void TMAG3001_Sleep(void) {
    uint8_t reg_val = 0x00;  // Power-Down mode
    if (HAL_I2C_Mem_Write(&hi2c1, TMAG3001_I2C_ADDR_8BIT, TMAG3001_REG_DEVICE_CONFIG_2, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) != HAL_OK) {
        printf("Failed to put TMAG3001 to sleep!\r\n");
    } else {
        printf("TMAG3001 entered sleep mode.\r\n");
    }
}

void TMAG3001_Wakeup(void) {
    uint8_t reg_val = 0x02;  // Continuous Measure Mode
    if (HAL_I2C_Mem_Write(&hi2c1, TMAG3001_I2C_ADDR_8BIT, TMAG3001_REG_DEVICE_CONFIG_2, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) != HAL_OK) {
        printf("Failed to wake up TMAG3001!\r\n");
    } else {
        printf("TMAG3001 woke up and resumed continuous mode.\r\n");
    }
    HAL_Delay(5);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500); // Half second delay at start

  TMAG3001_ConfigureSensor(); // For initialization of the sensor

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_Delay(500); //Half second delay

	  HAL_TIM_Base_Start_IT(&htim2); // Start timer for 3 seconds (3 second indicated in .ioc file)


	  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI); //Put the system to the sleep mode until interrupt

	  if (timer_elapsed_flag == 1) {
		  printf("Sleep process ended. \r\n");
		  TMAG3001_Wakeup();
		  read_tmag_data();  //reads data from x-y-z registers and prints it

		  timer_elapsed_flag = 0; // Clear the flag
		  BSP_LED_Off(LED_GREEN);
		  TMAG3001_Sleep();
		  printf("Sleep process started. \r\n");
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
