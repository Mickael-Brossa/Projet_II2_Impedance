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
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "math.h"
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

/* USER CODE BEGIN PV */
//uint16_t sinus12bit_360[360];
//uint16_t sinus12bit_180[180];
//uint16_t sinus12bit_90[90];
//uint16_t sinus12bit_45[45];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
//	char text[50]={};
//	static TS_StateTypeDef  TS_State;
//	uint32_t potl,potr,joystick_h, joystick_v;

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
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_ADC3_Init();
  MX_UART7_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+ BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  //BSP_LCD_DrawBitmap(0,0,(uint8_t*)HorombeRGB565_bmp);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  //BSP_LCD_SetBackColor(00);

  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

//  int i;
//  for(i=0; i<45;i++)	{
//	  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
//	  sinus12bit_180[i] = 2048 + 1024*sin(i*3.14159/90);
//	  sinus12bit_90[i] = 2048 + 1024*sin(i*3.14159/45);
//	  sinus12bit_45[i] = 2048 + 1024*sin(2*i*3.14159/45);
//  }
//  for(i=45; i<90;i++)	{
//	  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
//	  sinus12bit_180[i] = 2048 + 1024*sin(i*3.14159/90);
//	  sinus12bit_90[i] = 2048 + 1024*sin(i*3.14159/45);
//    }
//  for(i=90; i<180;i++)	{
//	  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
//	  sinus12bit_180[i] = 2048 + 1024*sin(i*3.14159/90);
//    }
//  for(i=180; i<360;i++)	{
//	  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
//    }

  //Démarrage du timer 7 avec interruptions

  //Démarrage du DAC associé au tableau sinus12bit via le DMA
//  if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) sinus12bit_45, 45,
//  DAC_ALIGN_12B_R) != HAL_OK) {
//	  Error_Handler();
//  }
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	HAL_GPIO_WritePin(LED13_GPIO_Port,LED13_Pin,HAL_GPIO_ReadPin(BP1_GPIO_Port,BP1_Pin));
//	HAL_GPIO_WritePin(LED14_GPIO_Port,LED14_Pin,HAL_GPIO_ReadPin(BP2_GPIO_Port,BP2_Pin));
//	sprintf(text,"BP1 : %d",HAL_GPIO_ReadPin(BP1_GPIO_Port,BP1_Pin));
//	BSP_LCD_DisplayStringAtLine(5,(uint8_t*) text);
//
//	sConfig.Channel = ADC_CHANNEL_6;
//	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
//	HAL_ADC_Start(&hadc3);
//	while(HAL_ADC_PollForConversion(&hadc3, 100)!=HAL_OK);
//	potr = HAL_ADC_GetValue(&hadc3);
//
//	sConfig.Channel = ADC_CHANNEL_7;
//	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
//	HAL_ADC_Start(&hadc3);
//	while(HAL_ADC_PollForConversion(&hadc3, 100)!=HAL_OK);
//	potl = HAL_ADC_GetValue(&hadc3);
//
//	sConfig.Channel = ADC_CHANNEL_8;
//	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
//	HAL_ADC_Start(&hadc3);
//	while(HAL_ADC_PollForConversion(&hadc3, 100)!=HAL_OK);
//	joystick_v = HAL_ADC_GetValue(&hadc3);
//
//	HAL_ADC_Start(&hadc1);
//	while(HAL_ADC_PollForConversion(&hadc1, 100)!=HAL_OK);
//	joystick_h = HAL_ADC_GetValue(&hadc1);
//
//	sprintf(text,"POTL : %4u POTR : %4u joy_v : %4u joy_h : %4u",(uint)potl,(uint)potr,(uint)joystick_v,(uint)joystick_h);
//	BSP_LCD_DisplayStringAtLine(9,(uint8_t*) text);
//
//	BSP_TS_GetState(&TS_State);
//	if(TS_State.touchDetected){
//	  BSP_LCD_FillCircle(TS_State.touchX[0],TS_State.touchY[0],4);
//	  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  if (htim->Instance == TIM7) {
	  HAL_GPIO_TogglePin(LED16_GPIO_Port, LED16_Pin);
  }
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
