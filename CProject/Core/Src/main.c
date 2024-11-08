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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Порты, к которым подключена клавиатура
GPIO_TypeDef* Keypad_port[2][4] = {
{Row1_GPIO_Port,Row2_GPIO_Port,Row3_GPIO_Port,Row4_GPIO_Port},
{Col1_GPIO_Port,Col2_GPIO_Port,Col3_GPIO_Port,Col4_GPIO_Port}
};
// �?омера контактов, к которым подключена клавиатура
uint16_t Keypad_pin[2][4] = {
{Row1_Pin,Row2_Pin,Row3_Pin,Row4_Pin},
{Col1_Pin,Col2_Pin,Col3_Pin,Col4_Pin}
};
// Значени�? кнопок на клавиатуре
int keys[4][4] = {
 {1, 2, 3, 10},
 {4, 5, 6, 11},
 {7, 8, 9, 12},
 {0, 15, 14, 13}
};

// �?абор кодов дл�? отображени�? чи�?ел от 0 до 15
// на �?еми�?егментном индикаторе �? общим анодом (Зажигает�?�? нулём)
int digits[16][8] = {
 // a,b,c,d,e,f,g,dp
{0,0,0,0,0,0,1,1}, //0
{1,0,0,1,1,1,1,1}, //1
{0,0,1,0,0,1,0,1}, //2
{0,0,0,0,1,1,0,1}, //3
{1,0,0,1,1,0,0,1}, //4
{0,1,0,0,1,0,0,1}, //5
{0,1,0,0,0,0,0,1}, //6
{0,0,0,1,1,1,1,1}, //7
{0,0,0,0,0,0,0,1}, //8
{0,0,0,0,1,0,0,1}, //9
{0,0,0,1,0,0,0,1}, //A
{1,1,0,0,0,0,0,1}, //b
{0,1,1,0,0,0,1,1}, //C
{1,0,0,0,0,1,0,1}, //d
{0,1,1,0,0,0,0,1}, //E
{0,1,1,1,0,0,0,1} //F
};

// Порты и номера контактов дл�? Семи�?егментного индикатора �?таршего разр�?да
GPIO_TypeDef* HighSeg_Port[8] = {A1_GPIO_Port, B1_GPIO_Port,
C1_GPIO_Port, D1_GPIO_Port, E1_GPIO_Port,
F1_GPIO_Port, G1_GPIO_Port, DP1_GPIO_Port};
uint16_t HighSeg_Pin[8] = {A1_Pin, B1_Pin, C1_Pin,
D1_Pin, E1_Pin, F1_Pin, G1_Pin, DP1_Pin};

GPIO_TypeDef* LowSeg_Port[8] = {A2_GPIO_Port, B2_GPIO_Port,
C2_GPIO_Port, D2_GPIO_Port, E2_GPIO_Port,
F2_GPIO_Port, G2_GPIO_Port, DP2_GPIO_Port};
uint16_t LowSeg_Pin[8] = {A2_Pin, B2_Pin, C2_Pin,
D2_Pin, E2_Pin, F2_Pin, G2_Pin, DP2_Pin};

void seg_out(GPIO_TypeDef** ports, uint16_t* pins, int dig)
{
	for(int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(ports[i],pins[i],digits[dig][i]);
	}
}

void seg_off(GPIO_TypeDef** ports, uint16_t* pins)
{
	for(int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(ports[i],pins[i],1);
	}
}

// Переве�?ти клавиатуру в и�?ходное �?о�?то�?ние
void reset_key_pins(){
	for(int i = 0; i<4; i++){
		HAL_GPIO_WritePin(Keypad_port[1][i],Keypad_pin[1][i],0);
	}
}

// Проверка, кака�? кнопка �?ейча�? нажата. Е�?ли не нажата, то возвращает -1
int read_key()
{
int col_num = -1;
int row_num = -1;
	for(int i = 0; i<4; i++)
	{
		// Ищем �?трочку, котора�? замкнула�?ь на землю
		if(!HAL_GPIO_ReadPin(Keypad_port[0][i],Keypad_pin[0][i])){
			row_num = i;
		}
	}
// Е�?ли найдена �?трока �? нажатой кнопкой
if(row_num != -1)
{
	for(int i = 0; i<4; i++)
	{
		// По очереди подаём логич. 1 на разные �?толбцы и от�?леживаем
		// на каком значение �?троки тоже у�?тановит�?�? в 1
		HAL_GPIO_WritePin(Keypad_port[1][i],Keypad_pin[1][i],1);
		if(col_num == -1 && HAL_GPIO_ReadPin(Keypad_port[0][row_num],Keypad_pin[0][row_num]))
		{
			col_num = i;
		}
	}
	// Обратно замыкаем в�?е контакты �?толбцов на землю
	reset_key_pins();
	if(col_num!=-1)
	{
		// По номерам �?троки и �?толбца определ�?ем чи�?ло
		int curKey = keys[row_num][col_num];
		return curKey;
	}
	// �?айдена �?трока, а �?толбец нет
return -1;
}
// �?и одна из кнопок не нажата
return -1;
}

// Выве�?ти чи�?ло на индикатор младшего разр�?да
void out_num_low(int num)
{
	seg_out(LowSeg_Port,LowSeg_Pin,num%16);
}
// Пога�?ить в�?е �?егменты индикатора младшего разр�?да
void off_low_seg()
{
	seg_off(LowSeg_Port,LowSeg_Pin);
}
// Выве�?ти чи�?ло на индикатор �?таршего разр�?да
void out_num_high(int num)
{
	seg_out(HighSeg_Port,HighSeg_Pin,num%16);
}
// Пога�?ить в�?е �?егменты индикатора �?таршего разр�?да
void off_high_seg()
{
	seg_off(HighSeg_Port,HighSeg_Pin);
}
// Выве�?ти чи�?ло на оба индикатора
void out_num_both(int num)
{
	out_num_high(num/10);
	out_num_low(num%10);
}
// Пога�?ить в�?е �?егменты обоих индикаторов
void off_seg()
{
	off_low_seg();
	off_high_seg();
}
int variant1(num1, num2)
{
	  if ((num1-num2)<0)
	  {
		  int result = 0;
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, 1);
		  return result;
	  }
	  else
	  {
		  int result = num1 - num2;
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, 0);
		  return result;
	  }
}
int variant2(num1, num2)
{
	  if ((num1&&num2) == 0)
	  {
		  HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, 1);
	  }
	  else if (num1 + num2 > 15)
	  {
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, 0);
	  }
	  return num1+num2;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int key; // Кнопка клавиатуры
	int num1 = -1;
	int num2 = -1;
	int num3 = -1;
	int num4 = -1;
	int num5 = -1;
	int num6 = -1;
	reset_key_pins();
	off_seg();

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while(!HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) || num1 == -1)
	  {
		  key = read_key();
		  if(key!=-1)
		  { // Е�?ли кнопка нажата
			  num1 = key;
			  // Выводим чи�?ло на �?таршем разр�?де
			  out_num_high(num1);
			  // Задержка, чтобы не �?читывать значени�? во врем�? дребезга
			  HAL_Delay(300);
		  }
	  }
	  while(!HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) || num2 == -1)
	  {
		  key = read_key();
		  if(key!=-1)
		  {
			  // Е�?ли кнопка нажата
			  num2 = key;
			  out_num_low(num2);
			  HAL_Delay(300);
		  }
	  }
	  num5 = num1*10 + num2;
	  while(!HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) || num3 == -1)
	  	  {
	  		  key = read_key();
	  		  if(key!=-1)
	  		  { // Е�?ли кнопка нажата
	  			  num3 = key;
	  			  // Выводим чи�?ло на �?таршем разр�?де
	  			  out_num_high(num3);
	  			  // Задержка, чтобы не �?читывать значени�? во врем�? дребезга
	  			  HAL_Delay(300);
	  		  }
	  	  }
	  	  while(!HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) || num4 == -1)
	  	  {
	  		  key = read_key();
	  		  if(key!=-1)
	  		  {
	  			  // Е�?ли кнопка нажата
	  			  num4 = key;
	  			  out_num_low(num4);
	  			  HAL_Delay(300);
	  		  }
	  	  }
	  	  num6 = num3*10 + num4;
	  // Ждём нажати�? кнопки дл�? вывода результата
	  while(!HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin)){}
	  int result = variant1(num5, num6);
	  int result = variant2(num5, num6);
	  // Выводим результат, и�?пользу�? оба разр�?да
	  out_num_both(result);
	  // Ждём, когда кнопку отпу�?т�?т
	  while(HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin)){}
	  HAL_Delay(100);
	  // Ждём нажати�? кнопки дл�? �?бро�?а
	  while(!HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin)){}
	  off_seg();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, B2_Pin|DP1_Pin|DP2_Pin|C2_Pin
                          |F2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, B1_Pin|C1_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G2_Pin|D2_Pin|E2_Pin|LED_RED_Pin
                          |LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, F1_Pin|E1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, A1_Pin|G1_Pin|Col1_Pin|Col2_Pin
                          |Col3_Pin|Col4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B2_Pin DP1_Pin DP2_Pin C2_Pin
                           F2_Pin */
  GPIO_InitStruct.Pin = B2_Pin|DP1_Pin|DP2_Pin|C2_Pin
                          |F2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_Pin Row1_Pin Row2_Pin Row3_Pin
                           Row4_Pin */
  GPIO_InitStruct.Pin = Button_Pin|Row1_Pin|Row2_Pin|Row3_Pin
                          |Row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin C1_Pin D1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|C1_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : A2_Pin */
  GPIO_InitStruct.Pin = A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : G2_Pin D2_Pin E2_Pin LED_RED_Pin
                           LED_BLUE_Pin */
  GPIO_InitStruct.Pin = G2_Pin|D2_Pin|E2_Pin|LED_RED_Pin
                          |LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : F1_Pin E1_Pin */
  GPIO_InitStruct.Pin = F1_Pin|E1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_Pin G1_Pin Col1_Pin Col2_Pin
                           Col3_Pin Col4_Pin */
  GPIO_InitStruct.Pin = A1_Pin|G1_Pin|Col1_Pin|Col2_Pin
                          |Col3_Pin|Col4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
