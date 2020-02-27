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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GP2Y0E          0x40//default
#define SHIFT_BYTE      0x02 //64 cm shift = 2 128 cm shift = 1
#define SHIFT_ADDR      0x35
#define DISTANCE_ADDR1  0x5E
#define DISTANCE_ADDR2  0x5F
#define RIGHT_EDGE_ADDR 0xF8 // C
#define LEFT_EDGE_ADDR  0xF9 // A
#define PEAK_EDGE_ADDR  0xFA // B
#define ADDR_DIS_1 		0x70
#define ADDR_MPU		0x68
#define ACCEL_XOUT_H 	0x3B
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const int motor_1_max=2150;//constante para la maxima posición del servomotor
const int motor_1_min=350;//constante para la minima posición del servomotor


char CDC_tx_buff[60];//El buffer para enviar los datos por usb
uint8_t CDC_tx_size;//tamaño de buffer
char CDC_rx_flag;//Variable que se modifica en el archivo  usbd_cdc_if.c
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);//Función que se usa para enviar por usbd_cdc_if.c
u_int16_t motor_1=900;//variable de posición del motor
uint8_t datai2c[2];
uint8_t data_tx_i2c;
uint8_t distance_cm;
unsigned char distance_cm;
uint8_t data;
uint8_t i=1;
int result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//Función donde llega la interrupción por Timer
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM4){
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
}
*/

void scanner_I2C(){
  	CDC_tx_size=sprintf(CDC_tx_buff,"Escaneando:\r\n");//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
  	CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
	for (i=1; i<128; i++)
		  	{
		  		int result = HAL_I2C_IsDeviceReady(&hi2c2,i<<1,1,1);
		  		if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		  		{
		  		}else{
		  			CDC_tx_size=sprintf(CDC_tx_buff,"\r\n");//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
		  			CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
		  			CDC_tx_size=sprintf(CDC_tx_buff,"0x%X", i);//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
		  			CDC_Transmit_FS((uint8_t *)CDC_tx_buff,CDC_tx_size);//Transmite por USB
		  		}
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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim2); //Habilita interrupción al final de la cuenta definido en sConfigOC.Pulse = 1000;
  //HAL_TIM_Base_Start_IT(&htim4); //Habilita interrupción al final de la cuenta definido en
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//Habilita PWM

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(100);
		scanner_I2C();
	  	CDC_tx_size=sprintf(CDC_tx_buff,"Inciando:\r\n");//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
	  	CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
	  	result = HAL_I2C_IsDeviceReady(&hi2c2,ADDR_DIS_1<<1,1,1);
		if (result!= HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		{
			CDC_tx_size=sprintf(CDC_tx_buff,"Sensor de distancia no responde \r\n");//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
			CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
		}else{
			data_tx_i2c=SHIFT_ADDR;
			HAL_I2C_Master_Transmit(&hi2c2,ADDR_DIS_1<<1, &data_tx_i2c, 1, 5000);
			HAL_I2C_Master_Receive(&hi2c2,ADDR_DIS_1<<1,&data, 1, 5000);
			CDC_tx_size=sprintf(CDC_tx_buff,"Shift %X\n\r",data);//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
			CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
			data_tx_i2c=DISTANCE_ADDR1;
			HAL_I2C_Master_Transmit(&hi2c2,ADDR_DIS_1<<1, &data_tx_i2c, 1, 5000);
			HAL_I2C_Master_Receive(&hi2c2,ADDR_DIS_1<<1,&datai2c[0], 1, 5000);
			data_tx_i2c=DISTANCE_ADDR2;
			HAL_I2C_Master_Transmit(&hi2c2,ADDR_DIS_1<<1, &data_tx_i2c, 1, 5000);
			HAL_I2C_Master_Receive(&hi2c2,ADDR_DIS_1<<1,&datai2c[1], 1, 5000);
			distance_cm = (datai2c[0]*16+datai2c[1])/64;//calculo de distancia
			CDC_tx_size=sprintf(CDC_tx_buff,"distancia %i\n\r",distance_cm);//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
			CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
			result=HAL_I2C_Mem_Read(&hi2c2,ADDR_DIS_1<<1,SHIFT_ADDR,I2C_MEMADD_SIZE_8BIT,&data,1,5000);
			if(result!=HAL_OK){
				CDC_tx_size=sprintf(CDC_tx_buff,"Paila %d\n\r",result);//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
				CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
			}else{
				CDC_tx_size=sprintf(CDC_tx_buff,"Shift Memoria %X\n\r",data);//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
				CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
			}
		}
		//MPU
		result=HAL_I2C_IsDeviceReady(&hi2c2,ADDR_MPU<<1,1,1);
		if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		{
			CDC_tx_size=sprintf(CDC_tx_buff,"MPU no responde\r\n");//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
			CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
		}else{
			data_tx_i2c=ACCEL_XOUT_H;
			HAL_I2C_Master_Transmit(&hi2c2,ADDR_MPU<<1,&data_tx_i2c, 1, 5000);
			HAL_I2C_Master_Receive(&hi2c2,ADDR_MPU<<1,&datai2c[0], 1, 5000);
			CDC_tx_size=sprintf(CDC_tx_buff,"mpu %X\n\r",datai2c[0]);//Guarda en la variable CDC_tx_buff el string y el tamaño del string queda en CDC_size_buff
			CDC_Transmit_FS((uint8_t *)&CDC_tx_buff,CDC_tx_size);//Transmite por USB
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 48;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 30000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

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
