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
#include<stdio.h>
#include<inttypes.h>

#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rxbuff[16];

int slow=75;

int fast=400;

int Buff1=20;

int Buff2=-20;

int BuffP=50;

int BuffN=-50;





int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1, ll2, rr2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PWM (uint16_t CCR1LoadValue)
{
	uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
	uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020020; // address of alternate function register of port A
    uint32_t *pPortAModeReg   =   (uint32_t*)0x40020000; // address of port A mode register
    uint32_t *pAPB1ClkCtrlReg =   (uint32_t*)0x40023840; // address of APB1 clock control register
	uint32_t *pTimer2PsclrReg =   (uint32_t*)0x40000C28; // address of timer 5 pre-scalar register
	uint32_t *pTimer2ArrReg   =   (uint32_t*)0x40000C2C; // address of timer 5 auto reload register
    uint32_t *pTimer2CntReg   =   (uint32_t*)0x40000C24; // address of timer 5 counter register
    uint32_t *pTimer2CCMR1Reg =   (uint32_t*)0x40000C18; // address of timer 5 capture/compare register
	uint32_t *pTimer2CcerReg  =   (uint32_t*)0x40000C20; // address of timer 5 capture/compare enable register
	uint32_t *pTimer2CCR1Reg  =   (uint32_t*)0x40000C34; // address of timer 5 capture/compare register
	uint32_t *pTimer2CR1Reg   =   (uint32_t*)0x40000C00; // address of timer 5 control register 1


			*pAHB1ClkCtrlReg |= 0x1;        // port A clock enable
		    *pAFRLPortAReg   |= 0x00000002; // alternate function of Timer 5 enabled
		    *pPortAModeReg   &= 0xFFFFFFFC;
		    *pPortAModeReg   |= 0x00000002; // port A Pin 0 configured for 'Alternate function'


		    *pAPB1ClkCtrlReg |= 0x8;           // timer 5 clock enable
		    *pTimer2PsclrReg = 10 - 1;         // pre-scalar value
		    *pTimer2ArrReg   = 26667 - 1 ;     // calculated auto reload value (60Hz PWM frequency)
		    *pTimer2CntReg   = 0;              // counter initialized to 0
		    *pTimer2CCMR1Reg = 0x0060;         // output Compare mode 1 enabled in Timer 5 Channel 1
		    *pTimer2CcerReg  = 1;              // configured as active low in Output compare mode
		    *pTimer2CCR1Reg  = CCR1LoadValue;  // duty cycle of PWM signal
		    *pTimer2CR1Reg   = 1;              // counter enable

}
void PWM_PA1_TIM2(uint16_t CCR1LoadValue) {
    uint32_t *pAHB1ClkCtrlReg = (uint32_t*)0x40023830;  // RCC AHB1 Clock Enable Register
    uint32_t *pAPB1ClkCtrlReg = (uint32_t*)0x40023840;  // RCC APB1 Clock Enable Register
    uint32_t *pGPIOA_MODER    = (uint32_t*)0x40020000;  // GPIOA Mode Register
    uint32_t *pGPIOA_AFRL     = (uint32_t*)0x40020020;  // GPIOA AFRL (Alternate Function Low Register)
    uint32_t *pTIM2_PSC       = (uint32_t*)0x40000028;  // TIM2 Prescaler Register
    uint32_t *pTIM2_ARR       = (uint32_t*)0x4000002C;  // TIM2 Auto-Reload Register
    uint32_t *pTIM2_CCR2      = (uint32_t*)0x40000038;  // TIM2 Capture/Compare Register 2
    uint32_t *pTIM2_CCMR1     = (uint32_t*)0x40000018;  // TIM2 Capture/Compare Mode Register 1
    uint32_t *pTIM2_CCER      = (uint32_t*)0x40000020;  // TIM2 Capture/Compare Enable Register
    uint32_t *pTIM2_CR1       = (uint32_t*)0x40000000;  // TIM2 Control Register 1

    // 1. Enable GPIOA and TIM2 clocks
    *pAHB1ClkCtrlReg |= (1 << 0);  // GPIOA clock enable
    *pAPB1ClkCtrlReg |= (1 << 0);  // TIM2 clock enable

    // 2. Set PA1 to alternate function mode (MODERy[1:0] = 10)
    *pGPIOA_MODER &= ~(0x3 << (1 * 2));  // Clear mode bits for PA1
    *pGPIOA_MODER |=  (0x2 << (1 * 2));  // Set PA1 to alternate function mode

    // 3. Configure PA1 for AF1 (TIM2_CH2)
    *pGPIOA_AFRL &= ~(0xF << (1 * 4));   // Clear AFRL bits for PA1
    *pGPIOA_AFRL |=  (0x1 << (1 * 4));   // Set AF1 for PA1

    // 4. Configure TIM2
    *pTIM2_PSC = 10 - 1;                 // Set prescaler value
    *pTIM2_ARR = 26667 - 1;              // Set auto-reload value for 60Hz
    *pTIM2_CCR2 = CCR1LoadValue;         // Set duty cycle
    *pTIM2_CCMR1 |= (0x6 << 12);         // Set PWM mode 1 on channel 2
    *pTIM2_CCER |= (1 << 4);             // Enable capture/compare for channel 2
    *pTIM2_CR1 |= (1 << 0);              // Enable TIM2
}
void PWM_PA5_TIM8(uint16_t CCR1LoadValue) {
    uint32_t *pAHB1ClkCtrlReg = (uint32_t*)0x40023830;  // RCC AHB1 Clock Enable Register
    uint32_t *pAPB2ClkCtrlReg = (uint32_t*)0x40023844;  // RCC APB2 Clock Enable Register
    uint32_t *pGPIOA_MODER    = (uint32_t*)0x40020000;  // GPIOA Mode Register
    uint32_t *pGPIOA_AFRL     = (uint32_t*)0x40020020;  // GPIOA AFRL (Alternate Function Low Register)
    uint32_t *pTIM8_PSC       = (uint32_t*)0x40010428;  // TIM8 Prescaler Register
    uint32_t *pTIM8_ARR       = (uint32_t*)0x4001042C;  // TIM8 Auto-Reload Register
    uint32_t *pTIM8_CCR1      = (uint32_t*)0x40010434;  // TIM8 Capture/Compare Register 1
    uint32_t *pTIM8_CCMR1     = (uint32_t*)0x40010418;  // TIM8 Capture/Compare Mode Register 1
    uint32_t *pTIM8_CCER      = (uint32_t*)0x40010420;  // TIM8 Capture/Compare Enable Register
    uint32_t *pTIM8_CR1       = (uint32_t*)0x40010400;  // TIM8 Control Register 1

    // 1. Enable GPIOA and TIM8 clocks
    *pAHB1ClkCtrlReg |= (1 << 0);  // GPIOA clock enable
    *pAPB2ClkCtrlReg |= (1 << 1);  // TIM8 clock enable

    // 2. Set PA5 to alternate function mode (MODERy[1:0] = 10)
    *pGPIOA_MODER &= ~(0x3 << (5 * 2));  // Clear mode bits for PA5
    *pGPIOA_MODER |=  (0x2 << (5 * 2));  // Set PA5 to alternate function mode

    // 3. Configure PA5 for AF3 (TIM8_CH1)
    *pGPIOA_AFRL &= ~(0xF << (5 * 4));   // Clear AFRL bits for PA5
    *pGPIOA_AFRL |=  (0x3 << (5 * 4));   // Set AF3 for PA5

    // 4. Configure TIM8
    *pTIM8_PSC = 10 - 1;                 // Set prescaler value
    *pTIM8_ARR = 26667 - 1;              // Set auto-reload value for 60Hz
    *pTIM8_CCR1 = CCR1LoadValue;         // Set duty cycle
    *pTIM8_CCMR1 |= (0x6 << 4);          // Set PWM mode 1 on channel 1
    *pTIM8_CCER |= (1 << 0);             // Enable capture/compare for channel 1
    *pTIM8_CR1 |= (1 << 0);              // Enable TIM8
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive(&huart4, rxbuff, 64,1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_StatusTypeDef status;

		 	 	   status=HAL_UART_Receive(&huart4, rxbuff,64,1000);



		 	 	   if (status == HAL_OK)

		 	 	   {

		 	 	   // Convert the received bytes to signed integers



		 	 	   lx = (rxbuff[1] & 0x80) ? (int32_t)rxbuff[1] - 256 : (int32_t)rxbuff[1];

		 	 	   ly = (rxbuff[2] & 0x80) ? (int32_t)rxbuff[2] - 256 : (int32_t)rxbuff[2];

		 	 	   rx = (rxbuff[3] & 0x80) ? (int32_t)rxbuff[3] - 256 : (int32_t)rxbuff[3];

		 	 	   ry = (rxbuff[4] & 0x80) ? (int32_t)rxbuff[4] - 256 : (int32_t)rxbuff[4];



		 	 	   cro= (rxbuff[5] & 0x80) ? (int32_t)rxbuff[5] - 256 : (int32_t)rxbuff[5];

		 	 	   squ= (rxbuff[6] & 0x80) ? (int32_t)rxbuff[6] - 256 : (int32_t)rxbuff[6];

		 	 	   tri= (rxbuff[7] & 0x80) ? (int32_t)rxbuff[7] - 256 : (int32_t)rxbuff[7];

		 	 	   cir= (rxbuff[8] & 0x80) ? (int32_t)rxbuff[8] - 256 : (int32_t)rxbuff[8];

		 	 	   up= (rxbuff[9] & 0x80) ? (int32_t)rxbuff[9] - 256 : (int32_t)rxbuff[9];

		 	 	   down= (rxbuff[10] & 0x80) ? (int32_t)rxbuff[10] - 256 : (int32_t)rxbuff[10];

		 	 	   left= (rxbuff[11] & 0x80) ? (int32_t)rxbuff[11] - 256 : (int32_t)rxbuff[11];

		 	 	   right=(rxbuff[12] & 0x80) ? (int32_t)rxbuff[12] - 256 : (int32_t)rxbuff[12];

		 	 	   ll1= (rxbuff[13] & 0x80) ? (int32_t)rxbuff[13] - 256 : (int32_t)rxbuff[13];

		 	 	   ll2= (rxbuff[14] & 0x80) ? (int32_t)rxbuff[14] - 256 : (int32_t)rxbuff[14];

		 	 	   rr1= (rxbuff[15] & 0x80) ? (int32_t)rxbuff[15] - 256 : (int32_t)rxbuff[15];

		 	 	   rr2= (rxbuff[16] & 0x80) ? (int32_t)rxbuff[16] - 256 : (int32_t)rxbuff[16];



		 	 	   // Print the received values

		 	 	   printf("Received Integers:\n");

		 	 	   printf("lx: %ld\n", lx);

		 	 	   printf("ly: %ld\n", ly);

		 	 	   printf("rx: %ld\n", rx);

		 	 	   printf("ry: %ld\n", ry);



		 	 	   printf("cro: %ld\n", cro);

		 	 	   printf("squ: %ld\n", squ);

		 	 	   printf("tri: %ld\n", tri);

		 	 	   printf("cir: %ld\n", cir);



		 	 	   printf("up: %ld\n", up);

		 	 	   printf("down: %ld\n", down);

		 	 	   printf("left: %ld\n", left);

		 	 	   printf("right: %ld\n", right);



		 	 	   printf("ll1: %ld\n", ll1);

		 	 	   printf("ll2: %ld\n", ll2);

		 	 	   printf("rr1: %ld\n", rr1);

		 	 	   printf("rr2: %ld\n", rr2);



		 	 	   }

		 	 	   else{

		 	 	   ry = 0;

		 	 	   rx = 0;

		 	 	   lx = 0;

		 	 	   ly = 0;

		 	 	   cro = 0;

		 	 	   squ = 0;

		 	 	   tri = 0;

		 	 	   cir = 0;

		 	 	   up = 0;

		 	 	   down = 0;

		 	 	   left = 0;

		 	 	   right = 0;

		 	 	   ll1=0;

		 	 	   ll2=0;

		 	 	   rr1=0;

		 	 	   rr2=0;





		 	 	   }







		 	 	   uint16_t dutycycle;

		 	 	  	dutycycle=0;



		 	 	  	//chassis



		 	 	  	//motors stop

		 	 	  	if(ly>=Buff2 && ly<=Buff1 && lx>=Buff2 && lx<=Buff1 && rx>=Buff2 && rx<=Buff1){
		 	 //

		 	 //
		 	 	  	dutycycle=0;
		 	 	  	PWM(dutycycle);
		 	 	  PWM_PA1_TIM2(dutycycle);
		 	 	  PWM_PA5_TIM8(dutycycle);


		 	 //


		 	 	  	}
		 	 //
		 	 //	  	// else{
		 	 //
		 	 //	  	//forward
		 	 //
		 	 	  	else if( ly>=Buff1 && (lx<=BuffP && lx>=BuffN) )

		 	 	  	{

		 	 	  	dutycycle=map(ly,Buff1,127,0,fast);

		 	 		PWM(dutycycle);
		 	 			 	 	  PWM_PA1_TIM2(0);
		 	 			 	 	  PWM_PA5_TIM8(dutycycle);
		 	 //
		 	 //	  //
		 	 //

		 	 //






		 	 	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);

		 //	 	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);



		 	 	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);



		 	 //	   	HAL_Delay(1000);

		 	 	   }

		 	 	   // backward

		 	 	   else if(ly<=Buff2 && (lx<=BuffP && lx>=BuffN) )

		 	 	   {

		 	 		 dutycycle=map(ly,-128,Buff2,fast,0);





		 	 		PWM(dutycycle);
		 	 			 	 	  PWM_PA1_TIM2(0);
		 	 			 	 	  PWM_PA5_TIM8(dutycycle);
		 	 //
		 	 //	  //
		 	 //



		 	 	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);

		 //	 	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);



		 	 	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);

		 	 //	    HAL_Delay(1000);





		 	 	   }

		 	 	   //	right

		 	 	   else if(lx>=Buff1 && (ly<=BuffP && ly>=BuffN))

		 	 	   {

		 	 	   dutycycle=map(lx,Buff1,127,0,fast);

		 	 		PWM(dutycycle);
		 	 			 	 	  PWM_PA1_TIM2(dutycycle);
		 	 			 	 	  PWM_PA5_TIM8(0);






		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);

		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);



		 //	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);




		 	 	   }

		 	 	   // left

		 	 	   else if(lx<=Buff2 && (ly<=BuffP && ly>=BuffN) )

		 	 	   {

		 	 	   dutycycle=map(lx,-128,Buff2,fast,0);



		 	 		PWM(dutycycle);
		 	 			 	 	  PWM_PA1_TIM2(dutycycle);
		 	 			 	 	  PWM_PA5_TIM8(0);






		 //	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);

		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);



		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);


		 	 	   }

		 	 	   //clockwise

		 	 	   else if(rx>=Buff1 && (ry<=BuffP && ry>=BuffN) ){

		 	 	   dutycycle=map(rx,Buff1,127,0,100);





		 	 		PWM(dutycycle);
		 	 			 	 	  PWM_PA1_TIM2(dutycycle);
		 	 			 	 	  PWM_PA5_TIM8(dutycycle);






		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);

		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);



		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);










		 	 	   }



		 	 	  // //anticlockwise

		 	 	   else if(rx<=Buff2 && (ry<=BuffP && ry>=BuffN) ){

		 	 	   dutycycle=map(rx,-128,Buff2,100,0);

		 	 		PWM(dutycycle);
		 	 			 	 	  PWM_PA1_TIM2(dutycycle);
		 	 			 	 	  PWM_PA5_TIM8(dutycycle);






		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);

		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);



		 	 	  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);

		 	 	   }
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
