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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"

#include "cJSON.h"
#include <string.h>
#include "HC_SR04.h"



#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern float Motor1Speed ;//speed of motor1
extern float Motor2Speed ;//Speed of motor2

extern tPid pidMotor1Speed;//Structure of pid control
extern tPid pidMotor2Speed;
extern tPid pidFollow;    
extern tPid pidMPU6050YawMovement;  //MPU6050 PID control
extern uint8_t Usart1_ReadBuf[255];	//Serial 1 buffer array
float p,i,d,a,b;//Json variable
uint8_t OledString[50];
extern float Mileage;

extern tPid pidHW_Tracking;
uint8_t g_ucaHW_Read[4] = {0};
int8_t g_cCurState = 0;
int8_t g_cLastState = 0; 
float g_fHW_PID_Out;//Infrared alignment
float g_fHW_PID_Out1;//Motor1
float g_fHW_PID_Out2;//Motor2

uint8_t g_ucUsart3ReceiveData;  

uint8_t Usart3String[50];//
float g_fHC_SR04_Read;//Ultra distance
float g_fFollow_PID_Out;//PID follow speed


float pitch,roll,yaw; 

float  g_fMPU6050YawMovePidOut = 0.00f; //
float  g_fMPU6050YawMovePidOut1 = 0.00f; //Motor1 output
float  g_fMPU6050YawMovePidOut2 = 0.00f; //Motor2 output

uint8_t g_ucMode = 0; 
//小车运动模式标志位 0:显示功能、1:PID循迹模式、2:手机遥控普通运动模式、3.超声波避障模式、4:PID跟随模式、5:遥控角度闭环
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();			
  OLED_Clear()  	; 
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//Start Timer 1 pwm output
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//Timer 1 channel 4
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//Start timer2
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);				//Start Timer2 interrupt
  HAL_TIM_Base_Start_IT(&htim4);                //Start timer4 interrupt
  
  HAL_TIM_Base_Start_IT(&htim1);                
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//Start serial 1 receive interupt
  PID_init();
  HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  //Serial 3 receive data
  
  HAL_Delay(500);
  MPU_Init(); 
  while(MPU_Init()!=0);
  while(mpu_dmp_init()!=0);

//  cJSON *cJsonData ,*cJsonVlaue;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);//Current mode
	OLED_ShowString(0,6,OledString,12);	
	
	sprintf((char *)Usart3String," g_ucMode:%d",g_ucMode);// bluetooth
	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
	
	if(g_ucMode == 0)
	{
	//0LED
		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);
		OLED_ShowString(0,0,OledString,12);
		
		sprintf((char*)OledString, "Mileage:%.2f", Mileage);
		OLED_ShowString(0,1,OledString,12);
		
		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());
		OLED_ShowString(0,2,OledString,12);
		
		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());
		OLED_ShowString(0,3,OledString,12);
		
		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);
		OLED_ShowString(0,4,OledString,12);
		
		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);
		OLED_ShowString(0,5,OledString,12);
		
	
		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
	
		
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  
		
		
		motorPidSetSpeed(0,0);
	}
	if(g_ucMode == 1)
	{
	///****   PID infrared tracking ******************/
	g_ucaHW_Read[0] = READ_HW_OUT_1;
	g_ucaHW_Read[1] = READ_HW_OUT_2;
	g_ucaHW_Read[2] = READ_HW_OUT_3;
	g_ucaHW_Read[3] = READ_HW_OUT_4;

	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("forword\r\n");
		g_cCurState = 0;
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("Right\r\n");
		g_cCurState = -1;
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("Quick right\r\n");
		g_cCurState = -2;
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
	{
//		printf("Quick right\r\n");
		g_cCurState = -3;
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
	{
//		printf("Quick left\r\n");
		g_cCurState = 1;
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
	{
//		printf("Quick left\r\n");
		g_cCurState = 2;
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
	{
//	    printf("Quick left\r\n");
		g_cCurState = 3;
	}
	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cCurState);

	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;
	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;
	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;
	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
	if(g_cCurState != g_cLastState)
	{
		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);
	}
	
	g_cLastState = g_cCurState;

	}
	if(g_ucMode == 2)
	{
		//***************Remote control mode***********************//
		//serial 3 interrupt
	}
	if(g_ucMode == 3)
	{
		//************Ultrasonic obstacle avoidance mode***************//

		if(HC_SR04_Read() > 25)
		{
			motorPidSetSpeed(1,1);
			HAL_Delay(100);
		}
		else{	
			motorPidSetSpeed(-1,1);// Right in place
			HAL_Delay(500);
			if(HC_SR04_Read() > 25)
			{
				motorPidSetSpeed(1,1);
				HAL_Delay(100);
			}
			else{
				motorPidSetSpeed(1,-1);//Left
				HAL_Delay(1000);
				if(HC_SR04_Read() >25)
				{
					 motorPidSetSpeed(1,1);
					HAL_Delay(100);
				}
				else{
					motorPidSetSpeed(-1,-1);
					HAL_Delay(1000);
					motorPidSetSpeed(-1,1);
					HAL_Delay(50);
				}
			}
		}
	}
	if(g_ucMode == 4)
	{
	//**********PID follow***********//
		g_fHC_SR04_Read=HC_SR04_Read();
		if(g_fHC_SR04_Read < 60){  
			g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);
			if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;
			if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
			motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);
		}
		else motorPidSetSpeed(0,0);
		HAL_Delay(10);
	}
	if(g_ucMode == 5)
	{
	//*************MPU6050 yaw angle PID control****************//

		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);
	   
	   //mpu_dmp_get_data(&pitch,&roll,&yaw);
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //****
		
		
		g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);

		g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;
		g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
		if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;
		if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
		if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
		if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
		motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
	
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
