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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "mpu6050.h"
#include "control.h"
#include "encoder.h"
#include "dqn.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX 7200
#define PWM_MIN -7200
#define MedAngle -1.45
MPU6050_t MPU6050;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int Moto1,Moto2,EncoderLeft,EncoderRight,Vertical_out,Velocity_out,Turn_out,PWM_out;
float pitch,roll,yaw; 		    //ŷ����
short aacx,aacy,aacz;			//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;		//������ԭʼ����
float temp;						//�¶�

int Moto;
extern int max_num;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

HAL_TIM_Encoder_Start (&htim2 ,TIM_CHANNEL_ALL );
HAL_TIM_Encoder_Start (&htim4 ,TIM_CHANNEL_ALL );


		HAL_TIM_Base_Start_IT (&htim3 );
HAL_GPIO_WritePin (GPIOA ,GPIO_PIN_5,GPIO_PIN_SET );

   while (MPU6050_Init(&hi2c2) == 1);
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
}

/* USER CODE BEGIN 4 */
/*�޷�����*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ==&htim3)
	{
		printf("123\r\n");
		 
		EncoderLeft=GetEncoder(2);
		EncoderRight=GetEncoder(4);																		//��ȡ������ת��
		
//	HAL_GPIO_TogglePin (GPIOC ,GPIO_PIN_13 );
		MPU6050_Read_All(&hi2c2, &MPU6050);//�õ�����������
			 if(MPU6050.KalmanAngleX ||MPU6050 .Gyro_X_RAW){HAL_GPIO_WritePin (GPIOC ,GPIO_PIN_13 ,GPIO_PIN_RESET );HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1  );
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4  );}
		else HAL_GPIO_WritePin (GPIOC ,GPIO_PIN_13 ,GPIO_PIN_SET );
		if (1){		// my_main(MPU6050.KalmanAngleX,MPU6050.Gyro_X_RAW) || 
			Vertical_out=Vertical(MedAngle,MPU6050.KalmanAngleX,MPU6050.Gyro_X_RAW+253);	//ֱ��������
			Velocity_out=Velocity(0, EncoderLeft,EncoderRight);							//�ٶȻ�����
			Turn_out=Turn(MPU6050 .Gyro_Z_RAW, 0);														  //ת�򻷼���
			PWM_out=Vertical_out-Velocity_out;								//�������
			Moto1=PWM_out-Turn_out;
			Moto2=PWM_out+Turn_out;											//�����������
		}
		else{
			switch (max_num){		// 5
				case 0: Moto = -6000; break;
				case 1: Moto = -1000; break;
				case 2: Moto = -100; break;
				case 3: Moto = -40; break;
				case 4: Moto = -10; break;
				case 5: Moto = 0; break;
				case 6: Moto = 10; break;
				case 7: Moto = 40; break; 
				case 8: Moto = 100; break;
				case 9: Moto = 1000; break;
				case 10: Moto = 6000; break;
				default: Moto = 0;
			}
			Moto1 = Moto;
			Moto2 = Moto;
		}
		Limit(&Moto1,&Moto2);
		if(MPU6050.KalmanAngleX >40||MPU6050.KalmanAngleX <-40) {Moto1 =0;Moto2 =0;}		//PWM�޷�
		Load (Moto1 ,Moto2  );
//		printf ("LEFT:%d\n",EncoderLeft);
//		printf ("RIGHT:%d\n",EncoderRight);
		
	}

}
int fputc(int ch,FILE *f)
{
HAL_UART_Transmit (&huart2  ,(uint8_t *)&ch,1,HAL_MAX_DELAY );

return ch;

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
