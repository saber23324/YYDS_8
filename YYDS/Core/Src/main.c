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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER0 CODE BEGIN Includes */
#include "k210_uart.h"
#include "k210_line_uart.h"
#include "my_servo.h"

#define debug_mode 0

/* ��ʼ��С��״̬�� */
QFsm car_state;  /* ʵ��������ָ�� */
QEvent car_signal;  /* ʵ�����¼��ṹ�� */
Car car;  /* ״̬������Ҫ�õ��ı��� */
Target target;
float movex,movey,movez;//�ƶ��ķ����Ƕ�
unsigned char head_led,back_led,left_led,right_led;
PID Move_Z_pid;//���PID


PID Head_Z_pid;//����ѭ��
PID Left_Z_pid;//����ѭ��

PID K210x_pid;//K210ѭ��
PID K210y_pid;//K210ѭ��
PID K210z_pid;//K210ѭ��

unsigned char flag_move;
unsigned char flag_site;//λ�ý���0 �ر�λ�� 1ֱ�� 2 ����

int head_bias;//ͷѭ��
int left_bias;//ͷѭ��
int debugprintmode;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* printf�ض��� */
int fputc(int ch, FILE *f)
{

    uint8_t temp[1] = {ch};
    HAL_UART_Transmit(&huart5, temp, 1, 2);//huart1��Ҫ������������޸�
    return ch;
}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rx_buff=0;
uint8_t uartRecData[22] = {0};//������
uint8_t Z_get[3];
uint8_t seveo[10]={0};//͸��
uint8_t uart_seveo[9]={0};
uint8_t uart_head[4]={0};//ѭ��
uint8_t uart_left[4]={0};//ѭ��

uint8_t uart_code[13]={0};//��ά����ɫ
uint8_t uart_color[12]={0};//��ά����ɫ
K210 uart_K210;


float yaw_expect=0;
float yaw;
float GY;
uint8_t YawH;
uint8_t YawL;
uint8_t WyL;
uint8_t WyH;



uint8_t piancha = 0x57;




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     int16_t time=0;

	
    /* �ж����ĸ����ڴ������ж� */
		
		if(huart ->Instance == UART5)
	{
			HAL_UART_Receive_IT(&huart5, uart_seveo, 9);
			if (uart_seveo[0] == 0xf1  && uart_seveo[8] == 0x11  )
			{
				
				for(int idx=0;idx<7;idx++)
				{
					seveo[idx]=uart_seveo[idx+1];
				}
			}
    }

		
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
    HAL_UART_Receive_IT(huart, &rx_buff, 1);
}

/**
* @file �� tim.c �ļ�������������
*/


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
	MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	 /* ����uart3�ж� */

		mpu_control_init();
		HAL_Delay(500);
		mpuInit();
		K210_uart_init();
		K210_line_uart_init();
		
		
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
				HAL_UART_Receive_IT(&huart5, uart_seveo, 9);
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);  //���������ж�
		HAL_UART_Receive_DMA(&huart4,uart_left,3);  //����DMA�����ж�
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  //���������ж�
		HAL_UART_Receive_DMA(&huart6,uart_head,3);  //����DMA�����ж�	

		
		//mpu_control_init();

	
	HAL_TIM_Base_Start_IT(&htim5);   /* ��ʱ��5ʹ�� */
  /* ��ʼ�����PWM */
	Drive_init();
	System_Init();
	
		init_maps();
		init_bias();
//Servo_mid_Pwm(0x8C,0x9B,0x50,0xC0,0xE8);//��ǰ�򿪻�е��
//							Servo_Pwm(0x8C,0x9B,0x50,0xC0,0xE8);	
		
//		Servo_mid_Pwm(0x8c,0x90,0x96,0x96,0x60);
		
		K210y_pid.SetPoint=0;
		K210x_pid.SetPoint=0;
		
//		K210y_pid.SetPoint=207.5;
//		K210x_pid.SetPoint=160.5;
//		flag_site=2;//����ͷ��λ��
//		flag_move=1;
////		K210_Line_begin(ENABLE_UART);
//		HAL_Delay(500);
//		K210y_pid.SetPoint=207.5;
//		K210x_pid.SetPoint=160.5;
//		flag_site=2;//����ͷ��λ��
//		flag_move=1;
		
//		uart_code[0]=1;


    K210_Line_begin(DISABLE_UART); 
		K210_Code_begin(GET_CODE);		

		//K210_Code_begin(DISABLE_UART);   

////����
//	uart_color[0]=1;
//	uart_color[1]=2;
//	uart_color[2]=3;
//	uart_color[5]=1;
//	uart_color[4]=2;
//	uart_color[3]=3;

//	uart_code[0]=2;
//	uart_code[2]=3;
//	uart_code[4]=1;
//	uart_code[6]=3;
//	uart_code[8]=1;
//	uart_code[10]=2;
//	uart_code[12]=0;


							
Servo_move_on_act;
flag_site=0;
seveo[6]=0x14;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#if !debug_mode
		QFsm_dispatch(&car_state, &car_signal);  //״̬�ַ�
		#endif
//debug����
#if debug_mode
		HAL_Delay(100);
		switch (seveo[6]) {
							case 1 :
								 Servo_Pwm(seveo[0],seveo[1],seveo[2],seveo[3],seveo[4],seveo[5]);
								 break; /* ��ѡ�� */
							case 2 :
								 debug_color_up();
								 break; /* ��ѡ�� */
							case 3 :
								 debug_color_down();
								 break; /* ��ѡ�� */
							case 4 :
									K210_Line_begin(BRIDGE);
									debugprintmode=0x10;
								 debug_bridge();
								 break; /* ��ѡ�� */
							case 5 :
//								K210_Line_begin(Get_6Line);
								debugprintmode=0x10;
								 debug_endput1();
								 break; /* ��ѡ�� */
							case 6 :
								K210_Line_begin(Get_6Line);
								debugprintmode=0x10;
								 debug_endput2();
								 break; /* ��ѡ�� */
							case 7 :
								 break; /* ��ѡ�� */
							case 8 :
							debugprintmode=8;
								 printf("print0x08 :%f,%f,%d,%d \n",movex,movey,left_bias,head_bias);
								 break; /* ��ѡ�� */
							case 9 :
							debugprintmode=9;
									printf("print0x09:%f,%f \n",movez,mpu_crtl.yaw);
								 break; /* ��ѡ�� */
							case 0x10 :
							debugprintmode=0x10;
								printf("print0x10:%f,%f,%f,%f \n",movex,movey,uart_K210.Rho0,uart_K210.Rho90);
								 break; /* ��ѡ�� */
							case 0x11 :
							K210_Line_begin(Get_6LineL);
							HAL_Delay(500);
							K210y_pid.SetPoint=B_k210.GetCode.ynum;
							K210x_pid.SetPoint=B_k210.GetCode.xnum;
							flag_site=2;//����ͷ��λ��
							flag_move=1;
							debugprintmode=0x10;
							seveo[6]=0x16;
							break; /* ��ѡ�� */
							case 0x12 :
								K210_Line_begin(Get_6LineL);
								HAL_Delay(500);
								K210y_pid.SetPoint=B_k210.GetCode2.ynum;
								K210x_pid.SetPoint=B_k210.GetCode2.xnum;
								flag_site=2;//����ͷ��λ��
								flag_move=1;
							debugprintmode=0x10;
								seveo[6]=0x16;
								 break; /* ��ѡ�� */
							case 0x13 :
								K210_Line_begin(BRIDGE);
								HAL_Delay(500);
								K210y_pid.SetPoint=B_k210.GetBridge.ynum;
								K210x_pid.SetPoint=B_k210.GetBridge.xnum;
								flag_site=2;//����ͷ��λ��	
							flag_move=1;
								seveo[6]=0x16;
								 break; /* ��ѡ�� */
							case 0x14 :
								 K210_Line_begin(Get_6Line);
								HAL_Delay(500);
								K210y_pid.SetPoint=B_k210.Get_End.ynum;
								K210x_pid.SetPoint=B_k210.Get_End.xnum;
								flag_site=2;//����ͷ��λ��	
								flag_move=1;
								debugprintmode=0x10;
								seveo[6]=0x16;
								 break; /* ��ѡ�� */
							case 0x15 :
								 K210_Line_begin(Get_6Line);
								HAL_Delay(500);
								K210y_pid.SetPoint=B_k210.Get_End2.ynum;
								K210x_pid.SetPoint=B_k210.Get_End2.xnum;
								flag_site=2;//����ͷ��λ��	
							flag_move=1;
															debugprintmode=0x10;

								seveo[6]=0x16;
								 break; /* ��ѡ�� */
							case 0x16 :
								if(fabs(K210x_pid.SetPoint-uart_K210.Rho0) <1 && fabs(K210y_pid.SetPoint-uart_K210.Rho90) <1 && flag_site==2)
								{
									HAL_Delay(300);
									if(fabs(K210x_pid.SetPoint-uart_K210.Rho0) <1 && fabs(K210y_pid.SetPoint-uart_K210.Rho90) <1 && flag_site==2)
									{
										if(fabs(K210x_pid.SetPoint-uart_K210.Rho0) <1 && fabs(K210y_pid.SetPoint-uart_K210.Rho90) <1 && flag_site==2)
										{
											BEEL_ON;
											HAL_Delay(300);
											BEEL_OFF;
											flag_move=0;
											seveo[6]=0x06;
										}
									}
								}
								 break; /* ��ѡ�� */
							case 0x17 :
									if(fabs(K210x_pid.SetPoint-uart_K210.Rho0) <1 && fabs(K210y_pid.SetPoint-uart_K210.Rho90) <1 && flag_site==2)
									{
										HAL_Delay(300);
										if(fabs(K210x_pid.SetPoint-uart_K210.Rho0) <1 && fabs(K210y_pid.SetPoint-uart_K210.Rho90) <1 && flag_site==2)
										{
											if(fabs(K210x_pid.SetPoint-uart_K210.Rho0) <1 && fabs(K210y_pid.SetPoint-uart_K210.Rho90) <1 && flag_site==2)
											{
												BEEL_ON;
												HAL_Delay(300);
												BEEL_OFF;
											}
										}
									}
								 break; /* ��ѡ�� */
							
							default : /* ��ѡ�� */
							 break; /* ��ѡ�� */

							}
#endif

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint16_t tim_cnt_20 = 0;
uint16_t tim_cnt_15 = 0;
uint16_t tim_cnt_10 = 0;
uint16_t text = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim5.Instance)
    {
        tim_cnt_20++;
        tim_cnt_15++;
        tim_cnt_10++;

        /* 15ms�������������� ���л�����ֵ�˲� */
        if(tim_cnt_15 >= 15)
        {
            tim_cnt_15 = 0;
						head_led =get_HEAD_LED();
						back_led =get_BACK_LED();
						left_led =get_LEFT_LED();
						right_led =get_RIGHT_LED();
					
					if(head_led!=0x0f && flag_move==1 &&flag_site!=2)
					HAL_UART_Transmit_DMA(&huart6, &piancha,1); 
					if(left_led!=0x3f && flag_move==1 && flag_site!=2)			
					HAL_UART_Transmit_DMA(&huart4, &piancha,1);					
//					if(flag_site==1 || flag_site==4 ){
//					if(head_led!=0x0f && flag_move==1)
//					HAL_UART_Transmit(&huart6, &piancha, 1, 3);}//ѭ��ģ��}
//					if(flag_site==3|| flag_site==4){
//					if(left_led!=0x3f && flag_move==1)			
//					HAL_UART_Transmit(&huart4, &piancha, 1, 3);//ѭ��ģ��
//					}
				}

        /* �ٶȱջ����� */
        if(tim_cnt_20 >= 20)
        {
            tim_cnt_20 = 0;
					
					

					//	Kinematic_Analysis(&target,movex,movey,movez);
					float MZ=Move_Z_pid.SetPoint-mpu_crtl.yaw;//����һ��
					
					float MPU_Z=0;//������ѭ��
					float Head_Y=0;//���ѭ��
					float Left_X=0;//���ѭ��
					if(MZ<-180)
					{
						MZ+=360;
					}
					else if(MZ>180)
					{
						MZ-=360;
					}
					if(flag_move==1  ){
					MPU_Z= pid_solve(&Move_Z_pid,MZ*10);  // ����ڶ��ʷ���
					}
					
					
					if(flag_site==1){
						Head_Y= pid_solve(&Head_Z_pid,Head_Z_pid.SetPoint-head_bias);  
						movey=Head_Y;
					}
					else if(flag_site==2)
					{
							movex=pid_solve(&K210x_pid,K210x_pid.SetPoint-uart_K210.Rho0);  
							movey=pid_solve(&K210y_pid,-(K210y_pid.SetPoint-uart_K210.Rho90)); 
						movex=MINMAX(movex,-medomspeed,medomspeed);
						movey=MINMAX(movey,-medomspeed,medomspeed);
					}
					else if(flag_site==3)
					{
						Left_X= pid_solve(&Left_Z_pid,left_bias-Left_Z_pid.SetPoint);
						movex=Left_X;
					}
					else if(flag_site==4)
					{
						Head_Y= pid_solve(&Head_Z_pid,Head_Z_pid.SetPoint-head_bias);  
						Left_X= pid_solve(&Left_Z_pid,left_bias-Left_Z_pid.SetPoint);
						movex=MINMAX(Left_X,-medomspeed,medomspeed);
						movey=MINMAX(Head_Y,-medomspeed,medomspeed);
					}
					else if(flag_site==0){
						movex=movex;
						movey=movey;

					}
					else if(flag_site==5)//������У׼
					{
//						movey=0;
//						movex=0;	
						movex=pid_solve(&K210x_pid,K210x_pid.SetPoint-uart_K210.Rho0);  
						movey=pid_solve(&K210y_pid,-(K210y_pid.SetPoint-uart_K210.Rho90)); 
						movex=MINMAX(movex,-slowspeed,slowspeed);
						movey=MINMAX(movey,-slowspeed,slowspeed);
						MPU_Z=pid_solve(&K210z_pid,-(K210z_pid.SetPoint-uart_K210.dis_circle_y));
//						if(fabs(K210z_pid.SetPoint-uart_K210.dis_circle_y) <0.2  )
//						{

////							mpuInit();
//							flag_site=0;
//							flag_move=0;
//						}
					}

					movez=	MPU_Z;

					
					//printf("movez:%f,%f \n",movez,mpu_crtl.yaw);//��������bug  Ϊɶprintf�����һ��ͻ�Ӱ�촮�ڣ�����������������������������
					//printf("movez:%f,%f,%d,%f \n",movez,mpu_crtl.yaw,head_bias,movey);?????????????????????????///�����治���
//						Speed_Calculation(&target,movex,movey,movez);
//					Speed_Calculation(&target,movex,movey,movez);
					//г���������ڿ���
//					ramp_calc(&target.BL_ramp, target.Target_BL);
//					ramp_calc(&target.BR_ramp, target.Target_BR);
//					ramp_calc(&target.HL_ramp, target.Target_HL);
//					ramp_calc(&target.HR_ramp, target.Target_HR);
//					target.Target_BL = (uint16_t)(target.BL_ramp.out);
//					target.Target_BR = (uint16_t)(target.BR_ramp.out);
//					target.Target_HL = (uint16_t)(target.HL_ramp.out);
//					target.Target_HR = (uint16_t)(target.HR_ramp.out);
					
					if(flag_move){
					Speed_Calculation(&target,movex,movey,movez);
					Set_Pwm(target.Target_HL,target.Target_HR,target.Target_BL,target.Target_BR);
					}
					else
					{
//						Speed_Calculation(&target,0,0,0);
					Set_Pwm(0,0,0,0);
					}
					
					#if debug_mode
					switch (debugprintmode) {
					case 8 :
							
								 printf("print0x08 :%f,%f,%d,%d \n",movex,movey,left_bias,head_bias);
								 break; /* ��ѡ�� */
							case 9 :
							
									printf("print0x09:%f,%f \n",movez,mpu_crtl.yaw);
								 break; /* ��ѡ�� */
							case 0x10 :
							
								printf("print0x10:%f,%f,%f,%f \n",movex,movey,uart_K210.Rho0,uart_K210.Rho90);
								 break; /* ��ѡ�� */
				}
					#endif
        }

        if(tim_cnt_10 >= 10)
        {
            tim_cnt_10 = 0;

        }
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
