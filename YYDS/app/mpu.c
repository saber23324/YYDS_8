#include "mpu.h"
#include "main.h"
#include "bsp_uart.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t uartRecData[2][SBUS_RX_BUF_NUM];
MPU_ctrl_t mpu_crtl;
/**
  * @brief          陀螺仪初始化
  * @param[in]      none
  * @retval         none
  */
void mpu_control_init(void)
{
    MCU_Init(uartRecData[0], uartRecData[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          陀螺仪协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 陀螺仪数据指
  * @retval         none
  */
static void sbus_to_mpu(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl);

void MPU6050Set(unsigned char ch)
{
	/* 发送一个字节数据到USART1 */

	HAL_UART_Transmit(&huart3, &ch, 1, 2);//huart1需要根据你的配置修改 	
		

}
extern PID Move_Z_pid;//舵机PID
void mpuInit(void)
{
unsigned char ch[5];
//	//格式化
//	ch[0]=0xff;
//	ch[1]=0xAA;
//	ch[2]=0x00;
//	ch[3]=0x01;
//	ch[4]=0x00;
//	HAL_UART_Transmit(&huart3, ch, 5, 0xffff);
//	HAL_Delay(500);
	//（设置陀螺仪静止阈值为0.05°/s，0x0032=50，50/1000=0.05(°/s)）
//	ch[0]=0xff;
//	ch[1]=0xAA;
//	ch[2]=0x61;
//	ch[3]=0x32;
//	ch[4]=0x00;
//	HAL_UART_Transmit(&huart3, ch, 5, 0xffff);
//	HAL_Delay(500);
	
	
	//清零
	ch[0]=0xff;
	ch[1]=0xAA;
	ch[2]=0x76;
	ch[3]=0x00;
	ch[4]=0x00;
	HAL_UART_Transmit(&huart3, ch, 5, 0xffff);
	HAL_Delay(500);
	Move_Z_pid.SetPoint=0;
}
void mpureturnzero(void)
{
	unsigned char ch[5];
	//清零
	ch[0]=0xff;
	ch[1]=0xAA;
	ch[2]=0x76;
	ch[3]=0x00;
	ch[4]=0x00;
	HAL_UART_Transmit(&huart3, ch, 5, 0xffff);
	HAL_Delay(500);
	Move_Z_pid.SetPoint=0;
}
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_mpu(uartRecData[0], &mpu_crtl);
							
                //记录数据接收时间
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_mpu(uartRecData[1], &mpu_crtl);
                //记录数据接收时间
            }
        }
    }

}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_mpu(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl)
{
    if (sbus_buf == NULL || mpu_ctrl == NULL)
    {
        return;
    }

		static float Bias,Last_Bias,integral,derivative;  //偏差和历史值
		
		uint8_t YawH,YawL,WyL,WyH;
		

			if (sbus_buf[12] == 0x53) // yaw 倾角的话为0x53, 陀螺仪为 0x52
			{
				YawH = sbus_buf[18];
				YawL = sbus_buf[17];
				mpu_ctrl->yaw=(((short)(YawH<<8))|YawL)/32768.0*180.0;
				Bias=mpu_ctrl->yaw_expect-mpu_ctrl->yaw;   //提取偏差
				//printf("yaw:%f \n",yaw);
			}
			if (sbus_buf[1] == 0x52) // yaw 倾角的话为0x53, 陀螺仪为 0x52
			{
				WyH = sbus_buf[5];
				WyL = sbus_buf[4];
				mpu_ctrl->GY=(((short)(WyH<<8))|WyL)/32768*2000;

			//	printf("GY:%f \n",GY);
			}
		
		
}
