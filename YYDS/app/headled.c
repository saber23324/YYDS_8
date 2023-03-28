#include "headled.h"
#include "main.h"
#include "bsp_uart.h"


extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart4_rx;

static uint8_t leduartRecData[2][SBUS_RX_BUF_NUM];


/**
  * @brief          陀螺仪初始化
  * @param[in]      none
  * @retval         none
  */
void led_control_init(void)
{
    LED_Init(leduartRecData[0], leduartRecData[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          陀螺仪协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 陀螺仪数据指
  * @retval         none
  */
static void sbus_to_led(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl);



void led_Transmit(void)
{
	unsigned char ch=0x57;
	HAL_UART_Transmit(&huart4, &ch, 1, 2);//huart1需要根据你的配置修改 	
}


void USART4_IRQHandler(void)
{
    if(huart4.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(UART4->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart4);

        if ((hdma_usart4_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart4_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart4_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart4_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart4_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart4_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_led(leduartRecData[0], &mpu_crtl);
							
                //记录数据接收时间
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart4_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart4_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart4_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart4_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_led(leduartRecData[1], &mpu_crtl);
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
static void sbus_to_led(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl)
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
