#include "mpu.h"
#include "main.h"
#include "bsp_uart.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t uartRecData[2][SBUS_RX_BUF_NUM];
MPU_ctrl_t mpu_crtl;
/**
  * @brief          �����ǳ�ʼ��
  * @param[in]      none
  * @retval         none
  */
void mpu_control_init(void)
{
    MCU_Init(uartRecData[0], uartRecData[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          ������Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ����������ָ
  * @retval         none
  */
static void sbus_to_mpu(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl);

void MPU6050Set(unsigned char ch)
{
	/* ����һ���ֽ����ݵ�USART1 */

	HAL_UART_Transmit(&huart3, &ch, 1, 2);//huart1��Ҫ������������޸� 	
		

}
extern PID Move_Z_pid;//���PID
void mpuInit(void)
{
unsigned char ch[5];
//	//��ʽ��
//	ch[0]=0xff;
//	ch[1]=0xAA;
//	ch[2]=0x00;
//	ch[3]=0x01;
//	ch[4]=0x00;
//	HAL_UART_Transmit(&huart3, ch, 5, 0xffff);
//	HAL_Delay(500);
	//�����������Ǿ�ֹ��ֵΪ0.05��/s��0x0032=50��50/1000=0.05(��/s)��
//	ch[0]=0xff;
//	ch[1]=0xAA;
//	ch[2]=0x61;
//	ch[3]=0x32;
//	ch[4]=0x00;
//	HAL_UART_Transmit(&huart3, ch, 5, 0xffff);
//	HAL_Delay(500);
	
	
	//����
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
	//����
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
    if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_mpu(uartRecData[0], &mpu_crtl);
							
                //��¼���ݽ���ʱ��
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_mpu(uartRecData[1], &mpu_crtl);
                //��¼���ݽ���ʱ��
            }
        }
    }

}

/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_mpu(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl)
{
    if (sbus_buf == NULL || mpu_ctrl == NULL)
    {
        return;
    }

		static float Bias,Last_Bias,integral,derivative;  //ƫ�����ʷֵ
		
		uint8_t YawH,YawL,WyL,WyH;
		

			if (sbus_buf[12] == 0x53) // yaw ��ǵĻ�Ϊ0x53, ������Ϊ 0x52
			{
				YawH = sbus_buf[18];
				YawL = sbus_buf[17];
				mpu_ctrl->yaw=(((short)(YawH<<8))|YawL)/32768.0*180.0;
				Bias=mpu_ctrl->yaw_expect-mpu_ctrl->yaw;   //��ȡƫ��
				//printf("yaw:%f \n",yaw);
			}
			if (sbus_buf[1] == 0x52) // yaw ��ǵĻ�Ϊ0x53, ������Ϊ 0x52
			{
				WyH = sbus_buf[5];
				WyL = sbus_buf[4];
				mpu_ctrl->GY=(((short)(WyH<<8))|WyL)/32768*2000;

			//	printf("GY:%f \n",GY);
			}
		
		
}
