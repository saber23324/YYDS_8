#include "headled.h"
#include "main.h"
#include "bsp_uart.h"


extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart4_rx;

static uint8_t leduartRecData[2][SBUS_RX_BUF_NUM];


/**
  * @brief          �����ǳ�ʼ��
  * @param[in]      none
  * @retval         none
  */
void led_control_init(void)
{
    LED_Init(leduartRecData[0], leduartRecData[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          ������Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ����������ָ
  * @retval         none
  */
static void sbus_to_led(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl);



void led_Transmit(void)
{
	unsigned char ch=0x57;
	HAL_UART_Transmit(&huart4, &ch, 1, 2);//huart1��Ҫ������������޸� 	
}


void USART4_IRQHandler(void)
{
    if(huart4.Instance->SR & UART_FLAG_RXNE)//���յ�����
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart4_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart4_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart4_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart4_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart4_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_led(leduartRecData[0], &mpu_crtl);
							
                //��¼���ݽ���ʱ��
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart4_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart4_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart4_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart4_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_led(leduartRecData[1], &mpu_crtl);
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
static void sbus_to_led(volatile const uint8_t *sbus_buf, MPU_ctrl_t *mpu_ctrl)
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
