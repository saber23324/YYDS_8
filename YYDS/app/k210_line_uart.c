#include "k210_line_uart.h"
#include "main.h"
#include "bsp_uart.h"



extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern K210 uart_K210;

static uint8_t K210_line_uartRecData[2][K210_LINE_RX_BUF_NUM];

//��main.c����3��ȫ�ֱ���
/* USER CODE BEGIN Includes */
uint8_t rx_buffer[K210_LINE_RX_BUF_NUM];   //�������ݵ�����
volatile uint8_t rx_len = 0; //�������ݵĳ���
volatile uint8_t recv_end_flag = 0; //���ս�����־λ
/* USER CODE END Includes */

/**
  * @brief          �����ǳ�ʼ��
  * @param[in]      none
  * @retval         none
  */
void K210_line_uart_init(void)
{
//    K210line_Init(K210_line_uartRecData[0], K210_line_uartRecData[1], K210_LINE_RX_BUF_NUM);
__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);  //���������ж�
HAL_UART_Receive_DMA(&huart2,rx_buffer,K210_LINE_FRAME_LENGTH);  //����DMA�����ж�

}

/**
  * @brief          ������Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ����������ָ
  * @retval         none
  */
static void uart_line_to_k210(volatile const uint8_t *uart_k210, K210 *uart_line_K210);



void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART1_IRQn 1 */
  uint8_t tmp_flag =__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); //��ȡIDLE״̬
  if((tmp_flag != RESET))//�жϽ����Ƿ����
    { 
      // recv_end_flag = 1; //���ս���
       __HAL_UART_CLEAR_IDLEFLAG(&huart2);//�����־λ
       HAL_UART_DMAStop(&huart2); 
//       uint8_t temp=__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);                 
//       rx_len =100-temp; //�������ݳ���
       HAL_UART_Receive_DMA(&huart2,rx_buffer,K210_LINE_FRAME_LENGTH);//����DMA
			uart_line_to_k210(rx_buffer,&uart_K210);
    }
  /* USER CODE END USART1_IRQn 1 */
}

//void USART2_IRQHandler(void)
//{
//    if(huart2.Instance->SR & UART_FLAG_RXNE)//���յ�����
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart2);
//    }
//    else if(USART2->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len2 = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart2);

//        if ((hdma_usart2_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart2_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len2 = K210_LINE_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart2_rx.Instance->NDTR = K210_LINE_RX_BUF_NUM;

//            //set memory buffer 1
//            //�趨������1
//            hdma_usart2_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart2_rx);

//            if(this_time_rx_len2 == K210_LINE_FRAME_LENGTH)
//            {
//                uart_line_to_k210(K210_line_uartRecData[0],&uart_K210);
//                //��¼���ݽ���ʱ��
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart2_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len2 = K210_LINE_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart2_rx.Instance->NDTR = K210_LINE_RX_BUF_NUM;

//            //set memory buffer 0
//            //�趨������0
//            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart2_rx);

//            if(this_time_rx_len2 == K210_LINE_FRAME_LENGTH)
//            {
//                //����ң��������
//                uart_line_to_k210(K210_line_uartRecData[1],&uart_K210);
////							HAL_Delay(100);
//                //��¼���ݽ���ʱ��
//            }
//        }
//    }

//}

/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
 

static void uart_line_to_k210(volatile const uint8_t *uart_k210, K210 *uart_line_K210)
{
    if (uart_k210 == NULL || uart_line_K210 == NULL)
    {
        return;
    }
if (uart_k210[0] == 0x3f  && uart_k210[1] == 0x22 && uart_k210[8] == 0x1f) // 
			{
			
//					BEEL_ON;
					uart_line_K210->Rho0=((float)((uart_k210)[3] << 8 | (uart_k210)[2]))/100;
					uart_line_K210->Rho90=((float)((uart_k210)[5] << 8 | (uart_k210)[4]))/100;
					uart_line_K210->dis_circle_y=((float)((uart_k210)[7] << 8 | (uart_k210)[6]))/100;


			}
}
	