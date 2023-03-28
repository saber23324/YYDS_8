#include "k210_line_uart.h"
#include "main.h"
#include "bsp_uart.h"



extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern K210 uart_K210;

static uint8_t K210_line_uartRecData[2][K210_LINE_RX_BUF_NUM];

//在main.c定义3个全局变量
/* USER CODE BEGIN Includes */
uint8_t rx_buffer[K210_LINE_RX_BUF_NUM];   //接收数据的数组
volatile uint8_t rx_len = 0; //接收数据的长度
volatile uint8_t recv_end_flag = 0; //接收结束标志位
/* USER CODE END Includes */

/**
  * @brief          陀螺仪初始化
  * @param[in]      none
  * @retval         none
  */
void K210_line_uart_init(void)
{
//    K210line_Init(K210_line_uartRecData[0], K210_line_uartRecData[1], K210_LINE_RX_BUF_NUM);
__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);  //开启空闲中断
HAL_UART_Receive_DMA(&huart2,rx_buffer,K210_LINE_FRAME_LENGTH);  //开启DMA接收中断

}

/**
  * @brief          陀螺仪协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 陀螺仪数据指
  * @retval         none
  */
static void uart_line_to_k210(volatile const uint8_t *uart_k210, K210 *uart_line_K210);



void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART1_IRQn 1 */
  uint8_t tmp_flag =__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); //获取IDLE状态
  if((tmp_flag != RESET))//判断接收是否结束
    { 
      // recv_end_flag = 1; //接收结束
       __HAL_UART_CLEAR_IDLEFLAG(&huart2);//清楚标志位
       HAL_UART_DMAStop(&huart2); 
//       uint8_t temp=__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);                 
//       rx_len =100-temp; //计算数据长度
       HAL_UART_Receive_DMA(&huart2,rx_buffer,K210_LINE_FRAME_LENGTH);//开启DMA
			uart_line_to_k210(rx_buffer,&uart_K210);
    }
  /* USER CODE END USART1_IRQn 1 */
}

//void USART2_IRQHandler(void)
//{
//    if(huart2.Instance->SR & UART_FLAG_RXNE)//接收到数据
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
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart2_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len2 = K210_LINE_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart2_rx.Instance->NDTR = K210_LINE_RX_BUF_NUM;

//            //set memory buffer 1
//            //设定缓冲区1
//            hdma_usart2_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart2_rx);

//            if(this_time_rx_len2 == K210_LINE_FRAME_LENGTH)
//            {
//                uart_line_to_k210(K210_line_uartRecData[0],&uart_K210);
//                //记录数据接收时间
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart2_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len2 = K210_LINE_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart2_rx.Instance->NDTR = K210_LINE_RX_BUF_NUM;

//            //set memory buffer 0
//            //设定缓冲区0
//            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart2_rx);

//            if(this_time_rx_len2 == K210_LINE_FRAME_LENGTH)
//            {
//                //处理遥控器数据
//                uart_line_to_k210(K210_line_uartRecData[1],&uart_K210);
////							HAL_Delay(100);
//                //记录数据接收时间
//            }
//        }
//    }

//}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
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
	