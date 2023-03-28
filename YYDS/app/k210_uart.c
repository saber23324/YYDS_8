#include "k210_uart.h"
#include "main.h"
#include "bsp_uart.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;


extern uint8_t uart_code[13];//二维码颜色
extern uint8_t uart_color[12];//二维码颜色

static uint8_t K210_uartRecData[2][K210_RX_BUF_NUM];

/**
  * @brief          陀螺仪初始化
  * @param[in]      none
  * @retval         none
  */
void K210_uart_init(void)
{
    K210code_Init(K210_uartRecData[0], K210_uartRecData[1], K210_RX_BUF_NUM);
}

/**
  * @brief          陀螺仪协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 陀螺仪数据指
  * @retval         none
  */
static void uart_to_k210(volatile const uint8_t *uart_code_buffer, uint8_t *uart_color,uint8_t *uart_code);




void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = K210_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = K210_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == K210_FRAME_LENGTH)
            {
                uart_to_k210(K210_uartRecData[0], uart_color,uart_code);
                //记录数据接收时间
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = K210_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = K210_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == K210_FRAME_LENGTH)
            {
                //处理遥控器数据
                uart_to_k210(K210_uartRecData[1],uart_color,uart_code);
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



static void uart_to_k210(volatile const uint8_t *uart_code_buffer, uint8_t *uart_color,uint8_t *uart_code)
{
    if (uart_code_buffer == NULL || uart_color == NULL)
    {
        return;
    }
if (uart_code_buffer[0] == 0x3f  && uart_code_buffer[1] == 0x22 && uart_code_buffer[9] == 0x1f) // 
			{
				
				if(uart_code_buffer[2] == 0x01)//数据类型
				{
					int ixx=0;
					for(int ix=3;ix<9;ix++)
					{
						uart_code[ixx]=uart_code_buffer[ix];
						ixx+=2;
					}
					//HAL_UART_Transmit(&huart2, (uart_code_buffer+3), 6, 3);//先试试
					uart_code[12]=0;//操作码给0
				}
				if(uart_code_buffer[2] == 0x02)//color
				{
					for(int ix=3;ix<9;ix++)
					{
						uart_color[ix-3]=uart_code_buffer[ix];
					}

				}
		
			}
}
