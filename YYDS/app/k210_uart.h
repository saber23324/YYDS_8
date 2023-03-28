#ifndef __K210_UART_H
#define __K210_UART_H

#include "main.h"

#include "usart.h"
#include "sys.h"

#define K210_RX_BUF_NUM 20u

#define K210_FRAME_LENGTH 10u



void K210_uart_init(void);

#endif
