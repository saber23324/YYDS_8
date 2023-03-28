#ifndef __K210_LINE_UART_H
#define __K210_LINE_UART_H

#include "main.h"

#include "usart.h"
#include "sys.h"

#define K210_LINE_RX_BUF_NUM 18u

#define K210_LINE_FRAME_LENGTH 9u



void K210_line_uart_init(void);

#endif
