#ifndef BSP_UART_H
#define BSP_UART_H
#include "struct_typedef.h"

extern void MCU_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void MCU_unable(void);
extern void MCU_restart(uint16_t dma_buf_num);

extern void K210code_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void K210code_unable(void);
extern void K210code_restart(uint16_t dma_buf_num);

extern void K210line_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void K210line_unable(void);
extern void K210line_restart(uint16_t dma_buf_num);

#endif
