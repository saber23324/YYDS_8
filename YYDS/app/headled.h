#ifndef __HEADLED_H
#define __HEADLED_H

#include "main.h"

#include "usart.h"
#include "sys.h"

#define SBUS_RX_BUF_NUM 44u

#define RC_FRAME_LENGTH 22u



void led_control_init(void);
void ledInit(void);

#endif
