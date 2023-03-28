#ifndef __MPU_H
#define __MPU_H

#include "main.h"

#include "usart.h"
#include "sys.h"

#define SBUS_RX_BUF_NUM 44u

#define RC_FRAME_LENGTH 22u

typedef __packed struct
{
		float yaw_expect;
		float yaw;
		float GY;

} MPU_ctrl_t;

void mpu_control_init(void);
void mpuInit(void);
extern MPU_ctrl_t mpu_crtl;
void mpureturnzero(void);

#endif
