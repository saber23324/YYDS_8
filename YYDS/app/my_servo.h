#ifndef __MY_SERVO_H
#define __MY_SERVO_H

#include "main.h"

#define Servo_move_on_act Servo_Pwm(0x4B,0x90,0x70,0x6A,0xBA,0X0A)
#define Servo_extend_act Servo_Pwm(0x8C,0x90,0x52,0xBA,0xBA,0X0A)


//÷∏¡Ó¬Î
#define ENABLE_UART  0x49
#define DISABLE_UART  0x50
#define RESET_COLOR 0x56
#define GET_COLOR   0x51
#define GET_CODE   0x52
#define Get_6Line 0x53 //83
#define BRIDGE   0x54
#define Put_End   0x55
#define Get_6LineL   0x56
#define CATCH	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0x95)
#define LOOSEN	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0xE8)
void Servo_Pwm(uint16_t yaw,uint16_t pitch0,uint16_t pitch1,uint16_t pitch2,uint16_t paw,uint16_t time);
//void Servo_mid_Pwm(uint16_t yaw,uint16_t pitch0,uint16_t pitch1,uint16_t pitch2,uint16_t paw);

void K210_Code_begin(uint8_t mode);
void K210_Line_begin(uint8_t mode);

void catch2_1(void);
void catch2_2(void);
void catch2_3(void);
void catch1_1(void);
void catch1_2(void);
void catch1_3(void);
void put_brige1(void);
void put_brige2(void);
void put_brige3(void);

void catch_brige1(void);
void catch_brige2(void);
void catch_brige3(void);

void end_put_2_1(void);
void end_put_2_2(void);
void end_put_2_3(void);
void end_put_1_1(void);
void end_put_1_2(void);
void end_put_1_3(void);
void end_put_1_11(void);
void end_put_1_22(void);
void end_put_1_33(void);

void put_carsite0(void);
void put_carsite1(void);
void put_carsite2(void);
void catch_carsite0(void);
void catch_carsite1(void);
void catch_carsite2(void);

void debug_color_down(void);
void debug_color_up(void);
void debug_bridge(void);
void debug_endput1(void);
void debug_endput2(void);
void debug_endput11(void);

#endif