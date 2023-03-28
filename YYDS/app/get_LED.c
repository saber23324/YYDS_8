#include "get_LED.h"

unsigned char get_LEFT_LED()
{
	unsigned char LEFT=0;
	
	LEFT |= HAL_GPIO_ReadPin(LEFT_1_GPIO_Port,LEFT_1_Pin);
	LEFT  = LEFT<<1;
	LEFT |=HAL_GPIO_ReadPin(LEFT_2_GPIO_Port,LEFT_2_Pin);
	LEFT  = LEFT<<1;
	LEFT |=HAL_GPIO_ReadPin(LEFT_3_GPIO_Port,LEFT_3_Pin);
	LEFT  = LEFT<<1;
	LEFT |=HAL_GPIO_ReadPin(LEFT_4_GPIO_Port,LEFT_4_Pin);
	LEFT  = LEFT<<1;
	LEFT |=HAL_GPIO_ReadPin(LEFT_5_GPIO_Port,LEFT_5_Pin);
	LEFT  = LEFT<<1;
	LEFT |=HAL_GPIO_ReadPin(LEFT_6_GPIO_Port,LEFT_6_Pin);

	return LEFT;
	
}
unsigned char get_RIGHT_LED()
{
		unsigned char RIGHT=0;
	
	RIGHT |= HAL_GPIO_ReadPin(RIGHT_1_GPIO_Port,RIGHT_1_Pin);
	RIGHT  = RIGHT<<1;
	RIGHT |=HAL_GPIO_ReadPin(RIGHT_2_GPIO_Port,RIGHT_2_Pin);
	RIGHT  = RIGHT<<1;
	RIGHT |=HAL_GPIO_ReadPin(RIGHT_3_GPIO_Port,RIGHT_3_Pin);
	RIGHT  = RIGHT<<1;
	RIGHT |=HAL_GPIO_ReadPin(RIGHT_4_GPIO_Port,RIGHT_4_Pin);
	RIGHT  = RIGHT<<1;
	RIGHT |=HAL_GPIO_ReadPin(RIGHT_5_GPIO_Port,RIGHT_5_Pin);
	RIGHT  = RIGHT<<1;
	RIGHT |=HAL_GPIO_ReadPin(RIGHT_6_GPIO_Port,RIGHT_6_Pin);

	return RIGHT;
}
unsigned char get_HEAD_LED()
{
		unsigned char HEAD=0;
	
//	HEAD |= HAL_GPIO_ReadPin(HEAD_1_GPIO_Port,HEAD_1_Pin);
	HEAD  = HEAD<<1;
//	HEAD |=HAL_GPIO_ReadPin(HEAD_2_GPIO_Port,HEAD_2_Pin);
	HEAD  = HEAD<<1;
	HEAD |=HAL_GPIO_ReadPin(HEAD_3_GPIO_Port,HEAD_3_Pin);
	HEAD  = HEAD<<1;
	HEAD |=HAL_GPIO_ReadPin(HEAD_4_GPIO_Port,HEAD_4_Pin);
	HEAD  = HEAD<<1;
	HEAD |=HAL_GPIO_ReadPin(HEAD_5_GPIO_Port,HEAD_5_Pin);
	HEAD  = HEAD<<1;
	HEAD |=HAL_GPIO_ReadPin(HEAD_6_GPIO_Port,HEAD_6_Pin);

	return HEAD;
}
unsigned char get_BACK_LED()
{
		unsigned char BACK=0;
	
	BACK |= HAL_GPIO_ReadPin(BACK_1_GPIO_Port,BACK_1_Pin);
	BACK  = BACK<<1;
	BACK |=HAL_GPIO_ReadPin(BACK_2_GPIO_Port,BACK_2_Pin);
	BACK  = BACK<<1;
	BACK |=HAL_GPIO_ReadPin(BACK_3_GPIO_Port,BACK_3_Pin);
	BACK  = BACK<<1;
	BACK |=HAL_GPIO_ReadPin(BACK_4_GPIO_Port,BACK_4_Pin);
	BACK  = BACK<<1;
	BACK |=HAL_GPIO_ReadPin(BACK_5_GPIO_Port,BACK_5_Pin);
	BACK  = BACK<<1;
	BACK |=HAL_GPIO_ReadPin(BACK_6_GPIO_Port,BACK_6_Pin);

	return BACK;
}
