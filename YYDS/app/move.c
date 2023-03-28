#include "move.h"
#if 0
int move_num_x,move_num_y;
extern float movex,movey,movez;//移动的方向或角度;
extern unsigned char head_led,back_led,left_led,right_led;
extern int num_head,set_head,num_left,set_left;


void FRONT(int n)
{
		num_head=n;
		while(movex!=fastspeed)
		{
			movex++;
			HAL_Delay(10);
		}
		while(1)
		{
			//for(;back_led==0x3f;);//之行时先判断后轮 在判断前轮
			while(back_led!=0x3f)
			{HAL_Delay(10);}
			HAL_Delay(100);
			while(head_led!=0x3f)
			{HAL_Delay(10);}
			num_head--;
			HAL_Delay(100);
			printf("num_head:%d \n",num_head);
			
			if(num_head<=1){//判断当走最后一格时
				
				while(movex!=slowspeed)//减速
			{
				movex--;
				HAL_Delay(10);
			}
			
			for(;head_led!=0;);//前轮过线
			while(1)//两侧判断
				{
					if(left_led==0x00 || right_led==0x00 )//弱判断 一边即可
						{
							movex=0;//停车
							movey=0;
							break;
						}
				}
				printf("ok \n ok \n ok \n ok");
				break;
				}
				
		}
}
void BACK(int n)
{
	num_head=n;
		while(movex!=-fastspeed)
		{
			movex--;
			HAL_Delay(10);
		}
		while(movex!=-fastspeed)
		{
			for(;head_led!=0;);//之行时先判断后轮 在判断前轮
			HAL_Delay(100);
			for(;back_led!=0;);
			num_head--;
			printf("num_head:%d \n",num_head);
			
			if(num_head<=1){//判断当走最后一格时
				
				while(movex!=-slowspeed)//减速
			{
				movex++;
				HAL_Delay(10);
			}
			
			for(;back_led!=0;);//前轮过线
			while(1)//两侧判断
				{
					if(left_led==0xff || right_led==0xff )//弱判断 一边即可
						{
							movex=0;//停车
							movey=0;
							break;
						}
				}
				break;
				}
				
		}
}
void LEFT(int n)
{
	num_left=n;
		while(movey!=-fastspeed)
		{
			movey--;
			HAL_Delay(10);
		}
		while(movey!=-fastspeed)
		{
			for(;right_led!=0;);//之行时先判断后轮 在判断前轮
			HAL_Delay(100);
			for(;left_led!=0;);
			num_left--;
			printf("num_head:%d \n",num_left);
			
			if(num_left<=1){//判断当走最后一格时
				
				while(movey!=-slowspeed)//减速
			{
				movey++;
				HAL_Delay(10);
			}
			
			for(;left_led!=0;);//前轮过线
			while(1)//两侧判断
				{
					if(head_led==0xff || back_led==0xff )//弱判断 一边即可
						{
							movex=0;//停车
							movey=0;
							break;
						}
				}
				break;
				}
				
		}
}
void RIGHT(int n)
{
		num_left=n;
		while(movey!=fastspeed)
		{
			movey++;
			HAL_Delay(10);
		}
		while(movey!=fastspeed)
		{
			for(;left_led!=0;);//之行时先判断后轮 在判断前轮
			HAL_Delay(100);
			for(;right_led!=0;);
			num_left--;
			printf("num_head:%d \n",num_left);
			
			if(num_left<=1){//判断当走最后一格时
				
				while(movey!=slowspeed)//减速
			{
				movey--;
				HAL_Delay(10);
			}
			
			for(;left_led!=0;);//前轮过线
			while(1)//两侧判断
				{
					if(head_led==0xff || back_led==0xff )//弱判断 一边即可
						{
							movex=0;//停车
							movey=0;
							break;
						}
				}
				break;
				}
				
		}
}

void Move(int x,int y)
{
	int num_x,num_y;
	num_x=x-move_num_x;
	num_y=y-move_num_y;
	move_num_x=x;
	move_num_y=y;
	if(num_x<0)
	{
		num_x=0-num_x;
		BACK(num_x);
		HAL_Delay(200);
	}
	else if(num_x>0)
	{
		FRONT(num_x);
		HAL_Delay(200);
	}
	if(num_y<0)
	{
		num_y=0-num_y;
		RIGHT(num_y);
		HAL_Delay(200);
	}
	else if(num_y>0)
	{
		LEFT(num_y);
		HAL_Delay(200);
	}
}

#endif
