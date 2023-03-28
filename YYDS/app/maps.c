#include "maps.h"

#include "main.h"

MAPS map;
Bias_number B_number;
Bias_k210 B_k210;
uint8_t uart_re[5];
//串口解析路径
static void uart_to_maps(volatile const uint8_t *maps_buf, MAPS *map);
//初始化
void init_maps(void);
void init_bias(void);
static void uart_to_bias(Bias_number *B_number,Bias_k210 *B_k210);

static void uart_to_maps(volatile const uint8_t *maps_buf, MAPS *map)
{
	map->set_head[0]=4;
	map->set_head[1]=2;
	map->set_head[2]=4;
	map->set_head[3]=0;
	map->set_head[4]=-2;
	map->set_head[5]=-2;
	
	map->set_left[0]=0;
	map->set_left[1]=0;
	map->set_left[2]=0;
	map->set_left[3]=-2;
	map->set_left[4]=0;
	map->set_left[5]=0;
	

	
	map->turn[0]=TAKE_FIRST_SITE;
	map->turn[1]=GETCODECOLOR;
	map->turn[2]=Find_Bridge;
	map->turn[3]=0;
	map->turn[4]=0;
	map->turn[5]=Find_End;
	
	
	map->set_head[6]=4;
	map->set_head[7]=0;
	map->set_head[8]=2;
	
	map->set_head[9]=4;
	map->set_head[10]=0;
	map->set_head[11]=-2;
	map->set_head[12]=2;
	map->set_head[13]=0;


	
	map->set_left[6]=0;
	map->set_left[7]=0;
	map->set_left[8]=0;
	
	map->set_left[9]=0;
	map->set_left[10]=-2;
	map->set_left[11]=0;
	map->set_left[12]=0;
	map->set_left[13]=0;
	
	
	map->turn[6]=TURN_LEFT;
	map->turn[7]=TURN_LEFT;
	map->turn[8]=GETCODECOLOR;
	map->turn[9]=Find_Bridge;
	map->turn[10]=0;
	map->turn[11]=0;
	map->turn[12]=Find_End;
	map->turn[13]=Finish;
	
//		//不过桥

//map->turn[3]=0;
//	map->turn[4]=Find_End;
//	map->turn[5]=0;
	
//	map->set_head[2]=0;
//	map->set_head[3]=3;
//	map->set_head[4]=-2;
//	map->set_head[5]=0;
	
//map->set_left[2]=-2;
//	map->set_left[3]=0;
//	map->set_left[4]=0;
//	map->set_left[5]=0;

//		map->set_head[9]=0;
//	map->set_head[10]=2;
//	map->set_head[11]=2;
//	map->set_head[12]=2;
//	map->set_head[13]=0;
//	
//	map->set_left[9]=-2;
//	map->set_left[10]=0;
//	map->set_left[11]=0;
//	map->set_left[12]=0;
//	map->set_left[13]=0;
//	
//		map->turn[9]=Find_Bridge;
//	map->turn[10]=0;
//	map->turn[11]=Find_End;
//	map->turn[12]=Finish;
//	map->turn[13]=Finish;
//	
	
	map->size=15;
	map->idx=0;
}
//x->0
//y->90
/*
x++ rh0++ left--

y++ rh90--head--
*/
static void uart_to_bias(Bias_number *B_number,Bias_k210 *B_k210)
{
	B_number->TakeSet.bias_num_head=-336;
	B_number->TakeSet.bias_num_left=0;
	
	B_number->GetCode.bias_num_head=300;
	B_number->GetCode.bias_num_left=300;
	
	B_number->GetBridge.bias_num_head=-210;
//	B_number->GetBridge.bias_num_left=256;
	B_number->GetBridge.bias_num_left=0;

	B_number->Get_End.bias_num_head=-280;
	B_number->Get_End.bias_num_left=320;

//	B_k210->GetCodeColor.xnum=180.3;
	B_k210->GetCodeColor.xnum=184.3;
	B_k210->GetCodeColor.ynum=113.59;
	B_k210->GetCodeColor.degree=183.99;
	
	B_k210->GetCode.xnum=161.9;//155
	B_k210->GetCode.ynum=198.92;
	B_k210->GetCode.degree=185.1;
	
		B_k210->GetCode2.xnum=169.3;
	B_k210->GetCode2.ynum=190.399;
	//B_k210->GetCode2.ynum=182.3;
	B_k210->GetCode2.degree=184.9;
	
//	B_k210->GetBridge.xnum=168;
//B_k210->GetBridge.ynum=220.9;
//B_k210->GetBridge.degree=186.99;

	B_k210->GetBridge.xnum=175;
B_k210->GetBridge.ynum=225.9;
B_k210->GetBridge.degree=186.11;

	B_k210->Get_End.xnum=106.54;
	B_k210->Get_End.ynum=160.19;
	B_k210->Get_End.degree=184.1;
	
	
	B_k210->Get_End2.xnum=184.306;
	B_k210->Get_End2.ynum=191.9;
	B_k210->Get_End2.degree=184.99;
}
void init_maps(void)
{
	uart_to_maps(uart_re,&map);
}

void init_bias(void)
{
	uart_to_bias(&B_number,&B_k210);
}