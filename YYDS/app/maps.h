#ifndef MAPS_H
#define MAPS_H
#include "struct_typedef.h"

#define map_len 15





typedef enum {
		TURN_LEFT=1,
		TURN_RIGHT,
		TURN_DISABLE,
		TAKE_FIRST_SITE,
		GETNUMBER,
    GETCODECOLOR,

    Find_Bridge,	
		Find_End,
		Return,
		Finish,
} Maps_Sign;

typedef struct {
	int num_head;
	int set_head[map_len];
	int num_left;
	int set_left[map_len];
	Maps_Sign turn[map_len];
	int size;
	uint8_t idx;
}MAPS;

extern MAPS map;
typedef struct {
	int bias_num_head;
	int bias_num_left;

}HandL;
typedef struct {
	float xnum;
	float ynum;
	float degree;
}K210x_y;
typedef struct {
	HandL TakeSet;
	HandL GetCode;
	HandL GetBridge;
	HandL Get_End;
}Bias_number;
typedef struct {
	K210x_y TakeSet;
	K210x_y GetCodeColor;
	K210x_y GetCode;
	K210x_y GetCode2;
	K210x_y GetBridge;
	K210x_y Get_End;
	K210x_y Get_End2;
}Bias_k210;
extern Bias_number B_number;
extern Bias_k210 B_k210;
void init_maps(void);
void init_bias(void);

#endif
