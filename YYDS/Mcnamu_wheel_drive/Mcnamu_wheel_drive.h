#ifndef __WHEEL_H
#define __WHEEL_H

#include "main.h"
#include "tim.h"
#include "sys.h"
#include "user_lib.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)


#define Maxspeed 1000
#define ratio 4//偏执系数  使低频部分控制更加精准
#define MaxFrz 28500

#define a_PARAMETER          (0.095f)               
#define b_PARAMETER          (0.086f)  

#define PERIOD_MINI Maxspeed
#define PERIOD_MAX 0
#define MOVE_CONTROL_TIME 0.5 //单位ms

#define slowspeed 350
#define medomspeed 450
#define fastspeed 750

#define wheel_HL 1
#define wheel_HR 2
#define wheel_BL 3
#define wheel_BR 4

typedef struct {
	volatile double SetPoint;            //设定值
	double Kp;                  //比例系数
	double Ki;                  //积分系数
	double Kd;                  //微分系数
	double LastError;           //最后一次误差数Er[-1]
	double PrevError;           //最后第二次误差数er[-2]
	double SumError;            //误差积分  

    double error;
    float i_max; //integrator_max
    float p_max; //integrator_max
    float d_max; //integrator_max
    float low_pass;
    float out_p;
    float out_i;
    float out_d;
}PID;



typedef struct {

	float Target_HL;
	float Target_HR;
	float Target_BL;
	float Target_BR;
	
	ramp_function_source_t HL_ramp;
	ramp_function_source_t HR_ramp;
	ramp_function_source_t BL_ramp;
	ramp_function_source_t BR_ramp;
}Target;     //电机目标值 归一化量程 0-100

typedef struct {

	float Rho0;
	float Rho90;
	float dis_circle_x;
	float dis_circle_y;
	unsigned char type;

}K210;     //电机目标值 归一化量程 0-100

void PID_CREATE(PID *pp, double _kp, double _ki, double _kd, double _low_pass, float max_p, float max_i, float max_d);
void Kinematic_Analysis(Target *target,float Vx,float Vy,float Vz);
void wheel_HL_speed_set(float speed);
void wheel_HR_speed_set(float speed);
void wheel_BL_speed_set(float speed);
void wheel_BR_speed_set(float speed);
void wheel_init();
void Speed_Calculation(Target *settarget,float X_speed, float Y_speed, float Z_speed);
void Speed_Calculation_Ramp(Target *settarget,int X_speed, int Y_speed, int Z_speed);
float pid_solve(PID *pid, float error);
void Drive_init();
void Set_Pwm(float hl,float hr,float bl,float br);
void speedup(int submovex,int submovey,int delaytime ,int speed);

#endif

