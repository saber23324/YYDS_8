#include "Mcnamu_wheel_drive.h"
#include "pwm_num.h"



/* �ٶȱջ���λ��ʽPID */
double PIDCalc(PID *pp, double NextPoint)   
{
	double dError,                              /* ��ǰ΢�� */
				 Error;                               /* ƫ�� */
	Error = pp->SetPoint - NextPoint;           /* ƫ��ֵ=�趨ֵ-����ֵ����ǰֵ��*/
	
	/* �����޷� */
	if(pp->SumError > 4000) pp->SumError = 400;
	if(pp->SumError < -4000) pp->SumError = -400;
	
	pp->SumError += Error;                      /* ����=����+ƫ��  --ƫ����ۼ� */
	dError = pp->LastError - pp->PrevError;     /* ��ǰ΢�� = ������ - ֮ǰ��� */
	pp->PrevError = pp->LastError;              /* ���¡�֮ǰ�� */
	pp->LastError = Error;                      /* ���¡������ */
	return (pp->Kp * Error                      /* ������ = �������� * ƫ�� */
			+   pp->Ki *  pp->SumError              /* ������ = ���ֳ��� * ������ */
			+   pp->Kd * dError                     /* ΢���� = ΢�ֳ��� * ��ǰ΢�� */
				 );
}

// ����PID
float pid_solve(PID *pid, float error)
{
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = error;
    pid->out_i += error;

    if (pid->Ki != 0) pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->Ki, pid->i_max / pid->Ki);

    return pid->Kp * pid->out_p + pid->Ki * pid->out_i + pid->Kd * pid->out_d;
}


void PID_CREATE(PID *pp, double _kp, double _ki, double _kd, double _low_pass, float max_p, float max_i, float max_d)
{
    pp->Kp = _kp;
    pp->Ki = _ki;
    pp->Kd = _kd;
    pp->low_pass = _low_pass;
    pp->out_p = 0;
    pp->out_i = 0;
    pp->out_d = 0;
    pp->p_max = max_p;
    pp->i_max = max_i;
    pp->d_max = max_d;
}



/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(Target *settarget,float Vx,float Vy,float Vz)
{
	
	settarget->Target_HL   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	settarget->Target_HR   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	settarget->Target_BL   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	settarget->Target_BR   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}

/**
 * @file        �ٶȽ��㣬�������ʽ����
 * @note        �������ġ�Mecanum��������о���
 * @date        2021
 */
void Speed_Calculation(Target *settarget,float X_speed, float Y_speed, float Z_speed)
{
	static float last_X_speed;
	static float last_Y_speed;
	static float last_Z_speed;
	if(Z_speed>1200) Z_speed = 1200;
	else if(Z_speed < -1200) Z_speed = -1200;   // ��ת���ٶ��޷�
//	if(Z_speed>1800) Z_speed = 1800;
//	else if(Z_speed < -1800) Z_speed = -1800;   // ��ת���ٶ��޷�
	if(fabs(X_speed-last_X_speed)>100){
	if(X_speed>0){
		if(last_X_speed<X_speed)
		{
			last_X_speed+=20;
		}
		else
		{
			last_X_speed=X_speed;
		}
	}
	else if(X_speed<0)
	{
		if(last_X_speed>X_speed)
		{
			last_X_speed-=20;
		}
		else
		{
			last_X_speed=X_speed;
		}
	}
	else
		last_X_speed=0;
}
	else{
	if(X_speed>0){
		if(last_X_speed<X_speed)
		{
			last_X_speed+=5;
		}
		else
		{
			last_X_speed=X_speed;
		}
	}
	else if(X_speed<0)
	{
		if(last_X_speed>X_speed)
		{
			last_X_speed-=5;
		}
		else
		{
			last_X_speed=X_speed;
		}
	}
	else
		last_X_speed=0;
}
	
if(fabs(Y_speed-last_Y_speed)>100){
	if(Y_speed>0){
		if(last_Y_speed<Y_speed)
		{
			last_Y_speed+=20;
		}
		else
		{
			last_Y_speed=Y_speed;
		}
	}
	else if(Y_speed<0)
	{
		if(last_Y_speed>Y_speed)
		{
			last_Y_speed-=20;
		}
		else
		{
			last_Y_speed=Y_speed;
		}
	}
	else
		last_Y_speed=0;
}
else
{
	if(Y_speed>0){
		if(last_Y_speed<Y_speed)
		{
			last_Y_speed+=5;
		}
		else
		{
			last_Y_speed=Y_speed;
		}
	}
	else if(Y_speed<0)
	{
		if(last_Y_speed>Y_speed)
		{
			last_Y_speed-=5;
		}
		else
		{
			last_Y_speed=Y_speed;
		}
	}
	else
		last_Y_speed=0;
}
//	if(last_Z_speed<Z_speed)
//	{
//		last_Z_speed+=20;
//	}
//	else
//	{
//		last_Z_speed=Z_speed;
//	}
//	settarget->Target_HL= last_X_speed - 0.3*last_Z_speed- last_Y_speed;
//	settarget->Target_HR= last_X_speed + 0.3*last_Z_speed+ last_Y_speed;
//	settarget->Target_BL= last_X_speed - 0.3*last_Z_speed+ last_Y_speed;
//	settarget->Target_BR= last_X_speed + 0.3*last_Z_speed- last_Y_speed;
	settarget->Target_HL= last_X_speed - 0.3*Z_speed- last_Y_speed;
	settarget->Target_HR= last_X_speed + 0.3*Z_speed+ last_Y_speed;
	settarget->Target_BL= last_X_speed - 0.3*Z_speed+ last_Y_speed;
	settarget->Target_BR= last_X_speed + 0.3*Z_speed- last_Y_speed;
}
/**
  * @brief          �ٶȽ��㣬�������ʽ����,����Ϊг������������
	* @author         gsw
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void Speed_Calculation_Ramp(Target *settarget,int X_speed, int Y_speed, int Z_speed)
{
    if(Z_speed>100) Z_speed = 100;
    else if(Z_speed < -100) Z_speed = -100;   // ��ת���ٶ��޷�

		settarget->Target_HL= X_speed - Z_speed- Y_speed;
    settarget->Target_HR= X_speed + Z_speed+ Y_speed;
    settarget->Target_BL= X_speed - Z_speed+ Y_speed;
    settarget->Target_BR= X_speed + Z_speed- Y_speed;
	
    settarget->HL_ramp.input= X_speed - Z_speed- Y_speed;
    settarget->HR_ramp.input= X_speed + Z_speed+ Y_speed;
    settarget->BL_ramp.input= X_speed - Z_speed+ Y_speed;
    settarget->BR_ramp.input= X_speed + Z_speed- Y_speed;
}

void wheel_init()
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_ALL);
}
/* �������ת������ */




		//�ٶ�ת����Ƶ��
uint16_t speed2pwm(int wheel,float speed)
{
		int frz;
		unsigned short psc;
		unsigned short arr;
	speed=fabsf(speed);
	speed=MINMAX(speed,1,Maxspeed);//�Է�ֹΪ0 ����һ�� һ��ʱ�䲻������ ���������

	frz = (speed/1000)*MaxFrz;

		if(1<=frz && frz<=MaxFrz)
	{
		if (frz<=max_arrpsc_num)
		{
			psc=ARRPSC[frz-1][0];
			arr=ARRPSC[frz-1][1];
		}
		else
		{
			psc=1;
			arr=84000000/((psc)*frz);
		}
	}
	switch (wheel){
		case wheel_HL:
			TIM8->ARR=arr;
			TIM8->PSC=psc;
			break;
		case wheel_HR:
			TIM11->ARR=arr;
			TIM11->PSC=psc;
			break;
		case wheel_BL:
			TIM3->ARR=arr;
			TIM3->PSC=psc;
			break;
		case wheel_BR:
			TIM10->ARR=arr;
			TIM10->PSC=psc;
			break;
	}
		
	return 1;
}
void wheel_HL_speed_set(float speed)
{

	if(speed > 0){
		HAL_GPIO_WritePin(DIR_HL_GPIO_Port, DIR_HL_Pin, GPIO_PIN_RESET);

	}
	else if(speed < 0){
		speed *= -1;
		HAL_GPIO_WritePin(DIR_HL_GPIO_Port, DIR_HL_Pin, GPIO_PIN_SET);
	}
	else if(speed == 0)
	{
		HAL_GPIO_TogglePin(DIR_HL_GPIO_Port, DIR_HL_Pin);
	}
		speed2pwm(wheel_HL,speed);
}

void wheel_HR_speed_set(float speed)
{


	if(speed > 0){
		
		HAL_GPIO_WritePin(DIR_HR_GPIO_Port, DIR_HR_Pin, GPIO_PIN_SET);
		
	}
	else if(speed < 0){
			
			HAL_GPIO_WritePin(DIR_HR_GPIO_Port, DIR_HR_Pin, GPIO_PIN_RESET);
	}
		else if(speed == 0)
	{
		HAL_GPIO_TogglePin(DIR_HR_GPIO_Port, DIR_HR_Pin);
	}
	speed2pwm(wheel_HR,speed);
}

void wheel_BL_speed_set(float speed)
{


	if(speed > 0){
		
		HAL_GPIO_WritePin(DIR_BL_GPIO_Port, DIR_BL_Pin, GPIO_PIN_RESET);
		
	}
	else if(speed < 0){
		speed *= -1;
		HAL_GPIO_WritePin(DIR_BL_GPIO_Port, DIR_BL_Pin, GPIO_PIN_SET);
	}
			else if(speed == 0)
	{
		HAL_GPIO_TogglePin(DIR_BL_GPIO_Port, DIR_BL_Pin);
	}
	speed2pwm(wheel_BL,speed);
}

void wheel_BR_speed_set(float speed)
{

	if(speed > 0){
		
		HAL_GPIO_WritePin(DIR_BR_GPIO_Port, DIR_BR_Pin, GPIO_PIN_SET);
		
	}
	else if(speed < 0){
		speed *= -1;
		HAL_GPIO_WritePin(DIR_BR_GPIO_Port, DIR_BR_Pin, GPIO_PIN_RESET);
	}
	else if(speed == 0)
	{
		HAL_GPIO_TogglePin(DIR_BR_GPIO_Port, DIR_BR_Pin);
	}
	speed2pwm(wheel_BR,speed);
}

void Set_Pwm(float hl,float hr,float bl,float br)
{

//	printf("V:%f,%f,%f,%f \n",hl,hr,bl,br);
	wheel_HL_speed_set(hl);
	wheel_HR_speed_set(hr);
	wheel_BL_speed_set(bl);
	wheel_BR_speed_set(br);

}



/**
  * @brief          �ĸ�����pwm��ʼ��
  * @author         gsw
  * @param[in]      none
  * @retval         ���ؿ�
  */
extern PID Move_Z_pid;//���PID
extern PID Head_Z_pid;
extern PID Left_Z_pid;
extern PID K210x_pid;//K210ѭ��
extern PID K210y_pid;//K210ѭ��
extern PID K210z_pid;//K210ѭ��

extern Target target;
void Drive_init()
{
		ramp_init(&target.BL_ramp, MOVE_CONTROL_TIME * 0.001f, PERIOD_MAX, PERIOD_MINI);
    ramp_init(&target.BR_ramp, MOVE_CONTROL_TIME * 0.001f, PERIOD_MAX, PERIOD_MINI);
		ramp_init(&target.HL_ramp, MOVE_CONTROL_TIME * 0.001f, PERIOD_MAX, PERIOD_MINI);
    ramp_init(&target.HR_ramp, MOVE_CONTROL_TIME * 0.001f, PERIOD_MAX, PERIOD_MINI);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);//Ϊ������chanall������ ����
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//	
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,30);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,30);
	__HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,30);
	__HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,30);
	
		PID_CREATE(&Move_Z_pid, 15, 0,15, 0.8, 7, 7, 15);  
		PID_CREATE(&Head_Z_pid, 0.2, 0,4, 0.8, 7, 7, 15); 
		PID_CREATE(&Left_Z_pid, 0.2, 0,4, 0.8, 7, 7, 15);
		PID_CREATE(&K210y_pid, 4, 0.05,0, 0.8, 7, 7, 15); 
		PID_CREATE(&K210x_pid, 4, 0.05,0, 0.8, 7, 7, 15); 
		PID_CREATE(&K210z_pid, 10, 0,0.5, 0.8, 7, 7, 15); 
		Move_Z_pid.SetPoint=0;
		Head_Z_pid.SetPoint=0;
		Left_Z_pid.SetPoint=0;
		K210y_pid.SetPoint=0;
		K210x_pid.SetPoint=0;
		K210z_pid.SetPoint=188.0;
	
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//Ϊ������chanall������  ��е��
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);//���
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);//���
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);//���
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);//���
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,0);
		flag_move=1;//��ʼ��
		flag_site=1;
}

void speedup(int submovex,int submovey,int delaytime ,int speed)
{
if(speed>0){
while(submovex<=speed)
	{
			submovex+=25;
			HAL_Delay(25);
			if(submovex>=speed)
				{
					submovex=speed;
					break;
				}
	}
	}
if(speed<0){
while(1)
	{
			submovex-=25;
			HAL_Delay(25);
			if(submovex<speed)
				{
					submovex=speed;
					break;
				}
	}
	}

	
}