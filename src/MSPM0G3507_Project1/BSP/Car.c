#include "bsp_tb6612.h"
#include "Car.h"
#include "board.h"
#include "bsp_tb6612.h"
#include <stdlib.h>

float Velcity_Kp=1.0,  Velcity_Ki=0.5,  Velcity_Kd; //相关速度PID参数
//Turn_Left 和Turn_Right一样：Turn_Right慢许多   (,)相差少一点
//AO是右轮 BO是左轮

/***************************************************************************
函数功能：电机的PID闭环控制
入口参数：左右电机的编码器值
返回值  ：电机的PWM
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //定义相关变量
		static int ControlVelocityA, Last_biasA; //静态变量，函数调用结束后其值依然存在
		
		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
		
		ControlVelocityA+=Velcity_Ki*(Bias-Last_biasA)+Velcity_Kp*Bias;  //增量式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		Last_biasA=Bias;	
	    if(ControlVelocityA>3600) ControlVelocityA=3600;
	    else if(ControlVelocityA<-3600) ControlVelocityA=-3600;
		return ControlVelocityA; //返回速度控制值
}

/***************************************************************************
函数功能：电机的PID闭环控制
入口参数：左右电机的编码器值
返回值  ：电机的PWM
***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //定义相关变量
		static int ControlVelocityB, Last_biasB; //静态变量，函数调用结束后其值依然存在
		
		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
		
		ControlVelocityB+=Velcity_Ki*(Bias-Last_biasB)+Velcity_Kp*Bias;  //增量式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
	   	Last_biasB=Bias;	
	    if(ControlVelocityB>3600) ControlVelocityB=3600;
	    else if(ControlVelocityB<-3600) ControlVelocityB=-3600;
		return ControlVelocityB; //返回速度控制值
}





// 电机控制函数
void Go_Ahead() {
	Set_PWM(320,320);
}

void Go_Back() {
	Set_PWM(-200,-200);
}

void Turn_Right_Slow() {
	Set_PWM(230,-230);
}

void Turn_Left_Slow() {
	Set_PWM(-230,230);
}

void Turn_Right() {
	Set_PWM(280,230);
}

void Turn_Left() {
    Set_PWM(230,280);
}

void Car_Stop() {
		Set_PWM(0,0);
}

void Turn_Right_Fast()
	{
	Set_PWM(200,0);
	
	}
void Turn_Left_Fast()
{
		Set_PWM(0,200);
}



