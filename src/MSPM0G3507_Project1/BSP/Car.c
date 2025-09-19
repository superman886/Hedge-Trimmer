#include "bsp_tb6612.h"
#include "Car.h"
#include "board.h"
#include "bsp_tb6612.h"
#include <stdlib.h>

float Velcity_Kp=1.0,  Velcity_Ki=0.5,  Velcity_Kd; //����ٶ�PID����
//Turn_Left ��Turn_Rightһ����Turn_Right�����   (,)�����һ��
//AO������ BO������

/***************************************************************************
�������ܣ������PID�ջ�����
��ڲ��������ҵ���ı�����ֵ
����ֵ  �������PWM
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //������ر���
		static int ControlVelocityA, Last_biasA; //��̬�������������ý�������ֵ��Ȼ����
		
		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocityA+=Velcity_Ki*(Bias-Last_biasA)+Velcity_Kp*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_biasA=Bias;	
	    if(ControlVelocityA>3600) ControlVelocityA=3600;
	    else if(ControlVelocityA<-3600) ControlVelocityA=-3600;
		return ControlVelocityA; //�����ٶȿ���ֵ
}

/***************************************************************************
�������ܣ������PID�ջ�����
��ڲ��������ҵ���ı�����ֵ
����ֵ  �������PWM
***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //������ر���
		static int ControlVelocityB, Last_biasB; //��̬�������������ý�������ֵ��Ȼ����
		
		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocityB+=Velcity_Ki*(Bias-Last_biasB)+Velcity_Kp*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
	   	Last_biasB=Bias;	
	    if(ControlVelocityB>3600) ControlVelocityB=3600;
	    else if(ControlVelocityB<-3600) ControlVelocityB=-3600;
		return ControlVelocityB; //�����ٶȿ���ֵ
}





// ������ƺ���
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



