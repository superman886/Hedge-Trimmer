#include "board.h"
#include <stdio.h>
#include "bsp_tb6612.h"
#include "Car.h"
#include "IRtracking.h"
#include "bsp_mpu6050.h" 
#include "inv_mpu.h"
#include "ararm.h"
#include "hw_timer.h"
#include "hw_encoder.h"

int32_t Get_Encoder_countA,encoderA_cnt,PWMA,Get_Encoder_countB,encoderB_cnt,PWMB;
float speed = 0;
int i=0,keynum=0,a=0;
volatile int32_t R_gEncoderCount = 0;
volatile int32_t L_gEncoderCount = 0;
// ȫ�ֱ���
int A = 0;              // ת�����
int current_task = 0;   // 0: ����Ȧ��, 1: Task1 (Ѱ��), 2: Task2
int N = 1;              // Ȧ�� (1~5)
volatile bool is_setting_mode = true;  // �Ƿ�������ģʽ

SpeedConfig speedConfigs[3]; // �ٶ���������******
int currentSpeedConfig = 0; // ��ǰ�ٶ���������(0-2)*******

//BASED:����ǰ���ٶ�	TURN_big:��׼ת���ٶ�	TURN_mid:��ת���ٶ�	TURN_small:��׼ת���ٶ�	Rotational_speed��//��ת�ٶ�	 S1://΢���ٶ�1��ת����ת�Ĳ	 S2:7/΢���ٶ�2
SpeedConfig speedConfigs[3] = {
    // ����1: ��׼�ٶ�
    {350, 350, 210, 130, 220, 190, 0},		 //350                      0   80    230   
    // ����2: ����ģʽ						//300
    {300, 300, 130, 100, 230, 170, 0},                    //��    ����   0  170   200  50  90
    // ����3: ����ģʽ						//370
    {400, 400, 260, 180, 230, 160, 0}
};


void Task1();
void Task2();

int main(void)
{
	
	board_init();
	while(1)
	{	

		if (current_task == 0)
        {
            // ����ģʽ���ȴ���������
            //printf("\r\nPress KEY1 to set laps (N=%d), KEY2 to start\r\n", N);
            delay_ms(500);  // ��ֹ��ӡ����
        }
        else if (current_task == 1)
        {
            Task1();  // ִ��Ѱ��
        }
        else if (current_task == 2)
        {
            Task2();  // ����������߼�
        }
		
	}
}


void Task1()
{
	 xun_ji();
	
	
	}


void Task2()
{
	while(1)
	{}
}










