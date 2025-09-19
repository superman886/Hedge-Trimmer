
#include <stdio.h>
#include "oled.h"
#include "Car.h"
#include "IRtracking.h"
#include "ti_msp_dl_config.h"
#include "bsp_tb6612.h"

extern int currentSpeedConfig;
extern SpeedConfig speedConfigs[3];

   
//// �ٶ����ã�����ʵ�ʵ��������                         Sew(���֣�����)
//int BASED =  330 ;   // ����ǰ���ٶ� (0-100)
//int TURN_big  =  330 ;  	// ��׼ת���ٶ�
//int TURN_mid  =  150 ;  	// ��ת���ٶ�
//int TURN_small  =  100 ;  	// ��׼ת���ٶ�
//int Rotational_speed  = 280 ;   	// ��ת�ٶ�
//int S1=50;							//΢���ٶ�1	
//int S2=100;							//΢���ٶ�2



/*********************Ѱ��***********************`.************/// 		��0     ��1
void xun_ji()								    
{
		    // ʹ�õ�ǰ�ٶ�����
    int BASED = speedConfigs[currentSpeedConfig].BASED;
    int TURN_big = speedConfigs[currentSpeedConfig].TURN_big;
    int TURN_mid = speedConfigs[currentSpeedConfig].TURN_mid;
    int TURN_small = speedConfigs[currentSpeedConfig].TURN_small;
    int Rotational_speed = speedConfigs[currentSpeedConfig].Rotational_speed;
    int S1 = speedConfigs[currentSpeedConfig].S1;
    int S2 = speedConfigs[currentSpeedConfig].S2;
// 1. ��ȡ������״̬��1=���ߣ�0=�׵ذ壩
    uint8_t sensors = (IR_DO1 << 6) | (IR_DO2 << 5) | (IR_DO3 << 4) |
                     (IR_DO4 << 3) | (IR_DO5 << 2) | (IR_DO6 << 1) | IR_DO7;

	
//	 //3. ��ӡ������״̬�������ã�
//    printf("\r\nSensors: %d%d%d%d%d%d%d  \r\n", 
//           IR_DO1, IR_DO2, IR_DO3, IR_DO4, IR_DO5, IR_DO6, IR_DO7);
	
		// 4. ���ƾ���
	switch(sensors) {

		case 0b0001000:  // ��ȷ����  444444
			//printf("FORWARD\n");
			Set_PWM(BASED,BASED);
			return;
		
		case 0b0011000:  // ƫ��      33333344444
			Set_PWM(TURN_big,TURN_mid);
			return;
			
		case 0b0001100:  // ƫ��     4444455555
			Set_PWM(TURN_mid,TURN_big);  // 
			return;
			
		case 0b0010000:  // ƫ��      3333333      ��

			Set_PWM(TURN_big,TURN_mid);
			return;
		
		case 0b0000100:  // ƫ��	55555
			Set_PWM(TURN_mid,TURN_big);
			return;
			
		case 0b0110000:  // ԭ����������     2222333333
			Set_PWM(TURN_big,TURN_small);  // 
			return;
			
		case 0b0000110:  // ԭ�����ң�����     5555566666
			Set_PWM(TURN_small,TURN_big);	
			return;
		
		case 0b0100000:  // ԭ����������    22222222222222
			Set_PWM(TURN_big,TURN_small);
			return;
		
		case 0b0000010:  // ԭ�����ң�����   6666666
			Set_PWM(TURN_small,TURN_big);	
			return;
			
		case 0b1100000:  // ԭ�����󣨿죩
			Set_PWM(-Rotational_speed,Rotational_speed);  //��ת
;		
			return;
			
		case 0b0000011:  // ԭ�����ң��죩
		Set_PWM(Rotational_speed,-Rotational_speed);  //��ת

			return;
		
		case 0b0000001:  // ԭ�����ң��죩
		Set_PWM(Rotational_speed,-Rotational_speed);  //��ת
		
			return;
			
		case 0b1000000:  // ԭ�����󣨿죩
			Set_PWM(-Rotational_speed,Rotational_speed);  //��ת

			return;
/********�������**************/
		case 0b0000000:  // ��ǰ����0.5s����ԭ����ת���еȣ�ֱ��DO4Ϊ�ڣ��˳�return.

			//Turn_90();
			Set_PWM(250,230);
			return;
			
		case 0b1111111:  // ԭ����ת������

			Turn_90();
			//Car_Stop();
			return;
		case 0b1111000:  // ��ǰ����0.5s����ԭ����ת���еȣ�ֱ��DO4Ϊ�ڣ��˳�return.

			Turn_90();
			//Car_Stop();
			return;
			
		case 0b1110000:  // ��ǰ����0.5s����ԭ����ת���еȣ�ֱ��DO4Ϊ�ڣ��˳�return.

			//Car_Stop();
			Turn_90();
			return;
		
		case 0b1111100:  // ��ǰ����0.5s����ԭ����ת���еȣ�ֱ��DO4Ϊ�ڣ��˳�return.
			Turn_90();
			//Car_Stop();
			return;
		
		case 0b1111110:  // ��ǰ����0.5s����ԭ����ת���еȣ�ֱ��DO4Ϊ�ڣ��˳�return.
			Turn_90();
			//Car_Stop();
			return;
	}
	
}


void Turn_90()
{
	//�غ�����ת90��
	// ��ǰ�ƶ���ֱ��DO4��⵽��Ե
		    // ʹ�õ�ǰ�ٶ�����
    int BASED = speedConfigs[currentSpeedConfig].BASED;
    int Rotational_speed = speedConfigs[currentSpeedConfig].Rotational_speed;
    int S1 = speedConfigs[currentSpeedConfig].S1;
	
	Car_Stop();
	delay_ms(250);
	Set_PWM(BASED,BASED);
	delay_ms(270);
	Set_PWM(-200,200);  //��ת
	delay_ms(100);
	while(IR_DO4 == 0);  // ����WHITE�Ǻ��ߵı�����ɫ
	// ��ת��ֱ��DO4�ٴμ�⵽����
	Set_PWM(200,-200);  //��ת
	while(IR_DO4 == 0);
	//Car_Stop();
	A++;
    if (A >= 4 * N)  // ��� N Ȧ��4*N ��ת�䣩
    {
        Car_Stop();
        current_task = 2;  // �л��� Task2
    }
} 











