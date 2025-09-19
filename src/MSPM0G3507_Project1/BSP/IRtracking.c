
#include <stdio.h>
#include "oled.h"
#include "Car.h"
#include "IRtracking.h"
#include "ti_msp_dl_config.h"
#include "bsp_tb6612.h"

extern int currentSpeedConfig;
extern SpeedConfig speedConfigs[3];

   
//// 速度配置（根据实际电机调整）                         Sew(右轮，左轮)
//int BASED =  330 ;   // 基础前进速度 (0-100)
//int TURN_big  =  330 ;  	// 快准转向速度
//int TURN_mid  =  150 ;  	// 中转向速度
//int TURN_small  =  100 ;  	// 慢准转向速度
//int Rotational_speed  = 280 ;   	// 自转速度
//int S1=50;							//微调速度1	
//int S2=100;							//微调速度2



/*********************寻迹***********************`.************/// 		白0     黑1
void xun_ji()								    
{
		    // 使用当前速度配置
    int BASED = speedConfigs[currentSpeedConfig].BASED;
    int TURN_big = speedConfigs[currentSpeedConfig].TURN_big;
    int TURN_mid = speedConfigs[currentSpeedConfig].TURN_mid;
    int TURN_small = speedConfigs[currentSpeedConfig].TURN_small;
    int Rotational_speed = speedConfigs[currentSpeedConfig].Rotational_speed;
    int S1 = speedConfigs[currentSpeedConfig].S1;
    int S2 = speedConfigs[currentSpeedConfig].S2;
// 1. 读取传感器状态（1=黑线，0=白地板）
    uint8_t sensors = (IR_DO1 << 6) | (IR_DO2 << 5) | (IR_DO3 << 4) |
                     (IR_DO4 << 3) | (IR_DO5 << 2) | (IR_DO6 << 1) | IR_DO7;

	
//	 //3. 打印传感器状态（调试用）
//    printf("\r\nSensors: %d%d%d%d%d%d%d  \r\n", 
//           IR_DO1, IR_DO2, IR_DO3, IR_DO4, IR_DO5, IR_DO6, IR_DO7);
	
		// 4. 控制决策
	switch(sensors) {

		case 0b0001000:  // 精确居中  444444
			//printf("FORWARD\n");
			Set_PWM(BASED,BASED);
			return;
		
		case 0b0011000:  // 偏左      33333344444
			Set_PWM(TURN_big,TURN_mid);
			return;
			
		case 0b0001100:  // 偏右     4444455555
			Set_PWM(TURN_mid,TURN_big);  // 
			return;
			
		case 0b0010000:  // 偏左      3333333      ·

			Set_PWM(TURN_big,TURN_mid);
			return;
		
		case 0b0000100:  // 偏右	55555
			Set_PWM(TURN_mid,TURN_big);
			return;
			
		case 0b0110000:  // 原地向左（慢）     2222333333
			Set_PWM(TURN_big,TURN_small);  // 
			return;
			
		case 0b0000110:  // 原地向右（慢）     5555566666
			Set_PWM(TURN_small,TURN_big);	
			return;
		
		case 0b0100000:  // 原地向左（慢）    22222222222222
			Set_PWM(TURN_big,TURN_small);
			return;
		
		case 0b0000010:  // 原地向右（慢）   6666666
			Set_PWM(TURN_small,TURN_big);	
			return;
			
		case 0b1100000:  // 原地向左（快）
			Set_PWM(-Rotational_speed,Rotational_speed);  //左转
;		
			return;
			
		case 0b0000011:  // 原地向右（快）
		Set_PWM(Rotational_speed,-Rotational_speed);  //右转

			return;
		
		case 0b0000001:  // 原地向右（快）
		Set_PWM(Rotational_speed,-Rotational_speed);  //右转
		
			return;
			
		case 0b1000000:  // 原地向左（快）
			Set_PWM(-Rotational_speed,Rotational_speed);  //左转

			return;
/********特殊情况**************/
		case 0b0000000:  // 向前走慢0.5s，再原地自转（中等）直达DO4为黑，退出return.

			//Turn_90();
			Set_PWM(250,230);
			return;
			
		case 0b1111111:  // 原地自转（慢）

			Turn_90();
			//Car_Stop();
			return;
		case 0b1111000:  // 向前走慢0.5s，再原地自转（中等）直达DO4为黑，退出return.

			Turn_90();
			//Car_Stop();
			return;
			
		case 0b1110000:  // 向前走慢0.5s，再原地自转（中等）直达DO4为黑，退出return.

			//Car_Stop();
			Turn_90();
			return;
		
		case 0b1111100:  // 向前走慢0.5s，再原地自转（中等）直达DO4为黑，退出return.
			Turn_90();
			//Car_Stop();
			return;
		
		case 0b1111110:  // 向前走慢0.5s，再原地自转（中等）直达DO4为黑，退出return.
			Turn_90();
			//Car_Stop();
			return;
	}
	
}


void Turn_90()
{
	//沿黑线右转90度
	// 向前移动，直到DO4检测到边缘
		    // 使用当前速度配置
    int BASED = speedConfigs[currentSpeedConfig].BASED;
    int Rotational_speed = speedConfigs[currentSpeedConfig].Rotational_speed;
    int S1 = speedConfigs[currentSpeedConfig].S1;
	
	Car_Stop();
	delay_ms(250);
	Set_PWM(BASED,BASED);
	delay_ms(270);
	Set_PWM(-200,200);  //左转
	delay_ms(100);
	while(IR_DO4 == 0);  // 假设WHITE是黑线的背景颜色
	// 右转，直到DO4再次检测到黑线
	Set_PWM(200,-200);  //右转
	while(IR_DO4 == 0);
	//Car_Stop();
	A++;
    if (A >= 4 * N)  // 完成 N 圈（4*N 次转弯）
    {
        Car_Stop();
        current_task = 2;  // 切换到 Task2
    }
} 











