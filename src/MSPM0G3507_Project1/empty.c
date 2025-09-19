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
// 全局变量
int A = 0;              // 转弯计数
int current_task = 0;   // 0: 设置圈数, 1: Task1 (寻迹), 2: Task2
int N = 1;              // 圈数 (1~5)
volatile bool is_setting_mode = true;  // 是否处于设置模式

SpeedConfig speedConfigs[3]; // 速度配置数组******
int currentSpeedConfig = 0; // 当前速度配置索引(0-2)*******

//BASED:基础前进速度	TURN_big:快准转向速度	TURN_mid:中转向速度	TURN_small:慢准转向速度	Rotational_speed：//自转速度	 S1://微调速度1（转弯自转的差）	 S2:7/微调速度2
SpeedConfig speedConfigs[3] = {
    // 配置1: 标准速度
    {350, 350, 210, 130, 220, 190, 0},		 //350                      0   80    230   
    // 配置2: 慢速模式						//300
    {300, 300, 130, 100, 230, 170, 0},                    //差    基础   0  170   200  50  90
    // 配置3: 快速模式						//370
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
            // 设置模式：等待按键输入
            //printf("\r\nPress KEY1 to set laps (N=%d), KEY2 to start\r\n", N);
            delay_ms(500);  // 防止打印过快
        }
        else if (current_task == 1)
        {
            Task1();  // 执行寻迹
        }
        else if (current_task == 2)
        {
            Task2();  // 完成任务后的逻辑
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










