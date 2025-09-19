
#ifndef _BSP_IRTRACKING_H_
#define _BSP_IRTRACKING_H_
 
#include "board.h"

/**************四个红外引脚设置***********/
#define IR_DO1   ( ( DL_GPIO_readPins( IRtracking_PORT, IRtracking_DO1_PIN ) & IRtracking_DO1_PIN ) ? 1 : 0 )//B24  DO1
#define IR_DO2   ( ( DL_GPIO_readPins( IRtracking_PORT, IRtracking_DO2_PIN ) & IRtracking_DO2_PIN ) ? 1 : 0 )//B20  DO2
#define IR_DO3   ( ( DL_GPIO_readPins( IRtracking_PORT, IRtracking_DO3_PIN ) & IRtracking_DO3_PIN ) ? 1 : 0 )//B19  DO3
#define IR_DO4   ( ( DL_GPIO_readPins( IRtracking_PORT, IRtracking_DO4_PIN ) & IRtracking_DO4_PIN ) ? 1 : 0 )//B18  DO4
#define IR_DO5   ( ( DL_GPIO_readPins( IRtracking_PORT, IRtracking_DO5_PIN ) & IRtracking_DO5_PIN ) ? 1 : 0 )//A7	DO5
#define IR_DO6   ( ( DL_GPIO_readPins( IRtracking_PORT, IRtracking_DO6_PIN ) & IRtracking_DO6_PIN ) ? 1 : 0 )//B2   DO6
#define IR_DO7   ( ( DL_GPIO_readPins( IRtracking_PORT, IRtracking_DO7_PIN ) & IRtracking_DO7_PIN ) ? 1 : 0 )//B3   DO7

void xun_ji();								
void xun_ji2();					
void Turn_90();



// 速度配置结构体
typedef struct {
    int BASED;         // 基础前进速度
    int TURN_big;      // 快准转向速度
    int TURN_mid;      // 中转向速度
    int TURN_small;    // 慢准转向速度
    int Rotational_speed; // 自转速度
    int S1;           // 微调速度1
    int S2;           // 微调速度2
} SpeedConfig;


// 声明外部变量
extern int A;
extern int current_task;
extern int N;  



#endif

