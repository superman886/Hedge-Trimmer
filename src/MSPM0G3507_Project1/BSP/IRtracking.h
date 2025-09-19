
#ifndef _BSP_IRTRACKING_H_
#define _BSP_IRTRACKING_H_
 
#include "board.h"

/**************�ĸ�������������***********/
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



// �ٶ����ýṹ��
typedef struct {
    int BASED;         // ����ǰ���ٶ�
    int TURN_big;      // ��׼ת���ٶ�
    int TURN_mid;      // ��ת���ٶ�
    int TURN_small;    // ��׼ת���ٶ�
    int Rotational_speed; // ��ת�ٶ�
    int S1;           // ΢���ٶ�1
    int S2;           // ΢���ٶ�2
} SpeedConfig;


// �����ⲿ����
extern int A;
extern int current_task;
extern int N;  



#endif

