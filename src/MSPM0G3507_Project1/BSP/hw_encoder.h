#ifndef _HW_ENCODER_H_
#define _HW_ENCODER_H_

#include "ti_msp_dl_config.h"
#include "board.h"
typedef enum {
    FORWARD,  // ����
    REVERSAL  // ����
} ENCODER_DIR;

typedef struct {
    volatile long long temp_count; //����ʵʱ����ֵ
    int count;         						 //���ݶ�ʱ��ʱ����µļ���ֵ
    ENCODER_DIR dir;            	 //��ת����
} ENCODER_RES;


int get_encoderA_count(void);
int get_encoderB_count(void);
//ENCODER_DIR get_encoderA_dir(void);
//ENCODER_DIR get_encoderB_dir(void);
void encoder_update(void);


extern int currentSpeedConfig;  // ��ǰʹ�õ��ٶ���������


#endif