#ifndef _HW_ENCODER_H_
#define _HW_ENCODER_H_

#include "ti_msp_dl_config.h"
#include "board.h"
typedef enum {
    FORWARD,  // 正向
    REVERSAL  // 反向
} ENCODER_DIR;

typedef struct {
    volatile long long temp_count; //保存实时计数值
    int count;         						 //根据定时器时间更新的计数值
    ENCODER_DIR dir;            	 //旋转方向
} ENCODER_RES;


int get_encoderA_count(void);
int get_encoderB_count(void);
//ENCODER_DIR get_encoderA_dir(void);
//ENCODER_DIR get_encoderB_dir(void);
void encoder_update(void);


extern int currentSpeedConfig;  // 当前使用的速度配置索引


#endif