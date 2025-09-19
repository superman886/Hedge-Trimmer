/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-08     LCKFB-LP    first version
 */
#include "bsp_tb6612.h"
#include <stdlib.h>

/******************************************************************
 * 函 数 名 称：AO_Control
 * 函 数 说 明：A端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ 99）
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
//乱版第一版
void Set_PWM(int pwma,int pwmb)
{
	//在使用我司的D153C驱动模块的时候，PA12接AIN2、PA13解AIN1,PB16接BIN2、PB0接着BIN1
	if(pwma>0) 
	{
		DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN);	//
		DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN);	//
		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwma),GPIO_PWM_C0_IDX);
	}
	else
	{
		DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN);
		DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwma),GPIO_PWM_C0_IDX);
	}
	if(pwmb>0)
	{
		
		DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN);
		DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN);////
		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwmb),GPIO_PWM_C1_IDX);
	}
    else
	{
		
		DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN);
		DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwmb),GPIO_PWM_C1_IDX);
	}		
}

//void Set_PWM(int pwma,int pwmb)
//{
//	
//	if(pwma>0)
//	{
//		DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN);
//		DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN);
//		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwmb),GPIO_PWM_C1_IDX);


//	}
//	else
//	{

//		DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN);
//		DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN);
//		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwmb),GPIO_PWM_C1_IDX);


//	}
//	if(pwmb>0)
//	{
//		
//		DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN);
//		DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN);
//		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwma),GPIO_PWM_C0_IDX);
//	}
//    else
//	{
//		
//		DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN);
//		DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN);
//		DL_Timer_setCaptureCompareValue(PWM_INST,abs(pwma),GPIO_PWM_C0_IDX);
//	}		
//}
