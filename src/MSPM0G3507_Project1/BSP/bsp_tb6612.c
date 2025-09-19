/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-08     LCKFB-LP    first version
 */
#include "bsp_tb6612.h"
#include <stdlib.h>

/******************************************************************
 * �� �� �� �ƣ�AO_Control
 * �� �� ˵ ����A�˿ڵ������
 * �� �� �� �Σ�dir��ת���� 1��ת0��ת   speed��ת�ٶȣ���Χ��0 ~ 99��
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
//�Ұ��һ��
void Set_PWM(int pwma,int pwmb)
{
	//��ʹ����˾��D153C����ģ���ʱ��PA12��AIN2��PA13��AIN1,PB16��BIN2��PB0����BIN1
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
