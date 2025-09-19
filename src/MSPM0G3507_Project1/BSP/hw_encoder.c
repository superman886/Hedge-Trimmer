#include "hw_encoder.h"
#include <stdio.h>  // ��ӱ�׼�������ͷ�ļ�
static ENCODER_RES encoderA,encoderB;
extern int N;
extern int current_task;
extern volatile bool is_setting_mode;
uint32_t gpio;


//��ȡ������A��ֵ
int get_encoderA_count(void)
{
	return encoderA.count;
}

//��ȡ������B��ֵ
int get_encoderB_count(void)
{
	return encoderB.count;
}


////��ȡ�������ķ���
//ENCODER_DIR get_encoderA_dir(void)
//{
//	return encoderA.dir;
//}

////��ȡ�������ķ���
//ENCODER_DIR get_encoderB_dir(void)
//{
//	return encoderB.dir;
//}

//���������ݸ���
//����һ��ʱ�����
void encoder_update(void)
{
	encoderA.count = encoderA.temp_count;
	encoderB.count = encoderB.temp_count;

//	//ȷ������
//	encoderA.dir = ( encoderA.count >= 0 ) ? FORWARD : REVERSAL;
//	encoderB.dir = ( encoderB.count >= 0 ) ? FORWARD : REVERSAL;

	encoderA.temp_count = 0;//����������ֵ����
	encoderB.temp_count = 0;//����������ֵ����
}

uint32_t gpio_interrup;

/*******************************************************
�������ܣ��ⲿ�ж�ģ��������ź�
��ں�������
����  ֵ����
***********************************************************/
void GROUP1_IRQHandler(void)
{

	gpio_interrup = DL_GPIO_getEnabledInterruptStatus(GPIOA,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN|ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
	gpio = DL_GPIO_getEnabledInterruptStatus(GPIOB,KEY_KEY1_PIN|KEY_KEY2_PIN|KEY_KEY3_PIN);
	
//	gpio_interrup = DL_GPIO_getEnabledInterruptStatus(GPIOA,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN|ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
//	gpio = DL_GPIO_getEnabledInterruptStatus(GPIOB,KEY_KEY1_PIN|KEY_KEY2_PIN);
	//encoderA
//	if((gpio_interrup & ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
//	{
//		if(!DL_GPIO_readPins(GPIOA,ENCODERA_E1B_PIN))
//		{
//			encoderA.temp_count--;
//		}
//		else
//		{
//			encoderA.temp_count++;
//		}
//	}
//	else if((gpio_interrup & ENCODERA_E1B_PIN)==ENCODERA_E1B_PIN)
//	{
//		if(!DL_GPIO_readPins(GPIOA,ENCODERA_E1A_PIN))
//		{
//			encoderA.temp_count++;
//		}
//		else
//		{
//			encoderA.temp_count--;
//		}
//	}
//	//encoderB
//	if((gpio_interrup & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
//	{
//		if(!DL_GPIO_readPins(GPIOA,ENCODERB_E2B_PIN))
//		{
//			encoderB.temp_count--;
//		}
//		else
//		{
//			encoderB.temp_count++;
//		}
//	}
//	else if((gpio_interrup & ENCODERB_E2B_PIN)==ENCODERB_E2B_PIN)
//	{
//		if(!DL_GPIO_readPins(GPIOA,ENCODERB_E2A_PIN))
//		{
//			encoderB.temp_count++;
//		}
//		else
//		{
//			encoderB.temp_count--;
//		}
//	}
	
	    // KEY1: ����Ȧ�� N (1~5)
    if ((gpio & KEY_KEY1_PIN) == KEY_KEY1_PIN)
    {
		delay_ms(20);  // �򵥷���
        if (is_setting_mode)
        {
            N = (N % 5) + 1;  // 1��2��3��4��5��1
            //printf("\r\nN : %d\r\n", N);
        }
    }

    // KEY2: ȷ�ϲ���ʼѰ��
    if ((gpio & KEY_KEY2_PIN) == KEY_KEY2_PIN)
    {
		delay_ms(20);  // �򵥷���
        if (is_setting_mode)
        {
            is_setting_mode = false;
            current_task = 1;  // �л���Ѱ������
            //printf("\r\nStart! Laps: %d\r\n", N);
        }
    }
	
		    // KEY3: �л��ٶ�����
    if ((gpio & KEY_KEY3_PIN) == KEY_KEY3_PIN) {
        delay_ms(20);  // ����
        if (is_setting_mode) {
            currentSpeedConfig = (currentSpeedConfig + 1) % 3;  // ѭ���л�0,1,2
            //printf("\r\nSpeed Config: %d\r\n", currentSpeedConfig + 1);
        }
    }
	
//	DL_GPIO_clearInterruptStatus(GPIOA,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN|ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
//	DL_GPIO_clearInterruptStatus(GPIOB,KEY_KEY1_PIN|KEY_KEY2_PIN);
	
	
	DL_GPIO_clearInterruptStatus(GPIOA,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN|ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
	DL_GPIO_clearInterruptStatus(GPIOB,KEY_KEY1_PIN|KEY_KEY2_PIN|KEY_KEY3_PIN);
}

