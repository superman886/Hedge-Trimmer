/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-01     LCKFB-LP    first version
 */

#include "Ultrasonic.h"

volatile unsigned int msHcCount = 0;//ms����
volatile float distance = 0;

/******************************************************************
 * �� �� �� �ƣ�bsp_ultrasonic
 * �� �� ˵ ������������ʼ��
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע��TRIG���Ÿ����ͳ��������崮
******************************************************************/
void Ultrasonic_Init(void)
{
        
    SYSCFG_DL_init();
    //�����ʱ���жϱ�־
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //ʹ�ܶ�ʱ���ж�
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
        
}
/******************************************************************
 * �� �� �� �ƣ�Open_Timer
 * �� �� ˵ �����򿪶�ʱ��
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע��
******************************************************************/
void Open_Timer(void)
{
        
    DL_TimerG_setTimerCount(TIMER_0_INST, 0);   // �����ʱ������  
        
    msHcCount = 0;  
        
    DL_TimerG_startCounter(TIMER_0_INST);   // ʹ�ܶ�ʱ��
}

/******************************************************************
 * �� �� �� �ƣ�Get_TIMER_Count
 * �� �� ˵ ������ȡ��ʱ����ʱʱ��
 * �� �� �� �Σ���
 * �� �� �� �أ�����
 * ��       �ߣ�LC
 * ��       ע��
******************************************************************/
uint32_t Get_TIMER_Count(void)
{
    uint32_t time  = 0;  
    time   = msHcCount*1000;                         // �õ�us 
    time  += DL_TimerG_getTimerCount(TIMER_0_INST);  // �õ�ms 
        
    DL_TimerG_setTimerCount(TIMER_0_INST, 0);   // �����ʱ������  
    delay_ms(1);
    return time ;          
}

/******************************************************************
 * �� �� �� �ƣ�Close_Timer
 * �� �� ˵ �����رն�ʱ��
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע��
******************************************************************/
void Close_Timer(void)
{
    DL_TimerG_stopCounter(TIMER_0_INST);     // �رն�ʱ�� 
}

/******************************************************************
 * �� �� �� �ƣ�TIMER_0_INST_IRQHandler
 * �� �� ˵ ������ʱ���жϷ�����
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע��1ms����һ��
******************************************************************/
void TIMER_0_INST_IRQHandler(void)
{
    //��������˶�ʱ���ж�
    switch( DL_TimerG_getPendingInterrupt(TIMER_0_INST) )
    {
        case DL_TIMER_IIDX_ZERO://�����0����ж�
                msHcCount++;
            break;
        
        default://�����Ķ�ʱ���ж�
            break;
    }
}


/******************************************************************
 * �� �� �� �ƣ�Hcsr04GetLength
 * �� �� ˵ ������ȡ��������
 * �� �� �� �Σ���
 * �� �� �� �أ���������
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
volatile float t = 0;
float Hcsr04GetLength(void)
{
        /*��5�����ݼ���һ��ƽ��ֵ*/
        volatile float length = 0;
                t = 0;
        volatile float sum = 0;
        volatile unsigned int  i = 0;
        
                Close_Timer();
        
       //while(i != 5)
        //{
                SR04_TRIG(0);//trig�����źţ������͵�ƽs   
                delay_1us(10);//����ʱ�䳬��5us                        
                        
                SR04_TRIG(1);//trig�����źţ������ߵ�ƽ
                        
                delay_1us(20);//����ʱ�䳬��10us
                        
                SR04_TRIG(0);//trig�����źţ������͵�ƽ
                        
                /*Echo�����ź� �ȴ������ź�*/
                /*���뷽����ģ����Զ�����8��40KHz�����������ͬʱ�ز����ţ�echo���˵ĵ�ƽ����0��Ϊ1��
                ����ʱӦ��������ʱ����ʱ���������������ر�ģ����յ�ʱ���ز��� �Ŷ˵ĵ�ƽ����1��Ϊ0��
                ����ʱӦ��ֹͣ��ʱ������������ʱ�����µ����ʱ�伴Ϊ
                                                �������ɷ��䵽���ص���ʱ����*/
                
                while(SR04_ECHO() == 0);//echo�ȴ�����
                
                Open_Timer();   //�򿪶�ʱ�� 

               // i++;
                
                while(SR04_ECHO() > 0);
								// while(SR04_ECHO()!=0);
                
                Close_Timer();   // �رն�ʱ��  
                
                //delay_ms(100);
                                
                t = (float)Get_TIMER_Count();   // ��ȡʱ��,�ֱ���Ϊ1us
               // length = (float)t / 58.0f;   // cm  
								length = ((float)t*1.0/10*0.34)/2;   // cm 
              // sum += length;    
                                

       // }
        //length = sum/5;//���ƽ��ֵ
       // distance = length;
        return length;
}
