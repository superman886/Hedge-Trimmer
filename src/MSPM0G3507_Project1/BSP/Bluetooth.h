/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-04     LCKFB-LP    first version
 */

#ifndef _BSP_BLUETOOTH_H_
#define _BSP_BLUETOOTH_H_
 
#include "string.h"
#include "board.h"

//�Ƿ�������0����     1��ʼ  0�ر�
#define     DEBUG   1


#define  BLERX_LEN_MAX  200 


#define BLUETOOTH_LINK      ( ( DL_GPIO_readPins( HC_05_PORT, HC_05_STATE_PIN ) & HC_05_STATE_PIN ) ? 1 : 0 )      

#define CONNECT             1       //�������ӳɹ�
#define DISCONNECT          0       //�������ӶϿ�

extern unsigned char BLERX_BUFF[BLERX_LEN_MAX];
extern unsigned char BLERX_FLAG;
extern unsigned char BLERX_LEN;

void Send_Bluetooth_Data(char *dat);
void  BLE_Send_Bit (unsigned char ch);
void BLE_send_String(unsigned char *str);
void Bluetooth_Init(void);
unsigned char Get_Bluetooth_ConnectFlag(void);
void Bluetooth_Mode(void);
void Receive_Bluetooth_Data(void);
void Clear_BLERX_BUFF(void);
#endif

