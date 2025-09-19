/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-04     LCKFB-LP    first version
 */

#ifndef _BSP_BLUETOOTH_H_
#define _BSP_BLUETOOTH_H_
 
#include "string.h"
#include "board.h"

//是否开启串口0调试     1开始  0关闭
#define     DEBUG   1


#define  BLERX_LEN_MAX  200 


#define BLUETOOTH_LINK      ( ( DL_GPIO_readPins( HC_05_PORT, HC_05_STATE_PIN ) & HC_05_STATE_PIN ) ? 1 : 0 )      

#define CONNECT             1       //蓝牙连接成功
#define DISCONNECT          0       //蓝牙连接断开

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

