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

#include "Bluetooth.h"
#include "stdio.h"
#include "board.h"
unsigned char Bluetooth_ConnectFlag = 0; //蓝牙连接状态 = 0没有手机连接   = 1有手机连接
unsigned char BLERX_BUFF[BLERX_LEN_MAX];
unsigned char BLERX_FLAG = 0;
unsigned char BLERX_LEN = 0;

/******************************************************************
 * 函 数 名 称：BLE_Send_Bit
 * 函 数 说 明：向蓝牙发送单个字符
 * 函 数 形 参：ch=ASCII字符
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void BLE_Send_Bit(unsigned char ch)
{
		//当串口1忙的时候等待，不忙的时候再发送传进来的字符
		while( DL_UART_isBusy(UART_1_INST) == true );
		//发送单个字符
		DL_UART_Main_transmitData(UART_1_INST, ch);
}  

/******************************************************************
 * 函 数 名 称：BLE_send_String
 * 函 数 说 明：向蓝牙发送字符串 
 * 函 数 形 参：str=发送的字符串
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void BLE_send_String(unsigned char *str)
{
        while( str && *str ) // 地址为空或者值为空跳出
        {        
                BLE_Send_Bit(*str++);
        }        
}

/******************************************************************
 * 函 数 名 称：Clear_BLERX_BUFF
 * 函 数 说 明：清除串口接收的数据
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void Clear_BLERX_BUFF(void)
{
        BLERX_LEN = 0;
        BLERX_FLAG = 0;
}

/******************************************************************
 * 函 数 名 称：Bluetooth_Init
 * 函 数 说 明：蓝牙初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：默认波特率为9600
******************************************************************/
void Bluetooth_Init(void)
{
        //清除串口中断标志
        NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
        //使能串口中断
        NVIC_EnableIRQ(UART_1_INST_INT_IRQN);                    
        
    #if DEBUG
         //在调试时，通过AT命令已经设置好模式
        printf("Bluetooth_Init succeed!\r\n");

    #endif
}

/******************************************************************
 * 函 数 名 称：Get_Bluetooth_ConnectFlag
 * 函 数 说 明：获取手机连接状态 
 * 函 数 形 参：无
 * 函 数 返 回：返回1=已连接                返回0=未连接
 * 作       者：LC
 * 备       注：使用该函数前，必须先调用 Bluetooth_Mode 函数
******************************************************************/
unsigned char Get_Bluetooth_ConnectFlag(void)
{
        return Bluetooth_ConnectFlag;
}

/******************************************************************
 * 函 数 名 称：Bluetooth_Mode
 * 函 数 说 明：判断蓝牙模块的连接状态
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：未连接时STATE低电平   连接成功时STATE高电平
******************************************************************/
void Bluetooth_Mode(void)
{
        static char flag = 0;
        //如果没有手机连接
        if( DISCONNECT == BLUETOOTH_LINK ) 
        {            
                //连接状态为未连接
                Bluetooth_ConnectFlag = 0;
                //如果之前是连接状态
                if( flag == 1 )     
                {
                        flag = 0;//修改状态
                }
                return;
        }
        //如果手机已经连接
        if( CONNECT == BLUETOOTH_LINK )
        {
                Bluetooth_ConnectFlag = 1;
                                                //如果之前是断开状态
                if( flag == 0 )
                {
                        flag = 1;//修改状态
                }
        }
}

/******************************************************************
 * 函 数 名 称：Receive_Bluetooth_Data
 * 函 数 说 明：接收蓝牙数据
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void Receive_Bluetooth_Data(void)
{
        if( BLERX_FLAG == 1 )//接收到蓝牙数据
        {
                //显示蓝牙发送过来的数据
                printf("data = %s\r\n",BLERX_BUFF);

                Clear_BLERX_BUFF();//清除接收缓存
        }

}

/******************************************************************
 * 函 数 名 称：Send_Bluetooth_Data
 * 函 数 说 明：向蓝牙模块发送数据
 * 函 数 形 参：dat=要发送的字符串
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：（如果手机连接了蓝牙，就是向手机发送数据）
******************************************************************/
void Send_Bluetooth_Data(char *dat)
{
		//获取蓝牙状态
		Bluetooth_Mode();
		//如果手机已经连接
		if( Bluetooth_ConnectFlag == 1 )
		{
				//发送数据
				BLE_send_String((unsigned char*)dat);
		}
}



