
#include "openmv.h"
#include "stdio.h"
#include "string.h"

#define USART_RECEIVE_LENGTH 200

char openmv4_recv_buff[USART_RECEIVE_LENGTH];    // 接收缓冲区
volatile int openmv4_recv_length = 0;            // 接收数据长度
volatile char openmv4_recv_complete_flag = 0;    // 接收数据完成标志位

/******************************************************************
 * 函 数 名 称：OpenMV4_usart_config
 * 函 数 说 明：
 * 函 数 形 参：
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：LC
******************************************************************/
void OpenMV4_usart_config(void)
{
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

/******************************************************************
 * 函 数 名 称：Openmv4DataAnalysis
 * 函 数 说 明：解析OpenMV4 发送过来的自定义格式数据 [%d,%d] 解析
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void Openmv4DataAnalysis(void)
{
    char temp[20] = {0};
    char *buff = NULL;
    int head = 0;
    int end = 0;

    // 没有接收到数据 或者 数据没有接收完成 则不进行处理
    if (openmv4_recv_complete_flag == 0) return;

    // 关中断，防止在解析过程中接收到新数据
    __disable_irq();

    // 串口0输出接收到的数据(调试)
    printf("%s\r\n", openmv4_recv_buff);

    // 找到格式的头的第一个 '['
    while ( (openmv4_recv_buff[head] != '[') && (head < openmv4_recv_length) )
    {
        head++;
    }
    if (head == openmv4_recv_length)
    {
        printf("NO Find Head!!\r\n");

        // 清除接收完成标志位，等待下一次接收
        openmv4_recv_complete_flag = 0;
        openmv4_recv_length = 0;
        // 开中断
        __enable_irq();
        return;
    }
    buff = &openmv4_recv_buff[head];

    // 找到结尾
    while ( (buff[end] != ']') && (end < openmv4_recv_length) )
    {
        end++;
    }
    if ((head + end) == openmv4_recv_length)
    {
        printf("NO Find end!!\r\n");

        // 清除接收完成标志位，等待下一次接收
        openmv4_recv_complete_flag = 0;
        openmv4_recv_length = 0;
        // 开中断
        __enable_irq();
        return;
    }

    // 从buff里复制长度为end的字符串到temp
    if (end + 1 < sizeof(temp))
    {
        strncpy(temp, buff, end + 1);
        // strncpy函数不会补零，需手动补上
        temp[end + 1] = '\0';

        printf("buff = %s\r\n", temp);
    }
    else
    {
        printf("Data too large for temp buffer!!\r\n");
    }

    // 清除接收完成标志位，等待下一次接收
    openmv4_recv_complete_flag = 0;
    openmv4_recv_length = 0;
    // 清除数据
    memset(openmv4_recv_buff, 0, USART_RECEIVE_LENGTH);
    // 开中断
    __enable_irq();
}

// 串口的中断服务函数
void UART_2_INST_IRQHandler(void)
{
    uint8_t RecvDATA = 0;

    // 如果产生了串口中断
    switch (DL_UART_getPendingInterrupt(UART_2_INST))
    {
        case DL_UART_IIDX_RX: // 如果是接收中断
            // 接收发送过来的数据保存
            RecvDATA = DL_UART_Main_receiveData(UART_2_INST);

            // 检查缓冲区是否已满
            if (openmv4_recv_length < USART_RECEIVE_LENGTH - 1)
            {
                openmv4_recv_buff[openmv4_recv_length++] = RecvDATA;
                openmv4_recv_buff[openmv4_recv_length] = '\0';
            }

            // 标记接收标志
            openmv4_recv_complete_flag = 1;

            break;

        default: // 其他的串口中断
            break;
    }
}

void HardFault_Handler()
{
    printf("entry Error!!!!\r\n");
}
