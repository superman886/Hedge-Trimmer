
#include "openmv.h"
#include "stdio.h"
#include "string.h"

#define USART_RECEIVE_LENGTH 200

char openmv4_recv_buff[USART_RECEIVE_LENGTH];    // ���ջ�����
volatile int openmv4_recv_length = 0;            // �������ݳ���
volatile char openmv4_recv_complete_flag = 0;    // ����������ɱ�־λ

/******************************************************************
 * �� �� �� �ƣ�OpenMV4_usart_config
 * �� �� ˵ ����
 * �� �� �� �Σ�
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע��LC
******************************************************************/
void OpenMV4_usart_config(void)
{
    //��������жϱ�־
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    //ʹ�ܴ����ж�
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

/******************************************************************
 * �� �� �� �ƣ�Openmv4DataAnalysis
 * �� �� ˵ ��������OpenMV4 ���͹������Զ����ʽ���� [%d,%d] ����
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void Openmv4DataAnalysis(void)
{
    char temp[20] = {0};
    char *buff = NULL;
    int head = 0;
    int end = 0;

    // û�н��յ����� ���� ����û�н������ �򲻽��д���
    if (openmv4_recv_complete_flag == 0) return;

    // ���жϣ���ֹ�ڽ��������н��յ�������
    __disable_irq();

    // ����0������յ�������(����)
    printf("%s\r\n", openmv4_recv_buff);

    // �ҵ���ʽ��ͷ�ĵ�һ�� '['
    while ( (openmv4_recv_buff[head] != '[') && (head < openmv4_recv_length) )
    {
        head++;
    }
    if (head == openmv4_recv_length)
    {
        printf("NO Find Head!!\r\n");

        // ���������ɱ�־λ���ȴ���һ�ν���
        openmv4_recv_complete_flag = 0;
        openmv4_recv_length = 0;
        // ���ж�
        __enable_irq();
        return;
    }
    buff = &openmv4_recv_buff[head];

    // �ҵ���β
    while ( (buff[end] != ']') && (end < openmv4_recv_length) )
    {
        end++;
    }
    if ((head + end) == openmv4_recv_length)
    {
        printf("NO Find end!!\r\n");

        // ���������ɱ�־λ���ȴ���һ�ν���
        openmv4_recv_complete_flag = 0;
        openmv4_recv_length = 0;
        // ���ж�
        __enable_irq();
        return;
    }

    // ��buff�︴�Ƴ���Ϊend���ַ�����temp
    if (end + 1 < sizeof(temp))
    {
        strncpy(temp, buff, end + 1);
        // strncpy�������Ჹ�㣬���ֶ�����
        temp[end + 1] = '\0';

        printf("buff = %s\r\n", temp);
    }
    else
    {
        printf("Data too large for temp buffer!!\r\n");
    }

    // ���������ɱ�־λ���ȴ���һ�ν���
    openmv4_recv_complete_flag = 0;
    openmv4_recv_length = 0;
    // �������
    memset(openmv4_recv_buff, 0, USART_RECEIVE_LENGTH);
    // ���ж�
    __enable_irq();
}

// ���ڵ��жϷ�����
void UART_2_INST_IRQHandler(void)
{
    uint8_t RecvDATA = 0;

    // ��������˴����ж�
    switch (DL_UART_getPendingInterrupt(UART_2_INST))
    {
        case DL_UART_IIDX_RX: // ����ǽ����ж�
            // ���շ��͹��������ݱ���
            RecvDATA = DL_UART_Main_receiveData(UART_2_INST);

            // ��黺�����Ƿ�����
            if (openmv4_recv_length < USART_RECEIVE_LENGTH - 1)
            {
                openmv4_recv_buff[openmv4_recv_length++] = RecvDATA;
                openmv4_recv_buff[openmv4_recv_length] = '\0';
            }

            // ��ǽ��ձ�־
            openmv4_recv_complete_flag = 1;

            break;

        default: // �����Ĵ����ж�
            break;
    }
}

void HardFault_Handler()
{
    printf("entry Error!!!!\r\n");
}
