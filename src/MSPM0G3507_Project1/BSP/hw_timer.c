#include "hw_timer.h"
#include "hw_encoder.h"
//#include "mid_button.h"


//����������������
void TIMER_TICK_INST_IRQHandler(void)
{
	//20ms�����жϴ���
	if( DL_TimerA_getPendingInterrupt(TIMER_TICK_INST) == DL_TIMER_IIDX_ZERO )
	{
		//����������
		encoder_update();

		//����ɨ��+�¼�����
		//flex_button_scan();

	}
}