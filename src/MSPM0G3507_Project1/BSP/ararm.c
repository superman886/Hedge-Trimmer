#include "ararm.h"
#include "board.h"
/**
  * @brief  �򵥽��ܣ����ⱨ��һ����
  * @param  �޲���
  * @retval �޷���ֵ
  */
void Alarm_Start(void)
{
	DL_GPIO_clearPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_setPins(LED_PORT,LED_LED1_PIN);  //����ߵ�ƽ
	delay_ms(500);//���ⱨ����һ��ֹͣ
	DL_GPIO_setPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_clearPins(LED_PORT,LED_LED1_PIN);//����͵�ƽ
}
void Alarm_Start1(void)
{
	DL_GPIO_clearPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_setPins(LED_PORT,LED_LED1_PIN);  //����ߵ�ƽ
}
void Alarm_Start0(void)
{
	DL_GPIO_setPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_clearPins(LED_PORT,LED_LED1_PIN);//����͵�ƽ
}