#include "ararm.h"
#include "board.h"
/**
  * @brief  简单介绍：声光报警一秒钟
  * @param  无参数
  * @retval 无返回值
  */
void Alarm_Start(void)
{
	DL_GPIO_clearPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_setPins(LED_PORT,LED_LED1_PIN);  //输出高电平
	delay_ms(500);//声光报警响一次停止
	DL_GPIO_setPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_clearPins(LED_PORT,LED_LED1_PIN);//输出低电平
}
void Alarm_Start1(void)
{
	DL_GPIO_clearPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_setPins(LED_PORT,LED_LED1_PIN);  //输出高电平
}
void Alarm_Start0(void)
{
	DL_GPIO_setPins(Buzzer_PORT ,Buzzer_Buzzer1_PIN);
	DL_GPIO_clearPins(LED_PORT,LED_LED1_PIN);//输出低电平
}