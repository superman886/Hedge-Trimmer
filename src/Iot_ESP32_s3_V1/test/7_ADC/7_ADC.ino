
/**
 ****************************************************************************************************
 *          果云科技 ESP32S3 扩展版 
 *           7- ADC采集 实验
 *           2024-8-29
 *          扩展板 电位器接的引脚是GPIO1 默认是断开的 需要把P7跳帽插上，
            不做这个实验的时候 把P7的跳帽拿掉 解放GPIO1引脚
 ****************************************************************************************************
 */
#include<Arduino.h>
#define ADC_Pin 1
uint16_t adc_value;
float adc_fvalue;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}
 
void loop() {
  // put your main code here, to run repeatedly:
  adc_value=analogRead(ADC_Pin);//获取原始ADC值 12位精度 范围 0-4096
  Serial.println(adc_value);
  adc_fvalue=adc_value/4096.0*3.3;//转换成电压值
  Serial.printf("ADC:%fV\n",adc_fvalue);
  delay(1000);

}
