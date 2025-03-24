#ifndef TASK_APP_H  
#define TASK_APP_H 

#include <Arduino.h> 
#include <stdint.h>
#include "sensor_driver.h" // 传感器驱动

extern const bool DEBUG_MODE;

/* I/O引脚配置 */
// ADC 引脚配置
extern const int adcPin ;    // 测电压引脚

//电量采集参数
extern float batteryPercentage ;//电量
// 定义电池电压范围（根据实际电池类型调整）
extern const float VOLTAGE_DIVIDER_RATIO ;       // 定义电压传感器的分压比例5:1
extern const float minVoltage ;                   // 0% 空电电压（例如4节锂电池串联的空电电压）12v 3v
extern const float maxVoltage ;                 // 100% 满电电压（例如4节锂电池串联的满电电压）16.8v 4.2v
//gps
extern char gpsRxBuffer[600];          // GPS串口接收缓存
extern unsigned int ii ;  // 串口接收缓存
extern GpsData Save_Data ;

/* 系统运行数据 */
extern long rssi;

/* 系统标志位 */
extern bool enable_Iot_data_upload;



void State_check_app();
void MQTT_event_app();
void Iot_data_upload_app();
void Serial1_analysis_app();
void Sensor_rw_app();


#endif //TASK_APP_H
