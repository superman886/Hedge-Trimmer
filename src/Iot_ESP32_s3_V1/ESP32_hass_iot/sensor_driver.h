#ifndef SENSOR_DRIVER_H  
#define SENSOR_DRIVER_H

#include <Arduino.h> 
#include <stdint.h>

extern const bool DEBUG_MODE;
//gps结构体类型
typedef struct {
    char GPS_Buffer[80];
    bool isGetData;     // 是否获取到GPS数据
    bool isParseData;   // 是否解析完成
    char UTCTime[11];   // UTC时间
    char latitude[11];  // 纬度
    char N_S[2];        // N/S
    char longitude[12]; // 经度
    char E_W[2];        // E/W
    bool isUsefull;     // 定位信息是否有效
}GpsData;


/* I/O引脚配置 */
// ADC 引脚配置
extern const int adcPin ;    // 测电压引脚

//电量采集参数
extern float batteryPercentage ;//电量
// 定义电池电压范围（根据实际电池类型调整）
extern const float VOLTAGE_DIVIDER_RATIO ;       // 定义电压传感器的分压比例5:1
extern const float minVoltage ;                   // 0% 空电电压（例如4节锂电池串联的空电电压）12v 3v
extern const float maxVoltage ;                 // 100% 满电电压（例如4节锂电池串联的满电电压）16.8v 4.2v

//extern const unsigned int gpsRxBufferLength ;   // GPS串口接收缓存长度
extern char gpsRxBuffer[600];          // GPS串口接收缓存
extern unsigned int ii ;  // 串口接收缓存
extern GpsData Save_Data ;

/* 系统运行数据 */
extern long rssi;

/* 系统标志位 */
extern bool enable_Iot_data_upload;


extern TinyGPSPlus gps;// GPS对象

void battery_senor_read();
void gps_senor_read();
#endif