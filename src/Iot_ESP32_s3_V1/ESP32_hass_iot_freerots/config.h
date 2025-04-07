
#pragma once
#include <TinyGPS++.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_VL53L0X.h"


#define SERIAL_DEBUG_BPS  115200      //串口0 DEBUG波特率
#define SERIAL_BPS  230400            // 串口0波特率
#define GPSSERIAL 9600                // GPS串口波特率
#define VISIONSERIAL 19200            // openmv串口波特率

/**************** 各个任务的执行周期 *****************/  //待修改
#define IOT_DATA_UPLOAD_DELAY 3000  // IOT数据上传周期(ms)
#define LINK_STATE_CHECK_DELAY 10000  // 设备在线状态检查周期(ms)
#define SENSOR_READ_DELAY 3000  //  传感器数据采集周期(ms)


/******************* FreeRTOS配置 ************************/   //待修改
#define TASK_STACK_SIZE 4096      // 默认堆栈大小
#define CORE_0 0
#define CORE_1 1

/***********引脚配置*********************/
// ADC 引脚配置
#define adcPin  34     // 测电压引脚
// 激光测距引脚
#define VL53L0X_SCL_PIN 14     //激光测距传感器SCL引脚
#define VL53L0X_SDA_PIN 13     //激光测距传感器SDA引脚
// 定义两个传感器的XSHUT引脚和I2C地址
#define LOX1_XSHUT_PIN 5   // 第一个传感器的XSHUT引脚
#define LOX2_XSHUT_PIN 6   // 第二个传感器的XSHUT引脚
#define LOX1_ADDRESS 0x30  // 第一个传感器的I2C地址
#define LOX2_ADDRESS 0x31  // 第二个传感器的I2C地址
// 电子罗盘引脚
#define COMPASS_SCL_PIN 38     //电子罗盘传感器SCL引脚
#define COMPASS_SDA_PIN 37     //电子罗盘传感器SDA引脚
/*
// TB6612引脚定义
#define PWMA 4
#define AIN1 6
#define AIN2 7
#define PWMB 5
#define BIN1 15
#define BIN2 16
//霍尔编码器引脚定义
#define ENCODER_LEFT_A 10
#define ENCODER_LEFT_B 11
#define ENCODER_RIGHT_A 12
#define ENCODER_RIGHT_B 13
*/
/**************电子罗盘配置****************/
#define HMC5883_WriteAddress 0x1E          // HMC5883 的 I2C 地址（写地址）
#define HMC5883_ModeRegisterAddress 0x02   // 模式寄存器地址，用于设置传感器的工作模式
#define HMC5883_ContinuousModeCommand 0x00 // MD0,MD1为0时是连续测量模式
#define HMC5883_DataOutputXMSBAddress 0x03 // 数据输出 X MSB寄存器起始地址，用于读取 X、Y、Z 轴的数据。
#define xMax 65535                         //x轴最大偏移量
#define xMin 0                             //x轴最小偏移量
#define yMax 65535
#define yMin 0

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

extern GpsData Save_Data ;

// PID结构体
typedef struct {
  float Kp = 1.2;
  float Ki = 0.5;
  float Kd = 0.1;
  float integral = 0;
  float lastError = 0;
} PIDController;

extern PIDController  pidLeft, pidRight;

extern TwoWire I2C_2; // 声明外部定义的 I2C_2
extern const bool DEBUG_MODE;
/***********  传感器数据   系统数据  ****************/
//电量采集参数
extern float batteryPercentage ;                //采集的电量%

// 定义电池电压范围（根据实际电池类型调整）
const float VOLTAGE_DIVIDER_RATIO = 5.0 ;       // 定义电压传感器的分压比例5:1
const float minVoltage = 6;                     // 0% 空电电压（例如4节锂电池串联的空电电压）12v 3v
const float maxVoltage = 8.4;                   // 100% 满电电压（例如4节锂电池串联的满电电压）16.8v 4.2v

//霍尔编码器
extern volatile int32_t leftEncoderCount ;      //volatile告诉编译器它们可能会被中断服务程序（ISR）修改
extern volatile int32_t rightEncoderCount ;

//gps
extern char gpsRxBuffer[600];                   // GPS串口接收缓存
extern unsigned int ii ;                        // 串口接收缓存

extern const int Obstacle_distance;
extern volatile bool obstacleDetected;  // 全局标志位，表示是否接近障碍物
extern volatile bool treelawnDetected;  // 全局标志位，是否接近绿化带
/***************  物联网  *******************/
/* WiFi相关配置信息 */
extern const char *wifi_ssid; //WiFi名
extern const char *wifi_password ; //WiFi密码
/* MQTT相关配置信息 */
extern const char *mqtt_broker_addr ; // MQTT服务器地址
extern const uint16_t mqtt_broker_port ; // MQTT服务器端口            
extern const char *mqtt_username ; // MQTT账号
extern const char *mqtt_password ; // MQTT密码
extern const uint16_t mqtt_client_buff_size ; // 客户端缓存大小（非必须）
extern String mqtt_client_id ; // 客户端ID
extern const char *mqtt_willTopic ; // MQTT连接遗嘱主题
extern const char *mqtt_topic_pub ; // 需要发布到的主题
extern const char *mqtt_topic_sub ; // 需要订阅的主题

/* 电子罗盘相关配置信息 */
extern const int xOffset;
extern const int yOffset;  //用于校准传感器的原始数据
//extern int outputData[6];

/* 系统标志位 */ 
extern bool enable_Iot_data_upload ;

/* 系统运行数据 */
extern long rssi ;
/**** 定义对象   *****/
extern TinyGPSPlus gps;// 定义GPS对象
extern WiFiClient tcpClient;
extern PubSubClient mqttClient;
//extern VL53L0X sensor1, sensor2, sensor3, sensor4;  //四个 VL53L0X 类型的对象
extern Adafruit_VL53L0X lox1;
extern Adafruit_VL53L0X lox2;
extern VL53L0X_RangingMeasurementData_t measure1, measure2;
//队列
//extern QueueHandle_t distanceQueue; //distanceQueue 存储的是一个包含 4 个传感器距离值的数组（uint16_t distances[4]）
//extern QueueHandle_t encoderQueue;  //encoderQueue 存储的是一个包含两个编码器计数值的数组（int32_t counts[2]）。（可以用互斥锁）

/* 互斥锁声明 */
extern SemaphoreHandle_t voltageMutex ;// 保护 batteryPercentage
extern SemaphoreHandle_t gpsMutex ;// 保护 Save_Data
extern SemaphoreHandle_t mqttMutex ;
