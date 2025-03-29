/*目前完成了  （多线程版本）停更...........（转战freertos）
物联网：WiFi连接，MQTT连接，MQTT服务器数据接收到ESP32，ESP32数据发送到Home Assistant
传感器：电压采集 GPS解析 
执行器：底盘驱动？ 
日期：2025年3月25日
*/
#include <TinyGPS++.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TaskScheduler.h>  // 多线程调度器
#include "MQTT_driver.h"  //  MQTT驱动
#include "task_app.h" //  协程回调函数
#include "sensor_driver.h" // 传感器驱动

#define SERIAL_DEBUG_BPS  115200      //串口0 DEBUG波特率
#define SERIAL_BPS  230400            // 串口0波特率
//#define Gps 9600                // GPS串口波特率
#define RX_PIN 16  // 接收引脚
#define TX_PIN 17  // 发送引脚
HardwareSerial GpsSerial(2); // 使用UART2


#define IOT_DATA_UPLOAD_DELAY 3000  // IOT数据上传周期(ms)
#define LINK_STATE_CHECK_DELAY 10000  // 设备在线状态检查周期(ms)
#define SENSOR_READ_DELAY 3000  //  传感器数据采集周期(ms)

#define freq 50000 		 // PWM波形频率5KHZ
#define resolution  8		// 使用PWM占空比的分辨率，占空比最大可写2^8-1=255

const bool DEBUG_MODE = true;  //调试模式

/* I/O引脚配置 */
/*// TB6612FNG驱动模块控制信号 共6个
const int IN1 = 14 ;        // 控制电机1的方向A，01为正转，10为反转 
const int IN2 = 12 ;       // 控制电机1的方向B，01为正转，10为反转 
const int IN3 = 27 ;      // 控制电机2的方向A，01为正转，10为反转 
const int IN4 = 26 ;       // 控制电机2的方向B，01为正转，10为反转 
const int PWMA = 25	;      // 控制电机1 PWM控制引脚
const int PWMB = 33 ;      // 控制电机2 PWM控制引脚
//霍尔编码器引脚
const int R1 = 4 ;         // 右轮编码器引脚1  
const int R2 = 5 ;         // 右轮编码器引脚2
const int L1 = 18 ;        // 左轮编码器引脚1
const int L2 = 19 ;        // 左轮编码器引脚2
*/
// ADC 引脚配置
const int adcPin = 34 ;    // 测电压引脚

/***********传感器数据   系统数据  ****************/
/*//速度pid 参数
volatile long Rcounter1 = 0, Rcounter2 = 0; // 右轮脉冲计数
volatile long Lcounter1 = 0, Lcounter2 = 0; // 左轮脉冲计数
float Kp = 10;  // 比例系数
float Ki = 0.6; // 积分系数
float Kd = 0.02;  // 微分系数
float targetSpeed = 1; // 目标速度 (rad/s)
float lastError = 0.0;
float integral = 0.0;*/

//电量采集参数
float batteryPercentage = 100;//电量
// 定义电池电压范围（根据实际电池类型调整）
const float VOLTAGE_DIVIDER_RATIO = 5.0 ;       // 定义电压传感器的分压比例5:1
const float minVoltage = 6;                   // 0% 空电电压（例如4节锂电池串联的空电电压）12v 3v
const float maxVoltage = 8.4;                 // 100% 满电电压（例如4节锂电池串联的满电电压）16.8v 4.2v

// 定义GPS数据区
//const unsigned int gpsRxBufferLength = 600;   // GPS串口接收缓存长度
char gpsRxBuffer[600] ;          // GPS串口接收缓存
unsigned int ii = 0;  // 串口接收缓存

//霍尔编码器定时
/*
Ticker timer1;  // 中断函数
int interrupt_time = 10; // 中断时间
int timer_flag = 0;       // 定时器标志
unsigned long startTime = 0;
*/

/* 系统事件时间记录 */
unsigned long previousConnectMillis = 0; // 毫秒时间记录
const long intervalConnectMillis = 5000; // 时间间隔
unsigned long previousPublishMillis = 0; // 毫秒时间记录
const long intervalPublishMillis = 5000; // 时间间隔
unsigned long currentMillis = 0;  // 当前时间记录

/* 系统标志位 */ 
bool enable_Iot_data_upload = true;

/* 系统运行数据 */
long rssi = 0;

/* WiFi相关配置信息 */
const char *wifi_ssid = "FFF";
const char *wifi_password = "87654321";

/* MQTT相关配置信息 */
const char *mqtt_broker_addr = "47.122.154.80"; // MQTT服务器地址
const uint16_t mqtt_broker_port = 1883; // MQTT服务器端口            
const char *mqtt_username = "text"; // MQTT账号
const char *mqtt_password = "666666"; // MQTT密码
const uint16_t mqtt_client_buff_size = 4096; // 客户端缓存大小（非必须）
String mqtt_client_id = "esp32_client"; // 客户端ID
const char *mqtt_willTopic = "esp32/state"; // MQTT连接遗嘱主题




TinyGPSPlus gps;// 定义GPS对象
WiFiClient tcpClient; // 声明TCP连接
PubSubClient mqttClient;// 声明MQTT连接
Scheduler ts; // 声明协程管理器

/* 创建多协程任务信息 */
Task Link_state_check_app_task(LINK_STATE_CHECK_DELAY,TASK_FOREVER,&State_check_app);  // 创建任务 连接状态检查任务 任务次数：始终
Task MQTT_event_app_task(TASK_IMMEDIATE,TASK_FOREVER,&MQTT_event_app);  // 创建任务 MQTT事务任务 任务次数：始终
Task Iot_data_upload_app_task(IOT_DATA_UPLOAD_DELAY,TASK_FOREVER,&Iot_data_upload_app);  // 创建任务 连接状态检查任务 任务次数：始终
Task Serial1_analysis_app_task(IOT_DATA_UPLOAD_DELAY,TASK_FOREVER,&Serial1_analysis_app);  // 创建任务 串口处理任务 任务次数：始终
Task Sensor_rw_app_task(TASK_IMMEDIATE,TASK_FOREVER,&Sensor_rw_app);  // 创建任务 传感器数据采集任务 任务次数：始终




void setup() {
    Serial_init();
    IO_init();
    setup_iot_server();


    //disableCore0WDT(); // 关闭定时器看门狗
    //协程初始化
    ts.init();//初始化 scheduler
    ts.addTask(Link_state_check_app_task);//将 Link_state_check_app_task 装载到任务管理器
    ts.addTask(MQTT_event_app_task);//将 MQTT_event_app_task 装载到任务管理器
    ts.addTask(Iot_data_upload_app_task);//将 Iot_data_upload_app_task 装载到任务管理器
    ts.addTask(Serial1_analysis_app_task);//将 Serial1_app_task 装载到任务管理器
    ts.addTask(Sensor_rw_app_task);//将 Sensor_read_app_task 装载到任务管理器

    //启动任务
    Link_state_check_app_task.enable(); //启动 Link_state_check_app_task 任务
    MQTT_event_app_task.enable(); //启动 MQTT_event_app_task 任务
    Iot_data_upload_app_task.enable(); //启动 Iot_data_upload_app_task 任务
    Serial1_analysis_app_task.enable(); //启动 Serial1_app_task 任务
    Sensor_rw_app_task.enable();  //启动 Sensor_read_app_task 任务

    
}

void loop() {
  // put your main code here, to run repeatedly:
    //digitalToggle(stateLedPin); // 20240720:此操作无实际作用，用于防止看门狗饿死，未来会移除
    currentMillis = millis(); // 读取当前时间
    ts.execute(); //多任务管理器保活（延迟特殊函数）
    delay(1); // 防止任务看门狗饿死   一直重启
}

/* 串口初始化 */
void Serial_init(){
    if(DEBUG_MODE) Serial.begin(SERIAL_DEBUG_BPS); // DEBUG模式
    else Serial.begin(SERIAL_BPS); //TX: RX:
      // 初始化与gps的通信串口
    GpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  }

  void IO_init(){
    //TB6612FNG引脚初始化（电机）
  /*  pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    //pinMode(STBY, OUTPUT);
    //霍尔编码器引脚初始化
    pinMode(L1, INPUT_PULLUP);
    pinMode(L2, INPUT_PULLUP);
    pinMode(R1, INPUT_PULLUP);
    pinMode(R2, INPUT_PULLUP);
   // attachInterrupt(digitalPinToInterrupt(ENCA), onEncoderAChange, CHANGE);  //onEncoderAChange 单线程
  //  attachInterrupt(digitalPinToInterrupt(ENCB), onEncoderBChange, CHANGE);*/
    //ADC 引脚初始化
    analogReadResolution(12); // 设置 ADC 分辨率为 12 位 (0-4095)
  }


