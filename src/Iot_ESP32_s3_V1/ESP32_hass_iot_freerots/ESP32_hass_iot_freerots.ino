/*FREERTOS版本  
FREERTOS (任务句柄、互斥锁)
物联网：WiFi连接，MQTT连接，MQTT服务器数据接收到ESP32，ESP32数据发送到Home Assistant
传感器：电压采集任务 GPS解析任务 霍尔编码器测速任务 激光测距  电子罗盘   
执行器：电机驱动
串口通讯：(OpenMV ) HT32（GPS解析任务?） 机械臂主控单片机(X)
日期：2025年4月12日 
*/
/*
#define configGENERATE_RUN_TIME_STATS    1  //启用启用FreeRTOS统计功能 用于获取所有任务的运行时间统计信息。
#define configUSE_TRACE_FACILITY    1         //启用 FreeRTOS 的跟踪工具支持。分析任务调度、资源竞争、死锁等问题。
#define configUSE_STATS_FORMATTING_FUNCTIONS 1  //实时监控任务状态和性能。
*/

#include <Arduino.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h" // 包含任务看门狗头文件
#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include "MQTT_driver.h" 
#include "task_app.h" 
#include "sensor_driver.h" 
#include "config.h"

/***********  常量定义  *************/
const bool DEBUG_MODE = true;  //调试模式false true

//电机和编码器
const float WHEEL_DIAMETER = 0.065;  // 轮径（米）
const int ENCODER_PPR = 260;         // 编码器每转脉冲数
const int PWM_FREQ = 20000;          // PWM频率（20kHz）
const int PWM_RESOLUTION = 8;        // PWM分辨率（8位）
const int TEST_DURATION = 2510;      // 测试持续时间（ms）

// 定义电池电压范围（根据实际电池类型调整）
const float VOLTAGE_DIVIDER_RATIO = 5.0 ;       // 定义电压传感器的分压比例5:1
const float minVoltage = 6;                     // 0% 空电电压（例如4节锂电池串联的空电电压）12v 3v
const float maxVoltage = 8.4;                   // 100% 满电电压（例如4节锂电池串联的满电电压）16.8v 4.2v

//障碍物检测警告距离（激光测距）
const int Obstacle_distance = 500;     //500mm

/***************** 电子罗盘配置信息 *****************/
const int regb = 0x01; // 配置寄存器 B 的地址
const int regbdata = 0x40; // 配置寄存器 B 的值（设置增益），0100 0000 对应的GN2(0),GN1(1),GN0(0)
// 计算偏移量（电子罗盘）
const int xOffset = (xMax + xMin) / 2;
const int yOffset = (yMax + yMin) / 2;

/*************** WiFi相关配置信息 ******************/
const char *wifi_ssid = "FFF";
const char *wifi_password = "87654321";

/***************** MQTT相关配置信息 *****************/
const char *mqtt_broker_addr = ""; // MQTT服务器地址
const uint16_t mqtt_broker_port = 1883; // MQTT服务器端口            
const char *mqtt_username = ""; // MQTT账号
const char *mqtt_password = ""; // MQTT密码
const uint16_t mqtt_client_buff_size = 4096; // 客户端缓存大小（非必须）
String mqtt_client_id = "esp32_client"; // 客户端ID
const char *mqtt_willTopic = "esp32/state"; // MQTT连接遗嘱主题

//系统标志位 
bool enable_Iot_data_upload = true;
//系统运行数据 
long rssi = 0;


/*****************  全局变量  **********************/
//电量采集参数
float batteryPercentage = 0.0;//电量

// 定义GPS数据区
char gpsRxBuffer[600] ;          // GPS串口接收缓存
unsigned int ii = 0;  // 串口接收缓存


//霍尔编码器
float leftOutputPID = 0.0 ;  //左轮pid
float rightOutputPID = 0.0 ; //右轮pid
//霍尔编码器
volatile int64_t  leftEncoderCount = 0;  //volatile告诉编译器它们可能会被中断服务程序（ISR）修改
volatile int64_t  rightEncoderCount = 0;
float maxSpeedLeft = 0;  // 新增：最大速度记录
float maxSpeedRight = 0;


//激光测距
volatile int frontDistance = 0;
volatile int leftDistance = 0;
//激光测距障碍物检测
volatile bool obstacleDetected = false;  // 全局标志位，是否接近障碍物
//绿化带检测
volatile bool treelawnDetected = false;  // 全局标志位，是否接近绿化带
//电子罗盘
float currentAngle = 0.0;          // 当前方位角（度）


//结构体
// 初始化任务看门狗（超时5秒，触发panic）
esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 5000,          // 超时时间（秒）
    .idle_core_mask = 0x3, // 监控所有核心的空闲任务（对于双核ESP32）
    .trigger_panic = true      // 超时时触发系统panic
};


//定义对象
Motor motorData = {0} ;
TinyGPSPlus gps;// 定义GPS对象
WiFiClient tcpClient; // 声明TCP连接
PubSubClient mqttClient;// 声明MQTT连接
// 定义两个I2C对象(电子罗盘)
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);


// 创建两个激光测距传感器对象
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// 创建测量对象实例
VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;


//任务句柄声明 
TaskHandle_t dataUploadTaskHandle = NULL;  
TaskHandle_t iotServiceTaskHandle = NULL;
TaskHandle_t batteryTaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t speedAndPIDTaskHandle = NULL;
TaskHandle_t distanceTaskHandle = NULL;
TaskHandle_t compassTaskHandle = NULL;
TaskHandle_t visionTaskHandle = NULL;

// 互斥锁声明 
SemaphoreHandle_t voltageMutex = NULL ;// 保护 batteryPercentage
SemaphoreHandle_t gpsMutex = NULL;// 保护 Save_Data
SemaphoreHandle_t mqttMutex = NULL; // 保护 mqtt
SemaphoreHandle_t encoderMutex = NULL;           // 编码器互斥锁
SemaphoreHandle_t compassMutex= NULL;    // 电子罗盘数据互斥锁
SemaphoreHandle_t dataMutex = NULL ;
SemaphoreHandle_t laserMutex = NULL ;

void setup() {
    Serial_init();
    IO_init();

    mqttMutex = xSemaphoreCreateMutex(); 
    voltageMutex = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();
    encoderMutex = xSemaphoreCreateMutex();
    compassMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    laserMutex = xSemaphoreCreateMutex();

    //测速（一次）
    //xTaskCreatePinnedToCore(maxSpeedTestTask, "SpeedTest", 4096, NULL, 3, NULL , CORE_0);

    //计算速度（更新速度0.01s）
    xTaskCreatePinnedToCore( speedAndPIDTask, "SpeedAndPIDTask", 4096, NULL, 5, &speedAndPIDTaskHandle, CORE_1 );

    // 激光测距任务（更新速度0.1s）
    xTaskCreatePinnedToCore(distanceTask, "DistanceTask", 4096, NULL, 4, &distanceTaskHandle, CORE_1 );

    //发现绿化带任务()
    xTaskCreatePinnedToCore( visionTask, "VisionTask", TASK_STACK_SIZE, NULL, 4, &visionTaskHandle, CORE_1 );

    //电子罗盘检测方位角任务(0.1s)
    xTaskCreatePinnedToCore( compassTask, "CompassTask", TASK_STACK_SIZE, NULL, 4, &compassTaskHandle, CORE_1 );

    //警报任务（向HT32发送）
    xTaskCreatePinnedToCore(laserWarningTask, "LaserWarning", 2048, NULL, 4, NULL, CORE_1);

    // GPS处理任务（3s更新）
    //xTaskCreatePinnedToCore( gpsTask, "GPS_Task", 4096, NULL, 3, &gpsTaskHandle, CORE_1 );

    // 电压传感器任务（3s更新）
    xTaskCreatePinnedToCore( batteryTask, "Battery_Task", 4096, NULL, 3, &batteryTaskHandle, CORE_1 );

    // 初始化物联网服务（10s检查网络状态）
    xTaskCreatePinnedToCore(iotServiceTask, "IoT_Service", 4096, NULL, 2, &iotServiceTaskHandle, CORE_0 );

    // 数据上传任务（5s上传一次数据）
    xTaskCreatePinnedToCore( dataUploadTask, "Data_Upload", 4096, NULL, 2, &dataUploadTaskHandle, CORE_0 );
    // 创建监控任务（）                          // 堆栈大小（监控任务需较大空间）
    //xTaskCreatePinnedToCore( monitorTask,  "MonitorTask",  4096, NULL, 1, NULL, CORE_0  );
    
    // 启动调度器（不用写了，arduino默认启动，其他编译环境需要ESP-IDE）
    /*vTaskStartScheduler();*/

}

void loop() {
    // 必须保留，即使为空
    vTaskDelay(portMAX_DELAY);
}

/* 串口初始化 */
void Serial_init(){
    if(DEBUG_MODE) Serial.begin(SERIAL_DEBUG_BPS); // DEBUG模式
    else Serial.begin(115200); //TX:43 RX:44            HT32
    Serial1.begin(GPSSERIAL); //TX:17 RX:18             GPS传感器
    Serial2.begin(VISIONSERIAL);; //TX:20 RX:19          Openmv
  }

/*引脚初始化*/
void IO_init(){

    // 初始化 看门狗
    esp_err_t ret = esp_task_wdt_init(&wdt_config);
    
    if (ret == ESP_OK) {
        printf("TWDT 初始化成功。\n");
    } else if (ret == ESP_ERR_INVALID_STATE) {
        printf("TWDT 已经初始化。\n");
    } else {
        printf("TWDT 初始化失败，错误代码: %d\n", ret);
    }

    //电压
    analogReadResolution(12); // 设置 ADC 分辨率为 12 位 (0-4095)

    //两路TB6612
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT); 
    pinMode(IN4, OUTPUT);

    // 配置PWM通道
    //ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
    //ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachChannel(ENA, PWM_FREQ, PWM_RESOLUTION , 0 );
    ledcAttachChannel(ENB, PWM_FREQ, PWM_RESOLUTION, 1 );

     // 初始化编码器
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);
    
    //初始化（SDA和SCL）总线
    if (!Wire.begin(VL53L0X_SDA_PIN, VL53L0X_SCL_PIN)) {
        if(DEBUG_MODE){Serial.println("Failed to initialize I2C.");}
    } 
    Wire.setClock(400000); // 提高 I2C 时钟速度

    /* 初始化激光测距模块*/
    // 设置XSHUT引脚为输出
    pinMode(LOX1_XSHUT_PIN, OUTPUT);
    pinMode(LOX2_XSHUT_PIN, OUTPUT);
    //pinMode(LOX3_XSHUT_PIN, OUTPUT);
    // 关闭所有传感器
    digitalWrite(LOX1_XSHUT_PIN, LOW);
    digitalWrite(LOX2_XSHUT_PIN, LOW);
    //digitalWrite(LOX3_XSHUT_PIN, LOW);
    delay(10);

    // 逐个激活并设置地址
    activateSensor(&lox1, LOX1_XSHUT_PIN, LOX1_ADDRESS);
    activateSensor(&lox2, LOX2_XSHUT_PIN, LOX2_ADDRESS);
    //activateSensor(&lox3, LOX3_XSHUT_PIN, LOX3_ADDRESS);
    if(DEBUG_MODE){
    Serial.println("Both VL53L0X sensors initialized!");}

    // 初始化电子罗盘
    Wire.beginTransmission(HMC5883_WriteAddress);
    if(Wire.endTransmission() != 0) {
        if(DEBUG_MODE){Serial.println("[Compass] HMC5883L not found!");}
        //vTaskDelete(NULL); // 删除任务
    }
    
    // 配置传感器（原初始化代码）
    Wire.beginTransmission(HMC5883_WriteAddress);
    Wire.write(0x00); // 配置寄存器A
    Wire.write(0x70); // 8平均采样，15Hz数据输出
    Wire.write(0x01); // 配置寄存器B
    Wire.write(0xA0); // 增益390LSb/Gauss
    Wire.write(0x02); // 模式寄存器
    Wire.write(0x00); // 连续测量模式
    Wire.endTransmission();

  }

// 激活传感器并设置地址

void activateSensor(Adafruit_VL53L0X* sensor, uint8_t xshut_pin, uint8_t newAddress) {
    // 激活当前传感器
    digitalWrite(xshut_pin, HIGH);
    delay(10);
    
    // 初始化传感器
    if (!sensor->begin(newAddress)) {
        if(DEBUG_MODE){Serial.print(F("Failed to boot VL53L0X at address 0x"));
        Serial.println(newAddress, HEX);}
    }
    
    // 打印初始化成功信息
    if(DEBUG_MODE){
    Serial.print(F("VL53L0X initialized at address 0x"));
    Serial.println(newAddress, HEX);
    }
}
