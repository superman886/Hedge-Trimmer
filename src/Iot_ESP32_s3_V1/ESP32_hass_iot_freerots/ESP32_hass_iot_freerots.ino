/*FREERTOS版本  
FREERTOS (任务句柄、互斥锁、队列)
物联网：WiFi连接，MQTT连接，MQTT服务器数据接收到ESP32，ESP32数据发送到Home Assistant
传感器：电压采集 GPS解析 霍尔编码器 激光测距
执行器：电机驱动（小方正在努力移植中.........） 
日期：2025年3月30日
*/
#include <TinyGPS++.h>
//#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "MQTT_driver.h"  //  MQTT驱动
#include "task_app.h" //  协程回调函数
#include "sensor_driver.h" // 传感器驱动
#include "config.h"
#include "Adafruit_VL53L0X.h"
#include "Wire.h"



const bool DEBUG_MODE = true;  //调试模式

//障碍物检测警告距离
const int Obstacle_distance = 500;     //500mm
//电量采集参数
float batteryPercentage = 0.0;//电量
//霍尔编码器
volatile int32_t leftEncoderCount = 0;  //volatile告诉编译器它们可能会被中断服务程序（ISR）修改
volatile int32_t rightEncoderCount = 0;
//激光测距障碍物检测
volatile bool obstacleDetected = false;  // 全局标志位，是否接近障碍物
//绿化带检测
volatile bool treelawnDetected = false;  // 全局标志位，是否接近绿化带
// 定义GPS数据区
//const unsigned int gpsRxBufferLength = 600;   // GPS串口接收缓存长度
char gpsRxBuffer[600] ;          // GPS串口接收缓存
unsigned int ii = 0;  // 串口接收缓存
/* 系统标志位 */ 
bool enable_Iot_data_upload = true;

/* 系统运行数据 */
long rssi = 0;
/*************** WiFi相关配置信息 ******************/
const char *wifi_ssid = "FFF";
const char *wifi_password = "87654321";

/***************** MQTT相关配置信息 *****************/
const char *mqtt_broker_addr = "********"; // MQTT服务器地址
const uint16_t mqtt_broker_port = 1883; // MQTT服务器端口            
const char *mqtt_username = "****"; // MQTT账号
const char *mqtt_password = "****"; // MQTT密码
const uint16_t mqtt_client_buff_size = 4096; // 客户端缓存大小（非必须）
String mqtt_client_id = "esp32_client"; // 客户端ID
const char *mqtt_willTopic = "esp32/state"; // MQTT连接遗嘱主题
/***************** 电子罗盘配置信息 *****************/
const int regb = 0x01; // 配置寄存器 B 的地址
const int regbdata = 0x40; // 配置寄存器 B 的值（设置增益），0100 0000 对应的GN2(0),GN1(1),GN0(0)

// 计算偏移量
const int xOffset = (xMax + xMin) / 2;
const int yOffset = (yMax + yMin) / 2;



/*************定义对象*********************/
TinyGPSPlus gps;// 定义GPS对象
//WiFiClient tcpClient; // 声明TCP连接
PubSubClient mqttClient;// 声明MQTT连接

// 定义两个I2C对象
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);
// 创建两个激光测距传感器对象
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
// 创建测量对象实例
VL53L0X_RangingMeasurementData_t measure1, measure2;



/*************队列句柄***********************/
//定义了一个 FreeRTOS 队列句柄，用于在任务之间传递传感器的距离数据。
//QueueHandle_t distanceQueue; //distanceQueue 存储的是一个包含 4 个传感器距离值的数组（uint16_t distances[4]）
//QueueHandle_t encoderQueue;  //encoderQueue 存储的是一个包含两个编码器计数值的数组（int32_t counts[2]）。（可以用互斥锁）

/* 任务句柄声明 */
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t dataUploadTaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t distanceTaskHandle = NULL;
TaskHandle_t visionTaskHandle = NULL;
TaskHandle_t compassTaskHandle = NULL;

/* 互斥锁声明 */
SemaphoreHandle_t voltageMutex = NULL ;// 保护 batteryPercentage
SemaphoreHandle_t gpsMutex = NULL;// 保护 Save_Data
SemaphoreHandle_t mqttMutex = NULL; // 保护 mqtt

void setup() {
    Serial_init();
    IO_init();
    /* 初始化激光测距模块*/
    Wire.begin(VL53L0X_SDA_PIN, VL53L0X_SCL_PIN); // SDA, SCL
    initVL53L0X();

    // 创建互斥锁
    mqttMutex = xSemaphoreCreateMutex(); 
    voltageMutex = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();

    // 创建FreeRTOS资源
    //distanceQueue = xQueueCreate(1, sizeof(uint16_t[4]));
    //encoderQueue = xQueueCreate(1, sizeof(int32_t[2]));

    // 初始化IoT服务（在核心1运行）
    //xTaskCreatePinnedToCore(iotServiceTask, "IoT_Service", 2048, NULL, 5, NULL, CORE_1 );

    // 传感器任务（核心0）
    xTaskCreatePinnedToCore( sensorTask, "Sensor_Task", TASK_STACK_SIZE, NULL, 3, &sensorTaskHandle, CORE_0 );

    // 数据上传任务（核心1）
    xTaskCreatePinnedToCore( dataUploadTask, "Data_Upload", TASK_STACK_SIZE, NULL, 2, &dataUploadTaskHandle, CORE_1 );

    // GPS处理任务（核心0）
    xTaskCreatePinnedToCore( gpsTask, "GPS_Task", TASK_STACK_SIZE, NULL, 2, &gpsTaskHandle, CORE_0 );
    
    // 激光测距任务（核心0）
    xTaskCreatePinnedToCore( distanceTask, "DistanceTask", TASK_STACK_SIZE, NULL, 2, &distanceTaskHandle, CORE_0 );

    //发现绿化带任务(核心？)
    xTaskCreatePinnedToCore( visionTask, "VisionTask", TASK_STACK_SIZE, NULL, 3, &visionTaskHandle, CORE_1 );

    //电子罗盘检测方位角任务(核心？)
    xTaskCreatePinnedToCore( compassTask, "CompassTask", TASK_STACK_SIZE, NULL, 2, &compassTaskHandle, CORE_1 );

    /*
    //电机逻辑函数
    xTaskCreatePinnedToCore( motorControlTask, "MotorControl", 4096, NULL, 3, NULL, 1 );

    // 编码器任务优先级最高
    xTaskCreatePinnedToCore( encoderTask, "EncoderTask", 4096, NULL, 4, NULL, 1 );
    //（句柄还没写）
    */
}

void loop() {
    // 必须保留，即使为空
    vTaskDelay(portMAX_DELAY);
}

/* 串口初始化 */
void Serial_init(){
    if(DEBUG_MODE) Serial.begin(SERIAL_DEBUG_BPS); // DEBUG模式
    else Serial.begin(SERIAL_BPS); //TX: RX:
    Serial1.begin(GPSSERIAL); //TX: RX: GPS传感器占用onEncoderBChange
    Serial2.begin(VISIONSERIAL, SERIAL_8N1, 18, 17); // openmv占用，指定 RX=18, TX=17
  }

void IO_init(){
    //ADC初始化（电压）
    analogReadResolution(12); // 设置 ADC 分辨率为 12 位 (0-4095)

    // 初始化电子罗盘模块I2C端口
    I2C_2.begin(COMPASS_SDA_PIN, COMPASS_SCL_PIN); 
    I2C_2.beginTransmission(HMC5883_WriteAddress);
    I2C_2.write(regb);
    I2C_2.write(regbdata);
    I2C_2.endTransmission();
    /*
    // 扫描第一个I2C端口上的设备
    scanI2CDevices(I2C_1);
    // 扫描第二个I2C端口上的设备
    scanI2CDevices(I2C_2);
    */

    /*// 初始化电机引脚
    pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

    // 初始化编码器
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);*/
}

/*寻找iic设备地址（专用函数）
void scanI2CDevices(TwoWire &I2C) {
  Serial.println("Scanning for I2C devices...");

  for (byte address = 1; address < 127; address++) {
    I2C.beginTransmission(address);
    if (I2C.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
    }
  }
}
*/


// 初始化VL53L0X传感器
void initVL53L0X() {
  // 设置XSHUT引脚为输出
  pinMode(LOX1_XSHUT_PIN, OUTPUT);
  pinMode(LOX2_XSHUT_PIN, OUTPUT);

  // 关闭所有传感器
  digitalWrite(LOX1_XSHUT_PIN, LOW);
  digitalWrite(LOX2_XSHUT_PIN, LOW);
  delay(10);

  // 逐个激活并设置地址
  activateSensor(&lox1, LOX1_XSHUT_PIN, LOX1_ADDRESS);
  activateSensor(&lox2, LOX2_XSHUT_PIN, LOX2_ADDRESS);

  Serial.println("Both VL53L0X sensors initialized!");
}

// 激活传感器并设置地址
void activateSensor(Adafruit_VL53L0X* sensor, uint8_t xshut_pin, uint8_t newAddress) {
  // 激活当前传感器
  digitalWrite(xshut_pin, HIGH);
  delay(10);
  
  // 初始化传感器
  if (!sensor->begin(newAddress)) {
    Serial.print(F("Failed to boot VL53L0X at address 0x"));
    Serial.println(newAddress, HEX);
    while (1);
  }
  
  // 打印初始化成功信息
  Serial.print(F("VL53L0X initialized at address 0x"));
  Serial.println(newAddress, HEX);
}

