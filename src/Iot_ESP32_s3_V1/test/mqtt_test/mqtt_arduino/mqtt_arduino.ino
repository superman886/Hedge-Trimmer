#include <WiFi.h>
#include <PubSubClient.h>

#define SERIAL_DEBUG_BPS  115200      //串口0 DEBUG波特率
#define SERIAL_BPS  230400            // 串口0波特率
#define GPSBaud 9600

// ADC 引脚配置
const int adcPin = 34;                // 使用 ESP32 的 GPIO34 作为 ADC 输入

// Wi-Fi 网络配置
const char* wifi_ssid = "FFF";
const char* wifi_password = "87654321";

// MQTT 代理配置
const char *mqtt_broker_addr = "47.122.154.80";             // MQTT服务器地址
const uint16_t mqtt_broker_port = 1883;                     // MQTT服务器端口            
const char *mqtt_username = "esp32";                        // MQTT账号
const char *mqtt_password = "123456";                       // MQTT密码
const uint16_t mqtt_client_buff_size = 4096;                // 客户端缓存大小（非必须）
String mqtt_client_id = "esp32_client";                     // 客户端ID
const char *mqtt_willTopic = "esp32/state";                 // MQTT连接遗嘱主题

const bool DEBUG_MODE = true;//调试模式

/* 传感器及系统状态数据 */

float batteryPercentage=100;//电量
// 定义电池电压范围（根据实际电池类型调整）
const float VOLTAGE_DIVIDER_RATIO = 5.0 ;       // 定义电压传感器的分压比例5:1
const float minVoltage = 6;                   // 0% 空电电压（例如4节锂电池串联的空电电压）12v 3v
const float maxVoltage = 8.4;                 // 100% 满电电压（例如4节锂电池串联的满电电压）16.8v 4.2v
/* 系统运行数据 */
long rssi = 0;

// 定义GPS对象
TinyGPSPlus gps;

WiFiClient tcpClient;         // 声明TCP连接
PubSubClient mqttClient;      // 声明MQTT连接

/* 串口初始化代码 */
void Serial_init(){
  if(DEBUG_MODE) Serial.begin(SERIAL_DEBUG_BPS);      // DEBUG模式
  /*else Serial.begin(SERIAL_BPS); //TX:43 RX:44
  Serial1.begin(SERIAL1_BPS); //TX:16 RX:15 气体传感器占用
  Serial2.begin(SERIAL2_BPS); //TX:20 RX:19 RP2040占用*/
}

/* WIFI初始化函数 */
void wifi_setup(){
  if(DEBUG_MODE) Serial.printf("\nConnecting to %s", wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    if(DEBUG_MODE) Serial.print(".");
  }
  if(DEBUG_MODE){
    Serial.println("[DEBUG]WIFI连接正常.");
    Serial.print("[DEBUG]IP地址: ");
    Serial.println(WiFi.localIP());
  }
  hass_debug_log("[SYS]WIFI连接成功!");
}

/* MQTT初始化函数 */
void mqtt_setup(){
  // 设置MQTT客户端
  mqttClient.setClient(tcpClient);
  mqttClient.setServer(mqtt_broker_addr, mqtt_broker_port);             //地址和端口
  //mqttClient.setBufferSize(mqtt_client_buff_size);
  mqttClient.setCallback(mqtt_callback);
  mqtt_client_id += String(WiFi.macAddress());                          // 每个客户端需要有唯一的ID，不然上线时会把其他相同ID的客户端踢下线
  if(DEBUG_MODE)Serial.println("[DEBUG]MQTT正在连接...");
  if (mqttClient.connect(mqtt_client_id.c_str(), mqtt_username, mqtt_password, mqtt_willTopic, false, 2, "offline"))
  {
    mqttClient.publish("esp32/state", "online");                        // 连接成功后发布状态
    if(DEBUG_MODE)Serial.println("[DEBUG]MQTT连接成功!");
    hass_debug_log("[SYS]MQTT连接成功!");
    mqtt_subscribe_setup();
    if(DEBUG_MODE)Serial.println("[DEBUG]MQTT订阅列表添加成功!");
    hass_debug_log("[SYS]MQTT订阅列表添加成功!");
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup_iot_server(){
  if(DEBUG_MODE) Serial.printf("\n[DEBUG]WIFI Connecting to %s", wifi_ssid);
  wifi_setup();
  rssi = WiFi.RSSI(); // 获取网络连接质量
  mqtt_setup();
}

void mqtt_subscribe_setup(){


}

/* HASS debug日志传输函数 */
void hass_debug_log(char *log){
  mqttClient.publish("esp32/system/debug", log); 
}


/* 连接状态检查及重连函数 */
void connect_check(){
  if(WiFi.status() != WL_CONNECTED){      // 如果WIFI未连接
    wifi_setup();
  }
  rssi = WiFi.RSSI();                    // 获取网络连接质量
  if (!mqttClient.connected())           // 如果MQTT未连接
    {
      mqtt_client_id += String(WiFi.macAddress()); // 每个客户端需要有唯一的ID，不然上线时会把其他相同ID的客户端踢下线
      if(DEBUG_MODE)Serial.println("[DEBUG]MQTT正在连接...");
      if (mqttClient.connect(mqtt_client_id.c_str(), mqtt_username, mqtt_password, mqtt_willTopic, false, 2, "offline"))// 尝试连接MQTT服务器
      {
        mqttClient.publish("esp32/state", "online"); // 连接成功后发布状态
        if(DEBUG_MODE)Serial.println("[DEBUG]MQTT连接成功!");
        hass_debug_log("[SYS]MQTT连接成功!");
        mqtt_subscribe_setup();
        if(DEBUG_MODE)Serial.println("[DEBUG]MQTT订阅列表添加成功!");
        hass_debug_log("[SYS]MQTTMQTT订阅列表添加成功!");
      }
    }
}

void battery_senor_read(){
  /*float sum = 0;

  // 多次采样并累加
  for (int i = 0; i < numSamples; i++) {
    int adcValue = analogRead(adcPin);
    sum += adcValue;
    delay(1); // 短暂延迟以稳定采样
  }

    // 计算平均值
  float avgAdcValue = sum / numSamples;*/

 
  int adcValue = analogRead(adcPin); // 读取 ADC 值

  
  float voltageAtADC = (adcValue / 4095.0) * 3.3 + 0.16;  //// 计算电压手动加了0.16v

  // 根据分压比例计算实际电压
  float actualVoltage = voltageAtADC * VOLTAGE_DIVIDER_RATIO;

  // 限制电压在最小值和最大值之间
  if (actualVoltage < minVoltage) {
    actualVoltage = minVoltage;
  } else if (actualVoltage > maxVoltage) {
    actualVoltage = maxVoltage;
  }

  // 计算电量百分比
  float percentage = ((actualVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;

  batteryPercentage = percentage;
  // 输出电压到串口监视器
  Serial.print("ADC Value: ");
  Serial.println(adcValue);
    Serial.print("voltageAtADC: ");
  Serial.println(voltageAtADC);
  Serial.print("Battery Voltage: ");
  Serial.print(actualVoltage);
  Serial.println(" V");
  Serial.print("batteryPercentage: ");
  Serial.print(batteryPercentage);
  Serial.println(" %");
  // 等待 1 秒
  delay(1000);
}

void gps_senor_read(){
  
}





void Iot_data_upload_app(){
  String sensor1_battery_string = "{\"battery\":" + String(batteryPercentage) + "}";
  if(DEBUG_MODE) Serial.printf("[DEBUG]MQTT发送:%s\n",sensor1_battery_string.c_str());
  mqttClient.publish("esp32/sensor/battery",sensor1_battery_string.c_str());

  String sensor1_gps_string = "{\"latitude\":" + String(batteryPercentage) + 
  ",\"longitude\":" + String(batteryPercentage) + 
  ",\"gps_accuracy\":" + String(batteryPercentage) + "}";
  if(DEBUG_MODE) Serial.printf("[DEBUG]MQTT发送:%s\n",sensor1_battery_string.c_str());
  mqttClient.publish("esp32/sensor/gps",sensor1_battery_string.c_str());
}

void setup() {
  Serial_init();
  //setup_iot_server();  //wifi mqtt 物联网初始化
  analogReadResolution(12); // 设置 ADC 分辨率为 12 位 (0-4095)
}

void loop() {
  battery_senor_read();
  Iot_data_upload_app();
  delay(10); 
}



