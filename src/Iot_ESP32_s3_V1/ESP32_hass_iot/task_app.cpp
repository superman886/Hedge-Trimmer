#include "task_app.h"
#include "MQTT_driver.h"  //  MQTT驱动
#include "sensor_driver.h" // 传感器驱动


/* 连接状态检查任务程序 */
void State_check_app(){
    connect_check();  //  连接状态检查
    // 定期发送消息
    if (mqttClient.connected())
    {
            previousPublishMillis = currentMillis;
            mqttClient.publish("esp32/state", "online");
            //mqttClient.publish("esp32/normalization/state", "online");
    }
    if(DEBUG_MODE)Serial.println("[DEBUG]连接检查正常!");
}

/* MQTT事务任务程序 *///？？？？
void MQTT_event_app(){
    mqttClient.loop(); // 处理MQTT事务
}

/* Iot数据上传任务程序 */
void Iot_data_upload_app(){
    if(enable_Iot_data_upload == false) return;   //数据是否在上传，不是就退出，是就执行函数
  // WIFI状态 RSSI值
    String rssi_string = "{\"RSSI\":" + String(rssi) + "}";
    mqttClient.publish("esp32/system/wifi",rssi_string.c_str());
    /*电量采集数据上传*/
    String sensor1_battery_string = "{\"battery\":" + String(batteryPercentage) + "}";
    if(DEBUG_MODE) Serial.printf("[DEBUG]MQTT发送:%s\n",sensor1_battery_string.c_str());
    mqttClient.publish("esp32/sensor/battery",sensor1_battery_string.c_str());

    // 假设 gps_accuracy 是一个浮点数或整数变量
    float gps_accuracy = 1.5; // 示例值，根据实际情况替换

    // 构建 JSON 字符串
    String sensor1_gps_string = "{\"latitude\":" + String(Save_Data.latitude) + 
                                ",\"longitude\":" + String(Save_Data.longitude) + 
                                ",\"gps_accuracy\":" + String(gps_accuracy) + "}";
    // 调试模式下打印 JSON 字符串
    if (DEBUG_MODE) {
      Serial.printf("[DEBUG] MQTT发送: %s\n", sensor1_gps_string.c_str());}
    // 发布 MQTT 消息
    mqttClient.publish("esp32/sensor/gps", sensor1_gps_string.c_str());

}

/* 串口处理程序  单独一个任务*/
void Serial1_analysis_app(){
    gps_senor_read();
  }
  
  /* 传感器数据采集程序 */
  void Sensor_rw_app(){
    battery_senor_read(); // 读取土壤湿度
    //actuator_write(); // 控制执行器
  }