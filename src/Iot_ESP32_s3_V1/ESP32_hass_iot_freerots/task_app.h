#pragma once
#include <Arduino.h> 
#include <stdint.h>
#include "sensor_driver.h" // 传感器驱动
#include "config.h"


void MQTT_event_app();
// 删除原有任务函数声明，改为FreeRTOS任务函数
void iotServiceTask(void *pvParameters);
void sensorTask(void *pvParameters);
void dataUploadTask(void *pvParameters);
void distanceTask(void *pvParameters);
void gpsTask(void *pvParameters);
void sensorTask(void *pvParameters);
void visionTask(void *pvParameters);
void compassTask(void *pvParameters);
//void encoderTask(void *pvParameters);
//void motorControlTask(void *pvParameters);
