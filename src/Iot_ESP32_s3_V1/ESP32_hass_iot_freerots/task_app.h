#pragma once
#include <Arduino.h> 
#include <stdint.h>
#include "sensor_driver.h" // 传感器驱动
#include "config.h"


void MQTT_event_app();
void iotServiceTask(void *pvParameters);
void dataUploadTask(void *pvParameters);
void batteryTask(void *pvParameters);
void gpsTask(void *pvParameters);
void speedAndPIDTask(void *pvParameters);
void distanceTask(void *pvParameters);
void maxSpeedTestTask(void *pvParams);
void compassTask(void *pvParameters);
void visionTask(void *pvParameters);
void laserWarningTask(void *pvParameters);