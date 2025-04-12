#include "task_app.h"
#include "MQTT_driver.h"  //  MQTT驱动
#include "sensor_driver.h" // 传感器驱动

/******************** 编码器数据采集任务 ********************/
void speedAndPIDTask(void *pvParams) {
    const TickType_t interval = pdMS_TO_TICKS(10); // 10ms周期
    TickType_t lastWakeTime = xTaskGetTickCount();

    int64_t lastLeftCount = 0, lastRightCount = 0;
    uint32_t lastTime = micros();

    while(1) {
        // 原子读取左右编码器值
        noInterrupts();
        int64_t currentLeftCount = leftEncoderCount;
        int64_t currentRightCount = rightEncoderCount;
        interrupts();

        // 计算时间差（秒）
        uint32_t currTime = micros();
        float deltaTime = (currTime - lastTime) / 1e6f;
        if(deltaTime < 0.001) deltaTime = 0.001; // 最小1ms

        // 计算左右速度
        float leftSpeed = (currentLeftCount - lastLeftCount) * PI * WHEEL_DIAMETER 
                         / (ENCODER_PPR * deltaTime);
        float rightSpeed = (currentRightCount - lastRightCount) * PI * WHEEL_DIAMETER 
                          / (ENCODER_PPR * deltaTime);

        // 更新共享数据
        if (xSemaphoreTake(encoderMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            motorData.leftCount = currentLeftCount;
            motorData.rightCount = currentRightCount;
            motorData.leftSpeed = leftSpeed;
            motorData.rightSpeed = rightSpeed;

            // 更新最大速度（左右独立）
            float absLeft = fabs(leftSpeed);
            if(absLeft > maxSpeedLeft) maxSpeedLeft = absLeft;

            float absRight = fabs(rightSpeed);
            if(absRight > maxSpeedRight) maxSpeedRight = absRight;

            xSemaphoreGive(encoderMutex);
        }

        // 保存历史值
        lastLeftCount = currentLeftCount;
        lastRightCount = currentRightCount;
        lastTime = currTime;

        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

/******************** 最大速度测试任务 ********************/
void maxSpeedTestTask(void *pvParams) {
    Serial.println("\n=== 最大速度测试 ===");
    maxSpeedLeft = 0;
    maxSpeedRight = 0;
    uint32_t startTime = millis();

    setMotor(150, 150);  // 左轮，右轮50PWM

    while(millis() - startTime < TEST_DURATION) {
        Serial.printf("左轮最大:%.2f | 实际:%.2f | 编码器:%lld\n",
                     maxSpeedLeft, fabs(motorData.leftSpeed), motorData.leftCount);
        Serial.printf("右轮最大:%.2f | 实际:%.2f | 编码器:%lld\n",
                     maxSpeedRight, fabs(motorData.rightSpeed), motorData.rightCount);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    setMotor(0, 0);
    Serial.printf("左轮最终最大速度: %.2f m/s\n", maxSpeedLeft);
    Serial.printf("右轮最终最大速度: %.2f m/s\n", maxSpeedRight);
    vTaskDelete(NULL);
}

// GPS处理任务
void gpsTask(void *pvParameters) {
    const TickType_t interval = pdMS_TO_TICKS(3000); // 3s周期
    TickType_t lastWakeTime = xTaskGetTickCount();

    TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();// 获取当前任务的句柄
    esp_err_t ret = esp_task_wdt_add(currentTaskHandle); // 将当前任务添加到看门狗监控
    if (ret = ESP_OK && DEBUG_MODE) {
    Serial.println("[deug] 已将gpsTask添加到看门狗监控.");}
    for(;;) {
        gps_senor_read();// 写入gps 更新数据
        esp_task_wdt_reset(); // 喂狗
        vTaskDelayUntil(&lastWakeTime, interval); // 确保释放 CPU
    }
}


//  电压传感器采集任务（电压、gps）
void batteryTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(3000);// 3s周期
    TickType_t xLastWakeTime = xTaskGetTickCount();

    TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();// 获取当前任务的句柄
    esp_err_t ret = esp_task_wdt_add(currentTaskHandle); // 将当前任务添加到看门狗监控
    if (ret = ESP_OK && DEBUG_MODE) {
    Serial.println("[deug] 已将dataUploadTask添加到看门狗监控.");}

    if (DEBUG_MODE) {Serial.println("启动电压采集");}
    for(;;) {
        battery_senor_read();// 写入电压 更新数据
        esp_task_wdt_reset(); // 喂狗
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 数据上传任务
void dataUploadTask(void *pvParameters) {
    TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
    esp_err_t ret = esp_task_wdt_add(currentTaskHandle); // 将当前任务添加到看门狗监控
    if (ret = ESP_OK && DEBUG_MODE) {
    Serial.println("[deug] 已将dataUploadTask添加到看门狗监控.");}
    const TickType_t xFrequency = pdMS_TO_TICKS(IOT_DATA_UPLOAD_DELAY);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    GpsData localCopy; // 定义局部副本
    float BatteryPercentage = 0.0;
    if (DEBUG_MODE) {Serial.println("数据上传任务");}
    // 复制数据到局部变量（减少锁占用时间）
    for(;;) {
        if(enable_Iot_data_upload) {    //  是否启动了数据上传
                      // 分别获取GPS和电压数据的锁
            if(xSemaphoreTake(voltageMutex, portMAX_DELAY) == pdTRUE) {
                BatteryPercentage = batteryPercentage;  //读取
                xSemaphoreGive(voltageMutex);
            }
                      // 分别获取GPS和电压数据的锁
            if(xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE) {
                localCopy = Save_Data;
                xSemaphoreGive(gpsMutex);
            }
          // WIFI状态 RSSI值
            String rssi_string = "{\"RSSI\":" + String(rssi) + "}";
            // 电压数据
            String sensor_batteryData = "{\"battery\":" + String(BatteryPercentage) + "}"; 
            // GPS数据
            String sensor_gpsData = "{"
            "\"state\":\"home\","         // 新增状态字段
            "\"latitude\":" + String(atof(localCopy.latitude) / 100.0 + 0.2036154, 7) + ","  // 保留7位小数精度
            "\"longitude\":" + String(atof(localCopy.longitude) / 100.0 + 0.0168450, 7) + "," 
            "\"accuracy\":" + String(36.7, 1) +  // 修改字段名并指定1位小数
            "}";

          // 发送数据
            if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
            mqttClient.publish("esp32/system/wifi",rssi_string.c_str());
            mqttClient.publish("esp32/sensor/battery", sensor_batteryData.c_str());
            mqttClient.publish("esp32/state/attributes", sensor_gpsData.c_str());

            if (DEBUG_MODE) {
            Serial.printf("[DEBUG] MQTT发送: %s\n", rssi_string.c_str());
            Serial.printf("[DEBUG] MQTT发送: %s\n", sensor_batteryData.c_str());
            Serial.printf("[DEBUG] MQTT发送: %s\n", sensor_gpsData.c_str());
            }
            xSemaphoreGive(mqttMutex);
          }
        }
        esp_task_wdt_reset(); // 喂狗
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//物联网连接
void iotServiceTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10000);// 3s周期
    TickType_t xLastWakeTime = xTaskGetTickCount();

    TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();// 获取当前任务的句柄
    esp_err_t ret = esp_task_wdt_delete(currentTaskHandle);
    if (ret = ESP_OK && DEBUG_MODE) {
        Serial.println("[deug] 已将iotServiceTask从看门狗删除.");}
    setup_iot_server();
    while (1) {
        // 连接状态检查
        connect_check();
                // 维持MQTT心跳
    if(mqttClient.connected()) {
       mqttClient.publish("esp32/state", "online");
      }
    if(DEBUG_MODE)Serial.println("[DEBUG]连接检查正常!"); 
        mqttClient.loop(); // 处理 MQTT 事件
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 确保释放 CPU
    }
}

// 测距任务（激光测距）
void distanceTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(00);// 0.1s周期
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;) {
    laser_ranging_sensor_read();// 写入激光测距更新数据
    if (DEBUG_MODE) {Serial.print("im fine");}
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* 电子罗盘任务 */
void compassTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 任务周期100ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();// 获取当前任务的句柄
    esp_err_t ret = esp_task_wdt_delete(currentTaskHandle);
    if (ret = ESP_OK && DEBUG_MODE) {
        Serial.println("[deug] 已将iotServiceTask从看门狗删除.");}

    for(;;) {
        // 获取互斥锁（保护I2C总线访问）
        if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            int outputData[6];
            bool dataValid = true;

            // 读取原始数据  发送起始条件 + 写地址 + 寄存器地址 0x03
            Wire.beginTransmission(HMC5883_WriteAddress);
            Wire.write(0x03); // 从数据寄存器开始读取
            if(Wire.endTransmission(false) != 0) {
                dataValid = false;
            }
            //发送起始条件 + 读地址 + 读取 6 字节数据
            if(dataValid && Wire.requestFrom(HMC5883_WriteAddress, 6) == 6) {
                for(int i=0; i<6; i++) {
                    outputData[i] = Wire.read();
                }
            } else {
                dataValid = false;
            }
            xSemaphoreGive(dataMutex); // 释放互斥锁
            if (!dataValid) {
              Serial.println("[deug]I2C communication failed!");
              // 可选：重试或记录错误
                            }
            if(dataValid) {
                // 解析数据（原处理逻辑）
                int x = (outputData[0] << 8) | outputData[1];
                int z = (outputData[2] << 8) | outputData[3];
                int y = (outputData[4] << 8) | outputData[5];

                // 数据有效性检查
                if(x == 0 || y == 0) { // 典型错误值检查
                    continue;
                }

                // 校准处理
                x -= xOffset;
                y -= yOffset;

                // 计算角度
                float angle = atan2(y, x) * (180.0 / PI);
                if(angle < 0) angle += 360.0;
                angle -= 135.0; // 磁偏角校准
                if(angle < 0) angle += 360.0;

                // 更新共享数据
                if (xSemaphoreTake(compassMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    currentAngle = angle;
                    xSemaphoreGive(compassMutex);
                }
                /*
                // 发送到队列（示例）
                if(uxQueueSpacesAvailable(angleQueue) > 0) {
                    xQueueSend(angleQueue, &angle, 0);
                }
                */
                // 调试输出（建议使用条件编译）
                if(DEBUG_MODE) {
                    Serial.printf("[Compass] Angle: %.2f°\n", angle);
                    if (((angle >= 0.13) && (angle <= 0.31))||((angle >= 269.68) && (angle < 269.71))) 
                    Serial.println("北");
                    else if (((angle >= 0.00) && (angle < 0.13))||((angle >= 359.87) && (angle <= 360)))
                      Serial.println("东北");
                    else if (((angle >= 359.78) && (angle < 359.87))||((angle >= 90.17) && (angle <= 90.22)))
                      Serial.println("东");
                    else if ((angle >= 89.96) && (angle < 90.17)) 
                      Serial.println("东南");
                    else if (((angle >= 180.14) && (angle <= 180.20))||((angle >= 89.79) && (angle < 89.96))) 
                      Serial.println("南");
                    else if ((angle >= 179.74) && (angle < 180.14))
                      Serial.println("西南");
                    else if ((angle >= 270.01) && (angle < 270.28))
                      Serial.println("西");
                    else if ((angle >= 269.71) && (angle < 270.01))
                      Serial.println("西北");
                    else
                      Serial.println("新角度，请修改");
                }
            }
        }
        // 精确周期控制
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 发现绿化带
void visionTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 任务周期100ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;) {
    vision_sensor_read();// 视觉发现绿化带后更新数据
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

//警报任务（可以让逻辑任务调用，先挂起或者就绪，1000ms调试）
void laserWarningTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        int currentDistance = 0;
        // 安全读取激光测距数据
        if(xSemaphoreTake(laserMutex, pdMS_TO_TICKS(50))) {
            currentDistance = frontDistance; // 读取前方距离
            xSemaphoreGive(laserMutex);
            
            // 判断距离并发送警告
            if(currentDistance > 0 && currentDistance < 100) {
                Serial.println("1"); // 通过串口0发送
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}