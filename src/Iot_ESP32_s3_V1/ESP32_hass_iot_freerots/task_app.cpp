#include "task_app.h"
#include "MQTT_driver.h"  //  MQTT驱动
#include "sensor_driver.h" // 传感器驱动


// IoT服务任务（包含WiFi和MQTT）
void iotServiceTask(void *pvParameters) {
    setup_iot_server(); // 初始化WiFi和MQTT
    while(1) {
        // 连接状态检查
        connect_check();
    }
        // 维持MQTT心跳
    if(mqttClient.connected()) {
       mqttClient.publish("esp32/state", "online");
      }
    if(DEBUG_MODE)Serial.println("[DEBUG]连接检查正常!");  
    vTaskDelay(5000 / portTICK_PERIOD_MS);   //多少秒执行一次
}

// 测距任务（激光测距）
void distanceTask(void *pvParameters) {
  uint16_t distances[4];
  while (1) {
    distances[0] = sensor1.readRangeContinuousMillimeters();
    distances[1] = sensor2.readRangeContinuousMillimeters();
    distances[2] = sensor3.readRangeContinuousMillimeters();
    distances[3] = sensor4.readRangeContinuousMillimeters();
    
    xQueueOverwrite(distanceQueue, &distances);
    vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz更新
  }
}

// 编码器读取任务
void encoderTask(void *pvParameters) {
  int32_t counts[2];
  while (1) {
    counts[0] = leftEncoderCount;
    counts[1] = rightEncoderCount;
    xQueueOverwrite(encoderQueue, &counts);
    vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz更新
  }
}

//  电压传感器采集任务（电压、gps）
void sensorTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_READ_DELAY);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        if(xSemaphoreTake(voltageMutex, portMAX_DELAY) == pdTRUE){
            battery_senor_read();// 写入电压 更新数据
            xSemaphoreGive(voltageMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// GPS处理任务
void gpsTask(void *pvParameters) {
    for(;;) {
            gps_senor_read();// 写入gps 更新数据
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// 电机控制任务（ai写的根据自己修改）
void motorControlTask(void *pvParameters) {
  PIDController pidLeft, pidRight;
  int32_t encoderCounts[2];
  uint16_t distances[4];
  float targetSpeed = 0.5; // m/s
  float wheelCircumference = 0.157; // 2*pi*r (假设轮径5cm)

  while (1) {
    // 获取编码器数据（转换为速度m/s）
    xQueuePeek(encoderQueue, &encoderCounts, 0);
    float leftSpeed = (encoderCounts[0] / 20.0) * wheelCircumference; // 假设20脉冲/转
    float rightSpeed = (encoderCounts[1] / 20.0) * wheelCircumference;

    // PID计算
    float leftError = targetSpeed - leftSpeed;
    pidLeft.integral += leftError * 0.01; // dt=10ms
    float leftOutput = pidLeft.Kp * leftError + pidLeft.Ki * pidLeft.integral;

    // 同理计算右轮PID...
        // 右轮 PID 计算（同理）
    float rightError = targetSpeed - rightSpeed;
    pidRight.integral += rightError * 0.01;
    float rightOutput = pidRight.Kp * rightError + pidRight.Ki * pidRight.integral;
    // 驱动电机
    setMotor(PWMA, AIN1, AIN2, (int)leftOutput);
    setMotor(PWMB, BIN1, BIN2, (int)rightOutput);

    // 避障逻辑
    xQueuePeek(distanceQueue, &distances, 0);
    if (distances[0] < 200 || distances[1] < 150 || distances[2] < 150) {
      setMotor(PWMA, AIN1, AIN2, 0);
      setMotor(PWMB, BIN1, BIN2, 0);
      vTaskDelay(pdMS_TO_TICKS(500));
      setMotor(PWMA, AIN1, AIN2, -150);
      setMotor(PWMB, BIN1, BIN2, -150);
      vTaskDelay(pdMS_TO_TICKS(300));
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}




// 数据上传任务
void dataUploadTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(IOT_DATA_UPLOAD_DELAY);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    GpsData localCopy; // 定义局部副本
    float BatteryPercentage = 0.0;
    // 复制数据到局部变量（减少锁占用时间）
    for(;;) {
        if(enable_Iot_data_upload) {    //  是否启动了数据上传
                      // 分别获取GPS和电压数据的锁
            if(xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE) {
                localCopy = Save_Data;
                xSemaphoreGive(gpsMutex);
            }
            if(xSemaphoreTake(voltageMutex, portMAX_DELAY) == pdTRUE) {
                BatteryPercentage = batteryPercentage;
                xSemaphoreGive(voltageMutex);
            }
          // WIFI状态 RSSI值
            String rssi_string = "{\"RSSI\":" + String(rssi) + "}";
            // 电压数据
            String sensor_batteryData = "{\"battery\":" + String(BatteryPercentage) + "}"; 
            // GPS数据
            String sensor_gpsData;
            sensor_gpsData = "{\"latitude\":" + String(localCopy.latitude) + 
                              ",\"longitude\":" + String(localCopy.longitude) + 
                              ",\"gps_accuracy\":" + String(1.5)+ "}";
          
          // 发送数据
            if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
            mqttClient.publish("esp32/system/wifi",rssi_string.c_str());
            mqttClient.publish("esp32/sensors/battery", sensor_batteryData.c_str());
            mqttClient.publish("esp32/sensors/gps", sensor_gpsData.c_str());

            if (DEBUG_MODE) {
            Serial.printf("[DEBUG] MQTT发送: %s\n", rssi_string.c_str());
            Serial.printf("[DEBUG] MQTT发送: %s\n", sensor_batteryData.c_str());
            Serial.printf("[DEBUG] MQTT发送: %s\n", sensor_gpsData.c_str());
            }
                        xSemaphoreGive(mqttMutex);
          }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



/* MQTT事务任务程序 *///？？？？
void MQTT_event_app(){
   if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
    mqttClient.loop(); // 处理MQTT事务
            xSemaphoreGive(mqttMutex);
    }
}
