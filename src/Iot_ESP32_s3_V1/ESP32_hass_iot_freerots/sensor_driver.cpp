//感知系统：激光测距*4 GPS*1 霍尔编码器*2   电压测量*1
//执行系统：底盘电机*2 LED*1 语音播报*1

#include "sensor_driver.h"
#include "MQTT_driver.h" 

GpsData Save_Data = {
    .GPS_Buffer = {0},
    .isGetData = false,
    .isParseData = false,
    .UTCTime = {0},
    .latitude = {0},
    .N_S = {0},
    .longitude = {0},
    .E_W = {0},
    .isUsefull = false,
};

/******************** 电机控制函数 ********************/
void setMotor(int L_pwm ,int R_pwm ) {
    L_pwm = constrain(L_pwm, -255, 255);  // PWM范围限制
    R_pwm = constrain(R_pwm, -255, 255);  // PWM范围限制
    // 设置方向
    digitalWrite(IN1, L_pwm >= 0 ? HIGH : LOW);
    digitalWrite(IN2, L_pwm >= 0 ? LOW : HIGH);
    digitalWrite(IN3, R_pwm >= 0 ? HIGH : LOW);
    digitalWrite(IN4, R_pwm >= 0 ? LOW : HIGH);
    // 设置PWM（绝对值）
  ledcWrite(ENA, abs(L_pwm));
  ledcWrite(ENB, abs(R_pwm));
}

/******************** 运动控制函数 ********************/
void moveForward(int L_pwm, int R_pwm) {  // 前进
    setMotor(L_pwm, R_pwm);  // 左右电机正转
}

void moveBackward(int L_pwm, int R_pwm) {  // 后退
    setMotor(-L_pwm, -R_pwm);  // 左右电机反转
}

void turnLeft(int L_pwm, int R_pwm) {  // 向左自转
    setMotor(-L_pwm, R_pwm);  // 左电机反转，右电机正转
}

void turnRight(int L_pwm, int R_pwm) {  // 向右自转
    setMotor(L_pwm, -R_pwm);  // 左电机正转，右电机反转
}

void stopMotors() { 
    setMotor(0, 0);  // 左右电机停止
}

// 编码器中断服务程序
void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount += digitalRead(ENCODER_LEFT_B) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount += digitalRead(ENCODER_RIGHT_B) ? 1 : -1;
}

//GPS读取解析
void gps_senor_read(){

    // 读取 GPS 数据
    while (Serial1.available()) {
        gpsRxBuffer[ii++] = Serial1.read();
          if (ii == 600) {
            memset(gpsRxBuffer, 0, 600); // 清空缓冲区
            ii = 0;
        }
    }

    // 查找 GPS 数据头
    char* GPS_BufferHead;    // GPS 数据头
    char* GPS_BufferTail;    // GPS 数据尾
    if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL) {   // 查找 GPS 数据头
        if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead)) {                     // 查找 GPS 数据尾
         if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);                                          // 拷贝 GPS 数据到缓冲区
            Save_Data.isGetData = true;                                                                                             // 设置标志位
              xSemaphoreGive(gpsMutex);  // 解锁
        }
            // 清空接收缓冲区
            memset(gpsRxBuffer, 0, 600);        
            ii = 0;
        }
    }

    // 解析 GPS 数据
    if (Save_Data.isGetData) {               // 判断是否获取到完整的数据
      if(xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE){
        Save_Data.isGetData = false;          // 重置标志位
        char *subString;                      // 创建子字符串指针
        char *subStringNext;                   // 创建子字符串指针
        for (int i = 0; i <= 6; i++) {          // 循环解析数据
            if (i == 0) {
                if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL) {        // 判断是否为空 
                    if(DEBUG_MODE){
                      Serial.print("[debug]解析空1");
                    }
                }
            } else {
                subString++;
                if ((subStringNext = strstr(subString, ",")) != NULL) {              //继续向前移动到下一个逗号，并使用strstr找到下一个逗号的位置。
                    char usefullBuffer[2];
                    switch (i) {
                        case 1: memcpy(Save_Data.UTCTime, subString, subStringNext - subString); break; // 获取 UTC 时间
                        case 2: memcpy(usefullBuffer, subString, subStringNext - subString); break; // 获取有效性标志
                        case 3: memcpy(Save_Data.latitude, subString, subStringNext - subString); break; // 获取纬度信息
                        case 4: memcpy(Save_Data.N_S, subString, subStringNext - subString); break; // 获取 N/S
                        case 5: memcpy(Save_Data.longitude, subString, subStringNext - subString); break; // 获取经度信息
                        case 6: memcpy(Save_Data.E_W, subString, subStringNext - subString); break; // 获取 E/W
                        default: break;
                    }
                    //更新子字符串指针和状态标志
                    subString = subStringNext;          // 移动到下一个子串的起始位置
                    Save_Data.isParseData = true;       // 设置为已解析
                    if (usefullBuffer[0] == 'A') {      // 判断是否为有效数据
                        Save_Data.isUsefull = true;        // 设置为有效
                    } else if (usefullBuffer[0] == 'V') {     // 判断是否为无效数据
                        Save_Data.isUsefull = false;          // 设置为无效
                    }
                } else {
                        if(DEBUG_MODE){
                        Serial.print("[debug]解析空2");
                    }
                }
            }
        }
      xSemaphoreGive(gpsMutex); //  解锁
      }
  }

    // 打印解析后的 GPS 数据
    if(DEBUG_MODE){
        if (Save_Data.isParseData) {
            Save_Data.isParseData = false;
    
            Serial.print("Save_Data.UTCTime = ");
            Serial.println(Save_Data.UTCTime);
    
            if (Save_Data.isUsefull) {
                Save_Data.isUsefull = false;
                Serial.print("Save_Data.latitude = ");
                Serial.println(Save_Data.latitude);
                Serial.print("Save_Data.N_S = ");
                Serial.println(Save_Data.N_S);
                Serial.print("Save_Data.longitude = ");
                Serial.println(Save_Data.longitude);
                Serial.print("Save_Data.E_W = ");
                Serial.println(Save_Data.E_W);
            } else {
                Serial.println("GPS DATA is not useful!");
            }
        }
    } 
}

//电压传感器读取
void battery_senor_read(){
    int adcValue = analogRead(adcPin); // 读取 ADC 值
    float voltageAtADC = (adcValue / 4095.0) * 3.3 + 0.05;  //// 计算电压手动加了0.16v
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
    if(xSemaphoreTake(voltageMutex, portMAX_DELAY) == pdTRUE){
    batteryPercentage = percentage;  //写入
      xSemaphoreGive(voltageMutex);
      }
    if (DEBUG_MODE) {   Serial.print("采集电量：");
                        Serial.print(percentage);
                        Serial.println("%");
                        Serial.print("采样");
                        Serial.println(adcValue);
                        Serial.println("电压");
                        Serial.println(voltageAtADC);
                        Serial.println("V");
                        Serial.println(actualVoltage);
                        Serial.print("V");
                                                  }
  }



/* 激光测距传感器 */
void laser_ranging_sensor_read() {
    VL53L0X_RangingMeasurementData_t measure1, measure2;

    // 读取前向传感器
    lox1.rangingTest(&measure1, false);
    if (measure1.RangeStatus == 0) { // 状态码0表示有效数据
        if (xSemaphoreTake(laserMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            frontDistance = measure1.RangeMilliMeter;
            xSemaphoreGive(laserMutex);
        }
        if(DEBUG_MODE){
            Serial.print(" 右 : ");
            Serial.print(frontDistance);
            Serial.println("mm");
        }  
    }
    else {
      if(DEBUG_MODE){Serial.println(" | 右 out of range");}
    }

    // 读取左侧传感器
    lox2.rangingTest(&measure2, false);
    if (measure2.RangeStatus == 0) {
        if (xSemaphoreTake(laserMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            leftDistance = measure2.RangeMilliMeter;
            xSemaphoreGive(laserMutex);
        }
        if(DEBUG_MODE){
            Serial.print(" 左 : ");
            Serial.print(frontDistance);
            Serial.println("mm");
        }  
    }
    else {
         if(DEBUG_MODE){Serial.println(" | 左 out of range");}
    }

}

// 获取当前角度的线程安全函数
float getCurrentAngle() {
    float angle = 0.0;
    if(xSemaphoreTake(compassMutex, pdMS_TO_TICKS(20))) {
        angle = currentAngle;
        xSemaphoreGive(compassMutex);
    }
    return angle;
}

/* 视觉传感器读取 */
void vision_sensor_read()
{
  if(Serial2.available())                         //判断串口2是否有数据
  {
    String data = Serial2.readStringUntil('\n');  //如果有数据，就接收，以一个字符串的形式
    //Serial.println(data);                       //把接收到的数据打印到串口监视器  
    if(data == "treelawn") 
    {
      treelawnDetected = true;                  // 设置treelawn检测标志位
      Serial.println(data);                       //把接收到的数据打印到串口监视器  
      Serial.print("i found it!!");
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  } 
  else
  {
    treelawnDetected = false;                     // 未发现绿化带，设置treelawn检测标志位
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
