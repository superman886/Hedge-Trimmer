//感知系统：激光测距*4 GPS*1 霍尔编码器*2   电压测量*1
//执行系统：底盘电机*2 LED*1 语音播报*1

#include "sensor_driver.h"
#include "MQTT_driver.h" 
#include "Wire.h"

//初始化1.1


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

//电压传感器读取
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
    //delay(1000);
  }
  
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

// 电机控制函数
void setMotor(int pwmPin, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, speed > 0 ? HIGH : LOW);
  digitalWrite(in2, speed > 0 ? LOW : HIGH);
  analogWrite(pwmPin, abs(speed));
}

/*
// 编码器中断服务程序
void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount += digitalRead(ENCODER_LEFT_B) ? -1 : 1;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount += digitalRead(ENCODER_RIGHT_B) ? -1 : 1;
}

*/
/* 激光测距传感器读取 */
void laser_ranging_sensor_read()
{
  /*
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4)
  { // 判断测量结果状态
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
    obstacleDetected = (measure.RangeMilliMeter <= Obstacle_distance);
    Serial.print(obstacleDetected);
    Serial.print("\n");
  } 
  else 
    Serial.println("Measurement failed!");
  */
  // 测量并输出第一个传感器的距离
  VL53L0X_RangingMeasurementData_t measure1;
  lox1.rangingTest(&measure1, false);
  if (measure1.RangeStatus != 4) {
    Serial.print("Sensor1 Distance (mm): ");
    Serial.print(measure1.RangeMilliMeter);
  } 
  else {
    Serial.print("Sensor1 out of range");
  }

  // 测量并输出第二个传感器的距离
  VL53L0X_RangingMeasurementData_t measure2;
  lox2.rangingTest(&measure2, false);
  if (measure2.RangeStatus != 4) {
    Serial.print(" | Sensor2 Distance (mm): ");
    Serial.println(measure2.RangeMilliMeter);
  } else {
    Serial.println(" | Sensor2 out of range");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
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
  //Serial.println(treelawnDetected);  
}

/*电子罗盘读取方位*/
void compass_sensor_read()
//void compass_sensor_read(TwoWire &wire)
{
  int i, x, y, z; // 三轴数据
  double angle;
  int outputData[6];

  I2C_2.beginTransmission(HMC5883_WriteAddress);
  I2C_2.write(HMC5883_ModeRegisterAddress);
  I2C_2.write(HMC5883_ContinuousModeCommand);    //设置 HMC5883L 的工作模式为连续测量模式，并等待 50ms 以确保模式切换完成。
  I2C_2.endTransmission();
  vTaskDelay(50 / portTICK_PERIOD_MS);


  I2C_2.requestFrom(HMC5883_WriteAddress, 6);    // 请求从 HMC5883L 读取 6 字节数据（X、Y、Z 轴的高字节和低字节）。
  vTaskDelay(50 / portTICK_PERIOD_MS);

  if (I2C_2.available() >= 6)                    // 如果有 6 字节数据可读
  { 
    for (i = 0; i < 6; i++)
      outputData[i] = I2C_2.read(); // 读取数据并存储到数组中
  } 
  else
  {
    Serial.println("Error: Not enough data available!");
    return; // 跳过本次循环
  }

  // 解析数据，将读取的 6 字节数据解析为 X、Y、Z 轴的 16 位整数值。
  // 每个轴的数据由两个字节组成（高字节和低字节），通过位操作组合成完整的数值。
  x = outputData[0] << 8 | outputData[1]; // 组合 X 轴数据，高位向左移8位或上低位即可组合成一个真正的数据
  z = outputData[2] << 8 | outputData[3]; // 组合 Z 轴数据
  y = outputData[4] << 8 | outputData[5]; // 组合 Y 轴数据
  
  //检查是否出现无效数据（x 和 y 都为 0）。如果无效，则打印错误信息并跳过本次循环。
  if (x == 0 && y == 0)
  {
    Serial.println("Error: Invalid magnetic data (x = 0, y = 0)");
    return; // 跳过本次循环
  }

  //对 X 和 Y 轴数据应用偏移量，校准传感器的原始数据。
  x -= xOffset;
  y -= yOffset;

  angle = atan2((double)y, (double)x) * (180 / 3.14159265); //使用 atan2(y, x) 计算方向角（弧度），然后将其转换为角度值（0~360 度）。
  if (angle < 0) angle += 360.0; // 将角度调整为 0~360 度范围 
  angle -= 135.0;// 手动补偿固定偏差（根据实际情况，可修改）
  if (angle < 0) angle += 360.0; // 将角度调整为 0~360 度范围
  // 打印角度值
  Serial.print(angle, 2); //保留两位小数
  Serial.println(" °");

  // 判断方向
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
  vTaskDelay(50 / portTICK_PERIOD_MS);
}



