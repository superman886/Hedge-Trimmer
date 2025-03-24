//感知系统：激光测距*4 GPS*1 霍尔编码器*2 电压测量*1  gps*1 电压测量*1
//执行系统：底盘电机*2 LED*1 语音播报*1

#include "sensor_driver.h"
#include "MQTT_driver.h" 


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
    // 输出电压到串口监视器
    /*Serial.print("ADC Value: ");
    Serial.println(adcValue);
    Serial.print("voltageAtADC: ");
    Serial.println(voltageAtADC);
    Serial.print("Battery Voltage: ");
    Serial.print(actualVoltage);
    Serial.println(" V");
    Serial.print("batteryPercentage: ");
    Serial.print(batteryPercentage);
    Serial.println(" %");*/ 
    // 等待 1 秒 
    delay(1000);
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
            memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);                                          // 拷贝 GPS 数据到缓冲区
            Save_Data.isGetData = true;                                                                                             // 设置标志位

            // 清空接收缓冲区
            memset(gpsRxBuffer, 0, 600);        
            ii = 0;
        }
    }

    // 解析 GPS 数据
    if (Save_Data.isGetData) {               // 判断是否获取到完整的数据
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






