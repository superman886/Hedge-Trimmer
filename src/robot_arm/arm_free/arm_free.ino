#include <Stepper.h>
#include <Encoder.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <HardwareSerial.h>
#include <config.h>

// 硬件配置
#define GPS_SERIAL_NUM 2
HardwareSerial GPSSerial(GPS_SERIAL_NUM);

// TB6612 引脚定义
const int PWMA = 18;
const int AIN1 = 2;
const int AIN2 = 4;

// FreeRTOS 句柄
TaskHandle_t controlTaskHandle, motorTaskHandle, cmdTaskHandle, gpsTaskHandle;
QueueHandle_t cmdQueue;
SemaphoreHandle_t uartMutex;

// 状态机定义
enum State {
  EXECUTE_GROUP1,
  PAUSE_AFTER_GROUP1,
  START_DC_MOTOR,
  STOP_DC_MOTOR,
  PAUSE_AFTER_DC_MOTOR,
  EXECUTE_GROUP2,
  IDLE
};

volatile State currentState = EXECUTE_GROUP1;
unsigned long dcMotorStartTime = 0;
unsigned long pauseStartTime = 0;

// 步进电机结构体和控制函数（保持原有结构）
// ... [保持原有步进电机相关代码不变] ...

// 命令类型定义
enum Command {
  CMD_GROUP1,
  CMD_GROUP2,
  CMD_DC_START,
  CMD_DC_STOP,
  CMD_EMERGENCY_STOP
};

// GPS数据结构
struct GPSData {
  float latitude;
  float longitude;
};

// 控制任务
void controlTask(void *pvParameters) {
  while(1) {
    switch(currentState) {
      case EXECUTE_GROUP1:
        MotionGroups::executeGroup1();
        currentState = PAUSE_AFTER_GROUP1;
        pauseStartTime = millis();
        break;

      case PAUSE_AFTER_GROUP1:
        if(millis() - pauseStartTime >= 4000) {
          currentState = START_DC_MOTOR;
        }
        break;

      case START_DC_MOTOR:
        if(!motor1.isRunning && !motor2.isRunning && !motor3.isRunning) {
          controlDCMotor(true, 255);
          dcMotorStartTime = millis();
          currentState = STOP_DC_MOTOR;
        }
        break;

      case STOP_DC_MOTOR:
        if(millis() - dcMotorStartTime > 10000) {
          controlDCMotor(false);
          currentState = PAUSE_AFTER_DC_MOTOR;
          pauseStartTime = millis();
        }
        break;

      case PAUSE_AFTER_DC_MOTOR:
        if(millis() - pauseStartTime >= 2000) {
          currentState = EXECUTE_GROUP2;
        }
        break;

      case EXECUTE_GROUP2:
        MotionGroups::executeGroup2();
        currentState = IDLE;
        break;

      case IDLE:
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// 电机控制任务
void motorTask(void *pvParameters) {
  while(1) {
    updateMotor(motor1);
    updateMotor(motor2);
    updateMotor(motor3);
    vTaskDelay(pdMS_TO_TICKS(1)); // 保持高响应频率
  }
}

// 串口命令处理任务
void cmdTask(void *pvParameters) {
  Command cmd;
  while(1) {
    if(xQueueReceive(cmdQueue, &cmd, portMAX_DELAY)) {
      switch(cmd) {
        case CMD_GROUP1:
          currentState = EXECUTE_GROUP1;
          break;
        case CMD_GROUP2:
          currentState = EXECUTE_GROUP2;
          break;
        case CMD_DC_START:
          controlDCMotor(true, 255);
          break;
        case CMD_DC_STOP:
          controlDCMotor(false);
          break;
        case CMD_EMERGENCY_STOP:
          // 紧急停止处理
          break;
      }
    }
  }
}

// GPS任务
void gpsTask(void *pvParameters) {
  GPSData gps;
  while(1) {
    // 从GPS模块读取数据（示例代码）
    if(GPSSerial.available()) {
      String data = GPSSerial.readStringUntil('\n');
      // 解析NMEA数据（需要根据实际模块协议实现）
      // 这里假设已经解析得到经纬度
      gps.latitude = 39.9042;  // 示例数据
      gps.longitude = 116.4074;

      xSemaphoreTake(uartMutex, portMAX_DELAY);
      Serial.print("GPS,");
      Serial.print(gps.latitude, 6);
      Serial.print(",");
      Serial.println(gps.longitude, 6);
      xSemaphoreGive(uartMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// 串口接收中断处理
void serialEvent() {
  static String inputBuffer;
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '\n') {
      Command cmd = parseCommand(inputBuffer);
      if(cmd != -1) xQueueSend(cmdQueue, &cmd, 0);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

Command parseCommand(String input) {
  input.trim();
  if(input == "G1") return CMD_GROUP1;
  if(input == "G2") return CMD_GROUP2;
  if(input == "DC ON") return CMD_DC_START;
  if(input == "DC OFF") return CMD_DC_STOP;
  if(input == "STOP") return CMD_EMERGENCY_STOP;
  return (Command)-1;
}

void setup() {
    // 初始化GPS串口
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.begin(115200);   //主控单片机   tx：   rx：
  // 初始化硬件（保持原有初始化代码）
    pinMode(motor1.pulPin, OUTPUT);
  pinMode(motor1.dirPin, OUTPUT);
  pinMode(motor2.pulPin, OUTPUT);
  pinMode(motor2.dirPin, OUTPUT);
  pinMode(motor3.pulPin, OUTPUT);
  pinMode(motor3.dirPin, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);


  // FreeRTOS初始化
  cmdQueue = xQueueCreate(10, sizeof(Command));
  uartMutex = xSemaphoreCreateMutex();

  // 创建任务
  xTaskCreatePinnedToCore(controlTask,"Control",4096, NULL, 2, &controlTaskHandle, 0);
                                                              // 较高优先级
  xTaskCreatePinnedToCore( motorTask, "MotorCtrl", 4096, NULL, 3,   &motorTaskHandle, 0);

  xTaskCreatePinnedToCore( cmdTask, "CmdTask", 2048, NULL, 1, &cmdTaskHandle, 0);

  xTaskCreatePinnedToCore( gpsTask, "GPS", 2048, NULL, 1, &gpsTaskHandle, 0);


}

void loop() {
  // FreeRTOS接管任务调度
  vTaskDelete(NULL);
}