/*
运行逻辑
  程序开始时进入 EXECUTE_GROUP1 状态，执行动作组 1。
  动作组 1 完成后，进入 PAUSE_AFTER_GROUP1 状态，暂停 2 秒。
  暂停结束后，进入 START_DC_MOTOR 状态，启动直流电机。
  直流电机运行 5 秒后，进入 STOP_DC_MOTOR 状态，停止直流电机。
  停止直流电机后，进入 PAUSE_AFTER_DC_MOTOR 状态，暂停 2 秒。
  暂停结束后，进入 EXECUTE_GROUP2 状态，执行动作组 2。
  动作组 2 完成后，进入 IDLE 状态，程序进入空闲模式。
*/
#include <Stepper.h>
#include <Encoder.h> 
#include <Arduino.h> 
#include <TinyGPS++.h>

#define GPSSERIAL 9600                // GPS串口波特率
// TB6612 引脚定义
const int PWMA = 18;
const int AIN1 = 2;
const int AIN2 = 4;

const int stepsPerRevolution = 3200;

// 定义GPS数据区
char gpsRxBuffer[600] ;          // GPS串口接收缓存
unsigned int ii = 0;  // 串口接收缓存

TinyGPSPlus gps;// 定义GPS对象

/*****************  结构体  *****************/
//gps结构体类型
typedef struct {
    char GPS_Buffer[80];
    bool isGetData;     // 是否获取到GPS数据
    bool isParseData;   // 是否解析完成
    char UTCTime[11];   // UTC时间
    char latitude[11];  // 纬度
    char N_S[2];        // N/S
    char longitude[12]; // 经度
    char E_W[2];        // E/W
    bool isUsefull;     // 定位信息是否有效
}GpsData;


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

struct MotorAction {
  int degrees;
  bool direction;
  unsigned int delayAfter;
};

struct StepperMotor {
  uint8_t pulPin;
  uint8_t dirPin;
  unsigned long pulseInterval;
  MotorAction* actions;
  uint8_t actionCount;
  uint8_t currentAction;
  int remainingSteps;
  bool isRunning;
  bool pulseState;
  unsigned long lastStepMicros;
  unsigned long actionStartTime;
  bool inDelayPhase;
};

// 电机配置
StepperMotor motor1 = {26, 27, 500, nullptr, 0};
StepperMotor motor2 = {14, 15, 100, nullptr, 0};
StepperMotor motor3 = {16, 17, 300, nullptr, 0};

namespace MotionGroups {
  // 动作组定义
  const MotorAction group1_motor1[] = {{90, HIGH, 200}, {90, LOW, 200}};
  const MotorAction group1_motor2[] = {{270, LOW, 500}, {270, HIGH, 500}};
  const MotorAction group1_motor3[] = {{120, HIGH, 300}, {120, LOW, 300}};
  
  const MotorAction group2_motor1[] = {{90, HIGH, 200}, {720, LOW, 200}};
  const MotorAction group2_motor2[] = {{180, LOW, 500}, {180, HIGH, 500}};
  const MotorAction group2_motor3[] = {{150, HIGH, 300}, {150, LOW, 300}};

  void startMotors() {
    // 初始化三个步进电机
    motor1.currentAction = 0;
    motor1.pulseState = LOW;
    motor1.remainingSteps = abs(motor1.actions[0].degrees * stepsPerRevolution / 360);
    motor1.isRunning = true;
    digitalWrite(motor1.dirPin, motor1.actions[0].direction);
    motor1.lastStepMicros = micros();

    motor2.currentAction = 0;
    motor2.pulseState = LOW;
    motor2.remainingSteps = abs(motor2.actions[0].degrees * stepsPerRevolution / 360);
    motor2.isRunning = true;
    digitalWrite(motor2.dirPin, motor2.actions[0].direction);
    motor2.lastStepMicros = micros();

    motor3.currentAction = 0;
    motor3.pulseState = LOW;
    motor3.remainingSteps = abs(motor3.actions[0].degrees * stepsPerRevolution / 360);
    motor3.isRunning = true;
    digitalWrite(motor3.dirPin, motor3.actions[0].direction);
    motor3.lastStepMicros = micros();
  }

  void executeGroup1() {
    motor1.isRunning = motor2.isRunning = motor3.isRunning = false;
    delay(100);

    motor1.actions = (MotorAction*)group1_motor1;
    motor1.actionCount = sizeof(group1_motor1) / sizeof(MotorAction);
    motor2.actions = (MotorAction*)group1_motor2;
    motor2.actionCount = sizeof(group1_motor2) / sizeof(MotorAction);
    motor3.actions = (MotorAction*)group1_motor3;
    motor3.actionCount = sizeof(group1_motor3) / sizeof(MotorAction);

    startMotors();
  }

  void executeGroup2() {
    motor1.isRunning = motor2.isRunning = motor3.isRunning = false;
    delay(100);

    motor1.actions = (MotorAction*)group2_motor1;
    motor1.actionCount = sizeof(group2_motor1) / sizeof(MotorAction);
    motor2.actions = (MotorAction*)group2_motor2;
    motor2.actionCount = sizeof(group2_motor2) / sizeof(MotorAction);
    motor3.actions = (MotorAction*)group2_motor3;
    motor3.actionCount = sizeof(group2_motor3) / sizeof(MotorAction);

    startMotors();
  }
}

void controlDCMotor(bool enable, int speed = 255) {
  if (enable) {
    Serial.println("Enabling DC Motor");
    digitalWrite(AIN1, HIGH);  // 正转方向
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);  // 设置速度 (0-255)
  } else {
    Serial.println("Disabling DC Motor");
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);      // 停止电机
  }
}

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

State currentState = EXECUTE_GROUP1;  // 初始状态
unsigned long dcMotorStartTime = 0;   // 直流电机启动时间
unsigned long pauseStartTime = 0;     // 暂停开始时间

void setup() {
  // 初始化所有引脚
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

  Serial.begin(115200);
  Serial1.begin(GPSSERIAL);
}

void loop() {
  updateMotor(motor1);
  updateMotor(motor2);
  updateMotor(motor3);

  switch (currentState) {
    case EXECUTE_GROUP1:
      MotionGroups::executeGroup1();
      currentState = PAUSE_AFTER_GROUP1;  // 转到暂停状态
      pauseStartTime = millis();          // 记录暂停开始时间
      break;

    case PAUSE_AFTER_GROUP1:
      if (millis() - pauseStartTime >= 4000) {  // 暂停 2 秒 ＋ 电机转动 2 秒
        currentState = START_DC_MOTOR;          // 转到启动直流电机状态
      }
      break;

    case START_DC_MOTOR:
      if (!motor1.isRunning && !motor2.isRunning && !motor3.isRunning) {
        controlDCMotor(true, 255);       // 启动直流电机
        dcMotorStartTime = millis();     // 记录启动时间
        currentState = STOP_DC_MOTOR;    // 转到停止直流电机状态
      }
      break;

    case STOP_DC_MOTOR:
      if (millis() - dcMotorStartTime > 10000) {  // 直流电机运行 5 秒
        controlDCMotor(false);            // 停止直流电机
        currentState = PAUSE_AFTER_DC_MOTOR;  // 转到暂停状态
        pauseStartTime = millis();        // 记录暂停开始时间
      }
      break;

    case PAUSE_AFTER_DC_MOTOR:
      if (millis() - pauseStartTime >= 2000) {  // 暂停2秒
        currentState = EXECUTE_GROUP2;          // 转到执行动作组2状态
      }
      break;

    case EXECUTE_GROUP2:
      MotionGroups::executeGroup2();      //执行动作组2
      currentState = IDLE;                // 转到空闲状态
      break;

    case IDLE:
      // 空闲状态，不做任何操作
      break;
  }
}

void generatePulse(StepperMotor &motor) {
  if (micros() - motor.lastStepMicros >= motor.pulseInterval) {
    digitalWrite(motor.pulPin, motor.pulseState ? HIGH : LOW);
    motor.pulseState = !motor.pulseState;
    if (!motor.pulseState) motor.remainingSteps--;
    motor.lastStepMicros = micros();
  }
}

void updateMotor(StepperMotor &motor) {
  if (!motor.isRunning) return;

  if (motor.remainingSteps > 0) {
    generatePulse(motor);
  } else {
    if (!motor.inDelayPhase) {
      motor.actionStartTime = millis();
      motor.inDelayPhase = true;
    }

    if (millis() - motor.actionStartTime >= motor.actions[motor.currentAction].delayAfter) {
      motor.inDelayPhase = false;

      if (++motor.currentAction < motor.actionCount) {
        digitalWrite(motor.dirPin, motor.actions[motor.currentAction].direction);
        motor.remainingSteps = abs(motor.actions[motor.currentAction].degrees * stepsPerRevolution / 360);
      } else {
        motor.isRunning = false;
      }
    }
  }
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
                        Serial.print("[debug]解析空2");
                }
            }
        }
  }

    // 打印解析后的 GPS 数据

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