#include <Stepper.h>
#include <Encoder.h> 
#include <Arduino.h> 
#include <TinyGPS++.h>

#define SERIAL_S3 19200                // 主控串口波特率
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

// 新增串口命令相关变量
char serialCommand[32] = {0};  // 接收命令缓冲区
volatile bool newCommand = false;  // 新命令标志
bool commandExecuting = false;    // 命令执行状态

namespace MotionGroups {
  // 动作组定义（保持不变）
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

// 新增：根据命令执行动作组   ！！！！！！！
void executeCommand(const char* cmd) {
  if (strcmp(cmd, "GROUP1") == 0) {
    currentState = EXECUTE_GROUP1 ;
  } else if (strcmp(cmd, "GROUP2") == 0) {
    currentState = EXECUTE_GROUP2;
  } else if (strcmp(cmd, "STOP") == 0) {
    // 停止所有电机
    motor1.isRunning = motor2.isRunning = motor3.isRunning = false;
    controlDCMotor(false);
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

// 状态机定义（保持不变）
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
unsigned long gpsSendTimer = 0;       // GPS发送定时器

// 新增：串口0中断处理函数
void serial0ISR() {
  static char c;
  while (Serial.available()) {
    c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialCommand[sizeof(serialCommand)-1] = '\0';  // 确保字符串终止
      newCommand = true;
    } else {
      for (int i = 0; i < sizeof(serialCommand)-1; i++) {
        if (serialCommand[i] == '\0') {
          serialCommand[i] = c;
          break;
        }
      }
    }
  }
}

void setup() {
  // 初始化所有引脚（保持不变）
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

  Serial.begin(SERIAL_S3);
  Serial1.begin(GPSSERIAL);

  // 启用串口0中断
  attachInterrupt(digitalPinToInterrupt(0), serial0ISR, CHANGE);
}

void loop() {
  updateMotor(motor1);
  updateMotor(motor2);
  updateMotor(motor3);

  // 处理串口命令
  if (newCommand && !commandExecuting) {
    commandExecuting = true;
    executeCommand(serialCommand);  //serialCommand名字！！！！！！！！
    memset(serialCommand, 0, sizeof(serialCommand));
    newCommand = false;
    commandExecuting = false;
  }


  // 状态机逻辑（保持不变）
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

// 电机控制函数（保持不变）
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
