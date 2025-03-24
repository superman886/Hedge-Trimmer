/**
 * @~Chinese
 * @file run_speed.ino
 * @brief 示例：以指定的速度（单位RPM）驱动电机。
 * @example run_speed.ino
 * 以指定的速度（单位RPM）驱动电机。
 */


#include "encoder_motor.h"
#include "encoder_motor_lib.h"

const int16_t speed_rpm = 100;

namespace {
constexpr uint32_t kPPR = 12;              // 每转脉冲数。
constexpr uint32_t kReductionRation = 90;  // 减速比。

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
em::EncoderMotor g_encoder_motor_0(  // 定义电机实例E0，适用于ESP32 Arduino核心版本3.0.0及以上
    GPIO_NUM_13,                     // 电机正极引脚编号
    GPIO_NUM_13,                     // 电机负极引脚编号
    GPIO_NUM_18,                     // 编码器A相引脚编号
    GPIO_NUM_19,                     // 编码器B相引脚编号
    kPPR,                            // 每转脉冲数
    kReductionRation,                // 减速比
    em::EncoderMotor::kAPhaseLeads   // 相位关系（前向旋转时A相领先或B相领先）
);

em::EncoderMotor g_encoder_motor_1(  // E1
    GPIO_NUM_4,                      // The pin number of the motor's positive pole.
    GPIO_NUM_2,                      // The pin number of the motor's negative pole.
    GPIO_NUM_5,                      // The pin number of the encoder's A phase.
    GPIO_NUM_23,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_2(  // E2
    GPIO_NUM_17,                     // The pin number of the motor's positive pole.
    GPIO_NUM_12,                     // The pin number of the motor's negative pole.
    GPIO_NUM_35,                     // The pin number of the encoder's A phase.
    GPIO_NUM_36,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_3(  // E3
    GPIO_NUM_15,                     // The pin number of the motor's positive pole.
    GPIO_NUM_14,                     // The pin number of the motor's negative pole.
    GPIO_NUM_34,                     // The pin number of the encoder's A phase.
    GPIO_NUM_39,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

#else  // The ESP32 Arduino Core Version is less than 3.0.0

em::EncoderMotor g_encoder_motor_0(  // E0
    GPIO_NUM_27,                     // The pin number of the motor's positive pole.
    0,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 0.
    GPIO_NUM_13,                     // The pin number of the motor's negative pole.
    1,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 1.
    GPIO_NUM_18,                     // The pin number of the encoder's A phase.
    GPIO_NUM_19,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_1(  // E1
    GPIO_NUM_4,                      // The pin number of the motor's positive pole.
    2,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 2.
    GPIO_NUM_2,                      // The pin number of the motor's negative pole.
    3,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 3.
    GPIO_NUM_5,                      // The pin number of the encoder's A phase.
    GPIO_NUM_23,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_2(  // E2
    GPIO_NUM_17,                     // The pin number of the motor's positive pole.
    4,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 4.
    GPIO_NUM_12,                     // The pin number of the motor's negative pole.
    5,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 5.
    GPIO_NUM_35,                     // The pin number of the encoder's A phase.
    GPIO_NUM_36,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_3(  // E3
    GPIO_NUM_15,                     // The pin number of the motor's positive pole.
    6,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 6.
    GPIO_NUM_14,                     // The pin number of the motor's negative pole.
    7,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 7.
    GPIO_NUM_34,                     // The pin number of the encoder's A phase.
    GPIO_NUM_39,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);
#endif
}  // namespace

void direction_if(){
  if (speed_rpm > 0) { // 如果速度为正，则电机前进
    AIN1 = 0;
    AIN2 = 1;
  } 
  else if (speed_rpm < 0) { // 如果速度为负，则电机后退
    AIN1 = 1;
    AIN2 = 0;
  }
  else { // 如果速度为零，则停止
    AIN1 = 0;
    AIN2 = 0;
  }
}

void setup() {
  Serial.begin(115200);
  printf("setting up\n");
  printf("Emakefun Encoder Motor Library Version: %s\n", em::esp_encoder_motor_lib::Version().c_str());
  g_encoder_motor_0.Init();
  g_encoder_motor_1.Init();
  g_encoder_motor_2.Init();
  g_encoder_motor_3.Init();
  printf("setup completed\n");
}

void loop() {
  
  direction_if();

  g_encoder_motor_0.RunSpeed(speed_rpm);
  g_encoder_motor_1.RunSpeed(speed_rpm);
  g_encoder_motor_2.RunSpeed(speed_rpm);
  g_encoder_motor_3.RunSpeed(speed_rpm);

  printf("target speed rpm: %" PRIi16 ", current speed rpm: [%4" PRId32 ", %4" PRId32 ", %4" PRId32 ", %4" PRId32
         "], pwm duties: [%5 " PRIi16 ", %5" PRIi16 ", %5" PRIi16 ", %5" PRIi16 "], pulse counts: [%" PRId64 ", %" PRId64
         ", %" PRId64 ", %" PRId64 "]\n",
         speed_rpm,
         g_encoder_motor_0.SpeedRpm(),
         g_encoder_motor_1.SpeedRpm(),
         g_encoder_motor_2.SpeedRpm(),
         g_encoder_motor_3.SpeedRpm(),
         g_encoder_motor_0.PwmDuty(),
         g_encoder_motor_1.PwmDuty(),
         g_encoder_motor_2.PwmDuty(),
         g_encoder_motor_3.PwmDuty(),
         g_encoder_motor_0.EncoderPulseCount(),
         g_encoder_motor_1.EncoderPulseCount(),
         g_encoder_motor_2.EncoderPulseCount(),
         g_encoder_motor_3.EncoderPulseCount());

  delay(100);
}