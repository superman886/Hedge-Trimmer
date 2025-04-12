
#pragma once
#include <Arduino.h> 
#include <stdint.h>
#include "config.h"

void battery_senor_read();
void gps_senor_read();
void moveForward(int L_pwm, int R_pwm);
void moveBackward(int L_pwm, int R_pwm);
void turnLeft(int L_pwm, int R_pwm);
void turnRight(int L_pwm, int R_pwm);
void stopMotors();
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
void laser_ranging_sensor_read();
void setMotor(int L_pwm ,int R_pwm );
void vision_sensor_read();
