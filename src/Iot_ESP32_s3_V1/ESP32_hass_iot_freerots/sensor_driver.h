
#pragma once
#include <Arduino.h> 
#include <stdint.h>
#include "config.h"
#include <Wire.h>

void setMotor(int pwmPin, int in1, int in2, int speed);
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
void battery_senor_read();
void gps_senor_read();
void laser_ranging_sensor_read();
void vision_sensor_read();
void compass_sensor_read();
