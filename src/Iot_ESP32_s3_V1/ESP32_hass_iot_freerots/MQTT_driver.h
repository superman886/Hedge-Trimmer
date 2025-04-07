#pragma once
#include <Arduino.h> 
#include <stdint.h>
#include "config.h"


void wifi_setup();
void mqtt_setup();
void mqtt_subscribe_setup();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void setup_iot_server();
void connect_check();
void hass_debug_log(char *log);
