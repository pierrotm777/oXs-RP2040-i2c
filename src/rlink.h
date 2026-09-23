#pragma once


#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include "i2c_slave.h"

//same adress as the Radiolink PRM-01
#define RLINK_I2C_ADDRESS 4

void i2c_rlink_handler(i2c_inst_t *i2c, i2c_slave_event_t event);
void runRlinkRequest();

//run setup I2C port configuration
void setupRlink();

//main Radiolink loop
void handleRlink();
// 1st packet witch start by 0x89, 0xAB
void set1();
// 2sd packet witch start by 0x89, 0xCD
void set2();

// Radiolink structure with defaults values
struct STREAM_DATA {
  float battVoltage = 0;
  float altitude = 0;
  int climb = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  uint8_t gps_sats = 0;
  int32_t gps_lon = 0;//degrés x 10000000
  int32_t gps_lat = 0;//degrés x 10000000
  float home_lon = 0;
  float home_lat = 0;
  float gps_speed = 0;
  bool gps_fix = false;
  float home_distance = 0;
};
