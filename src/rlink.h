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
  float battVoltage = 12.75;
  float altitude = 10;
  int climb = 55;
  float roll = 1;
  float pitch = 2;
  float yaw = 3;
  uint8_t gps_sats = 5;
  int32_t gps_lon = -6000000L;//aérodrome, degrés x 10000000
  int32_t gps_lat = 447333300L;//aérodrome, degrés x 10000000
  float home_lon = -0.5672574;//Villenave d'Ornon
  float home_lat = 44.7799813;//Villenave d'Ornon
  float gps_speed = 100;
  bool gps_fix = false;
  float home_distance = 220;
};
