#pragma once


#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include "i2c_slave.h"

#define RADIO_ENTRADA     3
#define ESC_SAIDA         2
#define WATCHDOG          13
#define HITEC_I2C_ADDRESS 0x08

// max time to wait for the ESC's tick
#define TIMEOUT_DATA      6000

#define FRAME_RESET            0
#define FRAME_REFERENCE        1
#define FRAME_VOLTAGE          2
#define FRAME_RIPPLE_VOLTAGE   3
#define FRAME_CURRENT          4
#define FRAME_THROTTLE         5
#define FRAME_OUTPUT_POWER     6
#define FRAME_RPM              7
#define FRAME_BEC_VOLTAGE      8
#define FRAME_BEC_CURRENT      9
#define FRAME_TEMP1           10
#define FRAME_TEMP2           11
#define FRAME_MAX             11

struct {
  uint8_t packid1;      // fixed, 0x11
  uint8_t header;       // fixed, 0xAF 
  uint8_t zero;         // fixed, 0x00
  uint8_t frametype;    // fixed, 0x2D
  uint8_t notused1;     // not used, 0x00
  uint8_t notused2;     // not used, 0x00
  uint8_t packid2;      // fixed, 0x11
} packet11;

struct {
  uint8_t packid1;      // fixed, 0x12
  uint16_t gpslatsec;   // gps latitude 1/100th of second
  uint16_t gpslatmin;   // gps latitude degre.minutes
  uint8_t gpstimesec;   // gps time seconds 
  uint8_t packid2;      // fixed, 0x12
} packet12;

struct {
  uint8_t packid1;      // fixed, 0x13
  uint16_t gpslongsec;  // gps longitude 1/100th of second
  uint16_t gpslongmin;  // gps longitude degre.minutes
  uint8_t temp2;        // temperature 2 - 40, in degre centigrades
  uint8_t packid2;      // fixed, 0x13
} packet13;

struct {
  uint8_t packid1;      // fixed, 0x14
  uint16_t speed;       // speed in km/h
  uint16_t alt;         // altitude in meters
  uint8_t temp1;        // temperature 1 - 40, in degre centigrades
  uint8_t packid2;      // fixed, 0x14
} packet14;

struct {
  uint8_t packid1;      // fixed, 0x15
  uint8_t fuelgauge;    // fuel level, 0 to 4
  uint16_t rpm1;        // RPM 1 / 10
  uint16_t rpm2;        // RPM 2 / 10
  uint8_t packid2;      // fixed, 0x15
} packet15;

struct {
  uint8_t packid1;      // fixed, 0x16
  uint8_t gpsdateyear;  // gps date year
  uint8_t gpsdatemonth; // gps date month
  uint8_t gpsdateday;   // gps date year
  uint8_t gpstimehour;  // gps time hour
  uint8_t gpstimemin;   // gps time minute
  uint8_t packid2;      // fixed, 0x16
} packet16;

struct {
  uint8_t packid1;      // fixed, 0x17
  uint16_t capdegree;   // gps heading
  uint8_t gpsstrength;  // gps signal strength
  uint8_t temp3;        // temperature 3 - 40, in degre centigrades
  uint8_t temp4;        // temperature 4 - 40, in degre centigrades
  uint8_t packid2;      // fixed, 0x17
} packet17;

struct {
  uint8_t packid1;      // fixed, 0x18
  uint16_t voltage;     // battery voltage * 10
  uint16_t current;     // current drawing * 14.5 + 164
  uint8_t zero;         // unknown
  uint8_t packid2;      // fixed, 0x18
} packet18;

#define PACKETCOUNT 8

void i2c_hitec_handler(i2c_inst_t *i2c, i2c_slave_event_t event);

//run setup I2C port configuration
void setupHitec();

//main Hitec loop
void handleHitec();

void runHitecRequest();