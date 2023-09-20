#pragma once

#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include "i2c_slave.h"
#include "xbus_spek_sensors.h"

uint8_t IDENTIFIER = 0;
uint8_t addressMask = 0;

#define XBUS_GPS_INFO_FLAGS_IS_NORTH_BIT 0
#define XBUS_GPS_INFO_FLAGS_IS_EAST_BIT 1
#define XBUS_GPS_INFO_FLAGS_LONG_GREATER_99_BIT 2
#define XBUS_GPS_INFO_FLAGS_NEGATIVE_ALT_BIT 7

#define NO_DATA 0xff
#define INT_NO_DATA 0xffff
#define UINT_NO_DATA_BE 0x7fff
#define UINT_NO_DATA_LE 0xff7f

typedef union
{
  unsigned char raw[2];
  unsigned int value;
} endianBuff_u;


//run setup I2C port configuration
void setupXbusSpektrum();

void i2c_spek_handler(i2c_inst_t *i2c, i2c_slave_event_t event);

//main Xbus Spektrum loop
void handleXbusSpektrum();

uint8_t bcd8(float value, uint8_t precision);
uint16_t bcd16(float value, uint8_t precision);
uint32_t bcd32(float value, uint8_t precision);