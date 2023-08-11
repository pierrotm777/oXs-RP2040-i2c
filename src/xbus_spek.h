#pragma once

#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include "i2c_slave.h"
#include "xbus_spek_sensors.h"

#define IDENTIFIER   TELE_DEVICE_GPS_LOC

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

