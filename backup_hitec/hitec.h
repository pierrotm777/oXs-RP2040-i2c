#pragma once


#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include "i2c_slave.h"



#define HITEC_I2C_ADDRESS 0x08
#define HITEC_TIMEOUT 1000

#define HITEC_FRAME_0X11 0
#define HITEC_FRAME_0X12 1
#define HITEC_FRAME_0X13 2
#define HITEC_FRAME_0X14 3
#define HITEC_FRAME_0X15 4
#define HITEC_FRAME_0X16 5
#define HITEC_FRAME_0X17 6
#define HITEC_FRAME_0X18 7
#define HITEC_FRAME_0X19 8
#define HITEC_FRAME_0X1A 9
#define HITEC_FRAME_0X1B 10

#define HITEC_FRAME_0X11_RX_BATT 0

#define HITEC_FRAME_0X12_GPS_LAT 0
#define HITEC_FRAME_0X12_TIME 1

#define HITEC_FRAME_0X13_GPS_LON 0
#define HITEC_FRAME_0X13_TEMP2 1

#define HITEC_FRAME_0X14_GPS_SPD 0
#define HITEC_FRAME_0X14_GPS_ALT 1
#define HITEC_FRAME_0X14_TEMP1 2

#define HITEC_FRAME_0X15_FUEL 0
#define HITEC_FRAME_0X15_RPM1 1
#define HITEC_FRAME_0X15_RPM2 2

#define HITEC_FRAME_0X16_DATE 0
#define HITEC_FRAME_0X16_TIME 1

#define HITEC_FRAME_0X17_COG 0
#define HITEC_FRAME_0X17_SATS 1
#define HITEC_FRAME_0X17_TEMP3 2
#define HITEC_FRAME_0X17_TEMP4 3

#define HITEC_FRAME_0X18_VOLT 0
#define HITEC_FRAME_0X18_AMP 1

#define HITEC_FRAME_0X19_AMP1 0
#define HITEC_FRAME_0X19_AMP2 1
#define HITEC_FRAME_0X19_AMP3 2
#define HITEC_FRAME_0X19_AMP4 3

#define HITEC_FRAME_0X1A_ASPD 0

#define HITEC_FRAME_0X1B_ALTU 0
#define HITEC_FRAME_0X1B_ALTF 1

void i2c_hitec_handler(i2c_inst_t *i2c, i2c_slave_event_t event);

//run setup I2C port configuration
void setupHitec();

//main Hitec loop
void handleHitec();

void runHitecRequest();

typedef struct sensor_hitec_t
{
    bool is_enabled_frame[11];
    float *frame_0x11[1];
    float *frame_0x12[2];
    float *frame_0x13[2];
    float *frame_0x14[3];
    float *frame_0x15[3];
    float *frame_0x16[2];
    float *frame_0x17[4];
    float *frame_0x18[2];
    float *frame_0x19[4];
    float *frame_0x1A[1];
    float *frame_0x1B[2];
} sensor_hitec_t;


