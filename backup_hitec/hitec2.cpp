
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"

#include "tools.h"
#include "config.h"
#include "param.h"
#include "vario.h"
#include "rpm.h"
#include "gps.h"
#include "tools.h"
#include "i2c_slave.h"
#include "hitec.h"
#include <cstdlib>
#include <cstring>

extern CONFIG config;

extern uint8_t debugTlm;
extern field fields[];  // list of all telemetry fields that are measured
uint32_t nowHitecMs=millisRp();
uint32_t nowHitecUs=microsRp();
int frame = 0;
int reference1ms = 0;
int temp1;
int temp2;

uint32_t startTimer = 0;

uint8_t *packetsHitec[PACKETCOUNT] = {
  (uint8_t *)&packet11,
  (uint8_t *)&packet12,
  (uint8_t *)&packet13,
  (uint8_t *)&packet14,
  (uint8_t *)&packet15,
  (uint8_t *)&packet16,
  (uint8_t *)&packet17,
  (uint8_t *)&packet18
};

void setupHitec(){
 
    if ( config.pinPrimIn == 255 || config.pinTlm == 255 || config.protocol != 'T') return; // skip if pins are not defined and if protocol is not Hitec  

    if(millisRp()-nowHitecMs>=2000)
    {
        printf("Hitec wait 2s for start:\n");
        // setup of packets to be sent to optima
        for (int iPacket = 0; iPacket < PACKETCOUNT; iPacket++)
        {
            memset(packetsHitec[iPacket], 0, 7);
            packetsHitec[iPacket][0] = 0x11 + iPacket;
            packetsHitec[iPacket][6] = 0x11 + iPacket;
        }
        packet11.header = 0xAF;
        packet11.frametype = 0x2D;

        i2c_init( i2c0, 400 * 1000);
        gpio_set_function(config.pinPrimIn, GPIO_FUNC_I2C);
        gpio_pull_up(config.pinPrimIn);
        gpio_set_function(config.pinTlm, GPIO_FUNC_I2C);
        gpio_pull_up(config.pinTlm);
        
        i2c_slave_init(i2c0, HITEC_I2C_ADDRESS, &i2c_hitec_handler);
        //nowHitecMs=millisRp(); /* Restart the Chrono for the printf */
    }
}

void i2c_hitec_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {

    switch (event) 
    {
        case I2C_SLAVE_RECEIVE: // master has written some data
            //not used here
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
			runHitecRequest();
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            //not used here
            break;
        default:
            break;
    }

}

void handleHitec()// run into main.cpp
{

    // detects the ESC's tick timeout
    // when it occurs, the pulse train must be restarted
    if (startTimer && (microsRp() - startTimer > TIMEOUT_DATA))
    {
        frame = (frame >= FRAME_MAX) ? 0 : frame + 1;
        startTimer = 0;

    }


    // update all fields here
    if (fields[VSPEED].available)
    {
        //streamData.climb = int_round(fields[VSPEED].value , 10); // from cm/sec to 0.1m/sec

    }
    if (fields[RELATIVEALT].available)
    {
        // *sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU] = int_round(fields[RELATIVEALT].value ,100) ; // from cm to m
        // sensor->is_enabled_frame[HITEC_FRAME_0X1B] = true;
    }

    if (fields[NUMSAT].available)
    {
        if (fields[LATITUDE].available) 
        {
            // //streamData.gps_lat = fields[LATITUDE].value; // degree / 10,000,000
            // *sensor->frame_0x12[HITEC_FRAME_0X12_GPS_LAT] = fields[LATITUDE].value;
            
        }       
        if (fields[LONGITUDE].available) 
        {
            // //streamData.gps_lon = fields[LONGITUDE].value; // degree / 10,000,000
            // *sensor->frame_0x13[HITEC_FRAME_0X13_GPS_LON] = fields[LONGITUDE].value;
        }
        if (fields[HEADING].available) // not used
        {
            //(uint16_t) (int_round(fields[HEADING].value , 10)); // from 0.01 deg to 0.1 deg
        }
        if (fields[GROUNDSPEED].available) 
        {
            //streamData.gps_speed = fields[GROUNDSPEED].value; // m/s
            // *sensor->frame_0x14[HITEC_FRAME_0X14_GPS_SPD] = fields[GROUNDSPEED].value; // m/s
            
        }
        if (fields[NUMSAT].available) 
        {
            // //streamData.gps_sats = fields[NUMSAT].value;
            // *sensor->frame_0x17[HITEC_FRAME_0X17_SATS] = fields[NUMSAT].value;
        }
        // sensor->is_enabled_frame[HITEC_FRAME_0X17] = true;
        // sensor->is_enabled_frame[HITEC_FRAME_0X12] = true;
        // sensor->is_enabled_frame[HITEC_FRAME_0X13] = true;
        // sensor->is_enabled_frame[HITEC_FRAME_0X14] = true;
        // sensor->is_enabled_frame[HITEC_FRAME_0X16] = true;               
    }
    if (fields[AIRSPEED].available) 
    {
        if (fields[AIRSPEED].value >= 0) 
        {
            // *sensor->frame_0x1A[HITEC_FRAME_0X1A_ASPD] = (uint16_t) int_round(fields[AIRSPEED].value * 36, 1000); //       from cm/sec to 1 km/h
            // sensor->is_enabled_frame[HITEC_FRAME_0X1A] = true;
        }
    }
    // RPM not used
    if (fields[MVOLT].available) 
    {
        //streamData.battVoltage = int_round(fields[MVOLT].value , 100) ; // from mvolt to 0.1V
        // *sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] = (uint16_t) (int_round( fields[MVOLT].value ,  10));  // Volts, 0.01V increments
        // sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;
    }
    if (fields[CURRENT].available) 
    {
        //streamData.battVoltage = int_round(fields[MVOLT].value , 100) ; // from mvolt to 0.1V
        // *sensor->frame_0x18[HITEC_FRAME_0X18_AMP] = (uint16_t) (int_round( fields[CURRENT].value ,  10));  //// Instantaneous current, 0.01A (0-655.34A)		7FFF-> no data  
        // sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;
    }
    if (fields[TEMP1].available) 
    {
        // *sensor->frame_0x17[HITEC_FRAME_0X17_TEMP3] = (uint16_t) fields[TEMP1].value * 10; // from degree to 0.1 degree
        // sensor->is_enabled_frame[HITEC_FRAME_0X17] = true;
    }

    if (fields[YAW].available) 
    {
        //streamData.yaw = (int) fields[YAW].value;
    }
    if (fields[PITCH].available) 
    {
        //streamData.pitch = (int) fields[PITCH].value;
    }
    if (fields[ROLL].available) 
    {
        //streamData.roll = (int) fields[ROLL].value;
    }
    if (fields[GPS_HOME_DISTANCE].available) 
    {
        //streamData.home_distance = fields[GPS_HOME_DISTANCE].value ; // from  m  to 0.1km
    }
    //if (fields[GPS_CUMUL_DIST].available) 
    //{
    //    fields[GPS_CUMUL_DIST].value = streamData.home_distance;
    //}


    if (debugTlm == 'Y')
    {
        if(millisRp()-nowHitecMs>=2000)
        {
          /*
            printf("Hitec Structure:\n");//https://koor.fr/C/cstdio/fprintf.wp
            printf("Nb Sats = %d\n" , (uint8_t)streamData.gps_sats);
            printf("VSpeed = %lu\n" , (ulong)streamData.climb);
            printf("Altitude = %.1f\n" , (float)streamData.altitude);
            printf("GSpeed = %lu\n" , (ulong)streamData.gps_speed);
            printf("Pitch = %.2f\n" , (float)streamData.pitch);
            printf("Roll = %.2f\n" , (float)streamData.roll);
            printf("Yaw = %.2f\n" , (float)streamData.yaw);
            printf("Lon = %.7f\n" , (float)streamData.gps_lon/10000000);
            printf("Lat = %.7f\n" , (float)streamData.gps_lat/10000000);
            printf("Distance = %lu\n" , (float)streamData.home_distance);
            printf("V1 = %.2f\n\n" , (float)streamData.battVoltage);
          */
            nowHitecMs=millisRp(); /* Restart the Chrono for the printf */
        }
    }    
 
}

void runHitecRequest()
{
    // Optima has requested a new data packet
    static int iPacket = 0; 
    //static char watchdog = 0;
    i2c_write_raw_blocking(i2c0, packetsHitec[iPacket++], 7);
    iPacket &= 7;
    //i2c_write_blocking(i2c0, HITEC_I2C_ADDRESS, bufferHitec, 7, false);    
}  

/*
void runHitecRequest()
{

    static uint8_t frameHitec = 0;
    int32_t valueS32;
    uint16_t valueU16;
    uint16_t valueS16;
    uint8_t valueU8;
    uint8_t bufferHitec[7] = {0};
    do
    {
        frameHitec++;
        frameHitec %= 11;
    } while (!sensor->is_enabled_frame[frameHitec]);
    bufferHitec[0] = frameHitec + 0x11;
    bufferHitec[6] = frameHitec + 0x11;
    switch (frameHitec)
    {
    case HITEC_FRAME_0X11:
        bufferHitec[1] = 0xAF;
        bufferHitec[3] = 0x2D;
        if (sensor->frame_0x11[HITEC_FRAME_0X11_RX_BATT])
        {
            valueU16 = *sensor->frame_0x11[HITEC_FRAME_0X11_RX_BATT] * 28;
            bufferHitec[4] = valueU16 >> 8;
            bufferHitec[5] = valueU16;
        }
        break;
    case HITEC_FRAME_0X12:
        if (sensor->frame_0x12[HITEC_FRAME_0X12_GPS_LAT])
        {
            float degF = *sensor->frame_0x12[HITEC_FRAME_0X12_GPS_LAT] / 60;
            int8_t deg = degF;
            int8_t min = (degF - deg) * 60;
            float sec = ((degF - deg) * 60 - min) * 60;
            int16_t sec_x_100 = sec * 100;
            int16_t deg_min = deg * 100 + min;
            bufferHitec[1] = sec_x_100 >> 8;
            bufferHitec[2] = sec_x_100;
            bufferHitec[3] = deg_min >> 8;
            bufferHitec[4] = deg_min;
        }
        if (sensor->frame_0x12[HITEC_FRAME_0X12_TIME])
        {
            valueU8 = *sensor->frame_0x12[HITEC_FRAME_0X12_TIME];
        }
        break;
    case HITEC_FRAME_0X13:
        if (sensor->frame_0x13[HITEC_FRAME_0X13_GPS_LON])
        {
            float degF = *sensor->frame_0x13[HITEC_FRAME_0X13_GPS_LON] / 60;
            int8_t deg = degF;
            int8_t min = (degF - deg) * 60;
            float sec = ((degF - deg) * 60 - min) * 60;
            int16_t sec_x_100 = sec * 100;
            int16_t deg_min = deg * 100 + min;
            bufferHitec[1] = sec_x_100 >> 8;
            bufferHitec[2] = sec_x_100;
            bufferHitec[3] = deg_min >> 8;
            bufferHitec[4] = deg_min;
        }
        if (sensor->frame_0x13[HITEC_FRAME_0X13_TEMP2])
        {
            //valueU8 = round(*sensor->frame_0x13[HITEC_FRAME_0X13_TEMP2] + 40);
            valueU8 = int_round(*sensor->frame_0x13[HITEC_FRAME_0X13_TEMP2] + 40,1);
            bufferHitec[5] = valueU8;
        }
        break;
    case HITEC_FRAME_0X14:
        if (sensor->frame_0x14[HITEC_FRAME_0X14_GPS_SPD])
        {
            //valueU16 = round(*sensor->frame_0x14[HITEC_FRAME_0X14_GPS_SPD] * 1.852);
            valueU16 = int_round(*sensor->frame_0x14[HITEC_FRAME_0X14_GPS_SPD] * 1.852,1);
            bufferHitec[1] = valueU16 >> 8;
            bufferHitec[2] = valueU16;
        }
        if (sensor->frame_0x14[HITEC_FRAME_0X14_GPS_ALT])
        {
            //valueS16 = round(*sensor->frame_0x14[HITEC_FRAME_0X14_GPS_ALT]);
            valueS16 = int_round(*sensor->frame_0x14[HITEC_FRAME_0X14_GPS_ALT],1);
            bufferHitec[3] = valueS16 >> 8;
            bufferHitec[4] = valueS16;
        }
        if (sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1])
        {
            //valueU8 = round(*sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] + 40);
            valueU8 = int_round(*sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] + 40,1);
            bufferHitec[5] = valueU8;
        }
        break;
    case HITEC_FRAME_0X15:
        if (sensor->frame_0x15[HITEC_FRAME_0X15_RPM1])
        {
            //valueU16 = round(*sensor->frame_0x15[HITEC_FRAME_0X15_RPM1]);
            valueU16 = int_round(*sensor->frame_0x15[HITEC_FRAME_0X15_RPM1],1);
            bufferHitec[2] = valueU16;
            bufferHitec[3] = valueU16 >> 8;
        }
        if (sensor->frame_0x15[HITEC_FRAME_0X15_RPM2])
        {
            //valueU16 = round(*sensor->frame_0x15[HITEC_FRAME_0X15_RPM2]);
            valueU16 = int_round(*sensor->frame_0x15[HITEC_FRAME_0X15_RPM2],1);
            bufferHitec[4] = valueU16;
            bufferHitec[5] = valueU16 >> 8;
        }
        break;
    case HITEC_FRAME_0X16:
        if (sensor->frame_0x16[HITEC_FRAME_0X16_DATE])
        {
            valueS32 = *sensor->frame_0x16[HITEC_FRAME_0X16_DATE];
            bufferHitec[3] = valueS32 / 10000;                                 // year
            bufferHitec[2] = (valueS32 - bufferHitec[3] * 10000UL) / 100;           // month
            bufferHitec[1] = valueS32 - bufferHitec[3] * 10000UL - bufferHitec[2] * 100; // day
        }
        if (sensor->frame_0x16[HITEC_FRAME_0X16_TIME])
        {
            valueS32 = *sensor->frame_0x16[HITEC_FRAME_0X16_TIME];
            bufferHitec[4] = valueS32 / 10000;                       // hour
            bufferHitec[5] = (valueS32 - bufferHitec[4] * 10000UL) / 100; // minute
        }
        break;
    case HITEC_FRAME_0X17:
        if (sensor->frame_0x17[HITEC_FRAME_0X17_COG])
        {
            //valueU16 = round(*sensor->frame_0x17[HITEC_FRAME_0X17_COG]);
            valueU16 = int_round(*sensor->frame_0x17[HITEC_FRAME_0X17_COG],1);
            bufferHitec[1] = valueU16 >> 8;
            bufferHitec[2] = valueU16;
        }
        if (sensor->frame_0x17[HITEC_FRAME_0X17_SATS])
        {
            valueU8 = *sensor->frame_0x17[HITEC_FRAME_0X17_SATS];
            bufferHitec[3] = valueU8;
        }
        if (sensor->frame_0x17[HITEC_FRAME_0X17_TEMP3])
        {
            //valueU8 = round(*sensor->frame_0x17[HITEC_FRAME_0X17_TEMP3] + 40);
            valueU8 = int_round(*sensor->frame_0x17[HITEC_FRAME_0X17_TEMP3] + 40,1);
            bufferHitec[4] = valueU8;
        }
        if (sensor->frame_0x17[HITEC_FRAME_0X17_TEMP4])
        {
            //valueU8 = round(*sensor->frame_0x17[HITEC_FRAME_0X17_TEMP4] + 40);
            valueU8 = int_round(*sensor->frame_0x17[HITEC_FRAME_0X17_TEMP4] + 40,1);
            bufferHitec[5] = valueU8;
        }
        break;
    case HITEC_FRAME_0X18:
        if (sensor->frame_0x18[HITEC_FRAME_0X18_VOLT])
        {
            //valueU16 = round((*sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] - 0.2) * 10);
            valueU16 = int_round((*sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] - 0.2) * 10,1);
            bufferHitec[1] = valueU16;
            bufferHitec[2] = valueU16 >> 8;
        }
        if (sensor->frame_0x18[HITEC_FRAME_0X18_AMP])
        {
            // value for stock transmitter (tbc) 
            valueU16 = (*sensor->frame_0x18[HITEC_FRAME_0X18_AMP] + 114.875) * 1.441;

            // value for opentx transmitter  
            //valueU16 = round(*sensor->frame_0x18[HITEC_FRAME_0X18_AMP]);
            valueU16 = int_round(*sensor->frame_0x18[HITEC_FRAME_0X18_AMP],1);

            bufferHitec[3] = valueU16;
            bufferHitec[4] = valueU16 >> 8;
        }
        break;
    case HITEC_FRAME_0X19:
        if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP1])
        {
            //valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP1] * 10);
            valueU8 = int_round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP1] * 10,1);
            bufferHitec[5] = valueU8;
        }
        if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP2])
        {
            //valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP2] * 10);
            valueU8 = int_round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP2] * 10,1);
            bufferHitec[5] = valueU8;
        }
        if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP3])
        {
            //valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP3] * 10);
            valueU8 = int_round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP3] * 10,1);
            bufferHitec[5] = valueU8;
        }
        if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP4])
        {
            //valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP4] * 10);
            valueU8 = int_round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP4] * 10,1);
            bufferHitec[5] = valueU8;
        }
        break;
    case HITEC_FRAME_0X1A:
        if (sensor->frame_0x1A[HITEC_FRAME_0X1A_ASPD])
        {
            //valueU16 = round(*sensor->frame_0x1A[HITEC_FRAME_0X1A_ASPD]);
            valueU16 = int_round(*sensor->frame_0x1A[HITEC_FRAME_0X1A_ASPD],1);
            bufferHitec[3] = valueU16 >> 8;
            bufferHitec[4] = valueU16;
        }
        break;
    case HITEC_FRAME_0X1B:
        if (sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU])
        {
            //valueU16 = round(*sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU]);
            valueU16 = int_round(*sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU],1);
            bufferHitec[1] = valueU16 >> 8;
            bufferHitec[2] = valueU16;
        }
        if (sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTF])
        {
            //valueU16 = round(*sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTF]);
            valueU16 = int_round(*sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTF],1);
            bufferHitec[3] = valueU16 >> 8;
            bufferHitec[4] = valueU16;
        }
        break;
    }

    i2c_write_raw_blocking(i2c0, bufferHitec, 7);
    //i2c_write_blocking(i2c0, HITEC_I2C_ADDRESS, bufferHitec, 7, false);
}
*/