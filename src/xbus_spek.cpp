
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
#include "xbus_spek.h"
#include <math.h>


extern CONFIG config;
// Globals
double prevSpekVoltage = 0.0;
XBUS_UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, 
                                NO_DATA, NO_DATA,
                                NO_DATA, NO_DATA, 
                                NO_DATA, NO_DATA, 
                                NO_DATA, NO_DATA, 
                                NO_DATA, NO_DATA, 
                                NO_DATA, NO_DATA, 
                                NO_DATA, NO_DATA 
                                };

extern uint8_t debugTlm;
extern field fields[];  // list of all telemetry fields that are measured
uint32_t nowSpekMs=millisRp();

void setupXbusSpektrum(){
 
    if ( config.pinPrimIn == 255 || config.pinTlm == 255 || config.protocol != 'X') return; // skip if pins are not defined and if protocol is not Spektrum Xbus  
    i2c_init( i2c0, 400 * 1000);
    gpio_set_function(config.pinPrimIn, GPIO_FUNC_I2C);
	gpio_pull_up(config.pinPrimIn);
    gpio_set_function(config.pinTlm, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinTlm);

    addressMask |= TELE_DEVICE_AIRSPEED;//0x11
    addressMask |= TELE_DEVICE_ALTITUDE;//0x12
    addressMask |= TELE_DEVICE_GPS_LOC;//0x16
    addressMask |= TELE_DEVICE_GPS_STATS;//0x17
    addressMask |= TELE_DEVICE_ESC;//0x20
    addressMask |= TELE_DEVICE_FP_MAH;//0x34
    addressMask |= TELE_DEVICE_RPM;//0x7E

    IDENTIFIER = addressMask;

	i2c_slave_init(i2c0, IDENTIFIER, &i2c_spek_handler);

}

void i2c_spek_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {

    switch (event) 
    {
        case I2C_SLAVE_RECEIVE: // master has written some data
            //not used here
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            i2c_write_raw_blocking(i2c0,TmBuffer.raw,sizeof(TmBuffer));// à valider
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            //not used here
            break;
        default:
            break;
    }

}

void handleXbusSpektrum(uint8_t address)
{
    static float alt = 0;
    switch (address)
    {
        case TELE_DEVICE_GPS_LOC:
        {
            uint8_t gps_flags = 0;
            float lat = fields[LATITUDE].value;
            if (lat < 0) // N=1,+, S=0,-
                lat *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_NORTH_BIT;
            TmBuffer.gpsloc.latitude = bcd32((uint16_t)(lat / 60) * 100 + fmod(lat, 60), 4);

            float lon = fields[LONGITUDE].value;
            if (lon < 0) // E=1,+, W=0,-
                lon *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_EAST_BIT;
            if (lon >= 6000)
            {
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
                lon -= 6000;
            }
            // sensor_formatted->gps_loc->longitude = bcd32((uint16_t)(lon / 60) * 100 + fmod(lon, 60), 4);
            // sensor_formatted->gps_loc->course = bcd16(*sensor->gps_loc[XBUS_GPS_LOC_COURSE], 1);
            // sensor_formatted->gps_loc->hdop = bcd8(*sensor->gps_loc[XBUS_GPS_LOC_HDOP], 1);
            TmBuffer.gpsloc.longitude = bcd32((uint16_t)(lon / 60) * 100 + fmod(lon, 60), 4);
            TmBuffer.gpsloc.course = bcd16(fields[GPS_CUMUL_DIST].value, 1);
            TmBuffer.gpsloc.HDOP = bcd8(fields[GPS_PDOP].value, 1);
            break;
        }
        case TELE_DEVICE_GPS_STATS:
        {
            // sensor_formatted->gps_stat->speed = bcd16(*sensor->gps_stat[XBUS_GPS_STAT_SPEED], 1);
            // sensor_formatted->gps_stat->utc = bcd32(*sensor->gps_stat[XBUS_GPS_STAT_TIME], 2);
            // sensor_formatted->gps_stat->num_sats = bcd8(*sensor->gps_stat[XBUS_GPS_STAT_SATS], 0);
            // sensor_formatted->gps_stat->altitude_high = bcd8((uint8_t)(alt / 1000), 0);
            TmBuffer.gpsstat.speed = bcd16(fields[GROUNDSPEED].value, 1);
            TmBuffer.gpsstat.UTC = bcd32(fields[GPS_TIME].value, 2);
            TmBuffer.gpsstat.numSats = bcd8(fields[NUMSAT].value, 0);
            TmBuffer.gpsstat.altitudeHigh = bcd8((uint8_t)(fields[NUMSAT].value / 1000), 0);
            break;    
        }
    }
}

/*
void handleXbusSpektrum()
{
    uint16_t tempU16;
    int16_t tempI16;
    int32_t tempI32;
    // update all fields here
    if (fields[VSPEED].available)
    {
        //streamData.climb = int_round(fields[VSPEED].value , 10); // from cm/sec to 0.1m/sec
    }
    if (fields[RELATIVEALT].available)
    {
        //streamData.altitude = int_round(fields[RELATIVEALT].value ,100) ; // from cm to m
        //TmBuffer.varioSimple.altitude = fields[RELATIVEALT].value;
    }

    if (fields[NUMSAT].available)
    {
        if (fields[LATITUDE].available) 
        {
            //TmBuffer.gpsloc.latitude = bcd32((uint16_t)(fields[LATITUDE].value / 60) * 100 + fmod(fields[LATITUDE].value, 60), 4);
            //TmBuffer.gpsloc.latitude = fields[LATITUDE].value;
            tempI32 = fields[LATITUDE].value; // degree / 10,000,000
            TmBuffer.gpsloc.latitude = swapBinary(tempI32);
        }       
        if (fields[LONGITUDE].available) 
        {
            //TmBuffer.gpsloc.latitude = bcd32((uint16_t)(fields[LONGITUDE].value / 60) * 100 + fmod(fields[LONGITUDE].value, 60), 4);
            tempI32 = fields[LONGITUDE].value; // degree / 10,000,000
            TmBuffer.gpsloc.longitude = swapBinary(tempI32);
        }
        if (fields[ALTITUDE].available)
        {
            //TmBuffer.gpsloc.altitudeLow = bcd16(fmod(fields[ALTITUDE].value, 1000), 1);
            //TmBuffer.gpsloc.altitudeLow = fields[ALTITUDE].value;
        }
        if (fields[HEADING].available)
        {
            //TmBuffer.attMag.heading = fields[HEADING].value;
        }
        if (fields[GROUNDSPEED].available) 
        {
            //TmBuffer.gpsbin.groundSpeed = bcd16(fields[GROUNDSPEED].value, 1);
        }
        if (fields[NUMSAT].available) 
        {
            //TmBuffer.gpsbin.numSats = fields[NUMSAT].value;
            //TmBuffer.gpsstat.numSats = bcd8(fields[NUMSAT].value, 0);
        }
        if (fields[GPS_TIME].available) 
        {
            //TmBuffer.gpsstat.UTC = fields[GPS_TIME].value;
        }        
    }
    if (fields[AIRSPEED].available) 
    {
        if (fields[AIRSPEED].value >= 0) 
        {
             //TmBuffer.speed.airspeed = __builtin_bswap16(fields[AIRSPEED].value);
        }
    }
    // RPM not used
    if (fields[MVOLT].available) 
    {
        //TmBuffer.escSPM.voltsInput = __builtin_bswap16(fields[MVOLT].value * 100);
    }
    if (fields[CURRENT].available) 
    {
         //TmBuffer.escSPM.currentMotor = __builtin_bswap16(fields[CURRENT].value * 10);
    }
    // TEMP1 not used
    // TEMP2 not used
    if (fields[YAW].available) 
    {
        //TmBuffer.attMag.attYaw = fields[YAW].value;
    }
    if (fields[PITCH].available) 
    {
        //TmBuffer.attMag.attPitch = fields[PITCH].value;
    }
    if (fields[ROLL].available) 
    {
        //TmBuffer.attMag.attRoll = fields[ROLL].value;
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
        if(millisRp()-nowSpekMs>=2000)
        {
          
            printf("RadioLink Structure:\n");//https://koor.fr/C/cstdio/fprintf.wp
            printf("Nb Sats = %d\n" , (uint8_t)TmBuffer.gpsbin.numSats);
            //printf("VSpeed = %lu\n" , (ulong)streamData.climb);
            //printf("Altitude = %.1f\n" , (float)streamData.altitude);
            printf("GSpeed = %lu\n" , (ulong)TmBuffer.gpsbin.groundSpeed);
            printf("Pitch = %.2f\n" , (float)TmBuffer.attMag.attPitch);
            printf("Roll = %.2f\n" , (float)TmBuffer.attMag.attRoll);
            printf("Yaw = %.2f\n" , (float)TmBuffer.attMag.attYaw);
            printf("Lon = %.7f\n" , (float)TmBuffer.gpsbin.longitude/10000000);
            printf("Lat = %.7f\n" , (float)TmBuffer.gpsbin.latitude/10000000);
            //printf("Distance = %lu\n" , (float)streamData.home_distance);
            //printf("V1 = %.2f\n\n" , (float)streamData.battVoltage);

            nowSpekMs=millisRp(); // Restart the Chrono for the printf 
        }
    }    
 
}  
*/

unsigned int SwapEndian(unsigned int i)
{
  endianBuff_u b;
  unsigned char temp;

  b.value = i;
  //Swap bytes
  temp     = b.raw[0];
  b.raw[0] = b.raw[1];
  b.raw[1] = temp;

  return(b.value);
}

uint8_t bcd8(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint8_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%02i", (uint8_t)value);
    for (int i = 0; i < 2; i++)
        output |= (buf[i] - 48) << ((1 - i) * 4);
    return output;
}

uint16_t bcd16(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint16_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%04i", (uint16_t)value);
    for (int i = 0; i < 4; i++)
        output |= (uint16_t)(buf[i] - 48) << ((3 - i) * 4);
    return output;
}

uint32_t bcd32(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint32_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%08li", (uint32_t)value);
    for (int i = 0; i < 8; i++)
        output |= (uint32_t)(buf[i] - 48) << ((7 - i) * 4);
    return output;
}