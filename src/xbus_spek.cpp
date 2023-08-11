
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

void handleXbusSpektrum()
{
    // update all fields here
    if (fields[VSPEED].available)
    {
        //streamData.climb = int_round(fields[VSPEED].value , 10); // from cm/sec to 0.1m/sec
    }
    if (fields[RELATIVEALT].available)
    {
        //streamData.altitude = int_round(fields[RELATIVEALT].value ,100) ; // from cm to m
        TmBuffer.varioSimple.altitude = fields[RELATIVEALT].value;
    }

    if (fields[NUMSAT].available)
    {
        if (fields[LATITUDE].available) 
        {
            TmBuffer.gpsbin.latitude = fields[LATITUDE].value;
        }       
        if (fields[LONGITUDE].available) 
        {
            TmBuffer.gpsbin.longitude = fields[LONGITUDE].value;
        }
        if (fields[HEADING].available)
        {
            TmBuffer.attMag.heading = fields[HEADING].value;
        }
        if (fields[GROUNDSPEED].available) 
        {
            TmBuffer.gpsbin.groundSpeed = fields[GROUNDSPEED].value;
        }
        if (fields[NUMSAT].available) 
        {
            TmBuffer.gpsbin.numSats = fields[NUMSAT].value;
        }
        if (fields[GPS_TIME].available) 
        {
            TmBuffer.gpsstat.UTC = fields[GPS_TIME].value;
        }        
    }
    if (fields[AIRSPEED].available) 
    {
        if (fields[AIRSPEED].value >= 0) 
        {
            //(uint16_t) int_round(fields[AIRSPEED].value * 36, 1000); //       from cm/sec to 1 km/h
             TmBuffer.speed.airspeed = fields[AIRSPEED].value;
        }
    }
    // RPM not used
    if (fields[MVOLT].available) 
    {
        TmBuffer.escSPM.voltsInput = fields[MVOLT].value;

    }
    if (fields[CURRENT].available) 
    {
         TmBuffer.escSPM.currentMotor = fields[CURRENT].value;
    }
    // TEMP1 not used
    // TEMP2 not used
    if (fields[YAW].available) 
    {
        TmBuffer.attMag.attYaw = fields[YAW].value;
    }
    if (fields[PITCH].available) 
    {
        TmBuffer.attMag.attPitch = fields[PITCH].value;
    }
    if (fields[ROLL].available) 
    {
        TmBuffer.attMag.attRoll = fields[ROLL].value;
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

            nowSpekMs=millisRp(); /* Restart the Chrono for the printf */
        }
    }    
 
}  

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