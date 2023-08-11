
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"

#include "tools.h"
#include "config.h"
#include "param.h"
#include "vario.h"
#include "ads1115.h"
#include "rpm.h"
#include "gps.h"
#include "tools.h"
#include "i2c_slave.h"
#include "rlink.h"


extern CONFIG config;
STREAM_DATA streamData;
bool packetRlinkSet = false;

extern uint8_t debugTlm;
extern field fields[];  // list of all telemetry fields that are measured
uint32_t nowMs=millisRp();
static void i2c_handler();

void setupRlink(){
 
    if ( config.pinPrimIn == 255 || config.pinTlm == 255 || config.protocol != 'R') return; // skip if pins are not defined and if protocol is not RadioLink  
    i2c_init( i2c0, 400 * 1000);
    
    gpio_set_function(config.pinPrimIn, GPIO_FUNC_I2C);
	gpio_pull_up(config.pinPrimIn);
    gpio_set_function(config.pinTlm, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinTlm);

    i2c_slave_init(i2c0, RLINK_I2C_ADDRESS, &i2c_rlink_handler);
    // i2c_set_slave_mode(i2c0, true, RLINK_I2C_ADDRESS);
    // *I2C0_INTR_MASK = I2C_INTR_MASK_RD_REQ;
    // irq_set_exclusive_handler(I2C0_IRQ, runRlinkRequest);
    // irq_set_enabled(I2C0_IRQ, true);
}

void runRlinkRequest()
{
    if (packetRlinkSet) 
    {
        packetRlinkSet = false;
        set1();
    } 
    else 
    {
        packetRlinkSet = true;
        set2();
    }
}

void i2c_rlink_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) 
    {
        case I2C_SLAVE_RECEIVE: // master has written some data
            //not used here
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            runRlinkRequest();
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            //not used here
            break;
        default:
            break;
    }

}

void handleRlink()
{
    // update all fields here
    if (fields[VSPEED].available)
    {
        streamData.climb = int_round(fields[VSPEED].value , 10); // from cm/sec to 0.1m/sec
    }
    if (fields[RELATIVEALT].available)
    {
        streamData.altitude = int_round(fields[RELATIVEALT].value ,100) ; // from cm to m
    }

    if (fields[NUMSAT].available)
    {
        if (fields[LATITUDE].available) 
        {
            streamData.gps_lat = fields[LATITUDE].value; // degree / 10,000,000
        }       
        if (fields[LONGITUDE].available) 
        {
            streamData.gps_lon = fields[LONGITUDE].value; // degree / 10,000,000
        }
        if (fields[HEADING].available) // not used
        {
            //(uint16_t) (int_round(fields[HEADING].value , 10)); // from 0.01 deg to 0.1 deg
        }
        if (fields[GROUNDSPEED].available) 
        {
            streamData.gps_speed = fields[GROUNDSPEED].value; // m/s
        }
        if (fields[NUMSAT].available) 
        {
                streamData.gps_sats = fields[NUMSAT].value;
        }        
    }
    if (fields[AIRSPEED].available) 
    {
        if (fields[AIRSPEED].value >= 0) 
        {
            //(uint16_t) int_round(fields[AIRSPEED].value * 36, 1000); //       from cm/sec to 1 km/h
        }
    }
    // RPM not used
    if (fields[MVOLT].available) 
    {
        streamData.battVoltage = int_round(fields[MVOLT].value , 100) ; // from mvolt to 0.1V
    }
    // TEMP1 not used
    // CURRENT not used
    // TEMP2 not used
    if (fields[YAW].available) 
    {
        streamData.yaw = (int) fields[YAW].value;
    }
    if (fields[PITCH].available) 
    {
        //streamData.pitch = (int) fields[PITCH].value;
        streamData.roll =  (fields[PITCH].value * 175) / 100  ; //roll V11
    }
    if (fields[ROLL].available) 
    {
        //streamData.roll = (int) fields[ROLL].value;
        streamData.roll =  (fields[ROLL].value * 175) / 100  ; //roll V11
    }
    if (fields[GPS_HOME_DISTANCE].available) 
    {
        streamData.home_distance = fields[GPS_HOME_DISTANCE].value ; // from  m  to 0.1km
    }
    //if (fields[GPS_CUMUL_DIST].available) 
    //{
    //    fields[GPS_CUMUL_DIST].value = streamData.home_distance;
    //}


    if (debugTlm == 'Y')
    {
        if(millisRp()-nowMs>=2000)
        {
          
            printf("RadioLink Structure:\n");//https://koor.fr/C/cstdio/fprintf.wp
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
          
            nowMs=millisRp(); /* Restart the Chrono for the printf */
        }
    }    
 
}  

//https://github.com/cleanflight/cleanflight/issues/1690
void set1() {

  int  alt = streamData.altitude * 10;//V10 m/s
  uint8_t  altHi = alt >> 8;//highByte(alt )  ;
  uint8_t  altLo = alt;//lowByte(alt ) ;

  int  yaw = streamData.yaw*100;//v9
  uint8_t yawHi = yaw >> 8;//highByte(yaw );
  uint8_t yawLo = yaw;//lowByte(yaw );

  int  speed2 = streamData.gps_speed/100;//V9 m/s
  uint8_t speedHi = speed2 >> 8;//highByte(speed2);
  uint8_t speedLo = speed2;//lowByte(speed2);


  int  roll = streamData.roll*10;//v10
  uint8_t rollHi = roll >> 8;//highByte(roll);
  uint8_t rollLo = roll;//lowByte(roll);


  int  pitch = streamData.pitch*10;//v10
  uint8_t pitchHi = pitch >> 8;//highByte(pitch);
  uint8_t pitchLo = pitch;//lowByte(pitch);

  //int distance = calc_dist(streamData.home_lat, streamData.home_lon, gps.location.lat(), gps.location.lng()) * 10;
  int distance = streamData.home_distance * 10; //V10 m
  uint8_t distanceHi = distance >> 8;//highByte(distance);
  uint8_t distanceLo = distance;//lowByte(distance);

  uint8_t bufferRlink[16] = {0x89, 0xAB, 
                      streamData.gps_sats, 
                      altHi, altLo, 
                      yawHi, yawLo, 
                      speedHi, speedLo, 
                      rollHi , rollLo, 
                      pitchHi, pitchLo, 
                      distanceHi, distanceLo, 
                      0x00};//v11
				  
  i2c_write_blocking(i2c0, RLINK_I2C_ADDRESS, bufferRlink, 16, false);
}

void set2() {

  int  rise = streamData.climb * 10;
  uint8_t  riseHi = rise >> 8;//highByte(rise);
  uint8_t  riseLo =  rise;//lowByte(rise);

  // tester float à la place de uint16_t
  uint16_t voltes = streamData.battVoltage * 100.0;//V9
  uint8_t voltesHi = voltes >>8;//highByte(voltes);
  uint8_t voltesLo = voltes;//lowByte(voltes);

  union u32_tag  {
    uint8_t  b[4];
    int32_t ui32;
  } latit, longt;
  longt.ui32 = streamData.gps_lon;//V8
  latit.ui32 = streamData.gps_lat;//V8;
  uint8_t sat = streamData.gps_sats;// satelites count
  uint8_t bufferRlink[16] = {0x89, 0xCD,
                      sat, 
                      riseHi, riseLo, 
                      voltesHi, voltesLo,
                      latit.b[3], latit.b[2], latit.b[1] , latit.b[0],
                      longt.b[3], longt.b[2], longt.b[1] , longt.b[0],
                      0x00};//v11

//   int32_t lat = streamData.gps_lat;
//   uint8_t latHHi = uint8_t((lat >> 24) & 0x000000FF);
//   uint8_t latHi  = uint8_t((lat >> 16) & 0x000000FF);
//   uint8_t latLo  = uint8_t((lat >> 8)  & 0x000000FF);
//   uint8_t latLLi = uint8_t((lat >> 0)  & 0x000000FF);

//   int32_t lon = streamData.gps_lon;
//   uint8_t lonHHi = uint8_t((lon >> 24) & 0x000000FF);
//   uint8_t lonHi  = uint8_t((lon >> 16) & 0x000000FF);
//   uint8_t lonLo  = uint8_t((lon >> 8)  & 0x000000FF);
//   uint8_t lonLLi = uint8_t((lon >> 0)  & 0x000000FF);

//uint8_t bufferRlink[16] = {0x89, 0xCD, streamData.gps_sats, riseHi, riseLo, voltesHi, voltesLo, latHHi, latHi, latLo , latLLi, lonHHi, lonHi, lonLo , lonLLi, 0x00};
  i2c_write_blocking(i2c0, RLINK_I2C_ADDRESS, bufferRlink, 16, false);
}
