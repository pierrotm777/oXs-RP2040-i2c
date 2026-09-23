
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/sync.h"  // save_and_disable_interrupts / restore_interrupts
#include <string.h>
#include "tools.h"
#include "config.h"
#include "param.h"
#include "vario.h"
#include "ads1115.h"
#include "rpm.h"
#include "gps.h"
#include "i2c_slave.h"
#include "rlink.h"


extern CONFIG config;
STREAM_DATA streamData;

extern uint8_t debugTlm;
extern field fields[];  // list of all telemetry fields that are measured
uint32_t nowMs = millisRp();

// Deux trames preparees hors interruption, comme Wire.write(buffer, 16).
static uint8_t rlinkFrame1[16];  // 0x89, 0xAB
static uint8_t rlinkFrame2[16];  // 0x89, 0xCD

// Le code Arduino commence par set2(), puis set1().
static bool packetRlinkSet = false;
static bool rlinkReadInProgress = false;
static uint8_t rlinkTxFrame[16]; // copie figee pendant UNE lecture du maitre
static uint8_t rlinkBytePos = 0;
static volatile uint32_t rlinkRequestCount = 0;
static volatile uint32_t rlinkCompleteCount = 0;
static volatile uint32_t rlinkPartialCount = 0;

// handleRlink() et setupRlink() tournent sur le coeur 0 : l'IRQ I2C0
// est installee sur ce meme coeur. Evite une trame partiellement actualisee.
static void publishRlinkFrame(uint8_t destination[16], const uint8_t source[16])
{
    uint32_t irqState = save_and_disable_interrupts();
    memcpy(destination, source, 16);
    restore_interrupts(irqState);
}

void setupRlink()
{
    if (config.pinPrimIn == 255 || config.pinTlm == 255 || config.protocol != 'R') return;

    // PRI = SCL0 (1/5/9/13), TLM = SDA0 (0/4/8/12).
    // Ce test empeche d'activer l'I2C sur une paire de broches erronee.
    if (config.pinPrimIn != config.pinTlm + 1) {
        printf("RadioLink: PRI (SCL0) must be TLM (SDA0) + 1\n");
        return;
    }

    set1();
    set2();
    packetRlinkSet = false;
    rlinkReadInProgress = false;
    rlinkBytePos = 0;

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(config.pinPrimIn, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinPrimIn);
    gpio_set_function(config.pinTlm, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinTlm);

    i2c_slave_init(i2c0, RLINK_I2C_ADDRESS, &i2c_rlink_handler);
    printf("RadioLink: I2C0 slave 0x%02X, SDA=GP%u, SCL=GP%u\n",
           (unsigned int)RLINK_I2C_ADDRESS,
           (unsigned int)config.pinTlm, (unsigned int)config.pinPrimIn);
}

// Appele depuis I2C_SLAVE_REQUEST, donc dans l'interruption I2C.
// UN octet par demande RD_REQ : on n'attend jamais dans l'IRQ.
void runRlinkRequest()
{
    if (!rlinkReadInProgress)
    {
        // Figer les 16 octets pour eviter de melanger deux mises a jour.
        memcpy(rlinkTxFrame, packetRlinkSet ? rlinkFrame1 : rlinkFrame2, 16);
        rlinkBytePos = 0;
        rlinkReadInProgress = true;
        ++rlinkRequestCount;
    }

    // Si le maitre lit plus de 16 octets, repondre sans le bloquer.
    const uint8_t value = (rlinkBytePos < 16) ? rlinkTxFrame[rlinkBytePos++] : 0x00;
    i2c_write_byte_raw(i2c0, value);
}

void i2c_rlink_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch (event)
    {
        case I2C_SLAVE_RECEIVE:
            // Vider les ecritures eventuelles du maitre.
            while (i2c_get_read_available(i2c) > 0)
            {
                (void)i2c_read_byte_raw(i2c);
            }
            break;

        case I2C_SLAVE_REQUEST:
            runRlinkRequest();
            break;

        case I2C_SLAVE_FINISH:
            if (rlinkReadInProgress)
            {
                // Une alternance par transaction, pas par octet.
                if (rlinkBytePos >= 16) ++rlinkCompleteCount;
                else ++rlinkPartialCount;
                packetRlinkSet = !packetRlinkSet;
                rlinkReadInProgress = false;
                rlinkBytePos = 0;
            }
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
        streamData.pitch =  (fields[PITCH].value * 175) / 100  ; //pitch V11
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


    // Preparer les trames hors interruption I2C (ne pas calculer dans le callback).
    set1();
    set2();

    if (debugTlm == 'Y')
    {
        if(millisRp()-nowMs>=2000)
        {
            printf("[RLINK] I2C reads=%lu full=%lu partial=%lu\n",
                   (unsigned long)rlinkRequestCount,
                   (unsigned long)rlinkCompleteCount,
                   (unsigned long)rlinkPartialCount);
            printf("[RLINK] Nb Sats = %u\n", (unsigned int)streamData.gps_sats);
            printf("[RLINK] VSpeed = %d\n", streamData.climb);
            printf("[RLINK] Altitude = %.1f\n", streamData.altitude);
            printf("[RLINK] GSpeed = %.2f\n", streamData.gps_speed);
            printf("[RLINK] Pitch = %.2f\n", streamData.pitch);
            printf("[RLINK] Roll = %.2f\n", streamData.roll);
            printf("[RLINK] Yaw = %.2f\n", streamData.yaw);
            printf("[RLINK] Lon = %.7f\n", streamData.gps_lon / 10000000.0);
            printf("[RLINK] Lat = %.7f\n", streamData.gps_lat / 10000000.0);
            printf("[RLINK] Distance = %.1f\n", streamData.home_distance);
            printf("[RLINK] V1 = %.2f\n\n", streamData.battVoltage);

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
				  
  publishRlinkFrame(rlinkFrame1, bufferRlink);
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
                      longt.b[3], longt.b[2], longt.b[1] , longt.b[0], // longitude d’abord
                      latit.b[3], latit.b[2], latit.b[1] , latit.b[0], // latitude ensuite
                      0x00};//v11

  publishRlinkFrame(rlinkFrame2, bufferRlink);
}
