#pragma once

#include <stdint.h>
#define VERSION "2.10.10 & I2C"

//#define DEBUG  // force the MCU to wait for some time for the USB connection; still continue if not connected

// Here some additional parameters that can't be changed via the serial terminal 

// -----------  for Sport and Fbus protocols -------------------------------
#define SPORT_DEVICEID    DATA_ID_FAS  // this line defines the physical ID used by sport. 

// default SPORT_SENSOR_ID use by some original frsky sensors
#define DATA_ID_VARIO  0x00  // = sensor 0 used for Vspeed, GPS long and lat as P1
#define DATA_ID_FLVSS  0xA1  //          1 used as P2
#define DATA_ID_FAS    0x22  //          2
#define DATA_ID_GPS    0x83  //          3 used as P3
#define DATA_ID_RPM    0xE4  //          4
#define DATA_ID_ACC    0x67  //          7

//list of all possible 28 device ID codes (in sequence)
// 0x00,0xA1,0x22,0x83,0xE4,0x45,0xC6,0x67,0x48,0xE9,0x6A,0xCB,0xAC,0x0D,0x8E,0x2F, 
// 0xD0,0x71,0xF2,0x53,0x34,0x95,0x16,0xB7,0x98,0x39,0xBA,0x1B



// -------------- for Sport, Fbus (Frsky) and Exbus (Jeti) ---------------------
// The #define below allow to setup the relative priority of telemetry fields.
// Do not delete the #define but change some values if you want.
// Each Value should be in range 3...250 
// Still value 0 means that this field may not be transmitted
//
// A field with a lower value will be transmitted more often than another field with a higher value  
// The values are relative.
//     E.g. if oXs has 3 fields to transmit; one with priority 5 and 2 with priority 50
//        the one with priority 5 will be transmitted about 10 X more often than the 2 others   
// There is no need to set the value to 0 for fields not available (e.g. for GPS fields if GPS pins are undefined)
// Note : currently some fields are never transmitted in exbus protocol (even if they have a valid value here)
//        For more details about the fields, look at the file doc/fields_per_protocol.txt 


#define P_LATITUDE             50
#define P_LONGITUDE            50
#define P_GROUNDSPEED          50
#define P_HEADING              80
#define P_ALTITUDE             80
#define P_NUMSAT              200
#define P_GPS_DATE            200
#define P_GPS_TIME            200
#define P_GPS_PDOP            200
#define P_GPS_HOME_BEARING     80
#define P_GPS_HOME_DISTANCE    80
#define P_MVOLT                80
#define P_CURRENT              80
#define P_RESERVE1             80
#define P_RESERVE2             80
#define P_CAPACITY            200
#define P_TEMP1                80
#define P_TEMP2                80
#define P_VSPEED               10
#define P_RELATIVEALT          20
#define P_PITCH                30
#define P_ROLL                 30
#define P_YAW                 200
#define P_RPM                  80
#define P_ADS_1_1             200
#define P_ADS_1_2             200
#define P_ADS_1_3             200
#define P_ADS_1_4             200
#define P_ADS_2_1             200
#define P_ADS_2_2             200
#define P_ADS_2_3             200
#define P_ADS_2_4             200
#define P_AIRSPEED             30
#define P_AIRSPEED_COMPENSATED_VSPEED 10
#define P_SBUS_HOLD_COUNTER   100
#define P_SBUS_FAILSAFE_COUNTER 100
#define P_GPS_CUMUL_DIST      200  
#define P_ACC_X                80
#define P_ACC_Y                80
#define P_ACC_Z                80


// -------------- for ELRS protocol  ------------------------------
#define VOLTAGE_FRAME_INTERVAL 500 // This version transmit only one voltage; it could be change in the future
#define VARIO_FRAME_INTERVAL 50   // This frame contains only Vertical speed
#define GPS_FRAME_INTERVAL 500     // This frame contains longitude, latitude, altitude, ground speed, heading and number of satellites
#define ATTITUDE_FRAME_INTERVAL 500 // This should normally contains pitch, roll and yaw.
#define BARO_ALTITUDE_FRAME_INTERVAL 500 // This frame contains only barometric relative altitude


// -------------- for Futaba Sbus2 protocol -----------------------
// For temperature, oXs emulates a SBS/01T
// For RPM, oXs emulates a SBS/01RO or 01RM
// For vario, oXs emulates a F1672 or F1712
// For Volt, current,capacity, oXs emulates SBS/01C ; seems to the same as F-1678
// For GPS, oXs emulates a SBS/01G
// V3 and V4 can be get using F125 or F1713 

// here you can define the slot being used by different sensors
#define SBUS2_SLOT_GPS_8  8   // this must be 8,16 or 24 because GPS requires 8 slots
#define SBUS2_SLOT_VARIO_2 16 // Vario requires 2 lots (Alt and Vspeed)
#define SBUS2_SLOT_BATTERY_3 18  // Battery requires 3 slots (volt, current, capacity)
#define SBUS2_SLOT_TEMP1_1 21    // Temp1 requires 1 slot
#define SBUS2_SLOT_TEMP2_1 22    // Temp2 requires 1 slot
#define SBUS2_SLOT_RESERVE1_1 23    // V3 requires 1 slot, emulate F1713
#define SBUS2_SLOT_RESERVE2_1 24    // V4 requires 1 slot, emulate F1713
#define SBUS2_SLOT_RPM_1 25    // Rpm requires 1 slot
#define SBUS2_SLOT_AIRSPEED_1 26    // Airspeed requires ? slot

#define SBUS2_SLOT_HOLD_COUNTER_1 28    // Count the number of hold frames ; require 1 slot; emulate F1713
#define SBUS2_SLOT_FAILSAFE_COUNTER_1 29    // Count the number of failsafeframes ; require 1 slot; emulate F1713

// ----------Parameters for multiplex protocol -----------------
// MPX can have max 16 telemetry fields
// The list below defines the fields that can be transmitted.
// The sequence defines the number (from 0 up to 15) used on the display of the handset
// 0XFF means that no field is transmitted
// You can change the order of the lines but you must keep 16 lines 
#define MPX_SEQUENCE_OF_FIELDS {\
            0XFF,\
            0XFF,\
            CURRENT,\
            VSPEED,\
            GROUNDSPEED,\
            RPM,\
            TEMP1,\
            GPS_HOME_BEARING,\
            RELATIVEALT,\
            MVOLT,\
            0XFF,\
            CAPACITY,\
            GPS_CUMUL_DIST,\
            GPS_HOME_DISTANCE,\
            ALTITUDE,\
            HEADING,\
}

// uncomment next line(s) if you want to get some alarm and then set the alarm value
//#define MPX_ALARM_MVOLT_MIN 1100 // alarm when Mvolt is lower that this value
//#define MPX_ALARM_MA_MAX 20000   // alarm when current (in mA) is higher than this value
//#define MPX_ALARM_MAH_MAX 2000  // alarm when consumed capacity (in mAh) is higher than this value
//#define MPX_ALARM_TEMP1_MAX 100  // alarm when temp1 (in degree) is higher than this value
//#define MPX_ALARM_TEMP2_MAX 100  // alarm when temp2 (in degree) is higher than this value
//#define MPX_ALARM_CM_MAX 12000  // alarm when relative altitude (in cm) is higher than this value


// --------- Parameters to remap the SBUS Rc channel values to PWM values ---------
#define FROM_SBUS_MIN 172  // This is equivalent to -100 on openTx
#define TO_PWM_MIN 988     // this is the PWM usec for -100
#define FROM_SBUS_MAX 1811  // This is equivalent to +100 on openTx
#define TO_PWM_MAX 2012     // this is the PWM usec for +100

// -------- Parameters for the vario -----
#define SENSITIVITY_MIN 100
#define SENSITIVITY_MAX 300
#define SENSITIVITY_MIN_AT 100
#define SENSITIVITY_MAX_AT 1000
#define VARIOHYSTERESIS 5

// --------- Parameters for GPS ---------------
#define GPS_REFRESH_RATE 10 // For Ublox GPS, it is possible to select a refresh rate of 1Hz, 5Hz (defeult) or 10Hz 
//                        note :a casic gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds
//                        this can be done using a FTDI and program GnssToolkit3.exe (to download from internet)

// --------- Parameter for RPM -------------------
#define RPM_COUNTER_INTERVAL_USEC 100000 // in usec (so 100000 = 100 msec)

// --------- Parameters for Ads1115 ----------------
#define I2C_ADS_Add1 0x48 // I2C address of ads1115 when addr pin is connected to ground
#define I2C_ADS_Add2 0x49 // I2C address of ads1115 when addr pin is connected to vdd

#define ADS1_MEASURE A0_TO_GND ,  A1_TO_GND , A2_TO_GND , A3_TO_GND // select 4 values between A0_TO_A1, A0_TO_A3, A1_TO_A3, A2_TO_A3, A0_TO_GND, A1_TO_GND, A2_TO_GND, A3_TO_GND, ADS_OFF
#define ADS1_FULL_SCALE_VOLT  MV4096, MV4096, MV4096, MV4096 //  select between MV6144 MV4096 MV2048 MV1024 MV512 MV256
#define ADS1_OFFSET 0.0, 0.0 , 0.0 , 0.0 // can be a float (positive or negative)
#define ADS1_SCALE 1.0, 1.0, 1.0, 1.0 // can be a float
#define ADS1_RATE  MS5 , MS5, MS5 , MS5 // select between MS137, MS69, MS35, MS18, MS9, MS5, MS3 , MS2
#define ADS1_AVERAGING_ON 10 , 10, 10, 10 // number of values used for averaging (must be between 1 and 254) 

#define ADS2_MEASURE A0_TO_GND ,  A1_TO_GND , A2_TO_GND , A3_TO_GND // select 4 values between A0_TO_A1, A0_TO_A3, A1_TO_A3, A2_TO_A3, A0_TO_GND, A1_TO_GND, A2_TO_GND, A3_TO_GND, ADS_OFF
#define ADS2_FULL_SCALE_VOLT  MV4096, MV4096, MV4096, MV4096 //  select between MV6144 MV4096 MV2048 MV1024 MV512 MV256
#define ADS2_OFFSET 0.0, 0.0 , 0.0 , 0.0 // can be a float (positive or negative)
#define ADS2_SCALE 1.0, 1.0, 1.0, 1.0 // can be a float
#define ADS2_RATE  MS5 , MS5, MS5 , MS5 // select between MS137, MS69, MS35, MS18, MS9, MS5, MS3 , MS2
#define ADS2_AVERAGING_ON 10 , 10, 10, 10 // number of values used for averaging (must be between 1 and 254) 

// --------- Parameters for MS4525 ----------------

#define MS4525_ADDRESS 0X28 // 0x28 is the default I2C adress of a 4525DO sensor ; 0X46 is an alternative

// --------- Parameters for SDP3x ----------------

//#define SDPXX_ADDRESS 0X21 // 0x21 is the default I2C adress of a SDP3X sensor
#define SDPXX_ADDRESS   0x25 // 0x25 is the I2C adress of a SDP810 sensor
//#define USE_ADP810_INSTEAD_OF_SDPxx  // uncoment this line if you use a ADP810 instead of a SDPxx

// --------- Parameter for MPU6050 ---------------------------------------
#define ACC_MAX_SCALE_G 2 // maximum acceleration in G (can only be 2,4,8, 16)
#define GYRO_MAX_SCALE_DEGREE 2000  // max gyro rate : degree per sec
#define CALIBRATE_GYRO_ON_RESET     // uncomment to avoid gyro calibration on reset
// --------- Parameters for Compensated Vspeed by airspeed ----------------
#define DTE_DEFAULT_COMPENSATION_FACTOR 1.10  // used when a channel is not used to setup the factor

// ---------- ESC --------------------------------------------------------
#define ESC_MAX_CURRENT 250000.0
#define ESC_MIN_THROTTLE 256    // used for Hobbywing V4 to reject dummy values ; 1024 = 100%; so e.g. 256 = 25% of max

// -------------- Camera stabilizer ----------------------------------------
// uncomment PITCH_CONTROL_CHANNEL and/or ROLL_CONTROL_CHANNEL if you want to stabilize a camera on those axis)
 
#define PITCH_CONTROL_CHANNEL config.CamPitchChannel // Channel used to control the servo for the camera (pitch); uncomment to activate the pitch stabilization
#define PITCH_RATIO_CHANNEL config.CamPitchRatio   // Channel used to set up the ratio between pitch and servo movement (optional)
#define PITCH_RATIO  100  // Ratio to use when PITCH_RATIO_CHANNEL is undefined (or 255); increase/decrease the value in case of under/over stabilisation  
#define PITCH_MAX 100     // adapt upper limit of servo travel (should normally be the same value as on TX) 
#define PITCH_MIN -100    // adapt lower limit of servo travel (should normally be the same value as on TX)
#define ROLL_CONTROL_CHANNEL config.CamRollChannel// Channel used to control the servo for the camera (roll); uncomment to activate the roll stabilization
#define ROLL_RATIO_CHANNEL config.CamRollRatio    // Channel used to set up the ratio between roll and servo movement
#define ROLL_RATIO  100  // Ratio to use when ROLL_RATIO_CHANNEL is undefined (or 255); increase/decrease the value in case of under/over stabilisation
#define ROLL_MAX 100     // adapt upper limit of servo travel 
#define ROLL_MIN -100    // adapt lower limit of servo travel

//Note:  when a channel is used to adjust a ratio (for pitch or roll), the ratio can varies from -200 (channel = -100%) up to +200 (channel = %100%)
//       the sign of the ratio define the direction of the compensation.
//       Setting the channel on 0% dissables the compensation. This can e.g. be done using a switch on the TX

// --------- Gyro Parameters ---------------
// 



// --------- Default parameters -------------
// Many parameters can be edited using a serial monitor without having to compile/reflash the RP2040  
// If you want to make an uf2 flie with specific parameters (and so, avoid having to use the serial monitor commands),
//     you can change the default parameters in this section
// Note: those parameters are used only for a RP2040 that did not yet had been configured (or when it has been completely erased)

//#include "config_Jeti.h"
//#include "config_Sport.h"
//#include "config_I2C.h"

 #define _pinChannels_1  0XFF
 #define _pinChannels_2  0XFF
 #define _pinChannels_3  0XFF
 #define _pinChannels_4  0XFF
 #define _pinChannels_5  0XFF
 #define _pinChannels_6  0XFF
 #define _pinChannels_7  0XFF
 #define _pinChannels_8  0XFF
 #define _pinChannels_9  0XFF
 #define _pinChannels_10  0XFF
 #define _pinChannels_11  0XFF
 #define _pinChannels_12  0XFF
 #define _pinChannels_13  0XFF
 #define _pinChannels_14  0XFF
 #define _pinChannels_15  0XFF
 #define _pinChannels_16  0XFF
 #define _pinGpsTx  0XFF
 #define _pinGpsRx  0XFF
 #define _pinPrimIn  0XFF
 #define _pinSecIn  0XFF 
 #define _pinSbusOut  0XFF
 #define _pinTlm  0XFF
 #define _pinVolt_1  0XFF
 #define _pinVolt_2  0XFF
 #define _pinVolt_3  0XFF
 #define _pinVolt_4  0XFF
 #define _pinSda  0XFF
 #define _pinScl  0XFF
 #define _pinRpm  0XFF
 #define _pinLed  16
 #define _protocol  'S' // S = Sport, C = crossfire, J = Jeti
 #define _crsfBaudrate  420000
 #define _scaleVolt1  1.0
 #define _scaleVolt2  1.0
 #define _scaleVolt3  1.0
 #define _scaleVolt4  1.0
 #define _offset1  0.0
 #define _offset2  0.0
 #define _offset3  0.0
 #define _offset4  0.0
 #define _gpsType  'U' 
 #define _rpmMultiplicator 1.0
 #define _failsafeType  'H'
//    crsf_channels_s failsafeChannels ;
// #define _accOffsetX;
// #define _accOffsetY;
// #define _accOffsetZ;
// #define _gyroOffsetX;
// #define _gyroOffsetY;
// #define _gyroOffsetZ;
#define _temperature 0XFF
#define _VspeedCompChannel 0XFF
#define _ledInverted 'N'
#define _CamPitchChannel 0xFF
#define _CamRollChannel 0xFF
#define _CamPitchRatio 0xFF
#define _CamRollRatio 0xFF
#define _pinLogger 0xFF
#define _loggerBaudrate 115200
#define _pinEsc 0xFF
#define _escType 0xFF
#define _pwmHz 50  // 50 hz per default

// ------  for gyro   -------
#define _gyroChanControl 0xFF // Rc channel used to say if gyro is implemented or not and to select the mode and the general gain. Value must be in range 1/16 or 255 (no gyro)
#define _gyroChan_AIL    0xFF // Rc channel used to transmit original Ail (Elv, Rud) stick position ; Value must be in range 1/16 when gyroControlChannel is not 255
#define _gyroChan_ELV    0xFF //                                           ELV
#define _gyroChan_RUD    0xFF //                                                RUD
#define _pid_param_rate_KP_AIL 500  // PID rates: normal mode - Kp - ail (roll)
#define _pid_param_rate_KP_ELV 500  // PID rates: normal mode - Kp - elv (pitch)
#define _pid_param_rate_KP_RUD 500  // PID rates: normal mode - Kp - rud (yaw)
#define _pid_param_hold_KP_AIL 500  // PID rates: hold   mode - Kp - ail (roll)
#define _pid_param_hold_KP_ELV 500  // PID rates: hold   mode - Kp - elv (pitch)
#define _pid_param_hold_KP_RUD 500  // PID rates: hold   mode - Kp - rud (yaw)
#define _pid_param_stab_KP_AIL 500  // PID rates: hold   mode - Kp - ail (roll)
#define _pid_param_stab_KP_ELV 500  // PID rates: hold   mode - Kp - elv (pitch)
#define _pid_param_stab_KP_RUD 500  // PID rates: hold   mode - Kp - rud (yaw)

#define _pid_param_rate_KI_AIL   0  // PID rates: normal mode - Ki - ail (roll)   // set ki to 0 for normal mode
#define _pid_param_rate_KI_ELV   0  // PID rates: normal mode - Ki - elv (pitch)  // set ki to 0 for normal mode
#define _pid_param_rate_KI_RUD   0  // PID rates: normal mode - Ki - rud (yaw)    // set ki to 0 for normal mode
#define _pid_param_hold_KI_AIL 500  // PID rates: hold   mode - Ki - ail (roll)
#define _pid_param_hold_KI_ELV 500  // PID rates: hold   mode - Ki - elv (pitch)
#define _pid_param_hold_KI_RUD 500  // PID rates: hold   mode - Ki - rud (yaw)
#define _pid_param_stab_KI_AIL   0  // PID rates: hold   mode - Ki - ail (roll)
#define _pid_param_stab_KI_ELV   0  // PID rates: hold   mode - Ki - elv (pitch)
#define _pid_param_stab_KI_RUD   0  // PID rates: hold   mode - Ki - rud (yaw)

#define _pid_param_rate_KD_AIL 500  // PID rates: normal mode - Kd - ail (roll)
#define _pid_param_rate_KD_ELV 500  // PID rates: normal mode - Kd - elv (pitch)
#define _pid_param_rate_KD_RUD 500  // PID rates: normal mode - Kd - rud (yaw)
#define _pid_param_hold_KD_AIL 500  // PID rates: hold   mode - Kd - ail (roll)
#define _pid_param_hold_KD_ELV 500  // PID rates: hold   mode - Kd - elv (pitch)
#define _pid_param_hold_KD_RUD 500  // PID rates: hold   mode - Kd - rud (yaw)
#define _pid_param_stab_KD_AIL 500  // PID rates: hold   mode - Kd - ail (roll)
#define _pid_param_stab_KD_ELV 500  // PID rates: hold   mode - Kd - elv (pitch)
#define _pid_param_stab_KD_RUD 500  // PID rates: hold   mode - Kd - rud (yaw)

#define _pid_param_rate_output_shift 8 // do not modify
#define _pid_param_hold_output_shift 8 // do not modify
#define _pid_param_stab_output_shift 8 // do not modify

#define _vr_gain_AIL       127      // store the gain per axis (to combine with global gain provided by gyroChanControl) (0/127 or 0/-127 to reverse )
#define _vr_gain_ELV       127      // store the gain per axis (to combine with global gain provided by gyroChanControl) (0/127 or 0/-127 to reverse )
#define _vr_gain_RUD       127      // store the gain per axis (to combine with global gain provided by gyroChanControl) (0/127 or 0/-127 to reverse ) 
#define _stick_gain_throw     1     // STICK_GAIN_THROW_FULL=1, STICK_GAIN_THROW_HALF=2, STICK_GAIN_THROW_QUARTER=3} 
                                    // STICK_GAIN_THROW allows to limit the compensation on a part of the stick travel (gain decreases more or less rapidly with stick offset)
#define _max_rotate           3     // MAX_ROTATE_VLOW=1, MAX_ROTATE_LOW=2, MAX_ROTATE_MED=3, MAX_ROTATE_HIGH=4
                                    // MAX_ROTATE is used in hold mode (correction depends more or less on stick offset)
#define _rate_mode_stick_rotate  1  // RATE_MODE_STICK_ROTATE_DISABLE=1, RATE_MODE_STICK_ROTATE_ENABLE=2
                                    // MAX_ROTATE is used also in rate mode when RATE_MODE_STICK_ROTATE_ENABLE is selected
#define _gyroAutolevel true         // true means that stabilize mode replace hold mode
#define _mpuOrientationH 4           // upper face when plane is horizontal, last 4 bits= axis being up when nose is up; default is 4 (Z axis point up)
#define _mpuOrientationV 0           // upper face when plane is vertical (nose up) ; default is 0 (X axis point to the nose)
                                        // for both  0=X+, 1=X- , 2=Y+ , 3=Y- , 4=Z+, 5=Z- , 6=error ;																									  
// --------- Reserve for developer. ---------


typedef struct {
  int32_t value ;
  uint8_t available ;
} oneMeasurement_t;

//Note:
// I activate this when I buid a device for Sport with voltage measured on VOLT2 instead of VOLT1 (because it is easier to solder the resistor to Grnd)
//#define SKIP_VOLT1_3_4


