#pragma once

// --------- Default parameters -------------
// Many parameters can be edited using a serial monitor without having to compile/reflash the RP2040  
// If you want to make an uf2 flie with specific parameters (and so, avoid having to use the serial monitor commands),
//     you can change the default parameters in this section
// Note: those parameters are used only for a RP2040 that did not yet had been configured (or when it has been completely erased)

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
 #define _pinGpsTx  12
 #define _pinGpsRx  13
 #define _pinPrimIn  9
 #define _pinSecIn  0XFF 
 #define _pinSbusOut  0XFF
 #define _pinTlm  8
 #define _pinVolt_1  26
 #define _pinVolt_2  27
 #define _pinVolt_3  28
 #define _pinVolt_4  29
 #define _pinSda  10
 #define _pinScl  11
 #define _pinRpm  15
 #define _pinLed  16
 #define _protocol  'X' // R = RadioLink, X = Xbys Spektrum, T = Hitec
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
// --------- Reserve for developer. ---------
