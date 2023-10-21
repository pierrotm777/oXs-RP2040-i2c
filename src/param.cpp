
#include "param.h"
#include "pico/stdlib.h"
#include "config.h"
#include "stdio.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "ads1115.h"
#include "ms4525.h"
#include "sdp3x.h"
#include <string.h>
#include <ctype.h>
#include "gps.h"
#include "hardware/flash.h"
#include <inttypes.h>
#include "stdlib.h"
#include  "hardware/sync.h"
#include "hardware/watchdog.h"
#include "crsf_out.h"
#include "pico/multicore.h"
#include "mpu.h"
#include "pico/util/queue.h"
#include "tools.h"
#include "jeti.h"
#include "exbus.h"
#include "hardware/pio.h"  // needed for sbus_out_pwm.h
#include "sbus_out_pwm.h"  // needed to print the PWM values
#include "sequencer.h"
#include <errno.h>   // used by strtol() to check for errors 
// commands could be in following form:
// C1 = 0/15  ... C16 = 0/15
// GPS_TX = 0/29
// GPS_RX = 0/29
// PRI = 5, 9, 21, 25  (UART1)
// SEC = 1, 13, 17,29 (UART0) 
// SBUS_OUT = 0/29
// TLM = 0/29
// VOLT1= 26/29 ... VOLT4 = 26/29
// SDA = 2, 6, 10, 14, 18, 22, 26  (I2C1)
// SCL = 3, 7, 11, 15, 19, 23, 27  (I2C1)
// RPM = 0/29 
// LED = 16
// PROTOCOL = C, S, J , M , I , F, 2, L, E
// for RP2040_zero, pin 16 = LED
// When no config is found in memory, a default config is loaded (defined in config.h)
// When a pin is not used, value = 0xFF
// When a pin is used twice, the config is not valid and a LED will blink Red 
// to store the pins, variable will be pinChannels[16], pinGpsTx, pinGpsRx, pinPrimIn, pinSecIn, 
//                                     pinSbusOut,pinTlm, pinVolt[4]  pinSda, pinScl,pinRpm, pinLed

#include "rlink.h"
#include "xbus.h"
#include "hitec.h"

#define CMD_BUFFER_LENTGH 3000
uint8_t cmdBuffer[CMD_BUFFER_LENTGH];
uint16_t cmdBufferPos = 0;

extern GPS gps;
extern sbusFrame_s sbusFrame;
extern uint32_t lastRcChannels;

//added aeropic
extern int32_t gyroX;
extern int32_t gyroY; 

CONFIG config;
uint8_t debugTlm = 'N';
uint8_t debugSbusOut = 'N';

uint8_t pinCount[30] = {0};

// for sequencer
int tempIntTable[10]; // temporary table to store n integers converted from the serial buffer (starting from pvalue)
uint8_t nextSequencerBegin;    // true when a step is the first of the next sequencer
uint8_t nextSequenceBegin;    // true when a step is the first of the next sequence

SEQUENCER seq;
extern  SEQ_DATA seqDatas[16];   // internal table to remember the state of each sequencer
extern bool seqDatasToUpload;    // flag to say if seqDatas[] must be updated or not

bool pinIsduplicated ;
extern bool configIsValid; 
extern bool multicoreIsRunning; 
volatile bool isPrinting = false;
extern field fields[];  // list of all telemetry fields and parameters used by Sport

extern MS5611 baro1 ;
extern SPL06  baro2 ;
extern BMP280 baro3 ; 

extern MS4525 ms4525;
extern SDP3X  sdp3x;

extern ADS1115 adc1 ;
extern ADS1115 adc2 ;    

extern MPU mpu;
extern queue_t qSendCmdToCore1;

extern uint8_t forcedFields;

extern float dteCompensationFactor;

extern sbusFrame_s sbusFrame;

extern uint16_t pwmTop; // just used for debugging

void handleUSBCmd(void){
    int c;
    while (1) {
        c = getchar_timeout_us(5);
        //printf("%X\n", (uint8_t) c);
        
        if ( c== PICO_ERROR_TIMEOUT) return;
        //printf("%X\n", (uint8_t) c);
        
        if (cmdBufferPos >= (CMD_BUFFER_LENTGH -1) ){  // discard previous char when buffer is full
            cmdBufferPos = 0;
        }
        if ( c != '\n') {
            cmdBuffer[cmdBufferPos++] = c & 0xFF; // save the char
           // printf("%c\n", (uint8_t) c);
        } else {
            cmdBuffer[cmdBufferPos] = 0x00 ; // put the char end of string
            cmdBufferPos = 0;                // reset the position
            processCmd();                    // process the cmd line
        }
    }
}

char * pkey = NULL;
char * pvalue = NULL;
    

void processCmd(){
    printf("processing cmd\n");
    bool updateConfig = false;      // after some cheks, says if we can save the config
    bool updateSequencers = false;    // after some checks, says if we ca save the sequencers   
    char *ptr;
    uint32_t ui;
    uint32_t ui2;
    double db;    
    pkey = NULL;
    pvalue = NULL;
    //printf("buffer0= %X\n", cmdBuffer[0]);
    if (cmdBuffer[0] == 0x0D){ // when no cmd is entered we print the current config
        printConfigAndSequencers();
        return; 
    }
    if (cmdBuffer[0] == '?'){ // when help is requested we print the instruction
        isPrinting = true; //use to discard incoming data from Sbus... while printing a long text (to avoid having the queue saturated)
        printf("\nCommands can be entered to change the config parameters\n");
        printf("- To activate a function, select the GPIO and enter function code = GPIO (e.g. PRI=5)\n");
        printf("    Function                  Code        Valid GPIO's\n");   

        if ((config.protocol == 'R') || (config.protocol == 'X') || (config.protocol == 'T'))
        {
            printf("    Primary channels input    PRI     = 0, 4, 8, 12 (for SDA0)\n");
        }
        else
        {
          printf("    Primary channels input    PRI     = 5, 9, 21, 25\n");  
        }

        printf("    Secondary channels input  SEC     = 1, 13, 17, 29\n");

        if ((config.protocol == 'R') || (config.protocol == 'X') || (config.protocol == 'T'))
        {
            printf("    Telemetry                 TLM     = 1, 5, 9, 13 (for SCL0)\n");
        }
        else
        {
            printf("    Telemetry                 TLM     = 0, 1, 2, ..., 29\n");
        }

        printf("    GPS Rx                    GPS_RX  = 0, 1, 2, ..., 29\n");
        printf("    GPS Tx                    GPS_TX  = 0, 1, 2, ..., 29\n");
        printf("    Sbus OUT                  SBUS_OUT= 0, 1, 2, ..., 29\n");
        printf("    RPM (only for Sport)      RPM     = 0, 1, 2, ..., 29\n");
        printf("    SDA (baro sensor)         SDA     = 2, 6, 10, 14, 18, 22, 26\n");
        printf("    SCL (baro sensor)         SCL     = 3, 7, 11, 15, 19, 23, 27\n");
        printf("    PWM Channels 1, ..., 16   C1 / C16= 0, 1, 2, ..., 15\n");
        printf("    Voltage 1, ..., 4         V1 / V4 = 26, 27, 28, 29\n");
        printf("    RGB led                   RGB     = 0, 1, 2, ..., 29\n");      
        printf("    CAMERA (PITCH,ROLL,PRATIO,RRATIO) = 0, ..., 29\n");
        printf("    Logger                    LOG     = 0, 1, 2, ..., 29\n");
        printf("    ESC                       ESC_PIN = 0, 1, 2, ..., 29\n");
        printf("- To disable a function, set GPIO to 255\n\n");
        printf("- To declare the type of ESC, enter ESC_TYPE=xxx where XXX = HW4(Hobbywing V4) or KON (Kontronik)\n");
        //printf("-To debug on USB/serial the telemetry frames, enter DEBUGTLM=Y or DEBUGTLM=N (default)\n");
        printf("-To change the protocol, enter PROTOCOL=x where x=");
        printf(" S(Sport Frsky), F(Fbus Frsky), C(CRSF/ELRS), H(Hott), M(Mpx), 2(Sbus2 Futaba), J(Jeti), E(jeti Exbus), L (spektrum SRXL2) ,or I(IBus/Flysky)\n");
        printf("-To change the CRSF baudrate, enter e.g. CRSFBAUD=420000\n");
        printf("-To change the logger baudrate, enter e.g. LOGBAUD=115200\n");
        printf("-To change the refresh rate of servos (PWM) and sequencer, enter e.g. PWMHZ=50 (value in range 50...333)\n");
        printf("-To change voltage scales, enter SCALEx=nnn.ddd e.g. SCALE1=2.3 or SCALE3=0.123\n")  ;
        printf("     Enter SCALEx=0 to avoid sending voltage x to the Transmitter (for Frsky or Jeti)\n")  ;
        printf("-If a TMP36 is used on V3, enter TEMP=1 (if a second one is on V4, enter TEMP=2)\n");
        printf("-To change voltage offset, enter OFFSETx=nnn.ddd e.g. OFFSET1=0.6789\n")  ;
        printf("-To change GPS type: for an Ublox, enter GPS=U (configured by oXs) or E (configured Externally) and for a CADIS, enter GPS=C\n");
        printf("-To change RPM multiplicator, enter e.g. RPM_MULT=0.5 to divide RPM by 2\n");
        printf("-To force a calibration of MP6050, enter MPUCAL\n");
        printf("-To use a channel to setup Airspeed compensation factor and/or to select between the 2 Vspeed, enter the channel with ACC=1...16\n");
    //    printf("-To select the signal generated on:\n");
    //    printf("     GPIO0 : enter GPIO0=SBUS or GPIO0=xx where xx = 01 up to 16\n");
    //    printf("     GPIO1 : enter GPIO1=xx where xx = 01 up to 13 (GPIO2...4 will generate channel xx+1...3)\n");
    //    printf("     GPIO5 : enter GPIO5=xx where xx = 01 up to 13 (GPIO6...8 will generate channel xx+1...3)\n");
    //    printf("     GPIO11: enter GPIO11=xx where xx = 01 up to 16\n");
        printf("-To change (invert) led color, enter LED=N or LED=I\n");
        printf("-To select the failsafe mode to HOLD, enter FAILSAFE=H\n")  ;
        printf("-To set the failsafe values on the current position, enter SETFAILSAFE\n")  ;
        
        printf("\n");
        printf("-To define one (or several) sequencers, enter SEQ= followed by one (or several) groups {s1 s2 s3 s4 s5 s6 s7} where\n");
        printf("        s1 = GPIO to be used by this sequencer(in range 0/16)\n");
        printf("        s2 = type of PWM (0=SERVO , 1=ANALOG)\n");
        printf("        s3 = number of milli seconds per sequencer clock (must be >= 20msec)\n");
        printf("        s4 = rc channel used to select the sequence to be generated (in range 1/16)\n");
        printf("        s5 = default PWM value (when no sequence has already been selected)\n");
        printf("        s6 = min PWM value\n");
        printf("        s7 = max PWM value\n");
        printf("     note: s5, s6, s7 must be in range -100/100 for SERVO and 0/100 for ANALOG\n");
        printf("     e.g. SEQ= {2 0 30 4 -100 -100 100} {3 1 100 5 0 0 100}\n");
        
        printf("-To erase all sequencers, enter SEQ=DEL\n");
        
        printf("-To define the sequences and the steps for the sequencers, enter STEP= followed by several groups {s1 s2 s3 s4 } where\n");
        printf("        s1 = RC channel value that activates that step (must be a multiple of 10 and in range -100...100; e.g. -100, -90,...0,10,...100)\n");
        printf("        s2 = number of clocks before reaching the PWM value (= smooth transition)(in range 0/255)\n");
        printf("        s3 = PWM value for this step(same range as default PWM value)\n");
        printf("        s4 = number of clocks where PWM value is kept before applying next step or restarting the sequence (in range 0/255; 255=do not restart)\n");
        printf("     To mark the first step of a new sequencer, insert a '+' just before the '{' of this group\n");
        printf("     e.g. STEP= {-100 0 10 4} {-100 10 50 20} {100 0 100 255} + {-100 0 0 255} {0 10 50 255} {100 0 100 40}\n");
        printf("     Note: steps must be declared in the same order as the sequencers\n");
        printf("           For each sequencer, rc channel value of step n+1 must be >= value of step n\n");
        
        printf("\n");
        printf("-To get the internal telemetry values currently calculated by oXs, enter FV (meaning Field Values)\n")  ;
        printf("-To test a protocol, you can force the internal telemetry values to some dummy values\n")  ;
        printf("        for dummy positive values, enter FVP; for dummy negative values, enter FVN\n")  ;
        printf("\n");
        printf("-To get the current PWM values (in micro sec, enter PWM)\n");
        printf("-To get the current config, just press Enter\n");
        printf("   Note: some changes require a reset to be applied (e.g. to unlock I2C bus)\n");
        isPrinting = false;
        return;  
    }
    if (cmdBuffer[0] != 0x0){
        char * equalPos = strchr( (char*)cmdBuffer, '=');  // search position of '='
        
        if (equalPos != NULL){ // there is = so search for value
            *equalPos = 0x0;      // replace '=' with End of string    
            equalPos++;           // point to next position  
            pvalue = skipWhiteSpace(equalPos);
            removeTrailingWhiteSpace(pvalue);
        }    
        pkey =  skipWhiteSpace((char*)cmdBuffer);  
        removeTrailingWhiteSpace(pkey);
    }
    upperStr(pkey);
    upperStr(pvalue);
    // pkey point to the key (before '=')
    // pvalue point to the value
    printf("\nCmd to execute: ");   
    if (pkey) printf("  %s", pkey);
    if (pvalue) printf("=%s", pvalue);
    printf("\n");
    
    // change PRI pin
    if (( strcmp("PRI", pkey) == 0 ) && ((config.protocol != 'R')||(config.protocol != 'X')||(config.protocol != 'T'))){ 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ((config.protocol != 'E' && config.protocol != 'F' && config.protocol != 'L') &&
                   ( !(ui == 5 or ui == 21 or ui == 9 or ui ==25 or ui ==255))) {
            printf("Error : PRI pin must be 5 ,21 , 9 , 25 or 255 for most protocols (except Exbus, Fbus and SRXL2)\n");
        } else if ( (config.protocol == 'E' || config.protocol == 'F' || config.protocol == 'L') &&
                   ( !(ui <=29 or ui ==255))) {
            printf("Error : PRI pin must be in range 0/29 or 255 for Exbus, Fbus and SRXL2 protocol\n");
        } else {    
            config.pinPrimIn = ui;
            printf("Pin for primary channels input = %u\n" , config.pinPrimIn);
            updateConfig = true;
        }
    }

    if (( strcmp("PRI", pkey) == 0 ) && ((config.protocol == 'R')||(config.protocol == 'X')||(config.protocol == 'T'))){ 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui == 1 or ui == 5 or ui == 9 or ui ==13 or ui ==255)) {
            printf("Error : pin must be 1, 5, 9, 13 (for SCL0) or 255\n");
        } else {    
            config.pinPrimIn = ui;
            printf("Pin for primary channels input = %u\n" , config.pinPrimIn);
            updateConfig = true;
        }
    }

    // change SEC pin
    if ( strcmp("SEC", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui == 1 or ui == 17 or ui == 13 or ui ==29 or ui ==255)) {
            printf("Error : pin must 1, 13, 17, 29 or 255");
        } else {    
            config.pinSecIn = ui;
            printf("Pin for secondary channels input = %u\n" , config.pinSecIn);
            updateConfig = true;
        }
    }


    // change TLM pin
    if (( strcmp("TLM", pkey) == 0 ) && ((config.protocol != 'R')||(config.protocol != 'X')||(config.protocol != 'T'))){ 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinTlm = ui;
            printf("Pin for telemetry = %u\n" , config.pinTlm );
            updateConfig = true;
        }
    }

    // change TLM pin
    if (( strcmp("TLM", pkey) == 0 ) && ((config.protocol == 'R')||(config.protocol == 'X')||(config.protocol == 'T'))){ 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui == 0 or ui == 4 or ui == 8 or ui ==12 or ui ==255)) {
            printf("Error : pin must be 0, 4, 8, 12 (for SDA0) or 255\n");
        } else {    
            config.pinTlm = ui;
            printf("Pin for telemetry = %u\n" , config.pinTlm );
            updateConfig = true;
        }
    }


    // change GPS Rx pin
    if ( strcmp("GPS_RX", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinGpsRx = ui;
            printf("Pin for GPS Rx = %u\n" , config.pinGpsRx );
            updateConfig = true;
        }
    }
    // change GPS Tx pin
    if ( strcmp("GPS_TX", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinGpsTx = ui;
            printf("Pin for GPS Tx = %u\n" , config.pinGpsTx );
            updateConfig = true;
        }
    }
    // change Sbus out pin
    if ( strcmp("SBUS_OUT", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinSbusOut = ui;
            printf("Pin for Sbus output = %u\n" , config.pinSbusOut );
            updateConfig = true;
        }
    }
    // change for RPM pin
    if ( strcmp("RPM", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinRpm = ui;
            printf("Pin for RPM = %u\n" , config.pinRpm );
            updateConfig = true;
        }
    }
    // change for SDA pin
    if ( strcmp("SDA", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui==2 or ui==6 or ui==10 or ui==14 or ui==18 or ui==22 or ui==26 or ui ==255)) {
            printf("Error : pin must be 2, 6, 10, 14, 18, 22, 26 or 255\n");
        } else {    
            config.pinSda = ui;
            printf("Pin for SDA (baro) = %u\n" , config.pinSda );
            updateConfig = true;
        }
    }
    // change for SCL pin
    if ( strcmp("SCL", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui==3 or ui==7 or ui==11 or ui==15 or ui==19 or ui==23 or ui==27 or ui ==255)) {
            printf("Error : pin must be 3, 7, 11, 15, 19, 23, 27 or 255\n");
        } else {    
            config.pinScl = ui;
            printf("Pin for Scl (baro) = %u\n" , config.pinScl );
            updateConfig = true;
        }
    }
    if ( strcmp("MPUCAL", pkey) == 0 ) {
        requestMpuCalibration();
        return; // do not continue in order to avoid printing config while config print some data too.
        /*
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            config.rpmMultiplicator = db;
            updateConfig = true;
        }
        */
    }
    
    // change for channels
    if ( *pkey == 'C' && * (pkey+1) >= '0' && * (pkey+1) <= '9') {
        pkey++; // skip 'C' char to extract the digits
        ui2 = strtoul(pkey, NULL, 10);
        //printf("channel is = %u\n", (int) ui2);
        if ( (ui2==0 or ui2>16 )){
            printf("Error : Channel number must be in range 1 / 16\n");
        } else {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0){
                printf("Error : pin must be an unsigned integer\n");
            } else if ( !(ui<16 or ui ==255)) {
                printf("Error : pin must be in range 0 / 15 or 255\n");
            } else {    
                config.pinChannels[ui2-1] = ui;
                printf("Pin for channel %" PRIu32 " = %u\n" , ui2 , config.pinChannels[ui2-1] );
                updateConfig = true;
            }    
        }
    }
    // change for voltages
    if ( *pkey == 'V' && * (pkey+1) >= '0' && * (pkey+1) <= '9') {
        pkey++; // skip 'V' char to extract the digits
        ui2 = strtoul(pkey, NULL, 10);
        if ( (ui2==0 or ui2>4 )){
            printf("Error : Voltage number must be in range 1 / 4\n");
        } else {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0){
                printf("Error : pin must be an unsigned integer\n");
            } else if ( ! ( ((ui>=26 and ui<=29)) or ui ==255) ) {
                printf("Error : pin must be in range 26 / 29 or 255\n");
            } else {    
                config.pinVolt[ui2-1] = ui;
                printf("Pin for voltage %" PRIu32 " = %u\n" , ui2, config.pinVolt[ui2-1] );
                updateConfig = true;
            }    
        }
    }


    // change for Camera pin Pitch
    if ( strcmp("PITCH", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=16 or ui ==255)) {
            printf("Error : pin must be in range 1/16 or 255\n");
        } else {    
            config.CamPitchChannel = ui;
            printf("Pin for Pitch = %u\n" , config.CamPitchChannel );
            updateConfig = true;
        }
    }
    // change for Camera pin Roll
    if ( strcmp("ROLL", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=16 or ui ==255)) {
            printf("Error : pin must be in range 1/16 or 255\n");
        } else {    
            config.CamRollChannel = ui;
            printf("Pin for Roll = %u\n" , config.CamRollChannel );
            updateConfig = true;
        }
    }
    // change for Camera pin Pitch Ratio
    if ( strcmp("PRATIO", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=16 or ui ==255)) {
            printf("Error : pin must be in range 1/16 or 255\n");
        } else {    
            config.CamPitchRatio = ui;
            printf("Pin for Pitch Ratio = %u\n" , config.CamPitchRatio );
            updateConfig = true;
        }
    }
    // change for Camera pin Roll Ratio
    if ( strcmp("RRATIO", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=16 or ui ==255)) {
            printf("Error : pin must be in range 1/16 or 255\n");
        } else {    
            config.CamRollRatio = ui;
            printf("Pin for Roll Ratio = %u\n" , config.CamRollRatio );
            updateConfig = true;
        }
    }


    // change crsf baudrate
    if ( strcmp("CRSFBAUD", pkey) == 0 ) { // if the key is CRSFBAUD
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : CRSF baudrate must be an unsigned integer\n");
        } else {
            config.crsfBaudrate = ui;
            printf("CRSF baudrate = %" PRIu32 "\n" , config.crsfBaudrate);
            updateConfig = true;
        }
    }
    
    // change debugTlm
    if ( strcmp("DEBUGTLM", pkey) == 0 ) { // if the key is DEBUGTLM
        if (strcmp("Y", pvalue) == 0) {
            debugTlm = 'Y';
        } else if (strcmp("N", pvalue) == 0) {
            debugTlm = 'N';
            //updateConfig = true; // this is not saved
        } else  {
            printf("Error : DEBUGTLM must be Y or N\n");
        }
    }
    
    // change debugSbusOut
    if ( strcmp("DEBUGSBUSOUT", pkey) == 0 ) { // if the key is DEBUGSBUSOUT
        if (strcmp("Y", pvalue) == 0) {
            debugSbusOut = 'Y';
        } else if (strcmp("N", pvalue) == 0) {
            debugSbusOut = 'N';
            //updateConfig = true; // this is not saved
        } else  {
            printf("Error : DEBUGSBUSOUT must be Y or N\n");
        }
    }
    
    // print current values of all telemetry fields
    if ( strcmp("FV", pkey) == 0 ) { 
            printFieldValues();
            return;
    }
    // force dummy positive value
    if ( strcmp("FVP", pkey) == 0 ) { 
            forcedFields = 1;
            fillFields(forcedFields);
            if (config.protocol == 'E') setupExbusList(true); // rebuild the list of fields being sent in jeti exbus
            if (config.protocol == 'J') initListOfJetiFields(true); // rebuild the list of fields being sent in jeti ex
            printFieldValues();
            printf("Internal telemetry fields are now filled with POSITIVE dummy values\n");
            printf("To get real values again, you have to power down\n");
            
            return;
    }
// force dummy negative value
    if ( strcmp("FVN", pkey) == 0 ) { 
            forcedFields = 2;
            fillFields(forcedFields);
            if (config.protocol == 'E') setupExbusList(true); // rebuild the list of fields being sent in jeti exbus
            if (config.protocol == 'J') initListOfJetiFields(true); // rebuild the list of fields being sent in jeti ex
            
            printFieldValues();
            printf("Internal telemetry fields are now filled with NEGATIVE dummy values\n");
            printf("To get real values again, you have to power down\n");
            return;
    }
    // print current values of all PWM fields
    if ( strcmp("PWM", pkey) == 0 ) { 
            printPwmValues();
            return;
    }
    // change protocol
    if ( strcmp("PROTOCOL", pkey) == 0 ) { // 
        if (strcmp("S", pvalue) == 0) {
            config.protocol = 'S';
            updateConfig = true;
        } else if (strcmp("C", pvalue) == 0) {
            config.protocol = 'C';
            updateConfig = true;
        } else if (strcmp("J", pvalue) == 0) {
            config.protocol = 'J';
            updateConfig = true;
        } else if (strcmp("E", pvalue) == 0) {
            config.protocol = 'E';
            updateConfig = true;
        } else if (strcmp("H", pvalue) == 0) {
            config.protocol = 'H';
            updateConfig = true;
        } else if (strcmp("M", pvalue) == 0) {
            config.protocol = 'M';
            updateConfig = true;
        } else if (strcmp("F", pvalue) == 0) {
            config.protocol = 'F';
            updateConfig = true;
        } else if (strcmp("I", pvalue) == 0) {
            config.protocol = 'I';
            updateConfig = true;
        } else if (strcmp("2", pvalue) == 0) {
            config.protocol = '2';
            updateConfig = true;
        } else if (strcmp("L", pvalue) == 0) {
            config.protocol = 'L';
            updateConfig = true;

/* Add I2C Protocols */
        } else if (strcmp("R", pvalue) == 0) {/*Ajout RadioLink*/
            config.protocol = 'R';
            updateConfig = true;
        } else if (strcmp("X", pvalue) == 0) {/*Ajout Spektrum Xbus*/
            config.protocol = 'X';
            updateConfig = true;
        } else if (strcmp("T", pvalue) == 0) {/*Ajout Hitec*/
            config.protocol = 'T';
            updateConfig = true;
/* Add I2C Protocols */

        } else  {
            printf("Error : protocol must be S(Sport Frsky), F(Fbus Frsky), C(CRSF=ELRS), J(Jeti), E(jeti Exbus), H(Hott), M(Mpx), 2(Sbus2 Futaba), L(SRXL2 Spektrum) or I(Ibus/Flysky)\n");
        }
    }
    
    
    // change scale
    if (( strcmp("SCALE1", pkey) == 0 ) || ( strcmp("SCALE2", pkey) == 0 )\
         || ( strcmp("SCALE3", pkey) == 0 )  || ( strcmp("SCALE4", pkey) == 0 ) ){ 
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            updateConfig = true;
            if (*(pkey+5) == '1' ) {config.scaleVolt1 = db;}
            else if (*(pkey+5) == '2' ) {config.scaleVolt2 = db;}
            else if (*(pkey+5) == '3' ) {config.scaleVolt3 = db;}
            else if (*(pkey+5) == '4' ) {config.scaleVolt4 = db;}
            else {
                printf("Error : x must be 1...4 in SCALEx\n");
                updateConfig = false;
            }
        }
    }
    // change offset
    if (( strcmp("OFFSET1", pkey) == 0 ) || ( strcmp("OFFSET2", pkey) == 0 )\
         || ( strcmp("OFFSET3", pkey) == 0 )  || ( strcmp("OFFSET4", pkey) == 0 ) ){ 
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            updateConfig = true;
            if (*(pkey+6) == '1' ) {config.offset1 = db;}
            else if (*(pkey+6) == '2' ) {config.offset2 = db;}
            else if (*(pkey+6) == '3' ) {config.offset3 = db;}
            else if (*(pkey+6) == '4' ) {config.offset4 = db;}
            else {
                printf("Error : x must be 1...4 in OFFSETx\n");
                updateConfig = false;
            }
        }
    }
    // change GPS
    if ( strcmp("GPS", pkey) == 0 ) {
        if (strcmp("U", pvalue) == 0) {
            config.gpsType = 'U';
            updateConfig = true;
        } else if (strcmp("E", pvalue) == 0) {
            config.gpsType = 'E';
            updateConfig = true;
        } else if (strcmp("C", pvalue) == 0) {
            config.gpsType = 'C';
            updateConfig = true;
        } else  {
            printf("Error : GPS type must be U, E or C\n");
        }
    }
    // change RPM multipicator
    if ( strcmp("RPM_MULT", pkey) == 0 ) {
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            config.rpmMultiplicator = db;
            updateConfig = true;
        }
    }
    

    
    // change gpio0
    /*
    if ( strcmp("GPIO0", pkey) == 0 ) {
        if (strcmp("SBUS", pvalue) == 0) {
            config.gpio0 = 0 ;
            printf("gpio0 = Sbus\n" );
            updateConfig = true;
        } else {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 16){
                printf("Error : GPIO0 must be SBUS or an integer between 1 and 16");
            } else {
                config.gpio0 = ui;
                printf("GPIO0 = %u\n" , (unsigned int) config.gpio0);
                updateConfig = true;
            }
        }    
    }
    
    // change gpio1
    if ( strcmp("GPIO1", pkey) == 0 ) {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 13){
                printf("Error : GPIO1 must be an integer between 1 and 13");
            } else {
                config.gpio1 = ui;
                printf("GPIO1 = %u\n" , (unsigned int) config.gpio1);
                updateConfig = true;
            }
    }
    
    
    // change gpio5
    if ( strcmp("GPIO5", pkey) == 0 ) {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 13){
                printf("Error : GPIO5 must be an integer between 1 and 13");
            } else {
                config.gpio5 = ui;
                printf("GPIO5 = %u\n" , (unsigned int) config.gpio5);
                updateConfig = true;
            }
    }

    
    // change gpio11
    if ( strcmp("GPIO11", pkey) == 0 ) {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 16){
                printf("Error : GPIO11 must be an integer between 1 and 16");
            } else {
                config.gpio11 = ui;
                printf("GPIO11 = %u\n" , (unsigned int) config.gpio11);
                updateConfig = true;
            }
    }
    */
    
    // change failsafe mode
    if ( strcmp("FAILSAFE", pkey) == 0 ) {
        if (strcmp("H", pvalue) == 0) {
            config.failsafeType = 'H';
            updateConfig = true;
        } else  {
            printf("Error : FAILSAFE mode must be H\n");
        }
    }
    // set failsafe to the current values
    if ( strcmp("SETFAILSAFE", pkey) == 0 ) { // if the key is Failsafe
        if ( lastRcChannels ) {
            config.failsafeType = 'C'; // remove 'H' for HOLD
            memcpy( &config.failsafeChannels , &sbusFrame.rcChannelsData, sizeof(config.failsafeChannels));
            updateConfig = true;
        } else {
            printf("Error : No RC channels have been received yet. FAILSAFE values are unknown\n");
        }    
    }
    // change number of temperature sensors
    if ( strcmp("TEMP", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : value must be an unsigned integer\n");
        } else if ( !(ui==0 or ui==1 or ui==2 or ui ==255)) {
            printf("Error : value must be 0, 1, 2 or 255\n");
        } else {    
            config.temperature = ui;
            printf("Number of temperature sensors = %u\n" , config.temperature );
            updateConfig = true;
        }
    }
    // change Vspeed compensation channel 
    if ( strcmp("ACC", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : channel must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 16 or ui ==255)) {
            printf("Error : channel must be 1...16 or 255");
        } else {    
            config.VspeedCompChannel = ui;
            printf("Vspeed compensation channel = %u\n" , config.VspeedCompChannel);
            updateConfig = true;
        }
    }
    // change for RGB led gpio
    if ( strcmp("RGB", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gpio must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui==255)) {
            printf("Error : gpio must be in range 0/29 or 255\n");
        } else {    
            config.pinLed = ui;
            printf("gpio for RGB led = %u\n" , config.pinLed );
            updateConfig = true;
        }
    }
    // change led color
    if ( strcmp("LED", pkey) == 0 ) {
        if (strcmp("N", pvalue) == 0) {
            config.ledInverted = 'N';
            updateConfig = true;
        } else if (strcmp("I", pvalue) == 0) {
            config.ledInverted = 'I';
            updateConfig = true;
        } else  {
            printf("Error : LED color must be N (normal) or I(inverted)\n");
        }
    }

    // change for Log pin
    if ( strcmp("LOG", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gpio must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui==255)) {
            printf("Error : gpio must be in range 0/29 or 255\n");
        } else {    
            config.pinLogger = ui;
            printf("Gpio for Logger = %u\n" , config.pinLogger );
            updateConfig = true;
        }
    }
    
    // change logger baudrate
    if ( strcmp("LOGBAUD", pkey) == 0 ) { // if the key is LOGBAUD
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : logger baudrate must be an unsigned integer\n");
        } else {
            config.loggerBaudrate = ui;
            printf("Logger baudrate = %" PRIu32 "\n" , config.loggerBaudrate);
            updateConfig = true;
        }
    }

    // change for Esc pin
    if ( strcmp("ESC_PIN", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinEsc = ui;
            printf("Pin for ESC = %u\n" , config.pinEsc );
            updateConfig = true;
        }
    }
    
    // change for type of esc
    if ( strcmp("ESC_TYPE", pkey) == 0 ) { 
        if (strcmp("HW4", pvalue) == 0) {
            config.escType = HW4 ;
            updateConfig = true; 
        //} else if  (strcmp("HW3", pvalue) == 0) {
        //    config.escType = HW3 ;
        //    updateConfig = true;
        } else if (strcmp("KON", pvalue) == 0) {
            config.escType = KONTRONIK ;
            updateConfig = true;
        } else {    
            printf("Error : ESC_TYPE must be HW4 or KON\n");
        }
    }

    // change PWM HZ
    if ( strcmp("PWMHZ", pkey) == 0 ) { // if the key is PWMHZ
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : PWMHZ must be an unsigned integer\n");
        } else if ((ui < 50 ) || (ui > 333)){
            printf("Error : PWMHZ must be in range 50...333 (included)\n");
        } else {
            config.pwmHz = ui;
            printf("PwmHz = %" PRIu32 "\n" , config.pwmHz);
            updateConfig = true;
        }
    }

    // get Sequencer definition
    if ( strcmp("SEQ", pkey) == 0 ) { 
        if (strcmp("DEL", pvalue) == 0) {
            seq.defsMax=0;
            seq.stepsMax=0;
            //sequencerIsValid=false;
            updateConfig = true;
            printf("All definitions for sequencer are deleted\n");
        } else {    
            if (getAllSequencers()){ // true when valid syntax is decoded and seq structure has been updated ;
                                  // we will save the structure and reboot; during reboot we will check if config is valid
                updateConfig = true;
            } else {
                printf("\nError in syntax or in a parameter: command SEQ= is discarded\n");
                return;
            }
        }  
    }
    
/*
    // get steps for Sequencer
    if ( strcmp("STEP", pkey) == 0 ) { 
        if (seq.defsMax == 0){
            printf("\nError in command STEP=: number of sequencers = 0; fill SEQ= command before entering STEP=\n");
            return;
        }
        if (getStepsSequencers()){ // true when valid syntax is decoded and step structure has been updated (not yet the seqDatas[] table);
                                  // we will save the structure and reboot; during reboot we will check if config is valid
            updateConfig = true;
        } else { 
            // in case of error, we just discard the command
            printf("\nError in syntax or in a parameter: command STEP= is discarded\n");
            return;
        }
    }
*/	  


    if (updateConfig) {
        saveConfig();
        saveSequencers();
        printf("config has been saved\n");  
        printf("Device will reboot but it could be that a reset or a (power down + power on) is required\n\n");
        watchdog_enable(1500,false);
        sleep_ms(1000);
        watchdog_reboot(0, 0, 100); // this force a reboot!!!!!!!!!!
        sleep_ms(5000);
        printf("OXS did not rebooted after 5000 ms\n");
    }
        
    if ( strcmp("N", pkey) == 0 ) {
        nextSimuSeqChVal();
        return;
    }    
    if ( strcmp("A", pkey) == 0 ) printAttitudeFrame(); // print Attitude frame with vario data
    if ( strcmp("G", pkey) == 0 ) printGpsFrame();      // print GPS frame
    if ( strcmp("B", pkey) == 0 ) printBatteryFrame();   // print battery frame 
    printConfigAndSequencers();                                       // print the current config
    printf("\n >> \n");
}


void addPinToCount(uint8_t pinId){
    if ( pinId != 255) {
        if (pinId > 29 ) {
            printf("Error in parameters: one pin number is %u : it must be <30", pinId);
            configIsValid = false;
        } else {
            pinCount[pinId]++;
        }
    }
}

void checkConfigAndSequencers(){     // set configIsValid 
    // each pin can be used only once (also in sequencer)
    // if SDA is defined SCL must be defined too, and the opposite
    // if GPS_TX is defined GPS_RX must be defined too and the opposite
    watchdog_update(); //sleep_ms(500);
    bool atLeastOnePwmPin = false;
    //pinIsduplicated = false; 
    for (uint8_t i = 0 ; i<30; i++) pinCount[i] = 0; // reset the counter
    configIsValid = true;
    addPinToCount(config.pinGpsTx); 
    addPinToCount(config.pinGpsRx);
    addPinToCount(config.pinPrimIn);
    addPinToCount(config.pinSecIn);
    addPinToCount(config.pinSbusOut);
    addPinToCount(config.pinTlm);
    addPinToCount(config.pinSda);
    addPinToCount(config.pinScl);
    addPinToCount(config.pinRpm);
    addPinToCount(config.pinLed);
    for (uint8_t i = 0 ; i<16 ; i++) {addPinToCount(config.pinChannels[i]);}
    for (uint8_t i = 0 ; i<16 ; i++) {
        if (config.pinChannels[i] != 255) atLeastOnePwmPin = true ;}
    for (uint8_t i = 0 ; i<4 ; i++) {addPinToCount(config.pinVolt[i]);}
    addPinToCount(config.pinLogger);
    addPinToCount(config.pinEsc);
    for (uint8_t i = 0 ; i<seq.defsMax ; i++) {
        if (seq.defs[i].pin > 29 ) {
            printf("Error in sequencer: one pin number is %u : it must be <30", seq.defs[i].pin);
            configIsValid = false;
        } else {
            pinCount[seq.defs[i].pin]++;   
        }
    }
    for (uint8_t i = 0 ; i<30; i++) {
        if (pinCount[i] > 1) {
            printf("Error in parameters: pin %u is used %u times\n", i , pinCount[i]);
            configIsValid=false;
            pinIsduplicated= true;
        }          
    }
    if ( (config.pinSda != 255 and config.pinScl==255) or
         (config.pinScl != 255 and config.pinSda==255) ) {
        printf("Error in parameters: SDA and SCL must both be defined or unused\n");
        configIsValid=false;
    }
    if ( (config.pinGpsTx != 255 and config.pinGpsRx==255) or 
         (config.pinGpsRx != 255 and config.pinGpsTx==255) ) {
        printf("Error in parameters: GPS_TX and GPS_RX must both be defined or unused\n");
        configIsValid=false;
    }
    if ( (config.pinSbusOut != 255 and config.pinPrimIn==255 and config.pinSecIn==255) ) { //check that when Sbus out is defined, PrimIn is defined too
        printf("Error in parameters: a pin is defined for Sbus Out but not for Primary nor Secondary channels input (PRI or SEC)\n");
        configIsValid=false;
    }
    if ( (atLeastOnePwmPin and config.pinPrimIn==255 and config.pinSecIn==255) ) { //check that when pwm is defined, PrimIn is defined too
        printf("Error in parameters: at least one PWM pin is defined but no pin for Primary nor Secondary channels input (PRI or SEC)\n");
        configIsValid=false;
    }
    
    //if ( (config.pinSecIn != 255 and config.pinPrimIn ==255) ) { //check that Prim is defined ff pinSec is defined
    //    printf("Error in parameters: pin is defined for secondary channels input but not for Primary channels input (PRI)\n");
    //    configIsValid=false;
    //}
    //if ( (config.pinSbusOut != 255 and ( config.protocol == 'S' or config.protocol == 'J') ) ) { //check that Prim is defined ff pinSec is defined
    //    printf("Warning: Sbus signal will not be generated for Sport or Jeti protocol\n");
    //}
    if (config.temperature == 1 and config.pinVolt[2] == 255){
        printf("Error in parameters: when 1 temperature sensor is used (TEMP = 1), a pin for V3 must be defined too)\n");
        configIsValid=false;
    }
    if (config.temperature == 2 && (config.pinVolt[2] == 255 || config.pinVolt[3] == 255)){
        printf("Error in parameters: when 2 temperature sensors are used (TEMP = 2), a pin for V3 and for V4 must be defined too)\n");
        configIsValid=false;
    }
    if ( config.protocol != 'E' && config.protocol != 'F' && config.protocol != 'L' &&
        ( !(config.pinPrimIn == 5 or config.pinPrimIn == 21 or config.pinPrimIn == 9 or config.pinPrimIn ==25 or config.pinPrimIn ==255))) {
            printf("Error : PRI pin must be 5 ,21 , 9 , 25 or 255 for most protocols (except Exbus, Fbus and SRXL2)\n");
            configIsValid=false;
    }
    if (config.protocol == '2' && config.pinPrimIn == 255){
        printf("Error in parameters: For Futaba Sbus2 protocol, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == '2' && config.pinTlm != 255 && ( config.pinTlm != (config.pinPrimIn - 1)) ){
        printf("Error in parameters: For Futaba SBUS2 protocol, TLM pin (when defined) must be equal to (PRI pin -1) \n");
        configIsValid=false;
    }
    if (config.protocol == 'F' && config.pinPrimIn == 255  ){
        printf("Error in parameters: For Frsky Fbus, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == 'E' && config.pinPrimIn == 255  ){
        printf("Error in parameters: For JEti Exbus, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == 'F' && config.pinTlm != 255  ){
        printf("Error in parameters: For Frsky Fbus, TLM pin may not be defined (but PRI must be defined)\n");
        configIsValid=false;
    }
    if (config.protocol == 'E' && config.pinTlm != 255  ){
        printf("Error in parameters: For jeti Exbus, TLM pin may not be defined (but PRI must be defined)\n");
        configIsValid=false;
    }
    if (config.protocol == 'L' && config.pinPrimIn == 255  ){
        printf("Error in parameters: For Spektrum SRXL2, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == 'L' && config.pinTlm != 255  ){
        printf("Error in parameters: For Spektrum SRXL2, TLM pin may not be defined (but PRI must be defined)\n");
        configIsValid=false;
    }
    if (!( config.VspeedCompChannel >= 1 or config.VspeedCompChannel <= 16 or config.VspeedCompChannel ==255)){
        printf("Error in parameters: Vspeed compensation channel must be in range 1...16 or 255\n");
        configIsValid=false;
    }
    if ( (config.pinLogger != 255) && ( config.loggerBaudrate < 9600) || (config.loggerBaudrate > 1000000 )){
        printf("Error in parameters: Logger baudrate must be in range 9600...1000000\n");
        configIsValid=false;
    }
    if ( (config.pinEsc != 255) && (config.pinVolt[0]!=255) ) {
        printf("Error in parameters: When gpio is defined for ESC, gpio for Volt1 (V1) must be undefined (=255)\n");
        configIsValid=false;
    }    
    if ( (config.pinEsc != 255) && (config.pinVolt[1]!=255)) {
        printf("Error in parameters: When gpio is defined for ESC, gpio for current = Volt2 (V2) must be undefined (=255)\n");
        configIsValid=false;
    }    
    if ( (config.pinEsc != 255) && (config.pinRpm!=255)) {
        printf("Error in parameters: When gpio is defined for ESC, gpio for RPM must be undefined (=255)\n");
        configIsValid=false;
    }
    if ( (config.pinEsc != 255) && (config.temperature!=255) && (config.temperature!=0)) {
        printf("Error in parameters: When gpio is defined for ESC, parameter about number of temperature (TEMP) must be 0 or 255\n");
        configIsValid=false;
    }    
    if ( (config.pinEsc != 255) && (config.escType!=HW3) && (config.escType!=HW4) && (config.escType!=KONTRONIK)) {
        printf("Error in parameters: When gpio is defined for ESC, esc type must be HW4 or KON\n");
        configIsValid=false;
    }    
    if ( (config.pwmHz < 50) || (config.pwmHz > 333)){
        printf("Error in parameters: pwmHz must be in range 50...333 (included)\n");
        configIsValid=false;
    }    

/* Add I2C Protocols */
    if (config.protocol == 'R' && config.pinPrimIn == 255  ){/*Ajout RadioLink*/
        printf("Error in parameters: For RadioLink, PRI pin must be defined\n");
        configIsValid=false;
    }
    if (config.protocol == 'R' && config.pinTlm == 255  ){/*Ajout RadioLink*/
        printf("Error in parameters: For RadioLink, TLM pin must be defined\n");
        configIsValid=false;
    }
    if (config.protocol == 'X' && config.pinPrimIn == 255  ){/*Ajout RadioLink*/
        printf("Error in parameters: For Spektrum Xbus, PRI pin must be defined\n");
        configIsValid=false;
    }
    if (config.protocol == 'X' && config.pinTlm == 255  ){/*Ajout RadioLink*/
        printf("Error in parameters: For Spektrum Xbus, TLM pin must be defined\n");
        configIsValid=false;
    }

    if (config.protocol == 'T' && config.pinPrimIn == 255  ){/*Ajout Hitec*/
        printf("Error in parameters: For Hitec, PRI pin must be defined\n");
        configIsValid=false;
    }
    if (config.protocol == 'T' && config.pinTlm == 255  ){/*Ajout Hitec*/
        printf("Error in parameters: For Hitec, TLM pin must be defined\n");
        configIsValid=false;
    }
/* Add I2C Protocols */
/* Add Camera */
    if ( (config.CamPitchChannel != 255 and config.CamRollChannel==255) or 
         (config.CamRollChannel != 255 and config.CamPitchChannel==255) or
         (config.CamPitchRatio != 255 and config.CamRollRatio==255) or
         (config.CamRollRatio != 255 and config.CamPitchRatio==255)) {
        printf("Error in parameters: PITCH, ROLL, PRATIO and RRATIO must both be defined or unused\n");
        configIsValid=false;
    }
/* Add Camera */
    checkSequencers();
    if ( configIsValid == false) {
        printf("\nAttention: error in config parameters\n");
    } else {
        printf("\nConfig parameters are OK\n");
    }
//    if ( sequencerIsValid == false) {
//        printf("\nAttention: error in sequencer parameters\n");
//    } else {
//        printf("\nSequencer parameters are OK\n");
//    }
    printf("Press ? + Enter to get help about the commands\n");
}

void printConfigAndSequencers(){
    //startTimerUs(0) ;  // to debug only - to know how long it takes to print the config
    isPrinting = true;
    uint8_t version[] =   VERSION ;
    printf("\nVersion = %s \n", version)  ;
    printf("    Function                GPIO  Change entering XXX=yyy (yyy=255 to disable)\n");

/* Add I2C Protocols */
    if ((config.protocol == 'R') || (config.protocol == 'X') || (config.protocol == 'T'))
    {
        printf("Primary channels input    = %4u  (PRI     = 1, 5, 9, 13 (for SCL0) )\n", config.pinPrimIn); 
    }
    else
    {
        printf("Primary channels input    = %4u  (PRI     = 5, 9, 21, 25)\n", config.pinPrimIn);      
    }
    printf("Secondary channels input  = %4u  (SEC     = 1, 13, 17, 29)\n", config.pinSecIn);
    if ((config.protocol == 'R') || (config.protocol == 'X') || (config.protocol == 'T'))
    {
        printf("Telemetry . . . . . . . . = %4u  (TLM     = 0, 4, 8, 12 (for SDA0) )\n", config.pinTlm);
    }
    else
    {
        printf("Telemetry . . . . . . . . = %4u  (TLM     = 0, 1, 2, ..., 29)\n", config.pinTlm);       
    }
/* Add I2C Protocols */

    printf("GPS Rx  . . . . . . . . . = %4u  (GPS_RX  = 0, 1, 2, ..., 29)\n", config.pinGpsRx );
    printf("GPS Tx  . . . . . . . . . = %4u  (GPS_TX  = 0, 1, 2, ..., 29)\n", config.pinGpsTx );
    printf("Sbus OUT  . . . . . . . . = %4u  (SBUS_OUT= 0, 1, 2, ..., 29)\n", config.pinSbusOut );
    printf("RPM   . . . . . . . . . . = %4u  (RPM     = 0, 1, 2, ..., 29)\n", config.pinRpm );
    printf("SDA (I2C sensors) . . . . = %4u  (SDA     = 2, 6, 10, 14, 18, 22, 26)\n", config.pinSda );
    printf("SCL (I2C sensors) . . . . = %4u  (SCL     = 3, 7, 11, 15, 19, 23, 27)\n", config.pinScl );
    printf("PWM Channels 1, 2, 3 ,4   = %4u %4u %4u %4u (C1 / C16= 0, 1, 2, ..., 15)\n", config.pinChannels[0] , config.pinChannels[1] , config.pinChannels[2] , config.pinChannels[3]);
    printf("PWM Channels 5, 6, 7 ,8   = %4u %4u %4u %4u\n", config.pinChannels[4] , config.pinChannels[5] , config.pinChannels[6] , config.pinChannels[7]);
    printf("PWM Channels 9,10,11,12   = %4u %4u %4u %4u\n", config.pinChannels[8] , config.pinChannels[9] , config.pinChannels[10] , config.pinChannels[11]);
    printf("PWM Channels 13,14,15,16  = %4u %4u %4u %4u\n", config.pinChannels[12] , config.pinChannels[13] , config.pinChannels[14] , config.pinChannels[15]);
    printf("Voltage 1, 2, 3, 4        = %4u %4u %4u %4u (V1 / V4 = 26, 27, 28, 29)\n", config.pinVolt[0] , config.pinVolt[1], config.pinVolt[2] , config.pinVolt[3]);
    printf("RGB led . . . . . . . . . = %4u  (RGB    = 0, 1, 2, ..., 29)\n", config.pinLed);
    printf("Camera  P,R,PR,RR         = %4u %4u %4u %4u (PITCH, ROLL, PRATIO, RRATIO  = 1 to 16)\n", config.CamPitchChannel , config.CamRollChannel, config.CamPitchRatio , config.CamRollRatio);
    printf("Logger  . . . . . . . . . = %4u  (LOG    = 0, 1, 2, ..., 29)\n", config.pinLogger );
    printf("ESC . . . . . . . . . . . = %4u  (ESC_PIN= 0, 1, 2, ..., 29)\n", config.pinEsc );
    
    if (config.escType == HW4) {
        printf("\nEsc type is HW4 (Hobbywing V4)\n")  ;
    } else if (config.escType == HW3) {
        printf("\nEsc type is HW3 (Hobbywing V3)\n")  ;
    } else if (config.escType == KONTRONIK) {
        printf("\nEsc type is KON (Kontronik)\n")  ;
    } else {
        printf("\nEsc type is not defined\n")  ;
    }    

    watchdog_update(); //sleep_ms(500);
 
    if (config.protocol == 'S'){
            printf("\nProtocol is Sport (Frsky)\n")  ;
        } else if (config.protocol == 'C'){
            printf("\nProtocol is CRSF (=ELRS)\n")  ;
        } else if (config.protocol == 'J'){
            printf("\nProtocol is Jeti (non Exbus)\n")  ;    
        } else if (config.protocol == 'E'){
            printf("\nProtocol is Jeti (Exbus)\n")  ;    
        } else if (config.protocol == 'H'){
            printf("\nProtocol is Hott\n")  ;    
        } else if (config.protocol == 'M'){
            printf("\nProtocol is Mpx\n")  ;    
        } else if (config.protocol == 'I'){
            printf("\nProtocol is ibus(Flysky)\n")  ;    
        } else if (config.protocol == '2'){
            printf("\nProtocol is Sbus2(Futaba)\n")  ;    
        } else if (config.protocol == 'F'){
            printf("\nProtocol is Fbus(Frsky)\n")  ;    
        } else if (config.protocol == 'L'){
            printf("\nProtocol is SRXL2 (Spektrum)\n")  ; 
/* Add I2C Protocols */
        } else if (config.protocol == 'R'){
            printf("\nProtocol is RadioLink (use PRI and TLM as I2C0 port)\n")  ; 
        } else if (config.protocol == 'X'){
            printf("\nProtocol is Spektrum Xbus (use PRI and TLM as I2C0 port)\n")  ;
        } else if (config.protocol == 'T'){
            printf("\nProtocol is Hitec (use PRI and TLM as I2C0 port)\n")  ;
/* Add I2C Protocols */ 
        } else {
            printf("\nProtocol is unknow\n")  ;
        }
    printf("CRSF baudrate   = %" PRIu32 "\n", config.crsfBaudrate)  ;
    printf("Logger baudrate = %" PRIu32 "\n", config.loggerBaudrate)  ;
    printf("PWM is generated at = %i Hz\n", (int) config.pwmHz)  ;
    
    printf("\nVoltage parameters:\n")  ;
    printf("    Scales : %f , %f , %f , %f \n", config.scaleVolt1 , config.scaleVolt2 ,config.scaleVolt3 ,config.scaleVolt4 )  ;
    printf("    Offsets: %f , %f , %f , %f \n", config.offset1 , config.offset2 ,config.offset3 ,config.offset4 )  ;
    if ( config.pinVolt[2] !=255 && config.temperature == 1) {
        printf("    One temperature sensor is connected on V3\n");
    } else if (config.pinVolt[2] !=255 && config.pinVolt[3] !=255 && config.temperature == 2){
         printf("    Temperature sensors are connected on V3 and V4\n");
    } else {
        printf("    No temperature sensors are connected on V3 and V4\n");
    }
    printf("RPM multiplier = %f\n", config.rpmMultiplicator);
    if (baro1.baroInstalled) {
        printf("Baro sensor is detected using MS5611\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else if (baro2.baroInstalled) {
        printf("Baro sensor is detected using SPL06\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else if (baro3.baroInstalled) {
        printf("Baro sensor is detected using BMP280\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else {
        printf("Baro sensor is not detected\n")  ;
    }
    if(mpu.mpuInstalled){
        printf("Acc/Gyro is detected using MP6050\n")  ;
        printf("     Acceleration offsets X, Y, Z = %i , %i , %i\n", config.accOffsetX , config.accOffsetY , config.accOffsetZ);
        printf("     Gyro offsets         X, Y, Z = %i , %i , %i\n", config.gyroOffsetX , config.gyroOffsetY , config.gyroOffsetZ); 
    } else {
       printf("Acc/Gyro is not detected\n")  ;     
    }
    if (ms4525.airspeedInstalled) {
        printf("Aispeed sensor is detected using MS4525\n")  ;        
    } else if (sdp3x.airspeedInstalled) {
        printf("Airspeed sensor is detected using SDP3X\n")  ;
    } else {
        printf("Airspeed sensor is not detected\n")  ;
    } 
    if (config.VspeedCompChannel != 255){
        printf("    Vspeed compensation channel = %i\n", config.VspeedCompChannel);
    } else {
        printf("    No Vspeed compensation channel defined; oXs uses default settings\n");
    }
    if (adc1.adsInstalled) {
        printf("First analog to digital sensor is detected using ads1115\n")  ;
        printf("    Measurement setup: %i , %i , %i ,%i\n", ads_Measure[0][0], ads_Measure[0][1], ads_Measure[0][2], ads_Measure[0][3]) ;
        printf("    Gains: %i , %i , %i ,%i\n", ads_Gain[0][0], ads_Gain[0][1], ads_Gain[0][2], ads_Gain[0][3]) ;
        printf("    Rates: %i , %i , %i ,%i\n", ads_Rate[0][0], ads_Rate[0][1], ads_Rate[0][2], ads_Rate[0][3]) ;
        printf("    Offsets: %f , %f , %f ,%f\n", ads_Offset[0][0], ads_Offset[0][1], ads_Offset[0][2], ads_Offset[0][3]) ;
        printf("    Scales: %f , %f , %f ,%f\n", ads_Scale[0][0], ads_Scale[0][1], ads_Scale[0][2], ads_Scale[0][3]) ;
        printf("    Averaged on: %i , %i , %i ,%i\n", ads_MaxCount[0][0], ads_MaxCount[0][1], ads_MaxCount[0][2], ads_MaxCount[0][3]) ;
    } else {
        printf("First analog to digital sensor is not detected\n")  ;
    }
    if (adc2.adsInstalled) {
        printf("Second analog to digital sensor is detected using ads1115\n")  ;
        printf("    Measurement setup: %i , %i , %i ,%i\n", ads_Measure[1][0], ads_Measure[1][1], ads_Measure[1][2], ads_Measure[1][3]) ;
        printf("    Gains: %i , %i , %i ,%i\n", ads_Gain[1][0], ads_Gain[1][1], ads_Gain[1][2], ads_Gain[1][3]) ;
        printf("    Rates: %i , %i , %i ,%i\n", ads_Rate[1][0], ads_Rate[1][1], ads_Rate[1][2], ads_Rate[1][3]) ;
        printf("    Offsets: %f , %f , %f ,%f\n", ads_Offset[1][0], ads_Offset[1][1], ads_Offset[1][2], ads_Offset[1][3]) ;
        printf("    Scales: %f , %f , %f ,%f\n", ads_Scale[1][0], ads_Scale[1][1], ads_Scale[1][2], ads_Scale[1][3]) ;
        printf("    Averaged on: %i , %i , %i ,%i\n", ads_MaxCount[1][0], ads_MaxCount[1][1], ads_MaxCount[1][2], ads_MaxCount[1][3]) ;
    }  else {
        printf("Second analog to digital sensor is not detected\n")  ;
    } 

    if (config.gpsType == 'U'){
            printf("Foreseen GPS type is Ublox (configured by oXs) :")  ;
        } else if (config.gpsType == 'C'){
            printf("Foreseen GPS type is Ublox (configured externally) :")  ;
        } else if (config.gpsType == 'C'){
            printf("Foreseen GPS type is CADIS  :")  ;
        } else {
            printf("Foreseen GPS type is unknown  :")  ;
        }
    if (gps.gpsInstalled && gps.GPS_fix) {
        printf("GPS is detected and has a fix\n")  ;
    } else if (gps.gpsInstalled ) {
        printf("GPS is detected but has not (yet) a fix\n")  ;
    } else {
        printf("GPS is not (yet) detected\n")  ;
    }
    if (config.ledInverted == 'I'){
        printf("Led color is inverted\n")  ;
    } else {
        printf("Led color is normal (not inverted)\n")  ;
    }
    //if (config.gpio0 == 0){
    //    printf("GPIO0 is used to output a Sbus signal\n");
    //} else if ( config.gpio0 < 17 ){
    //    printf("GPIO0 generates channel %u\n", (unsigned int) config.gpio0);
    //} else {
    //    printf("GPIO0 : Error in configuration\n");
    //}
    //if (config.gpio1 > 0 && config.gpio1 < 17){
    //    printf("GPIO1 (and GPIO2, 3, 4) generates channel %u (and next)\n", (unsigned int) config.gpio1);
    //} else {
    //    printf("GPIO1 : Error in configuration\n");
    //}
    //if (config.gpio5 > 0 && config.gpio5 < 17){
    //    printf("GPIO5 (and GPIO6, 7, 8) generates channel %u (and next)\n", (unsigned int) config.gpio5);
    //} else {
    //    printf("GPIO5 : Error in configuration\n");
    //}
    //if (config.gpio11 > 0 && config.gpio11 < 17){
    //    printf("GPIO11 generates channel %u \n", (unsigned int) config.gpio11);
    //} else {
    //    printf("GPIO11 : Error in configuration\n");
    //}
    if ( config.failsafeType == 'H'){
        printf("Failsafe type is HOLD\n")  ;
    } else {
        printf("Failsafe uses predefined values\n")  ;
        printf("     Chan 1...4  = %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch0  )\
                                                        , (int) fmap( config.failsafeChannels.ch1 )\
                                                        , (int) fmap( config.failsafeChannels.ch2 )\
                                                        , (int) fmap( config.failsafeChannels.ch3 ) );
        printf("     Chan 5...8  = %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch4 )\
                                                        , (int) fmap( config.failsafeChannels.ch5 )\
                                                        , (int) fmap( config.failsafeChannels.ch6 )\
                                                        , (int) fmap( config.failsafeChannels.ch7 ) );
        printf("     Chan 9...12 = %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch8 )\
                                                        , (int) fmap( config.failsafeChannels.ch9 )\
                                                        , (int) fmap( config.failsafeChannels.ch10 )\
                                                        , (int) fmap( config.failsafeChannels.ch11 ) );
        printf("     Chan 13...16= %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch12 )\
                                                        , (int) fmap( config.failsafeChannels.ch13 )\
                                                        , (int) fmap( config.failsafeChannels.ch14 )\
                                                        , (int) fmap( config.failsafeChannels.ch15 ) );
    }    

    if ( config.CamPitchChannel !=255 && config.CamRollChannel !=255 && config.CamPitchRatio !=255 && config.CamRollRatio !=255) {
        printf("Stabilized Camera is ready\n");
    } else {
        printf("Stabilized Camera is not used\n");
    }
    watchdog_update(); //sleep_ms(500);
    printSequencers(); 
    checkConfigAndSequencers();
    //getTimerUs(0);        // print the time enlapsed is the function print.

    isPrinting = false;
} // end printConfigAndSequencers()


#define FLASH_CONFIG_OFFSET (256 * 1024)
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_CONFIG_OFFSET);

void saveConfig() {
    //sleep_ms(1000); // let some printf to finish
    uint8_t buffer[FLASH_PAGE_SIZE] ;
    memset(buffer, 0xff, FLASH_PAGE_SIZE);
    memcpy(&buffer[0], &config, sizeof(config));
    // Note that a whole number of sectors must be erased at a time.
    // irq must be disable during flashing
    watchdog_enable(3000 , true);
    if (multicoreIsRunning) multicore_lockout_start_blocking();
    uint32_t irqStatus = save_and_disable_interrupts();
    flash_range_erase(FLASH_CONFIG_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_CONFIG_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(irqStatus);
    if (multicoreIsRunning) multicore_lockout_end_blocking();
    //sleep_ms(1000);
    //printf("New config has been saved\n");
    //printConfig(); 
}

void cpyChannelsAndSaveConfig() {    // used when pressing boot button to save failsafe value
    config.failsafeType = 'C'; // remove 'H' for HOLD
    memcpy( &config.failsafeChannels , &sbusFrame.rcChannelsData, sizeof(config.failsafeChannels));
    saveConfig();        
}

void upperStr( char *p){
    if (p == NULL ) return;
    while ( *p != 0){
        *p = toupper(*p);
        p++;
    }    
}

char * skipWhiteSpace(char * str)
{
	char *cp = str;
	if (cp)
		while (isspace(*cp))
			++cp;
	return cp;
}

void removeTrailingWhiteSpace( char * str)
{
	if (str == nullptr)
		return;
	char *cp = str + strlen(str) - 1;
	while (cp >= str && isspace(*cp))
		*cp-- = '\0';
}

void setupConfig(){   // The config is uploaded at power on
    if (*flash_target_contents == CONFIG_VERSION ) {
        memcpy( &config , flash_target_contents, sizeof(config));
        if (config.pwmHz == 0XFFFF) config.pwmHz = _pwmHz; // set default value when it has not been defined manually
    } else {
        config.version = CONFIG_VERSION;
        config.pinChannels[0] = _pinChannels_1;
        config.pinChannels[1] = _pinChannels_2;
        config.pinChannels[2] = _pinChannels_3;
        config.pinChannels[3] = _pinChannels_4;
        config.pinChannels[4] = _pinChannels_5;
        config.pinChannels[5] = _pinChannels_6;
        config.pinChannels[6] = _pinChannels_7;
        config.pinChannels[7] = _pinChannels_8;
        config.pinChannels[8] = _pinChannels_9;
        config.pinChannels[9] = _pinChannels_10;
        config.pinChannels[10] = _pinChannels_11;
        config.pinChannels[11] = _pinChannels_12;
        config.pinChannels[12] = _pinChannels_13;
        config.pinChannels[13] = _pinChannels_14;
        config.pinChannels[14] = _pinChannels_15;
        config.pinChannels[15] = _pinChannels_16;
        config.pinGpsTx = _pinGpsTx;
        config.pinGpsRx = _pinGpsRx;
        config.pinPrimIn = _pinPrimIn;
        config.pinSecIn = _pinSecIn; 
        config.pinSbusOut = _pinSbusOut;
        config.pinTlm = _pinTlm;
        config.pinVolt[0] = _pinVolt_1;
        config.pinVolt[1] = _pinVolt_2;
        config.pinVolt[2] = _pinVolt_3;
        config.pinVolt[3] = _pinVolt_4;
        config.pinSda = _pinSda;
        config.pinScl = _pinScl;
        config.pinRpm = _pinRpm;
        config.pinLed = _pinLed;
        config.protocol = _protocol; // default = sport
        config.crsfBaudrate = _crsfBaudrate;
        config.scaleVolt1 = _scaleVolt1;
        config.scaleVolt2 = _scaleVolt2;
        config.scaleVolt3 = _scaleVolt3;
        config.scaleVolt4 = _scaleVolt4;
        config.offset1 = _offset1;
        config.offset2 = _offset2;
        config.offset3 = _offset3;
        config.offset4 = _offset4;
        config.gpsType = _gpsType ;
        config.rpmMultiplicator = _rpmMultiplicator;
        //config.gpio0 = 0;
        //config.gpio1 = 1;
        //config.gpio5 = 6;
        //config.gpio11 = 11;
        config.failsafeType = _failsafeType;
        config.failsafeChannels.ch0 = 1<<10 ; // set default failsafe value to 1/2 of 11 bits
        config.failsafeChannels.ch1 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch2 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch3 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch4 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch6 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch6 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch7 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch8 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch9 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch10 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch11 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch12 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch13 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch14 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch15 = config.failsafeChannels.ch0 ;
        config.accOffsetX = 0;
        config.accOffsetY = 0;
        config.accOffsetZ = 0;
        config.gyroOffsetX = 0;
        config.gyroOffsetY = 0;
        config.gyroOffsetZ= 0;
        config.temperature = _temperature;
        config.VspeedCompChannel = _VspeedCompChannel;
        config.ledInverted = _ledInverted; 
        config.CamPitchChannel = _CamPitchChannel;
        config.CamRollChannel = _CamRollChannel;
        config.CamPitchRatio = _CamPitchRatio;
        config.CamRollRatio = _CamRollRatio;   
        config.pinLogger = _pinLogger;
        config.loggerBaudrate =_loggerBaudrate;
        config.pinEsc = _pinEsc ;
        config.escType = _escType; 
        config.pwmHz = _pwmHz;
    }   
 
} 


void requestMpuCalibration()  // 
{
    if (!mpu.mpuInstalled) {
        printf("Calibration not done: no MP6050 installed\n");
        return ;
    }
    uint8_t data = 0X01; // 0X01 = execute calibration
    printf("Before calibration:");
    printConfigOffsets();
    sleep_ms(1000); // wait that message is printed
    queue_try_add(&qSendCmdToCore1 , &data);

}    

void printConfigOffsets(){
    printf("\nOffset Values in config:\n");
	printf("Acc. X = %d, Y = %d, Z = %d\n", (int) config.accOffsetX , (int) config.accOffsetY, (int) config.accOffsetZ);    
    printf("Gyro. X = %d, Y = %d, Z = %d\n", (int) config.gyroOffsetX , (int) config.gyroOffsetY, (int) config.gyroOffsetZ);
}

void printFieldValues(){
    printf("\n");
    // added aeropic
    printf("GYRO_X = %.7f degreesec\n", ((float) gyroX) );
    printf("GYRO_Y = %.7f degreesec\n", ((float) gyroY) );
    printf("\n");
    for (uint8_t i=0; i< NUMBER_MAX_IDX ;i++){
        if (fields[i].onceAvailable ){
            switch (i) {
                case LATITUDE:
                    printf("GPS Latitude = %.7f degree\n", ((float) fields[i].value) / 10000000.0);
                    break;
                case LONGITUDE:
                    printf("GPS Longitude = %.7f degree\n", ((float) fields[i].value) / 10000000.0);
                    break;
                case GROUNDSPEED:
                    printf("GPS Groundspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;
                case HEADING:
                    printf("GPS Heading = %f degree\n", ((float) fields[i].value) / 100.0) ;
                    break;
                case ALTITUDE:
                    printf("GPS Altitude = %d cm\n", (int) fields[i].value) ;
                    break;
                case NUMSAT:
                    printf("GPS Num sat. = %d\n", (int) fields[i].value) ;
                    break;
                case GPS_DATE:
                    printf("GPS Date J M A = %d %d %d \n", (uint8_t) (fields[i].value >>8) , (uint8_t) (fields[i].value >> 16) ,
                        (uint8_t) (fields[i].value >> 24) ) ;
                    break;
                case GPS_TIME:
                    printf("GPS Time H M S = %d %d %d \n", (uint8_t) (fields[i].value >>24) , (uint8_t) (fields[i].value >> 16) ,
                        (uint8_t) (fields[i].value >> 8) ) ;
                    break;
                case GPS_PDOP:
                    printf("GPS Pdop = %d \n", (int) fields[i].value) ;
                    break;
                case GPS_HOME_BEARING:
                    printf("GPS Home bearing = %d degree\n", (int) fields[i].value) ;
                    break;
                case GPS_HOME_DISTANCE:
                    printf("GPS Home distance = %d m\n", (int) fields[i].value) ;
                    break;
                case MVOLT:
                    printf("Volt 1 = %d mVolt\n", (int) fields[i].value) ;
                    break;
                case CURRENT:
                    printf("Current (Volt 2) = %d mA\n", (int) fields[i].value) ;
                    break;
                case RESERVE1:
                    printf("Volt 3 = %d mVolt\n", (int) fields[i].value) ;
                    break;
                case RESERVE2:
                    printf("Volt 4 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case CAPACITY:
                    printf("Capacity (using current) = %d mAh\n", (int) fields[i].value) ;
                    break;        
                case TEMP1:
                    printf("Temp 1 (Volt 3) = %d degree\n", (int) fields[i].value) ;
                    break;        
                case TEMP2:
                    printf("Temp 2 (Volt 4) = %d degree\n", (int) fields[i].value) ;
                    break;        
                case VSPEED:
                    printf("Vspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;        
                case RELATIVEALT:
                    printf("Baro Rel altitude = %d cm\n", (int) fields[i].value) ;
                    break;        
                case PITCH:
                    printf("Pitch = %f degree\n", (float) fields[i].value * 0.01) ;
                    break;        
                case ROLL:
                    printf("Roll = %f degree\n", (float) fields[i].value * 0.01) ;
                    break;        
                case YAW:
                    printf("Yaw = %f degree\n", (float) fields[i].value * 0.01);
                    break;        
                case RPM:
                    printf("RPM = %d Hertz\n", (int) fields[i].value) ;
                    break;
                case ADS_1_1:
                    printf("Ads 1 1 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_1_2:
                    printf("Ads 1 2 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_1_3:
                    printf("Ads 1 3 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_1_4:
                    printf("Ads 1 4 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_1:
                    printf("Ads 2 1 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_2:
                    printf("Ads 2 2 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_3:
                    printf("Ads 2 3 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_4:
                    printf("Ads 2 4 = %d mVolt\n", (int) fields[i].value) ;
                    break;
                case AIRSPEED:
                    printf("Airspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;
                case AIRSPEED_COMPENSATED_VSPEED:
                    printf("Compensated Vspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;
                case SBUS_HOLD_COUNTER:
                    printf("Sbus hold counter = %d\n", (int) fields[i].value) ;
                    break;
                case SBUS_FAILSAFE_COUNTER:
                    printf("Sbus failsafe counter = %d\n", (int) fields[i].value) ;
                    break;
                case GPS_CUMUL_DIST :
                    printf("Gps cumulative distance = %d\n", (int) fields[i].value) ;
                    break;
                case ACC_X :
                    printf("Acc X = %fg\n", (float) fields[i].value * 0.001) ;
                    break;
                case ACC_Y :
                    printf("Acc Y = %fg\n", (float) fields[i].value * 0.001) ;
                    break;
                case ACC_Z :
                    printf("Acc Z = %fg\n", (float) fields[i].value * 0.001) ;
                    break;
                            
            } // end switch
        }
    }
    if (config.VspeedCompChannel != 255){
        printf("Vspeed compensation = %.2f\n", dteCompensationFactor);
    }
    printf("pwmTop= %i",pwmTop);
}

void printPwmValues(){
    if ( lastRcChannels == 1){
        printf("PWM values are not available - no rc channels data have been received\n");
    } else {
        uint16_t c[16];
        c[0] = (uint16_t) sbusFrame.rcChannelsData.ch0 ;
        c[1] = (uint16_t) sbusFrame.rcChannelsData.ch1 ;
        c[2] = (uint16_t) sbusFrame.rcChannelsData.ch2 ;
        c[3] = (uint16_t) sbusFrame.rcChannelsData.ch3 ;
        c[4] = (uint16_t) sbusFrame.rcChannelsData.ch4 ;
        c[5] = (uint16_t) sbusFrame.rcChannelsData.ch5 ;
        c[6] = (uint16_t) sbusFrame.rcChannelsData.ch6 ;
        c[7] = (uint16_t) sbusFrame.rcChannelsData.ch7 ;
        c[8] = (uint16_t) sbusFrame.rcChannelsData.ch8 ;
        c[9] = (uint16_t) sbusFrame.rcChannelsData.ch9 ;
        c[10] = (uint16_t) sbusFrame.rcChannelsData.ch10 ;
        c[11] = (uint16_t) sbusFrame.rcChannelsData.ch11 ;
        c[12] = (uint16_t) sbusFrame.rcChannelsData.ch12 ;
        c[13] = (uint16_t) sbusFrame.rcChannelsData.ch13 ;
        c[14] = (uint16_t) sbusFrame.rcChannelsData.ch14 ;
        c[15] = (uint16_t) sbusFrame.rcChannelsData.ch15 ;
        printf("PWM values us (sbus) 1... 8 ");
        for (uint8_t i = 0; i<8; i++){
            printf(" %5d(%5d)", (int) fmap( c[i]  ), c[i]);
        }
        printf("\n");
        printf("PWM values us (sbus) 9...16 ");
        for (uint8_t i = 8; i<16; i++){
            printf(" %5d(%5d)", (int) fmap( c[i] ) , c[i]);
        }
        printf("\n");
        
    }
}

//********************************** Sequencer *****************************************
/*
For the sequencer we have to fill 2 tables
- one with 5 items per sequencer (key = "SEQ")
- one with 4 items per step (key = "STEP")
note : the table with steps can contain several sequences; 
we consider that a new sequence begins each time the first item (= channel range) changes
furthermore we consider that when the next first item is lower than the previous first item, then the steps become part of next sequencer
For each table, items are comma separated.
Each set of items starts with { and end with}
whitespaces are skipped
Whe define a function that takes 1 param : number of items to read (so e.g. {1,2,3,4})
the function read from a pointer up to a '0'(or the number of item) and return true if we find the right number of param.


*/

#define FLASH_SEQUENCER_OFFSET FLASH_CONFIG_OFFSET + (4 * 1024) // Sequencer is 4K after config parameters
const uint8_t *flash_sequencer_contents = (const uint8_t *) (XIP_BASE + FLASH_SEQUENCER_OFFSET);

uint8_t seqIdx = 0;        // count the sequencer
uint16_t sequenceIdx = 0;  // count the sequence
uint8_t stepIdx = 0;       // count the steps
SEQ_DEF seqDefsTemp[16];   // temporary structure to avoid any change to seq in case of error detected here
SEQ_STEP stepsTemp[SEQUENCER_MAX_NUMBER_OF_STEPS]; // temporary structure to avoid any change to seq in case of error detected here


void setupSequencers(){   // The config is uploaded at power on
    if (*flash_sequencer_contents == SEQUENCER_VERSION ) {
        memcpy( &seq , flash_sequencer_contents, sizeof(seq));
        seqDatasToUpload = true; // set a flag to update the table seqDatas[] when perfroming a checkSequencer()
        //printf("loaded param defsmax=%i  stepsMax=%i\n", seq.defsMax, seq.stepsMax); 
        //seq.defsMax = 0 ; // for testing only to be modified
        //seq.stepsMax = 0 ; // for testing only to be modified
    } else {
        seq.version = SEQUENCER_VERSION;
        seq.defsMax = 0 ;
        seq.stepsMax = 0 ; 
    }
} 

void checkSequencers(){
    // this function use flag (seqDatasToUpdate) to say if the table seqData has to be updated or not
    // it must be to be updated after a restart, a SEQ or a STEP command, not after a ENTER that only print the current config
    
    // set configIsValid = false when an error is detected
    watchdog_update(); //sleep_ms(500);
    if ( seq.defsMax == 0) {
        //printf("No sequencer defined\n");
        return ; // skip when sequencer are not defined
    }
    if (seq.stepsMax == 0) {
        printf("Error in sequencer steps: no steps defined while %i sequencers are defined\n", seq.defsMax);
        configIsValid = false;
        return;
    }
    uint8_t seqIdx = 0;  // index of current sequencer
    uint16_t sequenceIdx = 0; // index of current sequence
    uint16_t stepIdx = 0;   // index of current step
    uint16_t prevStepIdx = 0;
    uint8_t rangeNumber = 0; // used to check that each sequencer has at least 2 sequences 
    uint32_t currentSeqMillis = millisRp();
    CH_RANGE prevRange = seq.steps[stepIdx].chRange; 
    while (stepIdx < seq.stepsMax) {                     // process all steps
        //printf("Seq=%i   Step=%i  range=%i", (int) seqIdx+1, (int) stepIdx+1 , (int) rangeNumber );
        if (( seq.steps[stepIdx].nextSequencerBegin == 1 ) && (stepIdx > 0)) {   // When the next (not the first one) sequencer begin , close the current
            if ( rangeNumber < 2) {
                printf("Error in sequencer: only one sequence for sequencers %i\n", seqIdx+1);
                configIsValid = false;
                return;
            }
            if (seqDatasToUpload) {
                seqDatas[seqIdx].stepEndAtIdx = prevStepIdx;                 // store end of current sequencer
            }
            seqIdx++;                                                    // handle next sequencer
        }
        if (seq.steps[stepIdx].nextSequencerBegin == 1 ) {    // for each begin of sequencer 
            //if (seqIdx >= seq.defsMax) {
            //    printf("Error in sequencer: number of sequencers found in STEP exceeds number of sequencers defined in sequencer%i)\n",seq.defsMax );
            //    configIsValid = false;
            //    return;
            //}
            if (seqDatasToUpload) {
                // initilize seqDatas for new sequencer
                seqDatas[seqIdx].stepStartAtIdx = stepIdx;
                seqDatas[seqIdx].state = STOPPED;
                seqDatas[seqIdx].currentChValue = 0; // use a dummy channel value in order to force a change when a channel value will be received from Rx
                seqDatas[seqIdx].lastreceivedRange = dummy; // use a dummy channel value in order to force a change when a channel value will be received from Rx
                seqDatas[seqIdx].currentStepIdx = 0xFFFF;  // use a dummy value at startup (to detect when range )
                seqDatas[seqIdx].delayedStepIdx = 0xFFFF;  // use a dummy value to say that there is no delayed step.
                //seqDatas[seqIdx].lastActionAtMs = 0;
                seqDatas[seqIdx].lastOutputVal = seq.defs[seqIdx].defValue ; // set default value
                seqDatas[seqIdx].nextActionAtMs = 0; // 0 means that we have still to apply the default value      
            }
            rangeNumber = 1; // restat a new counting of the number of different RC values
            prevRange = seq.steps[stepIdx].chRange;    // set Prev equal to allow counting the number of different RC channel values (ranges) 
        }
        
        
        if ( seq.steps[stepIdx].chRange > prevRange) rangeNumber++; // Count the number of Rc channel values (range) for this sequencer
        
        if ( ( seq.steps[stepIdx].chRange < prevRange )  && (seq.steps[stepIdx].nextSequencerBegin == 0 )){  
            printf("Error in sequencer steps: in the same sequencer, Rc values of step n+1 (%i) must be >= to the value of step n\n", seqIdx+1);
            configIsValid = false;
            return;
        }
        if ( seq.defs[seqIdx].type == 1) { // when seq has type ANALOG, PWM value must be between 0/100  
            if (seq.steps[stepIdx].value < 0 || ( seq.steps[stepIdx].value > 100 && seq.steps[stepIdx].value != 127))  {
                printf("Error in sequencer steps: for sequencer %i  step %i, type is ANALOG(=1); PWM output value must then be in range 0/100 or 127(=stop)\n"\
                    ,seqIdx + 1 , stepIdx + 1 );
                configIsValid = false;
                return;
            }
        }
        prevRange = seq.steps[stepIdx].chRange ; 
        prevStepIdx = stepIdx;
        stepIdx++;
    } // end while
    if ( rangeNumber < 2) {
        printf("Error in sequencer steps: only one sequence for step item number %i\n",(int) stepIdx );
        configIsValid = false;
        return;
    }
    if( (seqIdx + 1) != seq.defsMax) {
        printf("Error in sequencer steps: number of sequencers (%i) detected in STEP do not match the number of sequencers in SEQ (%i)\n", seqIdx+1 , seq.defsMax);
        configIsValid = false;
        return;
    }
    if (seqDatasToUpload) {
        seqDatas[seqIdx].stepEndAtIdx = prevStepIdx; // for the last sequencer, register the last valid stepIdx
    }
    if (seq.steps[stepIdx].nextSequenceBegin == 1) sequenceIdx++; // count the number of sequence
    //sequencerIsValid =  true ; // no error in sequencer detected
    //printf("%i pins are controlled by a sequencer; setup is valid\n", seq.defsMax);   
    
    //#define DEBUG_PRINT_SEQDATAS
    #ifdef DEBUG_PRINT_SEQDATAS
        for (uint8_t i = 0; i<seq.defsMax;i++){
            printf("Start=%i End=%i chVal=%i state=%i range=%i first=%i currStep=%i smmoth=%i nextMs=%i outVal=%i\n",\
            seqDatas[i].stepStartAtIdx , seqDatas[i].stepEndAtIdx , (int) seqDatas[i].state , seqDatas[i].currentChValue , (int) seqDatas[i].currentRange, \
            seqDatas[i].firstStepIdx , seqDatas[i].currentStepIdx , seqDatas[i].smoothUpToMs ,\
            seqDatas[i].nextActionAtMs , seqDatas[i].lastOutputVal);
        }
        printf("\n");
    #endif
    seqDatasToUpload = false; // reset the flag asking for an update of table seqDatas[]
    watchdog_update(); //sleep_ms(500);
}

void printSequencers(){
    //printf("\nSequencer struct uses %i bytes\n", sizeof(seq));
    //printf("Sequencer def[] uses %i bytes\n", sizeof(seq.defs));
    //printf("Sequencer steps[] uses %i bytes\n", sizeof(seq.steps));
    watchdog_update(); //sleep_ms(500); // for tesing to be modified
    uint8_t seqIdx = 0; 
    if( seq.defsMax == 0 ){
        printf("\nNo sequencers are defined\n");
        return;
    }
    isPrinting = true;
    printf("\nNumber of: sequencers=%i   sequences=%i   steps= %i\n", seq.defsMax , seq.sequencesMax, seq.stepsMax);
    printf("Sequencer = [  Gpio  Type(0=servo,1=analog) Clock(msec) ChannelNr Default Min Max ]\n");
    printf("Sequence  = (  RC_value(-100...100)  to_Repeat  Uninterrupted   Only_priority_interrupted   is_a_Priority_seq  )\n");
    printf("Step      = {  Smooth(clocks) Pwm%(-100...100) Keep(clocks)  }\n");
    printf("SEQ=");
    for (uint16_t i = 0 ; i < seq.stepsMax; i++){
        // for each step, look if the step is the first of a sequencer
        if ( seq.steps[i].nextSequencerBegin == 1){
            printf("\n[ %i %i %i %i %i %i %i ] ", seq.defs[seqIdx].pin , (int) seq.defs[seqIdx].type , seq.defs[seqIdx].clockMs ,\
             seq.defs[seqIdx].channel , seq.defs[seqIdx].defValue , seq.defs[seqIdx].minValue , seq.defs[seqIdx].maxValue );
            seqIdx++;
        }
        if ( seq.steps[i].nextSequenceBegin == 1){
            printf("\n  ( %i", seq.steps[i].chRange);
            if (seq.steps[i].toRepeat == 1) { printf(" R");}
            if (seq.steps[i].neverInterrupted == 1) {printf(" U");}
            if (seq.steps[i].priorityInterruptOnly == 1) {printf(" O");}
            if (seq.steps[i].isPriority == 1) {printf(" P");}
            printf(" )\n    ");
        }
        printf("{%i %i %i} ", seq.steps[i].smooth , seq.steps[i].value , seq.steps[i].keep);             
    }
    printf("\n");    
    isPrinting = false;
}

void saveSequencers() {
    //sleep_ms(1000); // let some printf to finish
    //uint8_t buffer[FLASH_PAGE_SIZE] = {0xff};
    //memcpy(&buffer[0], &seq, sizeof(seq));
    // Note that a whole number of sectors must be erased at a time.
    // irq must be disable during flashing
    watchdog_enable(5000 , true);
    if ( multicoreIsRunning) multicore_lockout_start_blocking();
    uint32_t irqStatus = save_and_disable_interrupts();
    flash_range_erase(FLASH_SEQUENCER_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SEQUENCER_OFFSET,  (uint8_t*) &seq, sizeof(seq));
    //flash_range_program(FLASH_SEQUENCER_OFFSET,  buffer, FLASH_PAGE_SIZE);
    
    restore_interrupts(irqStatus);
    if (multicoreIsRunning) multicore_lockout_end_blocking();
    //sleep_ms(1000);
    //printf("New config has been saved\n");
    //printConfig(); 
}

bool getAllSequencers(){  // try to get sequencer definition from a string pointed by pvalue; return true if valid (then seq structure is modified)
                       // when true we still have to check if this is valid with config (for use of pins and with steps)
    seqIdx = 0;        // count the sequencer
    sequenceIdx = 0;  // count the sequence
    stepIdx = 0;       // count the steps
    
    while ( (*pvalue) != 0X00 ) { // while not end of value buffer
        if (seqIdx >= 16) {
            printf("Error : to many sequencer definitions; max is 16\n");
            return false; 
        }
        if ( parseOneSequencer() == false){ // this will also parse the sequences and the steps
            printf("Error converting the sequencer definition numner %i\n",seqIdx+1);
            return false; 
        }
        seqIdx++;    // count number of sequencers
    } // end while
    memcpy(&seq.defs , seqDefsTemp , sizeof(seqDefsTemp));  // copy all sequencer definitions
    memcpy(&seq.steps , stepsTemp, sizeof(stepsTemp));      // copy all sequence & steps definitions
    seq.defsMax = seqIdx;        // store the number of sequencers
    seq.sequencesMax = sequenceIdx ; // store the number of sequences
    seq.stepsMax = stepIdx;      // store the number of steps
    seqDatasToUpload = true;    // set a flag to force an upload of seqDatas[] during the check process.
    printf("Number of sequencers= %i   sequences=%i   steps=%i\n",seq.defsMax, seq.sequencesMax , seq.stepsMax);
    return true;
}


bool parseOneSequencer(){  // try to read a string  with n integer space separated set between [ ]
                            // and then try to get all sequences and steps for this sequencer
    char * ptr ;                        // get the pos of first non converted integer 
    pvalue =  skipWhiteSpace(pvalue);   // skip space at the begining
    if (( * pvalue) != '['){            // first char must be {
        printf("Error : sequencer must begin with [ \n");
        return false ;
    }
    nextSequencerBegin = 1;
    pvalue++;
    for (uint8_t i = 0 ; i < 7; i++) {       // try to convert 7 integers
        errno = 0;
        tempIntTable[i] =  strtol(pvalue , &ptr ,10);  // convert to integer starting from pvalue; ptr point to the first non converted char; skip whitespace before
                                                   // *ptr = 0 when no error
        if ( ( ptr == pvalue ) || ( ptr == NULL)) {
            printf("Error : parameter %i of sequencer %i can't be converted to an integers\n",i,seqIdx+1 );
            return false;    
        }
        pvalue = ptr; 
        //printf(" seq %i = %i\n", i , tempIntTable[i]);
    }
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != ']') {
        printf("Error : parameters of sequencer %i must end with ] after 7 values\n",seqIdx+1);
        return false ;
    }
    if (tempIntTable[0] < 0 || tempIntTable[0] > 15){
        printf("Error : for sequencer number %i, gpio must be in range 0 / 15\n", seqIdx+1);
        return false;
    }
    if (tempIntTable[1] < 0 || tempIntTable[1] > 1){
        printf("Error : for sequencer number %i, type must be 0 (SERVO) or 1(ANALOG)\n" , seqIdx+1);
        return false;
    }
    if (tempIntTable[2] < 20 || tempIntTable[2] > 100000){
        printf("Error : for sequencer number %i, clock must be in range 20 / 100000 (msec)\n", seqIdx+1);
        return false;
    }
    if (tempIntTable[3] < 1 || tempIntTable[3] > 16){
        printf("Error : for sequencer number %i, channel must be in range 1 / 16\n", seqIdx+1);
        return false;
    }
    if (tempIntTable[1] == 0) {  // for SERVO
        if (tempIntTable[4] < -125 || tempIntTable[4] > 125){
            printf("Error : for sequencer number %i, when type = SERVO, default PWM value must be in range -125 / +125\n", seqIdx+1);
            return false;
        }
    } else {                 // for ANALOG
        if (tempIntTable[4] < 0 || tempIntTable[4] > 100){
            printf("Error : for sequencer number %i, when type = ANALOG, default PWM value must be in range 0 / 100\n", seqIdx+1);
            return false;
        }
    }
    if (tempIntTable[1] == 0) {  // for SERVO
        if (tempIntTable[5] < -125 || tempIntTable[5] > 125){
            printf("Error : for sequencer number %i, when type = SERVO, min PWM value must be in range -125 / +125\n", seqIdx+1);
            return false;
        }
    } else {                 // for ANALOG
        if (tempIntTable[5] < 0 || tempIntTable[5] > 100){
            printf("Error : for sequencer number %i, when type = ANALOG, min PWM value must be in range 0 / 100\n", seqIdx+1);
            return false;
        }
    }if (tempIntTable[1] == 0) {  // for SERVO
        if (tempIntTable[6] < -125 || tempIntTable[6] > 125){
            printf("Error : for sequencer number %i, when type = SERVO, max PWM value must be in range -125 / +125\n", seqIdx+1);
            return false;
        }
    } else {                 // for ANALOG
        if (tempIntTable[6] < 0 || tempIntTable[6] > 100){
            printf("Error : for sequencer number %i, when type = ANALOG, max PWM value must be in range 0 / 100\n", seqIdx+1);
            return false;
        }
    }
    // here we have a valid set of parameter for a sequencer that we can save in a temp structure
    seqDefsTemp[seqIdx].pin = tempIntTable[0];
    seqDefsTemp[seqIdx].type = (SEQ_OUTPUT_TYPE) tempIntTable[1];
    seqDefsTemp[seqIdx].clockMs = tempIntTable[2];
    seqDefsTemp[seqIdx].channel = tempIntTable[3];
    seqDefsTemp[seqIdx].defValue = tempIntTable[4];
    seqDefsTemp[seqIdx].minValue = tempIntTable[5];
    seqDefsTemp[seqIdx].maxValue = tempIntTable[6];    
    pvalue++;
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != '(') {
        printf("Error : after [...] for sequencer %i there must a ( to define a sequence\n",seqIdx+1);
        return false ;
    }
    // parse all sequences and steps from one sequencer (up to the end or up to next sequencer)
    while ( (( * pvalue) != 0x00)  && (( * pvalue) != '[')) {    
        // for each sequence
        if ( parseOneSequence() == false ) {  // when true, stepsTemp will be filled for this sequencer
            return false;
        }
        sequenceIdx++;                       // prepare next sequence.
    }        
    return true;
}    

bool parseOneSequence() { // parse one sequence and all steps from this sequence (up to the end or up to next sequence or sequencer)
    // seqIdx, sequenceIdx, stepIdx and stepsTemp[] are used and updated
    char * ptr ;
    // look for one sequence and n steps    
    if(( * pvalue) != '(') {     // "(" is used to mark a new sequence
        printf("Error: expecting a sequence beginning with ( for sequencer %i\n", seqIdx+1);
        return false;
    }    
    nextSequenceBegin = 1;
    pvalue++;                       // skip '(' 
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces
    errno = 0;
    tempIntTable[0] =  strtol(pvalue , &ptr ,10);  // convert Rc value to integer 
                                                    // *ptr = 0 when no error
    if ((ptr == pvalue) || ( ptr == NULL)) {
        printf("Error: for sequence %i, parameter RC channel value can't be converted to an integer\n",sequenceIdx+1);
        return false;    
    }
    pvalue = ptr;
    if (tempIntTable[0] < -100 || tempIntTable[0] > 100){
        printf("Error: for sequence %i, Rc channel value must in range -100...100 (included)\n" , sequenceIdx+1);
        return false;    
    }
    if (( tempIntTable[0] % 10) != 0){
        printf("Error: for sequence %i, Rc channel value must be a multiple of 10 (10, 20, ...100, -10, -20,...-100\n" , sequenceIdx+1);
        return false;    
    }
    
    
    tempIntTable[1] = 0; // set 4 optional flags to 0
    tempIntTable[2] = 0;
    tempIntTable[3] = 0;
    tempIntTable[4] = 0;
    for (uint8_t i=0;i<4;i++){ // search for a R, U O or P
        pvalue =  skipWhiteSpace(pvalue);   // skip next spaces
        if (( * pvalue) == 'R' ){
            tempIntTable[1]=1;
        } else if (( * pvalue) == 'U' ){
            tempIntTable[2]=1;
        } else if (( * pvalue) == 'O' ){
            tempIntTable[3]=1;
        } else if (( * pvalue) == 'P' ){
            tempIntTable[4]=1;
        } else if (( * pvalue) == ')' ){
            break;
        } else {
            printf("Error: sequence definition %i contains an invalid optional character (can only be R,U,O and/or P)\n", sequenceIdx+1);
            return false;
        }
        pvalue++;
        pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    } // end for (4 optional sequence param)
    if ( (tempIntTable[2]==1) && (tempIntTable[3]==1)){
        printf("Error: in sequence definition %i: options U and O may not be used toegether\n",sequenceIdx+1);
        return false;    
    }
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    if (( * pvalue) != ')'){            // sequence must end with )
        printf("Error: sequence definition %i must have a ) after max 4 optionnal characters\n",sequenceIdx+1);
        return false ;
    }
    pvalue++; // skip )
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    if (( * pvalue) != '{'){            // a step befinning with { must exist after a sequence 
        printf("Error: after sequence definition %i we must have a steps beginning with {\n",sequenceIdx+1);
        return false ;
    }
    // parse one or several steps
    while ( (( * pvalue) != 0X00) && (( * pvalue) != '(') && (( * pvalue) != '[')) {     // get one or several steps
        if (parseOneStep() == false) {  // data are stored in stepsTemp[] 
            return false;
        }
        if (stepIdx == SEQUENCER_MAX_NUMBER_OF_STEPS) {
            printf("Error: to many steps; maximum is %i\n",SEQUENCER_MAX_NUMBER_OF_STEPS);
        return false ;
        }
        stepIdx++;   // prepare for next step
    }
    return true;
}

bool parseOneStep(){
    char * ptr;
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    if (( * pvalue) != '{'){            // first char must be {
        printf("Error: in sequence %i, each step parameters must begin with { ; step=%i\n",sequenceIdx+1, stepIdx+1);
        return false ;
    }
    pvalue++;
    for (uint8_t i = 5 ; i < 8; i++) {
        errno = 0;
        tempIntTable[i] =  strtol(pvalue , &ptr ,10);  // convert to integer starting from pvalue; ptr point to the first non converted char; skip whitespace before
                                                // *ptr = 0 when no error
        if( ( ptr == pvalue ) || (ptr == NULL) ){
            printf("Error : for step %i, parameter %i can't be converted to an integer\n", stepIdx+1, i+1);
            return false;    
        }
        pvalue = ptr; 
        //printf(" seq %i = %i\n", i , tempIntTable[i]);
    }  // end for 
    // Check each of the 3 parameters                
    if (tempIntTable[5] < 0 || tempIntTable[5] > 255){
        printf("Error: for step %i, smooth must be in range 0 / 255 (included)\n" , stepIdx+1);
        return false;
    }
    if (tempIntTable[6] < -125 || tempIntTable[6] > 127 || tempIntTable[6] == 126)  {
        printf("Error: for step number %i, output value must be in range -125 / 125 (included) or 127 (stop at current position)\n", stepIdx+1);
        return false;
    }
    if (tempIntTable[5] >0 && tempIntTable[6] == 127)  {
        printf("Error: for step number %i, smooth must be 0 when output value is 127 (stop at current position)\n", stepIdx+1);
        return false;
    }
    if (tempIntTable[7] < 0 || tempIntTable[7] > 255){
        printf("Error: for step number %i, keep must be in range 0 / 255 (included)\n" , stepIdx+1);
        return false;
    }
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != '}') {
        printf("Error : for step %i, group of parameters must end with } after 3 values\n",stepIdx+1);
        return false ;
    }
    pvalue++;
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    // here we have a valid step (that includes also the sequence parameters)
    // all parameters of one step have been processed and are valid; store them in temp
    stepsTemp[stepIdx].chRange = ( CH_RANGE) tempIntTable[0];
    stepsTemp[stepIdx].toRepeat = tempIntTable[1];
    stepsTemp[stepIdx].neverInterrupted =  tempIntTable[2];
    stepsTemp[stepIdx].priorityInterruptOnly =  tempIntTable[3];
    stepsTemp[stepIdx].isPriority =  tempIntTable[4];
    stepsTemp[stepIdx].smooth = tempIntTable[5];
    stepsTemp[stepIdx].value = tempIntTable[6];
    stepsTemp[stepIdx].keep = tempIntTable[7];
    stepsTemp[stepIdx].nextSequencerBegin = nextSequencerBegin;
    stepsTemp[stepIdx].nextSequenceBegin = nextSequenceBegin; 
    nextSequencerBegin = 0; // reset the flags
    nextSequenceBegin = 0;
    //printf("rc val= %i  R=%i  U=%i  O=%i  P=%i sm=%i pos=%i ke=%i nrb=%i nsb=%i\n", \
    //            stepsTemp[stepIdx].chRange , stepsTemp[stepIdx].toRepeat , stepsTemp[stepIdx].neverInterrupted , \
    //            stepsTemp[stepIdx].priorityInterruptOnly , stepsTemp[stepIdx].isPriority , stepsTemp[stepIdx].smooth,\
    //            stepsTemp[stepIdx].value , stepsTemp[stepIdx].keep ,\
    //            stepsTemp[stepIdx].nextSequencerBegin , stepsTemp[stepIdx].nextSequenceBegin );
    return true; 
}  // end of handling one step; 


/*
bool parseOneSequencer(){  // try to read a string  with n integer space separated set between { }
    char * ptr ;                        // get the pos of first non converted integer 
    pvalue =  skipWhiteSpace(pvalue);   // skip space at the begining
    if (( * pvalue) != '['){            // first char must be {
        printf("Error : group of 7 values must begin with { \n");
        return false ;
    }
    pvalue++;
    for (uint8_t i = 0 ; i < 7; i++) {
        errno = 0;
        tempIntTable[i] =  strtol(pvalue , &ptr ,10);  // convert to integer starting from pvalue; ptr point to the first non converted char; skip whitespace before
                                                   // *ptr = 0 when no error
        if ( ( ptr == pvalue ) || ( ptr == NULL)) {
            printf("Error : parameter %i of one sequencer can't be converted to an integers\n",i);
            return false;    
        }
        pvalue = ptr; 
        //printf(" seq %i = %i\n", i , tempIntTable[i]);
    }
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != ']') {
        printf("Error : group of parameter of a sequencer must end with ] after %7 values \n");
        return false ;
    }


    pvalue++;
    return true;
}    
bool parseOneStep(){  
    // try to read a string with 
    //    a "+" (optionnal) as first char to identify a new sequencer
    //    a "/" (optionnal) as first char to identify a new sequence
    //          then 5 parameters space separated (Rc value, to repeat, never interrupt, priority interrupt only, is priority)
    //    then (mandatory) a "{" followed by
    //           3 integers space separated
    //           "}""
    // return true if OK
    char * ptr ; 
    nextSequencerBegin = 0;
    nextSequenceBegin = 0;
    pvalue =  skipWhiteSpace(pvalue);   // skip space at the begining
    if (( * pvalue) == '+'){            //  A "+" before "{" marks the first step of next sequencer
        pvalue++;                       // skip + before { (used to separate sequencer when we print the definition of steps)
        nextSequencerBegin = 1;      // 1 indicates that the next sequencer begins now
        //printf("new sequencer\n");
        pvalue =  skipWhiteSpace(pvalue);   // skip next spaces 
    }
    if(( * pvalue) == 'S') {     // "S" is used to mark the first step of a new sequence
        nextSequenceBegin = 1;     //
        //printf("new sequence\n");
        pvalue++;                       // skip - before { (used to separate sequencer when we print the definition of steps)
        pvalue =  skipWhiteSpace(pvalue);   // skip next spaces
        errno = 0;
        tempIntTable[0] =  strtol(pvalue , &ptr ,10);  // convert Rc value to integer 
                                                    // *ptr = 0 when no error
        if ((ptr == pvalue) || ( ptr == NULL)) {
            printf("Error: for a sequence, parameter RC channel value can't be converted to an integer\n");
            return false;    
        }
        pvalue = ptr;
        pvalue =  skipWhiteSpace(pvalue);   // skip next spaces
        tempIntTable[1] = 0; // set 4 optional flags to 0
        tempIntTable[2] = 0;
        tempIntTable[3] = 0;
        tempIntTable[4] = 0;
        for (uint8_t i=0;i<4;i++){ // search for a R, U O or P
            pvalue =  skipWhiteSpace(pvalue);   // skip next spaces
            if (( * pvalue) == 'R' ){
                tempIntTable[1]=1;
            } else if (( * pvalue) == 'U' ){
                tempIntTable[2]=1;
            } else if (( * pvalue) == 'O' ){
                tempIntTable[3]=1;
            } else if (( * pvalue) == 'P' ){
                tempIntTable[4]=1;
            } else if (( * pvalue) == '{' ){
                break;
            } else {
                printf("Error: sequence definition contains an invalid character (can only be R,U,O and/or P)\n");
                return false;
            }
            pvalue++;
            pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
        }
        if ( (tempIntTable[2]==1) && (tempIntTable[3]==1)){
            printf("Error: in sequence definition options U and O may not be used toegether)\n");
            return false;    
        }
    }
    if (( * pvalue) != '{'){            // first char must be {
        printf("Error: step parameters must begin with { (after some optionnal other parameters)\n");
        return false ;
    }
    pvalue++;
    for (uint8_t i = 5 ; i < 8; i++) {
        errno = 0;
        tempIntTable[i] =  strtol(pvalue , &ptr ,10);  // convert to integer starting from pvalue; ptr point to the first non converted char; skip whitespace before
                                                   // *ptr = 0 when no error
        if( ptr == pvalue ) {
            printf("Error : for a step, parameter %i can't be converted to an integer\n", (int) (i+1));
            return false;    
        }
        if ( ptr == NULL){
            printf("Error : for a step, parameter %i can't be converted to an integer\n", (int) (i+1));
            return false;
        }
        pvalue = ptr; 
        //printf(" seq %i = %i\n", i , tempIntTable[i]);
    }
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != '}') {
        printf("Error : for a step, group of parameters must end with } after 3 values \n");
        return false ;
    }
    pvalue++;
    return true;
}    
*/


/*
bool getSequencers(){  // try to get sequencer definition from a string pointed by pvalue; return true if valid (then seq structure is modified)
                       // when true we still have to check if this is valid with config (for use of pins and with steps)
    uint8_t seqIdx = 0;        // count the sequencer
    SEQ_DEF seqDefsTemp[16];   // temporary structure to avoid any change to seq in case of error detected here
    //bool isSeqValid = false;
    while ( (*pvalue) != 0X00 ) { // while not end of value buffer
        if (seqIdx >= 16) {
            printf("Error : to many sequencer definitions; max is 16\n");
            return false; 
        }
        if ( parseOneSequencer() == false){
            printf("Error converting the sequencer definition numner %i\n",seqIdx+1);
            return false; 
        }
        pvalue = skipWhiteSpace(pvalue);
        if (tempIntTable[0] < 0 || tempIntTable[0] > 15){
            printf("Error : for sequencer number %i, gpio must be in range 0 / 15\n", seqIdx+1);
            return false;
        }
        if (tempIntTable[1] < 0 || tempIntTable[1] > 1){
            printf("Error : for sequencer number %i, type must be 0 (SERVO) or 1(ANALOG)\n" , seqIdx+1);
            return false;
        }
        if (tempIntTable[2] < 20 || tempIntTable[2] > 100000){
            printf("Error : for sequencer number %i, clock must be in range 20 / 100000 (msec)\n", seqIdx+1);
            return false;
        }
        if (tempIntTable[3] < 1 || tempIntTable[3] > 16){
            printf("Error : for sequencer number %i, channel must be in range 1 / 16\n", seqIdx+1);
            return false;
        }
        if (tempIntTable[1] == 0) {  // for SERVO
            if (tempIntTable[4] < -125 || tempIntTable[4] > 125){
                printf("Error : for sequencer number %i, when type = SERVO, default PWM value must be in range -125 / +125\n", seqIdx+1);
                return false;
            }
        } else {                 // for ANALOG
            if (tempIntTable[4] < 0 || tempIntTable[4] > 100){
                printf("Error : for sequencer number %i, when type = ANALOG, default PWM value must be in range 0 / 100\n", seqIdx+1);
                return false;
            }
        }
        if (tempIntTable[1] == 0) {  // for SERVO
            if (tempIntTable[5] < -125 || tempIntTable[5] > 125){
                printf("Error : for sequencer number %i, when type = SERVO, min PWM value must be in range -125 / +125\n", seqIdx+1);
                return false;
            }
        } else {                 // for ANALOG
            if (tempIntTable[5] < 0 || tempIntTable[5] > 100){
                printf("Error : for sequencer number %i, when type = ANALOG, min PWM value must be in range 0 / 100\n", seqIdx+1);
                return false;
            }
        }if (tempIntTable[1] == 0) {  // for SERVO
            if (tempIntTable[6] < -125 || tempIntTable[6] > 125){
                printf("Error : for sequencer number %i, when type = SERVO, max PWM value must be in range -125 / +125\n", seqIdx+1);
                return false;
            }
        } else {                 // for ANALOG
            if (tempIntTable[6] < 0 || tempIntTable[6] > 100){
                printf("Error : for sequencer number %i, when type = ANALOG, max PWM value must be in range 0 / 100\n", seqIdx+1);
                return false;
            }
        }
        seqDefsTemp[seqIdx].pin = tempIntTable[0];
        seqDefsTemp[seqIdx].type = (SEQ_OUTPUT_TYPE) tempIntTable[1];
        seqDefsTemp[seqIdx].clockMs = tempIntTable[2];
        seqDefsTemp[seqIdx].channel = tempIntTable[3];
        seqDefsTemp[seqIdx].defValue = tempIntTable[4];
        seqDefsTemp[seqIdx].minValue = tempIntTable[5];
        seqDefsTemp[seqIdx].maxValue = tempIntTable[6];
        seqIdx++;
    } // end while
    memcpy(&seq.defs , seqDefsTemp , sizeof(seqDefsTemp));
    seq.defsMax = seqIdx;
    seqDatasToUpload = true;    // set a flag to force an upload of seqDatas[] during the check process.
    printf("Number of sequencers= %i\n",seq.defsMax);

    return true;
}

bool getStepsSequencers(){ // try to get all steps decoding a string pointed by pvalue
                            // return true when steps are valid (syntax and each value); seq structure is then updated
                            // we still have to check if this is valid compared to the number of sequencers
    uint8_t sequencerIdx = 0;  // count the number of sequencer
    uint16_t sequenceIdx = 0;  // count the sequence
    uint8_t stepIdx = 0;       // count the steps
    SEQ_STEP stepsTemp[SEQUENCER_MAX_NUMBER_OF_STEPS]; 
    //bool isStepValid = false;
    while ( (*pvalue) != 0X00 ) { // while not end of value buffer
        if (stepIdx >= SEQUENCER_MAX_NUMBER_OF_STEPS) {
            printf("Error : to many steps; max is %i\n",SEQUENCER_MAX_NUMBER_OF_STEPS);
            return false; 
        }
        if ( parseOneStep() == false){
            printf("Error converting the step numner %i\n",stepIdx+1);
            return false; 
        }
        pvalue = skipWhiteSpace(pvalue);
        //if (tempIntTable[0]!=-100 && tempIntTable[0]!=-75 && tempIntTable[0]!=-50 && tempIntTable[0]!=-25\
        //    && tempIntTable[0]!=100 && tempIntTable[0]!=75 && tempIntTable[0]!=50 && tempIntTable[0]!=25 && tempIntTable[0]!=0){
        //    printf("Error : for step number %i, channel range must be -100/-75/-50/-25/0/25/50/75/100\n", stepIdx+1);
        //    return false;
        //}
        if ((nextSequencerBegin == 1) && (nextSequenceBegin == 0)) {
            printf("Error: when a new sequence begins (with a +), a new sequence must exist (with a /)");
            return false;    
        }
        if ( nextSequencerBegin == 1){ // for a new sequencer
            sequencerIdx++; // count the number of sequencers
        }
        if ( nextSequenceBegin == 1){ // for a sequence, fields are : channel value, toRepeat, neverInterrupted , priorityInterruptOnly ,isPriority
            sequenceIdx++;       // count the sequences
            if (tempIntTable[0] < -100 || tempIntTable[0] > 100){
                printf("Error: for sequence number %i, channel value must in range -100...100 (included)\n" , sequenceIdx);
                return false;    
            }
            if (( tempIntTable[0] % 10) != 0){
                printf("Error: for step number %i, channel value must be a multiple of 10 (10, 20, ...100, -10, -20,...-100\n" , stepIdx+1);
                return false;    
            }
            if (tempIntTable[1] < 0 || tempIntTable[1] > 1){
                printf("Error: for sequence number %i, the first flag must be 0 (no repeat) or 1 (repeat sequence)\n" , sequenceIdx);
                return false;    
            }
            if (tempIntTable[2] < 0 || tempIntTable[2] > 1){
                printf("Error: for sequence number %i, the second flag must be 0 (may be interrupted) or 1 (never interrupted)\n" , sequenceIdx);
                return false;    
            }
            if (tempIntTable[3] < 0 || tempIntTable[3] > 1){
                printf("Error: for sequence number %i, the third flag must be 0 (interrupted by other seq.) or 1 (interrupted only by priority seq.)\n" , sequenceIdx);
                return false;    
            }
            if (tempIntTable[4] < 0 || tempIntTable[4] > 1){
                printf("Error: for sequence number %i, the fourth flag must be 0 (not a priority seq.) or 1 (is a priority seq.)\n" , sequenceIdx);
                return false;    
            }
        }

        if (tempIntTable[5] < 0 || tempIntTable[5] > 255){
            printf("Error: for step number %i, smooth must be in range 0 / 255 (included)\n" , stepIdx+1);
            return false;
        }
        if (tempIntTable[6] < -125 || tempIntTable[6] > 125){
            printf("Error: for step number %i, output value must be in range -125 / 125 (included)\n", stepIdx+1);
            return false;
        }
        if (tempIntTable[7] < 0 || tempIntTable[7] > 255){
            printf("Error: for step number %i, keep must be in range 0 / 255 (included)\n" , stepIdx+1);
            return false;
        }
        stepsTemp[stepIdx].chRange = ( CH_RANGE) tempIntTable[0];
        stepsTemp[stepIdx].toRepeat = tempIntTable[1];
        stepsTemp[stepIdx].neverInterrupted =  tempIntTable[2];
        stepsTemp[stepIdx].priorityInterruptOnly =  tempIntTable[3];
        stepsTemp[stepIdx].isPriority =  tempIntTable[4];
        stepsTemp[stepIdx].smooth = tempIntTable[5];
        stepsTemp[stepIdx].value = tempIntTable[6];
        stepsTemp[stepIdx].keep = tempIntTable[7];
        stepsTemp[stepIdx].nextSequencerBegin = nextSequencerBegin;
        stepsTemp[stepIdx].nextSequenceBegin = nextSequenceBegin; 
        //printf("rc val= %i  R=%i  U=%i  O=%i  P=%i sm=%i pos=%i ke=%i nrb=%i nsb=%i\n", \
        //            stepsTemp[stepIdx].chRange , stepsTemp[stepIdx].toRepeat , stepsTemp[stepIdx].neverInterrupted , \
        //            stepsTemp[stepIdx].priorityInterruptOnly , stepsTemp[stepIdx].isPriority , stepsTemp[stepIdx].smooth,\
        //            stepsTemp[stepIdx].value , stepsTemp[stepIdx].keep ,\
        //            stepsTemp[stepIdx].nextSequencerBegin , stepsTemp[stepIdx].nextSequenceBegin );
        stepIdx++;
    } // end while
    //isStepValid = true;
    memcpy(&seq.steps , &stepsTemp , sizeof(stepsTemp));
    seq.stepsMax = stepIdx;
    seq.sequencesMax = sequenceIdx;
    seqDatasToUpload = true;    // set a flag to force an upload of seqDatas[] during the check process.
    printf( "In STEP, number of: sequencers= %i   sequences= %i   steps= %i\n", sequencerIdx , sequenceIdx , stepIdx);
    return true; 
}
*/