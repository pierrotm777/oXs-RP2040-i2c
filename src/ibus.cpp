#include <stdio.h>
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "uart_ibus_tx_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "tools.h"
#include "gps.h"
#include "param.h"
#include <inttypes.h> // used by PRIu32
#include "ibus.h"
#include "ads1115.h"

// one pio and 2 state machines are used to manage the ibus in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When 4  bytes are received, it is a frame with 
//        - first byte = number of bytes = 04
//        - second = a command (+ adress)
//        - 3 and 4 = checksum
//    Depending on the command, there are 3 types of frames to reply
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   

// Rx sent on uart (115200 baud 8N1 not inverted) a frame of 4 char 
// When the second char is 0x8y (starting with 0X81) , it means that the RX ask if sensor y exist.
//    When sensor exists, oXs must reply after a delay of 100 usec, sending back the same char
//    Note : when oXs replies to a 0X8y cmd, Rx send only a cmd 0X9y as long it does not get a reply to his 0x9y cmd
//           so the receiver does not skip to next adr or to 0xAy commands 
// When the second char is 0x9y, it means that the sensor is asking the type of sensor.
//    oXs must reply with a frame of 6 bytes:
//           - first  = 0X06 = number of bytes
//           - second = 0X9y = the received byte
//           - 3d =  sensor type (code from iBus) for y sensor 
//           - 4th = size for this type (2 or 4)
//           - 5/6 = checksum
// When the second char is 0xAy, it means that oXs has to send a frame with the value
//    oXs must reply with a frame of 6 or 8 bytes
//           - length of the frame (6 or 8)
//           - 0XAy (as received)
//           - value (in 2 or 4 bytes)
//           - checksum

//List of types (code specified by ibus)



queue_t ibusRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO ibusPio = pio0;
uint ibusSmTx = 0; // to send the telemetry to sport
uint ibusSmRx = 1; // to get the request from sport
uint ibusOffsetTx ; 
uint ibusOffsetRx ; 

// dma channel is used to send ibus telemetry without blocking
int ibus_dma_chan;
dma_channel_config ibusDmaConfig;

uint8_t ibusTxBuffer[20];

enum IBUSSTATES {
    RECEIVING,
    //WAIT_FOR_SENDING,
    //SENDING,
    WAIT_END_OF_SENDING
};

uint8_t listOfIbusFields[16] ; // list of fields that can be provided (based on sensor being discovered)
uint8_t maxIbusFieldsIdx ;   // max number of fields that can be provided
int32_t ibusValue; // store a value to be transmitted in ibus format    

uint32_t lastIbusReceivedUs = 0;  // used to check the delay between char. 
uint32_t ibusStartWaiting = 0; // timestamp (usec) when we start waiting 
IBUSSTATES ibusState;

extern field fields[];  // list of all telemetry fields and parameters used by Sport
extern MS5611 baro1;
extern SPL06 baro2;
extern BMP280 baro3;
extern GPS gps;
extern CONFIG config;
extern ADS1115 adc1 ;     // class to handle first ads1115 (adr pin connected to grnd)
extern ADS1115 adc2 ;     // class to handle second ads1115 (adr pin connected to vdd)
extern uint8_t forcedFields; // FVP/FVN/FVPOWER simulation state
const uint16_t * getForcedCellValues(uint8_t forcedFields);


#define IBUS_BAUDRATE 115200

// Internal pseudo-fields used only by the oXs iBUS dispatcher.
// The sensor IDs sent on the bus remain the native AFHDS2A IDs used by EdgeTX.
#define IBUS_OXS_GPS_FULL_FIELD  0xD0U
#define IBUS_OXS_VOLT_FULL_FIELD 0xD1U
#define IBUS_OXS_ACC_FULL_FIELD  0xD2U
#define IBUS_OXS_CELL0_DUMMY     0xDFU
#define IBUS_OXS_CELL1_FIELD     0xE0U
#define IBUS_OXS_CELL6_FIELD     (IBUS_OXS_CELL1_FIELD + 5U)

uint8_t ibusTypes[NUMBER_MAX_IDX] = {  // list of ibus type in the same sequence as fieldId 
      IBUS_SENSOR_TYPE_GPS_LAT,    // GPS LATITUDE ,  //   GPS special format
      IBUS_SENSOR_TYPE_GPS_LON,    //  GPS LONGITUDE,    //   special format
      IBUS_SENSOR_TYPE_GROUND_SPEED, //GROUNDSPEED , //   GPS km/h (previously cm/s)
      IBUS_SENSOR_TYPE_COG,          //HEADING,      //    GPS 0.01 degree
      IBUS_SENSOR_TYPE_GPS_ALT,      //ALTITUDE ,    //   GPS cm

      IBUS_SENSOR_TYPE_GPS_STATUS,   //NUMSAT ,      //    GPS no unit   
      IBUS_SENSOR_TYPE_UNKNOWN,      // GPS_DATE ,    //  ?????????????? GPS special format AAMMJJFF
      IBUS_SENSOR_TYPE_UNKNOWN,      // GPS_TIME ,    //  ?????????????? GPS special format HHMMSS00
      IBUS_SENSOR_TYPE_UNKNOWN,      //GPS_PDOP ,    //  ?????????????? GPS no unit
      IBUS_SENSOR_TYPE_CMP_HEAD,     // GPS_HOME_BEARING, //   GPS degree

      IBUS_SENSOR_TYPE_GPS_DIST,     //GPS_HOME_DISTANCE, //   GPS  in m
      IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,     //MVOLT,             //    volt1   in mVolt
      IBUS_SENSOR_TYPE_BAT_CURR,     //CURRENT,           // IBUS_SENSOR_TYPE_BAT_CURR  volt2 must be in seq for voltage.cpp in mA (mV)
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE1, //  ???? volt3 must be in seq for voltage.cpp in mV
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE2, //  ???? volt4 must be in seq for voltage.cpp in mV
      
      IBUS_SENSOR_TYPE_FUEL,         //CAPACITY,    // IBUS_SENSOR_TYPE_FUEL    based on current (volt2) in mAh
      IBUS_SENSOR_TYPE_TEMPERATURE,  //TEMP1,       //  = Volt3 but saved as temp in degree
      IBUS_SENSOR_TYPE_TEMPERATURE,  //TEMP2,       // IBUS_SENSOR_TYPE_TEMPERATURE =  Volt4 but saved as temp in degree
      IBUS_SENSOR_TYPE_CLIMB_RATE,   //VSPEED,      //     baro       in cm/s
      IBUS_SENSOR_TYPE_ALT,          //RELATIVEALT , //       baro      in cm
      
      IBUS_SENSOR_TYPE_PITCH,        //PITCH,       //      imu        in degree 
      IBUS_SENSOR_TYPE_ROLL,         //ROLL,        //       imu           in degree
      IBUS_SENSOR_TYPE_UNKNOWN,      //YAW ,        // not used to save data  in degree
      IBUS_SENSOR_TYPE_RPM,          //RPM ,        //      RPM sensor    in rpm
      IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,      //ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 1

      IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,      //ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    25
      IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,      //ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3
      IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,      //ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2
      
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    30
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_4,      // Voltage provided by ads1115 nr 2 on pin 4 
      IBUS_SENSOR_TYPE_UNKNOWN,      //AIRSPEED,
      IBUS_SENSOR_TYPE_UNKNOWN,      //AIRSPEED_COMPENSATED_VSPEED,
      IBUS_SENSOR_TYPE_UNKNOWN,      //SBUS_HOLD_COUNTER,
      
      IBUS_SENSOR_TYPE_UNKNOWN,      //SBUS_FAILSAFE_COUNTER,
      IBUS_SENSOR_TYPE_UNKNOWN,      // GPS CUMULATIVE DISTANCE,      
      IBUS_SENSOR_TYPE_ACC_X,        // ACC_X, m/s2 * 100
      IBUS_SENSOR_TYPE_ACC_Y,        // ACC_Y, m/s2 * 100
      IBUS_SENSOR_TYPE_ACC_Z,        // ACC_Z, m/s2 * 100

      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE3
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE4
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE5
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE6
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE7
      
};

void setupIbus() {                                                 
// configure the queue to get the data from ibus in the irq handle
    queue_init (&ibusRxQueue, sizeof(uint8_t), 50);

// set up the DMA but do not yet start it to send data to Sport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    ibus_dma_chan = dma_claim_unused_channel(true);
    ibusDmaConfig = dma_channel_get_default_config(ibus_dma_chan);
    channel_config_set_read_increment(&ibusDmaConfig, true);
    channel_config_set_write_increment(&ibusDmaConfig, false);
    channel_config_set_dreq(&ibusDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&ibusDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        ibus_dma_chan,
        &ibusDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &ibusTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    ibusOffsetTx = pio_add_program(ibusPio, &ibus_uart_tx_program);
    ibus_uart_tx_program_init(ibusPio, ibusSmTx, ibusOffsetTx, config.pinTlm, IBUS_BAUDRATE , false); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , ibusPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    ibusOffsetRx = pio_add_program(ibusPio, &ibus_uart_rx_program);
    ibus_uart_rx_program_init(ibusPio, ibusSmRx, ibusOffsetRx, config.pinTlm, IBUS_BAUDRATE , false); // false = not inverted   
    setupListIbusFieldsToReply();
}

void addToIbus(uint8_t idx){
    if (maxIbusFieldsIdx >= 15) return; // skip if the table is already full
    maxIbusFieldsIdx++;
    listOfIbusFields[maxIbusFieldsIdx] = idx; // store the oXs code for the field in the list.
}

void setupListIbusFieldsToReply() {  // fill an array with the list of fields (field ID) that are defined
    // EdgeTX-native AFHDS2A layout.
    // Address 0 is kept as a harmless 0x04 placeholder because some receivers
    // treat instance 0 specially. OpenAVRc ignores cell instance 0.
    listOfIbusFields[0] = IBUS_OXS_CELL0_DUMMY;
    maxIbusFieldsIdx = 0;

    // Native extended AFHDS2A groups used by EdgeTX.
    addToIbus(IBUS_OXS_GPS_FULL_FIELD);   // address 1: 0xFD GPS_FULL
    addToIbus(IBUS_OXS_VOLT_FULL_FIELD);  // address 2: 0xF0 VOLT_FULL
    addToIbus(IBUS_OXS_ACC_FULL_FIELD);   // address 3: 0xEF ACC_FULL

    // Six individual cells remain six standard AFHDS2A 0x04 sensors.
    for (uint8_t i = 0; i < 6; ++i) {
        addToIbus(IBUS_OXS_CELL1_FIELD + i); // addresses 4..9
    }

    // Remaining native scalar AFHDS2A sensors.
    addToIbus(TEMP1);       // address 10 -> 0x01
    addToIbus(TEMP2);       // address 11 -> 0x01
    addToIbus(GROUNDSPEED); // address 12 -> 0x13, m/s * 100
    addToIbus(HEADING);     // address 13 -> 0x0A, deg * 100
    addToIbus(VSPEED);      // address 14 -> 0x09
    addToIbus(GPS_HOME_DISTANCE); // address 15 -> 0x14

    #ifdef DEBUG
    printf("List of ibus fields : ");
    for (uint8_t i = 0; i<= maxIbusFieldsIdx ; i++){
        printf(" %d ", listOfIbusFields[i]);
    }
    printf("\n");
    #endif
}

static void ibusPutU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static void ibusPutS16(uint8_t *dst, int16_t value)
{
    ibusPutU16(dst, (uint16_t)value);
}

static void ibusPutS32(uint8_t *dst, int32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static int16_t ibusAccelFromOxs(int32_t milliG)
{
    // oXs acceleration is 0.001 g. AFHDS2A/EdgeTX uses m/s2 * 100.
    int32_t value = (milliG * 981L) / 1000L;
    if (value > 32767L) value = 32767L;
    if (value < -32768L) value = -32768L;
    return (int16_t)value;
}

static uint16_t ibusCellCentivolts(uint8_t cellIdx)
{
    uint16_t cellMv = 0;
    const uint16_t *forcedCells = getForcedCellValues(forcedFields);
    if (forcedCells != nullptr) {
        cellMv = forcedCells[cellIdx];
    } else if (cellIdx < 4U && fields[ADS_1_1 + cellIdx].available) {
        cellMv = (uint16_t)fields[ADS_1_1 + cellIdx].value;
    } else if (cellIdx >= 4U && fields[ADS_2_1 + (cellIdx - 4U)].available) {
        cellMv = (uint16_t)fields[ADS_2_1 + (cellIdx - 4U)].value;
    }
    return (cellMv + 5U) / 10U;
}

static void formatIbusExtendedPayload(uint8_t fieldId, uint8_t *payload)
{
    if (fieldId == IBUS_OXS_GPS_FULL_FIELD) {
        // EdgeTX: [fix][sats][LAT x4][LON x4][ALT x4].
        const uint8_t sats = fields[NUMSAT].available ? (uint8_t)fields[NUMSAT].value : 0U;
        payload[0] = (sats >= 4U) ? 3U : 0U;
        payload[1] = sats;
        ibusPutS32(payload + 2,  fields[LATITUDE].available  ? fields[LATITUDE].value  : 0);
        ibusPutS32(payload + 6,  fields[LONGITUDE].available ? fields[LONGITUDE].value : 0);
        ibusPutS32(payload + 10, fields[ALTITUDE].available  ? fields[ALTITUDE].value  : 0);
        return;
    }

    if (fieldId == IBUS_OXS_VOLT_FULL_FIELD) {
        // EdgeTX expands 0xF0 into IDs 0x03..0x07, five 16-bit values.
        uint16_t extV = fields[MVOLT].available ? (uint16_t)int_round(fields[MVOLT].value, 10) : 0U;
        uint16_t cellAvg = 0U;
        uint32_t cellSum = 0U;
        uint8_t cellCount = 0U;
        for (uint8_t i = 0; i < 6U; ++i) {
            uint16_t cv = ibusCellCentivolts(i);
            if (cv) { cellSum += cv; ++cellCount; }
        }
        if (cellCount) cellAvg = (uint16_t)(cellSum / cellCount);
        int32_t curr = fields[CURRENT].available ? int_round(fields[CURRENT].value, 10) : 0;
        if (curr < 0) curr = 0;
        uint16_t capa = fields[CAPACITY].available ? (uint16_t)fields[CAPACITY].value : 0U;
        uint16_t rpm  = fields[RPM].available ? (uint16_t)fields[RPM].value : 0U;

        ibusPutU16(payload + 0, extV);
        ibusPutU16(payload + 2, cellAvg);
        ibusPutU16(payload + 4, (uint16_t)curr);
        ibusPutU16(payload + 6, capa);
        ibusPutU16(payload + 8, rpm);
        return;
    }

    if (fieldId == IBUS_OXS_ACC_FULL_FIELD) {
        // -----------------------------------------------------------------
        // AFHDS2A ACC_FULL (0xEF), native payload length = 12 bytes.
        //
        // Standard layout:
        //   bytes  0..1  AccX
        //   bytes  2..3  AccY
        //   bytes  4..5  AccZ
        //   bytes  6..7  Roll
        //   bytes  8..9  Pitch
        //   bytes 10..11 Yaw
        //
        // IMPORTANT:
        // Keep ACC_FULL exactly 12 bytes. Earlier tests with an additional
        // 32-bit iBUS sensor changed AFHDS2A RF packing and caused C1..C3
        // to disappear.
        //
        // oXs -> OpenAVRc private GPS-time extension:
        //
        // When GPS_TIME is available, the three attitude words are reused
        // without changing the packet size:
        //
        //   Roll  = 0x48HH   ('H' in the high byte, hour in the low byte)
        //   Pitch = 0x4DMM   ('M' in the high byte, minute in the low byte)
        //   Yaw   = 0x53SS   ('S' in the high byte, second in the low byte)
        //
        // Example:
        //   GPS_TIME = 0x160D1F00 = 22:13:31
        //
        //   Roll  = 0x4816
        //   Pitch = 0x4D0D
        //   Yaw   = 0x531F
        //
        // OpenAVRc checks all three ASCII tags H/M/S plus valid ranges
        // before accepting the data as GPS time. This avoids accidentally
        // decoding normal Roll/Pitch/Yaw values as a clock.
        //
        // If GPS_TIME is not available, normal Roll/Pitch/Yaw values are
        // transmitted exactly as in the validated V4 implementation.
        // -----------------------------------------------------------------

        ibusPutS16(payload + 0, fields[ACC_X].available ? ibusAccelFromOxs(fields[ACC_X].value) : 0);
        ibusPutS16(payload + 2, fields[ACC_Y].available ? ibusAccelFromOxs(fields[ACC_Y].value) : 0);
        ibusPutS16(payload + 4, fields[ACC_Z].available ? ibusAccelFromOxs(fields[ACC_Z].value) : 0);

        if (fields[GPS_TIME].available) {
            const uint32_t gpsTime = (uint32_t)fields[GPS_TIME].value;
            const uint8_t gpsHour   = (uint8_t)(gpsTime >> 24);
            const uint8_t gpsMinute = (uint8_t)(gpsTime >> 16);
            const uint8_t gpsSecond = (uint8_t)(gpsTime >> 8);

            // Only send the private HMS extension when the GPS time is valid.
            // Otherwise fall back to the normal attitude values below.
            if (gpsHour < 24U && gpsMinute < 60U && gpsSecond < 60U) {
                ibusPutU16(payload + 6,  (uint16_t)(0x4800U | gpsHour));    // H + hour
                ibusPutU16(payload + 8,  (uint16_t)(0x4D00U | gpsMinute));  // M + minute
                ibusPutU16(payload + 10, (uint16_t)(0x5300U | gpsSecond));  // S + second
                return;
            }
        }

        // Standard ACC_FULL fallback: preserve native Roll/Pitch/Yaw.
        ibusPutS16(payload + 6, fields[ROLL].available  ? (int16_t)(fields[ROLL].value * 100L)  : 0);
        ibusPutS16(payload + 8, fields[PITCH].available ? (int16_t)(fields[PITCH].value * 100L) : 0);
        ibusPutS16(payload + 10, fields[YAW].available  ? (int16_t)(fields[YAW].value * 100L)   : 0);
    }
}

void ibusPioRxHandlerIrq(){    // when a byte is received on the ibus bus, read the pio sbus fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
    irq_clear (PIO0_IRQ_0 );
    while (  ! pio_sm_is_rx_fifo_empty (ibusPio ,ibusSmRx)){ // when some data have been received
        uint8_t c = pio_sm_get (ibusPio , ibusSmRx) >> 24;         // read the data
        queue_try_add (&ibusRxQueue, &c);          // push to the queue
        if (c == 0X04) lastIbusReceivedUs = microsRp();    
    }
}

//#define DEBUG_IBUS_WITHOUT_RX
#ifdef DEBUG_IBUS_WITHOUT_RX
uint32_t lastIbusRequest = 0;
#endif
void handleIbusRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    //static uint8_t previous = 0;
    static uint8_t data[4];
    //static uint8_t pollingSimulation;
    uint8_t ibusCmd;
    uint8_t ibusAdr;
    uint8_t ibusType;
    uint8_t ibusValueLength;
    uint16_t checksum;
    uint16_t checksumCalc;

    if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
    switch (ibusState) {
        case RECEIVING :
            #ifdef DEBUG_IBUS_WITHOUT_RX  // simulate a RX sending a polling every 7 msec 
            if ( (millisRp() - lastIbusRequest) > 6)  { // to debug simulate a request once per 200msec
                ibusState = WAIT_FOR_SENDING;
                ibusStartWaiting = microsRp();
                data = pollingSimulation++;
                if (pollingSimulation >0X0F) pollingSimulation = 0;
                lastIbusRequest = millisRp();
            }
            break;
            #endif
            while (queue_get_level(&ibusRxQueue) > 4) queue_try_remove (&ibusRxQueue,&data[0]);  
            if ( queue_get_level(&ibusRxQueue) == 4) {
                queue_try_remove (&ibusRxQueue,&data[0]);
                //printf("%X ", data);
                if ( data[0] != 0X04 ) return ;  // first byte of a frame must be 0X04 
                queue_try_remove (&ibusRxQueue,&data[1]);
                queue_try_remove (&ibusRxQueue,&data[2]);
                queue_try_remove (&ibusRxQueue,&data[3]);
                checksum = data[2] | (data[3] << 8);
                checksumCalc = 0xFFFF - (data[0] + data[1]);
                if (checksumCalc != checksum) {
                    printf("ibus pooling with checksum error %x %x %x %x \n", data[0] , data[1], data[2], data[3]);
                    return; // skip if checksum is wrong
                }
                //printf("ibus get %x %x %x %x\n", data[0] , data[1], data[2], data[3]);
                ibusCmd = data[1] >> 4;
                ibusAdr =  data[1] & 0X0F ;
                const uint8_t fieldId = listOfIbusFields[ibusAdr];
                const bool isCell = (fieldId == IBUS_OXS_CELL0_DUMMY ||
                                     (fieldId >= IBUS_OXS_CELL1_FIELD &&
                                      fieldId <= IBUS_OXS_CELL6_FIELD));

                ibusValueLength = 2;
                if (fieldId == IBUS_OXS_GPS_FULL_FIELD) {
                    ibusType = IBUS_SENSOR_TYPE_GPS_FULL;
                    ibusValueLength = 14;
                } else if (fieldId == IBUS_OXS_VOLT_FULL_FIELD) {
                    ibusType = IBUS_SENSOR_TYPE_VOLT_FULL;
                    ibusValueLength = 10;
                } else if (fieldId == IBUS_OXS_ACC_FULL_FIELD) {
                    ibusType = IBUS_SENSOR_TYPE_ACC_FULL;
                    ibusValueLength = 12;
                } else {
                    ibusType = isCell ? IBUS_SENSOR_TYPE_CELL : ibusTypes[fieldId];
                    if( ibusType >= 0X80 && ibusType <= 0X8F ) ibusValueLength = 4;
                }
                switch (ibusCmd) {
                    case 0X08: // request discovering next sensor ; we reply the same value to confirm it exists
                        if ( ibusAdr > (maxIbusFieldsIdx)) {
                            //printf("ibus request for sensor adr %d max=%d\n", ibusAdr, maxIbusFieldsIdx);
                            return;
                        }        
                        ibusTxBuffer[0] = data[0];
                        ibusTxBuffer[1] = data[1];
                        ibusTxBuffer[2] = data[2];
                        ibusTxBuffer[3] = data[3];
                        break;
                    case 0X09:  // request specify the type for an index
                        if (ibusAdr > maxIbusFieldsIdx) {
                            printf("error ibus request 0X09 for sensor adr %d  max=%d\n", ibusAdr, maxIbusFieldsIdx);
                            return;
                        }    
                        if (ibusType == IBUS_SENSOR_TYPE_UNKNOWN ){
                            printf("error ibus request 0X09 for sensor adr %d  type=%x max=%d\n", ibusAdr, ibusType , maxIbusFieldsIdx);
                            return; // skip when this field is not supported // should not happen
                        }
                        ibusTxBuffer[0] = 0x06; // length of the frame
                        ibusTxBuffer[1] = data[1]; // original cmd +adr
                        ibusTxBuffer[2] = ibusType; // type of field being sent
                        ibusTxBuffer[3] = ibusValueLength;
                        checksum = 0xFFFF - (ibusTxBuffer[0] + ibusTxBuffer[1] + ibusTxBuffer[2] + ibusTxBuffer[3]);
                        ibusTxBuffer[4] = checksum;
                        ibusTxBuffer[5] = checksum >> 8;
                        break;
                    case 0x0A:
                        if (ibusAdr > maxIbusFieldsIdx) {
                            printf("error ibus request 0X0A for sensor adr %d\n", ibusAdr);
                            return;
                        }    
                        if (ibusType == IBUS_SENSOR_TYPE_UNKNOWN ) return; // skip when this field is not supported // should not happen
                        ibusTxBuffer[0] = ibusValueLength + 4; // length of the frame
                        ibusTxBuffer[1] = data[1]; // original cmd +adr
                        checksum = 0xFFFF - (ibusTxBuffer[0] + ibusTxBuffer[1]);

                        // A missing scalar value is sent as zero. Extended AFHDS2A
                        // groups are formatted exactly as EdgeTX decodes them.
                        if (fieldId == IBUS_OXS_GPS_FULL_FIELD ||
                            fieldId == IBUS_OXS_VOLT_FULL_FIELD ||
                            fieldId == IBUS_OXS_ACC_FULL_FIELD)
                        {
                            formatIbusExtendedPayload(fieldId, &ibusTxBuffer[2]);
                            for (uint8_t i = 0; i < ibusValueLength; ++i)
                                checksum -= ibusTxBuffer[2 + i];
                        }
                        else
                        {
                            formatIbusValue(ibusAdr);
                            for (uint8_t i = 0; i < ibusValueLength; i++)
                            {
                                const uint8_t pByte = (ibusValue >> (8 * i)) & 0xFF;
                                checksum -= pByte;
                                ibusTxBuffer[2 + i] = pByte;
                            }
                        }

                        ibusTxBuffer[ibusValueLength + 2] = checksum;
                        ibusTxBuffer[ibusValueLength + 3] = checksum >> 8;
                        break;
                    default:
                        return;
                        break;    
                } // end switch about a ibus request
                // at this point, a frame is prepared and can be sent    
                //printf("ibus frame sent:");
                //for (uint8_t i = 0 ; i < ibusTxBuffer[0]; i++){
                //    printf(" %x ", ibusTxBuffer[i]);
                //}
                //printf("\n");
                sleep_us(100);
                ibus_uart_rx_program_stop(ibusPio, ibusSmRx, config.pinTlm); // stop receiving
                ibus_uart_tx_program_start(ibusPio, ibusSmTx, config.pinTlm, false); // prepare to transmit
                // start the DMA channel with the data to transmit
                dma_channel_set_read_addr (ibus_dma_chan, &ibusTxBuffer[0], false);
                dma_channel_set_trans_count (ibus_dma_chan, ibusTxBuffer[0], true) ; // ibusTxBuffer contains the number of bytes
                ibusState = WAIT_END_OF_SENDING;
                ibusStartWaiting = microsRp();
            } // end if        
            break;
        case WAIT_END_OF_SENDING :
            if ( ( microsRp() - ibusStartWaiting) > (2500) ){ // wait until the longest extended iBUS reply has been sent ( ???? usec)
                ibus_uart_tx_program_stop(ibusPio, ibusSmTx, config.pinTlm );
                ibus_uart_rx_program_restart(ibusPio, ibusSmRx, config.pinTlm, false );  // false = not inverted
                ibusState = RECEIVING ;
                //printf("end sending\n");
            }
            break;     
    }
     
}

bool formatIbusValue( uint8_t ibusAdr){
    uint8_t fieldId = listOfIbusFields[ibusAdr];
    ibusValue = 0;

    if (fieldId == IBUS_OXS_CELL0_DUMMY) {
        ibusValue = 0;
        return true;
    }

    if (fieldId >= IBUS_OXS_CELL1_FIELD && fieldId <= IBUS_OXS_CELL6_FIELD) {
        const uint8_t cellIdx = fieldId - IBUS_OXS_CELL1_FIELD;
        ibusValue = ibusCellCentivolts(cellIdx);
        return ibusValue != 0;
    }

    if (fields[fieldId].available == false) return false; // false when the value is not available
    ibusValue= fields[fieldId].value ; // default do not use conversion
    switch (fieldId) {
        case MVOLT:
            ibusValue= int_round(fields[fieldId].value , 10); // from mvolt to 0.01V
            break;
        case CURRENT:
            ibusValue= int_round(fields[fieldId].value , 10); // from mamp to 0.01A
            if (ibusValue < 0) ibusValue =0;
            break;
        case NUMSAT:
            ibusValue= fields[fieldId].value * 256; // it seems that openTx discard the lowest byte
            break;
        case TEMP1:
            ibusValue= (fields[fieldId].value * 10) + 400; // from ° to 0.1°; it seems that openTx uses an offset of 400
            break;
        case TEMP2:
            ibusValue= (fields[fieldId].value * 10) + 400; // from ° to 0.1°; it seems that openTx uses an offset of 400
            break;
        //case CAPACITY:
        //    value= fields[fieldId].value ; // from mah to mah
        //    break;
        //case TEMP1:
        //    value= fields[fieldId].value ; // from ° to °
        //    break;
        //case TEMP2:
        //    value= fields[fieldId].value ; // from ° to °
        //    break;    
        //case VSPEED:
        //    value= fields[fieldId].value ; // from cm/sec to 0.1m/sec
        //    break;
        case GROUNDSPEED:
            // oXs stores cm/s, numerically identical to AFHDS2A m/s * 100.
            ibusValue = fields[fieldId].value;
            break;            
        //case RPM:
        //    value= fields[fieldId].value *60 / 100 ; // from Hz to 100 tr/min???? not sure it is ok
        //    break;            
        //case GPS_HOME_BEARING:
        //    value= fields[fieldId].value  ; // from  deg  to deg
        //    break;
        //case GPS_HOME_DISTANCE:
        //    value= fields[fieldId].value  ; // from  m  to m
        //    break;     
        //case RELATIVEALT:
        //    value= fields[fieldId].value ; // from cm to cm
        //    break;
        case ACC_X:
        case ACC_Y:
        case ACC_Z:
            // oXs stores acceleration in 0.001 g. EdgeTX AFHDS2A defines
            // IDs 0x0C..0x0E as signed m/s2 * 100.
            ibusValue = (fields[fieldId].value * 981L) / 1000L;
            break;
        case ADS_1_1:
            ibusValue= int_round(fields[fieldId].value , 10); // from mvolt to 0.01V
            break;
        case ADS_1_2:
            ibusValue= int_round(fields[fieldId].value , 10); // from mvolt to 0.01V
            break;
        case ADS_1_3:
            ibusValue= int_round(fields[fieldId].value , 10); // from mvolt to 0.01V
            break;
        case ADS_1_4:
            ibusValue= int_round(fields[fieldId].value , 10); // from mvolt to 0.01V
            break;
                    
    } // end switch 
    // fields[fieldId].available = false;   // reset the flag available // it seems we always have to send a value otherwise Rx try to rediscover the sensor
    return true;    
}


