/**
 * Castle Creations' Phoenix ICE ESC to Hitec's Optima 9 telemetry conversion box
 * Telemetry Conversion Box, for short
 * Designed by Ronaldo Capaverde based on documents by Castle Creations and 
 * reverse engeneering from rcgroups.com and myself
 */

#include <Wire.h>

#define RADIO_ENTRADA    3
#define ESC_SAIDA        2
#define WATCHDOG         13

// max time to wait for the ESC's tick
#define TIMEOUT_DATA  6000

#define FRAME_RESET            0
#define FRAME_REFERENCE        1
#define FRAME_VOLTAGE          2
#define FRAME_RIPPLE_VOLTAGE   3
#define FRAME_CURRENT          4
#define FRAME_THROTTLE         5
#define FRAME_OUTPUT_POWER     6
#define FRAME_RPM              7
#define FRAME_BEC_VOLTAGE      8
#define FRAME_BEC_CURRENT      9
#define FRAME_TEMP1           10
#define FRAME_TEMP2           11
#define FRAME_MAX             11

struct {
  uint8_t packid1;      // fixed, 0x11
  uint8_t header;       // fixed, 0xAF 
  uint8_t zero;         // fixed, 0x00
  uint8_t frametype;    // fixed, 0x2D
  uint8_t notused1;     // not used, 0x00
  uint8_t notused2;     // not used, 0x00
  uint8_t packid2;      // fixed, 0x11
} packet11;

struct {
  uint8_t packid1;      // fixed, 0x12
  uint16_t gpslatsec;   // gps latitude 1/100th of second
  uint16_t gpslatmin;   // gps latitude degre.minutes
  uint8_t gpstimesec;   // gps time seconds 
  uint8_t packid2;      // fixed, 0x12
} packet12;

struct {
  uint8_t packid1;      // fixed, 0x13
  uint16_t gpslongsec;  // gps longitude 1/100th of second
  uint16_t gpslongmin;  // gps longitude degre.minutes
  uint8_t temp2;        // temperature 2 - 40, in degre centigrades
  uint8_t packid2;      // fixed, 0x13
} packet13;

struct {
  uint8_t packid1;      // fixed, 0x14
  uint16_t speed;       // speed in km/h
  uint16_t alt;         // altitude in meters
  uint8_t temp1;        // temperature 1 - 40, in degre centigrades
  uint8_t packid2;      // fixed, 0x14
} packet14;

struct {
  uint8_t packid1;      // fixed, 0x15
  uint8_t fuelgauge;    // fuel level, 0 to 4
  uint16_t rpm1;        // RPM 1 / 10
  uint16_t rpm2;        // RPM 2 / 10
  uint8_t packid2;      // fixed, 0x15
} packet15;

struct {
  uint8_t packid1;      // fixed, 0x16
  uint8_t gpsdateyear;  // gps date year
  uint8_t gpsdatemonth; // gps date month
  uint8_t gpsdateday;   // gps date year
  uint8_t gpstimehour;  // gps time hour
  uint8_t gpstimemin;   // gps time minute
  uint8_t packid2;      // fixed, 0x16
} packet16;

struct {
  uint8_t packid1;      // fixed, 0x17
  uint16_t capdegree;   // gps heading
  uint8_t gpsstrength;  // gps signal strength
  uint8_t temp3;        // temperature 3 - 40, in degre centigrades
  uint8_t temp4;        // temperature 4 - 40, in degre centigrades
  uint8_t packid2;      // fixed, 0x17
} packet17;

struct {
  uint8_t packid1;      // fixed, 0x18
  uint16_t voltage;     // battery voltage * 10
  uint16_t current;     // current drawing * 14.5 + 164
  uint8_t zero;         // unknown
  uint8_t packid2;      // fixed, 0x18
} packet18;

#define PACKETCOUNT 8

uint8_t *packets[PACKETCOUNT] = {
  (uint8_t *)&packet11,
  (uint8_t *)&packet12,
  (uint8_t *)&packet13,
  (uint8_t *)&packet14,
  (uint8_t *)&packet15,
  (uint8_t *)&packet16,
  (uint8_t *)&packet17,
  (uint8_t *)&packet18
};

// hash table with precalculed ntc temperatures
#define NTC_COUNT     12
#define NTC_BASE      300
#define NTC_INTERVAL  200
float tempNtc[] = {106.5653802, 84.32856947, 70.29082597, 59.90730467,
  51.54376586, 44.42890738, 38.13498376, 32.39583709, 27.02918939,
  21.89813656, 16.88910995, 11.89668541};

int frame = 0;
int reference1ms = 0;
int temp1;
int temp2;

uint32_t startTimer = 0;

void setup()
{
  pinMode(RADIO_ENTRADA, INPUT);
  pinMode(ESC_SAIDA, OUTPUT);
  pinMode(WATCHDOG, OUTPUT);

  // keep the ESC's output high, since the pulse to it is inverted
  digitalWrite(ESC_SAIDA, HIGH);
    
  // setup of packets to be sent to optima
  for (int iPacket = 0; iPacket < PACKETCOUNT; iPacket++)
  {
    memset(packets[iPacket], 0, 7);
    packets[iPacket][0] = 0x11 + iPacket;
    packets[iPacket][6] = 0x11 + iPacket;
  }
    
  packet11.header = 0xAF;
  packet11.frametype = 0x2D;

  // configures the comunication with Optima tranceiver
  // this comunication is made via I2C protocol
  // Optima is the master and arduino is the slave on address 8
  Wire.begin(8);
  Wire.onRequest(optimaRequest);
  
  // interrupts when the radio's signal line changes its state
  // Optima outputs a rectangular signal with a frequency near to 50 Hz
  // and an active cicle between 1 and 2 miliseconds
  // we want to be notified when in raising and falling edges of this wave
  attachInterrupt(RADIO_ENTRADA - 2, radioRequest, CHANGE);

  // interrupts when the ESC signal the its data tick
  // this tick occurs between 0.5 e 5.5 miliseconds aftwer the signal
  // sent by Optima
  attachInterrupt(ESC_SAIDA - 2, escRequest, FALLING);
  
  // opens the serial port, for test purpose
  //Serial.begin(115200);
  //Serial.println("Phoenix-Aurora Telemetry Conversion Box");

  digitalWrite(WATCHDOG, HIGH);
}

void optimaRequest()
{
  // Optima has requested a new data packet
  static int iPacket = 0; 
  static char watchdog = 0;
  
  // turn on and off the watchdog led
  digitalWrite(13, (watchdog = 1 - watchdog));
  
  // sends the next data packet
  Wire.send(packets[iPacket++], 7);
  iPacket &= 7;
}

void radioRequest()
{
  // the throttle signal has started or ended
  boolean throttle = digitalRead(RADIO_ENTRADA);
  
  if (throttle == HIGH)
  {
    // throttle has transitioned from low to high, signaling the pulse start 
    digitalWrite(ESC_SAIDA, LOW);
  }
  else
  {
    // throttle has transitioned from high to low, signaling the pulse end
    // setup the ESC's port to read mode, since a pulse will be output in no more than 6 ms
    digitalWrite(ESC_SAIDA, HIGH);
    pinMode(ESC_SAIDA, INPUT);
    
    // saves the current time, to calculate the elapsed time to the tick output by the ESC
    startTimer = micros();
  }
}

void escRequest()
{
  uint32_t value = micros() - startTimer;

  // ignores any interruption if the pulse has not started
  if (!startTimer)
    return;
  
  frame++;

  // first pulse in the sequence
  if (frame == FRAME_REFERENCE)
  {
    reference1ms = value;
  }
  else if (reference1ms)
  {
    // deduces a half milisecond from the value read
    // reference is 1 milisecond long
    if (value > (reference1ms >> 1))
      value -= reference1ms >> 1;
    else
      value = 0;

    switch (frame)
    {
      case FRAME_VOLTAGE:
      {
        int voltage = value * 200 / reference1ms;
        packet18.voltage = voltage;
        int fuelgauge = (voltage - (32 * 4) + 5) / 4;
        if (fuelgauge < 0)
          fuelgauge = 0;
        if (fuelgauge > 4)
          fuelgauge= 4;
        packet15.fuelgauge = fuelgauge;
        break;
      }
        
      case FRAME_CURRENT:
      {
        packet18.current = value * 725 / reference1ms + 164;
        break;
      }
        
      case FRAME_RPM:
      {
        uint16_t rpm = value * 41 / reference1ms;
        packet15.rpm1 = (rpm > 8) ? rpm : 0;
        break;
      }

      case FRAME_TEMP1:
      {
        temp1 = value;
        break;
      }
        
      case FRAME_TEMP2:
      {
        temp2 = value;
        
        // acording to the Castle Link Live's protocol manual:
        // -- "Temperature is measured in one of two ways, and embedded temp sensor or an external 
        // -- NTC resistor. Only one of these two data fields will have valid data (greater than 0.5ms). 
        // -- If the controller is NTC type, logarithmic math is required to do the conversion."
        // We are not going to calculte logarithmics here. Instead, we gonna use a hash table
        // with pre-calculated temperatures, interpolating the in-between values.
        if (temp1 > temp2)
        {
          packet14.temp1 = temp1 * 30 / reference1ms;
        }
        else
        {
          long temp = temp2 * 1000L / reference1ms;
          int index = (temp - NTC_BASE) / NTC_INTERVAL;
          
          if (index >= 0 && index < NTC_COUNT - 1)
          {
            float tempAbove = (temp2 - (index * NTC_INTERVAL + NTC_BASE)) * tempNtc[index];
            float tempBelow = (((index + 1) * NTC_INTERVAL + NTC_BASE) - temp2) * tempNtc[index + 1];
            packet14.temp1 = (tempAbove + tempBelow) / NTC_INTERVAL + 40;
          }
        }
        
        break;
      }
    }
  }  
  
  startTimer = 0;
  pinMode(ESC_SAIDA, OUTPUT);
}

void loop()
{
  noInterrupts();
  // detects the ESC's tick timeout
  // when it occurs, the pulse train must be restarted
  if (startTimer && (micros() - startTimer > TIMEOUT_DATA))
  {
    frame = (frame >= FRAME_MAX) ? 0 : frame + 1;
    startTimer = 0;
    pinMode(ESC_SAIDA, OUTPUT);
  }
  interrupts();
  
  delay(1);
}