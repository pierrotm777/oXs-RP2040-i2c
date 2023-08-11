# openXsensor (oXs) on RP2040 board with I2C protocols

## This project add 3 I2C protocols to the original project:
- Original [oXs_on_RP2040](https://github.com/mstrens/oXs_on_RP2040/tree/test) project
- RadioLink
- Spektrum Xbus
- Hitec 

## --------- Wiring --------------------

RadioLink/Spektrum/Hitec/... receiver, MS5611, GPS and other sensors must share the same Gnd.  
Connect a 5V source to the Vcc pin of RP2040 board ( RP2040-zero or RP2040-TINY boards do not accept more than 5.5V on Vcc pin !! ).  

There is no default affectation of the RP2040 pins so user has to specify it with some parameters after flashing the firmware (see below).  
But this board use the **GPIO8 for TLM (SDA)** and **GPIO9 for PRI(SCL)**.

![I2C Wiring](https://github.com/pierrotm777/oXs-RP2040-i2c/blob/main/oXsRP2040Full_I2C.jpg)

Depending on the protocol, the pins used for PRIMARY/SECONDARY RC Channels and for Telemetry (TLM) varies
| protocol       | PRI pin is connectected to | SEC pin is connected to | TLM pin is connected to| Comment|
|----------      |----------------------------|-------------------------|------------------------|--------|
| R(RadioLink)   |    (SCL from Rx1)          |     Not used            | (SDA from RX1)         | (1)(2) |
| X(Xbus)        |    (SCL from Rx1)          |     Not used            | (SDA from RX1)         | (1)(2) |
| T(Hitec)       |    (SCL from Rx1)          |     Not used            | (SDA from RX1)         | (1)(2) |

(1) for safety, insert a 1 kOhm resistor between TLM pin and Rx 

(2) for safety, insert a 1 kOhm resistor between PRI pin and Rx

### RadioLink wiring
![R9DS](https://github.com/pierrotm777/oXs-RP2040-i2c/blob/main/RadioLink_R9DS.jpg)
![Connection](https://github.com/pierrotm777/oXs-RP2040-i2c/blob/main/RadioLink_Telemetry.jpg)  

### Optima 7 wiring
![Optima 7](https://github.com/pierrotm777/oXs-RP2040-i2c/blob/main/Hitec_Optima7_Telemetry.jpg)  

### Spektrum XBUS wiring
![Connection](https://github.com/pierrotm777/oXs-RP2040-i2c/blob/main/Spektrum_XBUS_Telemetry.jpg)  
