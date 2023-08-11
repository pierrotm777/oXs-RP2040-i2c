
#pragma once

#define	TELE_DEVICE_NODATA			(0x00)										// No data in packet, but telemetry is alive
#define	TELE_DEVICE_VOLTAGE			(0x01)										// High-Voltage sensor (INTERNAL)
#define	TELE_DEVICE_TEMPERATURE		(0x02)										// Temperature Sensor (INTERNAL)
#define	TELE_DEVICE_AMPS			(0x03)										// Amps (INTERNAL)
#define	TELE_DEVICE_RSV_04			(0x04)										// Reserved
#define	TELE_DEVICE_FLITECTRL		(0x05)										// Flight Controller Status Report
#define	TELE_DEVICE_RSV_06			(0x06)										// Reserved
#define	TELE_DEVICE_RSV_07			(0x07)										// Reserved
#define	TELE_DEVICE_RSV_08			(0x08)										// Reserved
//#define	DO_NOT_USE				(0x09)										// DO NOT USE!
#define	TELE_DEVICE_PBOX			(0x0A)										// PowerBox
#define	TELE_DEVICE_LAPTIMER		(0x0B)										// Lap Timer
#define	TELE_DEVICE_TEXTGEN			(0x0C)										// Text Generator
#define	TELE_DEVICE_VTX				(0x0D)										// Video Transmitter Feedback
#define	TELE_DEVICE_RSV_0E			(0x0E)										// Reserved
#define	TELE_DEVICE_RSV_0F			(0x0F)										// Reserved
#define	TELE_DEVICE_RSV_10			(0x10)										// Reserved
#define	TELE_DEVICE_AIRSPEED		(0x11)										// Air Speed (Eagle Tree Sensor)
#define	TELE_DEVICE_ALTITUDE		(0x12)										// Altitude (Eagle Tree Sensor)
#define	TELE_DEVICE_RSV_13			(0x13)										// Reserved
#define	TELE_DEVICE_GMETER			(0x14)										// G-Force (Eagle Tree Sensor)
#define	TELE_DEVICE_JETCAT			(0x15)										// Turbine interface (Eagle Tree)
#define	TELE_DEVICE_GPS_LOC			(0x16)										// GPS Location Data (Eagle Tree)
#define	TELE_DEVICE_GPS_STATS		(0x17)										// GPS Status (Eagle Tree)
#define	TELE_DEVICE_RX_MAH			(0x18)										// Receiver Pack Capacity (Dual)
#define	TELE_DEVICE_JETCAT_2		(0x19)										// Turbine interface, message 2 format (Eagle Tree)
#define	TELE_DEVICE_GYRO			(0x1A)										// 3-axis gyro
#define	TELE_DEVICE_ATTMAG			(0x1B)										// Attitude and Magnetic Compass
#define	TELE_DEVICE_TILT			(0x1C)										// Surface Tilt Sensor
#define	TELE_DEVICE_RSV_1D			(0x1D)										// Reserved
#define	TELE_DEVICE_AS6X_GAIN		(0x1E)										// Active AS6X Gains (new mode)
#define	TELE_DEVICE_AS3X_LEGACYGAIN	(0x1F)										// Active AS3X Gains for legacy mode
#define	TELE_DEVICE_ESC				(0x20)										// Electronic Speed Control
#define	TELE_DEVICE_RSV_21			(0x21)										// Reserved
#define	TELE_DEVICE_FUEL			(0x22)										// Fuel Flow Meter
#define	TELE_DEVICE_RSV_23			(0x23)										// Reserved
#define	TELE_DEVICE_ALPHA6			(0x24)										// Alpha6 Stabilizer
#define	TELE_DEVICE_RSV_25			(0x25)										// Reserved
#define	TELE_DEVICE_GPS_BINARY		(0x26)										// GPS, binary format
#define	TELE_DEVICE_RSV_27			(0x27)										// Reserved
#define	TELE_DEVICE_RSV_28			(0x28)										// Reserved
#define	TELE_DEVICE_RSV_29			(0x29)										// Reserved
#define	TELE_DEVICE_RSV_2A			(0x2A)										// Reserved
#define	TELE_DEVICE_RSV_2B			(0x2B)										// Reserved
#define	TELE_DEVICE_RSV_2C			(0x2C)										// Reserved
#define	TELE_DEVICE_RSV_2D			(0x2D)										// Reserved
#define	TELE_DEVICE_RSV_2E			(0x2E)										// Reserved
#define	TELE_DEVICE_RSV_2F			(0x2F)										// Reserved
//#define	DO_NOT_USE				(0x30)										// Internal ST sensor
//#define	DO_NOT_USE				(0x32)										// Internal ST sensor
#define	TELE_DEVICE_RSV_33			(0x33)										// Reserved
#define	TELE_DEVICE_FP_MAH			(0x34)										// Flight Battery Capacity (Dual)
#define	TELE_DEVICE_RSV_35			(0x35)										// Reserved
#define	TELE_DEVICE_DIGITAL_AIR		(0x36)										// Digital Inputs & Tank Pressure
#define	TELE_DEVICE_RSV_37			(0x37)										// Reserved
#define	TELE_DEVICE_STRAIN			(0x38)										// Thrust/Strain Gauge
#define	TELE_DEVICE_RSV_39			(0x39)										// Reserved
#define	TELE_DEVICE_LIPOMON			(0x3A)										// 6S Cell Monitor (LiPo taps)
#define	TELE_DEVICE_RSV_3B			(0x3B)										// Reserved
#define	TELE_DEVICE_RSV_3C			(0x3C)										// Reserved
#define	TELE_DEVICE_RSV_3D			(0x3D)										// Reserved
#define	TELE_DEVICE_RSV_3E			(0x3E)										// Reserved
#define	TELE_DEVICE_LIPOMON_14		(0x3F)										// 14S Cell Monitor (LiPo taps)
#define	TELE_DEVICE_VARIO_S			(0x40)										// Vario
#define	TELE_DEVICE_RSV_41			(0x41)										// Reserved
#define	TELE_DEVICE_SMARTBATT		(0x42)										// Spektrum SMART Battery (multiple structs)
#define	TELE_DEVICE_RSV_43			(0x43)										// Reserved
#define	TELE_DEVICE_RSV_44			(0x44)										// Reserved
#define	TELE_DEVICE_RSV_45			(0x45)										// Reserved
#define	TELE_DEVICE_RSV_46			(0x46)										// Reserved
#define	TELE_DEVICE_RSV_47			(0x47)										// Reserved
#define	TELE_DEVICE_RSV_48			(0x48)										// Reserved
#define	TELE_DEVICE_RSV_49			(0x49)										// Reserved
#define	TELE_DEVICE_RSV_4A			(0x4A)										// Reserved
#define	TELE_DEVICE_RSV_4B			(0x4B)										// Reserved
#define	TELE_DEVICE_RSV_4C			(0x4C)										// Reserved
#define	TELE_DEVICE_RSV_4D			(0x4D)										// Reserved
#define	TELE_DEVICE_RSV_4E			(0x4E)										// Reserved
#define	TELE_DEVICE_RSV_4F			(0x4F)										// Reserved
#define	TELE_DEVICE_USER_16SU		(0x50)										// User-Defined, XBUSSTRU_TELE_USER_16SU
#define	TELE_DEVICE_RSV_51			(0x51)										// Reserved
#define	TELE_DEVICE_USER_16SU32U	(0x52)										// User-Defined, XBUSSTRU_TELE_USER_16SU32U
#define	TELE_DEVICE_RSV_53			(0x53)										// Reserved
#define	TELE_DEVICE_USER_16SU32S	(0x54)										// User-Defined, XBUSSTRU_TELE_USER_16SU32S
#define	TELE_DEVICE_RSV_55			(0x55)										// Reserved
#define	TELE_DEVICE_USER_16U32SU	(0x56)										// User-Defined, XBUSSTRU_TELE_USER_16U32SU
#define	TELE_DEVICE_RSV_57			(0x57)										// Reserved
#define	TELE_DEVICE_RSV_58			(0x58)										// Reserved
#define	TELE_DEVICE_MULTICYLINDER	(0x59)										// Multi-cylinder temp sensor
#define	TELE_DEVICE_RSV_5A			(0x5A)										// Reserved
#define	TELE_DEVICE_RSV_5B			(0x5B)										// Reserved
#define	TELE_DEVICE_RSV_5C			(0x5C)										// Reserved
#define	TELE_DEVICE_RSV_5D			(0x5D)										// Reserved
#define	TELE_DEVICE_RSV_5E			(0x5E)										// Reserved
#define	TELE_DEVICE_RSV_5F			(0x5F)										// Reserved
#define	TELE_DEVICE_VSPEAK			(0x60)										// Reserved for V-Speak
#define	TELE_DEVICE_SMOKE_EL		(0x61)										// Reserved for Smoke-EL.de
#define	TELE_DEVICE_CROSSFIRE		(0x62)										// Reserved for Crossfire devices
#define	TELE_DEVICE_RSV_63			(0x63)										// Reserved
#define	TELE_DEVICE_RSV_64			(0x64)										// Reserved
#define	TELE_DEVICE_RSV_65			(0x65)										// Reserved
#define	TELE_DEVICE_EXTRF			(0x66)										// Reserved for Generic External RF sources
#define	TELE_DEVICE_RSV_67			(0x67)										// Reserved
#define	TELE_DEVICE_RSV_68			(0x68)										// Reserved
#define	TELE_DEVICE_RSV_69			(0x69)										// Reserved
#define	TELE_DEVICE_RSV_6A			(0x6A)										// Reserved
//#define	DO_NOT_USE				(0x6B)										// DO NOT USE!
#define	TELE_DEVICE_RSV_6C			(0x6C)										// Reserved
#define	TELE_DEVICE_RSV_6D			(0x6D)										// Reserved
#define	TELE_DEVICE_RSV_6E			(0x6E)										// Reserved
#define	TELE_DEVICE_RSV_6F			(0x6F)										// Reserved
#define	TELE_DEVICE_RSV_70			(0x70)										// Reserved
#define	TELE_XRF_LINKSTATUS			(0x71)										// External RF Link Status
#define	TELE_DEVICE_RSV_72			(0x72)										// Reserved
#define	TELE_DEVICE_RSV_73			(0x73)										// Reserved
#define	TELE_DEVICE_RSV_74			(0x74)										// Reserved
#define	TELE_DEVICE_RSV_75			(0x75)										// Reserved
#define	TELE_DEVICE_RSV_76			(0x76)										// Reserved
#define	TELE_DEVICE_RSV_77			(0x77)										// Reserved
#define	TELE_DEVICE_RSV_78			(0x78)										// Reserved
#define	TELE_DEVICE_RSV_79			(0x79)										// Reserved
#define	TELE_DEVICE_RSV_7A			(0x7A)										// Reserved
#define	TELE_DEVICE_ALT_ZERO		(0x7B)										// Pseudo-device setting Altitude "zero"
#define	TELE_DEVICE_RTC				(0x7C)										// Pseudo-device giving timestamp
#define	TELE_DEVICE_RPM				(0x7E)										// RPM sensor
#define	TELE_DEVICE_QOS				(0x7F)										// RxV + flight log data
#define	TELE_DEVICE_MAX				(0x7F)										// Last address available

#define	TELE_DEVICE_SHORTRANGE		(0x80)										// OR this bit to indicate data is from a short-range telemetry device (e.g. TM1100)

#define	TELE_DEVICE_MAX_PROGRAM		(0x70)										// Last programmable address when using sID

//////////////////////////////////////////////////////////////////////////////
//
//							TELEMETRY
//					DEVICE-SPECIFIC STRUCTURES
//
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT DATA SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t	identifier;															// Source device = 0x50
	uint8_t	sID;																// Secondary ID
	int16_t	sField1,															// Signed 16-bit data fields
			sField2,
			sField3;
	uint16_t	uField1, 															// Unsigned 16-bit data fields
			uField2,
			uField3,
			uField4;
} XBUSSTRU_TELE_USER_16SU;

//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT UNSIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t	identifier;															// Source device = 0x52
	uint8_t	sID;																// Secondary ID
	int16_t	sField1,															// Signed 16-bit data fields
			sField2;
	uint16_t	uField1, 															// Unsigned 16-bit data fields
			uField2,
			uField3;
	uint32_t	u32Field; 															// Unsigned 32-bit data field
} XBUSSTRU_TELE_USER_16SU32U;

//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT SIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t	identifier;															// Source device = 0x54
	uint8_t	sID;																// Secondary ID
	int16_t	sField1,															// Signed 16-bit data fields
			sField2;
	uint16_t	uField1, 															// Unsigned 16-bit data fields
			uField2,
			uField3;
	int32_t	s32Field; 															// Signed 32-bit data field
} XBUSSTRU_TELE_USER_16SU32S;

//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT UNSIGNED AND 32-BIT SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t	identifier;															// Source device = 0x56
	uint8_t	sID;																// Secondary ID
	uint16_t	uField1; 															// Unsigned 16-bit data field
	int32_t	s32Field; 															// Signed 32-bit data field
	uint32_t	u32Field1, 															// Unsigned 32-bit data fields
			u32Field2;
} XBUSSTRU_TELE_USER_16U32SU;

//////////////////////////////////////////////////////////////////////////////
//
//							POWERBOX
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t	identifier;															// Source device = 0x0A
	uint8_t	sID;																// Secondary ID
	uint16_t	volt1;																// Volts, 0.01v
	uint16_t	volt2;																// Volts, 0.01v
	uint16_t	capacity1;															// mAh, 1mAh
	uint16_t	capacity2;															// mAh, 1mAh
	uint16_t	spare16_1;
	uint16_t	spare16_2;
	uint8_t	spare;
	uint8_t	alarms;																// Alarm bitmask (see below)
} XBUSSTRU_TELE_POWERBOX;

#define	TELE_PBOX_ALARM_VOLTAGE_1			(0x01)
#define	TELE_PBOX_ALARM_VOLTAGE_2			(0x02)
#define	TELE_PBOX_ALARM_CAPACITY_1			(0x04)
#define	TELE_PBOX_ALARM_CAPACITY_2			(0x08)
//#define	TELE_PBOX_ALARM_RPM					(0x10)
//#define	TELE_PBOX_ALARM_TEMPERATURE			(0x20)
#define	TELE_PBOX_ALARM_RESERVED_1			(0x40)
#define	TELE_PBOX_ALARM_RESERVED_2			(0x80)

//////////////////////////////////////////////////////////////////////////////
//
//							VOLTAGE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x01
	uint8_t		sID;															// Secondary ID
	uint16_t		volts;															// 0.01V increments
} XBUSSTRU_TELE_HV;

//////////////////////////////////////////////////////////////////////////////
//
//							TEMPERATURE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x02
	uint8_t		sID;															// Secondary ID
	int16_t		temperature;													// Temperature in degrees Fahrenheit
} XBUSSTRU_TELE_TEMP;

//////////////////////////////////////////////////////////////////////////////
//
//						RX CAPACITY METER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x18
	uint8_t		sID;															// Secondary ID
	int16_t		current_A;														// Instantaneous current, 0.01A (0-328.7A)		7FFF-> no data
	uint16_t		chargeUsed_A;													// Integrated mAh used, 0.1mAh (0-3276.6mAh)
	uint16_t		volts_A;														// Volts, 0.01V increments (0-16.00V)
	int16_t		current_B;														// Instantaneous current, 0.01A (0-328.7A)		7FFF-> no data/sensor B
	uint16_t		chargeUsed_B;													// Integrated mAh used, 0.1mAh (0-3276.6mAh)
	uint16_t		volts_B;														// Volts, 0.01V increments (0-16.00V)
	uint8_t		alerts,															// Bit mapped alert conditions (see below)
				highCharge;														// High nybble is extra bits for chargeUsed_B, Low is for chargeUsed_A
} XBUSSTRU_TELE_RX_MAH;

#define	RXMAH_PS_ALERT_NONE			(0)											// No alarms
#define	RXMAH_PS_ALERT_RF_INT		(1 << 0)									// A or internal Remote failure
#define	RXMAH_PS_ALERT_RF_ANT1		(1 << 1)									// B remote power fault
#define	RXMAH_PS_ALERT_RF_ANT2		(1 << 2)									// L remote power fault
#define	RXMAH_PS_ALERT_RF_ANT3		(1 << 3)									// R remote power fault
#define	RXMAH_PS_ALERT_OVERVOLT_A	(1 << 4) 									// Battery A is over voltage
#define	RXMAH_PS_ALERT_OVERVOLT_B	(1 << 5) 									// Battery A is over voltage
#define	RXMAH_PS_ALERT_RFU1			(1 << 6)
#define	RXMAH_PS_ALERT_RFU2			(1 << 7)

//////////////////////////////////////////////////////////////////////////////
//
//							HIGH-CURRENT
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x03
	uint8_t		sID;															// Secondary ID
	int16_t		current,														// Range: +/- 150A     Resolution: 300A / 2048 = 0.196791 A/count
				dummy;															// TBD
} XBUSSTRU_TELE_IHIGH;

#define	IHIGH_RESOLUTION_FACTOR						((FP32)(0.196791))

//////////////////////////////////////////////////////////////////////////////
//
//							SIMPLE VARIO
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x40
	uint8_t		sID;															// Secondary ID
	int16_t		altitude;														// .1m increments
	int16_t		delta_0250ms,													// change in altitude last 250ms, 0.1m/s increments
				delta_0500ms,													// change in altitude last 500ms, 0.1m/s increments
				delta_1000ms,													// change in altitude last 1.0 seconds, 0.1m/s increments
				delta_1500ms,													// change in altitude last 1.5 seconds, 0.1m/s increments
				delta_2000ms,													// change in altitude last 2.0 seconds, 0.1m/s increments
				delta_3000ms;													// change in altitude last 3.0 seconds, 0.1m/s increments
} XBUSSTRU_TELE_VARIO_S;

//////////////////////////////////////////////////////////////////////////////
//
//							ALTIMETER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;
	uint8_t		sID;															// Secondary ID
	int16_t		altitude;														// .1m increments
	int16_t		maxAltitude;													// .1m increments
} XBUSSTRU_TELE_ALT;																// Eagle Tree Sensor

//////////////////////////////////////////////////////////////////////////////
//
//							AIRSPEED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;
	uint8_t		sID;															// Secondary ID
	uint16_t	airspeed;														// 1 km/h increments
	uint16_t	maxAirspeed;													// 1 km/h increments
} XBUSSTRU_TELE_SPEED;																// Eagle Tree Sensor

//////////////////////////////////////////////////////////////////////////////
//
//							LAP TIMER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;
	uint8_t		sID;															// Secondary ID
	uint8_t		lapNumber;														// Lap last finished
	uint8_t		gateNumber;														// Last gate passed
	uint32_t	lastLapTime;													// Time of lap in 1ms increments (NOT duration)
	uint32_t	gateTime;														// Duration between last 2 gates
	uint8_t		unused[4];
} XBUSSTRU_TELE_LAPTIMER;

//////////////////////////////////////////////////////////////////////////////
//
//							TEXT GENERATOR
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;
	uint8_t		sID;															// Secondary ID
	uint8_t		lineNumber;														// Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
	char		text[13];														// 0-terminated text when < 13 chars
} XBUSSTRU_TELE_TEXTGEN;

//////////////////////////////////////////////////////////////////////////////
//
//							VIDEO TRANSMITTER (VTX)
//
//////////////////////////////////////////////////////////////////////////////
//
//	VTX spec subject to change. Refer to Spektrum VTX Interfacing document for latest info
//
typedef struct
{
	uint8_t		identifier;
	uint8_t		sID;															// Secondary ID
	uint8_t		band;															// VTX Band (0 = Fatshark, 1 = Raceband, 2 = E, 3 = B, 4 = A, 5-7 = Reserved)
	uint8_t		channel;														// VTX Channel (0-7)
	uint8_t		pit;															// Pit/Race mode (0 = Race, 1 = Pit). Race = (normal operating) mode. Pit = (reduced power) mode. When PIT is set, it overrides all other power settings.
	uint8_t		power;															// VTX Power (0 = Off, 1 = 1mw to 14mW, 2 = 15mW to 25mW, 3 = 26mW to 99mW, 4 = 100mW to 299mW, 5 = 300mW to 600mW, 6 = 601mW+, 7 = manual control)
	uint16_t	powerDec;														// VTX Power as a decimal 1mw/unit
	uint8_t		region;															// Region (0 = USA, 1 = EU, 0xFF = Not Provided)
	uint8_t		unused[7];														// reserved
} XBUSSTRU_TELE_VTX;

//////////////////////////////////////////////////////////////////////////////
//
//							ESC
//
//////////////////////////////////////////////////////////////////////////////
//
//	Uses big-endian byte order
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x20
	uint8_t		sID;															// Secondary ID
	uint16_t	RPM;															// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
	uint16_t	voltsInput;														// Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
	uint16_t	tempFET;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	uint16_t	currentMotor;													// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
	uint16_t	tempBEC;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	uint8_t		currentBEC;														// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
	uint8_t		voltsBEC;														// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
	uint8_t		throttle;														// 0.5% (0-100%)                  0xFF ----> "No data"
	uint8_t		powerOut;														// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
} XBUSSTRU_TELE_ESC;

//////////////////////////////////////////////////////////////////////////////
//
//		(Liquid) Fuel Flow/Capacity (Two Tanks/Engines)
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x22
	uint8_t		sID;															// Secondary ID
	uint16_t	fuelConsumed_A;													// Integrated fuel consumption, 0.1mL
	uint16_t	flowRate_A;														// Instantaneous consumption, 0.01mL/min
	uint16_t	temp_A;															// Temperature, 0.1C (0-655.34C)
	uint16_t	fuelConsumed_B;													// Integrated fuel consumption, 0.1mL
	uint16_t	flowRate_B;														// Instantaneous consumption, 0.01mL/min
	uint16_t	temp_B;															// Temperature, 0.1C (0-655.34C)
	uint16_t	spare;															// Not used
} XBUSSTRU_TELE_FUEL;

//////////////////////////////////////////////////////////////////////////////
//
//		Battery Current/Capacity (Flight Pack Capacity)
//
//////////////////////////////////////////////////////////////////////////////
//
// AK 2013-11-19 make struct align with 0x03 device
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x34
	uint8_t		sID;															// Secondary ID
	int16_t		current_A;														// Instantaneous current, 0.1A (0-3276.6A)
	int16_t		chargeUsed_A;													// Integrated mAh used, 1mAh (0-32.766Ah)
	uint16_t	temp_A;															// Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
	int16_t		current_B;														// Instantaneous current, 0.1A (0-3276.6A)
	int16_t		chargeUsed_B;													// Integrated mAh used, 1mAh (0-32.766Ah)
	uint16_t	temp_B;															// Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
	uint16_t	spare;															// Not used
} XBUSSTRU_TELE_FP_MAH;

//////////////////////////////////////////////////////////////////////////////
//
//		Digital Input Status (Retract Status) and Tank Pressure
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = TELE_DEVICE_DIGITAL_AIR
	uint8_t		sID;															// Secondary ID
	uint16_t	digital;														// Digital inputs (bit per input)
	uint16_t	spare1;
	uint16_t	pressure[4];													// Tank pressure, 0.1PSI (0-6553.4PSI), 0xFFFF = Not Installed
	uint16_t	spare2;
} XBUSSTRU_TELE_DIGITAL_AIR;

//////////////////////////////////////////////////////////////////////////////
//
//		Thrust/Strain Gauge
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x38
	uint8_t		sID;															// Secondary ID
	uint16_t	strain_A,														// Strain sensor A
				strain_B,														// Strain sensor B
				strain_C,														// Strain sensor D
				strain_D;														// Strain sensor C
} XBUSSTRU_TELE_STRAIN;

//////////////////////////////////////////////////////////////////////////////
//
//						6S LiPo Cell Monitor
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x3A
	uint8_t		sID;															// Secondary ID
	uint16_t	cell[6];														// Voltage across cell 1, .01V steps
																				// 0x7FFF --> cell not present
	uint16_t	temp;															// Temperature, 0.1C (0-655.34C)
} XBUSSTRU_TELE_LIPOMON;

//////////////////////////////////////////////////////////////////////////////
//
//						14S LiPo Cell Monitor
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x3F
	uint8_t		sID;															// Secondary ID
	uint8_t		cell[14];														// Voltage across cell 1, .01V steps, excess of 2.56V
																				// (ie, 3.00V would report 300-256 = 44)
																				// 0xFF --> cell not present
} XBUSSTRU_TELE_LIPOMON_14;

//////////////////////////////////////////////////////////////////////////////
//
//								Smart Battery
//
//////////////////////////////////////////////////////////////////////////////
//
//			Uses little-endian byte order for all multi-byte fields
//
typedef struct
{
	uint8_t	identifier;															// Source device = 0x42
	uint8_t	sID;																// Secondary ID
	uint8_t	typeChannel;														// Upper nybble = Message type; Lower nybble = Battery number (0 or 1)
	uint8_t	msgData[13];														// Message-specific data, determined by upper nybble of typeChannel (see defs below)
} STRU_SMARTBATT_HEADER;

#define	SMARTBATT_MSG_TYPE_MASK_BATTNUMBER		(0x0F)
#define	SMARTBATT_MSG_TYPE_MASK_MSGTYPE			(0xF0)

#define SMARTBATT_MSG_TYPE_REALTIME				(0x00)
#define SMARTBATT_MSG_TYPE_CELLS_1_6			(0x10)
#define SMARTBATT_MSG_TYPE_CELLS_7_12			(0x20)
#define SMARTBATT_MSG_TYPE_CELLS_13_18			(0x30)
#define SMARTBATT_MSG_TYPE_ID					(0x80)
#define SMARTBATT_MSG_TYPE_LIMITS				(0x90)

//...........................................................................
// Real-time battery data when current sense is available
typedef struct
{
	uint8_t		identifier;														// Source device = 0x42
	uint8_t		sID;															// Secondary ID
	uint8_t		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_REALTIME | Battery number (0 or 1)
	int8_t		temperature_C;													// Temperature in degrees C, 1 degree increments (-128 = unavailable)
	uint32_t	dischargeCurrent_mA;											// Amount of current being drawn from battery, in mA steps (0xFFFFFFFF = unavailable)
	uint16_t	batteryCapacityUsage_mAh;										// Approximate battery capacity usage, in mAh (0xFFFF = unavailable)
	uint16_t	minCellVoltage_mV;												// Minimum cell voltage of pack, in mV
	uint16_t	maxCellVoltage_mV;												// Maximum cell voltage of pack, in mV
	uint8_t		rfu[2];
} STRU_SMARTBATT_REALTIME;

//...........................................................................
// Real-time cell voltage
typedef struct
{
	uint8_t		identifier;														// Source device = 0x42
	uint8_t		sID;															// Secondary ID
	uint8_t		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_CELLS_X_Y | Battery number (0 or 1)
	int8_t		temperature_C;													// Temperature in degrees C, 1 degree increments (-128 = unavailable)
	uint16_t	cellVoltage_mV[6];												// Cell voltage of first 6 cells, in mV (0xFFFF = unavailable)
} STRU_SMARTBATT_CELLS;

//...........................................................................
// Smart Battery ID and general info
typedef struct
{
	uint8_t		identifier;														// Source device = 0x42
	uint8_t		sID;															// Secondary ID
	uint8_t		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_ID | Battery number (0 or 1)
	uint8_t		chemistry;														// 0:LiHv, 1:LiPo, 2:LiIon, 3:LiFe, 4:Pb, 5:Ni-MH/Cd
	uint8_t		numOfCells;														// Number of cells in the battery
	uint8_t		manufacturer;													// 0:BattGo
	uint16_t	cycles;															// Number of charge/discharge cycles recorded (0 = unavailable)
	uint8_t		uniqueID[8];													// Unique battery ID, manufacturer-specific
																				// 0: [0]   = lower (first) byte of "Customer ID"
																				//    [1-3] = lower 3 bytes of "Special Mark of Battery"
																				//    [4-7] = 4-byte "Manufacturing Date"
} STRU_SMARTBATT_ID;

//...........................................................................
// Smart Battery Limits
typedef struct
{
	uint8_t		identifier;														// Source device = 0x42
	uint8_t		sID;															// Secondary ID
	uint8_t		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_LIMITS | Battery number (0 or 1)
	uint8_t		rfu;
	uint16_t	fullCapacity_mAh;												// Fully charged battery capacity, in mAh
	uint16_t	dischargeCurrentRating;											// Rated discharge current, in 0.1C
	uint16_t	overDischarge_mV;												// Limit below which battery is likely damaged, in mV
	uint16_t	zeroCapacity_mV;												// Voltage at which LVC protection should activate, in mV
	uint16_t	fullyCharged_mV;												// Voltage reading expected when fully charged, in mV
	int8_t		minWorkingTemp;													// Minimum working temperature in degrees C, 1 degree steps
	int8_t		maxWorkingTemp;													// Maximum working temperature in degrees C, 1 degree steps
} STRU_SMARTBATT_LIMITS;

//////////////////////////////////////////////////////////////////////////////
//
//							ACCELEROMETER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x14
	uint8_t		sID;															// Secondary ID
	int16_t		GForceX;														// force is reported as .01G increments
	int16_t		GForceY;														// 		Range = +/-4000 (+/- 40G) in Pro model
	int16_t		GForceZ;														// 		Range = +/-800 (+/- 8G) in Standard model
	int16_t		maxGForceX;														// abs(max G X-axis)   FORE/AFT
	int16_t		maxGForceY;														// abs (max G Y-axis)  LEFT/RIGHT
	int16_t		maxGForceZ;														// max G Z-axis        WING SPAR LOAD
	int16_t		minGForceZ;														// min G Z-axis        WING SPAR LOAD
} XBUSSTRU_TELE_G_METER;

//////////////////////////////////////////////////////////////////////////////
//
//						SURFACE TILT (ATTITUDE) SENSOR
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x1C TELE_DEVICE_TILT
	uint8_t		sID;															// Secondary ID
	int16_t		attQuatX;														// Quaternion representing attitude using RHR. X component in Q14.
	int16_t		attQuatY;														// Y component in Q14.
	int16_t		attQuatZ;														// Z component in Q14.
	int16_t		attQuatW;														// W component in Q14.
	uint16_t		spare[3];
} XBUSSTRU_TELE_TILT;

//////////////////////////////////////////////////////////////////////////////
//
//								TURBINE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x15
	uint8_t		sID;															// Secondary ID
	uint8_t		status;															// Table below
	uint8_t		throttle;														// (BCD) xx Percent
	uint16_t	packVoltage;													// (BCD) xx.yy
	uint16_t	pumpVoltage;													// (BCD) xx.yy
	uint32_t	RPM;															// (BCD)
	uint16_t	EGT;															// (BCD) Temperature, Celsius
	uint8_t		offCondition;													// Table below
	uint8_t		spare;
} XBUSSTRU_TELE_JETCAT;

enum XBUS_JETCAT_ECU_TURBINE_STATE {							// ECU Status definitions
		XBUS_JETCAT_ECU_STATE_OFF = 0x00,
		XBUS_JETCAT_ECU_STATE_WAIT_for_RPM = 0x01, // (Stby/Start)
		XBUS_JETCAT_ECU_STATE_Ignite = 0x02,
		XBUS_JETCAT_ECU_STATE_Accelerate = 0x03,
		XBUS_JETCAT_ECU_STATE_Stabilise = 0x04,
		XBUS_JETCAT_ECU_STATE_Learn_HI = 0x05,
		XBUS_JETCAT_ECU_STATE_Learn_LO = 0x06,
		XBUS_JETCAT_ECU_STATE_UNDEFINED = 0x07,
		XBUS_JETCAT_ECU_STATE_Slow_Down = 0x08,
		XBUS_JETCAT_ECU_STATE_Manual = 0x09,
		XBUS_JETCAT_ECU_STATE_AutoOff = 0x10,
		XBUS_JETCAT_ECU_STATE_Run = 0x11, // (reg.)
		XBUS_JETCAT_ECU_STATE_Accleleration_delay = 0x12,
		XBUS_JETCAT_ECU_STATE_SpeedReg = 0x13, // (Speed Ctrl)
		XBUS_JETCAT_ECU_STATE_Two_Shaft_Regulate = 0x14, // (only for secondary shaft)
		XBUS_JETCAT_ECU_STATE_PreHeat1 = 0x15,
		XBUS_JETCAT_ECU_STATE_PreHeat2 = 0x16,
		XBUS_JETCAT_ECU_STATE_MainFStart = 0x17,
		XBUS_JETCAT_ECU_STATE_NotUsed = 0x18,
		XBUS_JETCAT_ECU_STATE_KeroFullOn = 0x19,
		// undefined states 0x1A-0x1F
		XBUS_EVOJET_ECU_STATE_off = 0x20,
		XBUS_EVOJET_ECU_STATE_ignt = 0x21,
		XBUS_EVOJET_ECU_STATE_acce = 0x22,
		XBUS_EVOJET_ECU_STATE_run = 0x23,
		XBUS_EVOJET_ECU_STATE_cal = 0x24,
		XBUS_EVOJET_ECU_STATE_cool = 0x25,
		XBUS_EVOJET_ECU_STATE_fire = 0x26,
		XBUS_EVOJET_ECU_STATE_glow = 0x27,
		XBUS_EVOJET_ECU_STATE_heat = 0x28,
		XBUS_EVOJET_ECU_STATE_idle = 0x29,
		XBUS_EVOJET_ECU_STATE_lock = 0x2A,
		XBUS_EVOJET_ECU_STATE_rel = 0x2B,
		XBUS_EVOJET_ECU_STATE_spin = 0x2C,
		XBUS_EVOJET_ECU_STATE_stop = 0x2D,
		// undefined states 0x2E-0x2F
		XBUS_HORNET_ECU_STATE_OFF = 0x30,
		XBUS_HORNET_ECU_STATE_SLOWDOWN = 0x31,
		XBUS_HORNET_ECU_STATE_COOL_DOWN = 0x32,
		XBUS_HORNET_ECU_STATE_AUTO = 0x33,
		XBUS_HORNET_ECU_STATE_AUTO_HC = 0x34,
		XBUS_HORNET_ECU_STATE_BURNER_ON = 0x35,
		XBUS_HORNET_ECU_STATE_CAL_IDLE = 0x36,
		XBUS_HORNET_ECU_STATE_CALIBRATE = 0x37,
		XBUS_HORNET_ECU_STATE_DEV_DELAY = 0x38,
		XBUS_HORNET_ECU_STATE_EMERGENCY = 0x39,
		XBUS_HORNET_ECU_STATE_FUEL_HEAT = 0x3A,
		XBUS_HORNET_ECU_STATE_FUEL_IGNITE = 0x3B,
		XBUS_HORNET_ECU_STATE_GO_IDLE = 0x3C,
		XBUS_HORNET_ECU_STATE_PROP_IGNITE = 0x3D,
		XBUS_HORNET_ECU_STATE_RAMP_DELAY = 0x3E,
		XBUS_HORNET_ECU_STATE_RAMP_UP = 0x3F,
		XBUS_HORNET_ECU_STATE_STANDBY = 0x40,
		XBUS_HORNET_ECU_STATE_STEADY = 0x41,
		XBUS_HORNET_ECU_STATE_WAIT_ACC = 0x42,
		XBUS_HORNET_ECU_STATE_ERROR = 0x43,
		// undefined states 0x44-0x4F
		XBUS_XICOY_ECU_STATE_Temp_High = 0x50,
		XBUS_XICOY_ECU_STATE_Trim_Low = 0x51,
		XBUS_XICOY_ECU_STATE_Set_Idle = 0x52,
		XBUS_XICOY_ECU_STATE_Ready = 0x53,
		XBUS_XICOY_ECU_STATE_Ignition = 0x54,
		XBUS_XICOY_ECU_STATE_Fuel_Ramp = 0x55,
		XBUS_XICOY_ECU_STATE_Glow_Test = 0x56,
		XBUS_XICOY_ECU_STATE_Running = 0x57,
		XBUS_XICOY_ECU_STATE_Stop = 0x58,
		XBUS_XICOY_ECU_STATE_Flameout = 0x59,
		XBUS_XICOY_ECU_STATE_Speed_Low = 0x5A,
		XBUS_XICOY_ECU_STATE_Cooling = 0x5B,
		XBUS_XICOY_ECU_STATE_Igniter_Bad = 0x5C,
		XBUS_XICOY_ECU_STATE_Starter_F = 0x5D,
		XBUS_XICOY_ECU_STATE_Weak_Fuel = 0x5E,
		XBUS_XICOY_ECU_STATE_Start_On = 0x5F,
		XBUS_XICOY_ECU_STATE_Pre_Heat = 0x60,
		XBUS_XICOY_ECU_STATE_Battery = 0x61,
		XBUS_XICOY_ECU_STATE_Time_Out = 0x62,
		XBUS_XICOY_ECU_STATE_Overload = 0x63,
		XBUS_XICOY_ECU_STATE_Igniter_Fail = 0x64,
		XBUS_XICOY_ECU_STATE_Burner_On = 0x65,
		XBUS_XICOY_ECU_STATE_Starting = 0x66,
		XBUS_XICOY_ECU_STATE_SwitchOver = 0x67,
		XBUS_XICOY_ECU_STATE_Cal_Pump = 0x68,
		XBUS_XICOY_ECU_STATE_Pump_Limit = 0x69,
		XBUS_XICOY_ECU_STATE_No_Engine = 0x6A,
		XBUS_XICOY_ECU_STATE_Pwr_Boost = 0x6B,
		XBUS_XICOY_ECU_STATE_Run_Idle = 0x6C,
		XBUS_XICOY_ECU_STATE_Run_Max = 0x6D,
		// undefined states 0x6e-0x73
		XBUS_JETCENT_ECU_STATE_STOP = 0x74,
		XBUS_JETCENT_ECU_STATE_GLOW_TEST = 0x75,
		XBUS_JETCENT_ECU_STATE_STARTER_TEST = 0x76,
		XBUS_JETCENT_ECU_STATE_PRIME_FUEL = 0x77,
		XBUS_JETCENT_ECU_STATE_PRIME_BURNER = 0x78,
		XBUS_JETCENT_ECU_STATE_MAN_COOL = 0x79,
		XBUS_JETCENT_ECU_STATE_AUTO_COOL = 0x7A,
		XBUS_JETCENT_ECU_STATE_IGN_HEAT = 0x7B,
		XBUS_JETCENT_ECU_STATE_IGNITION = 0x7C,
		XBUS_JETCENT_ECU_STATE_PREHEAT = 0x7D,
		XBUS_JETCENT_ECU_STATE_SWITCHOVER = 0x7E,
		XBUS_JETCENT_ECU_STATE_TO_IDLE = 0x7F,
		XBUS_JETCENT_ECU_STATE_RUNNING = 0x80,
		XBUS_JETCENT_ECU_STATE_STOP_ERROR = 0x81,
		// undefined states 0x82-0x8F
		XBUS_SWIWIN_ECU_STATE_STOP = 0x90,
		XBUS_SWIWIN_ECU_STATE_READY = 0x91,
		XBUS_SWIWIN_ECU_STATE_IGNITION = 0x92,
		XBUS_SWIWIN_ECU_STATE_PREHEAT = 0x93,
		XBUS_SWIWIN_ECU_STATE_FUEL_RAMP = 0x94,
		XBUS_SWIWIN_ECU_STATE_RUNNING = 0x95,
		XBUS_SWIWIN_ECU_STATE_COOLING = 0x96,
		XBUS_SWIWIN_ECU_STATE_RESTART_SWOVER = 0x97,
		XBUS_SWIWIN_ECU_STATE_NOTUSED = 0x98,
		// undefined states 0x99-0x9F

		XBUS_TURBINE_ECU_MAX_STATE = 0x9F
};

enum XBUS_JETCAT_ECU_OFF_CONDITIONS {					// ECU off conditions. Valid only when the ECUStatus = XBUS_JETCAT_ECU_STATE_OFF
		XBUS_JETCAT_ECU_OFF_No_Off_Condition_defined = 0,
		XBUS_JETCAT_ECU_OFF_Shut_down_via_RC,
		XBUS_JETCAT_ECU_OFF_Overtemperature,
		XBUS_JETCAT_ECU_OFF_Ignition_timeout,
		XBUS_JETCAT_ECU_OFF_Acceleration_time_out,
		XBUS_JETCAT_ECU_OFF_Acceleration_too_slow,
		XBUS_JETCAT_ECU_OFF_Over_RPM,
		XBUS_JETCAT_ECU_OFF_Low_Rpm_Off,
		XBUS_JETCAT_ECU_OFF_Low_Battery,
		XBUS_JETCAT_ECU_OFF_Auto_Off,
		XBUS_JETCAT_ECU_OFF_Low_temperature_Off,
		XBUS_JETCAT_ECU_OFF_Hi_Temp_Off,
		XBUS_JETCAT_ECU_OFF_Glow_Plug_defective,
		XBUS_JETCAT_ECU_OFF_Watch_Dog_Timer,
		XBUS_JETCAT_ECU_OFF_Fail_Safe_Off,
		XBUS_JETCAT_ECU_OFF_Manual_Off, // (via GSU)
		XBUS_JETCAT_ECU_OFF_Power_fail, // (Battery fail)
		XBUS_JETCAT_ECU_OFF_Temp_Sensor_fail, // (only during startup)
		XBUS_JETCAT_ECU_OFF_Fuel_fail,
		XBUS_JETCAT_ECU_OFF_Prop_fail,
		XBUS_JETCAT_ECU_OFF_2nd_Engine_fail,
		XBUS_JETCAT_ECU_OFF_2nd_Engine_Diff_Too_High,
		XBUS_JETCAT_ECU_OFF_2nd_Engine_No_Comm,
		XBUS_JETCAT_ECU_MAX_OFF_COND,
		// Jet Central
		XBUS_JETCENT_ECU_OFF_No_Off_Condition_defined = 24,		// ECU off conditions. Valid only when the ECUStatus = XBUS_JETCENT_ECU_STATE_STOP or XBUS_JETCENT_ECU_STATE_STOP_ERROR or XBUS_JETCENT_ECU_STATE_RUNNING
		XBUS_JETCENT_ECU_OFF_IGNITION_ERROR,
		XBUS_JETCENT_ECU_OFF_PREHEAT_ERROR,
		XBUS_JETCENT_ECU_OFF_SWITCHOVER_ERROR,
		XBUS_JETCENT_ECU_OFF_STARTER_MOTOR_ERROR,
		XBUS_JETCENT_ECU_OFF_TO_IDLE_ERROR,
		XBUS_JETCENT_ECU_OFF_ACCELERATION_ERROR,
		XBUS_JETCENT_ECU_OFF_IGNITER_BAD,
		XBUS_JETCENT_ECU_OFF_MIN_PUMP_OK,
		XBUS_JETCENT_ECU_OFF_MAX_PUMP_OK,
		XBUS_JETCENT_ECU_OFF_LOW_RX_BATTERY,
		XBUS_JETCENT_ECU_OFF_LOW_ECU_BATTERY,
		XBUS_JETCENT_ECU_OFF_NO_RX,
		XBUS_JETCENT_ECU_OFF_TRIM_DOWN,
		XBUS_JETCENT_ECU_OFF_TRIM_UP,
		XBUS_JETCENT_ECU_OFF_FAILSAFE,
		XBUS_JETCENT_ECU_OFF_FULL,
		XBUS_JETCENT_ECU_OFF_RX_SETUP_ERROR,
		XBUS_JETCENT_ECU_OFF_TEMP_SENSOR_ERROR,
		XBUS_JETCENT_ECU_OFF_COM_TURBINE_ERROR,
		XBUS_JETCENT_ECU_OFF_MAX_TEMP,
		XBUS_JETCENT_ECU_OFF_MAX_AMPS,
		XBUS_JETCENT_ECU_OFF_LOW_RPM,
		XBUS_JETCENT_ECU_OFF_ERROR_RPM_SENSOR,
		XBUS_JETCENT_ECU_OFF_MAX_PUMP,
		XBUS_JETCENT_ECU_MAX_OFF_COND
};

typedef struct
{
	uint8_t		identifier;														// Source device = 0x19
	uint8_t		sID;															// Secondary ID
	uint16_t	FuelFlowRateMLMin;												// (BCD) mL per Minute
	uint32_t	RestFuelVolumeInTankML;											// (BCD) mL remaining in tank
	uint8_t		ECUbatteryPercent;												// (BCD) % battery pack capacity remaining
	// 7 bytes left
} XBUSSTRU_TELE_JETCAT2;

//////////////////////////////////////////////////////////////////////////////
//
//								GPS
//						  Packed-BCD Type
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x16
	uint8_t		sID;															// Secondary ID
	uint16_t	altitudeLow;													// BCD, meters, format 3.1 (Low order of altitude)
	uint32_t	latitude;														// BCD, format 4.4, Degrees * 100 + minutes, less than 100 degrees
	uint32_t	longitude;														// BCD, format 4.4 , Degrees * 100 + minutes, flag indicates > 99 degrees
	uint16_t	course;															// BCD, 3.1
	uint8_t		HDOP;															// BCD, format 1.1
	uint8_t		GPSflags;														// see definitions below
} XBUSSTRU_TELE_GPS_LOC;

typedef struct
{
	uint8_t		identifier;														// Source device = 0x17
	uint8_t		sID;															// Secondary ID
	uint16_t	speed;															// BCD, knots, format 3.1
	uint32_t	UTC;															// BCD, format HH:MM:SS.S, format 6.1
	uint8_t		numSats;														// BCD, 0-99
	uint8_t		altitudeHigh;													// BCD, meters, format 2.0 (High order of altitude)
} XBUSSTRU_TELE_GPS_STAT;

// GPS flags definitions:
#define	GPS_INFO_FLAGS_IS_NORTH_BIT					(0)
#define	GPS_INFO_FLAGS_IS_NORTH						(1 << GPS_INFO_FLAGS_IS_NORTH_BIT)
#define	GPS_INFO_FLAGS_IS_EAST_BIT					(1)
#define	GPS_INFO_FLAGS_IS_EAST						(1 << GPS_INFO_FLAGS_IS_EAST_BIT)
#define	GPS_INFO_FLAGS_LONGITUDE_GREATER_99_BIT		(2)
#define	GPS_INFO_FLAGS_LONGITUDE_GREATER_99			(1 << GPS_INFO_FLAGS_LONGITUDE_GREATER_99_BIT)
#define	GPS_INFO_FLAGS_GPS_FIX_VALID_BIT			(3)
#define	GPS_INFO_FLAGS_GPS_FIX_VALID				(1 << GPS_INFO_FLAGS_GPS_FIX_VALID_BIT)
#define	GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT		(4)
#define	GPS_INFO_FLAGS_GPS_DATA_RECEIVED			(1 << GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT)
#define	GPS_INFO_FLAGS_3D_FIX_BIT					(5)
#define	GPS_INFO_FLAGS_3D_FIX						(1 << GPS_INFO_FLAGS_3D_FIX_BIT)
#define GPS_INFO_FLAGS_NEGATIVE_ALT_BIT				(7)
#define GPS_INFO_FLAGS_NEGATIVE_ALT					(1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT)

//////////////////////////////////////////////////////////////////////////////
//
//								GPS
//							Binary Type
//
//		NOTE:	Data resolution for all fields matches Crossfire EXCEPT speed.
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x16
	uint8_t		sID;															// Secondary ID
	uint16_t	altitude;														// m, 1000m offset
	int32_t		latitude;														// degree / 10,000,000
	int32_t		longitude;														// degree / 10,000,000
	uint16_t	heading;														// degree / 10
	uint8_t		groundSpeed;													// km/h
	uint8_t		numSats;														// count
} XBUSSTRU_TELE_GPS_BINARY;

//////////////////////////////////////////////////////////////////////////////
//
//					AS3X Legacy Gain Report
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = TELE_DEVICE_AS3X_LEGACYGAIN
	uint8_t		sID;															// Secondary ID
	uint8_t		gainRoll;														// Configured normal gains per axis
	uint8_t		gainPitch;
	uint8_t		gainYaw;
	uint8_t		headRoll;														// Configured heading hold gains per axis
	uint8_t		headPitch;
	uint8_t		headYaw;
	uint8_t		activeRoll;														// Active gains per axis (as affected by FM channel)
	uint8_t		activePitch;
	uint8_t		activeYaw;
	uint8_t		flightMode;														// bit 7 1 --> FM present in bits 0,1 except 0xFF --> not present
	uint8_t		unused[4];
} XBUSSTRU_TELE_AS3X_LEGACY;

//////////////////////////////////////////////////////////////////////////////
//
//					AS6X Gain Report (AS3X Legacy + more fields)
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = TELE_DEVICE_AS6X_GAIN
	uint8_t		sID;															// Secondary ID
	uint8_t		gainRoll;														// Configured normal gains per axis
	uint8_t		gainPitch;
	uint8_t		gainYaw;
	uint8_t		headRoll;														// Configured heading hold gains per axis
	uint8_t		headPitch;
	uint8_t		headYaw;
	uint8_t		activeRoll;														// Active gains per axis (as affected by FM channel)
	uint8_t		activePitch;
	uint8_t		activeYaw;
	uint8_t		flightMode;														// bit 7 1 --> FM present in bits 0,1 except 0xFF --> not present
	// new fields go here:
	uint8_t		unused[4];
} XBUSSTRU_TELE_AS6X_GAIN;

//////////////////////////////////////////////////////////////////////////////
//
//							GYRO
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x1A
	uint8_t		sID;															// Secondary ID
	int16_t		gyroX;															// Rotation rates of the body - Rate is about the X Axis which is defined out the nose of the vehicle.
	int16_t		gyroY;															// Units are 0.1 deg/sec  - Rate is about the Y Axis which is define out the right wing of the vehicle.
	int16_t		gyroZ;															// Rate is about the Z axis which is defined down from the vehicle.
	int16_t		maxGyroX;														// Max rates (absolute value)
	int16_t		maxGyroY;
	int16_t		maxGyroZ;
} XBUSSTRU_TELE_GYRO;

//////////////////////////////////////////////////////////////////////////////
//
//						Alpha6 Stabilizer
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x24
	uint8_t		sID;															// Secondary ID
	uint16_t	volts;															// 0.01V increments
	uint8_t		state_FM;														// Flight Mode and System State (see below)
	uint8_t		gainRoll,														// Roll Gain,  high bit --> Heading Hold
				gainPitch,														// Pitch Gain
				gainYaw;														// Yaw Gain
	int16_t		attRoll,														// Roll Attitude, 0.1degree, RHR
				attPitch,														// Pitch Attitude
				attYaw;															// Yaw Attitude
	uint16_t	spare;
} XBUSSTRU_TELE_ALPHA6;

#define	GBOX_STATE_BOOT							(0x00)							// Alpha6 State - Boot
#define	GBOX_STATE_INIT							(0x01)							// Init
#define	GBOX_STATE_READY						(0x02)							// Ready
#define	GBOX_STATE_SENSORFAULT					(0x03)							// Sensor Fault
#define	GBOX_STATE_POWERFAULT					(0x04)							// Power Fault
#define	GBOX_STATE_MASK							(0x0F)

#define	GBOX_FMODE_FM0							(0x00)							// FM0 through FM4
#define	GBOX_FMODE_FM1							(0x10)
#define	GBOX_FMODE_FM2							(0x20)
#define	GBOX_FMODE_FM3							(0x30)
#define	GBOX_FMODE_FM4							(0x40)
#define	GBOX_FMODE_PANIC						(0x50)
#define	GBOX_FMODE_MASK							(0xF0)

//////////////////////////////////////////////////////////////////////////////
//
//						ATTITUDE & MAG COMPASS
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x1B
	uint8_t		sID;															// Secondary ID
	int16_t		attRoll;														// Attitude, 3 axes.  Roll is a rotation about the X Axis of the vehicle using the RHR.
	int16_t		attPitch;														// Units are 0.1 deg - Pitch is a rotation about the Y Axis of the vehicle using the RHR.
	int16_t		attYaw;															// Yaw is a rotation about the Z Axis of the vehicle using the RHR.
	int16_t		magX;															// Magnetic Compass, 3 axes
	int16_t		magY;															// Units are 0.1mG
	int16_t		magZ;															//
	uint16_t	heading;														// Heading, 0.1deg
} XBUSSTRU_TELE_ATTMAG;

//////////////////////////////////////////////////////////////////////////////
//
//						Altitude "Zero" Message
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x7B
	uint8_t		sID;															// Secondary ID
	uint8_t		spare[2];
	uint32_t	altOffset;														// Altitude "zero" log
} XBUSSTRU_TELE_ALT_ZERO;

//////////////////////////////////////////////////////////////////////////////
//
//						Real-Time Clock
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x7C
	uint8_t		sID;															// Secondary ID
	uint8_t		spare[6];
	uint64_t	UTC64;															// Linux 64-bit time_t for post-2038 date compatibility
} XBUSSTRU_TELE_RTC;

//////////////////////////////////////////////////////////////////////////////
//
//						V-Speak (Placeholder)
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x61
	uint8_t		sID;															// Secondary ID
	uint8_t		spare[14];														// Format TBD by V-Speak
} XBUSSTRU_TELE_V_SPEAK;

//////////////////////////////////////////////////////////////////////////////
//
//						www.Smoke-EL.de
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x61
	uint8_t		sID;															// Secondary ID
	uint16_t	batteryV;														// 0.01V, Range 0.00-70.00V
	uint16_t	countdown;														// 0.01s, Range 0.00-30.00s
	int16_t		GForce;															// 0.01g, Range = +/-8.00g
	uint8_t		cutoff;															// 1 count, Range 0-9
	uint8_t		connected;														// 0=not connected, 1=connected, x = TBD
	uint16_t	spare[3];
} XBUSSTRU_TELE_SMOKE_EL;

//////////////////////////////////////////////////////////////////////////////
//
//							MULTI-CYLINDER SENSOR
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = TELE_DEVICE_MULTICYLINDER
	uint8_t		sID;															// Secondary ID
	uint8_t		temperature[9];													// Temperature, 1C increments, Offset = 30C, 0xFF = NO DATA
				// 0x00 = 30C		(86F)
				// 0x01 = 31C ...	(88F)
				// 0xFE = 284C		(543F)
				// 0xFF = NO SENSOR ATTACHED.  Note that sensors must be installed cylinder 1-9 in sequence!
	uint8_t		throttlePct;													// Throttle percent (0-100% typical, 0xFF = NO DATA)
	uint16_t		RPM;															// 4 RPM increments, Offset = 400RPM, range 404-16776.
				// 0x000 = 0 RPM
				// 0x001 = 404 RPM
				// 0x002 = 408 RPM
				// 0xFFE = 16776 RPM
				// 0xFFF = NO SENSOR ATTACHED
				// NOTE:  HI NYBBLE RESERVED, set to 0xF to mark "NO DATA" for now
	uint8_t		batteryV;														// Voltage, 0.1V increments, Offset = 3.5V, 0xFF = NO DATA
				// 0x00 = 3.5V
				// 0x01 = 3.6V
				// 0xFE = 28.9V
				// 0xFF = NO SENSOR ATTACHED
	uint8_t		spare;															// 0xFF --> no data
} XBUSSTRU_TELE_MULTI_TEMP;

//////////////////////////////////////////////////////////////////////////////
//
//						Transmitter Frame Data
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x7D
	uint8_t		sID;															// Secondary ID
	uint16_t	chanData[7];													// Channel Data array
} XBUSSTRU_TELE_FRAMEDATA;

//////////////////////////////////////////////////////////////////////////////
//
//							AHRS Monitor
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = TELE_DEVICE_AHRS
	uint8_t		sID;															// Secondary ID
	int16_t		attRoll;														// Attitude, 3 axes.  Roll is a rotation about the X Axis of the vehicle using the RHR.
	int16_t		attPitch;														// Units are 0.1 deg - Pitch is a rotation about the Y Axis of the vehicle using the RHR.
	int16_t		attYaw;															// Roll is a rotation about the Z Axis of the vehicle using the RHR.
	int16_t		altitude;														// .1m increments
	uint8_t		waypoint;														// Waypoint number
	uint8_t		spare8;
	uint16_t	spare16[2];
} XBUSSTRU_TELE_AHRS;																// AHRS data from rx

//////////////////////////////////////////////////////////////////////////////
//
//							FLIGHT MODE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x05 TELE_DEVICE_FLITECTRL
	uint8_t		sID;															// Secondary ID
	uint8_t		fMode,															// Current flight mode (low nybble)
				spare8;
	uint16_t	spare[6];														// Growth
	// Ideas -
	//		arming status in a bitmap
	//		time in state
} XBUSSTRU_TELE_FLITECTRL;


//////////////////////////////////////////////////////////////////////////////
//
//							Crossfire QOS
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	uint8_t		identifier;														// Source device = TELE_XRF_LINKSTATUS
	uint8_t		sID;															// Secondary ID
	uint8_t		ant1,					// dBm * -1
				ant2,
				quality;				// %
	int8_t		SNR;					// dB
	uint8_t		activeAnt,				// ant1=0, ant2=1
				RFmode,					// 4fps=0, 50fps, 150Hz
				upPower,				// 0mW=0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW
				downlink,				// dBm * -1
				qualityDown;			// %
	int8_t		SNRdown;				// dB
} XBUSSTRU_TELE_XF_QOS;

//////////////////////////////////////////////////////////////////////////////
//
//						RPM/Volts/Temperature
//
//////////////////////////////////////////////////////////////////////////////
//
//	Uses big-endian byte order
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x7E
	uint8_t		sID;															// Secondary ID
	uint16_t	microseconds;													// microseconds between pulse leading edges
	uint16_t	volts;															// 0.01V increments (typically flight pack voltage)
	int16_t		temperature;													// Temperature in degrees F.  0x7FFF = "No Data"
	int8_t		dBm_A,															// Avg RSSI in dBm (<-1 = dBm, 0 = no data, >0 = % range) -- (legacy)antenna A in dBm
				dBm_B;															// Avg RSSI in % (<-1 = dBm, 0 = no data, >0 = % range)   -- (legacy)antenna B in dBm
																				// Note: Legacy use as antenna A/B dBm values is still supported. If only 1 antenna, set B = A.
																				//       The "no data" value is 0, but -1 (0xFF) is treated the same for backwards compatibility
	uint16_t	spare[2];
} XBUSSTRU_TELE_RPM;

//////////////////////////////////////////////////////////////////////////////
//
//							QoS DATA
//
//////////////////////////////////////////////////////////////////////////////
//
//	NOTE:  AR6410-series send:
//			id = 7F
//			sID = 0
//			A = 0
//			B = 0
//			L = 0
//			R = 0
//			F = fades
//			H = holds
//			rxV = 0xFFFF
//
typedef struct
{
	uint8_t		identifier;														// Source device = 0x7F
	uint8_t		sID;															// Secondary ID
	uint16_t	A;																// Internal/base receiver fades. 0xFFFF = "No data"
	uint16_t	B;																// Remote receiver fades. 0xFFFF = "No data"
	uint16_t	L;																// Third receiver fades. 0xFFFF = "No data"
	uint16_t	R;																// Fourth receiver fades. 0xFFFF = "No data"
	uint16_t	F;																// Frame losses. 0xFFFF = "No data"
	uint16_t	H;																// Holds. 0xFFFF = "No data"
	uint16_t	rxVoltage;														// Volts, .01V increment. 0xFFFF = "No data"
} XBUSSTRU_TELE_QOS;

//////////////////////////////////////////////////////////////////////////////
//
//					UNION OF ALL DEVICE MESSAGES
//
//////////////////////////////////////////////////////////////////////////////
//
typedef union
{ 
  uint8_t             raw[16];       // AndWho added missing byte def.   
  uint16_t            raw_int[8];
	XBUSSTRU_TELE_QOS			qos;
	XBUSSTRU_TELE_RPM			rpm;
	XBUSSTRU_TELE_HV			hv;
	XBUSSTRU_TELE_TEMP			temp;
	XBUSSTRU_TELE_IHIGH			amps;
	XBUSSTRU_TELE_ALT			alt;
	XBUSSTRU_TELE_SPEED			speed;
	XBUSSTRU_TELE_ESC			escSPM;
	XBUSSTRU_TELE_VARIO_S		varioSimple;
	XBUSSTRU_TELE_G_METER		accel;
	XBUSSTRU_TELE_JETCAT		jetcat;
	XBUSSTRU_TELE_JETCAT2		jetcat2;
	XBUSSTRU_TELE_GPS_LOC		gpsloc;
	XBUSSTRU_TELE_GPS_STAT		gpsstat;
	XBUSSTRU_TELE_GPS_BINARY	gpsbin;
	XBUSSTRU_TELE_AS3X_LEGACY	as3x;
	XBUSSTRU_TELE_AS6X_GAIN		as6x;
	XBUSSTRU_TELE_GYRO			gyro;
	XBUSSTRU_TELE_ALPHA6		alpha6;
	XBUSSTRU_TELE_ATTMAG		attMag;
	XBUSSTRU_TELE_POWERBOX		powerBox;
	XBUSSTRU_TELE_RX_MAH		rxMAH;
	XBUSSTRU_TELE_FP_MAH		fpMAH;
	XBUSSTRU_TELE_ESC			esc;
	XBUSSTRU_TELE_FUEL			fuel;
	XBUSSTRU_TELE_DIGITAL_AIR	digAir;
	XBUSSTRU_TELE_STRAIN		strain;
	XBUSSTRU_TELE_LIPOMON		lipomon;
	XBUSSTRU_TELE_LIPOMON_14	lipomon14;
	STRU_SMARTBATT_HEADER	smartBatt_header;
	STRU_SMARTBATT_REALTIME	smartBatt_realtime;
	STRU_SMARTBATT_CELLS	smartBatt_cells;
	STRU_SMARTBATT_ID		smartBatt_ID;
	STRU_SMARTBATT_LIMITS	smartBatt_limits;
	XBUSSTRU_TELE_USER_16SU		user_16SU;
	XBUSSTRU_TELE_USER_16SU32U	user_16SU32U;
	XBUSSTRU_TELE_USER_16SU32S	user_16SU32S;
	XBUSSTRU_TELE_USER_16U32SU	user_16U32SU;
	XBUSSTRU_TELE_TEXTGEN		textgen;
	XBUSSTRU_TELE_VTX			vtx;
	XBUSSTRU_TELE_V_SPEAK		vSpeak;
	XBUSSTRU_TELE_SMOKE_EL		smoke_el;
	XBUSSTRU_TELE_MULTI_TEMP	multiCylinder;
	XBUSSTRU_TELE_FLITECTRL		fControl;
	XBUSSTRU_TELE_TILT			tilt;
	XBUSSTRU_TELE_XF_QOS		xfire;
} XBUS_UN_TELEMETRY;																	// All telemetry messages

//////////////////////////////////////////////////////////////////
//
//					sID Field Functionality
//
//////////////////////////////////////////////////////////////////
//
//		if .sID == 0x00 then .identifier = device type (TELE_DEVICE_xxx) and address I2C bus
//		if .sID != 0x00 then .sID = device type and .identifer = address on I2C bus

