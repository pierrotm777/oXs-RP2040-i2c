// Spektrum X-Bus for oXs RP2040 (first hardware test, MSRC i2c_multi PIO).
// Reuses the already-tested Spektrum telemetry payload definitions shared
// with oXs SRXL2, without coupling X-Bus to the SRXL2 UART or frame state.
#include "xbus.h"
#include <stdint.h>
#include "srxl_sensors.h"
#include "i2c_multi.h"
#include "config.h"
#include "param.h"
#include "tools.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern CONFIG config;
extern field fields[];
extern uint8_t debugTlm;
// Filled by oXs core0 from MPU data; mpu.cpp is unchanged.
extern bool gyroIsInstalled;
extern int16_t gyroX;
extern int16_t gyroY;
extern int16_t gyroZ;

namespace {
constexpr uint8_t kPacketLength = 16;
constexpr uint8_t kTxLength = 32;  // safe padding if a master requests extra bytes
constexpr uint32_t kRefreshMs = 100;
constexpr uint8_t kSensorCount = 10;

// MSRC i2c_multi consumes the buffer *after* the request callback returns.
// A static transmit buffer is mandatory (MSRC's uint8_t buffer[16] on the
// callback stack would have an invalid lifetime).
uint8_t tx[kTxLength] = {};
uint8_t published[kSensorCount][kPacketLength] = {};
uint32_t requests[kSensorCount] = {};
uint32_t lastUpdate = 0;
uint32_t lastDebug = 0;
bool ready = false;


enum SensorIdx : uint8_t { AIR, GMETER, GPS_LOC, GPS_STAT, ENERGY, GYRO, ESC, VARIO_IDX, RPM_TEMP, ADS1 };
constexpr uint8_t ids[kSensorCount] = {
    TELE_DEVICE_AIRSPEED, TELE_DEVICE_GMETER,
    TELE_DEVICE_GPS_LOC, TELE_DEVICE_GPS_STATS, TELE_DEVICE_RX_MAH,
    TELE_DEVICE_GYRO, TELE_DEVICE_ESC, TELE_DEVICE_VARIO_S, TELE_DEVICE_RPM,
    TELE_DEVICE_USER_16SU // 0x50: four ADS1115 inputs in millivolts
};

static int8_t indexForAddress(uint8_t addr) {
    for (uint8_t i = 0; i < kSensorCount; ++i) if (ids[i] == addr) return static_cast<int8_t>(i);
    return -1;
}
static inline bool has(fieldIdx f) { return fields[f].available; }
static inline int32_t val(fieldIdx f) { return fields[f].value; }
static int32_t clamped(int64_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : v > hi ? hi : static_cast<int32_t>(v);
}
static int32_t roundDiv(int64_t value, int32_t divisor) {
    return static_cast<int32_t>(value >= 0 ?
        (value + divisor / 2) / divisor : (value - divisor / 2) / divisor);
}
static uint32_t bcd(uint32_t v) {
    uint32_t out = 0;
    for (uint32_t shift = 0; shift < 32; shift += 4) {
        out |= (v % 10) << shift;
        v /= 10;
    }
    return out;
}
static uint16_t be16(int32_t v) {
    return swapBinary(static_cast<uint16_t>(v));
}

static int16_t xbusSigned16(int32_t v) {
    return static_cast<int16_t>(be16(v));
}
static int16_t absolute16(int16_t v) {
    return v < 0 ? static_cast<int16_t>(-int32_t(v)) : v;
}
// oXs ACC_* is in mg, Spektrum G-meter in 0.01 g.
static int16_t xbusAcc(int32_t mg) {
    return static_cast<int16_t>(clamped(roundDiv(mg, 10), -4000, 4000));
}
// The core0 gyro variable is normalized: 32768 ~= 2000 deg/s.
// Spektrum gyro is in 0.1 deg/s (32768 -> 20000).
static int16_t xbusGyro(int16_t raw) {
    return static_cast<int16_t>(clamped(roundDiv(int64_t(raw) * 625, 1024), -32766, 32766));
}
static int16_t maxAccX = 0, maxAccY = 0, maxAccZ = 0, minAccZ = 0;
static int16_t maxGyroX = 0, maxGyroY = 0, maxGyroZ = 0;
// oXs lat/lon: degrees * 10^7. SRXL2/X-Bus: packed BCD DDMM.mmmm.
static uint32_t bcdCoordinate(int32_t input, bool longitude, uint8_t &flags) {
    int64_t absolute = input < 0 ? -int64_t(input) : int64_t(input);
    uint32_t degrees = static_cast<uint32_t>(absolute / 10000000);
    const int64_t fraction = absolute % 10000000;
    uint32_t m10000 = static_cast<uint32_t>((fraction * 600000LL + 5000000LL) / 10000000LL);
    if (m10000 == 600000) { m10000 = 0; ++degrees; }
    if (longitude && degrees >= 100) { flags |= GPS_INFO_FLAGS_LONGITUDE_GREATER_99; degrees -= 100; }
    if (longitude) { if (input >= 0) flags |= GPS_INFO_FLAGS_IS_EAST; }
    else if (input >= 0) flags |= GPS_INFO_FLAGS_IS_NORTH;
    return (bcd(degrees * 100 + m10000 / 10000) << 16) | bcd(m10000 % 10000);
}

// Don't perform floats, printf or field formatting in the I2C ISR.
static void request(uint8_t addr) {
    const int8_t idx = indexForAddress(addr);
    if (idx < 0) return;
    ++requests[idx];
    memcpy(tx, published[idx], kPacketLength);
    memset(tx + kPacketLength, 0xff, kTxLength - kPacketLength);
    i2c_multi_set_write_buffer(tx);
}

static void publish(uint8_t idx, const uint8_t *packet) {
    // callback and handleXbus run on core0; keep the snapshot atomic to IRQ.
    const uint32_t state = save_and_disable_interrupts();
    memcpy(published[idx], packet, kPacketLength);
    restore_interrupts(state);
}

template <typename T> static void publishStruct(uint8_t idx, const T &s) {
    static_assert(sizeof(T) <= kPacketLength, "SRXL2 payload larger than X-Bus frame");
    uint8_t packet[kPacketLength];
    memset(packet, 0xff, sizeof(packet));
    memcpy(packet, &s, sizeof(s));
    publish(idx, packet);
}

static void formatAll() {
    // Payload formats / IDs come from the existing oXs SRXL2 source.
    STRU_TELE_SPEED air{};
    air.identifier = TELE_DEVICE_AIRSPEED;
    air.sID = 0;
    air.airspeed = has(AIRSPEED) ? be16(clamped(roundDiv(int64_t(val(AIRSPEED)) * 36, 1000), 0, 65534)) : 0xffff;
    air.maxAirspeed = 0xffff;
    publishStruct(AIR, air);

    STRU_TELE_G_METER gm{};
    gm.identifier = TELE_DEVICE_GMETER;          // 0x14
    gm.sID = 0;
    gm.GForceX = gm.GForceY = gm.GForceZ = static_cast<int16_t>(0x7fff);
    gm.maxGForceX = gm.maxGForceY = gm.maxGForceZ = gm.minGForceZ = static_cast<int16_t>(0x7fff);
    if (has(ACC_X)) {
        const int16_t a = xbusAcc(val(ACC_X));
        gm.GForceX = xbusSigned16(a);
        if (absolute16(a) > maxAccX) maxAccX = absolute16(a);
        gm.maxGForceX = xbusSigned16(maxAccX);
    }
    if (has(ACC_Y)) {
        const int16_t a = xbusAcc(val(ACC_Y));
        gm.GForceY = xbusSigned16(a);
        if (absolute16(a) > maxAccY) maxAccY = absolute16(a);
        gm.maxGForceY = xbusSigned16(maxAccY);
    }
    if (has(ACC_Z)) {
        const int16_t a = xbusAcc(val(ACC_Z));
        gm.GForceZ = xbusSigned16(a);
        if (a > maxAccZ) maxAccZ = a;
        if (a < minAccZ) minAccZ = a;
        gm.maxGForceZ = xbusSigned16(maxAccZ);
        gm.minGForceZ = xbusSigned16(minAccZ);
    }
    publishStruct(GMETER, gm);

    STRU_TELE_GPS_LOC loc{};
    loc.identifier = TELE_DEVICE_GPS_LOC;
    loc.sID = 0;
    uint8_t gpsFlags = 0;
    if (has(LATITUDE)) loc.latitude = bcdCoordinate(val(LATITUDE), false, gpsFlags);
    if (has(LONGITUDE)) loc.longitude = bcdCoordinate(val(LONGITUDE), true, gpsFlags);
    uint32_t altitudeDecimeters = 0;
    if (has(ALTITUDE)) {
        const int64_t cm = val(ALTITUDE);
        if (cm < 0) gpsFlags |= GPS_INFO_FLAGS_NEGATIVE_ALT;
        altitudeDecimeters = static_cast<uint32_t>((cm < 0 ? -cm : cm) / 10);
        loc.altitudeLow = static_cast<uint16_t>(bcd(altitudeDecimeters % 10000));
    }
    if (has(HEADING)) loc.course = static_cast<uint16_t>(bcd(clamped(roundDiv(val(HEADING), 10), 0, 3599)));
    if (has(GPS_PDOP)) loc.HDOP = static_cast<uint8_t>(bcd(clamped(roundDiv(val(GPS_PDOP), 10), 0, 99)));
    if (has(NUMSAT) && val(NUMSAT) > 0) {
        gpsFlags |= GPS_INFO_FLAGS_GPS_DATA_RECEIVED;
        if (val(NUMSAT) >= 4) gpsFlags |= GPS_INFO_FLAGS_GPS_FIX_VALID | GPS_INFO_FLAGS_3D_FIX;
    }
    loc.GPSflags = gpsFlags;
    publishStruct(GPS_LOC, loc);

    STRU_TELE_GPS_STAT stat{};
    stat.identifier = TELE_DEVICE_GPS_STATS;
    stat.sID = 0;
    // speed: cm/s -> 0.1 knots (0.1943844 units per cm/s).
    if (has(GROUNDSPEED)) stat.speed = static_cast<uint16_t>(bcd(clamped(roundDiv(int64_t(val(GROUNDSPEED)) * 194384LL, 1000000), 0, 9999)));
    if (has(GPS_TIME)) {
        const uint32_t t = static_cast<uint32_t>(val(GPS_TIME));
        const uint32_t hh = bcd((t >> 24) & 0xff);
        const uint32_t mm = bcd((t >> 16) & 0xff);
        const uint32_t ss = bcd((t >> 8) & 0xff);
        stat.UTC = (hh << 24) | (mm << 16) | (ss << 8);
    }
    if (has(NUMSAT)) stat.numSats = static_cast<uint8_t>(bcd(clamped(val(NUMSAT) % 100, 0, 99)));
    stat.altitudeHigh = static_cast<uint8_t>(bcd(clamped(altitudeDecimeters / 10000, 0, 99)));
    stat.notUsed1 = stat.notUsed2 = stat.notUsed3 = 0xffff;
    publishStruct(GPS_STAT, stat);

    STRU_TELE_RX_MAH energy{};
    energy.identifier = TELE_DEVICE_RX_MAH;
    energy.sID = 0;
    energy.current_A = has(CURRENT) ? static_cast<int16_t>(be16(clamped(roundDiv(val(CURRENT), 10), -32766, 32766))) : static_cast<int16_t>(0x7fff);
    // This field is in tenths of mAh, nominally limited to 3276.6mAh.
    // FVP's 34321 mAh is saturated, NOT wrapped (highCharge extension TODO).
    energy.chargeUsed_A = has(CAPACITY) ? be16(clamped(int64_t(val(CAPACITY)) * 10, 0, 32766)) : 0xffff;
    energy.volts_A = has(MVOLT) ? be16(clamped(roundDiv(val(MVOLT), 10), 0, 65534)) : 0xffff;
    energy.current_B = static_cast<int16_t>(0x7fff);
    energy.chargeUsed_B = 0xffff;
    energy.volts_B = 0xffff;
    energy.alerts = energy.highCharge = 0;
    publishStruct(ENERGY, energy);

    STRU_TELE_GYRO gyro{};
    gyro.identifier = TELE_DEVICE_GYRO;          // 0x1A
    gyro.sID = 0;
    gyro.gyroX = gyro.gyroY = gyro.gyroZ = static_cast<int16_t>(0x7fff);
    gyro.maxGyroX = gyro.maxGyroY = gyro.maxGyroZ = static_cast<int16_t>(0x7fff);
    if (gyroIsInstalled) {
        const int16_t gx = xbusGyro(gyroX);
        const int16_t gy = xbusGyro(gyroY);
        const int16_t gz = xbusGyro(gyroZ);
        gyro.gyroX = xbusSigned16(gx);
        gyro.gyroY = xbusSigned16(gy);
        gyro.gyroZ = xbusSigned16(gz);
        if (absolute16(gx) > maxGyroX) maxGyroX = absolute16(gx);
        if (absolute16(gy) > maxGyroY) maxGyroY = absolute16(gy);
        if (absolute16(gz) > maxGyroZ) maxGyroZ = absolute16(gz);
        gyro.maxGyroX = xbusSigned16(maxGyroX);
        gyro.maxGyroY = xbusSigned16(maxGyroY);
        gyro.maxGyroZ = xbusSigned16(maxGyroZ);
    }
    publishStruct(GYRO, gyro);

    STRU_TELE_ESC esc{};
    esc.identifier = TELE_DEVICE_ESC;
    esc.sID = 0;
    esc.RPM = has(RPM) && val(RPM) > 0 ? be16(clamped(roundDiv(int64_t(val(RPM)) * 60, 10), 0, 65534)) : 0xffff;
    esc.voltsInput = has(MVOLT) ? be16(clamped(roundDiv(val(MVOLT), 10), 0, 65534)) : 0xffff;
    esc.tempFET = has(TEMP1) ? be16(clamped(int64_t(val(TEMP1)) * 10, 0, 65534)) : 0xffff;
    esc.currentMotor = has(CURRENT) ? be16(clamped(roundDiv(val(CURRENT), 10), 0, 65534)) : 0xffff;
    esc.tempBEC = has(TEMP2) ? be16(clamped(int64_t(val(TEMP2)) * 10, 0, 65534)) : 0xffff;
    esc.currentBEC = esc.voltsBEC = esc.throttle = esc.powerOut = 0xff;
    publishStruct(ESC, esc);

    STRU_TELE_VARIO_S vario{};
    vario.identifier = TELE_DEVICE_VARIO_S;
    vario.sID = 0;
    vario.altitude = has(RELATIVEALT) ? static_cast<int16_t>(be16(clamped(roundDiv(val(RELATIVEALT), 10), -32766, 32766))) : static_cast<int16_t>(0x7fff);
    // In the 0.1 m/s delta fields, use the current vertical-speed estimate
    // only in the 1s slot; historic 250/500/1500/... deltas are not known.
    vario.delta_0250ms = vario.delta_0500ms = static_cast<int16_t>(0x7fff);
    vario.delta_1000ms = has(VSPEED) ? static_cast<int16_t>(be16(clamped(roundDiv(val(VSPEED), 10), -32766, 32766))) : static_cast<int16_t>(0x7fff);
    vario.delta_1500ms = vario.delta_2000ms = vario.delta_3000ms = static_cast<int16_t>(0x7fff);
    publishStruct(VARIO_IDX, vario);

    STRU_TELE_RPM rpm{};
    rpm.identifier = TELE_DEVICE_RPM;
    rpm.sID = 0;
    rpm.microseconds = has(RPM) && val(RPM) > 0 ? be16(clamped(roundDiv(1000000LL, val(RPM)), 1, 65534)) : 0xffff;
    rpm.volts = has(MVOLT) ? be16(clamped(roundDiv(val(MVOLT), 10), 0, 65534)) : 0xffff;
    rpm.temperature = has(TEMP1) ? static_cast<int16_t>(be16(clamped(roundDiv(int64_t(val(TEMP1)) * 9, 5) + 32, -32766, 32766))) : static_cast<int16_t>(0x7fff);
    rpm.dBm_A = rpm.dBm_B = 0;
    rpm.spare[0] = rpm.spare[1] = 0xffff;
    publishStruct(RPM_TEMP, rpm);

    // ADS1115 #1: user-defined X-Bus frame 0x50.
    // First four 16-bit values are ADS_1_1 .. ADS_1_4 in oXs millivolts.
    // EdgeTX's unrecognized-Spektrum fallback reads payload offsets 0,2,4,6
    // as RAW sensor IDs 0x5000, 0x5002, 0x5004, 0x5006.
    // NOTE: actual discovery depends on the receiver polling address 0x50.
    STRU_TELE_USER_16SU ads1{};
    ads1.identifier = TELE_DEVICE_USER_16SU;
    ads1.sID = 0;
    ads1.sField1 = has(ADS_1_1) ? xbusSigned16(clamped(val(ADS_1_1), 0, 32766)) : static_cast<int16_t>(0xffff);
    ads1.sField2 = has(ADS_1_2) ? xbusSigned16(clamped(val(ADS_1_2), 0, 32766)) : static_cast<int16_t>(0xffff);
    ads1.sField3 = has(ADS_1_3) ? xbusSigned16(clamped(val(ADS_1_3), 0, 32766)) : static_cast<int16_t>(0xffff);
    ads1.uField1 = has(ADS_1_4) ? be16(clamped(val(ADS_1_4), 0, 65534)) : 0xffff;
    ads1.uField2 = ads1.uField3 = ads1.uField4 = 0xffff; // unused
    publishStruct(ADS1, ads1);
}
} // namespace

void setupXbusSpektrum() {
    if (config.protocol != 'X' || config.pinTlm == 255 || config.pinPrimIn == 255) return;
    if (config.pinTlm + 1 != config.pinPrimIn || config.pinTlm > 28) {
        printf("[XBUS] Expected TLM=SDA and PRI=SCL=TLM+1; abort.\n");
        return;
    }
    // Proof-of-concept uses all 4 SMs/IRQs on PIO0. Do not collide with
    // SBUS-out (SM2), ESC PIO reception (SM3) or PIO0 PWM/other users.
    if (config.pinSbusOut != 255 || config.pinEsc != 255) {
        printf("[XBUS] PIO0 conflict: disable SBUS_OUT and ESC for this test.\n");
        return;
    }
    for (uint8_t i = 0; i < kSensorCount; ++i) {
        memset(published[i], 0xff, kPacketLength);
        published[i][0] = ids[i];
        published[i][1] = 0;
    }
    memset(tx, 0xff, sizeof(tx));
    formatAll();
    i2c_multi_init(pio0, config.pinTlm);
    // PIO owns these pins: keep GPIO_FUNC_PIO0 set by i2c_multi_init().
    // RP2040 internal pull-ups (~50-80 kohm) only supplement external pull-ups.
    gpio_pull_up(config.pinTlm);       // SDA
    gpio_pull_up(config.pinPrimIn);    // SCL
    i2c_multi_set_request_handler(request);
    for (uint8_t i = 0; i < kSensorCount; ++i) i2c_multi_enable_address(ids[i]);
    ready = true;
    printf("[XBUS] PIO0 SDA=GP%u SCL=GP%u, %u addresses enabled\n", config.pinTlm, config.pinPrimIn, kSensorCount);
}

void handleXbusSpektrum() {
    if (!ready) return;
    const uint32_t now = millisRp();
    if (now - lastUpdate >= kRefreshMs) {
        lastUpdate = now;
        formatAll();
    }
    if (debugTlm == 'Y' && now - lastDebug >= 2000) {
        lastDebug = now;
        printf("[XBUS] req: 11=%lu 14=%lu 16=%lu 17=%lu 18=%lu 1A=%lu 20=%lu 40=%lu 7E=%lu 50=%lu\n",
               (unsigned long)requests[AIR], (unsigned long)requests[GMETER],
               (unsigned long)requests[GPS_LOC], (unsigned long)requests[GPS_STAT],
               (unsigned long)requests[ENERGY], (unsigned long)requests[GYRO],
               (unsigned long)requests[ESC], (unsigned long)requests[VARIO_IDX],
               (unsigned long)requests[RPM_TEMP], (unsigned long)requests[ADS1]);
        printf("[XBUS ADS1] 1=%ld 2=%ld 3=%ld 4=%ld mV (50 requests=%lu)\n",
               long(has(ADS_1_1) ? val(ADS_1_1) : -1),
               long(has(ADS_1_2) ? val(ADS_1_2) : -1),
               long(has(ADS_1_3) ? val(ADS_1_3) : -1),
               long(has(ADS_1_4) ? val(ADS_1_4) : -1),
               (unsigned long)requests[ADS1]);
        // Diagnostic only: core0 MPU values before/after X-Bus scaling.
        // Zero raw rates with gyroIsInstalled=1 can indicate that the gyro
        // mixer is disabled: mpu.cpp does not send raw rates in that mode.
        printf("[XBUS MPU] installed=%u gyroChan=%d raw=%d,%d,%d XBUS_0.1dps=%d,%d,%d ACC_mg=%ld,%ld,%ld\n",
               unsigned(gyroIsInstalled), int(config.gyroChanControl),
               int(gyroX), int(gyroY), int(gyroZ),
               int(xbusGyro(gyroX)), int(xbusGyro(gyroY)), int(xbusGyro(gyroZ)),
               long(has(ACC_X) ? val(ACC_X) : 0),
               long(has(ACC_Y) ? val(ACC_Y) : 0),
               long(has(ACC_Z) ? val(ACC_Z) : 0));
    }
}
