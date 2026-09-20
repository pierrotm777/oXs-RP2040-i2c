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

namespace {
constexpr uint8_t kPacketLength = 16;
constexpr uint8_t kTxLength = 32;  // safe padding if a master requests extra bytes
constexpr uint32_t kRefreshMs = 100;

// MSRC i2c_multi consumes the buffer *after* the request callback returns.
// A static transmit buffer is mandatory (MSRC's uint8_t buffer[16] on the
// callback stack would have an invalid lifetime).
uint8_t tx[kTxLength] = {};
uint8_t published[7][kPacketLength] = {};
uint32_t requests[7] = {};
uint32_t lastUpdate = 0;
uint32_t lastDebug = 0;
bool ready = false;

enum SensorIdx : uint8_t { AIR, GPS_LOC, GPS_STAT, ENERGY, ESC, VARIO_IDX, RPM_TEMP };
constexpr uint8_t ids[7] = {
    TELE_DEVICE_AIRSPEED, TELE_DEVICE_GPS_LOC, TELE_DEVICE_GPS_STATS,
    TELE_DEVICE_RX_MAH, TELE_DEVICE_ESC, TELE_DEVICE_VARIO_S,
    TELE_DEVICE_RPM
};

static int8_t indexForAddress(uint8_t addr) {
    for (uint8_t i = 0; i < 7; ++i) if (ids[i] == addr) return static_cast<int8_t>(i);
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
    for (uint8_t i = 0; i < 7; ++i) {
        memset(published[i], 0xff, kPacketLength);
        published[i][0] = ids[i];
        published[i][1] = 0;
    }
    memset(tx, 0xff, sizeof(tx));
    formatAll();
    i2c_multi_init(pio0, config.pinTlm);
    i2c_multi_set_request_handler(request);
    for (uint8_t i = 0; i < 7; ++i) i2c_multi_enable_address(ids[i]);
    ready = true;
    printf("[XBUS] PIO0 SDA=GP%u SCL=GP%u, 7 addresses enabled\n", config.pinTlm, config.pinPrimIn);
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
        printf("[XBUS] req: 11=%lu 16=%lu 17=%lu 18=%lu 20=%lu 40=%lu 7E=%lu\n",
               (unsigned long)requests[AIR], (unsigned long)requests[GPS_LOC],
               (unsigned long)requests[GPS_STAT], (unsigned long)requests[ENERGY],
               (unsigned long)requests[ESC], (unsigned long)requests[VARIO_IDX],
               (unsigned long)requests[RPM_TEMP]);
    }
}
