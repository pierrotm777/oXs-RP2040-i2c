#include "hitec.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"   // snapshot the IRQ diagnostics safely
#include "i2c_slave.h"
#include "config.h"
#include "param.h"
#include "tools.h"          // fields[], millisRp(), FVP/FVN support

extern CONFIG config;
extern field fields[];
extern uint8_t debugTlm;

namespace {
constexpr uint8_t kFrameCount = 11;   // 0x11..0x1B
constexpr uint8_t kFrameSize = 7;
constexpr uint32_t kNewFrameGapUs = 1000; // consecutive Optima polls are ~45 ms apart
constexpr uint32_t kRefreshMs = 100;

// Exact steady-state order measured on the genuine HTS-SS capture.
// GPS frames 0x12/0x13/0x14 are intentionally repeated three times per cycle.
constexpr uint8_t kHtsSsSchedule[] = {
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x12, 0x13, 0x14, 0x12, 0x13, 0x14
};
constexpr uint8_t kHtsSsScheduleCount = sizeof(kHtsSsSchedule) / sizeof(kHtsSsSchedule[0]);

struct HitecFrames {
    uint8_t bytes[kFrameCount][kFrameSize];
    bool enabled[kFrameCount];
};

// Main loop only writes the INACTIVE bank; interrupt only reads ACTIVE bank.
HitecFrames banks[2] = {};
volatile uint8_t activeBank = 0;
volatile uint8_t frameIndex = kFrameCount - 1;
volatile uint8_t scheduleIndex = kHtsSsScheduleCount - 1;
// Number of bytes already placed in the TX FIFO for the current read:
// seven HITEC bytes + one 0xFF padding byte for the Optima's extra clock.
volatile uint8_t sentBytes = 0;
volatile uint32_t lastFrameAtUs = 0;
volatile uint32_t requestCount = 0;
volatile uint32_t frameCount = 0;
// Diagnostics remain available through DEBUGTLM=Y/N; no printf in the IRQ.
volatile uint32_t partialCount = 0;
volatile uint32_t finishCount = 0;
volatile uint32_t paddingMissed = 0;
volatile uint32_t fifoShortCount = 0;
volatile uint32_t lateNewFrame = 0;
uint8_t lastFrame[kFrameSize] = {};
bool lastFrameValid = false;
uint8_t currentPacket[kFrameSize] = {};
uint32_t previousRefreshMs = 0;
uint32_t previousDebugMs = 0;
bool initialized = false;

static inline void putBE(uint8_t *out, int32_t v) {
    const uint16_t raw = static_cast<uint16_t>(v);
    out[0] = uint8_t(raw >> 8);
    out[1] = uint8_t(raw);
}
static inline void putLE(uint8_t *out, int32_t v) {
    const uint16_t raw = static_cast<uint16_t>(v);
    out[0] = uint8_t(raw);
    out[1] = uint8_t(raw >> 8);
}
static inline int32_t clamp(int64_t v, int32_t low, int32_t high) {
    if (v < low) return low;
    if (v > high) return high;
    return static_cast<int32_t>(v);
}
static inline bool has(fieldIdx id) { return fields[id].available; }
static inline int32_t val(fieldIdx id) { return fields[id].value; }

// HITEC 0x19: four independent one-byte servo-current fields (EdgeTX: 0.1 A).
// oXs ADS measurements are in mV. Send one raw step per 100 mV:
// 0 mV -> 0 (0.0 on the radio), 3300 mV -> 33 (3.3 on the radio).
// This is only telemetry formatting: ADC acquisition and fields[] stay unchanged.
// HITEC has no signed representation for these four values; clamp negatives to 0.
static inline uint8_t hitecAdsByte(fieldIdx id) {
    const int64_t mv = int64_t(val(id));
    return uint8_t(clamp((mv >= 0 ? mv + 50 : mv - 50) / 100, 0, 255));
}

// MSRC style: signed DDMM and signed SS.ss. Input is signed degrees x 10^7.
// Integer arithmetic avoids losing GPS precision with RP2040's 32-bit float.
static void formatCoord(uint8_t *frame, int32_t coordE7) {
    const bool negative = coordE7 < 0;
    const int64_t absolute = negative ? -int64_t(coordE7) : int64_t(coordE7);
    int32_t degrees = int32_t(absolute / 10000000LL);
    const int64_t fractional = absolute % 10000000LL;
    int32_t minutes = int32_t((fractional * 60LL) / 10000000LL);
    const int64_t remaining = fractional * 60LL - int64_t(minutes) * 10000000LL;
    int32_t centiseconds = int32_t((remaining * 6000LL + 5000000LL) / 10000000LL);
    if (centiseconds >= 6000) { centiseconds -= 6000; ++minutes; }
    if (minutes >= 60) { minutes -= 60; ++degrees; }
    const int32_t degMin = degrees * 100 + minutes;
    putBE(&frame[1], negative ? -centiseconds : centiseconds);
    putBE(&frame[3], negative ? -degMin : degMin);
}

static bool chooseNextPacket() {
    const HitecFrames &source = banks[activeBank];
#if HITEC_TEST_VALUES && HITEC_EXACT_SCHEDULE
    for (uint8_t i = 0; i < kHtsSsScheduleCount; ++i) {
        const uint8_t pos = (scheduleIndex + 1 + i) % kHtsSsScheduleCount;
        const uint8_t id = kHtsSsSchedule[pos];
        const uint8_t idx = id - 0x11;
        if (source.enabled[idx]) {
            scheduleIndex = pos;
            frameIndex = idx;
            memcpy(currentPacket, source.bytes[idx], kFrameSize);
            return true;
        }
    }
#else
    for (uint8_t i = 0; i < kFrameCount; ++i) {
        const uint8_t next = (frameIndex + 1 + i) % kFrameCount;
        if (source.enabled[next]) {
            frameIndex = next;
            memcpy(currentPacket, source.bytes[next], kFrameSize);
            return true;
        }
    }
#endif
    return false;
}

// Called from i2c_slave.cpp IRQ: no printf, malloc, delay, nor blocking read.
// Preload seven HITEC data bytes + 0xFF on the first RD_REQ.
// The padding byte lets the Optima finish its extra clock without stretching SCL.
// Preserve this timing: it was validated with external 1 kOhm pull-ups on SDA/SCL.
static void hitecIrq(i2c_inst_t *i2c, i2c_slave_event_t event) {
    const uint32_t irqStart = time_us_32();
    switch (event) {
    case I2C_SLAVE_RECEIVE:
        while (i2c_get_read_available(i2c)) (void)i2c_read_byte_raw(i2c);
        break;
    case I2C_SLAVE_REQUEST: {
        ++requestCount;
        // If the SDK never reported STOP/START, recognize a fresh poll by
        // its time gap. This check happens only once per unexpected RD_REQ.
        if (sentBytes == kFrameSize + 1 &&
            uint32_t(irqStart - lastFrameAtUs) > kNewFrameGapUs) {
            sentBytes = 0;
            ++lateNewFrame;
        }
        if (sentBytes == 0 && !chooseNextPacket()) {
            if (i2c_get_write_available(i2c)) {
                i2c_get_hw(i2c)->data_cmd = 0xffu;
            }
            break;
        }
        // Never block in the IRQ. Writing DATA_CMD directly after checking
        // free slots avoids repeated calls to i2c_write_raw_blocking().
        const uint freeSlots = i2c_get_write_available(i2c);
        if (freeSlots < (kFrameSize + 1 - sentBytes)) ++fifoShortCount;
        for (uint n = 0; n < freeSlots && sentBytes < kFrameSize + 1; ++n) {
            if (sentBytes < kFrameSize) {
                i2c_get_hw(i2c)->data_cmd = currentPacket[sentBytes];
                ++sentBytes;
                if (sentBytes == kFrameSize) {
                    ++frameCount;  // seven bytes enqueued, NOT verified by Optima
                    memcpy(lastFrame, currentPacket, kFrameSize);
                    lastFrameValid = true;
                    lastFrameAtUs = time_us_32();
                }
            } else {
                // Allow the Optima's extra (73rd) SCL pulse immediately.
                i2c_get_hw(i2c)->data_cmd = 0xffu;
                ++sentBytes;
            }
        }
        if (sentBytes == kFrameSize) ++paddingMissed;
        // Do not reset or re-arm the peripheral during a valid transaction.
        break;
    }
    case I2C_SLAVE_FINISH:
        ++finishCount;
        if (sentBytes > 0 && sentBytes < kFrameSize) ++partialCount;
        sentBytes = 0;
        break;
    }
}

static void enable(HitecFrames &f, uint8_t id) { f.enabled[id - 0x11] = true; }
static uint8_t *frame(HitecFrames &f, uint8_t id) { return f.bytes[id - 0x11]; }

static void refreshPackets() {
    const uint8_t staging = activeBank ^ 1u;
    HitecFrames &b = banks[staging];
    memset(&b, 0, sizeof b);
    for (uint8_t i = 0; i < kFrameCount; ++i) {
        b.bytes[i][0] = uint8_t(i + 0x11);
        b.bytes[i][6] = uint8_t(i + 0x11);
    }

    // HTS-SS 0x11 identification/status frame, seen in the genuine capture
    // and present in MSRC even when no RX battery sensor is available.
    enable(b, 0x11);
    frame(b, 0x11)[1] = 0xAF;
    frame(b, 0x11)[3] = 0x2D;

#if HITEC_TEST_VALUES
    // Fixed values for an independent HITEC bus / EdgeTX test.
    enable(b, 0x12); formatCoord(frame(b, 0x12), 488566000L);   // Paris latitude
    frame(b, 0x12)[5] = 31;                                      // GPS seconds: 12:30:31
    enable(b, 0x13); formatCoord(frame(b, 0x13), 23522000L);    // Paris longitude
    frame(b, 0x13)[5] = 40 + 40;                                // TEMP2 = 40 C
    enable(b, 0x14); putBE(&frame(b, 0x14)[1], 36);             // 36 km/h
    putBE(&frame(b, 0x14)[3], 125);                             // 125 m
    frame(b, 0x14)[5] = 40 + 35;                                // TEMP1 = 35 C
    enable(b, 0x15); putLE(&frame(b, 0x15)[2], 1500);           // RPM1
    putLE(&frame(b, 0x15)[4], 2500);                            // RPM2
    // EdgeTX HITEC decoder expects YY/MM/DD in bytes 1/2/3.
    enable(b, 0x16); frame(b, 0x16)[1] = 26;                    // year 2026
    frame(b, 0x16)[2] = 9; frame(b, 0x16)[3] = 16;              // month, day
    frame(b, 0x16)[4] = 12; frame(b, 0x16)[5] = 30;             // hour, minute
    enable(b, 0x17); putBE(&frame(b, 0x17)[1], 90);             // heading
    frame(b, 0x17)[3] = 10;                                    // 10 sats
    enable(b, 0x18); putLE(&frame(b, 0x18)[1], 123);            // (12.5-0.2)*10
    putLE(&frame(b, 0x18)[3], 3);                               // current A (MSRC/EdgeTX)
    // The test profile reproduces the verified HTS-SS 0x11..0x18 cycle.
#else
    // oXs fields[] use signed E7 GPS, cm/s, cm, mV, mA and RPM in Hz.
    // EdgeTX reads GPS seconds from the fifth data byte of frame 0x12,
    // then combines them with the hours/minutes from frame 0x16.
    if (has(LATITUDE) || has(GPS_TIME)) {
        enable(b, 0x12);
        if (has(LATITUDE)) formatCoord(frame(b, 0x12), val(LATITUDE));
        if (has(GPS_TIME)) {
            const uint32_t v = uint32_t(val(GPS_TIME)); // oXs: 0xHHMMSS00
            frame(b, 0x12)[5] = uint8_t((v >> 8) & 0xff); // SS
        }
    }
    if (has(LONGITUDE) || has(TEMP2)) {
        enable(b, 0x13);
        if (has(LONGITUDE)) formatCoord(frame(b, 0x13), val(LONGITUDE));
        if (has(TEMP2)) frame(b, 0x13)[5] = uint8_t(clamp(int64_t(val(TEMP2)) + 40, 0, 255));
    }
    if (has(GROUNDSPEED) || has(ALTITUDE) || has(TEMP1)) {
        enable(b, 0x14);
        if (has(GROUNDSPEED)) putBE(&frame(b, 0x14)[1], clamp((int64_t(val(GROUNDSPEED)) * 36 + 500) / 1000, 0, 65535));
        if (has(ALTITUDE)) putBE(&frame(b, 0x14)[3], clamp(val(ALTITUDE) / 100, -32768, 32767));
        if (has(TEMP1)) frame(b, 0x14)[5] = uint8_t(clamp(int64_t(val(TEMP1)) + 40, 0, 255));
    }
    if (has(RPM)) {
        enable(b, 0x15);
        putLE(&frame(b, 0x15)[2], clamp(int64_t(val(RPM)) * 60, 0, 65535));
    }
    if (has(GPS_DATE) || has(GPS_TIME)) {
        enable(b, 0x16);
        if (has(GPS_DATE)) {
            const uint32_t v = uint32_t(val(GPS_DATE));
            // oXs GPS_DATE = 0xYYMMDDFF; EdgeTX HITEC date bytes = YY/MM/DD.
            frame(b, 0x16)[1] = uint8_t((v >> 24) & 0xff);  // YY
            frame(b, 0x16)[2] = uint8_t((v >> 16) & 0xff);  // MM
            frame(b, 0x16)[3] = uint8_t((v >> 8) & 0xff);   // DD
        }
        if (has(GPS_TIME)) {
            const uint32_t v = uint32_t(val(GPS_TIME));
            frame(b, 0x16)[4] = uint8_t((v >> 24) & 0xff);  // HH
            frame(b, 0x16)[5] = uint8_t((v >> 16) & 0xff);  // MM
        }
    }
    if (has(HEADING) || has(NUMSAT)) {
        enable(b, 0x17);
        if (has(HEADING)) putBE(&frame(b, 0x17)[1], clamp(val(HEADING) / 100, 0, 359));
        if (has(NUMSAT)) frame(b, 0x17)[3] = uint8_t(clamp(val(NUMSAT), 0, 255));
    }
    if (has(MVOLT) || has(CURRENT)) {
        enable(b, 0x18);
        // MSRC: V1 displayed from raw = (volts - 0.2) * 10.
        if (has(MVOLT)) putLE(&frame(b, 0x18)[1], clamp((int64_t(val(MVOLT)) - 200 + 50) / 100, 0, 65535));
        if (has(CURRENT)) putLE(&frame(b, 0x18)[3], clamp((int64_t(val(CURRENT)) + 500) / 1000, 0, 65535));
    }
    // One ADS1115 = four independent EdgeTX telemetry sensors:
    // 0x1900 / 0x1901 / 0x1902 / 0x1903 (servo currents 1..4).
    // Payload bytes 1..4, NOT byte 6 (reserved for the trailing frame ID).
    // Enable 0x19 only when at least one ADS1 measurement is available.
    if (has(ADS_1_1) || has(ADS_1_2) || has(ADS_1_3) || has(ADS_1_4)) {
        enable(b, 0x19);
        if (has(ADS_1_1)) frame(b, 0x19)[1] = hitecAdsByte(ADS_1_1);
        if (has(ADS_1_2)) frame(b, 0x19)[2] = hitecAdsByte(ADS_1_2);
        if (has(ADS_1_3)) frame(b, 0x19)[3] = hitecAdsByte(ADS_1_3);
        if (has(ADS_1_4)) frame(b, 0x19)[4] = hitecAdsByte(ADS_1_4);
    }
    if (has(AIRSPEED)) {
        enable(b, 0x1A);
        putBE(&frame(b, 0x1A)[3], clamp((int64_t(val(AIRSPEED)) * 36 + 500) / 1000, 0, 65535));
    }
    if (has(RELATIVEALT)) {
        enable(b, 0x1B);
        // MSRC rounds barometric altitude to whole metres (0x1B ALTU).
        // oXs RELATIVEALT is in cm; round symmetrically, including negatives.
        // VSPEED has no defined field in this 7-byte frame (MSRC: ALTU/ALTF).
        const int64_t altitudeCm = val(RELATIVEALT);
        const int64_t altitudeM = altitudeCm >= 0
            ? (altitudeCm + 50) / 100
            : (altitudeCm - 50) / 100;
        putBE(&frame(b, 0x1B)[1], clamp(altitudeM, -32768, 32767));
    }
#endif
    // Publishing a single-byte bank index is atomic on RP2040.
    __asm volatile("" ::: "memory");
    activeBank = staging;
}
}  // namespace

void setupHitec() {
    if (config.protocol != 'T' || config.pinTlm == 255 || config.pinPrimIn == 255) return;
    if (config.pinTlm != 8 || config.pinPrimIn != 9) {
        printf("HITEC: this PCB requires TLM=8 (SDA0), PRI=9 (SCL0)\n");
        return;
    }
    refreshPackets();
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(config.pinTlm, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinTlm);
    gpio_set_function(config.pinPrimIn, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinPrimIn);
    i2c_slave_init(i2c0, HITEC_I2C_ADDRESS, hitecIrq);
    initialized = true;
    previousRefreshMs = millisRp();
    printf("HITEC: I2C0 slave 0x08, SDA GP8, SCL GP9, test=%d, FIFO=7+FF\n",
           HITEC_TEST_VALUES);
}

void handleHitec() {
    if (!initialized || config.protocol != 'T') return;
    const uint32_t ms = millisRp();
    if (ms - previousRefreshMs >= kRefreshMs) {
        previousRefreshMs = ms;
        refreshPackets();
    }
    // DEBUGTLM=Y/N: concise, non-blocking I2C diagnostics.
    if (debugTlm == 'Y' && ms - previousDebugMs >= 2000) {
        previousDebugMs = ms;
        uint32_t requests, frames, finishes, partial, padMisses, shortFifo, lateStarts;
        uint8_t last[kFrameSize];
        bool haveLast;
        const uint32_t irqState = save_and_disable_interrupts();
        requests = requestCount;
        frames = frameCount;
        finishes = finishCount;
        partial = partialCount;
        padMisses = paddingMissed;
        shortFifo = fifoShortCount;
        lateStarts = lateNewFrame;
        haveLast = lastFrameValid;
        memcpy(last, lastFrame, kFrameSize);
        restore_interrupts(irqState);

        printf("[HITEC] requests=%lu frames_queued=%lu finish=%lu partial=%lu\n",
               (unsigned long)requests, (unsigned long)frames,
               (unsigned long)finishes, (unsigned long)partial);
        printf("[HITEC] pad_missed=%lu fifo_short=%lu late_start=%lu\n",
               (unsigned long)padMisses, (unsigned long)shortFifo,
               (unsigned long)lateStarts);
        if (haveLast) {
            printf("[HITEC] last_frame=%02X %02X %02X %02X %02X %02X %02X\n",
                   (unsigned int)last[0], (unsigned int)last[1],
                   (unsigned int)last[2], (unsigned int)last[3],
                   (unsigned int)last[4], (unsigned int)last[5],
                   (unsigned int)last[6]);
        }
    }
}
