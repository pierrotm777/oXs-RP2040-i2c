# README_ModI2C_EN.md — I²C adaptations for oXs / RP2040-Zero

**Report on RadioLink → HITEC → Spektrum X-Bus work**  
**Project:** oXs on RP2040-Zero, Pico SDK 2.1.0, telemetry to TX16S / EdgeTX.  
**Status as of September 20, 2026:** RadioLink and HITEC have undergone fixes and testing; Spektrum X-Bus works with the AR6610T, and simultaneous startup with the RP2040 now appears reproducible.

> **Scope of this document.** It describes the fixes, additions, affected source files, and test results. It is **not** a guaranteed diff of the final project folder on the PC: check that the latest validated files include all the changes listed here. Abandoned experiments are distinguished from the retained solution.

## 1. RadioLink — `PROTOCOL=R`

### 1.1. Initial adaptation as an RP2040 I²C slave

- Creation/adaptation of the RadioLink transport around `src/rlink.cpp`, `src/rlink.h`, and the existing Pico SDK driver `src/i2c_slave.cpp/.h`; integration into `main.cpp` through `setupRlink()` and `handleRlink()`.
- Use of **I²C0 in slave mode, 7-bit address `0x04`**. Configuration mapping: **`TLM = SDA0`**, **`PRI = SCL0`**, with `PRI = TLM + 1` on a valid pin pair. Examples: GP0/GP1 or GP8/GP9 (the latter pair is the one documented for the HITEC/X-Bus setup).
- Two **16-byte** RadioLink frames, prefixed `89 CD` and `89 AB`; data reconstructed from oXs `fields[]`. The first packet sent is **`89 CD`**, then it alternates with `89 AB`.

### 1.2. I²C lockup fix and frame consistency

**File changed:** `src/rlink.cpp`. The initial transport attempt did not guarantee a correct response to every read request. Changes made:

- **One byte written per `I2C_SLAVE_REQUEST` event** using `i2c_write_byte_raw()`, rather than trying to fill all 16 bytes in one go inside the interrupt.
- **No blocking wait in the IRQ**; data is prepared outside the interrupt in `handleRlink()`.
- A frozen copy of the 16 bytes at the start of each read: the master cannot receive a frame mixing two updates.
- `CD`/`AB` alternation **at the end of the transaction** (`I2C_SLAVE_FINISH`), not after each byte; counting of complete or partial reads.
- Draining of any incoming writes, startup port/address diagnostics, and counters with `DEBUGTLM=Y`.

The `0x00` response beyond 16 bytes is a protective measure in this test driver's implementation; the expected normal read remains **16 bytes**.

### 1.3. RadioLink GPS fixes

**File changed:** `src/rlink.cpp`. Changes made:

- Coordinate order in the `89 CD` frame: **longitude before latitude**.
- USB coordinate diagnostics: floating-point division by `10000000.0`, instead of integer division before conversion to `float`.

**Important:** check that the retained `src/rlink.cpp` contains **both** GPS fixes and the non-blocking IRQ mechanism.

### 1.4. Parameter fixes for all three I²C protocols

In `src/param.cpp`:

- Correction of help text: **PRI = SCL0** (`1, 5, 9, 13`) and **TLM = SDA0** (`0, 4, 8, 12`).
- Fix for a condition using `||` instead of `&&`: the UART pin restrictions for `PRI`/`TLM` must **not** accidentally exclude protocols `R`, `X`, `T`.
- General validation adapted for these three protocols, with a check of the I²C0 pin pair: `PRI == TLM + 1` when the pins are defined.
- Additional guard in `setupRlink()` to reject an invalid pair.

This fix concerns the **shared configuration**; it was introduced during RadioLink testing and subsequently reused for HITEC and Spektrum X-Bus.

### 1.5. Flash memory saving — shared fix originating from RadioLink testing

**File changed:** `src/param.cpp`. This change is **not specific to RadioLink**: it protects `SAVE`, the sequencers, and the gyroscopic mixer throughout oXs.

**Problem:** the historical offset `FLASH_CONFIG_OFFSET = 256 * 1024` could fall inside the firmware image, which had grown larger. Running `SAVE` there could erase/program part of the program and cause boot failure.

**New layout, for an RP2040-Zero with 2 MiB of Flash:** the **last three 4 KiB sectors** are reserved. They are consecutive and separate:

| Saved item | Offset from start of Flash | Reserved size |
|---|---:|---:|
| Configuration `config` | `0x1FD000` | 4 KiB |
| Sequencers `seq` | `0x1FE000` | 4 KiB |
| Gyroscopic mixer `gyroMixer` | `0x1FF000` | 4 KiB |

Basic code formula:

```cpp
#define OXS_FLASH_BYTES (2u * 1024u * 1024u)
#define FLASH_CONFIG_OFFSET (OXS_FLASH_BYTES - 3u * FLASH_SECTOR_SIZE)
```

**Other protections added:**

- `static_assert(sizeof(config) <= FLASH_PAGE_SIZE)`: the configuration must fit within the **256-byte** page provided for writing it.
- For `seq` and `gyroMixer`, programmed size **rounded up to a multiple of 256 bytes**, buffer filled with `0xFF`, followed by a copy of the useful data.
- Compile-time check that each block fits within its **4 KiB** sector; `flash_range_program()` receives a length that meets Flash page constraints.
- Existing precautions around writing retained: core 1 placed in a safe state, interrupts suspended during erasing/programming, then restored.

**Mandatory steps after this change:** first install the **new UF2 in BOOTSEL mode**, then run `SAVE` only with that firmware. Old saves located at `256 KiB` are **not automatically migrated** to the new addresses: reconfigure and save again if necessary. This layout assumes a **2 MiB** board and a firmware image that **does not overlap the final 12 KiB**; check again if the program grows substantially.

## 2. HITEC — `PROTOCOL=T`

### 2.1. Emulation of the complete HTS-SS set

**Files:** `src/hitec.cpp` and `src/hitec.h`, `setupHitec()` / `handleHitec()` calls in `main.cpp`.

- **Hardware I²C0, slave `0x08`, 100 kHz**, on **GP8 = SDA / TLM=8** and **GP9 = SCL / PRI=9** for our board.
- oXs **I²C1**, used for physical sensors, remains separate (for example GP10/GP11 depending on configuration); those sensor drivers are not replaced.
- Construction of **7-byte** HITEC frames and support for the relevant families from `0x11` to `0x1B`.
- Periodic frame preparation from `fields[]` (approximately **100 ms**); active/inactive double buffering so the IRQ reads a stable version.
- Two modes provided by `HITEC_TEST_VALUES`: **`1`** for fixed bench-test values, **`0`** for oXs measurements and simulated `FVP`/`FVN` values. The final distributed version has `HITEC_TEST_VALUES=0` by default.

### 2.2. Tuning transactions with the Optima 7

**File changed:** `src/hitec.cpp`. Captures from the original HTS-SS set and tests on the Optima 7 made it possible to correct several intermediate attempts:

- Addition of the **`0x11`** identification/status frame and, in exact test mode, reproduction of the captured cycle: `11,12,13,14,15,16,17,18,12,13,14,12,13,14` (hexadecimal).
- Analysis of reads showing an **extra pulse/clock** after the seven useful bytes. Deferred slave reinitialization tests and various eighth-byte responses led to retaining the validated solution: **preload seven bytes + `0xFF` into the TX FIFO as soon as the request arrives**.
- Direct writes to `DATA_CMD` **only when the FIFO has room**, with no wait, `printf`, or I²C reinitialization inside the IRQ; handling of incomplete transactions and the beginning of a new query when STOP is not reported as expected.
- Diagnostic counters available through `DEBUGTLM=Y` remain outside the IRQ; the final variant removes temporary interrupt-duration prints.
- **External 1 kΩ pull-ups to 3.3 V on both SDA and SCL**: necessary in the Optima 7 setup tested. Do not infer that they should also be used on Spektrum.

The “reset after 5 ms,” `FF` byte without reset, and other extra-pulse experiments **are not changes to combine**. The retained version is `src/hitec.cpp` with a preloaded FIFO and without temporary interrupt-timing instrumentation.

### 2.3. Data conversions and EdgeTX fixes

**File changed:** `src/hitec.cpp`.

- Signed oXs GPS coordinates in **degrees × 10⁷** converted to the HITEC format using **integer arithmetic** to avoid losing precision on the RP2040.
- Corrections to the **`YY/MM/DD` date** bytes in frame `0x16`; **hour/minute** are also sent there, while **GPS seconds** belong in `0x12`, according to EdgeTX's HITEC decoder.
- Formatting of temperatures, ground/airspeed, RPM, heading, satellites, voltage, current, and altitude according to the frames used; notably the HITEC voltage offset and current unit were checked during testing.
- For the `0x1B` barometric altimeter, **symmetric rounding of centimeters to meters**, including negative values; no invented VSPEED field in this seven-byte frame.

### 2.4. Shared FVP/FVN simulations

**File changed:** `src/tools.cpp`.

An older branch of `fillFields(1)` (`FVP`) injected only a **subset of fields**, notably GPS: the other HITEC/X-Bus sensors therefore did not always reflect the simulated values. The fix walks through **all `posFieldValues[]`**, just as `FVN` walks through its negative values; fields are marked available and reapplied throughout the simulation. In particular, this prevents physical measurements (for example V3/V4) from permanently overwriting simulated temperatures.

This change concerns **test values shared by the protocols**; it is not a modification of the I²C bus.

## 3. Spektrum X-Bus — `PROTOCOL=X`

### 3.1. New multi-address I²C transport, separate from SRXL2

**Files added or adapted:** `src/xbus.cpp`, `src/xbus.h`, `src/i2c_multi.c`, `src/i2c_multi.h`, `src/i2c_multi.pio`, `src/main.cpp`, and `CMakeLists.txt`. Tested receiver: **Spektrum AR6610T**.

- Reuse of the existing **sensor identifiers, structures, and formats in `srxl_sensors.h`** to build the Spektrum payload.
- **SRXL2 remains a separate half-duplex UART transport**; X-Bus instead answers receiver reads at several I²C addresses.
- Adoption of the **MSRC `i2c_multi` driver on PIO0**, adapted to oXs: lighter header dependencies, C/C++ linkage, and a **static TX buffer**, because the driver continues reading bytes after the callback returns.
- Sensor format: **16 bytes**, including identifier/address, secondary identifier, and data; copies protected against IRQ interference and data calculated outside interrupts.
- `handleXbusSpektrum()` **takes no address argument**: the driver callback determines which address was queried and selects the corresponding frame.

**Required CMake integration** (retain it in the project where the driver is actually installed):

```cmake
pico_generate_pio_header(oXs ${CMAKE_CURRENT_LIST_DIR}/src/i2c_multi.pio)
target_sources(oXs PRIVATE src/i2c_multi.c)
```

`hardware_pio` is already linked in the codebase examined. **Caution: all four PIO0 state machines** are used by this driver; coexistence with SBUS OUT, ESC reception, PWM/PIO0, or other PIO0 consumers is **not automatically guaranteed**. The prototype explicitly rejects some conflicts (`SBUS_OUT`, `ESC`), but not all of them.

### 3.2. Enabled addresses

| 7-bit address | Sensor presented to receiver |
|---|---|
| `0x11` | Airspeed |
| `0x16` | GPS position |
| `0x17` | GPS status, time, satellites |
| `0x18` | Receiver current, voltage, capacity |
| `0x20` | ESC: RPM, voltage, current, temperatures, etc. |
| `0x40` | Variometer / altitude |
| `0x7E` | RPM, voltage, temperature |

**Address `0x34` was not added** in this version. Sensor data is prepared from `fields[]` approximately every **100 ms**. `DEBUGTLM=Y` displays the number of requests received for each of the seven addresses roughly every **2 s**.

### 3.3. Electrical fixes and bus validation

- Following analysis of Logic captures, **correction of an SDA/SCL mix-up**: wires must follow the **physical signals**, not the decoder's initial labels; for the validated code: **GP8/TLM = SDA** and **GP9/PRI = SCL**.
- On this receiver, the lines were measured at around **3.3 V at rest without the two external 1 kΩ resistors**. Those resistors were removed during Spektrum testing; the RP2040 could then start normally after the receiver. The 1 kΩ resistors required by the Optima 7 are therefore **not a universal rule**.
- Tests with the receiver powered and the RP2040 off had shown **approximately 1.7 V on the RP2040's 3.3 V rail**: avoid setups that cause this parasitic back-powering; never apply 5 V to an RP2040 GPIO.
- With the **RP2040 started before the AR6610T**, telemetry was received and `FVP` genuinely changed the displayed values. Request counters for `0x11`, `0x16`, `0x17`, `0x18`, and `0x20` increased together; `0x40` was seen once, while `0x7E` was not queried in the supplied logs. This alone does not mean those sensors are faulty.

### 3.4. Resolution of simultaneous receiver / RP2040 startup

**Initial problem:** in the usual setup, **the receiver supplies 5 V to the RP2040**. The receiver apparently completed X-Bus discovery before the RP2040's I²C slave had been initialized. Artificially starting the RP2040 before the receiver worked; starting both together did not.

**File changed:** `src/main.cpp`. **Chosen solution:** early startup conditional exclusively on X-Bus:

1. Run `setupConfig()` earlier in `setup()` to determine the protocol **before** the `DEBUG` USB pause.
2. **Only for `config.protocol == 'X'`**, skip the USB connection wait (up to ~1 s) and the additional **2 seconds** in `DEBUG` mode. Other protocols keep that delay.
3. Prepare `fields[]` and the two required queues, then call **`setupXbusSpektrum()` before `multicore_launch_core1(core1_main)`**, i.e., before the lengthy sensor discovery on core 1.
4. Remove the old X-Bus call located after waiting for core 1 setup to finish: **X-Bus is initialized only once**. The main loop retains `handleXbusSpektrum()` under `PROTOCOL=X`.

Excerpt showing the new location:

```cpp
// After initializing fields[] and the queues, BEFORE core 1:
if (config.protocol == 'X') {
    setupXbusSpektrum();
}
multicore_launch_core1(core1_main);
```

**Reported result:** after replacing `main.cpp`, telemetry **appears to work on every simultaneous power-up**, with the receiver powering the RP2040. This is a real-world validation reported by Pierrot; no quantified campaign of hundreds of startups was provided.

**The 2N2222 used to hold SCL low and the software forcing of GP9 to LOW are NOT part of the chosen solution.** No changes to the X-Bus frames or `i2c_multi` were needed to resolve this startup sequence.

### 3.5. EdgeTX GPS date/time and documented limitations

- Analysis of the supplied EdgeTX `spektrum.cpp` showed that GPS frame `0x17` supplies **UTC time**, while the `UNIT_DATETIME` sensor builds its **date from the radio's internal clock**. The `2000-01-01` date came from the TX16S date setting, **not from a faulty X-Bus frame**; after correcting the radio's date, the result matched.
- Capacity encoded in `0x18` remains limited to approximately **3276.6 mAh** in the prototype's 0.1 mAh field; an `FVP` simulation of **34,321 mAh** is **saturated and not transmitted in full**. Extending `highCharge` or using another sensor type remains to be investigated if needed.
- For `0x40`, only the variation value corresponding to the **1-second** interval is filled with `VSPEED`; the other, uncalculated time deltas remain unavailable.

## 4. Quick reference — affected source files and precautions

| File / group | Purpose |
|---|---|
| `src/param.cpp` | Help and `PRI/TLM` validation for R/X/T; **2 MiB Flash/SAVE layout**. |
| `src/main.cpp` | Protocol selection; **early startup conditional on X-Bus**. |
| `src/rlink.cpp`, `src/rlink.h` | RadioLink I²C slave `0x04`, alternating 16-byte frames; check that GPS and IRQ fixes are combined. |
| `src/hitec.cpp`, `src/hitec.h` | HITEC `0x08`, `7 + FF` FIFO, GPS/date/altitude, test/live mode. |
| `src/xbus.cpp`, `src/xbus.h` | Multiple Spektrum sensors, formats shared with SRXL2, data from `fields[]`. |
| `src/i2c_multi.c/.h/.pio` | Multi-address I²C slave on **PIO0** for X-Bus. |
| `src/tools.cpp` | `FVP` applied to all test fields, consistent with `FVN`. |
| `CMakeLists.txt` | Generation of the `i2c_multi.pio` header and compilation of `i2c_multi.c`. |

### Retained configuration and wiring for HITEC / X-Bus

```text
PROTOCOL=T   # HITEC Optima 7      (or X for Spektrum AR6610T)
TLM=8        # GP8  = SDA
PRI=9        # GP9  = SCL
# Common ground; check that the I²C lines are at the 3.3 V level.
```

**Do not generalize the same pull-up arrangement**: the Optima 7 worked with **1 kΩ on each line to 3.3 V**; on the AR6610T the external resistors were removed during testing. For Spektrum, retain **early startup in `main.cpp`**, not a GPIO-to-5 V bridge or an untested SCL hold circuit.

**Always keep the latest full project and the UF2 whose startup and sensors have been verified, before making any further changes.**
