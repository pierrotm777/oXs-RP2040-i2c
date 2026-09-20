#pragma once

// HITEC HTS-SS emulation for oXs / RP2040 SDK 2.1.0.
// SDA0 = TLM (GP8 on Pierrot's PCB), SCL0 = PRI (GP9).
#define HITEC_I2C_ADDRESS 0x08

// 0: live oXs fields[] (including FVP/FVN simulations); normal operation.
// 1: fixed values (Paris GPS, temperatures, RPM, voltage) for bench tests.
#ifndef HITEC_TEST_VALUES
#define HITEC_TEST_VALUES 0
#endif

// Working Optima 7 transfer: preload 7 HITEC bytes + 0xFF in the TX FIFO.
// External 1 kOhm pull-ups on SDA0/GP8 and SCL0/GP9 to 3.3 V.

// 1 = use the exact steady-state HTS-SS frame order measured on the Logic 2:
// 11,12,13,14,15,16,17,18,12,13,14,12,13,14, repeat.
// This setting is used only when HITEC_TEST_VALUES=1.
#ifndef HITEC_EXACT_SCHEDULE
#define HITEC_EXACT_SCHEDULE 1
#endif

void setupHitec();
void handleHitec();
