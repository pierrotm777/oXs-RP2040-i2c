#pragma once

// Spektrum X-Bus I2C multi-address slave.  Only instantiated when PROTOCOL=X.
// For the first proof-of-concept, the PIO0 block must be dedicated to X-Bus.
void setupXbusSpektrum();
void handleXbusSpektrum();
