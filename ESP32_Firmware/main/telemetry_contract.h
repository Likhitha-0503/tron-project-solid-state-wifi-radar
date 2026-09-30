#ifndef TELEMETRY_CONTRACT_H
#define TELEMETRY_CONTRACT_H

#include <stdint.h>

// Force the compiler to pack the struct tightly without memory padding
#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp;  // 4 bytes: Microsecond or tick counter
    uint8_t  beam_ID;    // 1 byte: 0-15 for the Butler Matrix array
    int8_t   rssi_value; // 1 byte: Typical RSSI range (-100 to 0)
    uint16_t crc16;      // 2 bytes: IAntegrity check
} spi_telemetry_packet_t; // Total: 8 bytes per transaction
#pragma pack(pop)

// Compile-time check to guarantee the struct is exactly 8 bytes.
_Static_assert(sizeof(spi_telemetry_packet_t) == 8, "spi_telemetry_packet_t size must be exactly 8 bytes!");

#endif // TELEMETRY_CONTRACT_H