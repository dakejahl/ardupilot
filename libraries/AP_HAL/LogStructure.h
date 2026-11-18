#pragma once

#include <AP_Logger/LogStructure.h>
#include "UARTDriver.h"
#include "SPITrace.h"

#if HAL_OSD_SPI_TRACE_ENABLED
#define LOG_IDS_FROM_HAL \
    LOG_UART_MSG, \
    LOG_OSPI_MSG
#else
#define LOG_IDS_FROM_HAL \
    LOG_UART_MSG
#endif

// @LoggerMessage: UART
// @Description: UART stats
// @Field: TimeUS: Time since system startup
// @Field: I: instance
// @Field: Tx: transmitted data rate bytes per second
// @Field: Rx: received data rate bytes per second, this is all incoming data, it may not all be processed by the driver using this port.
// @Field: RxDp: Data rate of dropped received bytes, ideally should be 0. This is the difference between the received data rate and the processed data rate.
struct PACKED log_UART {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float tx_rate;
    float rx_rate;
    float rx_drop_rate;
};

// @LoggerMessage: OSPI
// @Description: Debug trace of the OSD's SPI traffic. Each chip-select frame is split into one or more messages of 64 bytes per direction.
// @Field: TimeUS: Time since system startup at which the frame's chip select was asserted
// @Field: Seq: Frame number since boot. A gap means frames were dropped because the trace backlog was full
// @Field: Part: Index of this 64-byte chunk within the frame
// @Field: Fl: Bit 15 set when MOSI was captured (always), bit 14 set when MISO was captured, bit 13 set when the frame was longer than the bytes stored, bit 12 set when a transfer failed. Bits 0 to 11 hold the number of bytes stored for the frame
// @Field: Dur: Time the chip select was asserted, saturating at 65535
// @Field: Lost: Frames dropped before this frame was stored
// @Field: Tx: MOSI bytes of this chunk
// @Field: Rx: MISO bytes of this chunk, zero when MISO was not captured
struct PACKED log_OSPI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t part;
    uint16_t flags;
    uint16_t dur_us;
    uint32_t lost;
    // logged as 'a' because DFReader truncates 'Z' at the first zero byte
    uint8_t tx[64];
    uint8_t rx[64];
};

#if HAL_UART_STATS_ENABLED
#define LOG_STRUCTURE_FROM_HAL_UART                     \
    { LOG_UART_MSG, sizeof(log_UART),                   \
      "UART","QBfff","TimeUS,I,Tx,Rx,RxDp", "s#BBB", "F----" },
#else
#define LOG_STRUCTURE_FROM_HAL_UART
#endif

#if HAL_OSD_SPI_TRACE_ENABLED
#define LOG_STRUCTURE_FROM_HAL_OSPI                     \
    { LOG_OSPI_MSG, sizeof(log_OSPI),                   \
      "OSPI","QIBHHIaa","TimeUS,Seq,Part,Fl,Dur,Lost,Tx,Rx", "s---s---", "F---F---" },
#else
#define LOG_STRUCTURE_FROM_HAL_OSPI
#endif

#define LOG_STRUCTURE_FROM_HAL                          \
    LOG_STRUCTURE_FROM_HAL_UART                         \
    LOG_STRUCTURE_FROM_HAL_OSPI
