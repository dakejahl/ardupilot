#pragma once

#include "AP_HAL_Boards.h"

#ifndef HAL_OSD_SPI_TRACE_ENABLED
#define HAL_OSD_SPI_TRACE_ENABLED 0
#endif

#if HAL_OSD_SPI_TRACE_ENABLED

#include <stdint.h>
#include "AP_HAL_Macros.h"

#ifndef HAL_OSD_SPI_TRACE_BACKLOG_KB
// the OSD's boot traffic (probe plus a readback of every glyph) is
// about 120 KB and all of it happens before the log file opens
#define HAL_OSD_SPI_TRACE_BACKLOG_KB 128
#endif

#ifndef HAL_OSD_SPI_TRACE_LOG_RATE
// bytes per second of OSPI messages; well under SD card throughput so
// a drained backlog never crowds the vehicle's own messages out of
// the logger's buffer
#define HAL_OSD_SPI_TRACE_LOG_RATE 40000
#endif

class ByteBuffer;

namespace AP_HAL {

/*
  Records every chip-select frame of the OSD's SPI device (MOSI and
  MISO bytes) into a RAM backlog from boot, and drains the backlog
  into OSPI log messages once logging has started.

  The bus driver calls cs_asserted(), then segment_tx()/segment_rx()
  around each transfer, then cs_released(). All of those run with the
  bus semaphore held, so there is one producer at a time; the drain
  runs in the IO thread as the single consumer of the backlog.
 */
class SPITrace {
public:
    static SPITrace *get_singleton();

    static constexpr uint16_t MAX_STORED = 512;
    static constexpr uint8_t CHUNK = 64;

    // flags in the OSPI Fl field; bits 0-11 hold the stored length
    static constexpr uint16_t FLAG_TX = 1U << 15;
    static constexpr uint16_t FLAG_RX = 1U << 14;
    static constexpr uint16_t FLAG_TRUNCATED = 1U << 13;
    static constexpr uint16_t FLAG_FAILED = 1U << 12;
    static constexpr uint16_t LEN_MASK = 0x0FFF;

    void cs_asserted();
    // call before the transfer: it may overwrite send in place
    void segment_tx(const uint8_t *send, uint32_t len);
    // recv is nullptr when MISO was not sampled
    void segment_rx(const uint8_t *recv, uint32_t len, bool ok);
    void cs_released();

private:
    SPITrace() = default;

    struct PACKED Header {
        uint64_t start_us;
        uint32_t seq;
        uint32_t lost;
        uint16_t dur_us;
        uint16_t flags;
    };

    struct Frame {
        Header hdr;
        uint8_t tx[MAX_STORED];
        uint8_t rx[MAX_STORED];
    };

    void init();
    void drain();
    bool load_next();
    uint16_t stored_len(const Frame &f) const { return f.hdr.flags & LEN_MASK; }

    static SPITrace *_singleton;

    ByteBuffer *backlog {};
    uint32_t backlog_size {};

    // producer side
    Frame open {};
    bool frame_open {};
    bool rx_all {};
    bool any_segment {};
    uint32_t frame_len {};
    uint32_t next_seq {};
    uint32_t lost {};

    // consumer side
    Frame cur {};
    bool have_cur {};
    uint8_t cur_part {};
    uint8_t cur_parts {};
    uint32_t last_drain_us {};
    float tokens {};
    uint32_t retry_after_ms {};
    bool retry_pending {};
    bool announced {};
};

}

#endif  // HAL_OSD_SPI_TRACE_ENABLED
