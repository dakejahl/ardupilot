#include "SPITrace.h"

#if HAL_OSD_SPI_TRACE_ENABLED

#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace AP_HAL;

SPITrace *SPITrace::_singleton;

// gnu++11 needs these out-of-line for any use that binds a reference
constexpr uint16_t SPITrace::MAX_STORED;
constexpr uint8_t SPITrace::CHUNK;
constexpr uint16_t SPITrace::FLAG_TX;
constexpr uint16_t SPITrace::FLAG_RX;
constexpr uint16_t SPITrace::FLAG_TRUNCATED;
constexpr uint16_t SPITrace::FLAG_FAILED;
constexpr uint16_t SPITrace::LEN_MASK;

SPITrace *SPITrace::get_singleton()
{
    if (_singleton == nullptr) {
        _singleton = NEW_NOTHROW SPITrace();
        if (_singleton != nullptr) {
            _singleton->init();
        }
    }
    return _singleton;
}

void SPITrace::init()
{
    backlog = NEW_NOTHROW ByteBuffer(0);
    if (backlog != nullptr && backlog->set_size_best(HAL_OSD_SPI_TRACE_BACKLOG_KB * 1024U)) {
        backlog_size = backlog->get_size();
    }
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&SPITrace::drain, void));
}

void SPITrace::cs_asserted()
{
    if (frame_open) {
        cs_released();
    }
    frame_open = true;
    rx_all = true;
    any_segment = false;
    frame_len = 0;
    open.hdr.start_us = AP_HAL::micros64();
    open.hdr.seq = next_seq++;
    open.hdr.flags = 0;
}

void SPITrace::segment_tx(const uint8_t *send, uint32_t len)
{
    if (!frame_open || frame_len >= MAX_STORED) {
        return;
    }
    const uint32_t n = MIN(len, MAX_STORED - frame_len);
    if (send != nullptr) {
        memcpy(&open.tx[frame_len], send, n);
    } else {
        // the ChibiOS SPI driver clocks out its 0xFF dummy word on receive-only transfers
        memset(&open.tx[frame_len], 0xFF, n);
    }
}

void SPITrace::segment_rx(const uint8_t *recv, uint32_t len, bool ok)
{
    if (!frame_open) {
        return;
    }
    any_segment = true;
    if (!ok) {
        open.hdr.flags |= FLAG_FAILED;
    }
    if (recv == nullptr) {
        rx_all = false;
    }
    if (frame_len < MAX_STORED) {
        const uint32_t n = MIN(len, MAX_STORED - frame_len);
        if (recv != nullptr) {
            memcpy(&open.rx[frame_len], recv, n);
        } else {
            memset(&open.rx[frame_len], 0, n);
        }
    }
    frame_len += len;
}

void SPITrace::cs_released()
{
    if (!frame_open) {
        return;
    }
    frame_open = false;

    const uint64_t dur_us = AP_HAL::micros64() - open.hdr.start_us;
    open.hdr.dur_us = MIN(dur_us, uint64_t(UINT16_MAX));

    const uint16_t n = MIN(frame_len, uint32_t(MAX_STORED));
    // MOSI always has a value: the caller's bytes or the driver's 0xFF fill
    uint16_t flags = (open.hdr.flags & ~LEN_MASK) | FLAG_TX | n;
    if (frame_len > MAX_STORED) {
        flags |= FLAG_TRUNCATED;
    }
    // a frame with any transmit-only segment has no complete MISO record
    if (rx_all && any_segment) {
        flags |= FLAG_RX;
    }
    open.hdr.flags = flags;
    open.hdr.lost = lost;

    const uint32_t rx_len = (flags & FLAG_RX) ? n : 0;
    const uint32_t need = sizeof(Header) + n + rx_len;
    if (backlog_size == 0 || backlog->space() < need) {
        lost++;
        return;
    }
    // the drain only takes a record once all of it is in the buffer
    backlog->write((const uint8_t *)&open.hdr, sizeof(Header));
    backlog->write(open.tx, n);
    if (rx_len > 0) {
        backlog->write(open.rx, rx_len);
    }
}

bool SPITrace::load_next()
{
    if (backlog_size == 0) {
        return false;
    }
    const uint32_t avail = backlog->available();
    Header hdr;
    if (avail < sizeof(hdr) ||
        backlog->peekbytes((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr)) {
        return false;
    }
    const uint16_t n = hdr.flags & LEN_MASK;
    const uint32_t rx_len = (hdr.flags & FLAG_RX) ? n : 0;
    if (avail < sizeof(hdr) + n + rx_len) {
        // the producer is part way through writing this record
        return false;
    }
    backlog->advance(sizeof(hdr));
    cur.hdr = hdr;
    backlog->read(cur.tx, n);
    if (rx_len > 0) {
        backlog->read(cur.rx, rx_len);
    }
    cur_part = 0;
    cur_parts = MAX((n + CHUNK - 1) / CHUNK, 1);
    have_cur = true;
    return true;
}

void SPITrace::drain()
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_us = AP_HAL::micros();
    const uint32_t dt_us = now_us - last_drain_us;
    last_drain_us = now_us;

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr || !logger->logging_started()) {
        tokens = 0;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (retry_pending) {
        if (int32_t(now_ms - retry_after_ms) < 0) {
            return;
        }
        retry_pending = false;
    }

    // a short burst cap keeps a late IO tick from dumping a block of messages at once
    const float burst = 8 * sizeof(log_OSPI);
    tokens = MIN(tokens + dt_us * (HAL_OSD_SPI_TRACE_LOG_RATE * 1.0e-6f), burst);

    while (tokens >= sizeof(log_OSPI)) {
        if (!have_cur && !load_next()) {
            break;
        }
        const uint16_t n = cur.hdr.flags & LEN_MASK;
        const uint16_t ofs = cur_part * CHUNK;
        const uint16_t chunk = ofs < n ? MIN(uint16_t(n - ofs), uint16_t(CHUNK)) : 0;

        struct log_OSPI pkt {
            LOG_PACKET_HEADER_INIT(LOG_OSPI_MSG),
            time_us : cur.hdr.start_us,
            seq     : cur.hdr.seq,
            part    : cur_part,
            flags   : cur.hdr.flags,
            dur_us  : cur.hdr.dur_us,
            lost    : cur.hdr.lost,
            tx      : {},
            rx      : {},
        };
        memcpy(pkt.tx, &cur.tx[ofs], chunk);
        if (cur.hdr.flags & FLAG_RX) {
            memcpy(pkt.rx, &cur.rx[ofs], chunk);
        }

        // a refused write stays queued: the logger is still writing its
        // startup messages or its buffer is close to full
        if (!logger->WriteBlock_first_succeed(&pkt, sizeof(pkt))) {
            retry_pending = true;
            retry_after_ms = now_ms + 20;
            break;
        }
        tokens -= sizeof(pkt);
        if (++cur_part >= cur_parts) {
            have_cur = false;
        }
        if (!announced) {
            // sent after the first accepted write so that the text also reaches the log
            announced = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OSD SPI trace: %u KB backlog, %u frames lost",
                          unsigned(backlog_size / 1024), unsigned(lost));
        }
    }
#endif  // HAL_LOGGING_ENABLED
}

#endif  // HAL_OSD_SPI_TRACE_ENABLED
