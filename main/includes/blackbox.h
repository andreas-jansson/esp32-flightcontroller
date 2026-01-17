#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "ringbuffer.h"
#include "common_data.h"

namespace blackbox
{

    struct logItem
    {
        uint32_t magicNr{}; // generate a random number to distinguish between different runs?
        uint32_t seqNr{};
        TelemetryData data;
        uint8_t crc;
    };

    constexpr uint32_t flashBlockSize{0x1000}; // 4096
}

class Blackbox
{

    CircularHandle_t m_flashHandle{};
    CircularHandle_t m_telemetryHandle{};
    blackbox::logItem* m_logBuffer{};
    uint32_t m_nrItems{};
    esp_err_t get_telemetry();

public:
    Blackbox(CircularHandle_t blackbox_telemetry_handle);

    void *log_task(void *args);
};
