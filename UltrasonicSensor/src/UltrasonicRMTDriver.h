#pragma once

#include <Arduino.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "UltrasonicTypes.h"

template <size_t N>
class UltrasonicRMTDriver
{
public:
    static_assert(N <= 8, "ESP32 supports max 8 RMT channels");
    /// Create xQueueCreate(32, sizeof(UltrasonicEchoEvent)); and Pass
    UltrasonicRMTDriver(const UltrasonicConfig (&cfg)[N],
                        QueueHandle_t queue)
        : echoQueue(queue)
    {
        assert(queue != nullptr);

        for (size_t i = 0; i < N; i++)
        {
            configs[i] = cfg[i];
        }
    }

    void begin()
    {
        for (size_t i = 0; i < N; i++)
        {
            setupRMTChannel(i);
        }

        // Register global RX done callback
        rmt_rx_register_event_callbacks(&rx_callbacks, this);
    }

    // Call from manager/task after processing event
    void restartReceive(uint8_t sensorIndex)
    {
        if (sensorIndex >= N)
            return;

        startReceive(sensorIndex);
    }

    // ================================
    // 🔷 DROP STATS
    // ================================

    uint32_t getTotalDrops() const
    {
        return __atomic_load_n(&totalDrops, __ATOMIC_RELAXED);
    }

    uint32_t getSensorDrops(uint8_t i) const
    {
        return __atomic_load_n(&dropCounter[i], __ATOMIC_RELAXED);
    }

private:
    DRAM_ATTR UltrasonicConfig configs[N];
    QueueHandle_t echoQueue;

    // ================================
    // 🔷 CACHE + DROP TRACKING
    // ================================
    // ISR is asynchronous
    uint32_t dropCounter[N] = {0}; // dropCounter per sensor
    uint32_t totalDrops = 0;       // global

    // ================================
    // 🔷 RMT CALLBACK STRUCT
    // ================================
    // 👉 IRAM_ATTR tells the compiler: “Put this function in internal RAM (IRAM) instead of Flash.”
    // ✔ Function is stored in internal RAM
    // ✔ Always accessible
    // ✔ Safe during interrupts
    /// IRAM_ATTR = Make ISR safe and crash-proof
    static bool IRAM_ATTR onReceiveDone(
        rmt_channel_handle_t channel,
        const rmt_rx_done_event_data_t *edata,
        void *user_ctx)
    {
        auto *driver = static_cast<UltrasonicRMTDriver *>(user_ctx);

        UltrasonicEchoEvent evt;

        // Map channel → sensorId
        uint8_t ch = driver->channelToIndex(channel);
        evt.sensorId = static_cast<UltrasonicSensorId>(ch);

        // Duration in microseconds (depends on resolution). microsecond precision
        evt.duration = driver->parseDuration(edata);

        // 🔥 ISR-safe timestamp. Precision: 1–10 ms
        // if need high precision: evt.highResTime = (uint32_t)esp_timer_get_time();
        evt.timestamp = xTaskGetTickCountFromISR();

        evt.timeout = false;

        BaseType_t hpTaskWoken = pdFALSE;

        // 🔥 Push to global queue
        // ✔ ISR-safe API
        if (xQueueSendFromISR(driver->echoQueue, &evt, &hpTaskWoken) != pdPASS)
        {
            // 🔴 Drop handling: atomic update
            __atomic_add_fetch(&driver->dropCounter[ch], 1, __ATOMIC_RELAXED);
            __atomic_add_fetch(&driver->totalDrops, 1, __ATOMIC_RELAXED);
        }

        return hpTaskWoken == pdTRUE;
    }

    static constexpr rmt_rx_event_callbacks_t rx_callbacks = {
        .on_recv_done = onReceiveDone,
    };

    // ================================
    // 🔷 SETUP PER CHANNEL
    // ================================
    void setupRMTChannel(size_t i)
    {
        rmt_rx_channel_config_t rx_cfg = {};
        rx_cfg.gpio_num = (gpio_num_t)configs[i].echoPin;
        rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        rx_cfg.resolution_hz = 1000000; // 1 MHz → 1 tick = 1µs
        rx_cfg.mem_block_symbols = 64;
        rx_cfg.flags.with_dma = false;

        rmt_channel_handle_t channel = nullptr;
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &channel));

        channels[i] = channel;

        // Enable channel
        ESP_ERROR_CHECK(rmt_enable(channel));
    }

public:
    // ================================
    // 🔷 START RECEIVE (NON-BLOCKING)
    // ================================
    void startReceive(size_t i)
    {
        if (i >= N)
            return;

        rmt_receive_config_t cfg = {};
        cfg.signal_range_min_ns = 1000;     // ignore noise < 1µs
        cfg.signal_range_max_ns = 30000000; // 30ms max

        ESP_ERROR_CHECK(
            rmt_receive(
                channels[i],
                rxBuffers[i],
                sizeof(rxBuffers[i]),
                &cfg));
    }

private:
    // ================================
    // 🔷 INTERNAL STORAGE
    // ================================
    DRAM_ATTR rmt_channel_handle_t channels[N];

    // Each sensor gets its own buffer (NO dynamic alloc)→ no collision
    DRAM_ATTR rmt_symbol_word_t rxBuffers[N][64];

    // ================================
    // 🔷 HELPERS
    // ================================

    // 👉 Finds which sensor triggered the interrupt. Linear search → fine for ≤ 8 sensors
    // ISR runs when flash cache may be disabled. These functions live in flash → crash risk
    uint8_t IRAM_ATTR channelToIndex(rmt_channel_handle_t ch)
    {
// Unroll this loop (expand it manually instead of iterating).
#pragma unroll
        for (uint8_t i = 0; i < N; i++)
        {
            if (channels[i] == ch)
                return i;
        }
        return 0; // fallback (should not happen)
    }

    // HC-SR04: HIGH pulse = echo duration
    // RMT gives symbols (level0, duration0, level1, duration1)
    // symbol is a compact way to store a piece of waveform.
    // 👉 One symbol = two signal segments
    // Symbol = [level0 + duration0] + [level1 + duration1]
    uint32_t IRAM_ATTR parseDuration(const rmt_rx_done_event_data_t *edata)
    {
        if (!edata || edata->num_symbols == 0)
            return 0;

        const rmt_symbol_word_t *s = edata->received_symbols;

        uint32_t duration = 0;

        // Symbol 0
        if (s[0].level0 == 1)
            duration += s[0].duration0;
        if (s[0].level1 == 1)
            duration += s[0].duration1;

        // Symbol 1 (if exists)
        if (edata->num_symbols > 1)
        {
            if (s[1].level0 == 1)
                duration += s[1].duration0;
            if (s[1].level1 == 1)
                duration += s[1].duration1;
        }

        return duration; // in µs (since resolution = 1MHz)
    }
};
