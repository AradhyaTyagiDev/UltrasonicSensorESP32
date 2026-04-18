#pragma once

#include <Arduino.h>
#include <UltrasonicTypes.h>

template <size_t N>
class UltrasonicManager
{
private:
    UltrasonicConfig configs[N]; // immutable config
    UltrasonicState states[N];   // runtime state

public:
    // Compile-time safety check
    static_assert(N == static_cast<size_t>(UltrasonicSensorId::COUNT), "Sensor count must match UssSensorId::COUNT");

    // Constructor injection
    UltrasonicManager(const UltrasonicConfig (&cfg)[N])
    {
        for (size_t i = 0; i < N; i++)
        {
            configs[i] = cfg[i];
        }
    }

    void begin()
    {
        for (size_t i = 0; i < N; i++)
        {
            pinMode(configs[i].trigPin, OUTPUT);
            pinMode(configs[i].echoPin, INPUT);

            // Ensure stable LOW state
            digitalWrite(configs[i].trigPin, LOW);
        }
    }

    void update(uint32_t currentMillis)
    {
        for (size_t i = 0; i < N; i++)
        {
            if (currentMillis - states[i].lastRun >= configs[i].interval)
            {
                states[i].lastRun = currentMillis;
                states[i].lastDistance = readSensor(i);
            }
        }
    }

    // ✅ Type-safe API
    // id: Which Sensor: FRONT, LEFT...
    float getDistance(UltrasonicSensorId id) const
    {
        uint8_t index = static_cast<uint8_t>(id);

        if (index >= N)
            return -1.0f;

        return states[index].lastDistance;
    }

    // Get Distance by Index
    float getDistanceByIndex(size_t index) const
    {
        if (index >= N)
            return -1.0f;
        return states[index].lastDistance;
    }

    size_t getSensorCount() const
    {
        return N;
    }

    char getLabel(size_t index) const
    {
        if (index >= N)
            return '?';
        return configs[index].label;
    }

private:
    float readSensor(size_t i)
    {
        const UltrasonicConfig &s = configs[i];

        // Trigger pulse
        digitalWrite(s.trigPin, LOW);
        delayMicroseconds(2);

        digitalWrite(s.trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(s.trigPin, LOW);

        // ⚠️ Blocking (to be replaced later)
        /// Warning: pulseIn() is blocking. Need to replace with `Interrupt-based echo: RMT peripheral`
        /// Each sensor read can block up to 30ms. With 4 sensors: 30ms × 4 = 120ms blocking (❌)
        long duration = pulseIn(s.echoPin, HIGH, 30000);

        if (duration == 0)
            return -1;

        return duration * 0.0343f / 2.0f;
    }
};