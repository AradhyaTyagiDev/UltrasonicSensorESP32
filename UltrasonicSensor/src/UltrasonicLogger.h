#include <Arduino.h>
#include <UltrasonicManager.h>

// Ultrasonic Sensor Logger
class UltrasonicLogger
{
private:
    uint32_t lastLog = 0;
    uint32_t intervalMs = 200;
    bool enabled;

public:
    UltrasonicLogger(uint32_t interval = 200, bool enable = true)
        : intervalMs(interval), enabled(enable) {}

    void setEnabled(bool enable) { enabled = enable; }

    // 🔹 Multi-sensor (existing)
    template <typename Manager>
    void log(uint32_t now, const Manager &manager)
    {
        if (!shouldLog(now))
            return;

        Serial.print("T:");
        Serial.print(now / 1000);

        size_t count = manager.getSensorCount();

        for (size_t i = 0; i < count; i++)
        {
            Serial.print(" | ");
            Serial.print(manager.getLabel(i));
            Serial.print(":");

            float value = manager.getDistanceByIndex(i);

            printValue(value);
        }

        Serial.println();
    }

private:
    inline bool shouldLog(uint32_t now)
    {
        if (!enabled)
            return false;
        if (now - lastLog < intervalMs)
            return false;

        lastLog = now;
        return true;
    }

    inline void printValue(float value)
    {
        if (value < 0)
            Serial.print("NA");
        else
            Serial.print(value, 1);
    }
};