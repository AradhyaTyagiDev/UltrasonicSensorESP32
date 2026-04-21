#include <Arduino.h>
#include <UltrasonicManager.h>
#include <UltrasonicLogger.h>

// Pin definitions
// Define all sensors in ONE place (compile-time)
namespace
{
    constexpr UltrasonicConfig ultrasonicConfigs[] = {
        {5, 18, 50, 'F'}, // Front
        // {17, 16, 70, 'L'}, // Left
        // {4, 2, 90, 'R'},   // Right
        // {21, 19, 110, 'B'} // Rear
    };
}

UltrasonicManager<1> ultrasonicManager(ultrasonicConfigs);

/// @brief Logger to print logs. Do Comment or Disabled in Production.
// Small delay for readability (NOT for sensor timing)
// 👉 Serial readability: Without delay: You’ll flood Serial Monitor (thousands of lines/sec)
// ❌ Remove it for Production for Robot. ✅ Keep it ONLY if: Debugging via Serial
UltrasonicLogger logger(200, true);

// put your setup code here, to run once:
void setup()
{
    Serial.begin(115200);
    ultrasonicManager.begin();

    Serial.println("Ultrasonic System Initialized...");
}

// ================= LOOP =================
// put your main code here, to run repeatedly:
void loop()
{
    unsigned long now = millis();

    // Update all sensors (non-blocking scheduling inside)
    ultrasonicManager.update(now);

    // Read distances (type-safe)
    // float frontDist = ultrasonicManager.getDistance(UltrasonicSensorId::FRONT);

    logger.log(now, ultrasonicManager);
}
