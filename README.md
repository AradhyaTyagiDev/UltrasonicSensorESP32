## **Ultrasonic Sensor for ESP32 based on `pulseIn()`: Blocking API**

**Objective:** Obstacle detection + Distance measurement

**Tool Used**: Ardino IDO + Visual Studio + PlatformIO

**MCU**: ESP32 WROOM

**Sensor:** Ultrasonic Sensor: HC-SR04

**Buck Converter**: LM2596

**Voltage Divider**: 1k + 2k Resister

**Pin Configurations:**
TRIG → GPIO 5
ECHO → GPIO 18 (⚠️ via voltage divider) : 1kΩ + 2kΩ. GPIO18 Pin Voltage: 3.34v which safe for ESP32.

