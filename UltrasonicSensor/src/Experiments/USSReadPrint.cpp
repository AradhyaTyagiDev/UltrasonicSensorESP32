/// This program will read the duration from Ultrasonic sensor and convert into distance and print it on Console.

#include <Arduino.h>

// Pin definitions
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

// Variables
long duration;
float distance_cm;

// Timing
unsigned long currentTimeSec;

void setup1()
{
    Serial.begin(115200);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Serial.println("Ultrasonic Sensor Started...");
}

void loop1()
{
    // Ensure TRIG is LOW
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send 10us pulse to TRIG
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read ECHO pulse duration
    duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms

    // Convert to distance (cm)
    distance_cm = duration * 0.0343 / 2;

    // Get time in seconds since boot
    currentTimeSec = millis() / 1000;

    // Print output
    Serial.print("Time(s): ");
    Serial.print(currentTimeSec);
    Serial.print(" | Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");

    delay(500); // adjust as needed
}