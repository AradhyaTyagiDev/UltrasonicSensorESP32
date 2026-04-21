#pragma once
#include <Arduino.h>
#include <math.h>

/// @brief define Inside UltrasonicManager or Pass as dependency (clean architecture): ❌ Global variables → hard to control
struct UltrasonicTuning
{
    // ================================
    // 🔷 Scheduling / Timing
    // ================================
    struct Timing
    {
        uint32_t interSensorGapUs = 3000; // gap between triggers (3–5ms)
    } timing;

    // ================================
    // 🔷 Relevance / Selection
    // ================================
    struct Relevance
    {
        float threshold = 5.0f; // minimum score to consider sensor
    } relevance;

    // ================================
    // 🔷 Scoring Weights
    // ================================
    struct Scoring
    {
        float basePriority = 1.0f;     // default baseline (can be overridden per config)
        float alignmentWeight = 20.0f; // motion-direction importance
        float riskWeight = 100.0f;     // distance-based risk

        float closeDistanceThreshold = 30.0f; // cm
        float closeDistanceBoost = 50.0f;     // strong boost when very close

        float speedScale = 1.0f; // how speed affects score

        float velocityWeight = 40.0f;  // predictive boost
        float confidenceWeight = 1.0f; // scale final score
    } scoring;

    // ================================
    // 🔷 Noise Model
    // ================================
    struct Noise
    {
        // Valid measurement range
        float minValidDistance = 2.0f;   // cm (sensor limitation)
        float maxValidDistance = 400.0f; // cm

        // Soft zones (gradual degradation)
        float nearMinZoneWidth = 5.0f;  // cm from min
        float nearMaxZoneWidth = 50.0f; // cm from max

        // Penalty strengths (0 → 1)
        float nearMinPenalty = 0.3f;
        float nearMaxPenalty = 0.3f;

        // Timeout = worst case
        float timeoutNoise = 1.0f;

        // Threshold to classify as noisy (for boolean use)
        float threshold = 0.7f;
    } noise;

    struct GroupBias
    {
        float frontBoost = 20.0f;
        float sideBoost = 5.0f;
        float rearPenalty = -10.0f;
    } groupBias;

    struct Scheduling
    {
        float hysteresisBoost = 5.0f;
    } scheduling;
};

enum class UltrasonicSensorId : uint8_t
{
    FRONT,
    FRONT_LEFT,
    LEFT,
    FRONT_RIGHT,
    RIGHT,
    REAR_LEFT,
    REAR,
    REAR_RIGHT,
    COUNT
};

enum class SensorGroup : uint8_t
{
    FRONT,
    SIDE,
    REAR
};

inline SensorGroup getSensorGroup(UltrasonicSensorId id)
{
    switch (id)
    {
    case UltrasonicSensorId::FRONT:
    case UltrasonicSensorId::FRONT_LEFT:
    case UltrasonicSensorId::FRONT_RIGHT:
        return SensorGroup::FRONT;

    case UltrasonicSensorId::LEFT:
    case UltrasonicSensorId::RIGHT:
        return SensorGroup::SIDE;

    case UltrasonicSensorId::REAR:
    case UltrasonicSensorId::REAR_LEFT:
    case UltrasonicSensorId::REAR_RIGHT:
        return SensorGroup::REAR;

    default:
        return SensorGroup::SIDE;
    }
}

struct SensorGeometry
{
    float dirX; // normalized direction vector
    float dirY;
};

struct MotionVector
{
    float x; // direction
    float y;
    float speed; // magnitude
};

enum class RobotMotionState : uint8_t
{
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    BACKWARD,
    IDLE
};

/// State Machine
enum class UltrasonicSensorPhase : uint8_t
{
    IDLE,
    TRIGGERED,
    WAITING,
    RECEIVED,
    PROCESSING,
    DONE
};

struct UltrasonicConfig
{
    uint8_t trigPin;
    uint8_t echoPin;
    // Stagger intervals to avoid ultrasonic interference: Sensor Interference Avoided: 50ms, 70ms, 90ms, 110ms
    uint32_t minIntervalMs;

    float basePriority; // base weight (e.g., 1.0–10.0)
    SensorGeometry geometry;

    char label; // 'F', 'L', 'R', 'B'
};

// UltrasonicState state[N];     // final output
struct UltrasonicState
{
    uint32_t lastRun = 0;

    float distance = -1.0f;
    /// final published output
    float confidence = 0.0f;

    float dynamicPriority = 0.0f;

    bool valid = false;
};

/// @brief Runtime Behavior: live state container of one sensor
/// 1. When sensor was triggered
/// 2. Whether echo started/ended
/// 3. Whether result is ready
/// 4. Timeout handling
/// 5. Current state in pipeline
struct UltrasonicRuntime
{
    UltrasonicSensorPhase phase = UltrasonicSensorPhase::IDLE;
    // Timestamp when TRIG pulse was sent
    uint32_t triggerTime = 0;
    // echoStart = rising edge time
    // echoEnd = falling edge time. duration = echoEnd - echoStart
    uint32_t duration = 0; // from RMT directly
    // Updated when echo received
    float distance = -1.0f;
    float previousDistance = -1.0f;

    float relativeVelocity = 0.0f; // cm/s
    uint32_t lastUpdateTime = 0;   // for dt calculation

    // Flag set by ISR/queue when echo arrives
    bool echoReceived = false;
    bool timeout = false;

    bool enabled = true;
    float dynamicPriority = 0.0f;
    /// internal computation
    float confidence = 1.0f; // 0 → 1
};

/// @brief message container used to transfer data from: ISR (RMT interrupt) ➜ Main application (UltrasonicManager)
/// Step 1: RMT captures echo pulse
/// Step 2: ISR fires EchoEvent: xQueueSendFromISR(queueHandle, &evt, NULL); -> Pack data -> Push to queue -> Exit FAST ⚡
/// Step 3: Main loop processes it asyncronously
/// ISR = data producer
/// Manager = data consumer
/// Enables Scalability (1 → 8 sensors): Same queue handles all sensors: Queue = thread-safe communication
struct UltrasonicEchoEvent
{
    UltrasonicSensorId sensorId;
    uint32_t duration;  // µs : distance = duration * 0.0343 / 2;
    uint32_t timestamp; // micros()
    bool timeout;
};

/// @brief Usage: manager[UltrasonicSensorId::FRONT].distance = 120;
constexpr uint8_t toIndex(UltrasonicSensorId id)
{
    return static_cast<uint8_t>(id);
}

/// @brief keep in UltrasonicRMTManager
UltrasonicRuntime &operator[](UltrasonicSensorId id)
{
    return runtime[toIndex(id)];
}

const UltrasonicRuntime &operator[](UltrasonicSensorId id) const
{
    return runtime[toIndex(id)];
}

//----------------------------------PRIORITY-------------------------------------------------//
float computeDynamicPriority(
    UltrasonicSensorId id,
    const MotionVector &motion,
    const UltrasonicConfig &cfg,
    const UltrasonicRuntime &rt,
    const UltrasonicTuning &t)
{
    // ================================
    // 0. Early Exit
    // ================================
    if (!rt.enabled || !rt.valid || rt.confidence < 0.1f)
        return 0.0f;

    float score = cfg.basePriority;

    // ================================
    // 1. Normalize Motion Vector
    // ================================
    float mx = motion.x;
    float my = motion.y;

    float mag = sqrtf(mx * mx + my * my);
    if (mag > 0.0001f)
    {
        mx /= mag;
        my /= mag;
    }
    else
    {
        mx = 0.0f;
        my = 0.0f;
    }

    // ================================
    // 2. Direction Alignment
    // ================================
    float alignment = cfg.geometry.dirX * mx +
                      cfg.geometry.dirY * my;

    alignment = (alignment > 0.0f) ? alignment : 0.0f;

    score += alignment * t.scoring.alignmentWeight;

    // ================================
    // 🔥 3. Sensor Group Bias
    // ================================
    SensorGroup group = getSensorGroup(id);

    switch (group)
    {
    case SensorGroup::FRONT:
        if (mx > 0.0f) // moving forward
            score += t.groupBias.frontBoost;
        break;

    case SensorGroup::SIDE:
        score += t.groupBias.sideBoost;
        break;

    case SensorGroup::REAR:
        if (mx > 0.0f) // moving forward → deprioritize rear
            score += t.groupBias.rearPenalty;
        break;
    }

    // ================================
    // 4. Distance-based Risk
    // ================================
    if (rt.distance > 0.0f)
    {
        float d = clampf(rt.distance,
                         t.noise.minValidDistance,
                         t.noise.maxValidDistance);

        score += t.scoring.riskWeight / (d + 1.0f);

        if (d < t.scoring.closeDistanceThreshold)
        {
            float factor =
                (t.scoring.closeDistanceThreshold - d) /
                t.scoring.closeDistanceThreshold;

            score += factor * t.scoring.closeDistanceBoost;
        }

        // ================================
        // 5. Predictive Velocity Boost
        // ================================
        if (rt.relativeVelocity > 0.0f)
        {
            float v = clampf(rt.relativeVelocity, 0.0f, 200.0f);
            score += v * t.scoring.velocityWeight;
        }
    }

    // ================================
    // 6. Motion Speed Influence
    // ================================
    float speed = clampf(motion.speed, 0.0f, 5.0f);
    score *= (1.0f + speed * t.scoring.speedScale);

    // ================================
    // 7. Confidence Scaling
    // ================================
    score *= (clampf(rt.confidence, 0.0f, 1.0f) *
              t.scoring.confidenceWeight);

    // ================================
    // 8. Final Safety Clamp
    // ================================
    if (!isfinite(score) || score < 0.0f)
        return 0.0f;

    return clampf(score, 0.0f, 10000.0f);
}

inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi
                                    : v;
}

inline float computeNoiseFactor(const UltrasonicRuntime &rt,
                                const UltrasonicTuning &t)
{
    const auto &n = t.noise;

    // Hard invalid cases
    if (rt.timeout)
        return clampf(n.timeoutNoise, 0.0f, 1.0f);

    // If distance is invalid, treat as fully noisy
    if (!(rt.distance > 0.0f) ||
        rt.distance < n.minValidDistance ||
        rt.distance > n.maxValidDistance)
        return 1.0f;

    float factor = 0.0f;

    // -------- Near-min zone (soft degradation) --------
    if (n.nearMinZoneWidth > 0.0f)
    {
        const float minZoneEnd = n.minValidDistance + n.nearMinZoneWidth;
        if (rt.distance < minZoneEnd)
        {
            float ratio = (minZoneEnd - rt.distance) / n.nearMinZoneWidth;
            ratio = clampf(ratio, 0.0f, 1.0f);
            factor += ratio * n.nearMinPenalty;
        }
    }

    // -------- Near-max zone (soft degradation) --------
    if (n.nearMaxZoneWidth > 0.0f)
    {
        const float maxZoneStart = n.maxValidDistance - n.nearMaxZoneWidth;
        if (rt.distance > maxZoneStart)
        {
            float ratio = (rt.distance - maxZoneStart) / n.nearMaxZoneWidth;
            ratio = clampf(ratio, 0.0f, 1.0f);
            factor += ratio * n.nearMaxPenalty;
        }
    }

    return clampf(factor, 0.0f, 1.0f);
}

inline float computeConfidence(const UltrasonicRuntime &rt,
                               const UltrasonicTuning &t)
{
    // confidence = 1 - noise
    const float noise = computeNoiseFactor(rt, t);
    return 1.0f - noise;
}

inline bool isNoisy(const UltrasonicRuntime &rt,
                    const UltrasonicTuning &t)
{
    return computeNoiseFactor(rt, t) > t.noise.threshold;
}

inline bool isSensorRelevant(float priority,
                             const UltrasonicTuning &t)
{
    return priority > t.relevance.threshold;
}

#include <optional>

struct UltrasonicSchedulerState
{
    uint32_t lastTriggerTimeUs = 0;
    UltrasonicSensorId lastSelected = UltrasonicSensorId::COUNT;
};

// Optional: small epsilon to avoid jitter tie flips
static constexpr float kScoreEpsilon = 1e-3f;

// Optional fairness: small boost based on time since last run
inline float fairnessBoost(uint32_t now,
                           uint32_t lastRun,
                           float scale = 0.001f) // tune
{
    const uint32_t dt = (now > lastRun) ? (now - lastRun) : 0;
    return clampf(dt * scale, 0.0f, 5.0f);
}

std::optional<UltrasonicSensorId>
findNextSensor(uint32_t nowMs,
               uint32_t nowUs,
               const MotionVector &motion,
               const UltrasonicConfig configs[],
               UltrasonicRuntime runtime[],
               UltrasonicState states[],
               const UltrasonicTuning &t,
               UltrasonicSchedulerState &sched,
               size_t N)
{
    // ================================
    // 0. Global Gap Enforcement
    // ================================
    if ((nowUs - sched.lastTriggerTimeUs) < t.timing.interSensorGapUs)
        return std::nullopt;

    int bestIndex = -1;
    float bestScore = 0.0f;

    const uint32_t STARVATION_LIMIT_MS = 200; // tune

    for (size_t i = 0; i < N; i++)
    {
        auto &rt = runtime[i];
        const auto &cfg = configs[i];
        const auto &st = states[i];

        // ================================
        // 1. Eligibility Checks
        // ================================
        if (rt.phase != UltrasonicSensorPhase::IDLE)
            continue;

        if (!rt.enabled || !rt.valid)
            continue;

        if ((nowMs - st.lastRun) < cfg.minIntervalMs)
            continue;

        const auto id = static_cast<UltrasonicSensorId>(i);

        // ================================
        // 2. Base Score
        // ================================
        float score = computeDynamicPriority(
            id, motion, cfg, rt, t);

        // ================================
        // 🔥 3. Dynamic Hysteresis
        // ================================
        if (id == sched.lastSelected)
        {
            float hysteresis =
                t.scheduling.hysteresisBoost *
                (1.0f + clampf(motion.speed, 0.0f, 5.0f));

            score += hysteresis;
        }

        // ================================
        // 🔥 4. Fairness Boost (clamped)
        // ================================
        float fairness = fairnessBoost(nowMs, st.lastRun);
        score += fairness;

        // ================================
        // 🔥 5. Starvation Override
        // ================================
        uint32_t waitTime = nowMs - st.lastRun;
        if (waitTime > STARVATION_LIMIT_MS)
        {
            score += clampf(waitTime * 0.1f, 0.0f, 20.0f); // strong override boost
        }

        // ================================
        // 6. Store for Debug
        // ================================
        rt.dynamicPriority = score;

        // ================================
        // 7. Relevance Filter
        // ================================
        if (!isSensorRelevant(score, t))
            continue;

        // ================================
        // 8. Selection Logic (stable)
        // ================================
        if (bestIndex == -1 ||
            score > bestScore + kScoreEpsilon)
        {
            bestIndex = i;
            bestScore = score;
        }
    }

    // ================================
    // 9. No Candidate Found
    // ================================
    if (bestIndex < 0)
        return std::nullopt;

    // ================================
    // 10. Update Hysteresis Anchor
    // ================================
    sched.lastSelected = static_cast<UltrasonicSensorId>(bestIndex);

    return static_cast<UltrasonicSensorId>(bestIndex);
}

//----------------------------------Usage Example-------------------------------------------------//
// ✅ Usage Example
auto next = findNextSensor(now, motion, configs, runtime, states, tuning, N);

if (next)
{
    UltrasonicSensorId id = *next;
    triggerSensor(id);
}

update()
{
    processQueue();     // ISR → runtime
    updatePriorities(); // compute scores
    runScheduler();     // pick best sensor
    triggerSensor();    // fire TRIG
}

//----------------------------------OLDer PRIORITY Version-------------------------------------------------//

void runScheduler(uint32_t now)
{
    // If current sensor still processing → skip
    if (runtime[currentSensor].phase != UltrasonicSensorPhase::IDLE)
        return;

    int next = findNextSensor(now);

    if (next < 0)
        return; // no sensor ready

    currentSensor = next;

    triggerSensor(currentSensor);

    runtime[currentSensor].phase = UltrasonicSensorPhase::WAITING;
    runtime[currentSensor].triggerTime = now;

    states[currentSensor].lastRun = now;
}

void processQueue()
{
    EchoEvent evt;

    while (xQueueReceive(queue, &evt, 0))
    {
        auto &rt = (*this)[evt.sensorId];

        rt.duration = evt.duration;
        rt.distance = evt.duration * 0.0343f / 2.0f;

        rt.echoReceived = true;
        rt.timeout = evt.timeout;
        rt.phase = UltrasonicSensorPhase::DONE;
    }
}

//----------------------------------Delay-------------------------------------------------//
// Instead of  delayMicroseconds(10); we'll use esp_rom_delay_us
#include "esp_rom_sys.h"

inline void triggerSensor(uint8_t pin)
{
    digitalWrite(pin, LOW);
    esp_rom_delay_us(2);

    digitalWrite(pin, HIGH);
    esp_rom_delay_us(10);

    digitalWrite(pin, LOW);
}

// Avoid repeated digitalWrite overhead: ✔ Faster than digitalWrite, ✔ Better for tight loops
gpio_set_level((gpio_num_t)pin, 0);
esp_rom_delay_us(2);

gpio_set_level((gpio_num_t)pin, 1);
esp_rom_delay_us(10);

gpio_set_level((gpio_num_t)pin, 0);

//----------------------------------Gap between to Triggers-------------------------------------------------//
// Never trigger sensors back-to-back without gap
// 👉 That’s FAR more important than microsecond precision here.
interSensorGapUs = 3000;

//----------------------------------IMP POINT-------------------------------------------------//
/// Moving the cache out of the driver (ISR side) and into the manager is the clean, production-grade design.
///  Latest Value Cache per sensor: This must be **lock-free + atomic enough
volatile UltrasonicEchoEvent latest[N]; // lock-free cache