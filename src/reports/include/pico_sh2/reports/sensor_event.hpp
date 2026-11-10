// SPDX-License-Identifier: MIT
#pragma once

/// \file sensor_event.hpp
/// Typed events with fixed-point fields converted to engineering units
/// (1000-3625 §6.5). The sample variant and kind() identify its type; events
/// are trivially copyable.

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <variant>

#include "pico_sh2/reports/report_layout.hpp"
#include "pico_sh2/reports/sensor_id.hpp"

namespace pico_sh2::reports {

/// A three-axis sample in the report's engineering units (m/s^2, rad/s, uT).
struct Vec3 {
    float x;  ///< X axis.
    float y;  ///< Y axis.
    float z;  ///< Z axis.
};

/// Unit quaternion in wire order i, j, k, real (scalar w).
struct Quaternion {
    float i;     ///< Vector component about X.
    float j;     ///< Vector component about Y.
    float k;     ///< Vector component about Z.
    float real;  ///< Scalar (w) component.
};

/// Uncalibrated three-axis report (gyro 0x07, mag 0x0F): the corrected sample
/// plus the bias that was removed.
struct UncalibratedSample {
    Vec3 value;  ///< The corrected sample.
    Vec3 bias;   ///< The bias removed to obtain it.
};

/// Rotation vector with a heading-accuracy estimate in radians (0x05, 0x09,
/// 0x28).
struct RotationSample {
    Quaternion unit;     ///< The orientation.
    float accuracy_rad;  ///< Heading-accuracy estimate, radians.
};

/// Compact gyro quaternion and integrating angular velocity in rad/s (0x2A).
struct GyroIntegratedSample {
    Quaternion unit;        ///< The integrated orientation.
    Vec3 angular_velocity;  ///< Angular velocity it was integrated from, rad/s.
};

/// Tap detector (0x10): a bit field of detected single/double taps per axis.
struct TapSample {
    /// Bit field of detected single and double taps, one group per axis.
    std::uint8_t taps;
};

/// Step counter (0x11): cumulative step count and the detection latency.
struct StepCountSample {
    std::uint32_t latency_us;  ///< Detection latency, microseconds.
    std::uint16_t steps;       ///< Cumulative step count.
};

/// Step detector (0x18): a single detected step with its latency.
struct StepDetectSample {
    std::uint32_t latency_us;  ///< Detection latency, microseconds.
};

/// Stability classifier (0x13): the classification enum value on the wire.
struct StabilitySample {
    std::uint8_t classification;  ///< Classification enum value as sent.
};

/// Raw ADC counts and sensor timestamp in microseconds; no Q conversion.
struct RawSample {
    std::int16_t x;           ///< X axis, ADC counts.
    std::int16_t y;           ///< Y axis, ADC counts.
    std::int16_t z;           ///< Z axis, ADC counts.
    std::uint32_t timestamp;  ///< The sensor's own microsecond timestamp.
};

/// What a SensorEvent carries: the index of its payload's alternative, named.
enum class EventKind : std::uint8_t {
    /// No sample: std::monostate.
    NONE = 0,
    /// Vec3: accelerometer, calibrated gyro/mag, linear acceleration, gravity.
    VECTOR,
    /// UncalibratedSample: uncalibrated gyro or magnetometer, value and bias.
    UNCALIBRATED,
    /// RotationSample: rotation vector with an accuracy estimate.
    ROTATION,
    /// Quaternion: rotation vector without an accuracy estimate.
    GAME_ROTATION,
    /// GyroIntegratedSample: compact gyro-integrated rotation vector.
    GYRO_INTEGRATED,
    /// TapSample: tap detector.
    TAP,
    /// StepCountSample: step counter.
    STEP_COUNT,
    /// StepDetectSample: step detector.
    STEP_DETECT,
    /// StabilitySample: stability classifier.
    STABILITY,
    /// RawSample: raw accelerometer, gyroscope or magnetometer.
    RAW,
};

/// The sample a SensorEvent carries, in EventKind order: alternative N is the
/// sample of kind N.
using EventPayload =
    std::variant<std::monostate, Vec3, UncalibratedSample, RotationSample,
                 Quaternion, GyroIntegratedSample, TapSample, StepCountSample,
                 StepDetectSample, StabilitySample, RawSample>;

/// One decoded report: the common header plus the sample.
struct SensorEvent {
    SensorId sensor{};        ///< Which sensor produced the report.
    std::uint8_t sequence{};  ///< The sensor's own report sequence number.
    /// The hub's confidence in this sample.
    SensorAccuracy accuracy{SensorAccuracy::UNRELIABLE};
    /// Report delay as sent, in units of 100 us.
    std::uint16_t delay_100us{};
    /// Event time in microseconds from the lead edge, base/rebases and delay
    /// (SR-4). Set by the cargo walk; zero on direct decode.
    std::uint64_t event_us{};
    /// True for reserved Base Delta; event_us falls back to the HINTN edge.
    bool timebase_overflow{};
    /// Decoded sample; std::get_if returns null for a different type.
    EventPayload payload;

    /// Which sample `payload` holds.
    [[nodiscard]] constexpr EventKind kind() const noexcept {
        return static_cast<EventKind>(payload.index());
    }
};

/// The payload index that holds a sample of `kind`.
constexpr std::size_t payload_index(EventKind kind) noexcept {
    return static_cast<std::size_t>(kind);
}

/// True when Sample occupies kind; enum and variant must have identical order.
template <typename Sample>
constexpr bool kind_holds(EventKind kind) noexcept {
    return EventPayload{std::in_place_type<Sample>}.index() ==
           payload_index(kind);
}

static_assert(std::variant_size_v<EventPayload> ==
              payload_index(EventKind::RAW) + 1);
static_assert(kind_holds<std::monostate>(EventKind::NONE));
static_assert(kind_holds<Vec3>(EventKind::VECTOR));
static_assert(kind_holds<UncalibratedSample>(EventKind::UNCALIBRATED));
static_assert(kind_holds<RotationSample>(EventKind::ROTATION));
static_assert(kind_holds<Quaternion>(EventKind::GAME_ROTATION));
static_assert(kind_holds<GyroIntegratedSample>(EventKind::GYRO_INTEGRATED));
static_assert(kind_holds<TapSample>(EventKind::TAP));
static_assert(kind_holds<StepCountSample>(EventKind::STEP_COUNT));
static_assert(kind_holds<StepDetectSample>(EventKind::STEP_DETECT));
static_assert(kind_holds<StabilitySample>(EventKind::STABILITY));
static_assert(kind_holds<RawSample>(EventKind::RAW));

/// Events can be copied into consumer queues without allocation.
static_assert(std::is_trivially_copyable_v<SensorEvent>);

}  // namespace pico_sh2::reports
