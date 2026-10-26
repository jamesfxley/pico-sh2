// SPDX-License-Identifier: MIT
#pragma once

/// \file sensor_event.hpp
/// Decoded, unit-converted sensor event. Each in-scope report ID decodes to a
/// SensorEvent: a common header (which sensor, its per-sensor sequence,
/// accuracy, and the raw report delay) plus the sample, a variant whose
/// alternative is the event's kind(). Fixed-point wire fields are converted by
/// the report's Q point (SH-2 Reference Manual, section 6.5) before they reach
/// this type.
///
/// Read a sample through the variant, never by assuming its kind:
///
///     if (const auto* rotation = std::get_if<RotationSample>(&event.payload))
///     {
///         use(rotation->unit);
///     }
///
/// Every alternative is trivially copyable, so SensorEvent is too: it is
/// produced on the stack by the decoder and handed to the sink by value, never
/// heap-allocated.

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

/// A unit quaternion. `real` is the scalar (w) component; i/j/k are the vector
/// components, matching the wire order i, j, k, real.
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

/// Compact gyro-integrated rotation vector (0x2A), delivered alone on the
/// gyro-RV input channel: unit quaternion plus the angular velocity used to
/// integrate it, in rad/s.
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

/// Raw sensor report (0x14/0x15/0x16): unscaled ADC counts and the sensor's own
/// microsecond timestamp; no Q-point conversion applies.
struct RawSample {
    std::int16_t x;           ///< X axis, ADC counts.
    std::int16_t y;           ///< Y axis, ADC counts.
    std::int16_t z;           ///< Z axis, ADC counts.
    std::uint32_t timestamp;  ///< The sensor's own microsecond timestamp.
};

/// What a SensorEvent carries: the index of its payload's alternative, named.
enum class EventKind : std::uint8_t {
    NONE = 0,         ///< No sample: std::monostate.
    VECTOR,           ///< Vec3: accelerometer, calibrated gyro/mag, linear
                      ///< acceleration, gravity.
    UNCALIBRATED,     ///< UncalibratedSample: uncalibrated gyro or
                      ///< magnetometer, value and bias.
    ROTATION,         ///< RotationSample: rotation vector with an accuracy
                      ///< estimate.
    GAME_ROTATION,    ///< Quaternion: rotation vector without one.
    GYRO_INTEGRATED,  ///< GyroIntegratedSample: compact gyro-integrated
                      ///< rotation vector.
    TAP,              ///< TapSample: tap detector.
    STEP_COUNT,       ///< StepCountSample: step counter.
    STEP_DETECT,      ///< StepDetectSample: step detector.
    STABILITY,        ///< StabilitySample: stability classifier.
    RAW,              ///< RawSample: raw accelerometer, gyroscope or
                      ///< magnetometer.
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
    /// Absolute event time in microseconds, reconstructed from the HINTN edge,
    /// the cargo's base/rebase timestamps, and this report's delay (SR-4). Set
    /// during the cargo walk; zero on a directly-decoded report.
    std::uint64_t event_us{};
    /// True when the cargo's base delta was the reserved overflow value, so
    /// `event_us` fell back to the HINTN edge rather than a wrong absolute
    /// time.
    bool timebase_overflow{};
    /// The sample itself. std::get_if on it returns null for any type but the
    /// one the report decoded to.
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

/// Whether a payload holding a `Sample` reports `kind`. kind() is the payload's
/// index, so the enumeration and the variant must list the same samples in the
/// same order; each type appears once, so its index is unambiguous.
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

/// Handed to the sink by value and copied into consumers' queues.
static_assert(std::is_trivially_copyable_v<SensorEvent>);

}  // namespace pico_sh2::reports
