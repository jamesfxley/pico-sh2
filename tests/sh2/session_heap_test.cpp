// SPDX-License-Identifier: MIT
// No allocation after begin(). Only this translation unit defines the global
// operator new override; a second definition among the linked suites would be
// a duplicate (AR-1).
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "pico_sh2/sh2/command.hpp"
#include "pico_sh2/sh2/frs.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/testing/fake_hub.hpp"
#include "pico_sh2/testing/frame_builder.hpp"
#include "session_fixture.hpp"

// ASan supplies operator new/delete, so allocation counting runs only
// in the plain build; sanitiser checks do not establish zero allocation (AR-1).
#if defined(__SANITIZE_ADDRESS__)
#define PICO_SH2_TEST_ASAN
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
#define PICO_SH2_TEST_ASAN
#endif
#endif

// Only the allocation-counting override uses these headers.
#ifndef PICO_SH2_TEST_ASAN
#include <cstdlib>
#include <new>
#endif

// The counter tallies only while armed (AR-1). It is at file scope so the
// global operator new below can reach it.
namespace ar1 {
namespace {
std::size_t alloc_count = 0;
bool counting = false;
}  // namespace
}  // namespace ar1

#ifdef PICO_SH2_TEST_ASAN
// Allocation-counting tests compile under ASan but skip before exercising code.
constexpr bool COUNTING_AVAILABLE = false;
#else
constexpr bool COUNTING_AVAILABLE = true;

void* operator new(std::size_t bytes) {
    if (ar1::counting) {
        ++ar1::alloc_count;
    }
    void* block = std::malloc(bytes == 0 ? 1 : bytes);
    if (block == nullptr) {
        throw std::bad_alloc();
    }
    return block;
}

void operator delete(void* block) noexcept { std::free(block); }

void operator delete(void* block, [[maybe_unused]] std::size_t size) noexcept {
    std::free(block);
}

void* operator new[](std::size_t bytes) { return ::operator new(bytes); }

void operator delete[](void* memory) noexcept { ::operator delete(memory); }

void operator delete[](void* memory,
                       [[maybe_unused]] std::size_t size) noexcept {
    ::operator delete(memory);
}

void* operator new(std::size_t bytes, std::align_val_t alignment) {
    if (ar1::counting) {
        ++ar1::alloc_count;
    }
    const auto align = static_cast<std::size_t>(alignment);
    if (bytes > SIZE_MAX - align + 1U) {
        throw std::bad_alloc();
    }
    const auto rounded = ((bytes == 0 ? 1 : bytes) + align - 1) / align * align;
    void* memory = ::aligned_alloc(align, rounded);
    if (memory == nullptr) {
        throw std::bad_alloc();
    }
    return memory;
}

void* operator new[](std::size_t bytes, std::align_val_t alignment) {
    return ::operator new(bytes, alignment);
}

void operator delete(void* memory,
                     [[maybe_unused]] std::align_val_t alignment) noexcept {
    std::free(memory);
}

void operator delete[](void* memory, std::align_val_t alignment) noexcept {
    ::operator delete(memory, alignment);
}

void operator delete(void* memory, [[maybe_unused]] std::size_t size,
                     std::align_val_t alignment) noexcept {
    ::operator delete(memory, alignment);
}

void operator delete[](void* memory, [[maybe_unused]] std::size_t size,
                       std::align_val_t alignment) noexcept {
    ::operator delete(memory, alignment);
}
#endif  // PICO_SH2_TEST_ASAN

namespace {

using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::make_session;
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::frame;

using pico_sh2::Session;

// The counter must detect a non-elidable allocation as a negative control
// (AR-1).
TEST(Session, AllocationCounterIsLive) {
    if (!COUNTING_AVAILABLE) {
        GTEST_SKIP() << "allocation counter is unavailable under ASan";
    }
    ar1::alloc_count = 0;
    ar1::counting = true;
    void* memory = ::operator new(16);
    ::operator delete(memory);
    ar1::counting = false;
    EXPECT_GE(ar1::alloc_count, 1U);
}

TEST(Session, NoHeapAllocationAfterBegin) {
    if (!COUNTING_AVAILABLE) {
        GTEST_SKIP() << "allocation-counting override is disabled under ASan; "
                        "the no-heap invariant is covered by the plain build";
    }
    FakeHub hub;
    Session session = make_session(hub);
    // The event sink is left unset (a no-op), so event delivery does not
    // allocate inside the armed region.
    drive_to_ready(hub, session);

    // The fake allocates while queuing; only draining and decoding are
    // measured.
    std::vector<std::uint8_t> payload = {0xFB, 0x0A, 0x00, 0x00, 0x00,
                                         0x01, 0x00, 0x00, 0x05, 0x00,
                                         0x01, 0x00, 0x02, 0x00, 0xFF};
    for (std::uint8_t i = 0; i < 32; ++i) {
        hub.queue_transfer(frame(3, i, payload),
                           static_cast<std::uint64_t>(i) * 1000U);
    }

    ar1::alloc_count = 0;
    ar1::counting = true;
    session.service();  // drains + decodes all queued input, heap-free (AR-1)
    ar1::counting = false;
    EXPECT_EQ(ar1::alloc_count, 0U);
    EXPECT_EQ(session.counters().events_decoded, 32U);
}

// Control replies are drained with the counter armed (AR-1).
// Requests are sent before arming because FakeHub records writes in a vector.
TEST(Session, NoHeapAllocationHandlingControlReplies) {
    if (!COUNTING_AVAILABLE) {
        GTEST_SKIP() << "allocation-counting override is disabled under ASan; "
                        "the no-heap invariant is covered by the plain build";
    }
    FakeHub hub;
    Session session = make_session(hub);
    drive_to_ready(hub, session);
    ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
    session.service();
    ASSERT_TRUE(session.frs_read(pico_sh2::frs_type::SYSTEM_ORIENTATION).ok());
    session.service();

    hub.queue_transfer(
        frame(2, 1, pico_sh2::testing::feature_response(0x05, 5000)), 0);
    hub.queue_transfer(frame(2, 2,
                             pico_sh2::testing::command_response(
                                 pico_sh2::command::COUNTER, 0, false)),
                       0);
    hub.queue_transfer(
        frame(2, 3,
              pico_sh2::testing::frs_read_response(
                  pico_sh2::frs_type::SYSTEM_ORIENTATION, 3, 1, 0, 1, 0)),
        0);
    hub.queue_transfer(frame(2, 4, pico_sh2::testing::flush_completed(0x05)),
                       0);

    ar1::alloc_count = 0;
    ar1::counting = true;
    session.service();
    ar1::counting = false;
    EXPECT_EQ(ar1::alloc_count, 0U);
    EXPECT_EQ(session.counters().feature_responses, 1U);
    EXPECT_EQ(session.counters().command_completions, 1U);
    EXPECT_EQ(session.counters().frs_completions, 1U);
    EXPECT_EQ(session.counters().flushes_completed, 1U);
}

// A command timeout allocates nothing with the hook unset or bound;
// it fires inside the armed region in both cases (OR-2).
struct DiagTally {
    std::size_t timeouts = 0;

    void on_diag(pico_sh2::DiagEvent event) {
        if (event == pico_sh2::DiagEvent::COMMAND_TIMEOUT) {
            ++timeouts;
        }
    }
};

TEST(Session, NoHeapAllocationReportingADiagnostic) {
    if (!COUNTING_AVAILABLE) {
        GTEST_SKIP() << "allocation-counting override is disabled under ASan; "
                        "the no-heap invariant is covered by the plain build";
    }
    for (const bool bound : {false, true}) {
        FakeHub hub;
        Session session = make_session(hub);
        DiagTally tally;
        if (bound) {
            session.set_diag_sink(
                Session::DiagSink::bind<DiagTally, &DiagTally::on_diag>(
                    &tally));
        }
        drive_to_ready(hub, session);
        ASSERT_TRUE(session.send_command(pico_sh2::command::COUNTER).ok());
        session.service();
        hub.advance_time(500'001);

        ar1::alloc_count = 0;
        ar1::counting = true;
        session.service();
        ar1::counting = false;
        EXPECT_EQ(ar1::alloc_count, 0U) << "hook bound: " << bound;
        EXPECT_EQ(session.counters().command_timeouts, 1U)
            << "hook bound: " << bound;
        EXPECT_EQ(tally.timeouts, bound ? 1U : 0U);
    }
}

}  // namespace
