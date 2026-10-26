// SPDX-License-Identifier: MIT
// AR-1: no heap allocation on the steady-state path. This suite owns the
// global operator new override the check needs, so it lives alone in its own
// translation unit — defining operator new in more than one of the linked test
// objects would be a duplicate definition.
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

// Detect AddressSanitizer: it installs its own operator new/delete, so the
// AR-1 allocation-counting override below is compiled only for the plain build.
// Under ASan the same invariant is still guarded by leak/use-after-free checks.
#if defined(__SANITIZE_ADDRESS__)
#define PICO_SH2_TEST_ASAN
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
#define PICO_SH2_TEST_ASAN
#endif
#endif

// The override is the only user of these two headers, so they are included only
// where it is compiled.
#ifndef PICO_SH2_TEST_ASAN
#include <cstdlib>
#include <new>
#endif

// Allocation counter for the AR-1 (zero post-init heap) test. The overridden
// global operator new only tallies while armed, so it never perturbs the rest
// of the suite. Defined at global scope as operator new must be.
namespace ar1 {
namespace {
std::size_t alloc_count = 0;
bool counting = false;
}  // namespace
}  // namespace ar1

#ifdef PICO_SH2_TEST_ASAN
// The tests below still compile under ASan, so the code they exercise is
// analysed in every tree; they stop at their first line instead of counting.
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

// Named one at a time rather than with a using-directive: this suite
// reaches for a handful of helpers, and a budget line it does not need
// is budget spent for nothing. Each is named in the namespace that
// defines it.
using pico_sh2::sh2_test::drive_to_ready;
using pico_sh2::sh2_test::make_session;
using pico_sh2::testing::FakeHub;
using pico_sh2::testing::frame;

using pico_sh2::Session;

// AR-1: the counter must observe a deliberate non-elidable allocation.
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

    // Queue a batch of input cargoes. These allocations happen before the
    // counter is armed; the measured region only drains and decodes them.
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

// AR-1: the control path is steady state too. Each reply a ready session
// handles -- a Get Feature response, a command response, an FRS read
// response and a Flush Completed -- is drained with the counter armed. The
// requests go out before arming: the fake hub records writes in a vector,
// which is the test's allocation, not the session's.
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

// OR-2: reporting a diagnostic event allocates nothing, whether the hook is
// unset or bound. A command timeout is the event: it is raised inside the
// armed region, once with no hook and once with a hook that tallies it.
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
