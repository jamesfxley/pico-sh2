// SPDX-License-Identifier: MIT
#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <system_error>

#include "pico/stdio.h"
#include "pico_family/device_test.hpp"
#include "pico_sh2/sh2/hal_transport.hpp"
#include "pico_sh2/sh2/session.hpp"
#include "pico_sh2/util/result.hpp"

namespace {

class LocalTransport final : public pico_sh2::IHalTransport {
   public:
    bool data_ready() override { return false; }

    pico_sh2::Result<pico_sh2::ReceivedTransfer> read_transfer(
        std::span<std::uint8_t> /*dst*/) override {
        return pico_sh2::Sh2Error::BUS_ERROR;
    }

    pico_sh2::Result<void> write_transfer(
        std::span<const std::uint8_t> /*frame*/) override {
        return pico_sh2::Sh2Error::BUS_ERROR;
    }

    std::uint64_t now_us() override { return 0; }

    void set_reset(bool /*asserted*/) override {}
};

struct Field {
    std::string_view name;
    std::size_t pico_sh2::SessionBinding::* member;
};

constexpr std::array FIELDS{
    Field{.name = "max_rx_transfer_len",
          .member = &pico_sh2::SessionBinding::max_rx_transfer_len},
    Field{.name = "max_cargo_len",
          .member = &pico_sh2::SessionBinding::max_cargo_len},
    Field{.name = "max_tx_cargo",
          .member = &pico_sh2::SessionBinding::max_tx_cargo},
    Field{.name = "tx_queue_depth",
          .member = &pico_sh2::SessionBinding::tx_queue_depth},
    Field{.name = "frs_max_words",
          .member = &pico_sh2::SessionBinding::frs_max_words},
    Field{.name = "max_enabled_sensors",
          .member = &pico_sh2::SessionBinding::max_enabled_sensors},
    Field{.name = "session_size",
          .member = &pico_sh2::SessionBinding::session_size},
    Field{.name = "session_align",
          .member = &pico_sh2::SessionBinding::session_align},
};

// AR-5: compare before construction, which aborts on a binding mismatch.
void binding_case(pico_family::device_test::Run& run) {
    const auto caller = pico_sh2::Session::caller_binding();
    const auto library = pico_sh2::library_binding();
    for (const auto& field : FIELDS) {
        if (caller.*field.member != library.*field.member) {
            std::array<char, 128> detail{};
            auto* next = detail.data();
            const auto append = [&detail, &next](std::string_view text) {
                const auto remaining = static_cast<std::size_t>(
                    detail.data() + detail.size() - next);
                next = std::ranges::copy(text.substr(0, remaining), next).out;
            };
            append(field.name);
            append(" caller=");
            const auto caller_number = std::to_chars(
                next, detail.data() + detail.size(), caller.*field.member);
            if (caller_number.ec != std::errc{}) {
                run.check("AR-5", "target-consumer-matches-session-binding",
                          false, "binding detail conversion failed");
                return;
            }
            next = caller_number.ptr;
            append(" library=");
            const auto library_number = std::to_chars(
                next, detail.data() + detail.size(), library.*field.member);
            if (caller_number.ec != std::errc{} ||
                library_number.ec != std::errc{}) {
                run.check("AR-5", "target-consumer-matches-session-binding",
                          false, "binding detail conversion failed");
                return;
            }
            run.check("AR-5", "target-consumer-matches-session-binding", false,
                      std::string_view{detail.data(), static_cast<std::size_t>(
                                                          library_number.ptr -
                                                          detail.data())});
            return;
        }
    }
    static LocalTransport transport;
    static const pico_sh2::Session SESSION{transport, {}};
    run.check("AR-5", "target-consumer-matches-session-binding", true, "");
}

}  // namespace

int main() {
    stdio_init_all();
    pico_family::device_test::Run run;
    binding_case(run);
    run.finish();
}
