/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_transport.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

#ifdef __linux__
#include <climits>
#include <net/if.h>
#include <unistd.h>
#endif

int main() {
#ifdef __linux__
    if (::if_nametoindex("vcan0") == 0U) {
        std::cout << "SKIP: vcan0 is unavailable\n";
        return EXIT_SUCCESS;
    }
    char resolved[PATH_MAX]{};
    if (::realpath("/sys/class/net/vcan0", resolved) == nullptr ||
        std::strstr(resolved, "/virtual/") == nullptr) {
        std::cout << "SKIP: vcan0 is not a verified virtual interface\n";
        return EXIT_SUCCESS;
    }
    oa_transport *transport = nullptr;
    const auto status =
        oa_transport_open("vcan0", nullptr, nullptr, 0U, &transport);
    if (status != OA_TRANSPORT_OK) {
        std::cerr << "vcan0 open failed: " << status << '\n';
        return EXIT_FAILURE;
    }
    std::uint64_t now_ns = 0U;
    if (oa_transport_now_monotonic_ns(&now_ns) != OA_TRANSPORT_OK) {
        oa_transport_destroy(transport);
        return EXIT_FAILURE;
    }
    oa_transport_event event{};
    event.struct_size = sizeof(event);
    event.abi_version = OA_TRANSPORT_ABI_VERSION;
    const auto receive_status =
        oa_transport_receive(transport, now_ns + UINT64_C(1000000000), &event);
    if (receive_status != OA_TRANSPORT_OK ||
        event.kind != OA_TRANSPORT_EVENT_LINK) {
        std::cerr << "vcan0 initial link event failed: " << receive_status << '\n';
        oa_transport_destroy(transport);
        return EXIT_FAILURE;
    }
    oa_transport_destroy(transport);
    std::cout << "vcan0 open/close passed without transmitting\n";
    return EXIT_SUCCESS;
#else
    std::cout << "SKIP: SocketCAN is Linux-only\n";
    return EXIT_SUCCESS;
#endif
}
