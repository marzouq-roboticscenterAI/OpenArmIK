/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_SOCKETCAN_BACKEND_INTERNAL_HPP
#define OPENARM_SOCKETCAN_BACKEND_INTERNAL_HPP

#include "openarm_transport.h"

#include <array>
#include <cstddef>

namespace openarm::transport {

struct LinkTransitionBatch {
    std::array<bool, 64> states{};
    std::size_t count{};
};

oa_transport_status parseLinkDatagram(const void *data, std::size_t length,
                                      unsigned int ifindex,
                                      LinkTransitionBatch &out_batch) noexcept;

bool socketCanBackendPermitsAuthorityIssuance() noexcept;
bool netlinkReceiveWasTruncated(std::size_t received, std::size_t capacity,
                                int message_flags) noexcept;

} // namespace openarm::transport

#endif
