#ifndef OPENARM_COMMISSION_TEST_HOOKS_HPP
#define OPENARM_COMMISSION_TEST_HOOKS_HPP

#include <cstddef>

namespace openarm::commission::test {

void fail_next_allocation() noexcept;
void throw_next_exception() noexcept;
std::size_t active_handle_count() noexcept;
void exhaust_handle_tokens() noexcept;

}  // namespace openarm::commission::test

#endif
