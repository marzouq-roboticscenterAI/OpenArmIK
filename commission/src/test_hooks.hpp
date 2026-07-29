#ifndef OPENARM_COMMISSION_TEST_HOOKS_HPP
#define OPENARM_COMMISSION_TEST_HOOKS_HPP

namespace openarm::commission::test {

void fail_next_allocation() noexcept;
void throw_next_exception() noexcept;

}  // namespace openarm::commission::test

#endif
