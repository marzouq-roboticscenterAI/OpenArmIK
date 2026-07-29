// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__PAIRED_TRANSACTION_HPP_
#define OPENARM_IK_ROS__PAIRED_TRANSACTION_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include "openarm_model.h"
}

namespace openarm_ik_ros
{

inline constexpr const char * kContinuityPolicy = "continuity-v1";
inline constexpr std::int64_t kMinimumExpiryMilliseconds = 1LL;
inline constexpr std::int64_t kMaximumExpiryMilliseconds = 60000LL;
inline constexpr std::int64_t kMaximumFutureSkewNanoseconds = 1000000000LL;

struct PairedTarget
{
  std::size_t pose_count{};
  std::string frame_id;
  std::int64_t stamp_nanoseconds{};
  std::array<double, 3> left{};
  std::array<double, 3> right{};
};

struct TransactionResult
{
  std::uint64_t sequence{};
  bool committed{};
  bool achieved_available{};
  std::string reason;
  oa_ik_diagnostics left{};
  oa_ik_diagnostics right{};
};

class PairedTransactionProcessor
{
public:
  explicit PairedTransactionProcessor(std::int64_t expiry_nanoseconds);

  static bool valid_expiry_nanoseconds(std::int64_t expiry_nanoseconds) noexcept;
  TransactionResult process(const PairedTarget & request, std::int64_t now_nanoseconds);
  const std::array<double, OA_DOF> & left_q() const noexcept;
  const std::array<double, OA_DOF> & right_q() const noexcept;
  std::vector<std::string> joint_names() const;
  static const char * continuity_policy() noexcept;

private:
  static bool finite_target(const std::array<double, 3> & target) noexcept;
  static oa_ik_options options_for(const std::array<double, OA_DOF> & seed) noexcept;
  static void initialize_diagnostics(oa_ik_diagnostics * diagnostics) noexcept;

  std::int64_t expiry_nanoseconds_;
  std::uint64_t sequence_{};
  std::array<double, OA_DOF> left_q_{};
  std::array<double, OA_DOF> right_q_{};
};

}  // namespace openarm_ik_ros

#endif  // OPENARM_IK_ROS__PAIRED_TRANSACTION_HPP_
