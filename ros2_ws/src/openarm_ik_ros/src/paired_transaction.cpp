// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/paired_transaction.hpp"

#include <cmath>
#include <cstring>
#include <utility>

namespace openarm_ik_ros
{
namespace
{
constexpr const char * kWorldFrame = "world";
constexpr std::int64_t kMaximumFutureSkewNanoseconds = 1000000000LL;
}  // namespace

PairedTransactionProcessor::PairedTransactionProcessor(const std::int64_t expiry_nanoseconds)
: expiry_nanoseconds_(expiry_nanoseconds)
{
}

TransactionResult PairedTransactionProcessor::process(
  const PairedTarget & request, const std::int64_t now_nanoseconds)
{
  TransactionResult result;
  result.sequence = ++sequence_;
  initialize_diagnostics(&result.left);
  initialize_diagnostics(&result.right);

  if (request.pose_count != 2U) {
    result.reason = "expected_exactly_two_poses";
    return result;
  }
  if (request.frame_id != kWorldFrame) {
    result.reason = "frame_must_be_world";
    return result;
  }
  if (request.stamp_nanoseconds <= 0) {
    result.reason = "missing_stamp";
    return result;
  }
  if (now_nanoseconds - request.stamp_nanoseconds > expiry_nanoseconds_) {
    result.reason = "stale_request";
    return result;
  }
  if (request.stamp_nanoseconds - now_nanoseconds > kMaximumFutureSkewNanoseconds) {
    result.reason = "future_request";
    return result;
  }
  if (!finite_target(request.left) || !finite_target(request.right)) {
    result.reason = "nonfinite_xyz";
    return result;
  }

  const auto left_options = options_for(left_q_);
  const auto right_options = options_for(right_q_);
  const oa_status left_status = oa_ik_position_v2(
    oa_model_left_v10_bimanual(), request.left.data(), &left_options,
    OA_IK_DIAGNOSTICS_VERSION, OA_IK_DIAGNOSTICS_SIZE, &result.left);
  const oa_status right_status = oa_ik_position_v2(
    oa_model_right_v10_bimanual(), request.right.data(), &right_options,
    OA_IK_DIAGNOSTICS_VERSION, OA_IK_DIAGNOSTICS_SIZE, &result.right);
  if (left_status != OA_OK || right_status != OA_OK) {
    result.reason = "position_ik_failed";
    return result;
  }

  std::array<double, OA_DOF> next_left{};
  std::array<double, OA_DOF> next_right{};
  for (std::size_t index = 0; index < OA_DOF; ++index) {
    next_left[index] = result.left.q[index];
    next_right[index] = result.right.q[index];
  }
  left_q_ = next_left;
  right_q_ = next_right;
  result.committed = true;
  result.reason = "committed";
  return result;
}

const std::array<double, OA_DOF> & PairedTransactionProcessor::left_q() const noexcept
{
  return left_q_;
}

const std::array<double, OA_DOF> & PairedTransactionProcessor::right_q() const noexcept
{
  return right_q_;
}

std::vector<std::string> PairedTransactionProcessor::joint_names() const
{
  std::vector<std::string> names;
  names.reserve(16U);
  for (std::size_t index = 0; index < OA_DOF; ++index) {
    names.emplace_back(oa_model_joint_name(oa_model_left_v10_bimanual(), index));
  }
  for (std::size_t index = 0; index < OA_DOF; ++index) {
    names.emplace_back(oa_model_joint_name(oa_model_right_v10_bimanual(), index));
  }
  names.emplace_back("openarm_left_finger_joint1");
  names.emplace_back("openarm_right_finger_joint1");
  return names;
}

bool PairedTransactionProcessor::finite_target(const std::array<double, 3> & target) noexcept
{
  return std::isfinite(target[0]) && std::isfinite(target[1]) && std::isfinite(target[2]);
}

oa_ik_options PairedTransactionProcessor::options_for(
  const std::array<double, OA_DOF> & seed) noexcept
{
  oa_ik_options options{};
  options.abi_version = OA_MODEL_ABI_VERSION;
  options.struct_size = OA_IK_OPTIONS_REQUIRED_SIZE;
  options.position_tolerance_m = 1e-4;
  options.max_joint_step_rad = 0.15;
  options.damping_min = 1e-4;
  options.damping_max = 0.3;
  options.max_iterations = 300U;
  for (std::size_t index = 0; index < OA_DOF; ++index) {
    options.seed[index] = seed[index];
    options.posture[index] = seed[index];
    options.posture_weight[index] = 1.0;
  }
  return options;
}

void PairedTransactionProcessor::initialize_diagnostics(oa_ik_diagnostics * diagnostics) noexcept
{
  std::memset(diagnostics, 0, sizeof(*diagnostics));
  diagnostics->abi_version = OA_MODEL_ABI_VERSION;
  diagnostics->struct_size = OA_IK_DIAGNOSTICS_SIZE;
  diagnostics->status = OA_EINVAL;
}

}  // namespace openarm_ik_ros
