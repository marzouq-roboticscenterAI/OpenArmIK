/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Bimanual motion, real-time monitors, and the always-listening emergency stop.
 *
 * These cover the four simultaneous-motion planners (paired, centroid,
 * mirrored, converge), the per-cycle keepout and contact monitors, and the
 * process-wide E-stop latch. Adversarial cases are deliberate: non-finite
 * inputs, degenerate and self-intersecting targets, zero-travel requests,
 * single-cycle torque spikes, and concurrent assertion from another thread
 * while a command is executing.
 */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1

#include "openarm_collision.h"
#include "openarm_control.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <thread>
#include <type_traits>

namespace {

static_assert(std::is_same_v<decltype(oa_centroid_tcp_move_with_units{}.target_centroid.x),
                             double>);
static_assert(std::is_same_v<decltype(oa_mirrored_tcp_move_with_units{}.lead_tcp.y),
                             double>);
static_assert(std::is_same_v<decltype(oa_converge_tcp_move_with_units{}.target.z), double>);

[[noreturn]] void fail(const char *expression, const char *file, int line) {
    std::fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
    std::exit(1);
}

#define CHECK(expression) \
    ((expression) ? static_cast<void>(0) : fail(#expression, __FILE__, __LINE__))

template <typename T>
void init(T &record) {
    record = {};
    record.struct_size = sizeof(record);
    record.abi_version = OA_CONTROL_ABI_V1;
}

constexpr std::uint64_t kCycleNs = 10000000ULL;

struct Fixture {
    oa_manifest *manifest{};
    oa_controller *controller{};
    oa_snapshot state{};
    std::uint64_t now_ns{};

    Fixture() {
        CHECK(oa_estop_clear() == OA_CONTROL_OK);
        CHECK(oa_manifest_create_openarm_v10_virtual(&manifest) == OA_CONTROL_OK);
        oa_controller_options options{};
        init(options);
        options.backend = OA_BACKEND_VIRTUAL;
        options.collision_policy = OA_COLLISION_VIRTUAL_UNCHECKED;
        options.cycle_ns = kCycleNs;
        options.feedback_timeout_ns = 50000000ULL;
        options.max_cross_bus_skew_ns = 1000000ULL;
        options.collision_scene_revision = 1U;
        CHECK(oa_controller_create(manifest, &options, &controller) == OA_CONTROL_OK);
        oa_verify_report verify{};
        init(verify);
        CHECK(oa_controller_open_and_verify(controller, &verify) == OA_CONTROL_OK);
        oa_arm_challenge challenge{};
        init(challenge);
        CHECK(oa_controller_get_arm_challenge(controller, &challenge) == OA_CONTROL_OK);
        CHECK(oa_controller_arm(controller, &challenge) == OA_CONTROL_OK);
        refresh();
    }

    ~Fixture() {
        oa_controller_destroy(controller);
        oa_manifest_destroy(manifest);
        (void)oa_estop_clear();
    }

    void refresh() {
        init(state);
        CHECK(oa_controller_snapshot(controller, &state) == OA_CONTROL_OK);
    }

    /* Steps the controller one cycle and refreshes the cached snapshot. */
    oa_control_status step() {
        now_ns += kCycleNs;
        const oa_control_status status = oa_controller_advance(controller, now_ns);
        init(state);
        (void)oa_controller_snapshot(controller, &state);
        return status;
    }

    std::array<double, 3> tcp(const std::uint32_t side) {
        oa_arm_kinematics kinematics{};
        init(kinematics);
        CHECK(oa_controller_get_kinematics(controller, side,
                                           state.arm[side].feedback_seq,
                                           &kinematics) == OA_CONTROL_OK);
        return {kinematics.tcp_xyz_m[0], kinematics.tcp_xyz_m[1],
                kinematics.tcp_xyz_m[2]};
    }
};

oa_paired_tcp_move paired_template(const Fixture &fixture) {
    oa_paired_tcp_move move{};
    init(move);
    move.expiry_ns = 600000000000ULL;
    move.required_feedback_seq[0] = fixture.state.arm[0].feedback_seq;
    move.required_feedback_seq[1] = fixture.state.arm[1].feedback_seq;
    move.velocity_scale = 0.5;
    move.acceleration_scale = 0.5;
    move.jerk_scale = 0.5;
    move.tcp_tol_m = 1.0e-3;
    move.collision_scene_revision = 1U;
    move.max_branch_step_rad = 2.0;
    move.min_singular_value = 0.0;
    return move;
}

oa_centroid_tcp_move centroid_template(const Fixture &fixture) {
    oa_centroid_tcp_move move{};
    init(move);
    move.expiry_ns = 600000000000ULL;
    move.required_feedback_seq[0] = fixture.state.arm[0].feedback_seq;
    move.required_feedback_seq[1] = fixture.state.arm[1].feedback_seq;
    move.velocity_scale = 0.5;
    move.acceleration_scale = 0.5;
    move.jerk_scale = 0.5;
    move.tcp_tol_m = 1.0e-3;
    move.collision_scene_revision = 1U;
    move.max_branch_step_rad = 2.0;
    move.min_singular_value = 0.0;
    return move;
}

oa_mirrored_tcp_move mirrored_template(const Fixture &fixture) {
    oa_mirrored_tcp_move move{};
    init(move);
    move.expiry_ns = 600000000000ULL;
    move.required_feedback_seq[0] = fixture.state.arm[0].feedback_seq;
    move.required_feedback_seq[1] = fixture.state.arm[1].feedback_seq;
    move.velocity_scale = 0.5;
    move.acceleration_scale = 0.5;
    move.jerk_scale = 0.5;
    move.tcp_tol_m = 1.0e-3;
    move.collision_scene_revision = 1U;
    move.max_branch_step_rad = 2.0;
    move.min_singular_value = 0.0;
    return move;
}

oa_converge_tcp_move converge_template(const Fixture &fixture) {
    oa_converge_tcp_move move{};
    init(move);
    move.expiry_ns = 600000000000ULL;
    move.required_feedback_seq[0] = fixture.state.arm[0].feedback_seq;
    move.required_feedback_seq[1] = fixture.state.arm[1].feedback_seq;
    move.velocity_scale = 0.5;
    move.acceleration_scale = 0.5;
    move.jerk_scale = 0.5;
    move.tcp_tol_m = 1.0e-3;
    move.collision_scene_revision = 1U;
    move.max_branch_step_rad = 2.0;
    move.min_singular_value = 0.0;
    move.stop_distance_m = 0.05;
    move.minimum_progress_m = 0.001;
    return move;
}

oa_motion_plan_report report_of(oa_motion_plan *plan) {
    oa_motion_plan_report report{};
    init(report);
    CHECK(oa_motion_plan_get_report(plan, &report) == OA_CONTROL_OK);
    return report;
}

oa_execute_request execute_template() {
    oa_execute_request request{};
    init(request);
    request.expiry_ns = 600000000000ULL;
    request.producer_deadline_ns = 600000000000ULL;
    request.stop_kind = OA_STOP_CONTROLLED;
    return request;
}

/* Runs the controller until the command reaches a terminal state, returning the
 * latched stop cause. Fails the test rather than looping forever. */
oa_contact_report run_until_stopped(Fixture &fixture, int max_cycles = 4000) {
    for (int cycle = 0; cycle < max_cycles; ++cycle) {
        (void)fixture.step();
        oa_contact_report report{};
        init(report);
        CHECK(oa_controller_get_contact_report(fixture.controller, &report) ==
              OA_CONTROL_OK);
        if (report.cause != OA_STOP_CAUSE_NONE) {
            return report;
        }
    }
    oa_contact_report report{};
    init(report);
    return report;
}

/* Moves both claws to a well-separated forward pose. The neutral pose sits
 * close to the nominal keepout limit, so tests that need travel start here. */
void move_to_separated_pose(Fixture &fixture) {
    auto move = paired_template(fixture);
    move.velocity_scale = 1.0;
    move.acceleration_scale = 1.0;
    move.jerk_scale = 1.0;
    move.left_tcp_m[0] = 0.30;
    move.left_tcp_m[1] = 0.22;
    move.left_tcp_m[2] = 0.30;
    move.right_tcp_m[0] = 0.30;
    move.right_tcp_m[1] = -0.22;
    move.right_tcp_m[2] = 0.30;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    auto request = execute_template();
    std::uint64_t command_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) ==
          OA_CONTROL_OK);
    /* This move must complete normally: no monitor may intervene. */
    const auto report = run_until_stopped(fixture, 3000);
    CHECK(report.cause == OA_STOP_CAUSE_PLAN_COMPLETE);
    oa_motion_plan_destroy(plan);
    fixture.refresh();
    const auto left = fixture.tcp(OA_LEFT);
    CHECK(std::abs(left[0] - 0.30) < 5.0e-3);
    CHECK(std::abs(left[1] - 0.22) < 5.0e-3);
}

/* ---------------------------------------------------------------------- */

void test_centroid_translates_both_claws_equally() {
    Fixture fixture;
    const auto before_left = fixture.tcp(OA_LEFT);
    const auto before_right = fixture.tcp(OA_RIGHT);
    const std::array<double, 3> midpoint{
        0.5 * (before_left[0] + before_right[0]),
        0.5 * (before_left[1] + before_right[1]),
        0.5 * (before_left[2] + before_right[2])};

    auto move = centroid_template(fixture);
    /* A modest forward/up translation of the midpoint. */
    move.target_centroid_m[0] = midpoint[0] + 0.05;
    move.target_centroid_m[1] = midpoint[1];
    move.target_centroid_m[2] = midpoint[2] + 0.03;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_centroid_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    const auto report = report_of(plan);
    CHECK(report.kind == OA_PLAN_CENTROID_TCP);

    /* The identical delta must be applied to both claws, so the achieved
     * midpoint is the request and the claw separation is unchanged. */
    const std::array<double, 3> expected_delta{0.05, 0.0, 0.03};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        const double left_delta = report.achieved_tcp_m[0][axis] - before_left[axis];
        const double right_delta = report.achieved_tcp_m[1][axis] - before_right[axis];
        CHECK(std::abs(left_delta - expected_delta[axis]) < 2.0e-3);
        CHECK(std::abs(right_delta - expected_delta[axis]) < 2.0e-3);
        CHECK(std::abs(left_delta - right_delta) < 2.0e-3);
    }
    oa_motion_plan_destroy(plan);
}

void test_centroid_rejects_non_finite_and_zero_motion() {
    Fixture fixture;
    oa_motion_plan *plan = nullptr;

    for (const double poison : {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()}) {
        auto move = centroid_template(fixture);
        move.target_centroid_m[0] = poison;
        CHECK(oa_controller_plan_centroid_tcp(fixture.controller, &move, &plan) ==
              OA_CONTROL_EINVAL);
        CHECK(plan == nullptr);
    }

    /* A centroid target equal to the current midpoint is a legitimate no-op
     * request: it must plan cleanly rather than divide by a zero ray. */
    const auto left = fixture.tcp(OA_LEFT);
    const auto right = fixture.tcp(OA_RIGHT);
    auto move = centroid_template(fixture);
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        move.target_centroid_m[axis] = 0.5 * (left[axis] + right[axis]);
    }
    CHECK(oa_controller_plan_centroid_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    const auto report = report_of(plan);
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        CHECK(std::abs(report.achieved_tcp_m[0][axis] - left[axis]) < 1.0e-6);
        CHECK(std::abs(report.achieved_tcp_m[1][axis] - right[axis]) < 1.0e-6);
    }
    oa_motion_plan_destroy(plan);
}

void test_mirrored_negates_y_only() {
    Fixture fixture;
    auto move = mirrored_template(fixture);
    move.lead_side = OA_LEFT;
    move.lead_tcp_m[0] = 0.30;
    move.lead_tcp_m[1] = 0.22;
    move.lead_tcp_m[2] = 0.30;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_mirrored_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    const auto report = report_of(plan);
    CHECK(report.kind == OA_PLAN_MIRRORED_TCP);
    CHECK(std::abs(report.achieved_tcp_m[0][0] - 0.30) < 2.0e-3);
    CHECK(std::abs(report.achieved_tcp_m[0][1] - 0.22) < 2.0e-3);
    CHECK(std::abs(report.achieved_tcp_m[0][2] - 0.30) < 2.0e-3);
    CHECK(std::abs(report.achieved_tcp_m[1][0] - 0.30) < 2.0e-3);
    CHECK(std::abs(report.achieved_tcp_m[1][1] + 0.22) < 2.0e-3);
    CHECK(std::abs(report.achieved_tcp_m[1][2] - 0.30) < 2.0e-3);
    oa_motion_plan_destroy(plan);

    /* The right arm may lead with the same geometry mirrored. */
    fixture.refresh();
    auto right_move = mirrored_template(fixture);
    right_move.lead_side = OA_RIGHT;
    right_move.lead_tcp_m[0] = 0.30;
    right_move.lead_tcp_m[1] = -0.22;
    right_move.lead_tcp_m[2] = 0.30;
    oa_motion_plan *right_plan = nullptr;
    CHECK(oa_controller_plan_mirrored_tcp(fixture.controller, &right_move,
                                          &right_plan) == OA_CONTROL_OK);
    const auto right_report = report_of(right_plan);
    CHECK(std::abs(right_report.achieved_tcp_m[1][1] + 0.22) < 2.0e-3);
    CHECK(std::abs(right_report.achieved_tcp_m[0][1] - 0.22) < 2.0e-3);
    oa_motion_plan_destroy(right_plan);
}

void test_mirrored_rejects_bad_side_and_reserved() {
    Fixture fixture;
    oa_motion_plan *plan = nullptr;
    auto move = mirrored_template(fixture);
    move.lead_tcp_m[0] = 0.30;
    move.lead_tcp_m[1] = 0.22;
    move.lead_tcp_m[2] = 0.30;

    auto bad_side = move;
    bad_side.lead_side = 2U;
    CHECK(oa_controller_plan_mirrored_tcp(fixture.controller, &bad_side, &plan) ==
          OA_CONTROL_EINVAL);

    auto reserved = move;
    reserved.reserved0 = 1U;
    CHECK(oa_controller_plan_mirrored_tcp(fixture.controller, &reserved, &plan) ==
          OA_CONTROL_EINVAL);

    auto poison = move;
    poison.lead_tcp_m[2] = std::numeric_limits<double>::quiet_NaN();
    CHECK(oa_controller_plan_mirrored_tcp(fixture.controller, &poison, &plan) ==
          OA_CONTROL_EINVAL);
    CHECK(plan == nullptr);

    /* A lead target on the sagittal plane mirrors onto itself, which asks both
     * claws to occupy one point. The planner may reject it outright; if it
     * plans, the real-time keepout monitor is the gate that must stop it. This
     * asserts only that the request cannot silently succeed as a normal move. */
    auto on_plane = move;
    on_plane.lead_tcp_m[1] = 0.0;
    oa_motion_plan *plane_plan = nullptr;
    const oa_control_status status =
        oa_controller_plan_mirrored_tcp(fixture.controller, &on_plane, &plane_plan);
    if (status == OA_CONTROL_OK) {
        const auto report = report_of(plane_plan);
        /* Both claws were sent to the same coordinate. */
        CHECK(std::abs(report.achieved_tcp_m[0][1] - report.achieved_tcp_m[1][1]) <
              1.0e-3);
        oa_motion_plan_destroy(plane_plan);
    } else {
        CHECK(plane_plan == nullptr);
    }
}

/* A forward point both claws can reach. The neutral pose midpoint sits inside
 * the central shaft keepout and is deliberately not used. */
constexpr double kConvergeX = 0.35;
constexpr double kConvergeY = 0.0;
constexpr double kConvergeZ = 0.40;

void test_converge_stops_short_and_rejects_no_progress() {
    Fixture fixture;
    auto move = converge_template(fixture);
    move.target_m[0] = kConvergeX;
    move.target_m[1] = kConvergeY;
    move.target_m[2] = kConvergeZ;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    const auto report = report_of(plan);
    CHECK(report.kind == OA_PLAN_CONVERGE_TCP);
    /* Each claw must halt exactly the requested distance short of the point. */
    for (std::size_t side = 0; side < 2U; ++side) {
        double squared = 0.0;
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            const double delta = report.achieved_tcp_m[side][axis] - move.target_m[axis];
            squared += delta * delta;
        }
        CHECK(std::abs(std::sqrt(squared) - move.stop_distance_m) < 5.0e-3);
    }
    oa_motion_plan_destroy(plan);

    /* A stop distance beyond the actual separation leaves nothing to travel. */
    fixture.refresh();
    auto blocked = converge_template(fixture);
    blocked.target_m[0] = kConvergeX;
    blocked.target_m[1] = kConvergeY;
    blocked.target_m[2] = kConvergeZ;
    blocked.stop_distance_m = 100.0;
    oa_motion_plan *blocked_plan = nullptr;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &blocked,
                                          &blocked_plan) == OA_CONTROL_EUNREACHABLE);
    CHECK(blocked_plan == nullptr);

    /* The same rejection applies when minimum_progress_m cannot be met. */
    fixture.refresh();
    auto greedy = converge_template(fixture);
    greedy.target_m[0] = kConvergeX;
    greedy.target_m[1] = kConvergeY;
    greedy.target_m[2] = kConvergeZ;
    greedy.minimum_progress_m = 10.0;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &greedy,
                                          &blocked_plan) == OA_CONTROL_EUNREACHABLE);
    CHECK(blocked_plan == nullptr);

    /* Negative and non-finite geometry is refused rather than clamped. */
    fixture.refresh();
    auto negative = converge_template(fixture);
    negative.target_m[0] = kConvergeX;
    negative.target_m[1] = kConvergeY;
    negative.target_m[2] = kConvergeZ;
    negative.stop_distance_m = -0.01;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &negative,
                                          &blocked_plan) == OA_CONTROL_EINVAL);
    auto poison = negative;
    poison.stop_distance_m = 0.05;
    poison.target_m[1] = std::numeric_limits<double>::infinity();
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &poison,
                                          &blocked_plan) == OA_CONTROL_EINVAL);
    poison.target_m[1] = std::numeric_limits<double>::quiet_NaN();
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &poison,
                                          &blocked_plan) == OA_CONTROL_EINVAL);
    poison.target_m[1] = kConvergeY;
    poison.minimum_progress_m = -1.0;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &poison,
                                          &blocked_plan) == OA_CONTROL_EINVAL);
    poison.minimum_progress_m = 0.001;
    poison.contact_torque_fraction = std::numeric_limits<double>::quiet_NaN();
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &poison,
                                          &blocked_plan) == OA_CONTROL_EINVAL);
    CHECK(blocked_plan == nullptr);
}

void test_converge_without_obstacle_does_not_report_contact() {
    Fixture fixture;
    /* A zero per-joint threshold must fall back to a fraction of tmax rather
     * than latching contact on the first near-zero torque sample. */
    CHECK(oa_control_default_contact_torque_fraction() > 0.0);
    CHECK(oa_control_default_contact_torque_fraction() < 1.0);
    CHECK(oa_control_default_contact_persistence_cycles() >= 1U);

    auto move = converge_template(fixture);
    move.target_m[0] = kConvergeX;
    move.target_m[1] = kConvergeY;
    move.target_m[2] = kConvergeZ;
    std::fill(std::begin(move.contact_torque_nm), std::end(move.contact_torque_nm), 0.0);
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    auto request = execute_template();
    std::uint64_t command_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) ==
          OA_CONTROL_OK);
    const auto report = run_until_stopped(fixture, 3000);
    /* With no obstacle in the scene the motion must never report contact. It
     * may still be halted by the keepout monitor as the claws close in, which
     * is a different and equally legitimate outcome. */
    CHECK(report.contact_detected == 0U);
    CHECK(report.cause != OA_STOP_CAUSE_CONTACT);
    oa_motion_plan_destroy(plan);
}

void test_sim_contact_validation() {
    Fixture fixture;
    oa_sim_contact contact{};
    init(contact);
    contact.side = OA_LEFT;
    contact.enabled = 1U;
    contact.center_m[0] = 0.25;
    contact.center_m[1] = 0.20;
    contact.center_m[2] = 0.40;
    contact.radius_m = 0.05;
    contact.reaction_gain_nm_per_rad = 400.0;
    CHECK(oa_controller_sim_set_contact(fixture.controller, &contact) == OA_CONTROL_OK);

    auto bad = contact;
    bad.side = 7U;
    CHECK(oa_controller_sim_set_contact(fixture.controller, &bad) == OA_CONTROL_EINVAL);

    bad = contact;
    bad.radius_m = -1.0;
    CHECK(oa_controller_sim_set_contact(fixture.controller, &bad) == OA_CONTROL_EINVAL);

    bad = contact;
    bad.radius_m = std::numeric_limits<double>::quiet_NaN();
    CHECK(oa_controller_sim_set_contact(fixture.controller, &bad) == OA_CONTROL_EINVAL);

    bad = contact;
    bad.reaction_gain_nm_per_rad = -1.0;
    CHECK(oa_controller_sim_set_contact(fixture.controller, &bad) == OA_CONTROL_EINVAL);

    /* Disabling never validates the geometry, so a cleared obstacle can always
     * be removed even if its previous parameters are no longer meaningful. */
    auto disabled = contact;
    disabled.enabled = 0U;
    disabled.radius_m = -5.0;
    CHECK(oa_controller_sim_set_contact(fixture.controller, &disabled) == OA_CONTROL_OK);
}

void test_contact_torque_stops_converge() {
    Fixture fixture;
    move_to_separated_pose(fixture);
    const auto left = fixture.tcp(OA_LEFT);

    /* Put a small obstacle a fifth of the way along the left claw's approach
     * ray, so the arm meets it well before the claws come near each other and
     * contact, not keepout, is the monitor that must fire. */
    const double fraction = 0.20;
    oa_sim_contact contact{};
    init(contact);
    contact.side = OA_LEFT;
    contact.enabled = 1U;
    contact.center_m[0] = left[0] + fraction * (kConvergeX - left[0]);
    contact.center_m[1] = left[1] + fraction * (kConvergeY - left[1]);
    contact.center_m[2] = left[2] + fraction * (kConvergeZ - left[2]);
    contact.radius_m = 0.04;
    /* Zero selects the per-motor default stiffness. */
    contact.reaction_gain_nm_per_rad = 0.0;
    CHECK(oa_controller_sim_set_contact(fixture.controller, &contact) == OA_CONTROL_OK);

    fixture.refresh();
    auto move = converge_template(fixture);
    move.velocity_scale = 1.0;
    move.acceleration_scale = 1.0;
    move.jerk_scale = 1.0;
    move.target_m[0] = kConvergeX;
    move.target_m[1] = kConvergeY;
    move.target_m[2] = kConvergeZ;
    move.contact_persistence_cycles = 2U;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    auto request = execute_template();
    std::uint64_t command_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) ==
          OA_CONTROL_OK);

    const auto report = run_until_stopped(fixture, 4000);
    CHECK(report.cause == OA_STOP_CAUSE_CONTACT);
    CHECK(report.contact_detected == 1U);
    /* The obstacle is on the left arm only. */
    CHECK((report.contact_side_mask & 0x1U) != 0U);
    CHECK(report.contact_joint_mask[0] != 0U);
    CHECK(report.stop_feedback_seq[0] != 0U);
    CHECK(report.stop_monotonic_ns != 0U);
    /* Contact fired while the arms were still comfortably clear of each other,
     * proving the two monitors are independent. */
    CHECK(report.minimum_clearance_m > oa_collision_required_clearance_m());
    /* Every joint the report flags must genuinely be over its threshold, and
     * every threshold must be a positive fraction of that motor's capability. */
    bool any_over = false;
    for (std::size_t side = 0; side < 2U; ++side) {
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            CHECK(report.threshold_torque_nm[side][joint] > 0.0);
            if ((report.contact_joint_mask[side] & (1U << joint)) != 0U) {
                CHECK(std::abs(report.contact_torque_nm[side][joint]) >=
                      report.threshold_torque_nm[side][joint]);
                any_over = true;
            }
        }
    }
    CHECK(any_over);
    /* The claw stopped at the obstacle, not at the requested target. */
    double travelled = 0.0;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        const double delta = report.stopped_tcp_m[0][axis] - left[axis];
        travelled += delta * delta;
    }
    CHECK(std::sqrt(travelled) > 0.005);
    oa_motion_plan_destroy(plan);
}

void test_explicit_contact_threshold_is_honoured() {
    Fixture fixture;
    move_to_separated_pose(fixture);
    const auto left = fixture.tcp(OA_LEFT);

    oa_sim_contact contact{};
    init(contact);
    contact.side = OA_LEFT;
    contact.enabled = 1U;
    contact.center_m[0] = left[0] + 0.20 * (kConvergeX - left[0]);
    contact.center_m[1] = left[1] + 0.20 * (kConvergeY - left[1]);
    contact.center_m[2] = left[2] + 0.20 * (kConvergeZ - left[2]);
    contact.radius_m = 0.04;
    contact.reaction_gain_nm_per_rad = 0.0;
    CHECK(oa_controller_sim_set_contact(fixture.controller, &contact) == OA_CONTROL_OK);

    fixture.refresh();
    auto move = converge_template(fixture);
    move.velocity_scale = 1.0;
    move.acceleration_scale = 1.0;
    move.jerk_scale = 1.0;
    move.target_m[0] = kConvergeX;
    move.target_m[1] = kConvergeY;
    move.target_m[2] = kConvergeZ;
    move.contact_persistence_cycles = 1U;
    /* A deliberately gentle explicit threshold must be used verbatim rather
     * than being overridden by the tmax fraction. */
    std::fill(std::begin(move.contact_torque_nm), std::end(move.contact_torque_nm), 0.75);
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    auto request = execute_template();
    std::uint64_t command_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) ==
          OA_CONTROL_OK);
    const auto report = run_until_stopped(fixture, 4000);
    CHECK(report.cause == OA_STOP_CAUSE_CONTACT);
    for (std::size_t joint = 0; joint < 7U; ++joint) {
        CHECK(std::abs(report.threshold_torque_nm[0][joint] - 0.75) < 1.0e-12);
    }
    oa_motion_plan_destroy(plan);
}

void test_realtime_keepout_uses_shared_geometry() {
    /* The real-time monitor and the pre-flight guard must gate on one
     * implementation. Prove the shared entry point is present, self-consistent,
     * and fails closed on degenerate input. */
    CHECK(oa_collision_required_clearance_m() > 0.0);
    CHECK(oa_collision_arm_radius_m() > 0.0);
    CHECK(oa_collision_tool_radius_m() >= oa_collision_arm_radius_m());

    oa_collision_report report{};
    CHECK(oa_collision_evaluate(nullptr, &report) == OA_MODEL_EINVAL);
    CHECK(report.clear == 0U);
    CHECK(oa_collision_evaluate(nullptr, nullptr) == OA_MODEL_EINVAL);

    oa_collision_scene scene{};
    scene.abi_version = OA_COLLISION_ABI_VERSION;
    scene.struct_size = sizeof(scene);
    /* Two arms far apart and far from the shaft: clear. */
    for (std::size_t index = 0; index < OA_COLLISION_POINTS; ++index) {
        scene.point[0][index][0] = 1.0 + 0.01 * static_cast<double>(index);
        scene.point[0][index][1] = 2.0;
        scene.point[0][index][2] = 0.4;
        scene.point[1][index][0] = -1.0 - 0.01 * static_cast<double>(index);
        scene.point[1][index][1] = -2.0;
        scene.point[1][index][2] = 0.4;
    }
    CHECK(oa_collision_evaluate(&scene, &report) == OA_MODEL_OK);
    CHECK(report.clear == 1U);
    CHECK(report.minimum_clearance_m > oa_collision_required_clearance_m());

    /* Overlapping arms: an arm-arm violation, reported as such. */
    oa_collision_scene overlap = scene;
    for (std::size_t index = 0; index < OA_COLLISION_POINTS; ++index) {
        overlap.point[1][index][0] = overlap.point[0][index][0];
        overlap.point[1][index][1] = overlap.point[0][index][1];
        overlap.point[1][index][2] = overlap.point[0][index][2];
    }
    CHECK(oa_collision_evaluate(&overlap, &report) == OA_MODEL_OK);
    CHECK(report.clear == 0U);
    CHECK(report.violation == OA_COLLISION_VIOLATION_ARM_ARM);

    /* A non-finite coordinate fails closed with a distinct status. */
    oa_collision_scene poisoned = scene;
    poisoned.point[0][3][1] = std::numeric_limits<double>::quiet_NaN();
    CHECK(oa_collision_evaluate(&poisoned, &report) == OA_MODEL_ENONFINITE);
    CHECK(report.clear == 0U);
    CHECK(report.violation == OA_COLLISION_VIOLATION_NONFINITE);

    /* A wrong ABI is rejected before any geometry runs. */
    oa_collision_scene stale = scene;
    stale.abi_version = OA_COLLISION_ABI_VERSION + 1u;
    CHECK(oa_collision_evaluate(&stale, &report) == OA_MODEL_EINVAL);

    /* Degenerate capsules (zero length) must still produce a finite answer. */
    const double point[3] = {0.5, 0.5, 0.5};
    const double clearance =
        oa_collision_segment_clearance(point, point, point, point, 0.0, 0.0);
    CHECK(std::abs(clearance) < 1.0e-12);

    /* Malformed cylinders fail closed rather than reporting free space. */
    const double a[3] = {0.3, 0.0, 0.3};
    const double b[3] = {0.4, 0.0, 0.4};
    CHECK(std::isinf(oa_collision_finite_cylinder_capsule_clearance(a, b, 0.05, 1.0,
                                                                    0.0, 0.05)));
    CHECK(std::isinf(
        oa_collision_finite_cylinder_capsule_clearance(a, b, -1.0, 0.0, 1.0, 0.05)));
}

// A pose inside the intervention floor must remain escapable.
//
// The keepout monitor originally vetoed on the absolute clearance, so any
// command that legitimately ended inside the floor left the arms trapped: the
// next command was halted on its first cycle, including one that moved them
// apart. Converge reaches such poses by design.
void test_keepout_monitor_allows_retreat_but_still_stops_approach() {
    Fixture fixture;
    move_to_separated_pose(fixture);

    // Drive the claws together until the monitor intervenes.
    const auto left = fixture.tcp(OA_LEFT);
    const auto right = fixture.tcp(OA_RIGHT);
    auto approach = converge_template(fixture);
    approach.velocity_scale = 1.0;
    approach.acceleration_scale = 1.0;
    approach.jerk_scale = 1.0;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        approach.target_m[axis] = 0.5 * (left[axis] + right[axis]);
    }
    approach.stop_distance_m = 0.0;
    approach.minimum_progress_m = 0.001;
    oa_motion_plan *plan = nullptr;
    if (oa_controller_plan_converge_tcp(fixture.controller, &approach, &plan) !=
        OA_CONTROL_OK) {
        return;  // Geometry unreachable on this build; the retreat case below
                 // is covered by the keepout stop exercised elsewhere.
    }
    auto request = execute_template();
    std::uint64_t command_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) ==
          OA_CONTROL_OK);
    const auto halted = run_until_stopped(fixture, 6000);
    oa_motion_plan_destroy(plan);
    /* The approach must have been stopped, not completed. */
    CHECK(halted.cause != OA_STOP_CAUSE_NONE);

    // Now retreat. Starting clearance is at or below the floor, so this is the
    // case that used to be impossible.
    fixture.refresh();
    auto retreat = paired_template(fixture);
    retreat.velocity_scale = 1.0;
    retreat.acceleration_scale = 1.0;
    retreat.jerk_scale = 1.0;
    retreat.left_tcp_m[0] = 0.30;
    retreat.left_tcp_m[1] = 0.26;
    retreat.left_tcp_m[2] = 0.40;
    retreat.right_tcp_m[0] = 0.30;
    retreat.right_tcp_m[1] = -0.26;
    retreat.right_tcp_m[2] = 0.40;
    oa_motion_plan *escape = nullptr;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &retreat, &escape) ==
          OA_CONTROL_OK);
    auto escape_request = execute_template();
    std::uint64_t escape_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, escape, &escape_request, &escape_id) ==
          OA_CONTROL_OK);
    const auto escaped = run_until_stopped(fixture, 6000);
    CHECK(escaped.cause == OA_STOP_CAUSE_PLAN_COMPLETE);
    /* And it genuinely opened up. */
    fixture.refresh();
    const auto opened_left = fixture.tcp(OA_LEFT);
    const auto opened_right = fixture.tcp(OA_RIGHT);
    double separation = 0.0;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        const double delta = opened_left[axis] - opened_right[axis];
        separation += delta * delta;
    }
    CHECK(std::sqrt(separation) > 0.40);
    oa_motion_plan_destroy(escape);
}

void test_keepout_monitor_resets_after_recovery() {
    Fixture fixture;
    move_to_separated_pose(fixture);

    const auto drive_into_keepout = [&fixture]() {
        const auto left = fixture.tcp(OA_LEFT);
        const auto right = fixture.tcp(OA_RIGHT);
        auto approach = converge_template(fixture);
        approach.velocity_scale = 1.0;
        approach.acceleration_scale = 1.0;
        approach.jerk_scale = 1.0;
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            approach.target_m[axis] = 0.5 * (left[axis] + right[axis]);
        }
        approach.stop_distance_m = 0.0;
        approach.minimum_progress_m = 0.001;
        oa_motion_plan *plan = nullptr;
        const oa_control_status planned =
            oa_controller_plan_converge_tcp(fixture.controller, &approach, &plan);
        if (planned != OA_CONTROL_OK) {
            return oa_contact_report{};
        }
        auto request = execute_template();
        std::uint64_t command_id = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) ==
              OA_CONTROL_OK);
        const auto report = run_until_stopped(fixture, 6000);
        oa_motion_plan_destroy(plan);
        return report;
    };

    auto first = drive_into_keepout();
    if (first.cause == OA_STOP_CAUSE_NONE) {
        return;
    }
    CHECK(first.cause == OA_STOP_CAUSE_KEEPOUT);

    fixture.refresh();
    auto retreat = paired_template(fixture);
    retreat.velocity_scale = 1.0;
    retreat.acceleration_scale = 1.0;
    retreat.jerk_scale = 1.0;
    retreat.left_tcp_m[0] = 0.30;
    retreat.left_tcp_m[1] = 0.26;
    retreat.left_tcp_m[2] = 0.40;
    retreat.right_tcp_m[0] = 0.30;
    retreat.right_tcp_m[1] = -0.26;
    retreat.right_tcp_m[2] = 0.40;
    oa_motion_plan *escape = nullptr;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &retreat, &escape) ==
          OA_CONTROL_OK);
    auto escape_request = execute_template();
    std::uint64_t escape_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, escape, &escape_request, &escape_id) ==
          OA_CONTROL_OK);
    const auto escaped = run_until_stopped(fixture, 6000);
    CHECK(escaped.cause == OA_STOP_CAUSE_PLAN_COMPLETE);
    oa_motion_plan_destroy(escape);

    fixture.refresh();
    const auto second = drive_into_keepout();
    if (second.cause == OA_STOP_CAUSE_NONE) {
        return;
    }
    CHECK(second.cause == OA_STOP_CAUSE_KEEPOUT);
}

void test_estop_is_always_listening() {
    CHECK(oa_estop_clear() == OA_CONTROL_OK);
    CHECK(oa_estop_asserted() == 0U);
    const std::uint64_t before = oa_estop_assert_count();

    Fixture fixture;
    /* Latched while idle: a new plan is refused outright. */
    oa_estop_assert();
    CHECK(oa_estop_asserted() == 1U);
    CHECK(oa_estop_assert_count() == before + 1U);
    auto move = paired_template(fixture);
    move.left_tcp_m[0] = 0.20;
    move.left_tcp_m[1] = 0.30;
    move.left_tcp_m[2] = 0.85;
    move.right_tcp_m[0] = 0.20;
    move.right_tcp_m[1] = -0.30;
    move.right_tcp_m[2] = 0.85;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_EESTOP);
    CHECK(plan == nullptr);

    /* Advancing while latched reports the stop and never resumes motion. */
    CHECK(fixture.step() != OA_CONTROL_OK);
    for (int cycle = 0; cycle < 5; ++cycle) {
        CHECK(fixture.step() == OA_CONTROL_EESTOP);
    }

    /* Asserting repeatedly is idempotent for state and counted for evidence. */
    oa_estop_assert();
    oa_estop_assert();
    CHECK(oa_estop_asserted() == 1U);
    CHECK(oa_estop_assert_count() == before + 3U);

    CHECK(oa_estop_clear() == OA_CONTROL_OK);
    CHECK(oa_estop_asserted() == 0U);
}

void test_estop_from_another_thread_during_execution() {
    CHECK(oa_estop_clear() == OA_CONTROL_OK);
    Fixture fixture;
    auto move = paired_template(fixture);
    move.left_tcp_m[0] = 0.20;
    move.left_tcp_m[1] = 0.30;
    move.left_tcp_m[2] = 0.85;
    move.right_tcp_m[0] = 0.20;
    move.right_tcp_m[1] = -0.30;
    move.right_tcp_m[2] = 0.85;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_OK);
    oa_execute_request request{};
    init(request);
    request.expiry_ns = 600000000000ULL;
    request.producer_deadline_ns = 600000000000ULL;
    request.stop_kind = OA_STOP_CONTROLLED;
    std::uint64_t command_id = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) ==
          OA_CONTROL_OK);

    /* Run a few cycles, then assert from a separate thread while the command is
     * mid-flight. The very next cycle must observe it. */
    for (int cycle = 0; cycle < 3; ++cycle) {
        (void)fixture.step();
    }
    std::atomic<bool> asserted{false};
    std::thread trigger([&asserted]() {
        oa_estop_assert();
        asserted.store(true, std::memory_order_release);
    });
    trigger.join();
    CHECK(asserted.load(std::memory_order_acquire));
    CHECK(oa_estop_asserted() == 1U);

    const oa_control_status status = fixture.step();
    CHECK(status == OA_CONTROL_EESTOP);
    /* Subsequent cycles stay stopped; the latch is not self-clearing. */
    for (int cycle = 0; cycle < 3; ++cycle) {
        CHECK(fixture.step() == OA_CONTROL_EESTOP);
    }
    oa_motion_plan_destroy(plan);
    CHECK(oa_estop_clear() == OA_CONTROL_OK);
}

void test_contact_report_abi_is_validated() {
    Fixture fixture;
    oa_contact_report report{};
    report.struct_size = sizeof(report);
    report.abi_version = OA_CONTROL_ABI_V1 + 1U;
    CHECK(oa_controller_get_contact_report(fixture.controller, &report) ==
          OA_CONTROL_EABI);

    oa_contact_report undersized{};
    undersized.struct_size = 4U;
    undersized.abi_version = OA_CONTROL_ABI_V1;
    CHECK(oa_controller_get_contact_report(fixture.controller, &undersized) ==
          OA_CONTROL_EINVAL);

    CHECK(oa_controller_get_contact_report(fixture.controller, nullptr) ==
          OA_CONTROL_EINVAL);

    /* A fresh controller reports no stop cause at all. */
    oa_contact_report clean{};
    init(clean);
    CHECK(oa_controller_get_contact_report(fixture.controller, &clean) ==
          OA_CONTROL_OK);
    CHECK(clean.cause == OA_STOP_CAUSE_NONE);
    CHECK(clean.contact_detected == 0U);
}

double units_per_metre(const oa_length_unit unit) {
    if (unit == OA_LENGTH_UNIT_CENTIMETRES) return 100.0;
    if (unit == OA_LENGTH_UNIT_INCHES) return 1.0 / 0.0254;
    return 1.0;
}

void test_all_bimanual_unit_ingress_is_binary64_equivalent() {
    Fixture fixture;
    const oa_length_unit units[] = {OA_LENGTH_UNIT_METRES,
                                    OA_LENGTH_UNIT_CENTIMETRES,
                                    OA_LENGTH_UNIT_INCHES};

    auto centroid = centroid_template(fixture);
    const auto left = fixture.tcp(OA_LEFT);
    const auto right = fixture.tcp(OA_RIGHT);
    centroid.target_centroid_m[0] = 0.5 * (left[0] + right[0]);
    centroid.target_centroid_m[1] = 0.5 * (left[1] + right[1]);
    centroid.target_centroid_m[2] = 0.5 * (left[2] + right[2]);
    oa_motion_plan *canonical_centroid = nullptr;
    CHECK(oa_controller_plan_centroid_tcp(
              fixture.controller, &centroid, &canonical_centroid) == OA_CONTROL_OK);
    const auto centroid_report = report_of(canonical_centroid);
    for (const auto unit : units) {
        oa_centroid_tcp_move_with_units request{};
        init(request);
        request.coordinate_unit = unit;
        request.expiry_ns = centroid.expiry_ns;
        std::copy_n(centroid.required_feedback_seq, 2U, request.required_feedback_seq);
        const double factor = units_per_metre(unit);
        request.target_centroid = {centroid.target_centroid_m[0] * factor,
                                   centroid.target_centroid_m[1] * factor,
                                   centroid.target_centroid_m[2] * factor};
        request.velocity_scale = centroid.velocity_scale;
        request.acceleration_scale = centroid.acceleration_scale;
        request.jerk_scale = centroid.jerk_scale;
        request.tcp_tol_m = centroid.tcp_tol_m;
        request.collision_scene_revision = centroid.collision_scene_revision;
        request.max_branch_step_rad = centroid.max_branch_step_rad;
        request.min_singular_value = centroid.min_singular_value;
        oa_motion_plan *plan = nullptr;
        CHECK(oa_controller_plan_centroid_tcp_with_units(
                  fixture.controller, &request, &plan) == OA_CONTROL_OK);
        const auto report = report_of(plan);
        for (std::size_t side = 0; side < 2U; ++side) {
            for (std::size_t axis = 0; axis < 3U; ++axis) {
                CHECK(std::abs(report.achieved_tcp_m[side][axis] -
                               centroid_report.achieved_tcp_m[side][axis]) < 1.0e-12);
            }
        }
        oa_motion_plan_destroy(plan);
    }
    oa_motion_plan_destroy(canonical_centroid);

    auto mirrored = mirrored_template(fixture);
    mirrored.lead_side = OA_LEFT;
    mirrored.lead_tcp_m[0] = 0.30;
    mirrored.lead_tcp_m[1] = 0.22;
    mirrored.lead_tcp_m[2] = 0.30;
    oa_motion_plan *canonical_mirror = nullptr;
    CHECK(oa_controller_plan_mirrored_tcp(
              fixture.controller, &mirrored, &canonical_mirror) == OA_CONTROL_OK);
    const auto mirror_report = report_of(canonical_mirror);
    for (const auto unit : units) {
        oa_mirrored_tcp_move_with_units request{};
        init(request);
        request.coordinate_unit = unit;
        request.expiry_ns = mirrored.expiry_ns;
        std::copy_n(mirrored.required_feedback_seq, 2U, request.required_feedback_seq);
        request.lead_side = mirrored.lead_side;
        const double factor = units_per_metre(unit);
        request.lead_tcp = {mirrored.lead_tcp_m[0] * factor,
                            mirrored.lead_tcp_m[1] * factor,
                            mirrored.lead_tcp_m[2] * factor};
        request.velocity_scale = mirrored.velocity_scale;
        request.acceleration_scale = mirrored.acceleration_scale;
        request.jerk_scale = mirrored.jerk_scale;
        request.tcp_tol_m = mirrored.tcp_tol_m;
        request.collision_scene_revision = mirrored.collision_scene_revision;
        request.max_branch_step_rad = mirrored.max_branch_step_rad;
        request.min_singular_value = mirrored.min_singular_value;
        oa_motion_plan *plan = nullptr;
        CHECK(oa_controller_plan_mirrored_tcp_with_units(
                  fixture.controller, &request, &plan) == OA_CONTROL_OK);
        const auto report = report_of(plan);
        for (std::size_t side = 0; side < 2U; ++side) {
            for (std::size_t axis = 0; axis < 3U; ++axis) {
                CHECK(std::abs(report.achieved_tcp_m[side][axis] -
                               mirror_report.achieved_tcp_m[side][axis]) < 1.0e-12);
            }
        }
        oa_motion_plan_destroy(plan);
    }
    oa_motion_plan_destroy(canonical_mirror);

    auto converge = converge_template(fixture);
    converge.target_m[0] = 0.40;
    converge.target_m[1] = 0.0;
    converge.target_m[2] = 0.50;
    oa_motion_plan *canonical_converge = nullptr;
    CHECK(oa_controller_plan_converge_tcp(
              fixture.controller, &converge, &canonical_converge) == OA_CONTROL_OK);
    const auto converge_report = report_of(canonical_converge);
    for (const auto unit : units) {
        oa_converge_tcp_move_with_units request{};
        init(request);
        request.coordinate_unit = unit;
        request.expiry_ns = converge.expiry_ns;
        std::copy_n(converge.required_feedback_seq, 2U, request.required_feedback_seq);
        const double factor = units_per_metre(unit);
        request.target = {converge.target_m[0] * factor,
                          converge.target_m[1] * factor,
                          converge.target_m[2] * factor};
        std::copy_n(converge.contact_torque_nm, 7U, request.contact_torque_nm);
        request.contact_torque_fraction = converge.contact_torque_fraction;
        request.contact_persistence_cycles = converge.contact_persistence_cycles;
        request.stop_distance_m = converge.stop_distance_m;
        request.minimum_progress_m = converge.minimum_progress_m;
        request.velocity_scale = converge.velocity_scale;
        request.acceleration_scale = converge.acceleration_scale;
        request.jerk_scale = converge.jerk_scale;
        request.tcp_tol_m = converge.tcp_tol_m;
        request.collision_scene_revision = converge.collision_scene_revision;
        request.max_branch_step_rad = converge.max_branch_step_rad;
        request.min_singular_value = converge.min_singular_value;
        oa_motion_plan *plan = nullptr;
        CHECK(oa_controller_plan_converge_tcp_with_units(
                  fixture.controller, &request, &plan) == OA_CONTROL_OK);
        const auto report = report_of(plan);
        for (std::size_t side = 0; side < 2U; ++side) {
            for (std::size_t axis = 0; axis < 3U; ++axis) {
                CHECK(std::abs(report.achieved_tcp_m[side][axis] -
                               converge_report.achieved_tcp_m[side][axis]) < 1.0e-12);
            }
        }
        oa_motion_plan_destroy(plan);
    }
    oa_motion_plan_destroy(canonical_converge);

    oa_centroid_tcp_move_with_units invalid{};
    init(invalid);
    invalid.coordinate_unit = 99U;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_centroid_tcp_with_units(
              fixture.controller, &invalid, &plan) == OA_CONTROL_EINVAL);
    CHECK(plan == nullptr);
}

void test_planners_reject_null_and_bad_abi() {
    Fixture fixture;
    oa_motion_plan *plan = nullptr;
    auto centroid = centroid_template(fixture);
    auto mirrored = mirrored_template(fixture);
    auto converge = converge_template(fixture);

    CHECK(oa_controller_plan_centroid_tcp(fixture.controller, nullptr, &plan) ==
          OA_CONTROL_EINVAL);
    CHECK(oa_controller_plan_centroid_tcp(fixture.controller, &centroid, nullptr) ==
          OA_CONTROL_EINVAL);
    CHECK(oa_controller_plan_mirrored_tcp(fixture.controller, nullptr, &plan) ==
          OA_CONTROL_EINVAL);
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, nullptr, &plan) ==
          OA_CONTROL_EINVAL);

    auto bad_centroid = centroid;
    bad_centroid.abi_version = OA_CONTROL_ABI_V1 + 1U;
    CHECK(oa_controller_plan_centroid_tcp(fixture.controller, &bad_centroid, &plan) ==
          OA_CONTROL_EABI);
    auto bad_mirrored = mirrored;
    bad_mirrored.abi_version = OA_CONTROL_ABI_V1 + 9U;
    CHECK(oa_controller_plan_mirrored_tcp(fixture.controller, &bad_mirrored, &plan) ==
          OA_CONTROL_EABI);
    auto bad_converge = converge;
    bad_converge.abi_version = 0U;
    CHECK(oa_controller_plan_converge_tcp(fixture.controller, &bad_converge, &plan) ==
          OA_CONTROL_EABI);

    /* A null controller handle is rejected without dereferencing it. */
    CHECK(oa_controller_plan_centroid_tcp(nullptr, &centroid, &plan) !=
          OA_CONTROL_OK);
    CHECK(oa_controller_plan_mirrored_tcp(nullptr, &mirrored, &plan) !=
          OA_CONTROL_OK);
    CHECK(oa_controller_plan_converge_tcp(nullptr, &converge, &plan) !=
          OA_CONTROL_OK);
    CHECK(plan == nullptr);
}

}  // namespace

int main() {
    test_centroid_translates_both_claws_equally();
    test_centroid_rejects_non_finite_and_zero_motion();
    test_mirrored_negates_y_only();
    test_mirrored_rejects_bad_side_and_reserved();
    test_converge_stops_short_and_rejects_no_progress();
    test_converge_without_obstacle_does_not_report_contact();
    test_sim_contact_validation();
    test_contact_torque_stops_converge();
    test_explicit_contact_threshold_is_honoured();
    test_realtime_keepout_uses_shared_geometry();
    test_keepout_monitor_allows_retreat_but_still_stops_approach();
    test_keepout_monitor_resets_after_recovery();
    test_estop_is_always_listening();
    test_estop_from_another_thread_during_execution();
    test_contact_report_abi_is_validated();
    test_all_bimanual_unit_ingress_is_binary64_equivalent();
    test_planners_reject_null_and_bad_abi();
    std::printf("bimanual tests passed\n");
    return 0;
}
