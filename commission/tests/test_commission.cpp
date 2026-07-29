#include "openarm_commission.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            std::cerr << __FILE__ << ':' << __LINE__ << ": " #condition       \
                      << " failed\n";                                           \
            ++failures;                                                        \
        }                                                                       \
    } while (false)

template <typename T>
T output_record() {
    T value{};
    value.struct_size = sizeof(value);
    value.abi_version = OA_COMMISSION_ABI_V1;
    return value;
}

oa_commission_encoder_sample sample(std::uint64_t sequence,
                                    std::uint64_t time_ns,
                                    double q,
                                    double dq = 0.0,
                                    double torque = 0.0,
                                    std::uint32_t enabled = 0U) {
    auto value = output_record<oa_commission_encoder_sample>();
    value.feedback_seq = sequence;
    value.sample_time_ns = time_ns;
    value.q_output_rad = q;
    value.dq_output_rad_s = dq;
    value.torque_output_nm = torque;
    value.mos_temperature_c = 25.0;
    value.coil_temperature_c = 26.0;
    value.drive_enabled = enabled;
    return value;
}

oa_commission_manual_options manual_options(std::uint32_t references,
                                            std::int32_t sign) {
    auto value = output_record<oa_commission_manual_options>();
    value.side = OA_COMMISSION_LEFT;
    value.joint = 2U;
    value.expected_revision = 7U;
    value.reference_count = references;
    value.known_sign = sign;
    value.minimum_samples = 3U;
    value.maximum_sample_age_ns = 20U;
    value.stability_dwell_ns = 10U;
    value.reference_model_rad[0] = 0.5;
    value.reference_model_rad[1] = 1.0;
    value.maximum_position_spread_rad = 0.01;
    value.maximum_abs_velocity_rad_s = 0.02;
    value.minimum_reference_separation_rad = 0.1;
    value.maximum_scale_error = 0.05;
    std::strcpy(value.motor_serial, "DM-TEST-0001");
    return value;
}

void feed_manual_reference(oa_commission_manual_session *session,
                           std::uint32_t reference,
                           std::uint64_t sequence_base,
                           std::uint64_t time_base,
                           double q) {
    auto report = output_record<oa_commission_manual_report>();
    for (std::uint64_t index = 0; index < 3U; ++index) {
        const auto value = sample(sequence_base + index, time_base + index * 5U, q);
        CHECK(oa_commission_manual_sample(
                  session, reference, value.sample_time_ns + 1U, &value, &report) ==
              OA_COMMISSION_OK);
    }
}

void test_manual_known_sign() {
    const auto options = manual_options(1U, 1);
    oa_commission_manual_session *session = nullptr;
    CHECK(oa_commission_manual_create(&options, &session) == OA_COMMISSION_OK);
    CHECK(session != nullptr);
    feed_manual_reference(session, 0U, 1U, 100U, 0.2);

    auto report = output_record<oa_commission_manual_report>();
    CHECK(oa_commission_manual_get_report(session, &report) == OA_COMMISSION_OK);
    CHECK(report.state == OA_MANUAL_CANDIDATE);
    CHECK(report.candidate_a == 1.0);
    CHECK(std::abs(report.candidate_b_rad - 0.3) < 1.0e-12);
    CHECK(oa_commission_manual_begin_review(session, &report) == OA_COMMISSION_OK);
    CHECK(report.state == OA_MANUAL_REVIEW);

    auto patch = output_record<oa_commission_mapping_patch>();
    CHECK(oa_commission_manual_commit(session, 8U, "fixture-A", &patch) ==
          OA_COMMISSION_OK);
    CHECK(patch.expected_revision == 7U);
    CHECK(patch.replacement_revision == 8U);
    CHECK(patch.a == 1.0);
    CHECK(std::abs(patch.b_rad - 0.3) < 1.0e-12);
    CHECK(std::string(patch.evidence_record) == "fixture-A");
    CHECK(oa_commission_manual_abort(session) == OA_COMMISSION_ESTATE);
    oa_commission_manual_destroy(session);
}

void test_manual_two_reference_sign() {
    const auto options = manual_options(2U, 0);
    oa_commission_manual_session *session = nullptr;
    CHECK(oa_commission_manual_create(&options, &session) == OA_COMMISSION_OK);
    feed_manual_reference(session, 0U, 1U, 100U, -0.1);
    feed_manual_reference(session, 1U, 4U, 120U, -0.6);

    auto report = output_record<oa_commission_manual_report>();
    CHECK(oa_commission_manual_get_report(session, &report) == OA_COMMISSION_OK);
    CHECK(report.state == OA_MANUAL_CANDIDATE);
    CHECK(report.candidate_a == -1.0);
    CHECK(std::abs(report.candidate_b_rad - 0.4) < 1.0e-12);
    oa_commission_manual_destroy(session);
}

void test_manual_fail_closed() {
    auto options = manual_options(1U, 0);
    oa_commission_manual_session *session = nullptr;
    CHECK(oa_commission_manual_create(&options, &session) == OA_COMMISSION_EINVAL);
    CHECK(session == nullptr);

    options = manual_options(1U, 1);
    CHECK(oa_commission_manual_create(&options, &session) == OA_COMMISSION_OK);
    auto report = output_record<oa_commission_manual_report>();
    auto enabled = sample(1U, 100U, 0.2, 0.0, 0.0, 1U);
    CHECK(oa_commission_manual_sample(session, 0U, 101U, &enabled, &report) ==
          OA_COMMISSION_EINTERLOCK);
    auto fresh = sample(2U, 110U, 0.2);
    CHECK(oa_commission_manual_sample(session, 0U, 131U, &fresh, &report) ==
          OA_COMMISSION_ESTALE);
    auto moving = sample(3U, 120U, 0.2, 0.03);
    CHECK(oa_commission_manual_sample(session, 0U, 121U, &moving, &report) ==
          OA_COMMISSION_EUNSTABLE);
    CHECK(report.state == OA_MANUAL_COLLECT_REFERENCE_1);

    auto patch = output_record<oa_commission_mapping_patch>();
    std::memset(patch.motor_serial, 0x5a, sizeof(patch.motor_serial));
    const auto before = patch;
    CHECK(oa_commission_manual_commit(session, 8U, "fixture", &patch) ==
          OA_COMMISSION_ESTATE);
    CHECK(std::memcmp(&patch, &before, sizeof(patch)) == 0);
    CHECK(oa_commission_manual_abort(session) == OA_COMMISSION_OK);
    CHECK(oa_commission_manual_get_report(session, &report) == OA_COMMISSION_OK);
    CHECK(report.state == OA_MANUAL_ABORTED);
    oa_commission_manual_destroy(session);
}

oa_commission_recipe recipe() {
    auto value = output_record<oa_commission_recipe>();
    value.recipe_kind = OA_RECIPE_GRIPPER;
    value.side = OA_COMMISSION_RIGHT;
    value.joint = 7U;
    value.known_sign = 1;
    value.simulation_only = 1U;
    value.minimum_contact_samples = 2U;
    value.expected_revision = 10U;
    value.maximum_sample_age_ns = 50U;
    value.maximum_approach_time_ns = 100U;
    value.contact_dwell_ns = 10U;
    value.maximum_retreat_time_ns = 100U;
    value.stop_model_rad = 0.0;
    value.approach_direction = 1.0;
    value.start_min_output_rad = -0.2;
    value.start_max_output_rad = 0.2;
    value.maximum_speed_rad_s = 1.0;
    value.minimum_contact_travel_rad = 0.05;
    value.maximum_approach_travel_rad = 0.2;
    value.contact_velocity_rad_s = 0.01;
    value.minimum_contact_torque_nm = 1.0;
    value.maximum_torque_nm = 3.0;
    value.maximum_contact_energy_j = 1.0;
    value.maximum_temperature_c = 60.0;
    value.retreat_distance_rad = 0.05;
    value.repeatability_tolerance_rad = 0.02;
    std::strcpy(value.motor_serial, "DM-GRIPPER-SIM");
    std::strcpy(value.qualification_record, "simulation-recipe-v1");
    std::strcpy(value.fixture_record, "simulated-closed-stop");
    return value;
}

oa_commission_recipe_input recipe_input(std::uint64_t sequence,
                                        std::uint64_t time_ns,
                                        double q,
                                        double dq,
                                        double torque,
                                        std::uint32_t enabled) {
    auto value = output_record<oa_commission_recipe_input>();
    value.now_ns = time_ns + 1U;
    value.encoder = sample(sequence, time_ns, q, dq, torque, enabled);
    value.estop_clear = 1U;
    value.deadman_held = 1U;
    return value;
}

struct RecipeDriver {
    oa_commission_recipe_session *session{nullptr};
    oa_commission_next_action action{output_record<oa_commission_next_action>()};
    oa_commission_recipe_report report{output_record<oa_commission_recipe_report>()};
    std::uint64_t sequence{0U};
    std::uint64_t time{100U};

    explicit RecipeDriver(const oa_commission_recipe &configuration) {
        CHECK(oa_commission_recipe_create(&configuration, &session) == OA_COMMISSION_OK);
    }

    ~RecipeDriver() {
        oa_commission_recipe_destroy(session);
    }

    oa_commission_status send(double q,
                              double dq,
                              double torque,
                              std::uint32_t enabled,
                              std::uint32_t ready = 0U,
                              std::uint32_t review = OA_REVIEW_NONE,
                              std::uint64_t time_advance = 10U) {
        ++sequence;
        time += time_advance;
        auto input = recipe_input(sequence, time, q, dq, torque, enabled);
        input.operator_ready = ready;
        input.review_decision = review;
        action = output_record<oa_commission_next_action>();
        report = output_record<oa_commission_recipe_report>();
        const auto result =
            oa_commission_recipe_step(session, &input, &action, &report);
        CHECK(action.kind <= OA_RECIPE_ACTION_ABORT_DISABLE);
        if (action.kind == OA_RECIPE_ACTION_APPROACH ||
            action.kind == OA_RECIPE_ACTION_CONTACT_DWELL ||
            action.kind == OA_RECIPE_ACTION_RETREAT ||
            action.kind == OA_RECIPE_ACTION_REAPPROACH) {
            CHECK(action.valid_until_ns >= input.now_ns);
            CHECK(action.maximum_speed_rad_s > 0.0);
            CHECK(action.maximum_travel_rad > 0.0);
            CHECK(action.maximum_torque_nm > 0.0);
            CHECK(action.maximum_temperature_c > 0.0);
            CHECK(std::abs(std::abs(action.direction) - 1.0) < 1.0e-12);
        }
        return result;
    }

    void advance_to(std::uint32_t target_state) {
        if (target_state == OA_RECIPE_PRECHECK) {
            return;
        }
        CHECK(send(0.0, 0.0, 0.0, 0U) == OA_COMMISSION_OK);
        if (target_state == OA_RECIPE_WAIT) {
            return;
        }
        CHECK(send(0.0, 0.0, 0.0, 0U, 1U) == OA_COMMISSION_OK);
        if (target_state == OA_RECIPE_APPROACH) {
            return;
        }
        CHECK(send(0.06, 0.2, 0.2, 1U) == OA_COMMISSION_OK);
        CHECK(send(0.11, 0.005, 2.0, 1U) == OA_COMMISSION_OK);
        if (target_state == OA_RECIPE_CONTACT_DWELL) {
            return;
        }
        CHECK(send(0.111, 0.005, 2.0, 1U) == OA_COMMISSION_OK);
        if (target_state == OA_RECIPE_RETREAT) {
            return;
        }
        CHECK(send(0.061, 0.0, 0.2, 1U) == OA_COMMISSION_OK);
        if (target_state == OA_RECIPE_REAPPROACH) {
            return;
        }
        CHECK(send(0.12, 0.005, 2.0, 1U) == OA_COMMISSION_OK);
        if (target_state == OA_RECIPE_REPEATABILITY) {
            return;
        }
        CHECK(send(0.12, 0.0, 0.0, 0U) == OA_COMMISSION_OK);
        if (target_state == OA_RECIPE_CANDIDATE) {
            return;
        }
        CHECK(send(0.12, 0.0, 0.0, 0U, 0U, OA_REVIEW_ACCEPT) ==
              OA_COMMISSION_OK);
        CHECK(target_state == OA_RECIPE_REVIEW);
    }
};

void verify_no_commit(RecipeDriver &driver) {
    auto patch = output_record<oa_commission_mapping_patch>();
    std::memset(patch.motor_serial, 0x3c, sizeof(patch.motor_serial));
    const auto before = patch;
    CHECK(oa_commission_recipe_commit(driver.session, 11U, &patch) ==
          OA_COMMISSION_ESTATE);
    CHECK(std::memcmp(&patch, &before, sizeof(patch)) == 0);
}

void test_recipe_happy_path() {
    const auto configuration = recipe();
    RecipeDriver driver(configuration);
    driver.advance_to(OA_RECIPE_REVIEW);
    CHECK(driver.report.state == OA_RECIPE_REVIEW);
    CHECK(driver.action.kind == OA_RECIPE_ACTION_COMMIT_READY);
    CHECK(std::abs(driver.report.first_stop_output_rad - 0.1105) < 1.0e-12);
    CHECK(std::abs(driver.report.second_stop_output_rad - 0.12) < 1.0e-12);
    CHECK(driver.report.candidate_a == 1.0);
    CHECK(std::abs(driver.report.candidate_b_rad + 0.11525) < 1.0e-12);

    auto patch = output_record<oa_commission_mapping_patch>();
    CHECK(oa_commission_recipe_commit(driver.session, 11U, &patch) ==
          OA_COMMISSION_OK);
    CHECK(patch.replacement_revision == 11U);
    CHECK(patch.joint == 7U);
    CHECK(std::abs(patch.b_rad + 0.11525) < 1.0e-12);
    CHECK(std::string(patch.evidence_record) == "simulated-closed-stop");
    CHECK(oa_commission_recipe_abort(driver.session) == OA_COMMISSION_ESTATE);

    auto input = recipe_input(++driver.sequence, driver.time + 10U, 0.12, 0.0, 0.0, 0U);
    auto action = output_record<oa_commission_next_action>();
    auto report = output_record<oa_commission_recipe_report>();
    CHECK(oa_commission_recipe_step(driver.session, &input, &action, &report) ==
          OA_COMMISSION_ESTATE);
    CHECK(action.kind == OA_RECIPE_ACTION_NONE);
}

void test_unqualified_arm_recipe_rejected() {
    auto configuration = recipe();
    configuration.recipe_kind = OA_RECIPE_ARM_JOINT;
    configuration.joint = 3U;
    configuration.simulation_only = 0U;
    oa_commission_recipe_session *session = nullptr;
    CHECK(oa_commission_recipe_create(&configuration, &session) ==
          OA_COMMISSION_EUNSUPPORTED);
    CHECK(session == nullptr);

    configuration.hardware_qualified = 1U;
    configuration.qualification_revision = 42U;
    std::strcpy(configuration.qualification_record, "bench-record-42");
    CHECK(oa_commission_recipe_create(&configuration, &session) == OA_COMMISSION_OK);
    oa_commission_recipe_destroy(session);
}

void test_abort_from_every_nonterminal_state() {
    const std::vector<std::uint32_t> states{
        OA_RECIPE_PRECHECK,      OA_RECIPE_WAIT,       OA_RECIPE_APPROACH,
        OA_RECIPE_CONTACT_DWELL, OA_RECIPE_RETREAT,    OA_RECIPE_REAPPROACH,
        OA_RECIPE_REPEATABILITY, OA_RECIPE_CANDIDATE,  OA_RECIPE_REVIEW};
    for (const auto state : states) {
        RecipeDriver driver(recipe());
        driver.advance_to(state);
        CHECK(oa_commission_recipe_abort(driver.session) == OA_COMMISSION_OK);
        auto report = output_record<oa_commission_recipe_report>();
        CHECK(oa_commission_recipe_get_report(driver.session, &report) ==
              OA_COMMISSION_OK);
        CHECK(report.state == OA_RECIPE_ABORT);
        CHECK(report.abort_reason == OA_ABORT_CALLER);
        verify_no_commit(driver);
        auto input = recipe_input(
            ++driver.sequence, driver.time + 10U, 0.0, 0.0, 0.0, 0U);
        auto action = output_record<oa_commission_next_action>();
        CHECK(oa_commission_recipe_step(
                  driver.session, &input, &action, &report) == OA_COMMISSION_ESTATE);
        CHECK(action.kind == OA_RECIPE_ACTION_ABORT_DISABLE);
    }
}

void test_fault_from_every_nonterminal_state() {
    const std::vector<std::uint32_t> states{
        OA_RECIPE_PRECHECK,      OA_RECIPE_WAIT,       OA_RECIPE_APPROACH,
        OA_RECIPE_CONTACT_DWELL, OA_RECIPE_RETREAT,    OA_RECIPE_REAPPROACH,
        OA_RECIPE_REPEATABILITY, OA_RECIPE_CANDIDATE,  OA_RECIPE_REVIEW};
    for (const auto state : states) {
        RecipeDriver driver(recipe());
        driver.advance_to(state);
        ++driver.sequence;
        driver.time += 10U;
        auto input = recipe_input(driver.sequence, driver.time, 0.0, 0.0, 0.0, 0U);
        input.encoder.drive_fault = 1U;
        auto action = output_record<oa_commission_next_action>();
        auto report = output_record<oa_commission_recipe_report>();
        CHECK(oa_commission_recipe_step(driver.session, &input, &action, &report) ==
              OA_COMMISSION_EFAULT);
        CHECK(report.state == OA_RECIPE_ABORT);
        CHECK(report.abort_reason == OA_ABORT_INTERLOCK);
        CHECK(action.kind == OA_RECIPE_ACTION_ABORT_DISABLE);
        verify_no_commit(driver);
    }
}

void test_faults_abort_and_never_commit() {
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_APPROACH);
        CHECK(driver.send(0.01, 0.0, 0.0, 1U, 0U, OA_REVIEW_NONE, 101U) ==
              OA_COMMISSION_ELIMIT);
        CHECK(driver.report.abort_reason == OA_ABORT_TIMEOUT);
        CHECK(driver.action.kind == OA_RECIPE_ACTION_ABORT_DISABLE);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_APPROACH);
        CHECK(driver.send(-0.01, 0.1, 0.0, 1U) == OA_COMMISSION_ELIMIT);
        CHECK(driver.report.abort_reason == OA_ABORT_LIMIT);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_APPROACH);
        CHECK(driver.send(0.01, 1.1, 0.0, 1U) == OA_COMMISSION_ELIMIT);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_APPROACH);
        CHECK(driver.send(0.01, 0.1, 3.1, 1U) == OA_COMMISSION_ELIMIT);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_APPROACH);
        ++driver.sequence;
        driver.time += 10U;
        auto input = recipe_input(driver.sequence, driver.time, 0.01, 0.1, 0.0, 1U);
        input.deadman_held = 0U;
        auto action = output_record<oa_commission_next_action>();
        auto report = output_record<oa_commission_recipe_report>();
        CHECK(oa_commission_recipe_step(driver.session, &input, &action, &report) ==
              OA_COMMISSION_EINTERLOCK);
        CHECK(report.state == OA_RECIPE_ABORT);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_CONTACT_DWELL);
        CHECK(driver.send(0.115, 0.02, 0.2, 1U) == OA_COMMISSION_EFAULT);
        CHECK(driver.report.abort_reason == OA_ABORT_CONTACT);
        verify_no_commit(driver);
    }
    {
        auto configuration = recipe();
        configuration.maximum_contact_energy_j = 1.0e-12;
        RecipeDriver driver(configuration);
        driver.advance_to(OA_RECIPE_APPROACH);
        CHECK(driver.send(0.06, 0.2, 0.2, 1U) == OA_COMMISSION_ELIMIT);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_REAPPROACH);
        CHECK(driver.send(0.30, 0.0, 2.0, 1U) == OA_COMMISSION_ELIMIT);
        verify_no_commit(driver);
    }
    {
        auto configuration = recipe();
        configuration.repeatability_tolerance_rad = 0.001;
        RecipeDriver driver(configuration);
        driver.advance_to(OA_RECIPE_REAPPROACH);
        CHECK(driver.send(0.12, 0.005, 2.0, 1U) == OA_COMMISSION_OK);
        CHECK(driver.report.state == OA_RECIPE_REPEATABILITY);
        CHECK(driver.send(0.12, 0.0, 0.0, 0U) ==
              OA_COMMISSION_EREPEATABILITY);
        CHECK(driver.report.abort_reason == OA_ABORT_REPEATABILITY);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_CANDIDATE);
        CHECK(driver.send(0.12, 0.0, 0.0, 0U, 0U, OA_REVIEW_REJECT) ==
              OA_COMMISSION_ESTATE);
        CHECK(driver.report.abort_reason == OA_ABORT_REVIEW_REJECTED);
        verify_no_commit(driver);
    }
}

void test_false_contact_requires_prior_travel() {
    RecipeDriver driver(recipe());
    driver.advance_to(OA_RECIPE_APPROACH);
    CHECK(driver.send(0.01, 0.0, 2.0, 1U) == OA_COMMISSION_OK);
    CHECK(driver.report.state == OA_RECIPE_APPROACH);
    CHECK(driver.action.kind == OA_RECIPE_ACTION_APPROACH);
    verify_no_commit(driver);
}

void test_stale_and_temperature_faults() {
    {
        RecipeDriver driver(recipe());
        ++driver.sequence;
        driver.time += 100U;
        auto input = recipe_input(driver.sequence, driver.time - 60U, 0.0, 0.0, 0.0, 0U);
        input.now_ns = driver.time;
        auto action = output_record<oa_commission_next_action>();
        auto report = output_record<oa_commission_recipe_report>();
        CHECK(oa_commission_recipe_step(driver.session, &input, &action, &report) ==
              OA_COMMISSION_ESTALE);
        CHECK(report.state == OA_RECIPE_ABORT);
        CHECK(action.kind == OA_RECIPE_ACTION_ABORT_DISABLE);
        verify_no_commit(driver);
    }
    {
        RecipeDriver driver(recipe());
        driver.advance_to(OA_RECIPE_APPROACH);
        ++driver.sequence;
        driver.time += 10U;
        auto input = recipe_input(driver.sequence, driver.time, 0.01, 0.1, 0.0, 1U);
        input.encoder.mos_temperature_c = 61.0;
        auto action = output_record<oa_commission_next_action>();
        auto report = output_record<oa_commission_recipe_report>();
        CHECK(oa_commission_recipe_step(driver.session, &input, &action, &report) ==
              OA_COMMISSION_ELIMIT);
        verify_no_commit(driver);
    }
}

void test_abi_validation() {
    auto options = manual_options(1U, 1);
    options.abi_version = 999U;
    oa_commission_manual_session *session = nullptr;
    CHECK(oa_commission_manual_create(&options, &session) == OA_COMMISSION_EABI);
    options.abi_version = OA_COMMISSION_ABI_V1;
    options.struct_size = 4U;
    CHECK(oa_commission_manual_create(&options, &session) == OA_COMMISSION_EINVAL);
    CHECK(oa_commission_manual_abort(nullptr) == OA_COMMISSION_EINVAL);
    CHECK(oa_commission_recipe_abort(nullptr) == OA_COMMISSION_EINVAL);
}

}  // namespace

int main() {
    test_manual_known_sign();
    test_manual_two_reference_sign();
    test_manual_fail_closed();
    test_recipe_happy_path();
    test_unqualified_arm_recipe_rejected();
    test_abort_from_every_nonterminal_state();
    test_fault_from_every_nonterminal_state();
    test_faults_abort_and_never_commit();
    test_false_contact_requires_prior_travel();
    test_stale_and_temperature_faults();
    test_abi_validation();
    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "commission tests passed\n";
    return 0;
}
