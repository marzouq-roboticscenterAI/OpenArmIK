#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_control.h"
#include "openarm_runtime.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <future>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

extern "C" void oa_runtime_test_fail_allocation_after(std::int64_t countdown);
extern "C" oa_runtime_status oa_runtime_test_transport_raii_probe(void);
extern "C" int oa_runtime_test_hmac_sha256_known_vector(void);
extern "C" void oa_runtime_test_fail_fsync_after(std::int64_t countdown);

namespace {

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); \
    std::exit(1); } } while (false)

template <typename T>
void init(T &value) {
    value = {};
    value.struct_size = sizeof(value);
    value.abi_version = OA_RUNTIME_ABI_VERSION;
}

oa_runtime_options virtual_options() {
    oa_runtime_options options{};
    init(options);
    options.backend = OA_RUNTIME_BACKEND_VIRTUAL;
    options.allow_unchecked_virtual_motion = 1U;
    options.cycle_ns = 1000000U;
    options.feedback_timeout_ns = 50000000U;
    options.maximum_cross_bus_skew_ns = 1000000U;
    options.collision_scene_revision = 1U;
    return options;
}

void persistence_round_trip(oa_runtime_manifest *manifest) {
    char directory_template[] = "/tmp/openarm-runtime-test-XXXXXX";
    char *const directory = mkdtemp(directory_template);
    CHECK(directory != nullptr);
    const std::string path = std::string(directory) + "/manifest.oarm";
    std::uint8_t key[OA_RUNTIME_PERSISTENCE_KEY_BYTES]{};
    for (std::size_t index = 0U; index < sizeof(key); ++index) {
        key[index] = static_cast<std::uint8_t>(index + 1U);
    }
    oa_runtime_persistence_authority *authority = nullptr;
    CHECK(oa_runtime_persistence_authority_create("/tmp/..", key, "test-key-v1",
                                                  &authority) ==
          OA_RUNTIME_EPERMISSION);
    CHECK(oa_runtime_persistence_authority_create(directory, key, "test-key-v1",
                                                  &authority) == OA_RUNTIME_OK);
    CHECK(oa_runtime_manifest_save(manifest, nullptr, "manifest.oarm") ==
          OA_RUNTIME_EINVAL);
    CHECK(oa_runtime_manifest_save(manifest, authority, "../bad.oarm") ==
          OA_RUNTIME_EINVAL);
    CHECK(oa_runtime_manifest_save(manifest, authority, "manifest.oarm") == OA_RUNTIME_OK);
    oa_runtime_manifest_preview preview{};
    init(preview);
    oa_runtime_test_fail_allocation_after(0);
    CHECK(oa_runtime_manifest_preview_file(path.c_str(), &preview) == OA_RUNTIME_ENOMEM);
    oa_runtime_test_fail_allocation_after(-1);
    CHECK(oa_runtime_manifest_preview_file(path.c_str(), &preview) == OA_RUNTIME_OK);
    CHECK(preview.valid == 1U && preview.would_be_armable == 1U);
    oa_runtime_manifest *loaded = nullptr;
    CHECK(oa_runtime_manifest_load(path.c_str(), &loaded) == OA_RUNTIME_OK);
    oa_runtime_manifest_summary first{};
    oa_runtime_manifest_summary second{};
    init(first); init(second);
    CHECK(oa_runtime_manifest_get_summary(manifest, &first) == OA_RUNTIME_OK);
    CHECK(oa_runtime_manifest_get_summary(loaded, &second) == OA_RUNTIME_OK);
    CHECK(std::strcmp(first.content_sha256, second.content_sha256) == 0);
    CHECK(second.integrity_kind == OA_RUNTIME_INTEGRITY_HMAC_SHA256 &&
          second.authenticated == 0U);
    oa_runtime_manifest_destroy(loaded);

    CHECK(oa_runtime_manifest_load_authenticated(authority, "manifest.oarm", &loaded) ==
          OA_RUNTIME_OK);
    init(second);
    CHECK(oa_runtime_manifest_get_summary(loaded, &second) == OA_RUNTIME_OK);
    CHECK(second.authenticated == 1U &&
          std::strcmp(second.authentication_key_id, "test-key-v1") == 0);
    oa_runtime_manifest_destroy(loaded);

    std::uint8_t wrong_key[OA_RUNTIME_PERSISTENCE_KEY_BYTES]{};
    wrong_key[0] = 1U;
    oa_runtime_persistence_authority *wrong_authority = nullptr;
    CHECK(oa_runtime_persistence_authority_create(directory, wrong_key, "test-key-v1",
                                                  &wrong_authority) == OA_RUNTIME_OK);
    CHECK(oa_runtime_manifest_load_authenticated(wrong_authority, "manifest.oarm", &loaded) ==
          OA_RUNTIME_EPERMISSION);
    oa_runtime_persistence_authority_destroy(wrong_authority);

    const std::string symlink_path = std::string(directory) + "/link.oarm";
    CHECK(symlink(path.c_str(), symlink_path.c_str()) == 0);
    CHECK(oa_runtime_manifest_load(symlink_path.c_str(), &loaded) == OA_RUNTIME_EPERMISSION);
    CHECK(oa_runtime_manifest_save(manifest, authority, "link.oarm") ==
          OA_RUNTIME_EPERMISSION);
    CHECK(unlink(symlink_path.c_str()) == 0);

    const int file = open(path.c_str(), O_WRONLY | O_TRUNC | O_CLOEXEC);
    CHECK(file >= 0);
    static constexpr char corrupt[] = "OPENARM_RUNTIME_MANIFEST|1\n";
    CHECK(write(file, corrupt, sizeof(corrupt) - 1U) ==
          static_cast<ssize_t>(sizeof(corrupt) - 1U));
    CHECK(close(file) == 0);
    CHECK(oa_runtime_manifest_load(path.c_str(), &loaded) == OA_RUNTIME_ECORRUPT);
    CHECK(oa_runtime_manifest_load_authenticated(authority, "manifest.oarm", &loaded) ==
          OA_RUNTIME_ECORRUPT);
    oa_runtime_persistence_authority_destroy(authority);
    oa_runtime_persistence_authority_destroy(authority);
    CHECK(unlink(path.c_str()) == 0);
    CHECK(rmdir(directory) == 0);
}

oa_runtime_manifest *manual_calibration(oa_runtime *runtime, oa_runtime_manifest *base,
                                        std::uint32_t joint = 0U) {
    oa_commission_manual_options options{};
    options.struct_size = sizeof(options);
    options.abi_version = OA_COMMISSION_ABI_V1;
    options.side = 0U;
    options.joint = joint;
    options.expected_revision = 1U;
    options.reference_count = 1U;
    options.known_sign = 1;
    options.minimum_samples = 2U;
    options.maximum_sample_age_ns = 1000000000U;
    options.stability_dwell_ns = 1U;
    options.reference_model_rad[0] = 0.0;
    options.maximum_position_spread_rad = 0.001;
    options.maximum_abs_velocity_rad_s = 0.1;
    options.minimum_reference_separation_rad = 0.1;
    options.maximum_scale_error = 0.01;
    std::snprintf(options.motor_serial, sizeof(options.motor_serial), "VIRTUAL-0-%u",
                  joint);
    oa_runtime_calibration *session = nullptr;
    CHECK(oa_runtime_calibration_manual_begin(runtime, &options, &session) == OA_RUNTIME_OK);
    CHECK(oa_runtime_arm_virtual(runtime) == OA_RUNTIME_EBUSY);
    oa_commission_manual_report report{};
    report.struct_size = sizeof(report);
    report.abi_version = OA_COMMISSION_ABI_V1;
    for (unsigned accepted = 0U; accepted < 2U; ++accepted) {
        oa_runtime_status sample_status = OA_RUNTIME_ESTALE;
        for (unsigned attempt = 0U; attempt < 100U && sample_status != OA_RUNTIME_OK;
             ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            sample_status =
                oa_runtime_calibration_manual_sample(session, 0U, &report);
            CHECK(sample_status == OA_RUNTIME_OK || sample_status == OA_RUNTIME_ESTALE);
        }
        CHECK(sample_status == OA_RUNTIME_OK);
    }
    CHECK(oa_runtime_calibration_manual_begin_review(session, &report) == OA_RUNTIME_OK);
    oa_runtime_manifest *updated = nullptr;
    oa_runtime_manifest_preview preview{};
    init(preview);
    CHECK(oa_runtime_calibration_manual_commit(session, 2U, "fixture_verified",
                                                &updated, &preview) == OA_RUNTIME_OK);
    CHECK(preview.valid == 1U &&
          preview.mapping_change_mask == (UINT32_C(1) << joint) &&
          preview.result_revision == 2U);
    oa_runtime_manifest_summary summary{};
    init(summary);
    CHECK(oa_runtime_manifest_get_summary(updated, &summary) == OA_RUNTIME_OK);
    CHECK(summary.manifest_revision == 2U);
    oa_runtime_calibration_destroy(session);
    CHECK(oa_runtime_configuration_apply_physical(runtime, base) ==
          OA_RUNTIME_EUNSUPPORTED);
    return updated;
}

void persistence_revision_order(oa_runtime_manifest *base,
                                oa_runtime_manifest *updated) {
    char directory_template[] = "/tmp/openarm-runtime-revision-test-XXXXXX";
    char *const directory = mkdtemp(directory_template);
    CHECK(directory != nullptr);
    std::uint8_t key[OA_RUNTIME_PERSISTENCE_KEY_BYTES]{};
    key[0] = 0xa5U;
    key[31] = 0x5aU;
    oa_runtime_persistence_authority *authority = nullptr;
    CHECK(oa_runtime_persistence_authority_create(directory, key, "revision-key",
                                                  &authority) == OA_RUNTIME_OK);
    CHECK(oa_runtime_manifest_save(base, authority, "manifest.oarm") == OA_RUNTIME_OK);
    oa_runtime_test_fail_fsync_after(0);
    CHECK(oa_runtime_manifest_save(updated, authority, "manifest.oarm") ==
          OA_RUNTIME_EIO);
    oa_runtime_test_fail_fsync_after(-1);
    oa_runtime_manifest *loaded = nullptr;
    CHECK(oa_runtime_manifest_load_authenticated(authority, "manifest.oarm", &loaded) ==
          OA_RUNTIME_OK);
    oa_runtime_manifest_summary summary{};
    init(summary);
    CHECK(oa_runtime_manifest_get_summary(loaded, &summary) == OA_RUNTIME_OK);
    CHECK(summary.manifest_revision == 1U);
    oa_runtime_manifest_destroy(loaded);

    oa_runtime_test_fail_fsync_after(2);
    CHECK(oa_runtime_manifest_save(updated, authority, "manifest.oarm") ==
          OA_RUNTIME_EIO);
    oa_runtime_test_fail_fsync_after(-1);
    CHECK(oa_runtime_manifest_load_authenticated(authority, "manifest.oarm", &loaded) ==
          OA_RUNTIME_OK);
    init(summary);
    CHECK(oa_runtime_manifest_get_summary(loaded, &summary) == OA_RUNTIME_OK);
    CHECK(summary.manifest_revision == 1U);
    oa_runtime_manifest_destroy(loaded);
    CHECK(oa_runtime_manifest_save(updated, authority, "manifest.oarm") == OA_RUNTIME_OK);
    CHECK(oa_runtime_manifest_save(base, authority, "manifest.oarm") == OA_RUNTIME_ESTALE);
    CHECK(oa_runtime_manifest_load_authenticated(authority, "manifest.oarm", &loaded) ==
          OA_RUNTIME_OK);
    init(summary);
    CHECK(oa_runtime_manifest_get_summary(loaded, &summary) == OA_RUNTIME_OK);
    CHECK(summary.manifest_revision == 2U && summary.authenticated == 1U);
    oa_runtime_manifest_destroy(loaded);
    CHECK(oa_runtime_manifest_load_authenticated(
              authority, "manifest.oarm.previous", &loaded) == OA_RUNTIME_ESTALE);
    CHECK(loaded == nullptr);
    const std::string rollback = std::string(directory) + "/rollback.oarm";
    const std::string previous_path =
        std::string(directory) + "/manifest.oarm.previous";
    CHECK(link(previous_path.c_str(), rollback.c_str()) == 0);
    CHECK(oa_runtime_manifest_load_authenticated(authority, "rollback.oarm", &loaded) ==
          OA_RUNTIME_ESTALE);
    oa_runtime_persistence_authority *reopened_authority = nullptr;
    CHECK(oa_runtime_persistence_authority_create(directory, key, "revision-key",
                                                  &reopened_authority) ==
          OA_RUNTIME_OK);
    CHECK(oa_runtime_manifest_load_authenticated(
              reopened_authority, "rollback.oarm", &loaded) == OA_RUNTIME_ESTALE);
    CHECK(oa_runtime_manifest_load_authenticated(
              reopened_authority, "manifest.oarm", &loaded) == OA_RUNTIME_OK);
    oa_runtime_manifest_destroy(loaded);
    oa_runtime_persistence_authority_destroy(reopened_authority);
    CHECK(oa_runtime_manifest_load(rollback.c_str(), &loaded) == OA_RUNTIME_OK);
    oa_runtime *rollback_runtime = nullptr;
    const oa_runtime_options options = virtual_options();
    CHECK(oa_runtime_create(&options, loaded, &rollback_runtime) ==
          OA_RUNTIME_EPERMISSION);
    oa_runtime_manifest_destroy(loaded);
    oa_runtime_persistence_authority_destroy(authority);
    const std::string current = std::string(directory) + "/manifest.oarm";
    const std::string previous = current + ".previous";
    CHECK(unlink(current.c_str()) == 0);
    CHECK(unlink(previous.c_str()) == 0);
    CHECK(unlink(rollback.c_str()) == 0);
    CHECK(rmdir(directory) == 0);
}

void concurrent_persistence_conflict(oa_runtime_manifest *base,
                                     oa_runtime_manifest *first,
                                     oa_runtime_manifest *second) {
    char directory_template[] = "/tmp/openarm-runtime-race-test-XXXXXX";
    char *const directory = mkdtemp(directory_template);
    CHECK(directory != nullptr);
    std::uint8_t key[OA_RUNTIME_PERSISTENCE_KEY_BYTES]{};
    key[0] = 0x31U;
    key[31] = 0x73U;
    oa_runtime_persistence_authority *authority = nullptr;
    CHECK(oa_runtime_persistence_authority_create(directory, key, "race-key",
                                                  &authority) == OA_RUNTIME_OK);
    CHECK(oa_runtime_manifest_save(base, authority, "manifest.oarm") == OA_RUNTIME_OK);
    oa_runtime_status first_save = OA_RUNTIME_EIO;
    oa_runtime_status second_save = OA_RUNTIME_EIO;
    std::thread first_writer([&] {
        first_save = oa_runtime_manifest_save(first, authority, "manifest.oarm");
    });
    std::thread second_writer([&] {
        second_save = oa_runtime_manifest_save(second, authority, "manifest.oarm");
    });
    first_writer.join();
    second_writer.join();
    CHECK((first_save == OA_RUNTIME_OK && second_save == OA_RUNTIME_ESTALE) ||
          (second_save == OA_RUNTIME_OK && first_save == OA_RUNTIME_ESTALE));
    oa_runtime_manifest *loaded = nullptr;
    CHECK(oa_runtime_manifest_load_authenticated(authority, "manifest.oarm", &loaded) ==
          OA_RUNTIME_OK);
    oa_runtime_manifest_summary summary{};
    init(summary);
    CHECK(oa_runtime_manifest_get_summary(loaded, &summary) == OA_RUNTIME_OK);
    CHECK(summary.manifest_revision == 2U && summary.authenticated == 1U);
    oa_runtime_manifest_destroy(loaded);
    CHECK(oa_runtime_manifest_load_authenticated(
              authority, "manifest.oarm.previous", &loaded) == OA_RUNTIME_ESTALE);
    oa_runtime_persistence_authority_destroy(authority);
    const std::string current = std::string(directory) + "/manifest.oarm";
    const std::string previous = current + ".previous";
    CHECK(unlink(current.c_str()) == 0);
    CHECK(unlink(previous.c_str()) == 0);
    CHECK(rmdir(directory) == 0);
}

void invalid_calibration_does_not_deadlock(oa_runtime *runtime) {
    oa_commission_manual_options manual{};
    manual.struct_size = sizeof(manual);
    manual.abi_version = OA_COMMISSION_ABI_V1;
    manual.expected_revision = 1U;
    manual.known_sign = 1;
    std::snprintf(manual.motor_serial, sizeof(manual.motor_serial), "VIRTUAL-0-0");
    oa_runtime_calibration *session = nullptr;
    auto manual_result = std::async(std::launch::async, [&] {
        return oa_runtime_calibration_manual_begin(runtime, &manual, &session);
    });
    CHECK(manual_result.wait_for(std::chrono::milliseconds(500)) ==
          std::future_status::ready);
    CHECK(manual_result.get() == OA_RUNTIME_EINVAL);
    CHECK(session == nullptr);
    oa_runtime_error_detail detail{};
    init(detail);
    CHECK(oa_runtime_get_last_error(runtime, &detail) == OA_RUNTIME_OK);
    CHECK(detail.status == OA_RUNTIME_EINVAL &&
          detail.facility == OA_RUNTIME_FACILITY_COMMISSION &&
          detail.lower_code == static_cast<std::uint32_t>(OA_COMMISSION_EINVAL));

    oa_commission_recipe recipe{};
    recipe.struct_size = sizeof(recipe);
    recipe.abi_version = OA_COMMISSION_ABI_V1;
    recipe.expected_revision = 1U;
    std::snprintf(recipe.motor_serial, sizeof(recipe.motor_serial), "VIRTUAL-0-0");
    auto recipe_result = std::async(std::launch::async, [&] {
        return oa_runtime_calibration_recipe_begin(runtime, &recipe, &session);
    });
    CHECK(recipe_result.wait_for(std::chrono::milliseconds(500)) ==
          std::future_status::ready);
    CHECK(recipe_result.get() == OA_RUNTIME_EINVAL);
    CHECK(session == nullptr);
    init(detail);
    CHECK(oa_runtime_get_last_error(runtime, &detail) == OA_RUNTIME_OK);
    CHECK(detail.status == OA_RUNTIME_EINVAL &&
          detail.facility == OA_RUNTIME_FACILITY_COMMISSION &&
          detail.lower_code == static_cast<std::uint32_t>(OA_COMMISSION_EINVAL));
}

void supervised_calibration(oa_runtime *runtime) {
    oa_runtime_snapshot snapshot{};
    init(snapshot);
    CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
    oa_commission_recipe recipe{};
    recipe.struct_size = sizeof(recipe);
    recipe.abi_version = OA_COMMISSION_ABI_V1;
    recipe.recipe_kind = OA_RECIPE_ARM_JOINT;
    recipe.side = OA_COMMISSION_LEFT;
    recipe.joint = 0U;
    recipe.known_sign = 1;
    recipe.hardware_qualified = 1U;
    recipe.minimum_contact_samples = 2U;
    recipe.expected_revision = 1U;
    recipe.qualification_revision = 1U;
    recipe.fixture_revision = 1U;
    recipe.maximum_sample_age_ns = 1000000000U;
    recipe.maximum_approach_time_ns = 1000000000U;
    recipe.contact_dwell_ns = 1000000U;
    recipe.maximum_retreat_time_ns = 1000000000U;
    recipe.stop_model_rad = 0.0;
    recipe.approach_direction = 1.0;
    recipe.start_min_output_rad = -1.0;
    recipe.start_max_output_rad = 1.0;
    recipe.maximum_speed_rad_s = 1.0;
    recipe.minimum_contact_travel_rad = 0.05;
    recipe.maximum_approach_travel_rad = 0.2;
    recipe.contact_velocity_rad_s = 0.01;
    recipe.minimum_contact_torque_nm = 1.0;
    recipe.maximum_torque_nm = 3.0;
    recipe.maximum_contact_energy_j = 1.0;
    recipe.maximum_temperature_c = 60.0;
    recipe.retreat_distance_rad = 0.05;
    recipe.repeatability_tolerance_rad = 0.02;
    recipe.required_posture_mask = 0x7eU;
    recipe.posture_tolerance_rad = 0.1;
    for (std::size_t joint = 0U; joint < 7U; ++joint) {
        recipe.required_posture_output_rad[joint] = snapshot.arm[0].q_output_rad[joint];
    }
    std::snprintf(recipe.motor_serial, sizeof(recipe.motor_serial), "VIRTUAL-0-0");
    std::snprintf(recipe.qualification_record, sizeof(recipe.qualification_record),
                  "virtual_qualified_recipe");
    std::snprintf(recipe.fixture_record, sizeof(recipe.fixture_record), "virtual_fixture");
    oa_runtime_calibration *session = nullptr;
    CHECK(oa_runtime_calibration_recipe_begin(runtime, &recipe, &session) == OA_RUNTIME_OK);
    oa_commission_recipe_input input{};
    input.struct_size = sizeof(input);
    input.abi_version = OA_COMMISSION_ABI_V1;
    input.estop_clear = 1U;
    input.deadman_held = 1U;
    input.evidence_revision = UINT64_MAX;
    input.fixture_revision = UINT64_MAX;
    input.posture_mask = 0U;
    for (std::size_t joint = 0U; joint < 7U; ++joint) {
        input.posture_output_rad[joint] = 1000.0;
    }
    oa_commission_next_action action{};
    action.struct_size = sizeof(action);
    action.abi_version = OA_COMMISSION_ABI_V1;
    oa_commission_recipe_report report{};
    report.struct_size = sizeof(report);
    report.abi_version = OA_COMMISSION_ABI_V1;
    CHECK(oa_runtime_calibration_recipe_step(session, &input, &action, &report) ==
          OA_RUNTIME_OK);
    CHECK(action.kind == OA_RECIPE_ACTION_HOLD_DISABLED);
    input.operator_ready = 1U;
    input.estop_clear = 1U;
    input.deadman_held = 1U;
    oa_runtime_status step_status = OA_RUNTIME_ESTALE;
    for (unsigned attempt = 0U; attempt < 100U && step_status != OA_RUNTIME_OK; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        step_status = oa_runtime_calibration_recipe_step(session, &input, &action, &report);
        CHECK(step_status == OA_RUNTIME_OK || step_status == OA_RUNTIME_ESTALE);
    }
    CHECK(step_status == OA_RUNTIME_OK);
    CHECK(action.kind == OA_RECIPE_ACTION_HOLD_DISABLED);
    CHECK(oa_runtime_set_interlock(runtime, 0U, 1U) == OA_RUNTIME_OK);
    step_status = OA_RUNTIME_ESTALE;
    for (unsigned attempt = 0U; attempt < 100U && step_status != OA_RUNTIME_OK; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        step_status = oa_runtime_calibration_recipe_step(session, &input, &action, &report);
        CHECK(step_status == OA_RUNTIME_OK || step_status == OA_RUNTIME_ESTALE);
    }
    CHECK(step_status == OA_RUNTIME_OK);
    CHECK(action.kind == OA_RECIPE_ACTION_APPROACH);
    oa_runtime_status first_abort = OA_RUNTIME_OK;
    oa_runtime_status second_abort = OA_RUNTIME_OK;
    std::thread first([&] { first_abort = oa_runtime_calibration_abort(session); });
    std::thread second([&] { second_abort = oa_runtime_calibration_abort(session); });
    first.join();
    second.join();
    CHECK((first_abort == OA_RUNTIME_OK && second_abort == OA_RUNTIME_ESTATE) ||
          (second_abort == OA_RUNTIME_OK && first_abort == OA_RUNTIME_ESTATE));
    oa_runtime_calibration_destroy(session);
}

void execute_and_wait(oa_runtime *runtime, oa_runtime_plan *plan,
                      std::uint64_t horizon_ns) {
    std::uint64_t now = 0U;
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
          OA_RUNTIME_OK);
    oa_runtime_execute_request request{};
    init(request);
    request.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    request.start_runtime_monotonic_ns = now + 1000000U;
    const std::uint64_t execution_horizon = horizon_ns - 100000000U;
    request.expiry_runtime_monotonic_ns = now + execution_horizon;
    request.producer_deadline_runtime_monotonic_ns = now + execution_horizon;
    request.stop_kind = OA_RUNTIME_STOP_DISABLE;
    std::uint64_t command = 0U;
    CHECK(oa_runtime_execute(runtime, plan, &request, &command) == OA_RUNTIME_OK);
    const auto wall_deadline = std::chrono::steady_clock::now() +
                               std::chrono::nanoseconds(horizon_ns);
    bool completed = false;
    while (std::chrono::steady_clock::now() < wall_deadline && !completed) {
        oa_runtime_event event{};
        init(event);
        const oa_runtime_status status = oa_runtime_poll_event(runtime, 50000000U, &event);
        CHECK(status == OA_RUNTIME_OK || status == OA_RUNTIME_ETIMEOUT);
        if (status == OA_RUNTIME_OK && event.kind == OA_RUNTIME_EVENT_COMPLETED &&
            event.command_id == command) {
            CHECK(event.feedback_seq_valid_mask == 0U);
            CHECK(event.measurement_timestamp_valid == 0U);
            CHECK(event.feedback_seq[0] == 0U && event.feedback_seq[1] == 0U);
            CHECK(event.source_feedback_seq != 0U);
            std::uint64_t event_now = 0U;
            CHECK(oa_runtime_now_monotonic_ns(
                      runtime, OA_RUNTIME_CLOCK_MONOTONIC, &event_now) == OA_RUNTIME_OK);
            CHECK(event.event_runtime_monotonic_ns > 0U &&
                  event.event_runtime_monotonic_ns <= event_now &&
                  event_now - event.event_runtime_monotonic_ns < 100000000U);
            completed = true;
        }
    }
    CHECK(completed);
}

}

int main() {
    CHECK(oa_runtime_test_transport_raii_probe() == OA_RUNTIME_OK);
    CHECK(oa_runtime_test_hmac_sha256_known_vector() == 1);
    oa_runtime_manifest *manifest = nullptr;
    CHECK(oa_runtime_manifest_create_virtual(&manifest) == OA_RUNTIME_OK);
    oa_runtime_manifest_summary manifest_summary{};
    init(manifest_summary);
    CHECK(oa_runtime_manifest_get_summary(manifest, &manifest_summary) == OA_RUNTIME_OK);
    CHECK(manifest_summary.state == OA_RUNTIME_MANIFEST_ARMABLE);
    persistence_round_trip(manifest);

    const auto options = virtual_options();
    oa_runtime *runtime = nullptr;
    CHECK(oa_runtime_create(&options, manifest, &runtime) == OA_RUNTIME_OK);
    oa_runtime_capability_report capabilities{};
    init(capabilities);
    CHECK(oa_runtime_get_capabilities(runtime, &capabilities) == OA_RUNTIME_OK);
    CHECK((capabilities.capabilities & OA_RUNTIME_CAP_VIRTUAL_JOINT_MOTION) != 0U);
    CHECK((capabilities.capabilities & OA_RUNTIME_CAP_PHYSICAL_MOTION) == 0U);
    CHECK((capabilities.capabilities & (OA_RUNTIME_CAP_MODEL_FK |
                                        OA_RUNTIME_CAP_SINGLE_XYZ_IK |
                                        OA_RUNTIME_CAP_PAIRED_XYZ_IK)) == 0U);
    CHECK(std::strlen(capabilities.coordinate_identity_sha256) == 64U);
    oa_runtime_model_identity left_identity{};
    oa_runtime_model_identity right_identity{};
    init(left_identity);
    init(right_identity);
    CHECK(oa_runtime_get_model_identity(runtime, 0U, &left_identity) == OA_RUNTIME_OK);
    CHECK(oa_runtime_get_model_identity(runtime, 1U, &right_identity) == OA_RUNTIME_OK);
    CHECK(std::strcmp(left_identity.model_id, right_identity.model_id) != 0);
    CHECK(std::strlen(left_identity.model_data_sha256) == 64U &&
          std::strlen(left_identity.flattened_urdf_sha256) == 64U &&
          std::strlen(left_identity.tcp_frame) != 0U);
    CHECK(std::strcmp(left_identity.coordinate_identity_sha256,
                      capabilities.coordinate_identity_sha256) == 0);
    CHECK(oa_runtime_get_model_identity(runtime, 2U, &left_identity) ==
          OA_RUNTIME_EINVAL);

    oa_runtime_inventory *inventory = nullptr;
    CHECK(oa_runtime_inventory_query(runtime, nullptr, &inventory) == OA_RUNTIME_OK);
    oa_runtime_inventory_summary inventory_summary{};
    init(inventory_summary);
    CHECK(oa_runtime_inventory_get_summary(inventory, &inventory_summary) == OA_RUNTIME_OK);
    CHECK(inventory_summary.interface_count == 2U && inventory_summary.motor_count == 14U);
    oa_runtime_motor_evidence motor{};
    init(motor);
    CHECK(oa_runtime_inventory_get_motor(inventory, 13U, &motor) == OA_RUNTIME_OK);
    CHECK(motor.side == 1U && motor.joint == 6U &&
          motor.confidence == OA_RUNTIME_EVIDENCE_VIRTUAL_EXACT);
    std::uint64_t inventory_now = 0U;
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC,
                                      &inventory_now) == OA_RUNTIME_OK);
    CHECK(motor.query_sent_runtime_monotonic_ns > 1U &&
          motor.query_sent_runtime_monotonic_ns <= inventory_now &&
          motor.response_runtime_monotonic_ns ==
              motor.query_sent_runtime_monotonic_ns);
    CHECK(oa_runtime_inventory_get_summary(
              reinterpret_cast<const oa_runtime_inventory *>(manifest),
              &inventory_summary) == OA_RUNTIME_EINVAL);

    oa_runtime_snapshot snapshot{};
    init(snapshot);
    CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
    oa_runtime_kinematics kinematics{};
    init(kinematics);
    CHECK(oa_runtime_get_kinematics(runtime, 0U, snapshot.arm[0].feedback_seq,
                                    &kinematics) == OA_RUNTIME_OK);
    CHECK(kinematics.frame_id == OA_RUNTIME_FRAME_OPENARM_BODY_LINK0);

    invalid_calibration_does_not_deadlock(runtime);
    supervised_calibration(runtime);
    oa_runtime_manifest *updated_manifest = manual_calibration(runtime, manifest);
    oa_runtime_manifest *conflicting_manifest = manual_calibration(runtime, manifest, 1U);
    concurrent_persistence_conflict(manifest, updated_manifest,
                                    conflicting_manifest);
    persistence_revision_order(manifest, updated_manifest);
    oa_runtime_manifest_destroy(updated_manifest);
    oa_runtime_manifest_destroy(conflicting_manifest);
    CHECK(oa_runtime_set_interlock(runtime, 2U, 1U) == OA_RUNTIME_EINVAL);
    oa_runtime_error_detail semantic_detail{};
    init(semantic_detail);
    CHECK(oa_runtime_get_last_error(runtime, &semantic_detail) == OA_RUNTIME_OK);
    CHECK(semantic_detail.status == OA_RUNTIME_EINVAL &&
          semantic_detail.facility == OA_RUNTIME_FACILITY_RUNTIME &&
          semantic_detail.lower_code == 0U);
    CHECK(oa_runtime_now_monotonic_ns(runtime,
                                      static_cast<oa_runtime_clock_id>(0U), nullptr) ==
          OA_RUNTIME_EINVAL);
    init(semantic_detail);
    CHECK(oa_runtime_get_last_error(runtime, &semantic_detail) == OA_RUNTIME_OK);
    CHECK(semantic_detail.status == OA_RUNTIME_EINVAL &&
          semantic_detail.facility == OA_RUNTIME_FACILITY_RUNTIME &&
          semantic_detail.lower_code == 0U);
    CHECK(oa_runtime_set_interlock(runtime, 0U, 1U) == OA_RUNTIME_OK);
    CHECK(oa_runtime_arm_virtual(runtime) == OA_RUNTIME_OK);
    oa_runtime_joint_move move{};
    init(move);
    move.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    move.units_id = OA_RUNTIME_UNITS_SI_V1;
    std::uint64_t now = 0U;
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
          OA_RUNTIME_OK);
    move.expiry_runtime_monotonic_ns = now + 5000000000ULL;
    init(snapshot);
    CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
    move.required_feedback_seq = snapshot.arm[0].feedback_seq - 1U;
    move.side = 0U;
    move.joint = 0U;
    move.target_model_rad = snapshot.arm[0].q_model_rad[0] + 0.01;
    move.velocity_scale = 1.0;
    move.acceleration_scale = 1.0;
    move.jerk_scale = 1.0;
    move.position_tolerance_rad = 0.001;
    move.velocity_tolerance_rad_s = 0.02;
    move.required_model_revision = left_identity.model_revision;
    move.required_tcp_revision = left_identity.tcp_revision;
    move.collision_scene_revision = left_identity.collision_scene_revision;
    move.required_collision_policy = left_identity.collision_policy;
    std::snprintf(move.required_coordinate_identity_sha256,
                  sizeof(move.required_coordinate_identity_sha256), "%s",
                  left_identity.coordinate_identity_sha256);
    oa_runtime_plan *plan = nullptr;
    oa_runtime_options reject_options = options;
    reject_options.allow_unchecked_virtual_motion = 0U;
    oa_runtime *reject_runtime = nullptr;
    CHECK(oa_runtime_create(&reject_options, manifest, &reject_runtime) == OA_RUNTIME_OK);
    CHECK(oa_runtime_set_interlock(reject_runtime, 0U, 1U) == OA_RUNTIME_OK);
    CHECK(oa_runtime_arm_virtual(reject_runtime) == OA_RUNTIME_OK);
    oa_runtime_snapshot reject_snapshot{};
    init(reject_snapshot);
    CHECK(oa_runtime_snapshot_get(reject_runtime, &reject_snapshot) == OA_RUNTIME_OK);
    std::uint64_t reject_now = 0U;
    CHECK(oa_runtime_now_monotonic_ns(
              reject_runtime, OA_RUNTIME_CLOCK_MONOTONIC, &reject_now) == OA_RUNTIME_OK);
    oa_runtime_joint_move cross_runtime_move = move;
    cross_runtime_move.required_feedback_seq = reject_snapshot.arm[0].feedback_seq;
    cross_runtime_move.expiry_runtime_monotonic_ns = reject_now + 5000000000ULL;
    oa_runtime_plan *cross_runtime_plan = nullptr;
    CHECK(oa_runtime_plan_joint(reject_runtime, &cross_runtime_move,
                                &cross_runtime_plan) == OA_RUNTIME_EINVAL);
    cross_runtime_move.required_collision_policy = OA_RUNTIME_COLLISION_REJECT_ALL;
    CHECK(oa_runtime_plan_joint(reject_runtime, &cross_runtime_move,
                                &cross_runtime_plan) == OA_RUNTIME_EIDENTITY);
    CHECK(cross_runtime_plan == nullptr);
    oa_runtime_destroy(reject_runtime);
    const oa_runtime_clock_id valid_clock = move.clock_id;
    move.clock_id = 0U;
    CHECK(oa_runtime_plan_joint(runtime, &move, &plan) == OA_RUNTIME_EINVAL);
    move.clock_id = valid_clock;
    move.required_coordinate_identity_sha256[0] =
        move.required_coordinate_identity_sha256[0] == '0' ? '1' : '0';
    CHECK(oa_runtime_plan_joint(runtime, &move, &plan) == OA_RUNTIME_EIDENTITY);
    std::snprintf(move.required_coordinate_identity_sha256,
                  sizeof(move.required_coordinate_identity_sha256), "%s",
                  left_identity.coordinate_identity_sha256);
    CHECK(oa_runtime_plan_joint(runtime, &move, &plan) == OA_RUNTIME_ESTALE);
    oa_runtime_status fresh_plan_status = OA_RUNTIME_ESTALE;
    for (unsigned attempt = 0U;
         attempt < 100U && fresh_plan_status == OA_RUNTIME_ESTALE; ++attempt) {
        init(snapshot);
        CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
        CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
              OA_RUNTIME_OK);
        move.required_feedback_seq = snapshot.arm[0].feedback_seq;
        move.target_model_rad = snapshot.arm[0].q_model_rad[0] + 0.01;
        move.expiry_runtime_monotonic_ns = now + 5000000000ULL;
        fresh_plan_status = oa_runtime_plan_joint(runtime, &move, &plan);
    }
    CHECK(fresh_plan_status == OA_RUNTIME_OK);
    oa_runtime_plan *second_plan = nullptr;
    CHECK(oa_runtime_plan_joint(runtime, &move, &second_plan) == OA_RUNTIME_EBUSY);
    CHECK(second_plan == nullptr);
    std::uint64_t plan_clock_before = 0U;
    std::uint64_t plan_clock_after = 0U;
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC,
                                      &plan_clock_before) == OA_RUNTIME_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC,
                                      &plan_clock_after) == OA_RUNTIME_OK);
    CHECK(plan_clock_after > plan_clock_before);
    oa_runtime_error_detail plan_detail{};
    init(plan_detail);
    CHECK(oa_runtime_get_last_error(runtime, &plan_detail) == OA_RUNTIME_OK);
    CHECK(plan_detail.status == OA_RUNTIME_EBUSY &&
          plan_detail.facility == OA_RUNTIME_FACILITY_RUNTIME);
    oa_runtime_plan_report plan_report{};
    init(plan_report);
    CHECK(oa_runtime_plan_get_report(plan, &plan_report) == OA_RUNTIME_OK);
    CHECK(plan_report.collision_checked == 0U && plan_report.motion_authorized == 0U &&
          plan_report.model_revision == move.required_model_revision &&
          plan_report.tcp_revision[move.side] == move.required_tcp_revision &&
          plan_report.collision_scene_revision == move.collision_scene_revision &&
          plan_report.collision_policy == move.required_collision_policy &&
          std::strcmp(plan_report.coordinate_identity_sha256,
                      move.required_coordinate_identity_sha256) == 0);
    const std::uint64_t paused_feedback_seq = snapshot.arm[0].feedback_seq;
    const std::uint64_t paused_measurement_time =
        snapshot.arm[0].measurement_runtime_monotonic_ns;
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
    init(snapshot);
    CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
          OA_RUNTIME_OK);
    CHECK(snapshot.arm[0].feedback_seq == paused_feedback_seq &&
          snapshot.arm[0].measurement_runtime_monotonic_ns ==
              paused_measurement_time &&
          snapshot.arm[0].fresh_mask == 0U &&
          now - snapshot.arm[0].measurement_runtime_monotonic_ns >=
              options.feedback_timeout_ns);
    oa_runtime_plan_destroy(plan);
    plan = nullptr;
    bool feedback_resumed = false;
    for (unsigned attempt = 0U; attempt < 100U && !feedback_resumed; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        init(snapshot);
        CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
        CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
              OA_RUNTIME_OK);
        feedback_resumed = snapshot.arm[0].feedback_seq > paused_feedback_seq &&
                           snapshot.arm[0].fresh_mask == 0x7fU &&
                           snapshot.arm[0].measurement_runtime_monotonic_ns >=
                               paused_measurement_time &&
                           now - snapshot.arm[0].measurement_runtime_monotonic_ns <
                               options.feedback_timeout_ns;
    }
    CHECK(feedback_resumed);
    fresh_plan_status = OA_RUNTIME_ESTALE;
    for (unsigned attempt = 0U;
         attempt < 100U && fresh_plan_status == OA_RUNTIME_ESTALE; ++attempt) {
        move.required_feedback_seq = snapshot.arm[0].feedback_seq;
        move.target_model_rad = snapshot.arm[0].q_model_rad[0] + 0.01;
        move.expiry_runtime_monotonic_ns = now + 5000000000ULL;
        fresh_plan_status = oa_runtime_plan_joint(runtime, &move, &plan);
        if (fresh_plan_status == OA_RUNTIME_ESTALE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            init(snapshot);
            CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
            CHECK(oa_runtime_now_monotonic_ns(
                      runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) == OA_RUNTIME_OK);
        }
    }
    CHECK(fresh_plan_status == OA_RUNTIME_OK);
    execute_and_wait(runtime, plan, 5000000000ULL);
    oa_runtime_plan_destroy(plan);
    oa_runtime_plan_destroy(plan);

    CHECK(oa_runtime_heartbeat(runtime, 0U,
                               static_cast<oa_runtime_clock_id>(0U), now + 1U) ==
          OA_RUNTIME_EINVAL);
    init(semantic_detail);
    CHECK(oa_runtime_get_last_error(runtime, &semantic_detail) == OA_RUNTIME_OK);
    CHECK(semantic_detail.status == OA_RUNTIME_EINVAL &&
          semantic_detail.facility == OA_RUNTIME_FACILITY_RUNTIME &&
          semantic_detail.lower_code == 0U);
    CHECK(oa_runtime_disarm(runtime, static_cast<oa_runtime_clock_id>(0U), now + 1U) ==
          OA_RUNTIME_EINVAL);
    init(semantic_detail);
    CHECK(oa_runtime_get_last_error(runtime, &semantic_detail) == OA_RUNTIME_OK);
    CHECK(semantic_detail.status == OA_RUNTIME_EINVAL &&
          semantic_detail.facility == OA_RUNTIME_FACILITY_RUNTIME &&
          semantic_detail.lower_code == 0U);

    init(snapshot);
    CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
          OA_RUNTIME_OK);
    move.required_feedback_seq = snapshot.arm[0].feedback_seq;
    move.target_model_rad = snapshot.arm[0].q_model_rad[0];
    move.expiry_runtime_monotonic_ns = now + 5000000U;
    oa_runtime_plan *expired_plan = nullptr;
    CHECK(oa_runtime_plan_joint(runtime, &move, &expired_plan) == OA_RUNTIME_OK);
    oa_runtime_plan *replacement_plan = nullptr;
    oa_runtime_status replacement_status = OA_RUNTIME_EBUSY;
    for (unsigned attempt = 0U; attempt < 1000U && replacement_status != OA_RUNTIME_OK;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        init(snapshot);
        CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
        CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
              OA_RUNTIME_OK);
        move.required_feedback_seq = snapshot.arm[0].feedback_seq;
        move.target_model_rad = snapshot.arm[0].q_model_rad[0] + 0.001;
        move.expiry_runtime_monotonic_ns = now + 5000000000ULL;
        replacement_status = oa_runtime_plan_joint(runtime, &move, &replacement_plan);
        CHECK(replacement_status == OA_RUNTIME_OK || replacement_status == OA_RUNTIME_EBUSY ||
              replacement_status == OA_RUNTIME_ESTALE);
    }
    CHECK(replacement_status == OA_RUNTIME_OK);
    oa_runtime_execute_request expired_execute{};
    init(expired_execute);
    expired_execute.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    expired_execute.start_runtime_monotonic_ns = now + 1000000U;
    expired_execute.expiry_runtime_monotonic_ns = now + 1000000000U;
    expired_execute.producer_deadline_runtime_monotonic_ns = now + 1000000000U;
    expired_execute.stop_kind = OA_RUNTIME_STOP_DISABLE;
    std::uint64_t expired_command = 0U;
    CHECK(oa_runtime_execute(runtime, expired_plan, &expired_execute,
                             &expired_command) == OA_RUNTIME_ESTALE);
    oa_runtime_plan_destroy(expired_plan);
    second_plan = nullptr;
    CHECK(oa_runtime_plan_joint(runtime, &move, &second_plan) == OA_RUNTIME_EBUSY);
    oa_runtime_plan_destroy(replacement_plan);

    init(snapshot);
    CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
    oa_runtime_paired_tcp_move paired{};
    init(paired);
    paired.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    paired.units_id = OA_RUNTIME_UNITS_SI_V1;
    paired.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    paired.orientation_policy = OA_RUNTIME_ORIENTATION_FREE;
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
          OA_RUNTIME_OK);
    paired.expiry_runtime_monotonic_ns = now + 30000000000ULL;
    paired.required_feedback_seq[0] = snapshot.arm[0].feedback_seq;
    paired.required_feedback_seq[1] = snapshot.arm[1].feedback_seq;
    paired.left_tcp_m[0] = 0.20;
    paired.left_tcp_m[1] = 0.30;
    paired.left_tcp_m[2] = 0.85;
    paired.right_tcp_m[0] = 0.20;
    paired.right_tcp_m[1] = -0.30;
    paired.right_tcp_m[2] = 0.85;
    paired.velocity_scale = 1.0;
    paired.acceleration_scale = 1.0;
    paired.jerk_scale = 1.0;
    paired.tcp_tolerance_m = 0.001;
    paired.collision_scene_revision = 1U;
    paired.required_model_revision = capabilities.model_revision;
    paired.required_tcp_revision[0] = left_identity.tcp_revision;
    paired.required_tcp_revision[1] = right_identity.tcp_revision;
    paired.required_collision_policy = capabilities.collision_policy;
    std::snprintf(paired.required_coordinate_identity_sha256,
                  sizeof(paired.required_coordinate_identity_sha256), "%s",
                  capabilities.coordinate_identity_sha256);
    paired.maximum_branch_step_rad = 2.0;
    paired.minimum_singular_value = 0.0;
    paired.required_coordinate_identity_sha256[0] =
        paired.required_coordinate_identity_sha256[0] == '0' ? '1' : '0';
    CHECK(oa_runtime_plan_paired_tcp_body(runtime, &paired, &plan) ==
          OA_RUNTIME_EIDENTITY);
    std::snprintf(paired.required_coordinate_identity_sha256,
                  sizeof(paired.required_coordinate_identity_sha256), "%s",
                  capabilities.coordinate_identity_sha256);
    oa_runtime_status paired_plan_status = OA_RUNTIME_ESTALE;
    for (unsigned attempt = 0U;
         attempt < 100U && paired_plan_status == OA_RUNTIME_ESTALE; ++attempt) {
        init(snapshot);
        CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
        CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
              OA_RUNTIME_OK);
        paired.required_feedback_seq[0] = snapshot.arm[0].feedback_seq;
        paired.required_feedback_seq[1] = snapshot.arm[1].feedback_seq;
        paired.expiry_runtime_monotonic_ns = now + 30000000000ULL;
        paired_plan_status = oa_runtime_plan_paired_tcp_body(runtime, &paired, &plan);
    }
    CHECK(paired_plan_status == OA_RUNTIME_OK);
    execute_and_wait(runtime, plan, 30000000000ULL);
    oa_runtime_plan_destroy(plan);

    init(snapshot);
    CHECK(oa_runtime_snapshot_get(runtime, &snapshot) == OA_RUNTIME_OK);
    paired.required_feedback_seq[0] = snapshot.arm[0].feedback_seq;
    paired.required_feedback_seq[1] = snapshot.arm[1].feedback_seq;
    CHECK(oa_runtime_now_monotonic_ns(runtime, OA_RUNTIME_CLOCK_MONOTONIC, &now) ==
          OA_RUNTIME_OK);
    paired.expiry_runtime_monotonic_ns = now + 5000000000ULL;
    paired.left_tcp_m[0] = 100.0;
    paired.right_tcp_m[0] = 100.0;
    plan = nullptr;
    CHECK(oa_runtime_plan_paired_tcp_body(runtime, &paired, &plan) ==
          OA_RUNTIME_EUNREACHABLE);
    CHECK(plan == nullptr);
    oa_runtime_error_detail unreachable_detail{};
    init(unreachable_detail);
    CHECK(oa_runtime_get_last_error(runtime, &unreachable_detail) == OA_RUNTIME_OK);
    CHECK(unreachable_detail.status == OA_RUNTIME_EUNREACHABLE &&
          unreachable_detail.facility == OA_RUNTIME_FACILITY_CONTROL &&
          unreachable_detail.lower_code ==
              static_cast<std::uint32_t>(OA_CONTROL_EUNREACHABLE));

    CHECK(oa_runtime_set_interlock(runtime, 0U, 0U) == OA_RUNTIME_EFAULT);
    oa_runtime_error_detail interlock_detail{};
    init(interlock_detail);
    CHECK(oa_runtime_get_last_error(runtime, &interlock_detail) == OA_RUNTIME_OK);
    CHECK(interlock_detail.status == OA_RUNTIME_EFAULT &&
          interlock_detail.facility == OA_RUNTIME_FACILITY_CONTROL &&
          interlock_detail.lower_code != 0U);
    oa_runtime_inventory_destroy(inventory);
    oa_runtime_inventory_destroy(inventory);
    oa_runtime_destroy(runtime);
    oa_runtime_destroy(runtime);

    oa_runtime *lifetime_runtime = nullptr;
    CHECK(oa_runtime_create(&options, manifest, &lifetime_runtime) == OA_RUNTIME_OK);
    std::atomic<bool> lifetime_ok{true};
    std::thread caller([&] {
        for (unsigned attempt = 0U; attempt < 1000U; ++attempt) {
            oa_runtime_snapshot value{};
            init(value);
            const oa_runtime_status status =
                oa_runtime_snapshot_get(lifetime_runtime, &value);
            if (status != OA_RUNTIME_OK && status != OA_RUNTIME_EINVAL &&
                status != OA_RUNTIME_ESTATE) lifetime_ok.store(false);
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    oa_runtime_destroy(lifetime_runtime);
    caller.join();
    CHECK(lifetime_ok.load());
    oa_runtime_destroy(lifetime_runtime);

    auto query_options = virtual_options();
    query_options.backend = OA_RUNTIME_BACKEND_SOCKETCAN_QUERY;
    oa_runtime *query_runtime = nullptr;
    CHECK(oa_runtime_create(&query_options, manifest, &query_runtime) == OA_RUNTIME_OK);
    oa_runtime_capability_report query_capabilities{};
    init(query_capabilities);
    CHECK(oa_runtime_get_capabilities(query_runtime, &query_capabilities) == OA_RUNTIME_OK);
    CHECK((query_capabilities.capabilities & OA_RUNTIME_CAP_PHYSICAL_REGISTER_QUERY) != 0U);
    CHECK((query_capabilities.capabilities & OA_RUNTIME_CAP_PHYSICAL_CONFIGURATION) == 0U);
    CHECK((query_capabilities.capabilities & (OA_RUNTIME_CAP_MODEL_FK |
                                              OA_RUNTIME_CAP_SINGLE_XYZ_IK |
                                              OA_RUNTIME_CAP_PAIRED_XYZ_IK)) == 0U);
    std::uint64_t query_now_before = 0U;
    std::uint64_t query_now_after = 0U;
    CHECK(oa_runtime_now_monotonic_ns(query_runtime, OA_RUNTIME_CLOCK_MONOTONIC,
                                      &query_now_before) == OA_RUNTIME_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    CHECK(oa_runtime_now_monotonic_ns(query_runtime, OA_RUNTIME_CLOCK_MONOTONIC,
                                      &query_now_after) == OA_RUNTIME_OK);
    CHECK(query_now_before > 0U && query_now_after > query_now_before);
    init(kinematics);
    CHECK(oa_runtime_get_kinematics(query_runtime, 2U, 0U, &kinematics) ==
          OA_RUNTIME_EINVAL);
    CHECK(oa_runtime_get_kinematics(query_runtime, 0U, 0U, &kinematics) ==
          OA_RUNTIME_EUNSUPPORTED);
    oa_runtime_inventory_query_options query{};
    init(query);
    std::snprintf(query.interface_name, sizeof(query.interface_name), "oa_absent");
    query.per_query_timeout_ns = 1000000U;
    query.maximum_received_frames = 1U;
    oa_runtime_inventory_query_options semantic_bad = query;
    semantic_bad.per_query_timeout_ns = 0U;
    CHECK(oa_runtime_inventory_query(query_runtime, &semantic_bad, &inventory) ==
          OA_RUNTIME_EINVAL);
    oa_runtime_inventory_query_options abi_bad = query;
    abi_bad.struct_size = 0U;
    CHECK(oa_runtime_inventory_query(query_runtime, &abi_bad, &inventory) ==
          OA_RUNTIME_EABI);
    CHECK(oa_runtime_inventory_query(query_runtime, &query, &inventory) == OA_RUNTIME_OK);
    init(inventory_summary);
    CHECK(oa_runtime_inventory_get_summary(inventory, &inventory_summary) == OA_RUNTIME_OK);
    CHECK(inventory_summary.interface_count == 0U && inventory_summary.motor_count == 0U);
    CHECK(oa_runtime_arm_virtual(query_runtime) == OA_RUNTIME_EUNSUPPORTED);
    CHECK(oa_runtime_configuration_apply_physical(query_runtime, manifest) ==
          OA_RUNTIME_EUNSUPPORTED);
    oa_commission_manual_options physical_manual{};
    physical_manual.struct_size = sizeof(physical_manual);
    physical_manual.abi_version = OA_COMMISSION_ABI_V1;
    std::snprintf(physical_manual.motor_serial, sizeof(physical_manual.motor_serial), "x");
    oa_runtime_calibration *physical_calibration = nullptr;
    CHECK(oa_runtime_calibration_manual_begin(query_runtime, &physical_manual,
                                              &physical_calibration) ==
          OA_RUNTIME_EUNSUPPORTED);
    oa_commission_recipe physical_recipe{};
    physical_recipe.struct_size = sizeof(physical_recipe);
    physical_recipe.abi_version = OA_COMMISSION_ABI_V1;
    CHECK(oa_runtime_calibration_recipe_begin(query_runtime, &physical_recipe,
                                              &physical_calibration) ==
          OA_RUNTIME_EUNSUPPORTED);
    physical_recipe.side = 2U;
    CHECK(oa_runtime_calibration_recipe_begin(query_runtime, &physical_recipe,
                                              &physical_calibration) ==
          OA_RUNTIME_EINVAL);
    oa_runtime_inventory_destroy(inventory);
    oa_runtime_destroy(query_runtime);

    auto offline_options = virtual_options();
    offline_options.backend = OA_RUNTIME_BACKEND_OFFLINE;
    oa_runtime *offline_runtime = nullptr;
    CHECK(oa_runtime_create(&offline_options, manifest, &offline_runtime) == OA_RUNTIME_OK);
    std::uint64_t offline_before = 0U;
    std::uint64_t offline_after = 0U;
    CHECK(oa_runtime_now_monotonic_ns(offline_runtime, OA_RUNTIME_CLOCK_MONOTONIC,
                                      &offline_before) == OA_RUNTIME_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    CHECK(oa_runtime_now_monotonic_ns(offline_runtime, OA_RUNTIME_CLOCK_MONOTONIC,
                                      &offline_after) == OA_RUNTIME_OK);
    CHECK(offline_before > 0U && offline_after > offline_before);
    oa_runtime_destroy(offline_runtime);

    oa_runtime_options invalid_options = options;
    invalid_options.backend = 99U;
    CHECK(oa_runtime_create(&invalid_options, manifest, &offline_runtime) ==
          OA_RUNTIME_EINVAL);
    invalid_options = options;
    invalid_options.struct_size = 0U;
    CHECK(oa_runtime_create(&invalid_options, manifest, &offline_runtime) ==
          OA_RUNTIME_EABI);
    oa_runtime_manifest_destroy(manifest);
    return 0;
}
