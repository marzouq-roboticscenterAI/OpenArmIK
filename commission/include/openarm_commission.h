#ifndef OPENARM_COMMISSION_H
#define OPENARM_COMMISSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OA_COMMISSION_ABI_V1 UINT32_C(1)
#define OA_COMMISSION_TEXT_CAPACITY 64U

typedef uint32_t oa_commission_status;

enum {
    OA_COMMISSION_OK = 0,
    OA_COMMISSION_EINVAL = 1,
    OA_COMMISSION_EABI = 2,
    OA_COMMISSION_ESTATE = 3,
    OA_COMMISSION_ESTALE = 4,
    OA_COMMISSION_EINTERLOCK = 5,
    OA_COMMISSION_ELIMIT = 6,
    OA_COMMISSION_EUNSTABLE = 7,
    OA_COMMISSION_EREPEATABILITY = 8,
    OA_COMMISSION_EUNSUPPORTED = 9,
    OA_COMMISSION_ENOMEM = 10,
    OA_COMMISSION_EFAULT = 11
};

enum {
    OA_COMMISSION_LEFT = 0,
    OA_COMMISSION_RIGHT = 1
};

enum {
    OA_MANUAL_COLLECT_REFERENCE_1 = 1,
    OA_MANUAL_COLLECT_REFERENCE_2 = 2,
    OA_MANUAL_CANDIDATE = 3,
    OA_MANUAL_REVIEW = 4,
    OA_MANUAL_COMMITTED = 5,
    OA_MANUAL_ABORTED = 6
};

enum {
    OA_RECIPE_PRECHECK = 1,
    OA_RECIPE_WAIT = 2,
    OA_RECIPE_APPROACH = 3,
    OA_RECIPE_CONTACT_DWELL = 4,
    OA_RECIPE_RETREAT = 5,
    OA_RECIPE_REAPPROACH = 6,
    OA_RECIPE_REPEATABILITY = 7,
    OA_RECIPE_CANDIDATE = 8,
    OA_RECIPE_REVIEW = 9,
    OA_RECIPE_COMMIT = 10,
    OA_RECIPE_ABORT = 11
};

enum {
    OA_RECIPE_ARM_JOINT = 1,
    OA_RECIPE_GRIPPER = 2
};

enum {
    OA_RECIPE_ACTION_NONE = 0,
    OA_RECIPE_ACTION_HOLD_DISABLED = 1,
    OA_RECIPE_ACTION_APPROACH = 2,
    OA_RECIPE_ACTION_CONTACT_DWELL = 3,
    OA_RECIPE_ACTION_RETREAT = 4,
    OA_RECIPE_ACTION_REAPPROACH = 5,
    OA_RECIPE_ACTION_REVIEW = 6,
    OA_RECIPE_ACTION_COMMIT_READY = 7,
    OA_RECIPE_ACTION_ABORT_DISABLE = 8
};

enum {
    OA_REVIEW_NONE = 0,
    OA_REVIEW_ACCEPT = 1,
    OA_REVIEW_REJECT = 2
};

enum {
    OA_ABORT_NONE = 0,
    OA_ABORT_CALLER = 1,
    OA_ABORT_BAD_SAMPLE = 2,
    OA_ABORT_STALE = 3,
    OA_ABORT_INTERLOCK = 4,
    OA_ABORT_LIMIT = 5,
    OA_ABORT_TIMEOUT = 6,
    OA_ABORT_CONTACT = 7,
    OA_ABORT_REPEATABILITY = 8,
    OA_ABORT_REVIEW_REJECTED = 9,
    OA_ABORT_UNQUALIFIED = 10
};

typedef struct oa_commission_manual_session oa_commission_manual_session;
typedef struct oa_commission_recipe_session oa_commission_recipe_session;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t feedback_seq;
    uint64_t sample_time_ns;
    double q_output_rad;
    double dq_output_rad_s;
    double torque_output_nm;
    double mos_temperature_c;
    double coil_temperature_c;
    uint32_t drive_enabled;
    uint32_t drive_fault;
} oa_commission_encoder_sample;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t side;
    uint32_t joint;
    uint64_t expected_revision;
    uint32_t reference_count;
    int32_t known_sign;
    uint32_t minimum_samples;
    uint32_t reserved0;
    uint64_t maximum_sample_age_ns;
    uint64_t stability_dwell_ns;
    double reference_model_rad[2];
    double maximum_position_spread_rad;
    double maximum_abs_velocity_rad_s;
    double minimum_reference_separation_rad;
    double maximum_scale_error;
    char motor_serial[OA_COMMISSION_TEXT_CAPACITY];
} oa_commission_manual_options;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t state;
    uint32_t active_reference;
    uint32_t accepted_samples[2];
    uint32_t reserved0;
    double mean_output_rad[2];
    double spread_output_rad[2];
    double candidate_a;
    double candidate_b_rad;
} oa_commission_manual_report;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expected_revision;
    uint64_t replacement_revision;
    uint32_t side;
    uint32_t joint;
    double a;
    double b_rad;
    char motor_serial[OA_COMMISSION_TEXT_CAPACITY];
    char evidence_record[OA_COMMISSION_TEXT_CAPACITY];
} oa_commission_mapping_patch;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t recipe_kind;
    uint32_t side;
    uint32_t joint;
    int32_t known_sign;
    uint32_t hardware_qualified;
    uint32_t simulation_only;
    uint32_t minimum_contact_samples;
    uint64_t expected_revision;
    uint64_t qualification_revision;
    uint64_t maximum_sample_age_ns;
    uint64_t maximum_approach_time_ns;
    uint64_t contact_dwell_ns;
    uint64_t maximum_retreat_time_ns;
    double stop_model_rad;
    double approach_direction;
    double start_min_output_rad;
    double start_max_output_rad;
    double maximum_speed_rad_s;
    double minimum_contact_travel_rad;
    double maximum_approach_travel_rad;
    double contact_velocity_rad_s;
    double minimum_contact_torque_nm;
    double maximum_torque_nm;
    double maximum_contact_energy_j;
    double maximum_temperature_c;
    double retreat_distance_rad;
    double repeatability_tolerance_rad;
    char motor_serial[OA_COMMISSION_TEXT_CAPACITY];
    char qualification_record[OA_COMMISSION_TEXT_CAPACITY];
    char fixture_record[OA_COMMISSION_TEXT_CAPACITY];
} oa_commission_recipe;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t now_ns;
    oa_commission_encoder_sample encoder;
    uint32_t estop_clear;
    uint32_t deadman_held;
    uint32_t operator_ready;
    uint32_t action_complete;
    uint32_t review_decision;
    uint32_t reserved0;
} oa_commission_recipe_input;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t kind;
    uint32_t state;
    uint64_t valid_until_ns;
    double direction;
    double target_output_rad;
    double maximum_speed_rad_s;
    double maximum_travel_rad;
    double maximum_torque_nm;
    double maximum_temperature_c;
} oa_commission_next_action;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t state;
    uint32_t abort_reason;
    uint64_t last_feedback_seq;
    double first_stop_output_rad;
    double second_stop_output_rad;
    double candidate_a;
    double candidate_b_rad;
    double accumulated_contact_energy_j;
} oa_commission_recipe_report;

oa_commission_status oa_commission_manual_create(
    const oa_commission_manual_options *options,
    oa_commission_manual_session **out_session);
void oa_commission_manual_destroy(oa_commission_manual_session *session);
oa_commission_status oa_commission_manual_sample(
    oa_commission_manual_session *session,
    uint32_t reference_index,
    uint64_t now_ns,
    const oa_commission_encoder_sample *sample,
    oa_commission_manual_report *out_report);
oa_commission_status oa_commission_manual_begin_review(
    oa_commission_manual_session *session,
    oa_commission_manual_report *out_report);
oa_commission_status oa_commission_manual_commit(
    oa_commission_manual_session *session,
    uint64_t replacement_revision,
    const char *evidence_record,
    oa_commission_mapping_patch *out_patch);
oa_commission_status oa_commission_manual_abort(
    oa_commission_manual_session *session);
oa_commission_status oa_commission_manual_get_report(
    const oa_commission_manual_session *session,
    oa_commission_manual_report *out_report);

oa_commission_status oa_commission_recipe_create(
    const oa_commission_recipe *recipe,
    oa_commission_recipe_session **out_session);
void oa_commission_recipe_destroy(oa_commission_recipe_session *session);
oa_commission_status oa_commission_recipe_step(
    oa_commission_recipe_session *session,
    const oa_commission_recipe_input *input,
    oa_commission_next_action *out_action,
    oa_commission_recipe_report *out_report);
oa_commission_status oa_commission_recipe_commit(
    oa_commission_recipe_session *session,
    uint64_t replacement_revision,
    oa_commission_mapping_patch *out_patch);
oa_commission_status oa_commission_recipe_abort(
    oa_commission_recipe_session *session);
oa_commission_status oa_commission_recipe_get_report(
    const oa_commission_recipe_session *session,
    oa_commission_recipe_report *out_report);

#ifdef __cplusplus
}
#endif

#endif
