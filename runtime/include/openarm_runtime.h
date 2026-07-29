/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_RUNTIME_H
#define OPENARM_RUNTIME_H

#include <stddef.h>
#include <stdint.h>

#include <openarm_commission.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OA_RUNTIME_ABI_VERSION UINT32_C(1)
#define OA_RUNTIME_ARMS UINT32_C(2)
#define OA_RUNTIME_DOF UINT32_C(7)
#define OA_RUNTIME_MOTORS UINT32_C(14)
#define OA_RUNTIME_MAX_INTERFACES UINT32_C(32)
#define OA_RUNTIME_MAX_QUERY_MOTORS UINT32_C(16)
#define OA_RUNTIME_TEXT_CAPACITY UINT32_C(64)
#define OA_RUNTIME_DIGEST_CAPACITY UINT32_C(65)
#define OA_RUNTIME_PROVENANCE_CAPACITY UINT32_C(192)
#define OA_RUNTIME_PERSISTENCE_KEY_BYTES UINT32_C(32)

typedef uint32_t oa_runtime_status;
#define OA_RUNTIME_OK UINT32_C(0x52000000)
#define OA_RUNTIME_EINVAL UINT32_C(0x52000001)
#define OA_RUNTIME_EABI UINT32_C(0x52000002)
#define OA_RUNTIME_ESTATE UINT32_C(0x52000003)
#define OA_RUNTIME_ESTALE UINT32_C(0x52000004)
#define OA_RUNTIME_ETIMEOUT UINT32_C(0x52000005)
#define OA_RUNTIME_EBUSY UINT32_C(0x52000006)
#define OA_RUNTIME_EIO UINT32_C(0x52000007)
#define OA_RUNTIME_ECORRUPT UINT32_C(0x52000008)
#define OA_RUNTIME_EIDENTITY UINT32_C(0x52000009)
#define OA_RUNTIME_EPERMISSION UINT32_C(0x5200000a)
#define OA_RUNTIME_EUNSUPPORTED UINT32_C(0x5200000b)
#define OA_RUNTIME_EFAULT UINT32_C(0x5200000c)
#define OA_RUNTIME_ENOMEM UINT32_C(0x5200000d)
#define OA_RUNTIME_ECOLLISION UINT32_C(0x5200000e)
#define OA_RUNTIME_EUNREACHABLE UINT32_C(0x5200000f)
/* Post-commit rollback could not be confirmed; target state/durability is unknown. */
#define OA_RUNTIME_EDURABILITY UINT32_C(0x52000010)

typedef uint32_t oa_runtime_facility;
#define OA_RUNTIME_FACILITY_RUNTIME UINT32_C(1)
#define OA_RUNTIME_FACILITY_CAN UINT32_C(2)
#define OA_RUNTIME_FACILITY_TRANSPORT UINT32_C(3)
#define OA_RUNTIME_FACILITY_MODEL UINT32_C(4)
#define OA_RUNTIME_FACILITY_COMMISSION UINT32_C(5)
#define OA_RUNTIME_FACILITY_CONTROL UINT32_C(6)
#define OA_RUNTIME_FACILITY_SYSTEM UINT32_C(7)

typedef uint32_t oa_runtime_backend;
#define OA_RUNTIME_BACKEND_VIRTUAL UINT32_C(1)
#define OA_RUNTIME_BACKEND_SOCKETCAN_QUERY UINT32_C(2)
#define OA_RUNTIME_BACKEND_OFFLINE UINT32_C(3)

typedef uint32_t oa_runtime_clock_id;
#define OA_RUNTIME_CLOCK_MONOTONIC UINT32_C(0x4f41524d)

typedef uint32_t oa_runtime_frame_id;
#define OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 UINT32_C(1)

typedef uint32_t oa_runtime_units_id;
#define OA_RUNTIME_UNITS_SI_V1 UINT32_C(1)

typedef uint32_t oa_runtime_orientation_policy;
#define OA_RUNTIME_ORIENTATION_FREE UINT32_C(1)

typedef uint32_t oa_runtime_collision_policy;
#define OA_RUNTIME_COLLISION_REJECT_ALL UINT32_C(1)
#define OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED UINT32_C(2)

typedef uint32_t oa_runtime_integrity_kind;
#define OA_RUNTIME_INTEGRITY_UNKEYED_SHA256 UINT32_C(1)
#define OA_RUNTIME_INTEGRITY_HMAC_SHA256 UINT32_C(2)

#define OA_RUNTIME_EVENT_VERIFIED UINT32_C(1)
#define OA_RUNTIME_EVENT_ARMED UINT32_C(2)
#define OA_RUNTIME_EVENT_STARTED UINT32_C(3)
#define OA_RUNTIME_EVENT_COMPLETED UINT32_C(4)
#define OA_RUNTIME_EVENT_STOPPED UINT32_C(5)
#define OA_RUNTIME_EVENT_FAULTED UINT32_C(6)
#define OA_RUNTIME_EVENT_DISARMED UINT32_C(7)
#define OA_RUNTIME_EVENT_QUEUED UINT32_C(8)
#define OA_RUNTIME_EVENT_SETTLING UINT32_C(9)
#define OA_RUNTIME_EVENT_ABORTED UINT32_C(10)
#define OA_RUNTIME_EVENT_ESTOP UINT32_C(11)

#define OA_RUNTIME_STOP_DISABLE UINT32_C(1)
#define OA_RUNTIME_STOP_CONTROLLED UINT32_C(2)

typedef uint64_t oa_runtime_capability;
/* Reserved operation bits. Runtime V1 has no standalone FK/IK entry points and
 * therefore never advertises these bits. */
#define OA_RUNTIME_CAP_MODEL_FK (UINT64_C(1) << 0)
#define OA_RUNTIME_CAP_SINGLE_XYZ_IK (UINT64_C(1) << 1)
#define OA_RUNTIME_CAP_PAIRED_XYZ_IK (UINT64_C(1) << 2)
#define OA_RUNTIME_CAP_VIRTUAL_COORDINATES (UINT64_C(1) << 3)
#define OA_RUNTIME_CAP_VIRTUAL_JOINT_MOTION (UINT64_C(1) << 4)
#define OA_RUNTIME_CAP_VIRTUAL_PAIRED_XYZ_MOTION (UINT64_C(1) << 5)
#define OA_RUNTIME_CAP_VIRTUAL_MANUAL_CALIBRATION (UINT64_C(1) << 6)
#define OA_RUNTIME_CAP_VIRTUAL_SUPERVISED_CALIBRATION (UINT64_C(1) << 7)
#define OA_RUNTIME_CAP_INTERFACE_ENUMERATION (UINT64_C(1) << 8)
#define OA_RUNTIME_CAP_PHYSICAL_REGISTER_QUERY (UINT64_C(1) << 9)
#define OA_RUNTIME_CAP_MANIFEST_PREVIEW (UINT64_C(1) << 10)
#define OA_RUNTIME_CAP_MANIFEST_PERSISTENCE (UINT64_C(1) << 11)
#define OA_RUNTIME_CAP_PHYSICAL_CONFIGURATION (UINT64_C(1) << 20)
#define OA_RUNTIME_CAP_PHYSICAL_CALIBRATION_MOTION (UINT64_C(1) << 21)
#define OA_RUNTIME_CAP_PHYSICAL_MOTION (UINT64_C(1) << 22)
#define OA_RUNTIME_CAP_COLLISION_VALIDATED_MOTION (UINT64_C(1) << 23)

typedef uint32_t oa_runtime_manifest_state;
#define OA_RUNTIME_MANIFEST_DRAFT UINT32_C(1)
#define OA_RUNTIME_MANIFEST_LOCALLY_VALID UINT32_C(2)
#define OA_RUNTIME_MANIFEST_INVENTORY_MATCHED UINT32_C(3)
#define OA_RUNTIME_MANIFEST_ARMABLE UINT32_C(4)

typedef uint32_t oa_runtime_evidence_confidence;
#define OA_RUNTIME_EVIDENCE_NONE UINT32_C(0)
#define OA_RUNTIME_EVIDENCE_VIRTUAL_EXACT UINT32_C(1)
#define OA_RUNTIME_EVIDENCE_QUERY_CORRELATED UINT32_C(2)
#define OA_RUNTIME_EVIDENCE_AMBIGUOUS UINT32_C(3)

#define OA_RUNTIME_INTERFACE_KIND_UNKNOWN UINT32_C(0)
#define OA_RUNTIME_INTERFACE_KIND_PHYSICAL UINT32_C(1)
#define OA_RUNTIME_INTERFACE_KIND_VIRTUAL UINT32_C(2)

typedef struct oa_runtime oa_runtime;
typedef struct oa_runtime_manifest oa_runtime_manifest;
typedef struct oa_runtime_inventory oa_runtime_inventory;
typedef struct oa_runtime_calibration oa_runtime_calibration;
typedef struct oa_runtime_plan oa_runtime_plan;
typedef struct oa_runtime_persistence_authority oa_runtime_persistence_authority;

typedef struct oa_runtime_error_detail {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_status status;
    oa_runtime_facility facility;
    uint32_t lower_code;
    uint32_t system_error;
} oa_runtime_error_detail;

typedef struct oa_runtime_options {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_backend backend;
    uint32_t allow_unchecked_virtual_motion;
    uint64_t cycle_ns;
    uint64_t feedback_timeout_ns;
    uint64_t maximum_cross_bus_skew_ns;
    uint64_t collision_scene_revision;
} oa_runtime_options;

typedef struct oa_runtime_capability_report {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_backend backend;
    oa_runtime_clock_id clock_id;
    oa_runtime_units_id units_id;
    oa_runtime_frame_id xyz_frame_id;
    oa_runtime_orientation_policy orientation_policy;
    oa_runtime_collision_policy collision_policy;
    uint32_t collision_checked;
    uint64_t model_revision;
    oa_runtime_capability capabilities;
    char coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
} oa_runtime_capability_report;

typedef struct oa_runtime_model_identity {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t side;
    oa_runtime_frame_id xyz_frame_id;
    oa_runtime_units_id units_id;
    oa_runtime_orientation_policy orientation_policy;
    oa_runtime_collision_policy collision_policy;
    uint64_t model_revision;
    uint64_t tcp_revision;
    uint64_t collision_scene_revision;
    char model_id[OA_RUNTIME_TEXT_CAPACITY];
    char provenance[OA_RUNTIME_PROVENANCE_CAPACITY];
    char model_data_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    char flattened_urdf_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    char source_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    char tcp_frame[OA_RUNTIME_TEXT_CAPACITY];
    char coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
} oa_runtime_model_identity;

typedef struct oa_runtime_interface {
    uint32_t struct_size;
    uint32_t abi_version;
    char name[16];
    uint32_t ifindex;
    uint32_t flags;
    uint32_t mtu;
    uint32_t bitrate;
    uint32_t data_bitrate;
    uint32_t link_up;
    uint32_t fd_enabled;
    uint32_t interface_kind;
} oa_runtime_interface;

typedef struct oa_runtime_motor_evidence {
    uint32_t struct_size;
    uint32_t abi_version;
    char interface_name[16];
    uint16_t requested_send_id;
    uint16_t expected_receive_id;
    uint16_t observed_receive_id;
    uint16_t reserved0;
    uint64_t query_sent_runtime_monotonic_ns;
    uint64_t response_runtime_monotonic_ns;
    uint64_t register_presence_mask;
    uint64_t register_mismatch_mask;
    uint32_t duplicate_count;
    uint32_t fault_observed;
    uint32_t enabled_observed;
    uint32_t stale_observed;
    oa_runtime_evidence_confidence confidence;
    uint32_t unresolved_assignment;
    uint32_t side;
    uint32_t joint;
    uint32_t serial_number;
    uint32_t hardware_version;
    uint32_t software_version;
    uint32_t firmware_subversion;
    uint32_t configured_send_id;
    uint32_t configured_receive_id;
    uint32_t control_mode;
    uint32_t bitrate;
    uint32_t timeout_ticks;
    int32_t direction;
    float pmax_rad;
    float vmax_rad_s;
    float tmax_nm;
    float gear_ratio;
    char immutable_identity[OA_RUNTIME_TEXT_CAPACITY];
} oa_runtime_motor_evidence;

typedef struct oa_runtime_inventory_summary {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t interface_count;
    uint32_t motor_count;
    uint32_t unknown_mask;
    uint32_t ambiguous_mask;
    uint32_t conflict_mask;
    uint32_t unresolved_assignment;
    uint64_t inventory_revision;
    oa_runtime_clock_id clock_id;
    char fingerprint_sha256[OA_RUNTIME_DIGEST_CAPACITY];
} oa_runtime_inventory_summary;

typedef struct oa_runtime_query_candidate {
    uint32_t struct_size;
    uint32_t abi_version;
    uint16_t send_id;
    uint16_t receive_id;
} oa_runtime_query_candidate;

typedef struct oa_runtime_inventory_query_options {
    uint32_t struct_size;
    uint32_t abi_version;
    char interface_name[16];
    uint32_t candidate_count;
    oa_runtime_query_candidate candidate[OA_RUNTIME_MAX_QUERY_MOTORS];
    uint64_t per_query_timeout_ns;
    uint32_t maximum_received_frames;
} oa_runtime_inventory_query_options;

typedef struct oa_runtime_motor_manifest {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t side;
    uint32_t joint;
    uint32_t motor_type;
    uint32_t send_id;
    uint32_t receive_id;
    uint32_t embedded_motor_id;
    uint32_t control_mode;
    uint32_t bitrate;
    uint32_t timeout_ticks;
    uint32_t hardware_version;
    uint32_t software_version;
    uint32_t firmware_subversion;
    double q_scale;
    double q_offset_rad;
    double lower_rad;
    double upper_rad;
    double maximum_velocity_rad_s;
    double maximum_acceleration_rad_s2;
    double maximum_jerk_rad_s3;
    double protocol_pmax_rad;
    double protocol_vmax_rad_s;
    double protocol_tmax_nm;
    double gear_ratio_metadata;
    int32_t direction;
    uint32_t evidence_kind;
    uint64_t evidence_revision;
    char serial[32];
    char joint_name[48];
    char evidence_record[OA_RUNTIME_TEXT_CAPACITY];
} oa_runtime_motor_manifest;

typedef struct oa_runtime_manifest_summary {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_manifest_state state;
    oa_runtime_backend intended_backend;
    uint64_t manifest_revision;
    uint64_t model_revision;
    uint64_t inventory_revision;
    oa_runtime_integrity_kind integrity_kind;
    uint32_t authenticated;
    char inventory_fingerprint_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    char content_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    char authentication_key_id[OA_RUNTIME_TEXT_CAPACITY];
} oa_runtime_manifest_summary;

typedef struct oa_runtime_manifest_preview {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t valid;
    uint32_t changed_motor_mask;
    uint32_t identity_change_mask;
    uint32_t mapping_change_mask;
    uint32_t limit_change_mask;
    uint32_t would_be_armable;
    uint64_t base_revision;
    uint64_t result_revision;
    oa_runtime_status validation_status;
} oa_runtime_manifest_preview;

typedef struct oa_runtime_arm_snapshot {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t feedback_seq;
    uint64_t measurement_runtime_monotonic_ns;
    uint32_t expected_mask;
    uint32_t fresh_mask;
    uint32_t fault_mask;
    double q_model_rad[OA_RUNTIME_DOF];
    double dq_model_rad_s[OA_RUNTIME_DOF];
    double tau_model_nm[OA_RUNTIME_DOF];
    double q_output_rad[OA_RUNTIME_DOF];
    double dq_output_rad_s[OA_RUNTIME_DOF];
    double tau_output_nm[OA_RUNTIME_DOF];
    uint8_t status[OA_RUNTIME_DOF];
    uint8_t mos_temperature_c[OA_RUNTIME_DOF];
    uint8_t coil_temperature_c[OA_RUNTIME_DOF];
} oa_runtime_arm_snapshot;

typedef struct oa_runtime_snapshot {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_clock_id clock_id;
    oa_runtime_units_id units_id;
    oa_runtime_frame_id frame_id;
    uint32_t lifecycle;
    uint64_t manifest_revision;
    uint64_t model_revision;
    uint64_t maximum_cross_bus_skew_ns;
    char coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    oa_runtime_arm_snapshot arm[OA_RUNTIME_ARMS];
} oa_runtime_snapshot;

typedef struct oa_runtime_kinematics {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_frame_id frame_id;
    oa_runtime_units_id units_id;
    oa_runtime_orientation_policy orientation_policy;
    uint32_t side;
    uint64_t feedback_seq;
    uint64_t model_revision;
    uint64_t tcp_revision;
    oa_runtime_collision_policy collision_policy;
    char model_id[OA_RUNTIME_TEXT_CAPACITY];
    char tcp_frame[OA_RUNTIME_TEXT_CAPACITY];
    char model_data_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    char flattened_urdf_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    char coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    double q_model_rad[OA_RUNTIME_DOF];
    double joint_xyz_m[OA_RUNTIME_DOF][3];
    double joint_axis_body[OA_RUNTIME_DOF][3];
    double tcp_transform_row_major[16];
    double tcp_xyz_m[3];
} oa_runtime_kinematics;

typedef struct oa_runtime_joint_move {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_clock_id clock_id;
    oa_runtime_units_id units_id;
    uint64_t expiry_runtime_monotonic_ns;
    uint64_t required_feedback_seq;
    uint32_t side;
    uint32_t joint;
    double target_model_rad;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double position_tolerance_rad;
    double velocity_tolerance_rad_s;
    uint64_t required_model_revision;
    uint64_t required_tcp_revision;
    uint64_t collision_scene_revision;
    oa_runtime_collision_policy required_collision_policy;
    char required_coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
} oa_runtime_joint_move;

typedef struct oa_runtime_paired_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_clock_id clock_id;
    oa_runtime_units_id units_id;
    oa_runtime_frame_id frame_id;
    oa_runtime_orientation_policy orientation_policy;
    uint64_t expiry_runtime_monotonic_ns;
    uint64_t required_feedback_seq[OA_RUNTIME_ARMS];
    double left_tcp_m[3];
    double right_tcp_m[3];
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tolerance_m;
    uint64_t collision_scene_revision;
    uint64_t required_model_revision;
    uint64_t required_tcp_revision[OA_RUNTIME_ARMS];
    oa_runtime_collision_policy required_collision_policy;
    char required_coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    double maximum_branch_step_rad;
    double minimum_singular_value;
} oa_runtime_paired_tcp_move;

typedef struct oa_runtime_plan_report {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t kind;
    uint32_t collision_checked;
    uint32_t motion_authorized;
    oa_runtime_frame_id frame_id;
    oa_runtime_units_id units_id;
    uint64_t seed_feedback_seq[OA_RUNTIME_ARMS];
    uint64_t duration_ns;
    uint64_t manifest_revision;
    uint64_t model_revision;
    uint64_t tcp_revision[OA_RUNTIME_ARMS];
    uint64_t collision_scene_revision;
    oa_runtime_collision_policy collision_policy;
    char coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    double target_q_model_rad[OA_RUNTIME_ARMS][OA_RUNTIME_DOF];
    double achieved_tcp_m[OA_RUNTIME_ARMS][3];
    double tcp_residual_m[OA_RUNTIME_ARMS];
} oa_runtime_plan_report;

typedef struct oa_runtime_execute_request {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_clock_id clock_id;
    uint64_t start_runtime_monotonic_ns;
    uint64_t expiry_runtime_monotonic_ns;
    uint64_t producer_deadline_runtime_monotonic_ns;
    uint32_t stop_kind;
} oa_runtime_execute_request;

typedef struct oa_runtime_event {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t kind;
    oa_runtime_facility source_facility;
    oa_runtime_status status;
    uint32_t source_status;
    oa_runtime_clock_id clock_id;
    uint64_t event_runtime_monotonic_ns;
    uint64_t measurement_runtime_monotonic_ns;
    uint64_t manifest_revision;
    uint64_t inventory_revision;
    uint64_t calibration_revision;
    uint64_t model_revision;
    uint64_t scene_revision;
    uint64_t feedback_seq[OA_RUNTIME_ARMS];
    uint64_t source_feedback_seq;
    uint32_t feedback_seq_valid_mask;
    uint32_t measurement_timestamp_valid;
    uint64_t command_id;
    uint32_t lifecycle;
    uint32_t collision_checked;
    uint32_t motion_authorized;
    oa_runtime_capability capabilities;
} oa_runtime_event;

oa_runtime_status oa_runtime_manifest_create_virtual(oa_runtime_manifest **out_manifest);
oa_runtime_status oa_runtime_manifest_get_summary(const oa_runtime_manifest *manifest,
                                                  oa_runtime_manifest_summary *out_summary);
oa_runtime_status oa_runtime_manifest_get_motor(const oa_runtime_manifest *manifest,
                                                uint32_t side, uint32_t joint,
                                                oa_runtime_motor_manifest *out_motor);
oa_runtime_status oa_runtime_manifest_preview_file(const char *absolute_path,
                                                   oa_runtime_manifest_preview *out_preview);
oa_runtime_status oa_runtime_manifest_load(const char *absolute_path,
                                           oa_runtime_manifest **out_manifest);
oa_runtime_status oa_runtime_persistence_authority_create(
    const char *absolute_directory,
    const uint8_t authentication_key[OA_RUNTIME_PERSISTENCE_KEY_BYTES],
    const char *authentication_key_id,
    oa_runtime_persistence_authority **out_authority);
void oa_runtime_persistence_authority_destroy(oa_runtime_persistence_authority *authority);
oa_runtime_status oa_runtime_manifest_load_authenticated(
    const oa_runtime_persistence_authority *authority, const char *file_name,
    oa_runtime_manifest **out_manifest);
oa_runtime_status oa_runtime_manifest_save(const oa_runtime_manifest *manifest,
                                           const oa_runtime_persistence_authority *authority,
                                           const char *file_name);
void oa_runtime_manifest_destroy(oa_runtime_manifest *manifest);

oa_runtime_status oa_runtime_create(const oa_runtime_options *options,
                                    const oa_runtime_manifest *manifest,
                                    oa_runtime **out_runtime);
void oa_runtime_destroy(oa_runtime *runtime);
oa_runtime_status oa_runtime_get_capabilities(const oa_runtime *runtime,
                                              oa_runtime_capability_report *out_report);
oa_runtime_status oa_runtime_get_model_identity(const oa_runtime *runtime, uint32_t side,
                                                oa_runtime_model_identity *out_identity);
oa_runtime_status oa_runtime_now_monotonic_ns(const oa_runtime *runtime,
                                              oa_runtime_clock_id clock_id,
                                              uint64_t *out_now_ns);
oa_runtime_status oa_runtime_get_last_error(const oa_runtime *runtime,
                                            oa_runtime_error_detail *out_detail);

oa_runtime_status oa_runtime_list_interfaces(const oa_runtime *runtime,
                                             oa_runtime_interface *interfaces,
                                             size_t capacity, size_t *out_count);
oa_runtime_status oa_runtime_inventory_query(oa_runtime *runtime,
                                             const oa_runtime_inventory_query_options *options,
                                             oa_runtime_inventory **out_inventory);
oa_runtime_status oa_runtime_inventory_get_summary(const oa_runtime_inventory *inventory,
                                                   oa_runtime_inventory_summary *out_summary);
oa_runtime_status oa_runtime_inventory_get_interface(const oa_runtime_inventory *inventory,
                                                     size_t index,
                                                     oa_runtime_interface *out_interface);
oa_runtime_status oa_runtime_inventory_get_motor(const oa_runtime_inventory *inventory,
                                                 size_t index,
                                                 oa_runtime_motor_evidence *out_motor);
void oa_runtime_inventory_destroy(oa_runtime_inventory *inventory);

oa_runtime_status oa_runtime_snapshot_get(oa_runtime *runtime,
                                          oa_runtime_snapshot *out_snapshot);
oa_runtime_status oa_runtime_get_kinematics(oa_runtime *runtime, uint32_t side,
                                            uint64_t required_feedback_seq,
                                            oa_runtime_kinematics *out_kinematics);
oa_runtime_status oa_runtime_arm_virtual(oa_runtime *runtime);
oa_runtime_status oa_runtime_plan_joint(oa_runtime *runtime,
                                        const oa_runtime_joint_move *request,
                                        oa_runtime_plan **out_plan);
oa_runtime_status oa_runtime_plan_paired_tcp_body(oa_runtime *runtime,
                                                  const oa_runtime_paired_tcp_move *request,
                                                  oa_runtime_plan **out_plan);
oa_runtime_status oa_runtime_plan_get_report(const oa_runtime_plan *plan,
                                             oa_runtime_plan_report *out_report);
oa_runtime_status oa_runtime_execute(oa_runtime *runtime, const oa_runtime_plan *plan,
                                     const oa_runtime_execute_request *request,
                                     uint64_t *out_command_id);
oa_runtime_status oa_runtime_heartbeat(oa_runtime *runtime, uint64_t command_id,
                                      oa_runtime_clock_id clock_id,
                                      uint64_t producer_deadline_runtime_monotonic_ns);
oa_runtime_status oa_runtime_set_interlock(oa_runtime *runtime, uint32_t estop_active,
                                           uint32_t deadman_active);
oa_runtime_status oa_runtime_stop(oa_runtime *runtime, uint32_t stop_kind);
oa_runtime_status oa_runtime_disarm(oa_runtime *runtime,
                                    oa_runtime_clock_id clock_id,
                                    uint64_t deadline_runtime_monotonic_ns);
oa_runtime_status oa_runtime_poll_event(oa_runtime *runtime,
                                       uint64_t wait_timeout_ns,
                                       oa_runtime_event *out_event);
void oa_runtime_plan_destroy(oa_runtime_plan *plan);

oa_runtime_status oa_runtime_calibration_manual_begin(
    oa_runtime *runtime, const oa_commission_manual_options *options,
    oa_runtime_calibration **out_calibration);
oa_runtime_status oa_runtime_calibration_manual_sample(
    oa_runtime_calibration *calibration, uint32_t reference_index,
    oa_commission_manual_report *out_report);
oa_runtime_status oa_runtime_calibration_manual_begin_review(
    oa_runtime_calibration *calibration, oa_commission_manual_report *out_report);
oa_runtime_status oa_runtime_calibration_manual_commit(
    oa_runtime_calibration *calibration, uint64_t replacement_revision,
    const char *evidence_record, oa_runtime_manifest **out_manifest,
    oa_runtime_manifest_preview *out_preview);
oa_runtime_status oa_runtime_calibration_recipe_begin(
    oa_runtime *runtime, const oa_commission_recipe *recipe,
    oa_runtime_calibration **out_calibration);
oa_runtime_status oa_runtime_calibration_recipe_step(
    oa_runtime_calibration *calibration, const oa_commission_recipe_input *input,
    oa_commission_next_action *out_action, oa_commission_recipe_report *out_report);
oa_runtime_status oa_runtime_calibration_recipe_commit(
    oa_runtime_calibration *calibration, uint64_t replacement_revision,
    oa_runtime_manifest **out_manifest, oa_runtime_manifest_preview *out_preview);
oa_runtime_status oa_runtime_calibration_abort(oa_runtime_calibration *calibration);
void oa_runtime_calibration_destroy(oa_runtime_calibration *calibration);

oa_runtime_status oa_runtime_configuration_preview_physical(
    const oa_runtime_manifest *manifest, const oa_runtime_inventory *inventory,
    oa_runtime_manifest_preview *out_preview);
oa_runtime_status oa_runtime_configuration_apply_physical(
    oa_runtime *runtime, const oa_runtime_manifest *manifest);

#ifdef __cplusplus
}
#endif

#endif
