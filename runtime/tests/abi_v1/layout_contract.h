#ifndef OPENARM_RUNTIME_V1_LAYOUT_CONTRACT_H
#define OPENARM_RUNTIME_V1_LAYOUT_CONTRACT_H
#include <stddef.h>
#if defined(__cplusplus)
#define OA_RUNTIME_V1_ASSERT(c, m) static_assert(c, m)
#define OA_RUNTIME_V1_ALIGNOF(t) alignof(t)
#else
#define OA_RUNTIME_V1_ASSERT(c, m) _Static_assert(c, m)
#define OA_RUNTIME_V1_ALIGNOF(t) _Alignof(t)
#endif
#define OA_RUNTIME_V1_SIZE(type, size) OA_RUNTIME_V1_ASSERT(sizeof(type) == size, #type " layout")
OA_RUNTIME_V1_ASSERT(OA_RUNTIME_ABI_VERSION == 1U, "Runtime ABI version");
OA_RUNTIME_V1_SIZE(oa_runtime_error_detail, 24U);
OA_RUNTIME_V1_SIZE(oa_runtime_options, 48U);
OA_RUNTIME_V1_SIZE(oa_runtime_capability_report, 128U);
OA_RUNTIME_V1_SIZE(oa_runtime_model_identity, 640U);
OA_RUNTIME_V1_SIZE(oa_runtime_interface, 56U);
OA_RUNTIME_V1_SIZE(oa_runtime_motor_evidence, 216U);
OA_RUNTIME_V1_SIZE(oa_runtime_inventory_summary, 112U);
OA_RUNTIME_V1_SIZE(oa_runtime_query_candidate, 12U);
OA_RUNTIME_V1_SIZE(oa_runtime_inventory_query_options, 240U);
OA_RUNTIME_V1_SIZE(oa_runtime_motor_manifest, 304U);
OA_RUNTIME_V1_SIZE(oa_runtime_manifest_summary, 248U);
OA_RUNTIME_V1_SIZE(oa_runtime_persistence_checkpoint, 88U);
OA_RUNTIME_V1_SIZE(oa_runtime_manifest_preview, 56U);
OA_RUNTIME_V1_SIZE(oa_runtime_arm_snapshot, 400U);
OA_RUNTIME_V1_SIZE(oa_runtime_snapshot, 920U);
OA_RUNTIME_V1_SIZE(oa_runtime_kinematics, 920U);
OA_RUNTIME_V1_SIZE(oa_runtime_joint_move, 184U);
OA_RUNTIME_V1_SIZE(oa_runtime_paired_tcp_move, 248U);
OA_RUNTIME_V1_SIZE(oa_runtime_plan_report, 344U);
OA_RUNTIME_V1_SIZE(oa_runtime_execute_request, 48U);
OA_RUNTIME_V1_SIZE(oa_runtime_event, 152U);
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_capability_report, collision_checked) == 32U, "cap collision offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_capability_report, capabilities) == 48U, "capabilities offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_manifest_summary, inventory_fingerprint_sha256) == 52U, "summary fingerprint offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_manifest_summary, content_sha256) == 117U, "summary content offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_snapshot, arm) == 120U, "snapshot arm offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_kinematics, q_model_rad) == 376U, "kinematics offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_joint_move, required_model_revision) == 88U, "joint identity offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_paired_tcp_move, maximum_branch_step_rad) == 232U, "paired branch offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_plan_report, collision_scene_revision) == 88U, "plan scene offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_plan_report, target_q_model_rad) == 168U, "plan targets offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_runtime_event, command_id) == 120U, "event command offset");

#define OA_COMMISSION_V1_LAYOUT(type, size) \
    OA_RUNTIME_V1_ASSERT(sizeof(type) == size, #type " size"); \
    OA_RUNTIME_V1_ASSERT(OA_RUNTIME_V1_ALIGNOF(type) == 8U, #type " alignment")
OA_RUNTIME_V1_ASSERT(OA_COMMISSION_ABI_V1 == 1U, "Commission ABI version");
OA_COMMISSION_V1_LAYOUT(oa_commission_encoder_sample, 72U);
OA_COMMISSION_V1_LAYOUT(oa_commission_manual_options, 168U);
OA_COMMISSION_V1_LAYOUT(oa_commission_manual_report, 80U);
OA_COMMISSION_V1_LAYOUT(oa_commission_mapping_patch, 200U);
OA_COMMISSION_V1_LAYOUT(oa_commission_recipe, 544U);
OA_COMMISSION_V1_LAYOUT(oa_commission_recipe_input, 184U);
OA_COMMISSION_V1_LAYOUT(oa_commission_next_action, 72U);
OA_COMMISSION_V1_LAYOUT(oa_commission_recipe_report, 64U);
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_encoder_sample, feedback_seq) == 8U, "encoder offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_encoder_sample, drive_fault) == 68U, "encoder tail");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_manual_options, expected_revision) == 16U, "manual options offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_manual_options, motor_serial) == 104U, "manual options tail");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_manual_report, accepted_samples) == 16U, "manual report offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_manual_report, candidate_b_rad) == 72U, "manual report tail");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_mapping_patch, expected_revision) == 8U, "mapping patch offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_mapping_patch, evidence_record) == 136U, "mapping patch tail");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe, expected_revision) == 40U, "recipe offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe, required_posture_output_rad) == 224U, "recipe posture offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe, fixture_record) == 480U, "recipe tail");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe_input, now_ns) == 8U, "recipe input offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe_input, encoder) == 16U, "recipe input encoder");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe_input, posture_output_rad) == 128U, "recipe input tail");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_next_action, valid_until_ns) == 16U, "next action offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_next_action, maximum_temperature_c) == 64U, "next action tail");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe_report, last_feedback_seq) == 16U, "recipe report offset");
OA_RUNTIME_V1_ASSERT(offsetof(oa_commission_recipe_report, accumulated_contact_energy_j) == 56U, "recipe report tail");
#endif
