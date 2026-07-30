#ifndef OPENARM_RUNTIME_V1_SYMBOL_REFERENCES_H
#define OPENARM_RUNTIME_V1_SYMBOL_REFERENCES_H
#if defined(__GNUC__)
#define OA_V1_RETAIN __attribute__((used, retain, section("oa_runtime_v1_refs")))
#else
#define OA_V1_RETAIN
#endif
#define OA_V1_REF(result, name, arguments) result (*const oa_v1_ref_##name) arguments OA_V1_RETAIN = &name
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_create_virtual, (oa_runtime_manifest **));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_get_summary, (const oa_runtime_manifest *, oa_runtime_manifest_summary *));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_get_motor, (const oa_runtime_manifest *, uint32_t, uint32_t, oa_runtime_motor_manifest *));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_preview_file, (const char *, oa_runtime_manifest_preview *));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_load, (const char *, oa_runtime_manifest **));
OA_V1_REF(oa_runtime_status, oa_runtime_persistence_authority_create, (const char *, const uint8_t *, const char *, oa_runtime_persistence_authority **));
OA_V1_REF(oa_runtime_status, oa_runtime_persistence_authority_open_v2, (const char *, const uint8_t *, const char *, const oa_runtime_persistence_checkpoint *, oa_runtime_persistence_authority **));
OA_V1_REF(void, oa_runtime_persistence_authority_destroy, (oa_runtime_persistence_authority *));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_load_authenticated, (const oa_runtime_persistence_authority *, const char *, oa_runtime_manifest **));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_save, (const oa_runtime_manifest *, const oa_runtime_persistence_authority *, const char *));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_load_authenticated_v2, (const oa_runtime_persistence_authority *, const char *, const oa_runtime_persistence_checkpoint *, oa_runtime_manifest **, oa_runtime_persistence_checkpoint *));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_save_v2, (const oa_runtime_manifest *, const oa_runtime_persistence_authority *, const char *, const oa_runtime_persistence_checkpoint *, oa_runtime_persistence_checkpoint *));
OA_V1_REF(oa_runtime_status, oa_runtime_manifest_recover_v2, (const oa_runtime_persistence_authority *, const char *, const oa_runtime_persistence_checkpoint *, oa_runtime_manifest **, oa_runtime_persistence_checkpoint *));
OA_V1_REF(void, oa_runtime_manifest_destroy, (oa_runtime_manifest *));
OA_V1_REF(oa_runtime_status, oa_runtime_create, (const oa_runtime_options *, const oa_runtime_manifest *, oa_runtime **));
OA_V1_REF(void, oa_runtime_destroy, (oa_runtime *));
OA_V1_REF(oa_runtime_status, oa_runtime_get_capabilities, (const oa_runtime *, oa_runtime_capability_report *));
OA_V1_REF(oa_runtime_status, oa_runtime_get_model_identity, (const oa_runtime *, uint32_t, oa_runtime_model_identity *));
OA_V1_REF(oa_runtime_status, oa_runtime_now_monotonic_ns, (const oa_runtime *, oa_runtime_clock_id, uint64_t *));
OA_V1_REF(oa_runtime_status, oa_runtime_get_last_error, (const oa_runtime *, oa_runtime_error_detail *));
OA_V1_REF(oa_runtime_status, oa_runtime_list_interfaces, (const oa_runtime *, oa_runtime_interface *, size_t, size_t *));
OA_V1_REF(oa_runtime_status, oa_runtime_inventory_query, (oa_runtime *, const oa_runtime_inventory_query_options *, oa_runtime_inventory **));
OA_V1_REF(oa_runtime_status, oa_runtime_inventory_get_summary, (const oa_runtime_inventory *, oa_runtime_inventory_summary *));
OA_V1_REF(oa_runtime_status, oa_runtime_inventory_get_interface, (const oa_runtime_inventory *, size_t, oa_runtime_interface *));
OA_V1_REF(oa_runtime_status, oa_runtime_inventory_get_motor, (const oa_runtime_inventory *, size_t, oa_runtime_motor_evidence *));
OA_V1_REF(void, oa_runtime_inventory_destroy, (oa_runtime_inventory *));
OA_V1_REF(oa_runtime_status, oa_runtime_snapshot_get, (oa_runtime *, oa_runtime_snapshot *));
OA_V1_REF(oa_runtime_status, oa_runtime_get_kinematics, (oa_runtime *, uint32_t, uint64_t, oa_runtime_kinematics *));
OA_V1_REF(oa_runtime_status, oa_runtime_arm_virtual, (oa_runtime *));
OA_V1_REF(oa_runtime_status, oa_runtime_plan_joint, (oa_runtime *, const oa_runtime_joint_move *, oa_runtime_plan **));
OA_V1_REF(oa_runtime_status, oa_runtime_plan_paired_tcp_body, (oa_runtime *, const oa_runtime_paired_tcp_move *, oa_runtime_plan **));
OA_V1_REF(oa_runtime_status, oa_runtime_plan_get_report, (const oa_runtime_plan *, oa_runtime_plan_report *));
OA_V1_REF(oa_runtime_status, oa_runtime_execute, (oa_runtime *, const oa_runtime_plan *, const oa_runtime_execute_request *, uint64_t *));
OA_V1_REF(oa_runtime_status, oa_runtime_heartbeat, (oa_runtime *, uint64_t, oa_runtime_clock_id, uint64_t));
OA_V1_REF(oa_runtime_status, oa_runtime_set_interlock, (oa_runtime *, uint32_t, uint32_t));
OA_V1_REF(oa_runtime_status, oa_runtime_stop, (oa_runtime *, uint32_t));
OA_V1_REF(oa_runtime_status, oa_runtime_disarm, (oa_runtime *, oa_runtime_clock_id, uint64_t));
OA_V1_REF(oa_runtime_status, oa_runtime_poll_event, (oa_runtime *, uint64_t, oa_runtime_event *));
OA_V1_REF(void, oa_runtime_plan_destroy, (oa_runtime_plan *));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_manual_begin, (oa_runtime *, const oa_commission_manual_options *, oa_runtime_calibration **));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_manual_sample, (oa_runtime_calibration *, uint32_t, oa_commission_manual_report *));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_manual_begin_review, (oa_runtime_calibration *, oa_commission_manual_report *));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_manual_commit, (oa_runtime_calibration *, uint64_t, const char *, oa_runtime_manifest **, oa_runtime_manifest_preview *));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_recipe_begin, (oa_runtime *, const oa_commission_recipe *, oa_runtime_calibration **));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_recipe_step, (oa_runtime_calibration *, const oa_commission_recipe_input *, oa_commission_next_action *, oa_commission_recipe_report *));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_recipe_commit, (oa_runtime_calibration *, uint64_t, oa_runtime_manifest **, oa_runtime_manifest_preview *));
OA_V1_REF(oa_runtime_status, oa_runtime_calibration_abort, (oa_runtime_calibration *));
OA_V1_REF(void, oa_runtime_calibration_destroy, (oa_runtime_calibration *));
OA_V1_REF(oa_runtime_status, oa_runtime_configuration_preview_physical, (const oa_runtime_manifest *, const oa_runtime_inventory *, oa_runtime_manifest_preview *));
OA_V1_REF(oa_runtime_status, oa_runtime_configuration_apply_physical, (oa_runtime *, const oa_runtime_manifest *));
#endif
