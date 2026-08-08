/* SPDX-License-Identifier: Apache-2.0 */
#include <openarm_real.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(FILE *stream) {
    fprintf(stream,
        "Usage:\n"
        "  openarm_real_cli ready|read|connect|disconnect|stop|estop|estop-clear|neutral\n"
        "  openarm_real_cli joint left|right 1..7 TARGET_RAD\n"
        "  openarm_real_cli tcp left|right m|cm|in X Y Z [SCALE]\n"
        "  openarm_real_cli paired m|cm|in LX LY LZ RX RY RZ [SCALE]\n"
        "  openarm_real_cli gripper left|right|both OPENING_M TORQUE_NM SPEED_M_S [CONTACT]\n"
        "CAN is configured separately with: bash scripts/setup_can_interfaces.sh\n");
}

static int number(const char *text, double *out) {
    char *end = NULL;
    double value;
    errno = 0;
    value = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(value)) return 0;
    *out = value;
    return 1;
}

static int unsigned_number(const char *text, unsigned long *out) {
    char *end = NULL;
    unsigned long value;
    errno = 0;
    value = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') return 0;
    *out = value;
    return 1;
}

static int side(const char *text, oa_real_side *out) {
    if (strcmp(text, "left") == 0) {*out = OA_REAL_SIDE_LEFT; return 1;}
    if (strcmp(text, "right") == 0) {*out = OA_REAL_SIDE_RIGHT; return 1;}
    return 0;
}

static int gripper_mask(const char *text, oa_real_gripper_mask *out) {
    if (strcmp(text, "left") == 0) {*out = OA_REAL_GRIPPER_LEFT; return 1;}
    if (strcmp(text, "right") == 0) {*out = OA_REAL_GRIPPER_RIGHT; return 1;}
    if (strcmp(text, "both") == 0) {*out = OA_REAL_GRIPPER_BOTH; return 1;}
    return 0;
}

static int unit(const char *text, oa_length_unit *out) {
    if (strcmp(text, "m") == 0) {*out = OA_LENGTH_UNIT_METRES; return 1;}
    if (strcmp(text, "cm") == 0) {*out = OA_LENGTH_UNIT_CENTIMETRES; return 1;}
    if (strcmp(text, "in") == 0) {*out = OA_LENGTH_UNIT_INCHES; return 1;}
    return 0;
}

static int report(oa_real_status status, const oa_real_result *result) {
    printf("status=%s (%d) command_id=%llu progress=%.17g collision_checked=%u reason=%s\n",
        oa_real_status_string(status), (int)status,
        (unsigned long long)result->command_id, result->measured_progress,
        result->collision_checked, result->reason);
    return status == OA_REAL_OK ? 0 : 1;
}

static int read_snapshot(oa_real_client *client) {
    oa_real_snapshot value;
    size_t arm;
    size_t joint;
    oa_real_status status;
    oa_real_snapshot_init(&value);
    status = oa_real_client_read(client, 5000U, &value);
    if (status != OA_REAL_OK) {
        fprintf(stderr, "read: %s (%d)\n", oa_real_status_string(status), (int)status);
        return 1;
    }
    printf("controller_available=%u connected=%u armed=%u estop=%u busy=%u encoder_valid=%u active_side_mask=%u\n",
        value.controller_available, value.connected, value.armed,
        value.estop_asserted, value.busy, value.encoder_state_valid,
        value.active_side_mask);
    if (!value.encoder_state_valid) return 0;
    for (arm = 0U; arm < OA_REAL_ARMS; ++arm) {
        printf("%s q_rad=[", arm == 0U ? "left" : "right");
        for (joint = 0U; joint < OA_REAL_JOINTS; ++joint) {
            printf("%s%.17g", joint == 0U ? "" : ",", value.joint_position_rad[arm][joint]);
        }
        printf("] tcp_m=[%.17g,%.17g,%.17g] gripper_m=%.17g\n",
            value.tcp_m[arm][0], value.tcp_m[arm][1], value.tcp_m[arm][2],
            value.gripper_opening_m[arm]);
    }
    return 0;
}

int main(int argc, char **argv) {
    oa_real_client *client = NULL;
    oa_real_result result;
    oa_real_status status;
    int exit_status = 1;
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        usage(argc < 2 ? stderr : stdout);
        return argc < 2 ? 2 : 0;
    }
    status = oa_real_client_create(&client);
    if (status != OA_REAL_OK) {
        fprintf(stderr, "create: %s (%d)\n", oa_real_status_string(status), (int)status);
        return 1;
    }
    oa_real_result_init(&result);
    status = oa_real_client_wait_ready(client, 5000U, &result);
    if (status != OA_REAL_OK) {
        exit_status = report(status, &result);
        goto done;
    }
    if (strcmp(argv[1], "ready") == 0 && argc == 2) {
        exit_status = report(status, &result);
    } else if (strcmp(argv[1], "read") == 0 && argc == 2) {
        exit_status = read_snapshot(client);
    } else if (argc == 2 && strcmp(argv[1], "connect") == 0) {
        exit_status = report(oa_real_client_connect(client, 15000U, &result), &result);
    } else if (argc == 2 && strcmp(argv[1], "disconnect") == 0) {
        exit_status = report(oa_real_client_disconnect(client, 10000U, &result), &result);
    } else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        exit_status = report(oa_real_client_stop(client, 10000U, &result), &result);
    } else if (argc == 2 && strcmp(argv[1], "estop") == 0) {
        exit_status = report(oa_real_client_estop(client, 10000U, &result), &result);
    } else if (argc == 2 && strcmp(argv[1], "estop-clear") == 0) {
        exit_status = report(oa_real_client_estop_clear(client, 10000U, &result), &result);
    } else if (argc == 2 && strcmp(argv[1], "neutral") == 0) {
        exit_status = report(oa_real_client_neutral(client, 10000U, &result), &result);
    } else if (argc == 5 && strcmp(argv[1], "joint") == 0) {
        oa_real_side arm;
        unsigned long joint;
        double target;
        if (!side(argv[2], &arm) || !unsigned_number(argv[3], &joint) ||
            !number(argv[4], &target) || joint > UINT32_MAX) goto invalid;
        exit_status = report(oa_real_client_move_joint(client, arm, (uint32_t)joint,
            target, 120000U, &result), &result);
    } else if ((argc == 7 || argc == 8) && strcmp(argv[1], "tcp") == 0) {
        oa_real_side arm;
        oa_length_unit coordinate_unit;
        oa_vec3d target;
        double scale = 0.5;
        if (!side(argv[2], &arm) || !unit(argv[3], &coordinate_unit) ||
            !number(argv[4], &target.x) || !number(argv[5], &target.y) ||
            !number(argv[6], &target.z) || (argc == 8 && !number(argv[7], &scale))) goto invalid;
        exit_status = report(oa_real_client_move_tcp(client, arm, &target,
            coordinate_unit, scale, 120000U, &result), &result);
    } else if ((argc == 9 || argc == 10) && strcmp(argv[1], "paired") == 0) {
        oa_length_unit coordinate_unit;
        oa_vec3d left;
        oa_vec3d right;
        double scale = 0.5;
        if (!unit(argv[2], &coordinate_unit) || !number(argv[3], &left.x) ||
            !number(argv[4], &left.y) || !number(argv[5], &left.z) ||
            !number(argv[6], &right.x) || !number(argv[7], &right.y) ||
            !number(argv[8], &right.z) || (argc == 10 && !number(argv[9], &scale))) goto invalid;
        exit_status = report(oa_real_client_move_paired_tcp(client, &left, &right,
            coordinate_unit, scale, 120000U, &result), &result);
    } else if ((argc == 6 || argc == 7) && strcmp(argv[1], "gripper") == 0) {
        oa_real_gripper_mask mask;
        double opening;
        double torque;
        double speed;
        unsigned long contact = 0U;
        if (!gripper_mask(argv[2], &mask) || !number(argv[3], &opening) ||
            !number(argv[4], &torque) || !number(argv[5], &speed) ||
            (argc == 7 && !unsigned_number(argv[6], &contact)))
            goto invalid;
        exit_status = report(oa_real_client_move_gripper(client, mask, opening, speed,
            torque, (uint32_t)contact, 120000U, &result), &result);
    } else {
invalid:
        usage(stderr);
        exit_status = 2;
    }
done:
    oa_real_client_destroy(client);
    return exit_status;
}
