/* SPDX-License-Identifier: Apache-2.0 */
/* The ROS parameter boundary rejects unsafe expiry conversion before node
 * startup. C port of the former test_invalid_expiry_parameter.py. */
#include "test_support.h"

int main(void) {
    static const char *const rejected[] = {"0", "-1", "9223372036854775807", "60001"};
    for (size_t index = 0u; index < sizeof(rejected) / sizeof(rejected[0]); ++index) {
        char parameter[128];
        snprintf(parameter, sizeof(parameter), "request_expiry_ms:=%s", rejected[index]);
        char *const command[] = {(char *)"ros2",
                                 (char *)"run",
                                 (char *)"openarm_ik_ros",
                                 (char *)"openarm_ik_ros_node",
                                 (char *)"--ros-args",
                                 (char *)"-p",
                                 parameter,
                                 NULL};
        oa_buffer output = {NULL, 0u, 0u};
        int timed_out = 0;
        const int status = oa_run_capture(command, 5.0, &output, &timed_out);
        /* A node that survives the 5 s window has accepted the value, which is
         * exactly the failure this test exists to catch. */
        if (timed_out) {
            oa_buffer_free(&output);
            oa_fail("unsafe expiry %s unexpectedly kept the node running",
                    rejected[index]);
        }
        if (oa_exit_code(status) == 0) {
            oa_buffer_free(&output);
            oa_fail("unsafe expiry %s unexpectedly started the node", rejected[index]);
        }
        oa_buffer_free(&output);
    }
    return 0;
}
