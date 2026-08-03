/* SPDX-License-Identifier: Apache-2.0 */
/* Prove the visualization executable has no CAN dependency or PF_CAN syscall.
 * C port of the former test_no_can_linkage.py. */
#include "test_support.h"

#include <ctype.h>

/* The Python original matched
 *   \b(?:oa_controller_|oa_motion_plan_|oa_manifest_(?!runtime))\w+
 * over nm output. Reproduced here without a regex engine: scan for each
 * prefix at a symbol boundary, then apply the manifest negative lookahead. */
static int forbidden_symbol_at(const char *text, size_t index, char *out_symbol,
                               size_t capacity) {
    static const char *const prefixes[] = {"oa_controller_", "oa_motion_plan_",
                                           "oa_manifest_"};
    if (index > 0u && (isalnum((unsigned char)text[index - 1u]) ||
                       text[index - 1u] == '_')) {
        return 0; /* not on a word boundary */
    }
    for (size_t which = 0u; which < 3u; ++which) {
        const char *const prefix = prefixes[which];
        const size_t length = strlen(prefix);
        if (strncmp(text + index, prefix, length) != 0) {
            continue;
        }
        /* oa_manifest_runtime* is the permitted runtime facade spelling. */
        if (which == 2u && strncmp(text + index + length, "runtime", 7u) == 0) {
            return 0;
        }
        size_t end = index + length;
        while (isalnum((unsigned char)text[end]) || text[end] == '_') {
            ++end;
        }
        if (end == index + length) {
            return 0; /* prefix with no trailing \w+ */
        }
        const size_t symbol_length = end - index;
        if (symbol_length + 1u > capacity) {
            return 0;
        }
        memcpy(out_symbol, text + index, symbol_length);
        out_symbol[symbol_length] = '\0';
        return 1;
    }
    return 0;
}

int main(int argc, char **argv) {
    const char *const executable = oa_required_argument(argc, argv, "--executable");
    const char *const session_library =
        oa_required_argument(argc, argv, "--session-library");

    /* 1. The session must consume the runtime facade, not the control layer. */
    oa_buffer undefined = {NULL, 0u, 0u};
    {
        char *const command[] = {(char *)"nm", (char *)"-u",
                                 (char *)session_library, NULL};
        const int status = oa_run_capture(command, 60.0, &undefined, NULL);
        if (oa_exit_code(status) != 0) {
            oa_fail("nm -u failed on %s:\n%s", session_library,
                    undefined.data == NULL ? "" : undefined.data);
        }
    }
    if (undefined.data != NULL) {
        char symbol[256];
        for (size_t index = 0u; index < undefined.size; ++index) {
            if (forbidden_symbol_at(undefined.data, index, symbol, sizeof(symbol))) {
                oa_fail("session bypasses OpenArm::Runtime: %s", symbol);
            }
        }
    }
    if (!oa_contains(&undefined, "oa_runtime_create") ||
        !oa_contains(&undefined, "oa_runtime_snapshot_get")) {
        oa_fail("session does not consume the runtime facade:\n%s",
                undefined.data == NULL ? "" : undefined.data);
    }
    oa_buffer_free(&undefined);

    /* 2. The executable must not link any CAN library. */
    oa_buffer linked = {NULL, 0u, 0u};
    {
        char *const command[] = {(char *)"ldd", (char *)executable, NULL};
        const int status = oa_run_capture(command, 60.0, &linked, NULL);
        if (oa_exit_code(status) != 0) {
            oa_fail("ldd failed on %s:\n%s", executable,
                    linked.data == NULL ? "" : linked.data);
        }
    }
    if (oa_contains_nocase(&linked, "openarm_can") ||
        oa_contains_nocase(&linked, "socketcan") ||
        oa_contains_nocase(&linked, "libcan")) {
        oa_fail("CAN linkage found:\n%s", linked.data);
    }
    oa_buffer_free(&linked);

    /* 3. The running node must never open a PF_CAN socket. */
    char trace_path[] = "/tmp/openarm-no-can-XXXXXX";
    const int trace_fd = mkstemp(trace_path);
    if (trace_fd < 0) {
        oa_fail("cannot create trace file: %s", strerror(errno));
    }
    close(trace_fd);

    oa_buffer strace_output = {NULL, 0u, 0u};
    {
        char *const command[] = {(char *)"strace",  (char *)"-f",
                                 (char *)"-e",      (char *)"trace=socket",
                                 (char *)"-o",      trace_path,
                                 (char *)"timeout", (char *)"1",
                                 (char *)executable, NULL};
        const int status = oa_run_capture(command, 60.0, &strace_output, NULL);
        const int code = oa_exit_code(status);
        /* 124 is `timeout` reporting the expected deadline. */
        if (code != 0 && code != 124) {
            unlink(trace_path);
            oa_fail("node failed during syscall isolation check (exit %d):\n%s", code,
                    strace_output.data == NULL ? "" : strace_output.data);
        }
    }
    oa_buffer_free(&strace_output);

    size_t trace_size = 0u;
    char *const trace = oa_read_file(trace_path, &trace_size);
    unlink(trace_path);
    if (trace != NULL &&
        (strstr(trace, "AF_CAN") != NULL || strstr(trace, "PF_CAN") != NULL)) {
        oa_fail("PF_CAN syscall found:\n%s", trace);
    }
    free(trace);
    return 0;
}
