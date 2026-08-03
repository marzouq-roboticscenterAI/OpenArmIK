/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal process and string helpers shared by the C test programs that
 * replaced the previous Python harnesses. */
#ifndef OPENARM_TEST_SUPPORT_H
#define OPENARM_TEST_SUPPORT_H

/* These programs are built as strict C11, which hides the POSIX process,
 * timing, and string declarations they rely on. */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include <errno.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/* Growable byte buffer. */
typedef struct {
    char *data;
    size_t size;
    size_t capacity;
} oa_buffer;

static inline void oa_fail(const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    vfprintf(stderr, format, arguments);
    va_end(arguments);
    fputc('\n', stderr);
    exit(1);
}

static inline void oa_buffer_free(oa_buffer *buffer) {
    free(buffer->data);
    buffer->data = NULL;
    buffer->size = 0;
    buffer->capacity = 0;
}

static inline void oa_buffer_append(oa_buffer *buffer, const char *bytes, size_t length) {
    if (buffer->size + length + 1u > buffer->capacity) {
        size_t capacity = buffer->capacity == 0u ? 4096u : buffer->capacity;
        while (capacity < buffer->size + length + 1u) {
            capacity *= 2u;
        }
        char *grown = realloc(buffer->data, capacity);
        if (grown == NULL) {
            oa_fail("out of memory growing capture buffer");
        }
        buffer->data = grown;
        buffer->capacity = capacity;
    }
    memcpy(buffer->data + buffer->size, bytes, length);
    buffer->size += length;
    buffer->data[buffer->size] = '\0';
}

/* Runs argv, capturing stdout and stderr together. Kills the child after
 * timeout_seconds (0 disables the timeout). Returns the wait status; sets
 * *timed_out when the child had to be killed. */
static inline int oa_run_capture(char *const argv[], double timeout_seconds,
                          oa_buffer *out_output, int *out_timed_out) {
    int pipe_fds[2];
    if (pipe(pipe_fds) != 0) {
        oa_fail("pipe failed: %s", strerror(errno));
    }
    const pid_t child = fork();
    if (child < 0) {
        oa_fail("fork failed: %s", strerror(errno));
    }
    if (child == 0) {
        close(pipe_fds[0]);
        dup2(pipe_fds[1], STDOUT_FILENO);
        dup2(pipe_fds[1], STDERR_FILENO);
        close(pipe_fds[1]);
        /* A fresh process group so a timeout can reap the whole tree. */
        setpgid(0, 0);
        execvp(argv[0], argv);
        _exit(127);
    }
    close(pipe_fds[1]);
    setpgid(child, child);

    if (out_timed_out != NULL) {
        *out_timed_out = 0;
    }
    const double deadline_step = 0.01;
    double waited = 0.0;
    int status = 0;
    int reaped = 0;
    for (;;) {
        char chunk[4096];
        const ssize_t bytes = read(pipe_fds[0], chunk, sizeof(chunk));
        if (bytes > 0) {
            if (out_output != NULL) {
                oa_buffer_append(out_output, chunk, (size_t)bytes);
            }
            continue;
        }
        if (bytes == 0) {
            break;
        }
        if (errno != EINTR && errno != EAGAIN) {
            break;
        }
    }
    close(pipe_fds[0]);
    while (!reaped) {
        const pid_t done = waitpid(child, &status, WNOHANG);
        if (done == child) {
            reaped = 1;
            break;
        }
        if (done < 0) {
            oa_fail("waitpid failed: %s", strerror(errno));
        }
        if (timeout_seconds > 0.0 && waited >= timeout_seconds) {
            kill(-child, SIGKILL);
            waitpid(child, &status, 0);
            if (out_timed_out != NULL) {
                *out_timed_out = 1;
            }
            reaped = 1;
            break;
        }
        struct timespec sleep_for;
        sleep_for.tv_sec = 0;
        sleep_for.tv_nsec = (long)(deadline_step * 1.0e9);
        nanosleep(&sleep_for, NULL);
        waited += deadline_step;
    }
    return status;
}

/* Convenience: exit code, or -1 when the child did not exit normally. */
static inline int oa_exit_code(const int status) {
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static inline int oa_contains(const oa_buffer *buffer, const char *needle) {
    return buffer->data != NULL && strstr(buffer->data, needle) != NULL;
}

/* Case-insensitive substring search. */
static inline int oa_contains_nocase(const oa_buffer *buffer, const char *needle) {
    if (buffer->data == NULL) {
        return 0;
    }
    const size_t needle_length = strlen(needle);
    if (needle_length == 0u) {
        return 1;
    }
    for (size_t index = 0; index + needle_length <= buffer->size; ++index) {
        if (strncasecmp(buffer->data + index, needle, needle_length) == 0) {
            return 1;
        }
    }
    return 0;
}

static inline char *oa_read_file(const char *path, size_t *out_size) {
    FILE *const stream = fopen(path, "rb");
    if (stream == NULL) {
        oa_fail("cannot open %s: %s", path, strerror(errno));
    }
    oa_buffer buffer = {NULL, 0u, 0u};
    for (;;) {
        char chunk[8192];
        const size_t bytes = fread(chunk, 1u, sizeof(chunk), stream);
        if (bytes > 0u) {
            oa_buffer_append(&buffer, chunk, bytes);
        }
        if (bytes < sizeof(chunk)) {
            break;
        }
    }
    fclose(stream);
    if (out_size != NULL) {
        *out_size = buffer.size;
    }
    return buffer.data;
}

/* Returns the value of --name from argv, or NULL. */
static inline const char *oa_argument(int argc, char **argv, const char *name) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (strcmp(argv[index], name) == 0) {
            return argv[index + 1];
        }
    }
    return NULL;
}

static inline const char *oa_required_argument(int argc, char **argv, const char *name) {
    const char *const value = oa_argument(argc, argv, name);
    if (value == NULL) {
        oa_fail("missing required argument %s", name);
    }
    return value;
}

#endif
