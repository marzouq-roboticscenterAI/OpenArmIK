/* SPDX-License-Identifier: Apache-2.0
 *
 * Hand-guided joint range calibration for a physically connected OpenArm.
 *
 * You move each joint by hand through its full travel while this records what
 * the encoder reports. Select an arm and a motor, press Start, move the joint
 * from one hard stop to the other, press Stop. Repeat for all sixteen motors,
 * then Save.
 *
 * READ ONLY, by construction. The motors are never enabled, so the arms stay
 * limp and you are moving dead weight rather than fighting a servo. The only
 * frame this program can transmit is DaMiao refresh-status, built here from a
 * motor ID by oa_can_make_refresh_status; there is no code path that takes a
 * caller-supplied payload, and no enable, zero, motion or save encoder is
 * referenced anywhere in this file.
 *
 * Why the whole path is recorded and not just the endpoints:
 *
 *   Endpoints alone cannot tell 90 degrees of travel from 270. A joint that
 *   goes A -> B directly and one that overshoots, comes back, and ends at B
 *   have identical first and last samples but very different ranges. Worse, the
 *   DaMiao position field spans +/-12.5 rad and wraps inside its 16 bits, so a
 *   joint crossing that boundary makes a small move look like a 25 rad jump.
 *
 *   So every sample is kept, and the signal is unwrapped as it arrives: each
 *   step is reduced to the shortest equivalent delta before being accumulated.
 *   Extent then comes from the unwrapped minimum and maximum, and path length
 *   from the sum of absolute deltas. The two differ exactly when the joint
 *   doubled back, which is the case endpoints get wrong.
 */
/* The build is strict C11, which hides IFNAMSIZ (net/if.h) and OA_PI (math.h)
 * behind feature-test macros. Ask for them explicitly rather than relaxing the
 * standard for the whole target. */
#define _DEFAULT_SOURCE 1
#define _GNU_SOURCE 1

#include <gtk/gtk.h>
#include <json.h>

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "openarm_can.h"

#define OA_MOTORS_PER_ARM 8u
#define OA_ARMS 2u
/* One hour of samples at the poll rate below. Recording a single joint sweep
 * takes seconds, so this is a runaway guard rather than a real bound. */
#define OA_MAX_SAMPLES 180000u
#define OA_POLL_MS 20
/* Spelled out rather than relying on OA_PI, which is not in standard C. */
#define OA_PI 3.14159265358979323846

typedef struct {
    double elapsed_s;
    double raw_rad;      /* exactly as reported, before unwrapping */
    double unwrapped_rad;
} oa_sample;

typedef struct {
    oa_sample *samples;
    size_t count;
    int recorded;        /* a completed Start..Stop pass exists */
    double minimum;
    double maximum;
    double path_length;  /* sum of |delta|, so doubling back is visible */
    double first_raw;
} oa_track;

typedef struct {
    char interface[2][IFNAMSIZ];
    int socket_fd[2];
    oa_track track[OA_ARMS][OA_MOTORS_PER_ARM];

    /* Recording state */
    int recording;
    unsigned active_arm;
    unsigned active_motor;
    double previous_unwrapped;
    double previous_raw;
    int have_previous;
    struct timespec started_at;

    /* Widgets */
    GtkWidget *window;
    GtkWidget *arm_combo;
    GtkWidget *motor_combo;
    GtkWidget *start_button;
    GtkWidget *stop_button;
    GtkWidget *save_button;
    GtkWidget *live_label;
    GtkWidget *status_label;
    GtkWidget *grid_label;
    GtkWidget *dial;
    guint timer_id;

    /* Live diagnostics. A sweep that reads far smaller than it felt is almost
     * always dropped samples rather than bad scaling, so the drop count and the
     * achieved rate are shown rather than left to be inferred. */
    double dial_start;
    double dial_now;
    double dial_min;
    double dial_max;
    unsigned long reads_ok;
    unsigned long reads_failed;
    double rate_window_start;
    unsigned long rate_window_reads;
    double achieved_hz;
} oa_app;

static double monotonic_seconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
}

/* ---- CAN, read only ---------------------------------------------------- */

static int open_can(const char *name, char *error, size_t error_size) {
    int handle = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct ifreq request;
    struct sockaddr_can address;
    if (handle < 0) {
        snprintf(error, error_size, "socket(%s): %s", name, strerror(errno));
        return -1;
    }
    memset(&request, 0, sizeof(request));
    snprintf(request.ifr_name, IFNAMSIZ, "%s", name);
    if (ioctl(handle, SIOCGIFINDEX, &request) < 0) {
        snprintf(error, error_size, "%s does not exist", name);
        close(handle);
        return -1;
    }
    memset(&address, 0, sizeof(address));
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(handle, (struct sockaddr *)&address, sizeof(address)) < 0) {
        snprintf(error, error_size, "%s is down; run ./scripts/setup_can_interfaces.sh", name);
        close(handle);
        return -1;
    }
    return handle;
}

/* The only transmit in this program. It takes a motor ID, not bytes, so no
 * caller can turn it into a command frame. */
static int request_status(int handle, uint16_t send_id) {
    oa_can_frame frame;
    struct can_frame outgoing;
    memset(&frame, 0, sizeof(frame));
    frame.struct_size = (uint32_t)sizeof(frame);
    frame.abi_version = OA_CAN_ABI_VERSION;
    if (oa_can_make_refresh_status(send_id, &frame) != OA_CAN_OK) return -1;
    memset(&outgoing, 0, sizeof(outgoing));
    outgoing.can_id = frame.can_id;
    outgoing.can_dlc = frame.dlc;
    memcpy(outgoing.data, frame.data, sizeof(outgoing.data));
    return write(handle, &outgoing, sizeof(outgoing)) == (ssize_t)sizeof(outgoing) ? 0 : -1;
}

/* Poll one motor and return its position. Non-zero on failure. */
static int read_position(int handle, uint16_t send_id, double *out_rad) {
    struct can_frame incoming;
    struct pollfd descriptor;
    oa_can_frame frame;
    oa_can_feedback feedback;
    double deadline;

    if (request_status(handle, send_id) != 0) return -1;
    deadline = monotonic_seconds() + 0.030;
    while (monotonic_seconds() < deadline) {
        descriptor.fd = handle;
        descriptor.events = POLLIN;
        descriptor.revents = 0;
        if (poll(&descriptor, 1, 5) <= 0) continue;
        if (read(handle, &incoming, sizeof(incoming)) != (ssize_t)sizeof(incoming)) continue;
        if ((incoming.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) != 0u) continue;
        if (incoming.can_dlc != 8u) continue;
        if ((incoming.can_id & CAN_SFF_MASK) != (uint32_t)(send_id + 0x10u)) continue;

        memset(&frame, 0, sizeof(frame));
        frame.struct_size = (uint32_t)sizeof(frame);
        frame.abi_version = OA_CAN_ABI_VERSION;
        frame.can_id = incoming.can_id & CAN_SFF_MASK;
        frame.dlc = 8u;
        memcpy(frame.data, incoming.data, sizeof(frame.data));

        memset(&feedback, 0, sizeof(feedback));
        feedback.struct_size = (uint32_t)sizeof(feedback);
        feedback.abi_version = OA_CAN_ABI_VERSION;
        /* Position is quantised against +/-12.5 rad for every DaMiao type in
         * this family, so the angle is exact whichever type answered. */
        if (oa_can_decode_feedback(&frame, (uint16_t)frame.can_id,
                                   (uint8_t)(frame.data[0] & 0x0fu),
                                   OA_CAN_MOTOR_DM8009, &feedback) != OA_CAN_OK) continue;
        *out_rad = feedback.position_rad;
        return 0;
    }
    return -1;
}

/* ---- Unwrapping -------------------------------------------------------- */

/* Reduce a step to its shortest equivalent within the +/-12.5 rad field. A
 * genuine jump larger than half the span cannot be produced by a hand-moved
 * joint between two samples 20 ms apart, so anything that large is the field
 * wrapping and is folded back. */
static double shortest_delta(double delta) {
    const double span = 25.0;
    while (delta > span * 0.5) delta -= span;
    while (delta < -span * 0.5) delta += span;
    return delta;
}

/* ---- Dial -------------------------------------------------------------- */

/* A circle showing where the joint is now and how far it has swept. The filled
 * arc spans the recorded minimum to maximum, so the travel is legible at a
 * glance rather than only as a number. */
static gboolean on_draw_dial(GtkWidget *widget, cairo_t *cr, gpointer data) {
    oa_app *app = data;
    const double width = gtk_widget_get_allocated_width(widget);
    const double height = gtk_widget_get_allocated_height(widget);
    const double cx = width * 0.5;
    const double cy = height * 0.5;
    const double radius = (width < height ? width : height) * 0.5 - 14.0;
    const double swept = app->dial_max - app->dial_min;
    char text[64];

    if (radius <= 4.0) return FALSE;

    /* Track */
    cairo_set_line_width(cr, 12.0);
    cairo_set_source_rgb(cr, 0.17, 0.20, 0.25);
    cairo_arc(cr, cx, cy, radius, 0.0, 2.0 * OA_PI);
    cairo_stroke(cr);

    /* Swept arc, measured from where recording started so it grows outward in
     * both directions as the joint is moved. Angles are drawn directly in
     * radians of joint travel, so a full turn of the dial is a full turn of the
     * joint and there is no hidden scaling to misread. */
    if (swept > 1.0e-6) {
        cairo_set_source_rgb(cr, 0.24, 0.57, 0.90);
        cairo_arc(cr, cx, cy, radius,
                  -OA_PI * 0.5 + (app->dial_min - app->dial_start),
                  -OA_PI * 0.5 + (app->dial_max - app->dial_start));
        cairo_stroke(cr);
    }

    /* Needle at the present position */
    cairo_set_line_width(cr, 3.0);
    cairo_set_source_rgb(cr, 0.95, 0.85, 0.35);
    cairo_move_to(cr, cx, cy);
    cairo_line_to(cr,
                  cx + radius * cos(-OA_PI * 0.5 + (app->dial_now - app->dial_start)),
                  cy + radius * sin(-OA_PI * 0.5 + (app->dial_now - app->dial_start)));
    cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0.91, 0.93, 0.96);
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 22.0);
    snprintf(text, sizeof(text), "%.1f deg", swept * 180.0 / OA_PI);
    cairo_text_extents_t extents;
    cairo_text_extents(cr, text, &extents);
    cairo_move_to(cr, cx - extents.width * 0.5, cy + extents.height * 0.5);
    cairo_show_text(cr, text);

    cairo_set_font_size(cr, 12.0);
    snprintf(text, sizeof(text), "swept");
    cairo_text_extents(cr, text, &extents);
    cairo_move_to(cr, cx - extents.width * 0.5, cy + 28.0);
    cairo_show_text(cr, text);
    return FALSE;
}

/* ---- Recording --------------------------------------------------------- */

static void reset_track(oa_track *track) {
    free(track->samples);
    memset(track, 0, sizeof(*track));
}

static gboolean on_tick(gpointer data) {
    oa_app *app = data;
    unsigned arm = (unsigned)gtk_combo_box_get_active(GTK_COMBO_BOX(app->arm_combo));
    unsigned motor = (unsigned)gtk_combo_box_get_active(GTK_COMBO_BOX(app->motor_combo));
    double raw = 0.0;
    char text[512];

    if (app->recording) {
        arm = app->active_arm;
        motor = app->active_motor;
    }
    if (arm >= OA_ARMS || motor >= OA_MOTORS_PER_ARM) return TRUE;

    if (read_position(app->socket_fd[arm], (uint16_t)(motor + 1u), &raw) != 0) {
        app->reads_failed++;
        gtk_label_set_text(GTK_LABEL(app->live_label),
                           "no reply from this motor (check it is powered)");
        return TRUE;
    }
    app->reads_ok++;
    app->rate_window_reads++;
    {
        const double now_s = monotonic_seconds();
        if (app->rate_window_start <= 0.0) app->rate_window_start = now_s;
        if (now_s - app->rate_window_start >= 1.0) {
            app->achieved_hz = (double)app->rate_window_reads / (now_s - app->rate_window_start);
            app->rate_window_start = now_s;
            app->rate_window_reads = 0;
        }
    }

    if (!app->recording) {
        app->dial_start = raw;
        app->dial_now = raw;
        app->dial_min = raw;
        app->dial_max = raw;
        snprintf(text, sizeof(text),
                 "live %+.4f rad (%+.2f deg)   %.0f Hz   reads ok %lu / failed %lu",
                 raw, raw * 180.0 / OA_PI, app->achieved_hz,
                 app->reads_ok, app->reads_failed);
        gtk_label_set_text(GTK_LABEL(app->live_label), text);
        gtk_widget_queue_draw(app->dial);
        return TRUE;
    }

    oa_track *track = &app->track[arm][motor];
    double unwrapped;
    if (!app->have_previous) {
        app->have_previous = 1;
        app->previous_raw = raw;
        app->previous_unwrapped = raw;
        track->first_raw = raw;
        track->minimum = raw;
        track->maximum = raw;
        unwrapped = raw;
        app->dial_start = raw;
    } else {
        const double step = shortest_delta(raw - app->previous_raw);
        unwrapped = app->previous_unwrapped + step;
        track->path_length += fabs(step);
        if (unwrapped < track->minimum) track->minimum = unwrapped;
        if (unwrapped > track->maximum) track->maximum = unwrapped;
    }
    app->previous_raw = raw;
    app->previous_unwrapped = unwrapped;

    if (track->count < OA_MAX_SAMPLES) {
        if (track->count % 256u == 0u) {
            const size_t room = track->count + 256u;
            oa_sample *grown = realloc(track->samples, room * sizeof(oa_sample));
            if (grown == NULL) return TRUE;
            track->samples = grown;
        }
        track->samples[track->count].elapsed_s =
            monotonic_seconds() - ((double)app->started_at.tv_sec +
                                   (double)app->started_at.tv_nsec * 1e-9);
        track->samples[track->count].raw_rad = raw;
        track->samples[track->count].unwrapped_rad = unwrapped;
        track->count++;
    }

    app->dial_now = unwrapped;
    app->dial_min = track->minimum;
    app->dial_max = track->maximum;

    snprintf(text, sizeof(text),
             "RECORDING  now %+.4f rad   extent %.1f deg   path %.1f deg   "
             "%zu samples at %.0f Hz   dropped reads %lu",
             unwrapped,
             (track->maximum - track->minimum) * 180.0 / OA_PI,
             track->path_length * 180.0 / OA_PI,
             track->count, app->achieved_hz, app->reads_failed);
    gtk_label_set_text(GTK_LABEL(app->live_label), text);
    gtk_widget_queue_draw(app->dial);
    return TRUE;
}

static void refresh_grid(oa_app *app) {
    char text[2048];
    size_t used = 0;
    used += (size_t)snprintf(text + used, sizeof(text) - used,
                             "recorded (extent in degrees, '-' not yet done)\n");
    for (unsigned arm = 0; arm < OA_ARMS; ++arm) {
        used += (size_t)snprintf(text + used, sizeof(text) - used, "%-6s ",
                                 app->interface[arm]);
        for (unsigned motor = 0; motor < OA_MOTORS_PER_ARM; ++motor) {
            const oa_track *track = &app->track[arm][motor];
            if (track->recorded) {
                used += (size_t)snprintf(text + used, sizeof(text) - used, "%7.1f",
                                         (track->maximum - track->minimum) * 180.0 / OA_PI);
            } else {
                used += (size_t)snprintf(text + used, sizeof(text) - used, "      -");
            }
        }
        used += (size_t)snprintf(text + used, sizeof(text) - used, "\n");
    }
    gtk_label_set_text(GTK_LABEL(app->grid_label), text);
}

static void on_start(GtkButton *button, gpointer data) {
    oa_app *app = data;
    (void)button;
    app->active_arm = (unsigned)gtk_combo_box_get_active(GTK_COMBO_BOX(app->arm_combo));
    app->active_motor = (unsigned)gtk_combo_box_get_active(GTK_COMBO_BOX(app->motor_combo));
    if (app->active_arm >= OA_ARMS || app->active_motor >= OA_MOTORS_PER_ARM) return;

    /* Starting again discards the previous pass for this motor rather than
     * merging into it, so a botched sweep is simply redone. */
    reset_track(&app->track[app->active_arm][app->active_motor]);
    app->have_previous = 0;
    app->recording = 1;
    app->reads_failed = 0;
    app->reads_ok = 0;
    app->dial_min = 0.0;
    app->dial_max = 0.0;
    clock_gettime(CLOCK_MONOTONIC, &app->started_at);

    gtk_widget_set_sensitive(app->start_button, FALSE);
    gtk_widget_set_sensitive(app->stop_button, TRUE);
    gtk_widget_set_sensitive(app->arm_combo, FALSE);
    gtk_widget_set_sensitive(app->motor_combo, FALSE);
    gtk_label_set_text(GTK_LABEL(app->status_label),
                       "Move this joint slowly from one hard stop to the other, then Stop.");
}

static void on_stop(GtkButton *button, gpointer data) {
    oa_app *app = data;
    oa_track *track;
    char text[256];
    (void)button;
    if (!app->recording) return;
    app->recording = 0;
    track = &app->track[app->active_arm][app->active_motor];
    track->recorded = track->count > 1u;

    gtk_widget_set_sensitive(app->start_button, TRUE);
    gtk_widget_set_sensitive(app->stop_button, FALSE);
    gtk_widget_set_sensitive(app->arm_combo, TRUE);
    gtk_widget_set_sensitive(app->motor_combo, TRUE);

    if (!track->recorded) {
        snprintf(text, sizeof(text), "Too few samples; nothing recorded. Try again.");
    } else if (track->path_length > (track->maximum - track->minimum) * 1.5) {
        /* Path much longer than extent means the joint doubled back. Not an
         * error, but worth saying, because it usually means the sweep was not
         * a clean single pass. */
        snprintf(text, sizeof(text),
                 "Recorded %.1f deg extent from %.1f deg of travel: the joint doubled "
                 "back. Extent is still correct.",
                 (track->maximum - track->minimum) * 180.0 / OA_PI,
                 track->path_length * 180.0 / OA_PI);
    } else {
        snprintf(text, sizeof(text), "Recorded %.1f deg of travel over %zu samples.",
                 (track->maximum - track->minimum) * 180.0 / OA_PI, track->count);
    }
    gtk_label_set_text(GTK_LABEL(app->status_label), text);
    refresh_grid(app);
}

static void on_save(GtkButton *button, gpointer data) {
    oa_app *app = data;
    const char *home = getenv("HOME");
    char path[512];
    struct json_object *root = json_object_new_object();
    struct json_object *arms = json_object_new_array();
    /* Roomy enough for the message plus a full-length path, which is what
     * the truncation warning is about. */
    char text[768];
    unsigned recorded_total = 0;
    (void)button;

    snprintf(path, sizeof(path), "%s/.openarm_calibration.json",
             home != NULL ? home : "/tmp");

    json_object_object_add(root, "schema", json_object_new_string("openarm-hand-calibration-1"));
    json_object_object_add(root, "read_only",
                           json_object_new_string("recorded with motors unpowered; no motor "
                                                  "was enabled, zeroed or commanded"));
    for (unsigned arm = 0; arm < OA_ARMS; ++arm) {
        struct json_object *arm_object = json_object_new_object();
        struct json_object *motors = json_object_new_array();
        json_object_object_add(arm_object, "interface",
                               json_object_new_string(app->interface[arm]));
        for (unsigned motor = 0; motor < OA_MOTORS_PER_ARM; ++motor) {
            const oa_track *track = &app->track[arm][motor];
            struct json_object *motor_object = json_object_new_object();
            struct json_object *path_array = json_object_new_array();
            json_object_object_add(motor_object, "send_id", json_object_new_int((int)motor + 1));
            json_object_object_add(motor_object, "recorded",
                                   json_object_new_boolean(track->recorded));
            if (track->recorded) {
                recorded_total++;
                json_object_object_add(motor_object, "minimum_rad",
                                       json_object_new_double(track->minimum));
                json_object_object_add(motor_object, "maximum_rad",
                                       json_object_new_double(track->maximum));
                json_object_object_add(motor_object, "extent_rad",
                                       json_object_new_double(track->maximum - track->minimum));
                json_object_object_add(motor_object, "path_length_rad",
                                       json_object_new_double(track->path_length));
                json_object_object_add(motor_object, "first_raw_rad",
                                       json_object_new_double(track->first_raw));
                json_object_object_add(motor_object, "sample_count",
                                       json_object_new_int((int)track->count));
                /* The whole path, not just the endpoints. This is what lets a
                 * reader tell a clean 90 degree sweep from an overshoot that
                 * ended in the same place, and what makes the unwrapping
                 * auditable after the fact. */
                for (size_t index = 0; index < track->count; ++index) {
                    struct json_object *point = json_object_new_array();
                    json_object_array_add(point,
                        json_object_new_double(track->samples[index].elapsed_s));
                    json_object_array_add(point,
                        json_object_new_double(track->samples[index].raw_rad));
                    json_object_array_add(point,
                        json_object_new_double(track->samples[index].unwrapped_rad));
                    json_object_array_add(path_array, point);
                }
            }
            json_object_object_add(motor_object, "path_t_raw_unwrapped", path_array);
            json_object_array_add(motors, motor_object);
        }
        json_object_object_add(arm_object, "motors", motors);
        json_object_array_add(arms, arm_object);
    }
    json_object_object_add(root, "buses", arms);

    if (json_object_to_file_ext(path, root, JSON_C_TO_STRING_PRETTY) == 0) {
        snprintf(text, sizeof(text), "Saved %u recorded motor(s) to %s", recorded_total, path);
    } else {
        snprintf(text, sizeof(text), "Could not write %s", path);
    }
    gtk_label_set_text(GTK_LABEL(app->status_label), text);
    json_object_put(root);
}

/* ---- GUI --------------------------------------------------------------- */

static void build_gui(oa_app *app) {
    GtkWidget *box, *row, *button_row, *frame;
    PangoAttrList *monospace;
    char label[128];

    app->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(app->window), "OpenArm hand calibration (read only)");
    gtk_window_set_default_size(GTK_WINDOW(app->window), 880, 480);
    gtk_container_set_border_width(GTK_CONTAINER(app->window), 12);
    g_signal_connect(app->window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_add(GTK_CONTAINER(app->window), box);

    frame = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(frame),
        "<b>The motors are not powered.</b> Select an arm and motor, press Start, move that "
        "joint by hand from one hard stop to the other, then press Stop. Repeat for every "
        "motor, then Save.");
    gtk_label_set_line_wrap(GTK_LABEL(frame), TRUE);
    gtk_label_set_max_width_chars(GTK_LABEL(frame), 88);
    gtk_label_set_xalign(GTK_LABEL(frame), 0.0f);
    gtk_box_pack_start(GTK_BOX(box), frame, FALSE, FALSE, 0);

    /* Two rows rather than one. Five controls on a single row get squeezed at
     * narrow window widths, and a combo that is too narrow truncates its text
     * rather than shrinking gracefully. */
    row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_box_pack_start(GTK_BOX(box), row, FALSE, FALSE, 0);

    app->arm_combo = gtk_combo_box_text_new();
    for (unsigned arm = 0; arm < OA_ARMS; ++arm) {
        snprintf(label, sizeof(label), "%s", app->interface[arm]);
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(app->arm_combo), label);
    }
    gtk_combo_box_set_active(GTK_COMBO_BOX(app->arm_combo), 0);
    gtk_widget_set_size_request(app->arm_combo, 120, -1);
    gtk_box_pack_start(GTK_BOX(row), gtk_label_new("Arm:"), FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(row), app->arm_combo, FALSE, FALSE, 0);

    app->motor_combo = gtk_combo_box_text_new();
    for (unsigned motor = 0; motor < OA_MOTORS_PER_ARM; ++motor) {
        if (motor < 7u) {
            snprintf(label, sizeof(label), "motor 0x%02x  (joint %u)", motor + 1u, motor + 1u);
        } else {
            snprintf(label, sizeof(label), "motor 0x%02x  (gripper)", motor + 1u);
        }
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(app->motor_combo), label);
    }
    gtk_combo_box_set_active(GTK_COMBO_BOX(app->motor_combo), 0);
    gtk_widget_set_size_request(app->motor_combo, 220, -1);
    gtk_box_pack_start(GTK_BOX(row), gtk_label_new("Motor:"), FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(row), app->motor_combo, FALSE, FALSE, 0);

    button_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_box_pack_start(GTK_BOX(box), button_row, FALSE, FALSE, 0);

    app->start_button = gtk_button_new_with_label("Start");
    app->stop_button = gtk_button_new_with_label("Stop");
    app->save_button = gtk_button_new_with_label("Save JSON");
    gtk_widget_set_sensitive(app->stop_button, FALSE);
    gtk_widget_set_size_request(app->start_button, 110, -1);
    gtk_widget_set_size_request(app->stop_button, 110, -1);
    gtk_widget_set_size_request(app->save_button, 130, -1);
    gtk_box_pack_start(GTK_BOX(button_row), app->start_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(button_row), app->stop_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(button_row), app->save_button, FALSE, FALSE, 0);
    g_signal_connect(app->start_button, "clicked", G_CALLBACK(on_start), app);
    g_signal_connect(app->stop_button, "clicked", G_CALLBACK(on_stop), app);
    g_signal_connect(app->save_button, "clicked", G_CALLBACK(on_save), app);

    app->live_label = gtk_label_new("live: waiting for a reading");
    gtk_label_set_xalign(GTK_LABEL(app->live_label), 0.0f);
    gtk_label_set_line_wrap(GTK_LABEL(app->live_label), TRUE);
    gtk_label_set_selectable(GTK_LABEL(app->live_label), TRUE);
    gtk_box_pack_start(GTK_BOX(box), app->live_label, FALSE, FALSE, 0);

    app->status_label = gtk_label_new("Ready.");
    gtk_label_set_line_wrap(GTK_LABEL(app->status_label), TRUE);
    gtk_label_set_max_width_chars(GTK_LABEL(app->status_label), 88);
    gtk_label_set_xalign(GTK_LABEL(app->status_label), 0.0f);
    gtk_box_pack_start(GTK_BOX(box), app->status_label, FALSE, FALSE, 0);

    /* The dial sits in its own row with a fixed height, so it cannot squeeze
     * the labels above it or be clipped by them. */
    app->dial = gtk_drawing_area_new();
    gtk_widget_set_size_request(app->dial, 200, 200);
    gtk_widget_set_halign(app->dial, GTK_ALIGN_CENTER);
    g_signal_connect(app->dial, "draw", G_CALLBACK(on_draw_dial), app);
    gtk_box_pack_start(GTK_BOX(box), app->dial, FALSE, FALSE, 6);

    app->grid_label = gtk_label_new("");
    gtk_label_set_xalign(GTK_LABEL(app->grid_label), 0.0f);
    monospace = pango_attr_list_new();
    pango_attr_list_insert(monospace, pango_attr_family_new("monospace"));
    gtk_label_set_attributes(GTK_LABEL(app->grid_label), monospace);
    pango_attr_list_unref(monospace);
    gtk_box_pack_start(GTK_BOX(box), app->grid_label, FALSE, FALSE, 0);
    refresh_grid(app);
}

int main(int argc, char **argv) {
    static oa_app app;
    char error[256];
    const char *names[2] = {"can0", "can1"};

    if (argc >= 3) {
        names[0] = argv[1];
        names[1] = argv[2];
    }
    gtk_init(&argc, &argv);

    for (unsigned arm = 0; arm < OA_ARMS; ++arm) {
        snprintf(app.interface[arm], IFNAMSIZ, "%s", names[arm]);
        app.socket_fd[arm] = open_can(names[arm], error, sizeof(error));
        if (app.socket_fd[arm] < 0) {
            fprintf(stderr, "openarm_calibrate_gui: %s\n", error);
            return 1;
        }
    }

    build_gui(&app);
    app.timer_id = g_timeout_add(OA_POLL_MS, on_tick, &app);
    gtk_widget_show_all(app.window);
    gtk_main();

    for (unsigned arm = 0; arm < OA_ARMS; ++arm) {
        if (app.socket_fd[arm] >= 0) close(app.socket_fd[arm]);
        for (unsigned motor = 0; motor < OA_MOTORS_PER_ARM; ++motor) {
            free(app.track[arm][motor].samples);
        }
    }
    return 0;
}
