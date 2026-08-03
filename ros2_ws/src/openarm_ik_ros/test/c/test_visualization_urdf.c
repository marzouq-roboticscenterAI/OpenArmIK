/* SPDX-License-Identifier: Apache-2.0 */
/* Prove the Stage-A visualization URDF is a narrow canonical-model overlay.
 * C port of the former test_visualization_urdf.py, using libxml2 for XML,
 * json-c for the viewer manifest, and libcrypto for SHA-256. */
#include "test_support.h"

#include <json-c/json.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <openssl/evp.h>

#include <ctype.h>
#include <math.h>
#include <sys/stat.h>

static const char kCanonicalSha256[] =
    "dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55";

struct finger_joint_contract {
    const char *name;
    const char *parent;
    const char *child;
    const char *xyz;
    const char *axis;
    const char *mimic; /* NULL when the joint has no mimic element */
};

static const struct finger_joint_contract kFingerJoints[] = {
    {"openarm_left_finger_joint1", "openarm_left_link7", "openarm_left_right_finger",
     "0.0 -0.005 0.1025", "0 -1 0", NULL},
    {"openarm_left_finger_joint2", "openarm_left_link7", "openarm_left_left_finger",
     "0.0 0.005 0.1025", "0 1 0", "openarm_left_finger_joint1"},
    {"openarm_right_finger_joint1", "openarm_right_link7", "openarm_right_right_finger",
     "0.0 -0.005 0.1025", "0 -1 0", NULL},
    {"openarm_right_finger_joint2", "openarm_right_link7", "openarm_right_left_finger",
     "0.0 0.005 0.1025", "0 1 0", "openarm_right_finger_joint1"},
};
static const size_t kFingerJointCount =
    sizeof(kFingerJoints) / sizeof(kFingerJoints[0]);

static const char *const kFingerLinks[] = {
    "openarm_left_left_finger", "openarm_left_right_finger",
    "openarm_right_left_finger", "openarm_right_right_finger"};
static const size_t kFingerLinkCount = sizeof(kFingerLinks) / sizeof(kFingerLinks[0]);

/* ---------------------------------------------------------------- hashing */

static void sha256_hex(const unsigned char *data, size_t length, char out[65]) {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int digest_length = 0u;
    EVP_MD_CTX *const context = EVP_MD_CTX_new();
    if (context == NULL || EVP_DigestInit_ex(context, EVP_sha256(), NULL) != 1 ||
        EVP_DigestUpdate(context, data, length) != 1 ||
        EVP_DigestFinal_ex(context, digest, &digest_length) != 1) {
        oa_fail("SHA-256 computation failed");
    }
    EVP_MD_CTX_free(context);
    for (unsigned int index = 0u; index < digest_length; ++index) {
        snprintf(out + index * 2u, 3u, "%02x", digest[index]);
    }
}

/* ------------------------------------------------------------------- XML */

static const struct finger_joint_contract *finger_joint(const char *name) {
    for (size_t index = 0u; index < kFingerJointCount; ++index) {
        if (strcmp(kFingerJoints[index].name, name) == 0) {
            return &kFingerJoints[index];
        }
    }
    return NULL;
}

static int is_finger_link(const char *name) {
    for (size_t index = 0u; index < kFingerLinkCount; ++index) {
        if (strcmp(kFingerLinks[index], name) == 0) {
            return 1;
        }
    }
    return 0;
}

static xmlNodePtr first_child_element(xmlNodePtr parent, const char *name) {
    for (xmlNodePtr child = parent->children; child != NULL; child = child->next) {
        if (child->type == XML_ELEMENT_NODE &&
            xmlStrcmp(child->name, (const xmlChar *)name) == 0) {
            return child;
        }
    }
    return NULL;
}

static const char *attribute(xmlNodePtr node, const char *name, const char *fallback) {
    xmlChar *const value = xmlGetProp(node, (const xmlChar *)name);
    if (value == NULL) {
        return fallback;
    }
    /* Interned for the lifetime of the document by libxml2's dictionary in the
     * common case; copy defensively into a rotating buffer instead. */
    static char storage[8][512];
    static size_t slot = 0u;
    char *const target = storage[slot];
    slot = (slot + 1u) % 8u;
    snprintf(target, sizeof(storage[0]), "%s", (const char *)value);
    xmlFree(value);
    return target;
}

/* Byte-identical serialization. Both sides use this same serializer, so the
 * comparison is exact even though the bytes differ from ElementTree's. */
static char *serialize(xmlNodePtr node) {
    xmlBufferPtr buffer = xmlBufferCreate();
    if (buffer == NULL) {
        oa_fail("cannot allocate serialization buffer");
    }
    if (xmlNodeDump(buffer, node->doc, node, 0, 0) < 0) {
        oa_fail("cannot serialize node <%s>", (const char *)node->name);
    }
    char *const text = strdup((const char *)xmlBufferContent(buffer));
    xmlBufferFree(buffer);
    if (text == NULL) {
        oa_fail("out of memory serializing node");
    }
    return text;
}

static size_t attribute_count(xmlNodePtr node) {
    size_t count = 0u;
    for (xmlAttrPtr property = node->properties; property != NULL;
         property = property->next) {
        ++count;
    }
    return count;
}

static char *trimmed_text(xmlNodePtr node) {
    oa_buffer buffer = {NULL, 0u, 0u};
    for (xmlNodePtr child = node->children; child != NULL; child = child->next) {
        if (child->type == XML_TEXT_NODE && child->content != NULL) {
            oa_buffer_append(&buffer, (const char *)child->content,
                             strlen((const char *)child->content));
        }
    }
    if (buffer.data == NULL) {
        return strdup("");
    }
    size_t begin = 0u;
    while (begin < buffer.size && isspace((unsigned char)buffer.data[begin])) {
        ++begin;
    }
    size_t end = buffer.size;
    while (end > begin && isspace((unsigned char)buffer.data[end - 1u])) {
        --end;
    }
    char *const text = strndup(buffer.data + begin, end - begin);
    oa_buffer_free(&buffer);
    return text;
}

/* Attribute-order-insensitive structural equality, matching the Python
 * `semantic()` tuple comparison. */
static int semantic_equal(xmlNodePtr left, xmlNodePtr right) {
    if (xmlStrcmp(left->name, right->name) != 0) {
        return 0;
    }
    if (attribute_count(left) != attribute_count(right)) {
        return 0;
    }
    for (xmlAttrPtr property = left->properties; property != NULL;
         property = property->next) {
        xmlChar *const mine = xmlGetProp(left, property->name);
        xmlChar *const theirs = xmlGetProp(right, property->name);
        const int same = theirs != NULL && xmlStrcmp(mine, theirs) == 0;
        xmlFree(mine);
        if (theirs != NULL) {
            xmlFree(theirs);
        }
        if (!same) {
            return 0;
        }
    }
    char *const left_text = trimmed_text(left);
    char *const right_text = trimmed_text(right);
    const int text_same = strcmp(left_text, right_text) == 0;
    free(left_text);
    free(right_text);
    if (!text_same) {
        return 0;
    }
    xmlNodePtr left_child = left->children;
    xmlNodePtr right_child = right->children;
    for (;;) {
        while (left_child != NULL && left_child->type != XML_ELEMENT_NODE) {
            left_child = left_child->next;
        }
        while (right_child != NULL && right_child->type != XML_ELEMENT_NODE) {
            right_child = right_child->next;
        }
        if (left_child == NULL || right_child == NULL) {
            return left_child == NULL && right_child == NULL;
        }
        if (!semantic_equal(left_child, right_child)) {
            return 0;
        }
        left_child = left_child->next;
        right_child = right_child->next;
    }
}

/* ------------------------------------------------------------ kinematics */

typedef struct {
    double m[16];
} matrix4;

static matrix4 identity(void) {
    matrix4 result;
    memset(result.m, 0, sizeof(result.m));
    result.m[0] = 1.0;
    result.m[5] = 1.0;
    result.m[10] = 1.0;
    result.m[15] = 1.0;
    return result;
}

static matrix4 multiply(const matrix4 left, const matrix4 right) {
    matrix4 result;
    for (size_t row = 0u; row < 4u; ++row) {
        for (size_t column = 0u; column < 4u; ++column) {
            double sum = 0.0;
            for (size_t index = 0u; index < 4u; ++index) {
                sum += left.m[row * 4u + index] * right.m[index * 4u + column];
            }
            result.m[row * 4u + column] = sum;
        }
    }
    return result;
}

static void parse_vector(const char *text, double out[3]) {
    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = 0.0;
    if (text == NULL) {
        return;
    }
    if (sscanf(text, "%lf %lf %lf", &out[0], &out[1], &out[2]) != 3) {
        oa_fail("cannot parse vector: %s", text);
    }
}

static matrix4 translation(const double xyz[3]) {
    matrix4 result = identity();
    result.m[3] = xyz[0];
    result.m[7] = xyz[1];
    result.m[11] = xyz[2];
    return result;
}

static matrix4 rpy_rotation(const double rpy[3]) {
    const double cr = cos(rpy[0]);
    const double sr = sin(rpy[0]);
    const double cp = cos(rpy[1]);
    const double sp = sin(rpy[1]);
    const double cy = cos(rpy[2]);
    const double sy = sin(rpy[2]);
    matrix4 result = identity();
    result.m[0] = cy * cp;
    result.m[1] = cy * sp * sr - sy * cr;
    result.m[2] = cy * sp * cr + sy * sr;
    result.m[4] = sy * cp;
    result.m[5] = sy * sp * sr + cy * cr;
    result.m[6] = sy * sp * cr - cy * sr;
    result.m[8] = -sp;
    result.m[9] = cp * sr;
    result.m[10] = cp * cr;
    return result;
}

static matrix4 axis_rotation(const double axis[3], const double angle) {
    const double x = axis[0];
    const double y = axis[1];
    const double z = axis[2];
    const double cosine = cos(angle);
    const double sine = sin(angle);
    const double one_minus = 1.0 - cosine;
    matrix4 result = identity();
    result.m[0] = cosine + x * x * one_minus;
    result.m[1] = x * y * one_minus - z * sine;
    result.m[2] = x * z * one_minus + y * sine;
    result.m[4] = y * x * one_minus + z * sine;
    result.m[5] = cosine + y * y * one_minus;
    result.m[6] = y * z * one_minus - x * sine;
    result.m[8] = z * x * one_minus - y * sine;
    result.m[9] = z * y * one_minus + x * sine;
    result.m[10] = cosine + z * z * one_minus;
    return result;
}

/* A posture is a flat name/value list; the joint sets here are tiny. */
typedef struct {
    char names[32][64];
    double values[32];
    size_t count;
} posture;

static double posture_value(const posture *state, const char *name) {
    for (size_t index = 0u; index < state->count; ++index) {
        if (strcmp(state->names[index], name) == 0) {
            return state->values[index];
        }
    }
    return 0.0;
}

typedef struct {
    char names[64][64];
    matrix4 transforms[64];
    size_t count;
} frame_table;

static void frame_put(frame_table *table, const char *name, const matrix4 value) {
    if (table->count >= 64u) {
        oa_fail("frame table overflow");
    }
    snprintf(table->names[table->count], 64u, "%s", name);
    table->transforms[table->count] = value;
    ++table->count;
}

static const matrix4 *frame_get(const frame_table *table, const char *name) {
    for (size_t index = 0u; index < table->count; ++index) {
        if (strcmp(table->names[index], name) == 0) {
            return &table->transforms[index];
        }
    }
    return NULL;
}

static matrix4 joint_transform(xmlNodePtr joint, const posture *state) {
    xmlNodePtr const origin = first_child_element(joint, "origin");
    double xyz[3] = {0.0, 0.0, 0.0};
    double rpy[3] = {0.0, 0.0, 0.0};
    if (origin != NULL) {
        parse_vector(attribute(origin, "xyz", "0 0 0"), xyz);
        parse_vector(attribute(origin, "rpy", "0 0 0"), rpy);
    }
    matrix4 transform = multiply(translation(xyz), rpy_rotation(rpy));
    const char *const type = attribute(joint, "type", "");
    const char *const name = attribute(joint, "name", "");
    const double position = posture_value(state, name);
    if (strcmp(type, "revolute") == 0 || strcmp(type, "continuous") == 0) {
        double axis[3];
        parse_vector(attribute(first_child_element(joint, "axis"), "xyz", "1 0 0"), axis);
        transform = multiply(transform, axis_rotation(axis, position));
    } else if (strcmp(type, "prismatic") == 0) {
        double axis[3];
        parse_vector(attribute(first_child_element(joint, "axis"), "xyz", "1 0 0"), axis);
        const double scaled[3] = {axis[0] * position, axis[1] * position,
                                  axis[2] * position};
        transform = multiply(transform, translation(scaled));
    } else if (strcmp(type, "fixed") != 0) {
        oa_fail("unsupported joint type %s", type);
    }
    return transform;
}

static void frame_transforms(xmlNodePtr robot, const posture *state,
                             frame_table *out) {
    out->count = 0u;
    frame_put(out, "world", identity());
    /* Breadth-first over the joint tree; the model has 25 joints. */
    for (size_t guard = 0u; guard < 64u; ++guard) {
        int progressed = 0;
        for (xmlNodePtr joint = robot->children; joint != NULL; joint = joint->next) {
            if (joint->type != XML_ELEMENT_NODE ||
                xmlStrcmp(joint->name, (const xmlChar *)"joint") != 0) {
                continue;
            }
            const char *const parent_name =
                attribute(first_child_element(joint, "parent"), "link", "");
            char parent_copy[64];
            snprintf(parent_copy, sizeof(parent_copy), "%s", parent_name);
            const char *const child_name =
                attribute(first_child_element(joint, "child"), "link", "");
            if (frame_get(out, child_name) != NULL) {
                continue;
            }
            const matrix4 *const parent_transform = frame_get(out, parent_copy);
            if (parent_transform == NULL) {
                continue;
            }
            const matrix4 combined =
                multiply(*parent_transform, joint_transform(joint, state));
            frame_put(out, child_name, combined);
            progressed = 1;
        }
        if (!progressed) {
            break;
        }
    }
}

/* Deterministic PRNG. The Python original used random.Random(0xA11CE); its
 * exact Mersenne Twister stream is not reproducible here and is not what the
 * test depends on. What matters is a fixed, repeatable set of in-limit
 * postures, which this provides. */
static uint64_t rng_state = 0xA11CEu;

static double next_uniform(const double lower, const double upper) {
    rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
    const double unit =
        (double)((rng_state >> 11u) & ((1ULL << 53u) - 1u)) / (double)(1ULL << 53u);
    return lower + unit * (upper - lower);
}

static void compare_kinematics(xmlNodePtr canonical, xmlNodePtr visualization) {
    char names[32][64];
    double lower[32];
    double upper[32];
    size_t count = 0u;
    for (xmlNodePtr joint = canonical->children; joint != NULL; joint = joint->next) {
        if (joint->type != XML_ELEMENT_NODE ||
            xmlStrcmp(joint->name, (const xmlChar *)"joint") != 0) {
            continue;
        }
        const char *const name = attribute(joint, "name", "");
        char name_copy[64];
        snprintf(name_copy, sizeof(name_copy), "%s", name);
        if (finger_joint(name_copy) != NULL) {
            continue;
        }
        if (strcmp(attribute(joint, "type", ""), "revolute") != 0) {
            continue;
        }
        xmlNodePtr const limit = first_child_element(joint, "limit");
        if (limit == NULL) {
            oa_fail("revolute joint %s has no limit", name_copy);
        }
        if (count >= 32u) {
            oa_fail("too many revolute joints");
        }
        snprintf(names[count], 64u, "%s", name_copy);
        lower[count] = atof(attribute(limit, "lower", "0"));
        upper[count] = atof(attribute(limit, "upper", "0"));
        ++count;
    }
    if (count == 0u) {
        oa_fail("no revolute arm joints found");
    }

    posture postures[15];
    memset(postures, 0, sizeof(postures));
    for (size_t which = 0u; which < 15u; ++which) {
        postures[which].count = count;
        for (size_t index = 0u; index < count; ++index) {
            snprintf(postures[which].names[index], 64u, "%s", names[index]);
            if (which == 0u) {
                postures[which].values[index] = 0.0;
            } else if (which == 1u) {
                postures[which].values[index] = lower[index];
            } else if (which == 2u) {
                postures[which].values[index] = upper[index];
            } else {
                postures[which].values[index] = next_uniform(lower[index], upper[index]);
            }
        }
    }

    for (size_t which = 0u; which < 15u; ++which) {
        frame_table canonical_frames;
        frame_table visualization_frames;
        frame_transforms(canonical, &postures[which], &canonical_frames);
        frame_transforms(visualization, &postures[which], &visualization_frames);
        if (canonical_frames.count != visualization_frames.count) {
            oa_fail("frame set differs between canonical and visualization models");
        }
        for (size_t index = 0u; index < canonical_frames.count; ++index) {
            const matrix4 *const derived =
                frame_get(&visualization_frames, canonical_frames.names[index]);
            if (derived == NULL) {
                oa_fail("visualization model lacks frame %s",
                        canonical_frames.names[index]);
            }
            for (size_t element = 0u; element < 16u; ++element) {
                const double error =
                    fabs(canonical_frames.transforms[index].m[element] -
                         derived->m[element]);
                if (!(error < 1.0e-12)) {
                    oa_fail("frame %s differs by %g at element %zu",
                            canonical_frames.names[index], error, element);
                }
            }
        }
    }
}

/* --------------------------------------------------------------- helpers */

static xmlNodePtr find_named(xmlNodePtr robot, const char *element_name,
                             const char *name) {
    for (xmlNodePtr node = robot->children; node != NULL; node = node->next) {
        if (node->type != XML_ELEMENT_NODE ||
            xmlStrcmp(node->name, (const xmlChar *)element_name) != 0) {
            continue;
        }
        xmlChar *const value = xmlGetProp(node, (const xmlChar *)"name");
        const int same = value != NULL && strcmp((const char *)value, name) == 0;
        if (value != NULL) {
            xmlFree(value);
        }
        if (same) {
            return node;
        }
    }
    return NULL;
}

static size_t count_elements(xmlNodePtr robot, const char *element_name) {
    size_t count = 0u;
    for (xmlNodePtr node = robot->children; node != NULL; node = node->next) {
        if (node->type == XML_ELEMENT_NODE &&
            xmlStrcmp(node->name, (const xmlChar *)element_name) == 0) {
            ++count;
        }
    }
    return count;
}

static size_t count_children(xmlNodePtr parent, const char *child_name) {
    size_t count = 0u;
    for (xmlNodePtr child = parent->children; child != NULL; child = child->next) {
        if (child->type == XML_ELEMENT_NODE &&
            xmlStrcmp(child->name, (const xmlChar *)child_name) == 0) {
            ++count;
        }
    }
    return count;
}

static void require_attributes(xmlNodePtr node, const char *label,
                               const char *const pairs[][2], size_t pair_count) {
    if (node == NULL) {
        oa_fail("%s element is absent", label);
    }
    if (attribute_count(node) != pair_count) {
        oa_fail("%s has %zu attributes, expected %zu", label, attribute_count(node),
                pair_count);
    }
    for (size_t index = 0u; index < pair_count; ++index) {
        xmlChar *const value = xmlGetProp(node, (const xmlChar *)pairs[index][0]);
        if (value == NULL || strcmp((const char *)value, pairs[index][1]) != 0) {
            oa_fail("%s attribute %s is %s, expected %s", label, pairs[index][0],
                    value == NULL ? "(absent)" : (const char *)value, pairs[index][1]);
        }
        xmlFree(value);
    }
}

static json_object *require_field(json_object *parent, const char *key) {
    json_object *value = NULL;
    if (!json_object_object_get_ex(parent, key, &value)) {
        oa_fail("viewer manifest lacks '%s'", key);
    }
    return value;
}

static void require_string_field(json_object *parent, const char *key,
                                 const char *expected) {
    json_object *const value = require_field(parent, key);
    const char *const text = json_object_get_string(value);
    if (text == NULL || strcmp(text, expected) != 0) {
        oa_fail("viewer manifest %s is '%s', expected '%s'", key,
                text == NULL ? "(null)" : text, expected);
    }
}

int main(int argc, char **argv) {
    const char *const canonical_path = oa_required_argument(argc, argv, "--canonical");
    const char *const visualization_path =
        oa_required_argument(argc, argv, "--visualization");
    const char *const manifest_path = oa_required_argument(argc, argv, "--manifest");
    const char *const license_path = oa_required_argument(argc, argv, "--license");
    const char *const description_root =
        oa_required_argument(argc, argv, "--description-root");
    const char *const generator_path = oa_required_argument(argc, argv, "--generator");
    const char *const cmake_path = oa_required_argument(argc, argv, "--cmake");

    size_t canonical_size = 0u;
    size_t visualization_size = 0u;
    char *const canonical_bytes = oa_read_file(canonical_path, &canonical_size);
    char *const visualization_bytes =
        oa_read_file(visualization_path, &visualization_size);

    char digest[65];
    sha256_hex((const unsigned char *)canonical_bytes, canonical_size, digest);
    if (strcmp(digest, kCanonicalSha256) != 0) {
        oa_fail("canonical URDF digest is %s, expected %s", digest, kCanonicalSha256);
    }
    char notice[256];
    snprintf(notice, sizeof(notice), "Derived from canonical SHA-256\n       %s.",
             kCanonicalSha256);
    if (strstr(visualization_bytes, notice) == NULL) {
        oa_fail("visualization URDF lacks the canonical derivation notice");
    }

    /* Regeneration must be byte-reproducible from the canonical input. */
    char directory[] = "/tmp/openarm-visualization-XXXXXX";
    if (mkdtemp(directory) == NULL) {
        oa_fail("cannot create temporary directory: %s", strerror(errno));
    }
    char regenerated[4096];
    snprintf(regenerated, sizeof(regenerated), "%s/visualization.urdf", directory);
    {
        char input_define[4200];
        char output_define[4200];
        snprintf(input_define, sizeof(input_define), "-DINPUT_URDF=%s", canonical_path);
        snprintf(output_define, sizeof(output_define), "-DOUTPUT_URDF=%s", regenerated);
        char *const command[] = {(char *)cmake_path, input_define, output_define,
                                 (char *)"-P",       (char *)generator_path, NULL};
        oa_buffer output = {NULL, 0u, 0u};
        const int status = oa_run_capture(command, 120.0, &output, NULL);
        if (oa_exit_code(status) != 0) {
            oa_fail("visualization regeneration failed:\n%s",
                    output.data == NULL ? "" : output.data);
        }
        oa_buffer_free(&output);
    }
    size_t regenerated_size = 0u;
    char *const regenerated_bytes = oa_read_file(regenerated, &regenerated_size);
    unlink(regenerated);
    rmdir(directory);
    if (regenerated_size != visualization_size ||
        memcmp(regenerated_bytes, visualization_bytes, visualization_size) != 0) {
        oa_fail("regenerated visualization URDF is not byte-identical");
    }
    free(regenerated_bytes);

    LIBXML_TEST_VERSION
    xmlDocPtr canonical_document =
        xmlReadMemory(canonical_bytes, (int)canonical_size, "canonical.urdf", NULL,
                      XML_PARSE_NONET);
    xmlDocPtr visualization_document =
        xmlReadMemory(visualization_bytes, (int)visualization_size, "visualization.urdf",
                      NULL, XML_PARSE_NONET);
    if (canonical_document == NULL || visualization_document == NULL) {
        oa_fail("cannot parse one of the URDF documents");
    }
    xmlNodePtr const canonical = xmlDocGetRootElement(canonical_document);
    xmlNodePtr const visualization = xmlDocGetRootElement(visualization_document);

    if (count_elements(canonical, "link") != 26u ||
        count_elements(visualization, "link") != 26u) {
        oa_fail("expected 26 links in both models");
    }
    if (count_elements(canonical, "joint") != 25u ||
        count_elements(visualization, "joint") != 25u) {
        oa_fail("expected 25 joints in both models");
    }

    for (xmlNodePtr link = canonical->children; link != NULL; link = link->next) {
        if (link->type != XML_ELEMENT_NODE ||
            xmlStrcmp(link->name, (const xmlChar *)"link") != 0) {
            continue;
        }
        char name[64];
        snprintf(name, sizeof(name), "%s", attribute(link, "name", ""));
        xmlNodePtr const derived = find_named(visualization, "link", name);
        if (derived == NULL) {
            oa_fail("visualization model lacks link %s", name);
        }
        if (!is_finger_link(name)) {
            char *const mine = serialize(link);
            char *const theirs = serialize(derived);
            const int same = strcmp(mine, theirs) == 0;
            free(mine);
            free(theirs);
            if (!same) {
                oa_fail("link %s is not carried through verbatim", name);
            }
            continue;
        }
        /* Finger links drop their inertial block and keep everything else. */
        if (count_children(link, "inertial") != 1u) {
            oa_fail("canonical finger link %s has no single inertial block", name);
        }
        if (count_children(derived, "inertial") != 0u) {
            oa_fail("visualization finger link %s retains an inertial block", name);
        }
        xmlNodePtr mine = link->children;
        xmlNodePtr theirs = derived->children;
        for (;;) {
            while (mine != NULL &&
                   (mine->type != XML_ELEMENT_NODE ||
                    xmlStrcmp(mine->name, (const xmlChar *)"inertial") == 0)) {
                mine = mine->next;
            }
            while (theirs != NULL && theirs->type != XML_ELEMENT_NODE) {
                theirs = theirs->next;
            }
            if (mine == NULL || theirs == NULL) {
                if (mine != NULL || theirs != NULL) {
                    oa_fail("finger link %s child sets differ", name);
                }
                break;
            }
            if (!semantic_equal(mine, theirs)) {
                oa_fail("finger link %s child <%s> differs", name,
                        (const char *)mine->name);
            }
            mine = mine->next;
            theirs = theirs->next;
        }
    }

    for (xmlNodePtr joint = canonical->children; joint != NULL; joint = joint->next) {
        if (joint->type != XML_ELEMENT_NODE ||
            xmlStrcmp(joint->name, (const xmlChar *)"joint") != 0) {
            continue;
        }
        char name[64];
        snprintf(name, sizeof(name), "%s", attribute(joint, "name", ""));
        xmlNodePtr const derived = find_named(visualization, "joint", name);
        if (derived == NULL) {
            oa_fail("visualization model lacks joint %s", name);
        }
        const struct finger_joint_contract *const contract = finger_joint(name);
        if (contract == NULL) {
            char *const mine = serialize(joint);
            char *const theirs = serialize(derived);
            const int same = strcmp(mine, theirs) == 0;
            free(mine);
            free(theirs);
            if (!same) {
                oa_fail("joint %s is not carried through verbatim", name);
            }
            continue;
        }
        /* The canonical finger joint is a mimicking prismatic pair. */
        if (strcmp(attribute(joint, "type", ""), "prismatic") != 0) {
            oa_fail("canonical finger joint %s is not prismatic", name);
        }
        {
            const char *const parent_pairs[][2] = {{"link", contract->parent}};
            const char *const child_pairs[][2] = {{"link", contract->child}};
            const char *const origin_pairs[][2] = {{"rpy", "0 0 0"},
                                                   {"xyz", contract->xyz}};
            const char *const axis_pairs[][2] = {{"xyz", contract->axis}};
            const char *const limit_pairs[][2] = {{"effort", "333"},
                                                  {"lower", "0.0"},
                                                  {"upper", "0.044"},
                                                  {"velocity", "10.0"}};
            require_attributes(first_child_element(joint, "parent"), "canonical parent",
                               parent_pairs, 1u);
            require_attributes(first_child_element(joint, "child"), "canonical child",
                               child_pairs, 1u);
            require_attributes(first_child_element(joint, "origin"), "canonical origin",
                               origin_pairs, 2u);
            require_attributes(first_child_element(joint, "axis"), "canonical axis",
                               axis_pairs, 1u);
            require_attributes(first_child_element(joint, "limit"), "canonical limit",
                               limit_pairs, 4u);
        }
        xmlNodePtr const mimic = first_child_element(joint, "mimic");
        if (contract->mimic == NULL) {
            if (mimic != NULL) {
                oa_fail("canonical joint %s has an unexpected mimic", name);
            }
        } else {
            const char *const mimic_pairs[][2] = {{"joint", contract->mimic}};
            require_attributes(mimic, "canonical mimic", mimic_pairs, 1u);
        }

        /* The visualization overlay freezes it to a fixed joint. */
        if (strcmp(attribute(derived, "type", ""), "fixed") != 0) {
            oa_fail("visualization finger joint %s is not fixed", name);
        }
        {
            const char *const parent_pairs[][2] = {{"link", contract->parent}};
            const char *const child_pairs[][2] = {{"link", contract->child}};
            const char *const origin_pairs[][2] = {{"rpy", "0 0 0"},
                                                   {"xyz", contract->xyz}};
            require_attributes(first_child_element(derived, "parent"), "derived parent",
                               parent_pairs, 1u);
            require_attributes(first_child_element(derived, "child"), "derived child",
                               child_pairs, 1u);
            require_attributes(first_child_element(derived, "origin"), "derived origin",
                               origin_pairs, 2u);
        }
        if (first_child_element(derived, "axis") != NULL ||
            first_child_element(derived, "limit") != NULL ||
            first_child_element(derived, "mimic") != NULL) {
            oa_fail("visualization finger joint %s retains actuation elements", name);
        }
        size_t element_index = 0u;
        static const char *const expected_order[] = {"parent", "child", "origin"};
        for (xmlNodePtr child = derived->children; child != NULL; child = child->next) {
            if (child->type != XML_ELEMENT_NODE) {
                continue;
            }
            if (element_index >= 3u ||
                xmlStrcmp(child->name, (const xmlChar *)expected_order[element_index]) !=
                    0) {
                oa_fail("visualization finger joint %s has unexpected child order", name);
            }
            ++element_index;
        }
        if (element_index != 3u) {
            oa_fail("visualization finger joint %s does not have exactly parent, child, "
                    "origin",
                    name);
        }
    }

    compare_kinematics(canonical, visualization);

    /* ---- viewer asset manifest ---- */
    size_t manifest_size = 0u;
    char *const manifest_text = oa_read_file(manifest_path, &manifest_size);
    json_object *const manifest = json_tokener_parse(manifest_text);
    if (manifest == NULL) {
        oa_fail("cannot parse viewer manifest JSON");
    }
    if (json_object_get_int(require_field(manifest, "schema")) != 1) {
        oa_fail("viewer manifest schema is not 1");
    }
    json_object *const upstream = require_field(manifest, "upstream");
    require_string_field(upstream, "repository",
                         "https://github.com/enactic/openarm_description");
    require_string_field(upstream, "commit",
                         "6c7b720f1ba48e8bafa3a3dc752c45f397b42221");
    require_string_field(upstream, "license", "Apache-2.0");
    require_string_field(upstream, "license_file",
                         "viewer/openarm_description-LICENSE.txt");
    const char *const license_sha =
        json_object_get_string(require_field(upstream, "license_sha256"));
    if (license_sha == NULL ||
        strcmp(license_sha,
               "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4") != 0) {
        oa_fail("unexpected upstream license digest");
    }

    size_t license_size = 0u;
    char *const license_bytes = oa_read_file(license_path, &license_size);
    if (license_size != 11357u) {
        oa_fail("license is %zu bytes, expected 11357", license_size);
    }
    sha256_hex((const unsigned char *)license_bytes, license_size, digest);
    if (strcmp(digest, license_sha) != 0) {
        oa_fail("license digest mismatch");
    }
    free(license_bytes);

    json_object *const meshes = require_field(manifest, "meshes");
    const size_t mesh_count = (size_t)json_object_array_length(meshes);
    if (mesh_count != 11u) {
        oa_fail("expected 11 meshes, found %zu", mesh_count);
    }
    const int64_t total_bytes =
        json_object_get_int64(require_field(manifest, "total_bytes"));
    const int64_t total_triangles =
        json_object_get_int64(require_field(manifest, "total_triangles"));
    if (total_bytes != 2498724 || total_triangles != 49956) {
        oa_fail("viewer manifest totals changed: %lld bytes, %lld triangles",
                (long long)total_bytes, (long long)total_triangles);
    }
    int64_t summed_bytes = 0;
    int64_t summed_triangles = 0;
    for (size_t index = 0u; index < mesh_count; ++index) {
        json_object *const mesh = json_object_array_get_idx(meshes, index);
        summed_bytes += json_object_get_int64(require_field(mesh, "bytes"));
        summed_triangles += json_object_get_int64(require_field(mesh, "triangles"));
    }
    if (summed_bytes != total_bytes || summed_triangles != total_triangles) {
        oa_fail("per-mesh totals do not sum to the declared totals");
    }

    /* Routes must be unique. */
    for (size_t index = 0u; index < mesh_count; ++index) {
        const char *const route = json_object_get_string(
            require_field(json_object_array_get_idx(meshes, index), "route"));
        for (size_t other = index + 1u; other < mesh_count; ++other) {
            const char *const compare = json_object_get_string(
                require_field(json_object_array_get_idx(meshes, other), "route"));
            if (route != NULL && compare != NULL && strcmp(route, compare) == 0) {
                oa_fail("duplicate viewer route: %s", route);
            }
        }
    }

    /* Manifest sources must exactly match the visualization collision meshes. */
    size_t collision_count = 0u;
    char collision_sources[32][512];
    for (xmlNodePtr link = visualization->children; link != NULL; link = link->next) {
        if (link->type != XML_ELEMENT_NODE ||
            xmlStrcmp(link->name, (const xmlChar *)"link") != 0) {
            continue;
        }
        for (xmlNodePtr collision = link->children; collision != NULL;
             collision = collision->next) {
            if (collision->type != XML_ELEMENT_NODE ||
                xmlStrcmp(collision->name, (const xmlChar *)"collision") != 0) {
                continue;
            }
            xmlNodePtr const geometry = first_child_element(collision, "geometry");
            if (geometry == NULL) {
                continue;
            }
            for (xmlNodePtr mesh = geometry->children; mesh != NULL;
                 mesh = mesh->next) {
                if (mesh->type != XML_ELEMENT_NODE ||
                    xmlStrcmp(mesh->name, (const xmlChar *)"mesh") != 0) {
                    continue;
                }
                char filename[512];
                snprintf(filename, sizeof(filename), "%s",
                         attribute(mesh, "filename", ""));
                int seen = 0;
                for (size_t index = 0u; index < collision_count; ++index) {
                    if (strcmp(collision_sources[index], filename) == 0) {
                        seen = 1;
                    }
                }
                if (!seen) {
                    if (collision_count >= 32u) {
                        oa_fail("too many distinct collision meshes");
                    }
                    snprintf(collision_sources[collision_count], 512u, "%s", filename);
                    ++collision_count;
                }
            }
        }
    }
    if (collision_count != mesh_count) {
        oa_fail("visualization has %zu distinct collision meshes, manifest lists %zu",
                collision_count, mesh_count);
    }

    static const char prefix[] = "package://openarm_description/";
    const size_t prefix_length = sizeof(prefix) - 1u;
    for (size_t index = 0u; index < mesh_count; ++index) {
        json_object *const mesh = json_object_array_get_idx(meshes, index);
        const char *const route = json_object_get_string(require_field(mesh, "route"));
        const char *const source = json_object_get_string(require_field(mesh, "source"));
        const char *const expected_digest =
            json_object_get_string(require_field(mesh, "sha256"));
        const int64_t bytes = json_object_get_int64(require_field(mesh, "bytes"));
        const int64_t triangles = json_object_get_int64(require_field(mesh, "triangles"));
        if (route == NULL || strncmp(route, "/viewer/mesh/", 13u) != 0) {
            oa_fail("unexpected viewer route: %s", route == NULL ? "(null)" : route);
        }
        if (source == NULL || strncmp(source, prefix, prefix_length) != 0) {
            oa_fail("unexpected mesh source: %s", source == NULL ? "(null)" : source);
        }
        int matched = 0;
        for (size_t which = 0u; which < collision_count; ++which) {
            if (strcmp(collision_sources[which], source) == 0) {
                matched = 1;
            }
        }
        if (!matched) {
            oa_fail("manifest mesh %s is not referenced by the visualization URDF",
                    source);
        }
        char asset[4096];
        snprintf(asset, sizeof(asset), "%s/%s", description_root, source + prefix_length);
        size_t payload_size = 0u;
        char *const payload = oa_read_file(asset, &payload_size);
        if ((int64_t)payload_size != bytes) {
            oa_fail("%s is %zu bytes, manifest says %lld", asset, payload_size,
                    (long long)bytes);
        }
        sha256_hex((const unsigned char *)payload, payload_size, digest);
        if (expected_digest == NULL || strcmp(digest, expected_digest) != 0) {
            oa_fail("%s digest mismatch", asset);
        }
        /* Binary STL: 84-byte header plus 50 bytes per triangle. */
        if ((int64_t)payload_size != 84 + 50 * triangles) {
            oa_fail("%s is not a binary STL of %lld triangles", asset,
                    (long long)triangles);
        }
        free(payload);
    }

    json_object_put(manifest);
    free(manifest_text);
    xmlFreeDoc(canonical_document);
    xmlFreeDoc(visualization_document);
    xmlCleanupParser();
    free(canonical_bytes);
    free(visualization_bytes);
    return 0;
}
