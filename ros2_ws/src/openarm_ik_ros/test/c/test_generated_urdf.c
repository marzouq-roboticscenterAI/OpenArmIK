/* SPDX-License-Identifier: Apache-2.0 */
/* Validate the frozen bimanual URDF contract and all mesh package paths.
 * C port of the former test_generated_urdf.py, using libxml2. */
#include "test_support.h"

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>

#include <sys/stat.h>

static xmlNodePtr find_named_joint(xmlXPathContextPtr context, const char *name) {
    char expression[256];
    snprintf(expression, sizeof(expression), "/robot/joint[@name='%s']", name);
    xmlXPathObjectPtr result =
        xmlXPathEvalExpression((const xmlChar *)expression, context);
    if (result == NULL) {
        return NULL;
    }
    xmlNodePtr node = NULL;
    if (result->nodesetval != NULL && result->nodesetval->nodeNr > 0) {
        node = result->nodesetval->nodeTab[0];
    }
    xmlXPathFreeObject(result);
    return node;
}

/* Returns the "link" attribute of the named child element, or NULL. */
static char *child_link(xmlNodePtr parent, const char *child_name) {
    for (xmlNodePtr child = parent->children; child != NULL; child = child->next) {
        if (child->type == XML_ELEMENT_NODE &&
            xmlStrcmp(child->name, (const xmlChar *)child_name) == 0) {
            return (char *)xmlGetProp(child, (const xmlChar *)"link");
        }
    }
    return NULL;
}

static int joint_exists(xmlXPathContextPtr context, const char *name) {
    return find_named_joint(context, name) != NULL;
}

static int is_regular_file(const char *path) {
    struct stat info;
    return stat(path, &info) == 0 && S_ISREG(info.st_mode);
}

int main(int argc, char **argv) {
    const char *const urdf_path = oa_required_argument(argc, argv, "--urdf");
    const char *const description_root =
        oa_required_argument(argc, argv, "--description-root");

    LIBXML_TEST_VERSION
    xmlDocPtr document = xmlReadFile(urdf_path, NULL, XML_PARSE_NONET);
    if (document == NULL) {
        oa_fail("cannot parse URDF: %s", urdf_path);
    }
    xmlXPathContextPtr context = xmlXPathNewContext(document);
    if (context == NULL) {
        oa_fail("cannot create XPath context");
    }

    xmlNodePtr world_joint = find_named_joint(context, "openarm_body_world_joint");
    if (world_joint == NULL) {
        oa_fail("world -> openarm_body_link0 fixed TF root is absent");
    }
    char *const joint_type = (char *)xmlGetProp(world_joint, (const xmlChar *)"type");
    if (joint_type == NULL || strcmp(joint_type, "fixed") != 0) {
        oa_fail("world -> openarm_body_link0 fixed TF root is absent");
    }
    xmlFree(joint_type);
    char *const parent_link = child_link(world_joint, "parent");
    char *const child = child_link(world_joint, "child");
    if (parent_link == NULL || child == NULL ||
        strcmp(parent_link, "world") != 0 ||
        strcmp(child, "openarm_body_link0") != 0) {
        oa_fail("unexpected static TF root frames");
    }
    xmlFree(parent_link);
    xmlFree(child);

    for (int index = 1; index <= 7; ++index) {
        char name[64];
        snprintf(name, sizeof(name), "openarm_left_joint%d", index);
        if (!joint_exists(context, name)) {
            oa_fail("missing generated joint: %s", name);
        }
        snprintf(name, sizeof(name), "openarm_right_joint%d", index);
        if (!joint_exists(context, name)) {
            oa_fail("missing generated joint: %s", name);
        }
    }
    if (!joint_exists(context, "openarm_left_finger_joint1") ||
        !joint_exists(context, "openarm_right_finger_joint1")) {
        oa_fail("missing generated finger joints");
    }
    if (!joint_exists(context, "openarm_left_finger_joint2") ||
        !joint_exists(context, "openarm_right_finger_joint2")) {
        oa_fail("generated mimic finger joints are absent");
    }

    xmlXPathObjectPtr meshes =
        xmlXPathEvalExpression((const xmlChar *)"//mesh", context);
    if (meshes == NULL || meshes->nodesetval == NULL ||
        meshes->nodesetval->nodeNr == 0) {
        oa_fail("generated URDF has no mesh references");
    }
    static const char prefix[] = "package://openarm_description/";
    const size_t prefix_length = sizeof(prefix) - 1u;
    for (int index = 0; index < meshes->nodesetval->nodeNr; ++index) {
        char *const filename = (char *)xmlGetProp(meshes->nodesetval->nodeTab[index],
                                                  (const xmlChar *)"filename");
        if (filename == NULL) {
            oa_fail("mesh element without a filename attribute");
        }
        if (strncmp(filename, prefix, prefix_length) != 0) {
            oa_fail("unexpected mesh URI: %s", filename);
        }
        char resolved[4096];
        snprintf(resolved, sizeof(resolved), "%s/%s", description_root,
                 filename + prefix_length);
        if (!is_regular_file(resolved)) {
            oa_fail("unresolved mesh: %s -> %s", filename, resolved);
        }
        xmlFree(filename);
    }
    xmlXPathFreeObject(meshes);
    xmlXPathFreeContext(context);
    xmlFreeDoc(document);
    xmlCleanupParser();
    return 0;
}
