/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OA_MODEL_INTERNAL_H
#define OA_MODEL_INTERNAL_H

#include "openarm_model.h"

struct oa_model {
    const char *id;
    const char *provenance;
    const char *data_sha256;
    const char *flattened_urdf_sha256;
    const char *source_sha256;
    const char *joint_name[OA_DOF];
    const char *tip_frame;
    oa_transform base_in_body;
    oa_transform origin[OA_DOF];
    double axis[OA_DOF][3];
    double lower[OA_DOF];
    double upper[OA_DOF];
    oa_transform tcp_in_link7;
};

#endif
