/* SPDX-License-Identifier: Apache-2.0 */
/* Exact OpenArm v1.0 claw collision-mesh evidence behind the public C ABI. */
#include "openarm_collision.h"

#include <fcl/fcl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

namespace {

using Mesh = fcl::BVHModel<fcl::OBBRSSd>;
using MeshPtr = std::shared_ptr<Mesh>;

#include "generated/oa_claw_mesh_data.inc"

constexpr double kMillimetresToMetres = 0.001;
constexpr double kMeshOriginFromTcpZ = -0.744501;

template <std::size_t TriangleCount>
MeshPtr make_mesh(const double (&triangles)[TriangleCount][3U][3U],
                  const std::size_t part) {
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> indices;
    vertices.reserve(TriangleCount * 3U);
    indices.reserve(TriangleCount);
    for (std::size_t triangle = 0U; triangle < TriangleCount; ++triangle) {
        const int first = static_cast<int>(vertices.size());
        for (std::size_t vertex = 0U; vertex < 3U; ++vertex) {
            const double raw_x = triangles[triangle][vertex][0U];
            const double raw_y = triangles[triangle][vertex][1U];
            const double raw_z = triangles[triangle][vertex][2U];
            const double x = kMillimetresToMetres * raw_x;
            const double y = part == 0U
                                 ? kMillimetresToMetres * raw_y
                                 : (part == 1U
                                        ? -0.045 + kMillimetresToMetres * raw_y
                                        : 0.045 - kMillimetresToMetres * raw_y);
            const double z = kMeshOriginFromTcpZ + kMillimetresToMetres * raw_z;
            vertices.emplace_back(x, y, z);
        }
        indices.emplace_back(first, first + 1, first + 2);
    }
    auto mesh = std::make_shared<Mesh>();
    if (mesh->beginModel() != fcl::BVH_OK ||
        mesh->addSubModel(vertices, indices) != fcl::BVH_OK ||
        mesh->endModel() != fcl::BVH_OK) {
        return {};
    }
    return mesh;
}

const std::array<MeshPtr, 3U> &claw_meshes() {
    static const std::array<MeshPtr, 3U> meshes{
        make_mesh(kHandTrianglesMm, 0U), make_mesh(kFingerTrianglesMm, 1U),
        make_mesh(kFingerTrianglesMm, 2U)};
    return meshes;
}

bool transform_from_c(const oa_transform *source, fcl::Transform3d &output) {
    if (source == nullptr) return false;
    for (double value : source->m) {
        if (!std::isfinite(value)) return false;
    }
    output = fcl::Transform3d::Identity();
    output.linear() << source->m[0U], source->m[1U], source->m[2U],
        source->m[4U], source->m[5U], source->m[6U], source->m[8U],
        source->m[9U], source->m[10U];
    output.translation() =
        fcl::Vector3d(source->m[3U], source->m[7U], source->m[11U]);
    return output.matrix().allFinite();
}

double signed_mesh_gap(const fcl::CollisionObjectd &left,
                       const fcl::CollisionObjectd &right) {
    fcl::CollisionRequestd collision_request(128U, true);
    fcl::CollisionResultd collision_result;
    fcl::collide(&left, &right, collision_request, collision_result);
    if (collision_result.isCollision()) {
        std::vector<fcl::Contactd> contacts;
        collision_result.getContacts(contacts);
        double maximum_penetration = 0.0;
        for (const auto &contact : contacts) {
            if (std::isfinite(contact.penetration_depth)) {
                maximum_penetration =
                    std::max(maximum_penetration, contact.penetration_depth);
            }
        }
        return -maximum_penetration;
    }
    fcl::DistanceRequestd distance_request(true);
    fcl::DistanceResultd distance_result;
    const double distance =
        fcl::distance(&left, &right, distance_request, distance_result);
    return std::isfinite(distance) && distance >= 0.0
               ? distance
               : -std::numeric_limits<double>::infinity();
}

}  // namespace

extern "C" oa_model_status oa_collision_claw_mesh_evidence(
    const oa_transform *left_hand_tcp, const oa_transform *right_hand_tcp,
    double *hand_gap_m, double *minimum_other_gap_m) {
    if (hand_gap_m == nullptr || minimum_other_gap_m == nullptr) {
        return OA_MODEL_EINVAL;
    }
    *hand_gap_m = -std::numeric_limits<double>::infinity();
    *minimum_other_gap_m = -std::numeric_limits<double>::infinity();
    fcl::Transform3d left_transform;
    fcl::Transform3d right_transform;
    if (!transform_from_c(left_hand_tcp, left_transform) ||
        !transform_from_c(right_hand_tcp, right_transform)) {
        return OA_MODEL_ENONFINITE;
    }
    const auto &meshes = claw_meshes();
    if (std::any_of(meshes.begin(), meshes.end(),
                    [](const MeshPtr &mesh) { return mesh == nullptr; })) {
        return OA_MODEL_EINVAL;
    }
    std::array<std::unique_ptr<fcl::CollisionObjectd>, 3U> left;
    std::array<std::unique_ptr<fcl::CollisionObjectd>, 3U> right;
    for (std::size_t part = 0U; part < meshes.size(); ++part) {
        left[part] =
            std::make_unique<fcl::CollisionObjectd>(meshes[part], left_transform);
        right[part] =
            std::make_unique<fcl::CollisionObjectd>(meshes[part], right_transform);
    }
    double minimum_other = std::numeric_limits<double>::infinity();
    for (std::size_t left_part = 0U; left_part < meshes.size(); ++left_part) {
        for (std::size_t right_part = 0U; right_part < meshes.size(); ++right_part) {
            const double gap = signed_mesh_gap(*left[left_part], *right[right_part]);
            if (!std::isfinite(gap)) return OA_MODEL_ENONFINITE;
            if (left_part == 0U && right_part == 0U) {
                *hand_gap_m = gap;
            } else {
                minimum_other = std::min(minimum_other, gap);
            }
        }
    }
    if (!std::isfinite(*hand_gap_m) || !std::isfinite(minimum_other)) {
        return OA_MODEL_ENONFINITE;
    }
    *minimum_other_gap_m = minimum_other;
    return OA_MODEL_OK;
}
