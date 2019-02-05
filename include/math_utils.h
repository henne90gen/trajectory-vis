#pragma once

#include "types.h"

namespace ellipsoid_trajectory {
    // TODO: quaternion class?
    vec3 quat_rotate(vec3 position, vec4 quat);
    vec4 quat_mul(vec4 q0, vec4 q1);
    vec4 quat_normed(vec4 q);
    vec4 slerp(vec4 qa, vec4 qb, double t);

    // euler angle in radian to quaternion
    vec4 to_quat(double pitch, double roll, double yaw);

    // creates the vertices of a ellipsoid storing the position, normals and texture coordinates
    void create_ellipsoid_vertices(std::vector<vec4>& vertices, std::vector<vec4>& normals, std::vector<vec2>& texture_coord, vec3 axes, unsigned int stacks = 15, unsigned int slices = 10, vec4 center = vec4(0.0f, 0.0f, 0.0f, 1.0f));

    // create vertices and indices for wired box from a min and max coordinates
    void create_box_vertices(std::vector<vec3>& vertices, std::vector<unsigned int>& indices, vec3 min, vec3 max, unsigned int restart_id);
}