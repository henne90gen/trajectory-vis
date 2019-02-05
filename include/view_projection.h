#pragma once

#include "types.h"

namespace ellipsoid_trajectory {

    class view_projection_matrix
    {
    public:
        view_projection_matrix();

        void compute_view_matrix(const dvec3 _eye, dvec3 _center, dvec3 _up);
        void compute_projection_matrix(float fovy, int width, int height);

        mat projection;
        mat view;
        vec3 view_position;
    };
}