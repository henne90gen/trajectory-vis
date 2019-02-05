#include <cgv/math/transformations.h>

#include "view_projection.h"

namespace ellipsoid_trajectory {
    view_projection_matrix::view_projection_matrix()
    {
        view = cgv::math::mat<float>(4,4);
        view.identity();
        projection = cgv::math::mat<float>(4,4);
        projection.identity();
    }

    void view_projection_matrix::compute_view_matrix(const dvec3 _eye, dvec3 _center, dvec3 _up)
    {
        cgv::math::vec<float> eye((float)_eye[0],
                                  (float)_eye[1],
                                  (float)_eye[2]);
        cgv::math::vec<float> center((float)_center[0],
                                  (float)_center[1],
                                  (float)_center[2]);
        cgv::math::vec<float> up((float)_up[0],
                                  (float)_up[1],
                                  (float)_up[2]);

        view_position = vec3((float)_eye[0], (float)_eye[1], (float)_eye[2]);

        view = cgv::math::look_at_44(eye, center, up);
    }

    void view_projection_matrix::compute_projection_matrix(float fovy, int width, int height)
    {
        float aspect = width / (float)height;
        float z_near = 0.01f;
        float z_far = 10000.0f;

        projection = cgv::math::perspective_44(fovy, aspect, z_near, z_far);
    }
}