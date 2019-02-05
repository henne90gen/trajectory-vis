#pragma once

#include <cgv/math/fvec.h>
#include <cgv/math/mat.h>
#include <cgv/media/color.h>

namespace ellipsoid_trajectory {
    typedef cgv::math::mat<float> mat;

    typedef cgv::math::fvec<float, 2> vec2;
    typedef cgv::math::fvec<float, 3> vec3;
    typedef cgv::math::fvec<float, 4> vec4;

    typedef cgv::math::fvec<double,3> dvec3;

    typedef cgv::media::color<float> clr_type;

    struct Bounding_Box
    {
        vec3 min;
        vec3 max;
        vec3 center;
    };

    struct Material {
        clr_type ambient;
        clr_type diffuse;
        clr_type specular;
        float shininess;
    }; 

    struct Light {
        vec3 position;

        vec3 ambient;
        vec3 diffuse;
        vec3 specular;
    };

    struct Texture2D {
        int height;
        int width;
        std::vector<vec3> texture;
        std::vector<vec2> coord;
    };
}