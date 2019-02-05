#include "lighting.h"

namespace ellipsoid_trajectory {
    lighting::lighting()
    {
        // default light
        light.position = vec3(50.2f, 100.0f, 50.0f);
        vec3 light_color = vec3(1.0f, 1.0f, 1.0f);
        light.diffuse = light_color * 0.4f;
        light.ambient = light_color * 0.85f;
        light.specular = light_color;
    }

    void lighting::set_light_position(vec3 pos)
    {
        light.position = pos;
    }
}