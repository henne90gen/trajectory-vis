#pragma once

#include "types.h"

namespace ellipsoid_trajectory {

    class lighting
    {
    public:
        lighting();

        void set_light_position(vec3 pos);
        
        Light light;
    };
}