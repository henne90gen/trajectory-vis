#pragma once

#include <cgv/render/shader_program.h>
#include <cgv/render/context.h>
#include <cgv/render/vertex_buffer.h>

#include "types.h"

namespace ellipsoid_trajectory {

    class traj_velocity_renderer
    {
    public:
        traj_velocity_renderer();

        // inits renderer by creating shader program
        void init(cgv::render::context& ctx);

        // resets necessary properties of renderer for new set up and avoids generating new VBOs
        void reset();

        // creates a VAO and all necessary buffers needed for this renderer on GPU
        void set_buffers(cgv::render::context& ctx, std::vector<vec3>& vertices, std::vector<vec3>& velocities, bool use_value_color);

        void update_position_buffer(std::vector<vec3>& vertices);
        void update_velocity_buffer(std::vector<vec3>& vertices);
        void set_color(bool use_value_color);

        // enables shader and VAO and draws elements determined by EBO
        void draw(cgv::render::context& ctx);

        // determine if it is the first rendering pass for this render
        bool initial;

    private:
        // compiled shader program
        cgv::render::shader_program prog;

        bool value_color;
        float max_velocity;

        // ids of all buffers
        unsigned int VAO;
        unsigned int VBO_positions;
        unsigned int VBO_velocities;
        unsigned int nr_vertices;
    };
}
