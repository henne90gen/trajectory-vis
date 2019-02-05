#pragma once

#include <cgv/render/shader_program.h>
#include <cgv/render/context.h>
#include <cgv/render/vertex_buffer.h>

#include "types.h"

namespace ellipsoid_trajectory {

    class traj_line_renderer
    {
    public:
        traj_line_renderer();

        // inits renderer by creating shader program
        void init(cgv::render::context& ctx);

        // resets necessary properties of renderer for new set up and avoids generating new VBOs
        void reset();

        // creates a VAO and all necessary buffers (VBO and EBO) needed for this renderer on GPU
        void set_buffers(cgv::render::context& ctx, std::vector<vec3>& positions, std::vector<vec4>& colors, std::vector<unsigned int>& indices);

        // update element buffer while letting all vertex data the same on GPU
        void update_element_buffer(std::vector<unsigned int>& indices);
        void update_position_buffer(std::vector<vec3>& positions);

        // enables shader and VAO and draws elements determined by EBO
        void draw(cgv::render::context& ctx, mat view, mat projection);

        // determine if it is the first rendering pass for this render
        bool initial;

    private:
        // compiled shader program
        cgv::render::shader_program prog;

        // ids of all buffers
        unsigned int VAO;
        unsigned int EBO;
        unsigned int VBO_positions;
        unsigned int VBO_colors;
        unsigned int nr_elements;

        // model matrix (here not really used yet)
        mat model;
    };
}
