#pragma once

#include <cgv/render/shader_program.h>
#include <cgv/render/context.h>
#include <cgv/render/vertex_buffer.h>

#include "types.h"
#include "lighting.h"

namespace ellipsoid_trajectory {

    class traj_ribbon_renderer
    {
    public:
        traj_ribbon_renderer();

        // inits renderer by creating shader program
        void init(cgv::render::context& ctx, lighting* _scene_lighting, int _tick_sample_count);

        // resets necessary properties of renderer for new set up (e.g. current_index)
        void reset();

        // creates a VAO and all necessary buffers (VBO and EBO) needed for this renderer on GPU
        void set_buffers(cgv::render::context& ctx, std::vector<vec3>& vertices, std::vector<vec4>& colors, std::vector<unsigned int>& indices);

        // update element buffer while letting all vertex data the same on GPU
        void update_element_buffer(std::vector<unsigned int>& indices);

        // enables shader and VAO and draws elements determined by EBO
        void draw(cgv::render::context& ctx, vec3 view_position);

        void create_vertices(std::vector<vec3>& vertices_out, std::vector<vec4>& colors_out, std::vector<vec3>& positions_in, vec3 axes_in, std::vector<vec4>& orientations_in, std::vector<vec4>&colors_in);

        // determine if it is the first rendering pass for this render
        bool initial;

        std::vector<std::vector<unsigned int>> indices;

    private:
        // compiled shader program
        cgv::render::shader_program prog;
        lighting* scene_light;
        int tick_sample_count;

        unsigned int current_index;

        // ids of all buffers
        unsigned int VAO;
        unsigned int EBO;
        unsigned int VBO_positions;
        unsigned int VBO_colors;
        unsigned int nr_elements;
    };
}
