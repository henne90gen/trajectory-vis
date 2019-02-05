#pragma once

#include <cgv/render/shader_program.h>
#include <cgv/render/context.h>
#include <cgv/render/vertex_buffer.h>

#include "types.h"
#include "lighting.h"

namespace ellipsoid_trajectory {

    class traj_ribbon_3d_renderer_gpu
    {
    public:
        traj_ribbon_3d_renderer_gpu();

        // inits renderer by creating shader program
        void init(cgv::render::context& ctx, lighting* _scene_lighting, Material _material, int _tick_sample_count);

        // resets necessary properties of renderer for new set up (e.g. current_index)
        void reset();

        // creates a VAO and all necessary buffers (VBO and EBO) needed for this renderer on GPU
        void set_buffers(cgv::render::context& ctx, std::vector<vec3>& positions, std::vector<vec4>& colors, std::vector<vec3>& axes, std::vector<vec4>& orientations, std::vector<vec3>& normals, std::vector<unsigned int>& indices);

        // update element buffer while letting all vertex data the same on GPU
        void update_element_buffer(std::vector<unsigned int>& indices);
        void update_material(Material _material);

        // enables shader and VAO and draws elements determined by EBO
        void draw(cgv::render::context& ctx, mat view, mat projection, vec3 view_position);

        // determine if it is the first rendering pass for this render
        bool initial;
        float height;

    private:
        // compiled shader program
        cgv::render::shader_program prog;
        lighting* scene_light;
        Material material;
        int tick_sample_count;

        // ids of all buffers
        unsigned int VAO;
        unsigned int EBO;
        unsigned int VBO_positions;
        unsigned int VBO_axes;
        unsigned int VBO_orientations;
        unsigned int VBO_normals;
        unsigned int VBO_colors;
        unsigned int nr_elements;
    };
}
