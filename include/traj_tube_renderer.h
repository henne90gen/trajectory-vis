#pragma once

#include <cgv_gl/renderer.h>
#include <cgv/math/mat.h>
#include <cgv/render/context.h>
#include <cgv/render/vertex_buffer.h>

#include "types.h"
#include "lighting.h"

namespace ellipsoid_trajectory {

    class traj_tube_renderer
    {
    public:
        traj_tube_renderer();

        // inits renderer by creating shader program
        void init(cgv::render::context& ctx, lighting* _scene_light, Material _material);

        // resets necessary properties of renderer for new set up
        void reset();
        
        // creates a VAO and all necessary buffers (VBOs) needed for this renderer on GPU
        void set_buffers(cgv::render::context& ctx, std::vector<vec4>& vertices, std::vector<vec4>& normals, std::vector<vec3>& translations, std::vector<vec4>& orientations, std::vector<vec4>& colors);

        void update_translation_buffer(std::vector<vec3>& translations);
        void update_orientation_buffer(std::vector<vec4>& orientations);
        void update_color_buffer(std::vector<vec4>& colors);
        void update_material(Material _material);

        // enables shader and VAO and draws all vertices
        void draw(cgv::render::context& ctx, vec3 view_position);

        // determine if it is the first rendering pass for this render
        bool initial;

    private:
        // compiled shader program
        cgv::render::shader_program prog;
        lighting* scene_light;
        Material material;

        // variables for instanced rendering
        unsigned int nr_vertices;
        unsigned int nr_instances;

        // ids of all buffers
        unsigned int VAO;
        unsigned int VBO_translations;
        unsigned int VBO_orientations;
        unsigned int VBO_positions;
        unsigned int VBO_normals;
        unsigned int VBO_colors;

        mat model;
    };
}
