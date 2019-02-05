#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

#include <cgv/math/mat.h>

#include "traj_ribbon_3d_renderer_gpu.h"

using namespace cgv::render;

namespace ellipsoid_trajectory {

    traj_ribbon_3d_renderer_gpu::traj_ribbon_3d_renderer_gpu()
    {
        initial = true;
        nr_elements = 0;
        height = 0.1;
    }

    void traj_ribbon_3d_renderer_gpu::init(context& ctx, lighting* _scene_light, Material _material, int _tick_sample_count)
    {
        scene_light = _scene_light;
        material = _material;
        tick_sample_count = _tick_sample_count;
        
        if (!prog.is_created()) {
            if (!prog.build_program(ctx, "traj_ribbon_3d_gpu_shader.glpr", true)) {
                std::cerr << "ERROR in traj_ribbon_3d_renderer_gpu::init() ... could not build program traj_ribbon_3d_gpu_shader.glpr" << std::endl;
            }
        }

        // generate and bind vertex attribute object and its VBOs
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO_positions);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glGenBuffers(1, &VBO_colors);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glGenBuffers(1, &VBO_axes);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_axes);
        glGenBuffers(1, &VBO_orientations);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_orientations);
        glGenBuffers(1, &VBO_normals);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBindVertexArray(0);
    }

    void traj_ribbon_3d_renderer_gpu::reset()
    {
        // for external checks
        initial = true;

        // internal values
        nr_elements = 0;
    }

    void traj_ribbon_3d_renderer_gpu::set_buffers(context& ctx, std::vector<vec3>& positions, std::vector<vec4>& colors, std::vector<vec3>& axes, std::vector<vec4>& orientations, std::vector<vec3>& normals, std::vector<unsigned int>& indices)
    {
        // Account for CGV shaderpath not being set until after ::init (This might not be the optimal place to put this)
        glGetError(); // <-- Take care of potentially orphaned previous errors to prevent false failure detection in CGV shader building code
        if (!prog.is_linked()) {
            if (!prog.build_program(ctx, "traj_ribbon_3d_gpu_shader.glpr", true)) {
                std::cerr << "ERROR in traj_ribbon_3d_renderer_gpu::init() ... could not build program traj_ribbon_3d_gpu_shader.glpr" << std::endl;
            }
        }

        // bind vertex attribute object
        glBindVertexArray(VAO);

        // bind vertex buffer object for positions
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * 3 * sizeof(float), (float*)positions[0], GL_STATIC_DRAW);

        int loc = prog.get_attribute_location(ctx, "position");
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind vertex buffer object for colors
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * 4 * sizeof(float), (float*)colors[0], GL_STATIC_DRAW);

        loc = prog.get_attribute_location(ctx, "color");
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind vertex buffer object for axes
        glBindBuffer(GL_ARRAY_BUFFER, VBO_axes);
        glBufferData(GL_ARRAY_BUFFER, axes.size() * 3 * sizeof(float), (float*)axes[0], GL_STATIC_DRAW);

        loc = prog.get_attribute_location(ctx, "main_axis");
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind vertex buffer object for orientations
        glBindBuffer(GL_ARRAY_BUFFER, VBO_orientations);
        glBufferData(GL_ARRAY_BUFFER, orientations.size() * 4 * sizeof(float), (float*)orientations[0], GL_STATIC_DRAW);

        loc = prog.get_attribute_location(ctx, "orientation");
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind vertex buffer object for normals
        glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * 3 * sizeof(float), (float*)normals[0], GL_STATIC_DRAW);

        loc = prog.get_attribute_location(ctx, "normal");
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind element buffer object
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_DYNAMIC_DRAW);

        nr_elements = indices.size();

        // unbind VAO
        glBindVertexArray(0);
    }

    void traj_ribbon_3d_renderer_gpu::update_element_buffer(std::vector<unsigned int>& indices)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_DYNAMIC_DRAW);

        nr_elements = indices.size();

        glBindVertexArray(0);
    }

    void traj_ribbon_3d_renderer_gpu::update_material(Material _material)
    {
        material = _material;
    }

    void traj_ribbon_3d_renderer_gpu::draw(context& ctx, mat view, mat projection, vec3 view_position)
    {
        // enable VAO and shader with all its variables
        glBindVertexArray(VAO);

        // enable shader and set all uniform shader variables
        prog.enable(ctx);
        prog.set_uniform(ctx, "view", view);
        prog.set_uniform(ctx, "projection", projection);

        prog.set_uniform(ctx, "tick_sample_count", tick_sample_count);
        prog.set_uniform(ctx, "height", height);

        prog.set_uniform(ctx, "view_pos", view_position);
        prog.set_uniform(ctx, "light.ambient", scene_light->light.ambient);
        prog.set_uniform(ctx, "light.diffuse", scene_light->light.diffuse);
        prog.set_uniform(ctx, "light.specular", scene_light->light.specular);
        prog.set_uniform(ctx, "light.position", scene_light->light.position);

        // material properties
        prog.set_uniform(ctx, "material.ambient", material.ambient);
        prog.set_uniform(ctx, "material.diffuse", material.diffuse);
        prog.set_uniform(ctx, "material.specular", material.specular);
        prog.set_uniform(ctx, "material.shininess", material.shininess);

        // draw call
        glDrawElements(GL_LINES, nr_elements, GL_UNSIGNED_INT, 0);

        // disable everything again
        glBindVertexArray(0);
        prog.disable(ctx);
    }
}
