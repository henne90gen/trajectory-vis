#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

#include <cgv/math/mat.h>
#include <cgv/math/fvec.h>

#include "traj_ribbon_renderer.h"
#include "math_utils.h"

using namespace cgv::render;

namespace ellipsoid_trajectory {

    traj_ribbon_renderer::traj_ribbon_renderer()
    {
        initial = true;
        nr_elements = 0;
        current_index = 0;
    }

    void traj_ribbon_renderer::init(context& ctx, lighting* _scene_light, int _tick_sample_count)
    {
        scene_light = _scene_light;
        tick_sample_count = _tick_sample_count;
        
        if (!prog.is_created()) {
            if (!prog.build_program(ctx, "traj_ribbon_shader.glpr", true)) {
                std::cerr << "ERROR in traj_ribbon_renderer::init() ... could not build program traj_ribbon_shader.glpr" << std::endl;
            }
        }

        // generate and bind vertex attribute object and its VBOs
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO_positions);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glGenBuffers(1, &VBO_colors);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBindVertexArray(0);
    }

    void traj_ribbon_renderer::reset()
    {
        // for external checks
        initial = true;

        // internal values
        current_index = 0;
        nr_elements = 0;

        indices.resize(0);
    }

    void traj_ribbon_renderer::set_buffers(context& ctx, std::vector<vec3>& vertices, std::vector<vec4>& colors, std::vector<unsigned int>& indices)
    {
        // Account for CGV shaderpath not being set until after ::init (This might not be the optimal place to put this)
        glGetError(); // <-- Take care of potentially orphaned previous errors to prevent false failure detection in CGV shader building code
        if (!prog.is_linked()) {
            if (!prog.build_program(ctx, "traj_ribbon_shader.glpr", true)) {
                std::cerr << "ERROR in traj_ribbon_renderer::init() ... could not build program traj_ribbon_shader.glpr" << std::endl;
            }
        }

        // bind vertex attribute object
        glBindVertexArray(VAO);

        // bind vertex buffer object for positions
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * 3 * sizeof(float), (float*)vertices[0], GL_STATIC_DRAW);

        int loc = prog.get_attribute_location(ctx, "position");
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind vertex buffer object for colors
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * 4 * sizeof(float), (float*)colors[0], GL_STATIC_DRAW);

        loc = prog.get_attribute_location(ctx, "color");
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind element buffer object
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        nr_elements = indices.size();

        // unbind VAO
        glBindVertexArray(0);
    }

    void traj_ribbon_renderer::update_element_buffer(std::vector<unsigned int>& indices)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        nr_elements = indices.size();

        glBindVertexArray(0);
    }

    void traj_ribbon_renderer::draw(context& ctx, vec3 view_position)
    {
        // enable VAO and shader with all its variables
        glBindVertexArray(VAO);

        glDisable(GL_CULL_FACE);

        // enable shader and set all uniform shader variables
        prog.enable(ctx);
        prog.set_uniform(ctx, "tick_sample_count", tick_sample_count);

        // draw call
        glDrawElements(GL_TRIANGLE_STRIP, nr_elements, GL_UNSIGNED_INT, 0);

        // disable everything again
        glBindVertexArray(0);
        prog.disable(ctx);

        glEnable(GL_CULL_FACE);
    }

    void traj_ribbon_renderer::create_vertices(std::vector<vec3>& vertices_out, std::vector<vec4>& colors_out, std::vector<vec3>& positions_in, vec3 axes_in, std::vector<vec4>& orientations_in, std::vector<vec4>&colors_in)
    {
        // find largest axis
        float axis_max = 0.0f;
        vec3 main_axis = vec3(axes_in[0], 0.0f, 0.0f);

        if (axes_in[0] > axis_max){
            axis_max = axes_in[0];
        }
        if (axes_in[1] > axis_max){
            axis_max = axes_in[1];
            main_axis = vec3(0.0f, axes_in[1], 0.0f);
        }
        if (axes_in[2] > axis_max){
            axis_max = axes_in[2];
            main_axis = vec3(0.0f, 0.0f, axes_in[2]);
        }

        // both axis directions
        vec3 axis_positive = main_axis;
        vec3 axis_negative = axis_positive * -1;

        std::vector<unsigned int> _indices;

        for (size_t t = 0; t < positions_in.size(); t++) {
            // apply current orientation
            vec3 v1 = quat_rotate(axis_positive, orientations_in[t]);
            vec3 v2 = quat_rotate(axis_negative, orientations_in[t]);

            // compute points at outer edge of axis along trajectory
            vertices_out.push_back(v1 + positions_in[t]);
            vertices_out.push_back(v2 + positions_in[t]);

            // store indices
            _indices.push_back(current_index);
            current_index++;
            _indices.push_back(current_index);
            current_index++;

            colors_out.push_back(colors_in[t]);
            colors_out.push_back(colors_in[t]);

        }

        indices.push_back(_indices);
    }
}
