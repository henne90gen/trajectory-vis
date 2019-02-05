#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

#include <cgv/math/mat.h>

#include "traj_line_renderer.h"

using namespace cgv::render;

namespace ellipsoid_trajectory {

    traj_line_renderer::traj_line_renderer()
    {
        model = cgv::math::mat<float>(4,4);
        model.identity();

        initial = true;
        nr_elements = 0;
    }

    void traj_line_renderer::init(context& ctx)
    {
        if (!prog.is_created()) {
            if (!prog.build_program(ctx, "traj_line_shader.glpr", true)) {
                std::cerr << "ERROR in traj_line_renderer::init() ... could not build program traj_line_shader.glpr" << std::endl;
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

    void traj_line_renderer::reset()
    {
        // for external checks
        initial = true;

        // internal values
        nr_elements = 0;
    }

    void traj_line_renderer::set_buffers(context& ctx, std::vector<vec3>& positions, std::vector<vec4>& colors, std::vector<unsigned int>& indices)
    {
        // Account for CGV shaderpath not being set until after ::init (This might not be the optimal place to put this)
        glGetError(); // <-- Take care of potentially orphaned previous errors to prevent false failure detection in CGV shader building code
        if (!prog.is_linked()) {
            if (!prog.build_program(ctx, "traj_line_shader.glpr", true)) {
                std::cerr << "ERROR in traj_line_renderer::init() ... could not build program traj_line_shader.glpr" << std::endl;
            }
        }

        // bind vertex attribute object
        glBindVertexArray(VAO);

        // bind vertex buffer object for positions
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * 3 * sizeof(float), (float*)positions[0], GL_STATIC_DRAW);

        // position attribute
        int loc = prog.get_attribute_location(ctx, "position");
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind vertex buffer object for colors
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * 4 * sizeof(float), (float*)colors[0], GL_STATIC_DRAW);

        // color attribute
        loc = prog.get_attribute_location(ctx, "color");
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind element buffer object
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_DYNAMIC_DRAW);

        nr_elements = indices.size();

        // unbind VAO
        glBindVertexArray(0);
    }

    void traj_line_renderer::update_element_buffer(std::vector<unsigned int>& indices)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_DYNAMIC_DRAW);

        nr_elements = indices.size();

        glBindVertexArray(0);
    }

    void traj_line_renderer::update_position_buffer(std::vector<vec3>& positions)
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * 3 * sizeof(float), (float*)positions[0], GL_DYNAMIC_DRAW);
    }

    void traj_line_renderer::draw(context& ctx, mat view, mat projection)
    {
        // enable VAO and shader with all its variables
        glBindVertexArray(VAO);

        // enable shader and set all uniform shader variables
        prog.enable(ctx);
        prog.set_uniform(ctx, "model", model);
        prog.set_uniform(ctx, "view", view);
        prog.set_uniform(ctx, "projection", projection);

        // draw call
        // glDrawElements(GL_LINES, nr_elements, GL_UNSIGNED_INT, 0);
        glDrawElements(GL_LINE_STRIP, nr_elements, GL_UNSIGNED_INT, 0);

        // disable everything again
        glBindVertexArray(0);
        prog.disable(ctx);
    }
}
