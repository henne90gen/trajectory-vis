#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

#include <cgv/math/mat.h>

#include "traj_velocity_renderer.h"

using namespace cgv::render;

namespace ellipsoid_trajectory {

    traj_velocity_renderer::traj_velocity_renderer()
    {
        initial = true;
        value_color = true;
        max_velocity = 0.0f;
    }

    void traj_velocity_renderer::init(context& ctx)
    {
        if (!prog.is_created()) {
            if (!prog.build_program(ctx, "traj_velocity_shader.glpr", true)) {
                std::cerr << "ERROR in traj_velocity_renderer::init() ... could not build program traj_velocity_shader.glpr" << std::endl;
            }
        }

        // generate and bind vertex attribute object and its VBOs
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO_positions);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glGenBuffers(1, &VBO_velocities);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_velocities);
        glBindVertexArray(0);
    }

    void traj_velocity_renderer::reset()
    {
        // for external checks
        initial = true;
    }

    void traj_velocity_renderer::set_buffers(context& ctx, std::vector<vec3>& vertices, std::vector<vec3>& velocities, bool use_value_color)
    {
        // Account for CGV shaderpath not being set until after ::init (This might not be the optimal place to put this)
        glGetError(); // <-- Take care of potentially orphaned previous errors to prevent false failure detection in CGV shader building code
        if (!prog.is_linked()) {
            if (!prog.build_program(ctx, "traj_velocity_shader.glpr", true)) {
                std::cerr << "ERROR in traj_velocity_renderer::init() ... could not build program traj_velocity_shader.glpr" << std::endl;
            }
        }

        // bind vertex attribute object
        glBindVertexArray(VAO);

        // bind vertex buffer object for positions
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * 3 * sizeof(float), (float*)vertices[0], GL_DYNAMIC_DRAW);

        // position attribute
        int loc = prog.get_attribute_location(ctx, "position");
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // bind vertex buffer object for positions
        glBindBuffer(GL_ARRAY_BUFFER, VBO_velocities);
        glBufferData(GL_ARRAY_BUFFER, velocities.size() * 3 * sizeof(float), (float*)velocities[0], GL_DYNAMIC_DRAW);

        // position attribute
        loc = prog.get_attribute_location(ctx, "velocity");
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // update maximum value for velocity
        for (size_t v = 0; v < velocities.size(); v++) {
            if (velocities[v].length() > max_velocity)
                max_velocity = velocities[v].length();
        }

        nr_vertices = vertices.size();
        value_color = use_value_color;

        // unbind VAO
        glBindVertexArray(0);
    }

    void traj_velocity_renderer::update_position_buffer(std::vector<vec3>& vertices)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * 3 * sizeof(float), (float*)vertices[0], GL_DYNAMIC_DRAW);

        nr_vertices = vertices.size();

        glBindVertexArray(0);
    }

    void traj_velocity_renderer::update_velocity_buffer(std::vector<vec3>& velocities)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO_velocities);
        glBufferData(GL_ARRAY_BUFFER, velocities.size() * 3 * sizeof(float), (float*)velocities[0], GL_DYNAMIC_DRAW);

        glBindVertexArray(0);

        // update maximum value for velocity
        max_velocity = 0.0f;
        for (size_t v = 0; v < velocities.size(); v++) {
            if (velocities[v].length() > max_velocity)
                max_velocity = velocities[v].length();
        }
    }

    void traj_velocity_renderer::set_color(bool use_value_color)
    {
        value_color = use_value_color;
    }

    void traj_velocity_renderer::draw(context& ctx, mat view, mat projection)
    {
        // enable VAO and shader with all its variables
        glBindVertexArray(VAO);

        // enable shader and set all uniform shader variables
        prog.enable(ctx);
        prog.set_uniform(ctx, "view", view);
        prog.set_uniform(ctx, "projection", projection);
        prog.set_uniform(ctx, "value_color", value_color);
        prog.set_uniform(ctx, "max_velocity", max_velocity);

        // draw call
        glDrawArrays(GL_POINTS, 0, nr_vertices);

        // disable everything again
        glBindVertexArray(0);
        prog.disable(ctx);
    }
}
