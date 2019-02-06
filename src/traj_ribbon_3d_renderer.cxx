#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

#include <cgv/math/mat.h>

#include "traj_ribbon_3d_renderer.h"
#include "math_utils.h"

using namespace cgv::render;

namespace ellipsoid_trajectory {

    traj_ribbon_3d_renderer::traj_ribbon_3d_renderer()
    {
        initial = true;
        nr_elements = 0;
        current_index = 0;
    }

    void traj_ribbon_3d_renderer::init(context& ctx, lighting* _scene_light, Material _material, int _tick_sample_count)
    {
        scene_light = _scene_light;
        material = _material;
        tick_sample_count = _tick_sample_count;
        
        if (!prog.is_created()) {
            if (!prog.build_program(ctx, "traj_ribbon_3d_shader.glpr", true)) {
                std::cerr << "ERROR in traj_ribbon_3d_renderer::init() ... could not build program traj_ribbon_3d_shader.glpr" << std::endl;
            }
        }

        // generate and bind vertex attribute object and its VBOs
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO_positions);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glGenBuffers(1, &VBO_colors);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glGenBuffers(1, &VBO_normals);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBindVertexArray(0);
    }

    void traj_ribbon_3d_renderer::reset()
    {
        // for external checks
        initial = true;

        // internal values
        current_index = 0;
        nr_elements = 0;

        indices_top.resize(0);
        indices_side1.resize(0);
        indices_bottom.resize(0);
        indices_side2.resize(0);
    }

    void traj_ribbon_3d_renderer::reserve_memory(size_t trajs, size_t time_steps)
    {
        indices_top.reserve(trajs * time_steps * 2);
        indices_side1.reserve(trajs * time_steps * 2);
        indices_bottom.reserve(trajs * time_steps * 2);
        indices_side2.reserve(trajs * time_steps * 2);
    }

    void traj_ribbon_3d_renderer::set_buffers(context& ctx, std::vector<vec3>& positions, std::vector<vec4>& colors, std::vector<vec3>& normals, std::vector<unsigned int>& indices)
    {
        // Account for CGV shaderpath not being set until after ::init (This might not be the optimal place to put this)
        glGetError(); // <-- Take care of potentially orphaned previous errors to prevent false failure detection in CGV shader building code
        if (!prog.is_linked()) {
            if (!prog.build_program(ctx, "traj_ribbon_3d_shader.glpr", true)) {
                std::cerr << "ERROR in traj_ribbon_3d_renderer::init() ... could not build program traj_ribbon_3d_shader.glpr" << std::endl;
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

    void traj_ribbon_3d_renderer::update_element_buffer(std::vector<unsigned int>& indices)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_DYNAMIC_DRAW);

        nr_elements = indices.size();

        glBindVertexArray(0);
    }

    void traj_ribbon_3d_renderer::update_material(Material _material)
    {
        material = _material;
    }

    void traj_ribbon_3d_renderer::draw(context& ctx, vec3 view_position)
    {
        // enable VAO and shader with all its variables
        glBindVertexArray(VAO);

        glDisable(GL_CULL_FACE);

        // enable shader and set all uniform shader variables
        prog.enable(ctx);

        prog.set_uniform(ctx, "tick_sample_count", tick_sample_count);

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
        glDrawElements(GL_TRIANGLE_STRIP, nr_elements, GL_UNSIGNED_INT, 0);

        // disable everything again
        glBindVertexArray(0);
        prog.disable(ctx);

        glEnable(GL_CULL_FACE);
    }

    void traj_ribbon_3d_renderer::create_vertices(std::vector<vec3>& vertices_out, std::vector<vec4>& colors_out, std::vector<vec3>& normals_out, std::vector<vec3>& positions_in, vec3 main_axis_in, std::vector<vec3>& normals_in, std::vector<vec4>& orientations_in, std::vector<vec4>&colors_in)
    {
        // both axis directions
        vec3 axis_positive = main_axis_in;
        vec3 axis_negative = axis_positive * -1;

        std::vector<unsigned int> _indices_top;
        indices_top.reserve(positions_in.size() * 2);
        std::vector<unsigned int> _indices_side1;
        indices_side1.reserve(positions_in.size() * 2);
        std::vector<unsigned int> _indices_bottom;
        indices_bottom.reserve(positions_in.size() * 2);
        std::vector<unsigned int> _indices_side2;
        indices_side2.reserve(positions_in.size() * 2);

        // form a mantle of a 3D ribbon:
        //        v1   next1
        //        o----o 
        //       /    /|
        //   v2 o----o o _next1
        //      |    |/
        //      o----o
        //     _v2  _next2
        // height of ribbon
        float height = 0.1f;

        for (size_t t = 0; t < positions_in.size(); t++) {
            // color for this tmestep
            vec4 color;
            // adding darker ticks at every 10. time step
            color = colors_in[t];



            // apply current orientation to axis
            vec3 axis1 = quat_rotate(axis_positive, orientations_in[t]);
            vec3 axis2 = quat_rotate(axis_negative, orientations_in[t]);

            // points on ribbon mantle with original points in the middle of them
            vec3 height_offset = (normals_in[t] * -1 * height) / 2.0f;
            vec3 v1 = axis1 + positions_in[t] - height_offset;
            vec3 v2 = axis2 + positions_in[t] - height_offset;
            vec3 _v1 = v1 + (height_offset * 2);
            vec3 _v2 = v2 + (height_offset * 2);

            // normals for sides
            vec3 normal_side = axis1.normalize();

            // top of ribbon
            //
            // compute points at outer edge of axis along trajectory
            vertices_out.push_back(v1);
            vertices_out.push_back(v2);

            // store indices
            _indices_top.push_back(current_index++);
            _indices_top.push_back(current_index++);

            normals_out.push_back(normals_in[t]);
            normals_out.push_back(normals_in[t]);

            colors_out.push_back(color);
            colors_out.push_back(color);


            // side of ribbon
            //
            // compute points at outer edge of axis along trajectory
            vertices_out.push_back(v1);
            vertices_out.push_back(_v1);

            // store indices
            _indices_side1.push_back(current_index++);
            _indices_side1.push_back(current_index++);

            normals_out.push_back(normal_side);
            normals_out.push_back(normal_side);

            colors_out.push_back(color);
            colors_out.push_back(color);


            // bottom of ribbon
            //
            // compute points at outer edge of axis along trajectory
            vertices_out.push_back(_v2);
            vertices_out.push_back(_v1);

            // store indices
            _indices_bottom.push_back(current_index++);
            _indices_bottom.push_back(current_index++);

            normals_out.push_back(normals_in[t] * -1);
            normals_out.push_back(normals_in[t] * -1);

            colors_out.push_back(color);
            colors_out.push_back(color);


            // side 2 of ribbon
            //
            // compute points at outer edge of axis along trajectory
            vertices_out.push_back(v2);
            vertices_out.push_back(_v2);

            // store indices
            _indices_side2.push_back(current_index++);
            _indices_side2.push_back(current_index++);

            normals_out.push_back(normal_side * -1);
            normals_out.push_back(normal_side * -1);

            colors_out.push_back(color);
            colors_out.push_back(color);
        }

        indices_top.push_back(_indices_top);
        indices_side1.push_back(_indices_side1);
        indices_bottom.push_back(_indices_bottom);
        indices_side2.push_back(_indices_side2);
    }
}
