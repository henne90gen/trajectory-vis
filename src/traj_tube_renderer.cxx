#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

#include <cgv/math/mat.h>
#include <cgv/math/transformations.h>

#include "traj_tube_renderer.h"
#include "math_utils.h"

using namespace cgv::render;

namespace ellipsoid_trajectory {

    traj_tube_renderer::traj_tube_renderer()
    {
        model = cgv::math::mat<float>(4,4);
        model.identity();

        nr_vertices = 0;
        nr_instances = 0;

        initial = true;
    }

    void traj_tube_renderer::init(context& ctx, lighting* _scene_light, Material _material)
    {
        scene_light = _scene_light;
        material = _material;
        
        if (!prog.is_created()) {
            if (!prog.build_program(ctx, "traj_tube_shader.glpr", true)) {
                std::cerr << "ERROR in traj_tube_renderer::init() ... could not build program traj_tube_shader.glpr" << std::endl;
            }
        }

        // generate and bind vertex attribute object and its VBOs
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO_positions);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glGenBuffers(1, &VBO_colors);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glGenBuffers(1, &VBO_translations);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_translations);
        glGenBuffers(1, &VBO_orientations);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_orientations);
        glGenBuffers(1, &VBO_normals);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
        glBindVertexArray(0);
    }

    void traj_tube_renderer::reset()
    {
        // for external checks
        initial = true;
    }

    void traj_tube_renderer::set_buffers(context& ctx, std::vector<vec4>& vertices, std::vector<vec4>& normals, std::vector<vec3>& translations, std::vector<vec4>& orientations, std::vector<vec4>& colors)
    {
        // Account for CGV shaderpath not being set until after ::init (This might not be the optimal place to put this)
        glGetError(); // <-- Take care of potentially orphaned previous errors to prevent false failure detection in CGV shader building code
        if (!prog.is_linked()) {
            if (!prog.build_program(ctx, "traj_tube_shader.glpr", true)) {
                std::cerr << "ERROR in traj_tube_renderer::init() ... could not build program traj_tube_shader.glpr" << std::endl;
            }
        }

        // bind vertex attribute object
        glBindVertexArray(VAO);

        // bind vertex buffer object for positions
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * 4 * sizeof(float), (float*)vertices[0], GL_STATIC_DRAW);

        int loc = prog.get_attribute_location(ctx, "position");
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        nr_vertices = vertices.size();

        // bind vertex buffer object for normals
        glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * 4 * sizeof(float), (float*)normals[0], GL_STATIC_DRAW);

        loc = prog.get_attribute_location(ctx, "normal");
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // create instanced attribute for translation
        glBindBuffer(GL_ARRAY_BUFFER, VBO_translations);
        glBufferData(GL_ARRAY_BUFFER, translations.size() * 3 * sizeof(float), &translations[0], GL_DYNAMIC_DRAW);

        loc = prog.get_attribute_location(ctx, "translation");
        glEnableVertexAttribArray(loc);
        glVertexAttribPointer(loc, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

        // this is an instanced attribute
        glVertexAttribDivisor(loc, 1);

        // create instanced attribute for translation
        glBindBuffer(GL_ARRAY_BUFFER, VBO_orientations);
        glBufferData(GL_ARRAY_BUFFER, orientations.size() * 4 * sizeof(float), &orientations[0], GL_DYNAMIC_DRAW);

        loc = prog.get_attribute_location(ctx, "orientation");
        glEnableVertexAttribArray(loc);
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

        // this is an instanced attribute
        glVertexAttribDivisor(loc, 1);

        // bind vertex buffer object for colors
        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * 4 * sizeof(float), (float*)colors[0], GL_DYNAMIC_DRAW);

        loc = prog.get_attribute_location(ctx, "color");
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // this is an instanced attribute
        glVertexAttribDivisor(loc, 1);

        nr_instances = translations.size();

        // unbind VAO
        glBindVertexArray(0);
    }

    void traj_tube_renderer::update_translation_buffer(std::vector<vec3>& translations)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO_translations);
        glBufferData(GL_ARRAY_BUFFER, translations.size() * 3 * sizeof(float), &translations[0], GL_DYNAMIC_DRAW);

        nr_instances = translations.size();

        glBindVertexArray(0);
    }

    void traj_tube_renderer::update_orientation_buffer(std::vector<vec4>& orientations)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO_orientations);
        glBufferData(GL_ARRAY_BUFFER, orientations.size() * 4 * sizeof(float), &orientations[0], GL_DYNAMIC_DRAW);

        nr_instances = orientations.size();

        glBindVertexArray(0);
    }

    void traj_tube_renderer::update_color_buffer(std::vector<vec4>& colors)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * 4 * sizeof(float), &colors[0], GL_DYNAMIC_DRAW);

        nr_instances = colors.size();

        glBindVertexArray(0);
    }

    void traj_tube_renderer::update_material(Material _material)
    {
        material = _material;
    }

    void traj_tube_renderer::draw(context& ctx, vec3 view_position)
    {
        // TODO: change winding order for ellipsoids
        // work around for wrong winding order of triangles (here clockwise)
        // therefore culling the front faces instead of back faces does the trick
        glCullFace(GL_FRONT);

        // enable VAO and shader with all its variables
        glBindVertexArray(VAO);

        // enable shader and set all uniform shader variables
        prog.enable(ctx);

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
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, nr_vertices, nr_instances);

        // disable everything again
        glBindVertexArray(0);
        prog.disable(ctx);
        
        glCullFace(GL_BACK);
    }
}
