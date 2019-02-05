#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

#include <cgv/math/mat.h>
#include <cgv/math/transformations.h>

#include "ellipsoid_instanced_renderer.h"
#include "math_utils.h"

using namespace cgv::render;

namespace ellipsoid_trajectory {

    ellipsoid_instanced_renderer::ellipsoid_instanced_renderer()
    {
        model = cgv::math::mat<float>(4,4);
        model.identity();

        nr_vertices = 0;
        nr_instances = 0;

        initial = true;
        textured = false;
    }

    void ellipsoid_instanced_renderer::init(context& ctx, lighting* _scene_light, Material _material, bool _textured)
    {
        scene_light = _scene_light;
        material = _material;
        textured = _textured;
        
        if (!prog.is_created()) {
            if (!prog.build_program(ctx, "traj_ellipsoid_shader.glpr", true)) {
                std::cerr << "ERROR in ellipsoid_instanced_renderer::init() ... could not build program traj_ellipsoid_shader.glpr" << std::endl;
            }
        }

        // generate and bind vertex attribute object and its VBOs
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO_positions);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
        glGenBuffers(1, &VBO_normals);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
        glGenBuffers(1, &VBO_tex_coord);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_tex_coord);

        glGenTextures(1, &texture_2D);
        glBindTexture(GL_TEXTURE_2D, texture_2D);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glGenBuffers(1, &VBO_translations);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_translations);
        glGenBuffers(1, &VBO_orientations);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_orientations);
        glBindVertexArray(0);
    }

    void ellipsoid_instanced_renderer::reset()
    {
        // for external checks
        initial = true;
    }

    void ellipsoid_instanced_renderer::set_buffers(context& ctx, std::vector<vec4>& vertices, std::vector<vec4>& normals, Texture2D& tex, std::vector<vec3>& translations, std::vector<vec4>& orientations)
    {
        // Account for CGV shaderpath not being set until after ::init (This might not be the optimal place to put this)
        glGetError(); // <-- Take care of potentially orphaned previous errors to prevent false failure detection in CGV shader building code
        if (!prog.is_linked()) {
            if (!prog.build_program(ctx, "traj_ellipsoid_shader.glpr", true)) {
                std::cerr << "ERROR in ellipsoid_instanced_renderer::init() ... could not build program traj_ellipsoid_shader.glpr" << std::endl;
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

        // bind vertex buffer object for texture coordinates
        glBindBuffer(GL_ARRAY_BUFFER, VBO_tex_coord);
        glBufferData(GL_ARRAY_BUFFER, tex.coord.size() * 2 * sizeof(float), (float*)tex.coord[0], GL_STATIC_DRAW);
        loc = prog.get_attribute_location(ctx, "tex_coord");
        glVertexAttribPointer(loc, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(loc);

        // set texture
        glBindTexture(GL_TEXTURE_2D, texture_2D);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex.width, tex.height, 0, GL_RGB, GL_FLOAT, &tex.texture[0]);
        glGenerateMipmap(GL_TEXTURE_2D);

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
        nr_instances = translations.size();

        // unbind VAO
        glBindVertexArray(0);
    }

    void ellipsoid_instanced_renderer::update_translation_buffer(std::vector<vec3>& translations)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO_translations);
        glBufferData(GL_ARRAY_BUFFER, translations.size() * 3 * sizeof(float), &translations[0], GL_DYNAMIC_DRAW);

        nr_instances = translations.size();

        glBindVertexArray(0);
    }

    void ellipsoid_instanced_renderer::update_orientation_buffer(std::vector<vec4>& orientations)
    {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO_orientations);
        glBufferData(GL_ARRAY_BUFFER, orientations.size() * 4 * sizeof(float), &orientations[0], GL_DYNAMIC_DRAW);

        nr_instances = orientations.size();

        glBindVertexArray(0);
    }

    void ellipsoid_instanced_renderer::update_material(Material _material)
    {
        material = _material;
    }

    void ellipsoid_instanced_renderer::draw(context& ctx, mat view, mat projection, vec3 view_position)
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_2D);
        
        // enable VAO and shader with all its variables
        glBindVertexArray(VAO);

        // enable shader and set all uniform shader variables
        prog.enable(ctx);

        prog.set_uniform(ctx, "view", view);
        prog.set_uniform(ctx, "projection", projection);
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

        prog.set_uniform(ctx, "texture1", 0);
        prog.set_uniform(ctx, "textured", textured);


        // draw call
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, nr_vertices, nr_instances);

        // disable everything again
        glBindVertexArray(0);
        prog.disable(ctx);
    }
}
