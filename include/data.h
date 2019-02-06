#pragma once

#include <vector>
#include <memory>

#include "types.h"

namespace ellipsoid_trajectory {

struct input_data
{
    // axes of the ellipsoids
    // index corresponds with particle
    std::vector<vec3> axes;

    // trajectory data as data per particle and per time step (inner vector)
    std::vector<std::vector<vec3>> positions;    // x,y,z-coordinate
    std::vector<std::vector<vec4>> orientations; // quaternion with order: i,j,k, real part
    
    // physical time information of each time step
    // index corresponds with time step
    std::vector<float> times;
};

// data per time step
struct trajectory_data
{
    Bounding_Box b_box;                   // bounding box of trajectory

    std::vector<vec3> positions;          // position at each time step
    std::vector<vec3> velocities;         // velocity at each time step
    std::vector<vec3> angular_velocities; // velocity at each time step
    std::vector<vec4> orientations;       // orientation at each time step (order: i, j, k, real part)
    std::vector<vec3> main_axis_normals;  // normal vector for main axis (axes[0])

    // vertex index if all data is send to GPU at once
    std::vector<unsigned int> indices_strip;  // lines strip: 1-2-3-4
    std::vector<unsigned int> indices;        // line: 1-2, 2-3, 3-4
};

struct dynamic_particle_data
{
    // index corresponds with particle
    std::vector<size_t> axis_ids;         // stores id of axis of particle in axes vector
    std::vector<std::shared_ptr<trajectory_data>> trajs;  // trajectory data for each particle

    // index corresponds with time step
    std::vector<float> times;            // physical time of each time step
};


// index of vectors always correspond with particle
struct stationary_particle_data
{
    std::vector<size_t> axis_ids;   // stores id of axis of particle in axes vector
    std::vector<vec3> positions;    // position of each particle
    std::vector<vec4> orientations; // orientation of each particle (order: i, j, k, real part)
};


template<typename T>
std::istream& binary_read(std::istream& stream, T& value){
    return stream.read(reinterpret_cast<char*>(&value), sizeof(T));
}

class data 
{
public:
    data();

    // loads data from given list of files from start to end index with given resolution
    bool load(std::vector<std::pair<double, std::string>>& files, int start, int end, int time_resolution = 1, bool cut = false, bool same_start = true, bool create_equidistant = false, float tolerance = 0.90);
    // randomly generates trajectories with varying direction, rotation and velocity
    bool generate_random(size_t _number_particles = 10000, size_t _time_steps = 1000, float start_velocity = 0.0f, int seed = 0, bool cut_trajs = false, bool same_start = false);
    // generates start screen sample trajectories
    bool generate_sample();

    Bounding_Box b_box;
    size_t max_time_steps;

    // stationary particles
    stationary_particle_data stationaries;

    // stores trajectory of each particle
    dynamic_particle_data dynamics;

    // stores different ellipsoid axes
    std::vector<vec3> axes;

private:
    input_data tmp_data;

    bool read_files(std::string directory_name);
    // read Fortran binary file of given time step
    bool read_f90_file(std::string file_name, size_t t, size_t number_particles);
    
    // transfers tmp_data to data storage used for visualization
    void post_process(bool cut, bool same_start, bool create_equidistant, float tolerance);
    
    // updates bounding box measures
    void compute_data_bounding_box();
};

}
