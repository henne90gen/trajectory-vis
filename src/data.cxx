#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <limits>
#include <random>

#include "data.h"
#include "math_utils.h"

namespace ellipsoid_trajectory {

data::data()
{
    max_time_steps = 0;

    b_box.min = vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    b_box.max = vec3(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
}

bool data::load(std::vector<std::pair<double, std::string>>& files, int start, int end, int time_resolution , bool cut, bool same_start, bool create_equidistant, float tolerance)
{
    std::cout << "start reading files ... " << std::endl;

    // parse number of particles of one file
    uint32_t record_length;
    uint32_t s_number_particles;
    std::ifstream file (files[0].second, std::ios::in | std::ios::binary);
    file.seekg(0, std::ios::beg);
    binary_read(file, record_length);
    binary_read(file, s_number_particles);
    file.close();

    // set overall number of time steps and particles
    max_time_steps = ceil((end - (start - 1)) / (time_resolution * 1.0f));
    size_t number_particles = (size_t) s_number_particles;

    // reserve size of all vectors
    std::cout << "  .. reserve memory" << std::endl;
    tmp_data.axes.resize(number_particles);
    tmp_data.positions.resize(number_particles);
    tmp_data.orientations.resize(number_particles);
    tmp_data.times.resize(max_time_steps);

    // fill inner vectors
    for (size_t p = 0; p < number_particles; p++) {
        std::vector<vec3> pos;
        pos.resize(max_time_steps);
        tmp_data.positions[p] = pos;

        std::vector<vec4> orientation;
        orientation.resize(max_time_steps);
        tmp_data.orientations[p] = orientation;
    }

    std::cout << "  .. parse files" << std::endl;

    bool success = true;
    size_t index = 0;
    // parse data of each file
    for (int i = start - 1; i < end; i++) {
        // just read every x-th time step
        if (i % time_resolution == 0) {
            std::string file_name = files[i].second;
            std::cout << "     " << file_name << std::endl;

            // read data for all particles of current time step from file
            success = read_f90_file(file_name, index, number_particles);
            index++;
        }
    }

    if (success) {
        // fill stationary and trajectories vectors
        post_process(cut, same_start, create_equidistant, tolerance);
        return true;
    } else {
        std::cerr << "ERROR: reading data from directory" << std::endl;
        return false;
    }
}

void data::post_process(bool cut, bool same_start, bool create_equidistant, float tolerance)
{
    std::cout << "post processing of data ... " << std::endl;

    // for(int i = 1; i < tmp_data.times.size(); i++) {
    //     if (tmp_data.times[i-1] > tmp_data.times[i])
    //         std::cerr << "time error" << std::endl;
    // }

    // 1. store axes of ellipsoids in a grouped way
    dynamics.axis_ids.reserve(tmp_data.positions.size());
    for (size_t p = 0; p < tmp_data.axes.size(); p++) {
        bool axis_exists = false;
        size_t id;

        // check if axis exists
        for (id = 0; id < axes.size(); id++) {
            if (axes[id] == tmp_data.axes[p]) {
                axis_exists = true;
                break;
            }
        }

        // add current axis if not already in axes vector
        if (!axis_exists) {
            axes.push_back(tmp_data.axes[p]);
        }

        // store id of current axis in axes vector
        dynamics.axis_ids.push_back(id);
    }


    // 2. remove "trajectories" of stationary particles
    std::cout << "  .. remove stationary particles from trajectories" << std::endl;
    int write_ptr = 0;

    for (size_t p = 0; p < tmp_data.positions.size(); p++) {
        bool stationary = true;

        // go through position at each time step to detect stationary particles
        for (size_t t = 1; t < tmp_data.positions[p].size(); t++) {
            // if position changed at any time the particle is not stationary
            if (tmp_data.positions[p][t-1] != tmp_data.positions[p][t]) {
                stationary = false;
                break;
            }
        }

        if (stationary) {
            // add particle to stationary vector
            stationaries.axis_ids.push_back(dynamics.axis_ids[p]);
            stationaries.positions.push_back(tmp_data.positions[p][0]);
            stationaries.orientations.push_back(tmp_data.orientations[p][0]);
        } else {
            // update vectors of dynamic particles
            // this will overwrite the data of stationary particles and therefore delete them
            // from the dynamics vector
            dynamics.axis_ids[write_ptr] = dynamics.axis_ids[p];
            // for the trajs entry it is usefull to have ptr otherwise all data will be copied
            tmp_data.positions[write_ptr] = tmp_data.positions[p];
            tmp_data.orientations[write_ptr] = tmp_data.orientations[p];

            write_ptr++;
        }
    }

    // resize all vectors of dynamic particles
    // everything after write_ptr index has to be deleted
    dynamics.axis_ids.resize(write_ptr);
    // the shared_ptr of the trajs-vector copes with the deletion of all the vectors inside
    tmp_data.positions.resize(write_ptr);
    tmp_data.orientations.resize(write_ptr);


    // 3. set global bounding box
    std::cout << "  .. compute bounding box of data set" << std::endl;
    for (size_t p = 0; p < tmp_data.positions.size(); p++) {
        for (size_t t = 0; t < tmp_data.times.size(); t++) {
            // global bounding box
            if (tmp_data.positions[p][t][0] < b_box.min[0])
                b_box.min[0] = tmp_data.positions[p][t][0];
            if (tmp_data.positions[p][t][1] < b_box.min[1])
                b_box.min[1] = tmp_data.positions[p][t][1];
            if (tmp_data.positions[p][t][2] < b_box.min[2])
                b_box.min[2] = tmp_data.positions[p][t][2];
            if (tmp_data.positions[p][t][0] > b_box.max[0])
                b_box.max[0] = tmp_data.positions[p][t][0];
            if (tmp_data.positions[p][t][1] > b_box.max[1])
                b_box.max[1] = tmp_data.positions[p][t][1];
            if (tmp_data.positions[p][t][2] > b_box.max[2])
                b_box.max[2] = tmp_data.positions[p][t][2];
        }
    }

    // compute center of bounding box
    vec3 diff = b_box.max - b_box.min;
    b_box.center = vec3(b_box.min + (diff / 2));


    // 4. handling of trajectories that moved out of bound (just x and z axis)
    // and therefore entered the scene at the opposing side again.
    // these trajectories will be cut
    if (cut)
        std::cout << "  .. split out of bound trajectories" << std::endl;
    else
        std::cout << "  .. continue out of bound trajectories" << std::endl;

    // threshold for detection cuts
    vec3 b_box_dimensions = (b_box.max - b_box.min);

    // go through all stored trajectories (here one trajectory for each particle)
    for (size_t p = 0; p < tmp_data.positions.size(); p++) {
        // either cut trajectory and create new one or update position data to continue
        // outside of the boudning box
        if (cut) {
            // store indices where cuts of trajectories appear
            std::vector<size_t> cuts;

            // go through each time step of trajectory
            for (size_t t = 1; t < tmp_data.positions[p].size(); t++) {
                // if position changed by bounding box measures the particle moved out of bound
                vec3 diff = tmp_data.positions[p][t]
                             - tmp_data.positions[p][t-1];
                if (abs(diff[0]) > (tolerance * b_box_dimensions[0])
                        || abs(diff[2]) > (tolerance * b_box_dimensions[2])) {
                    cuts.push_back(t);
                }
            }

            // create new trajectories for each cut
            for (size_t c = 0; c < cuts.size(); c++) {
                // size of new trajectory
                size_t size = 0;
                if (c == (cuts.size() - 1)) {
                    size = tmp_data.positions[p].size() - cuts[c];
                } else {
                    size = cuts[c+1] - cuts[c];
                }

                // create new trajectory data
                // the trajectory has the same size as the original once
                // all invalid positions are filled with NAN
                std::vector<vec3> new_positions;
                std::vector<vec4> new_orientations;
                new_positions.resize(max_time_steps, NAN);
                new_orientations.resize(max_time_steps, NAN);

                std::copy(tmp_data.positions[p].begin() + cuts[c],
                          tmp_data.positions[p].begin() + cuts[c] + size,
                          new_positions.begin() + cuts[c]);
                std::copy(tmp_data.orientations[p].begin() + cuts[c],
                          tmp_data.orientations[p].begin() + cuts[c] + size,
                          new_orientations.begin() + cuts[c]);

                assert(new_positions.size() == max_time_steps
                    && new_orientations.size() == max_time_steps
                    && "Newly added trajectories have to be of same size than original once");

                tmp_data.positions.push_back(new_positions);
                tmp_data.orientations.push_back(new_orientations);

                dynamics.axis_ids.push_back(dynamics.axis_ids[p]);
            }

            // fill current trajectory vector with invalid values
            if (cuts.size() > 0) {
                std::fill(tmp_data.positions[p].begin() + cuts[0], tmp_data.positions[p].end(), vec3(NAN, NAN, NAN));
                std::fill(tmp_data.orientations[p].begin() + cuts[0], tmp_data.orientations[p].end(), vec4(NAN, NAN, NAN, NAN));
            }
        } else {
            // go through each time step of trajectory
            for (size_t t = 1; t < tmp_data.positions[p].size(); t++) {
                // if position changed by bounding box measures the particle moved out of bound
                vec3 diff = tmp_data.positions[p][t]
                            - tmp_data.positions[p][t-1];

                if (abs(diff[0]) > (tolerance * b_box_dimensions[0])) {
                    int multiple_box = (diff[0]) / (tolerance * b_box_dimensions[0]);
                    tmp_data.positions[p][t] -= vec3(b_box_dimensions[0] * multiple_box, 0.0f, 0.0f);

                }

                if (abs(diff[2]) > (tolerance * b_box_dimensions[2])){
                    int multiple_box = (diff[2]) / (tolerance * b_box_dimensions[2]);
                    tmp_data.positions[p][t] -= vec3(0.0f, 0.0f, b_box_dimensions[2] * multiple_box);
                }
            }
        }
    }


    // 5. create data points that are equidistant in time
    if (create_equidistant)
        std::cout << "  .. create data points equidistant in time" << std::endl;
    float time_span = tmp_data.times[max_time_steps - 1] - tmp_data.times[0];
    float time_tick = time_span / ((float)max_time_steps - 1.0f);

    // reserve space for final data structure
    // and create initial entry
    dynamics.times.resize(max_time_steps);
    dynamics.times[0] = tmp_data.times[0];
    dynamics.times[max_time_steps - 1] = tmp_data.times[max_time_steps - 1];
    
    dynamics.trajs.resize(tmp_data.positions.size());
    for (size_t p = 0; p < tmp_data.positions.size(); p++) {
        std::shared_ptr<trajectory_data> traj(new trajectory_data());
        traj->positions.resize(max_time_steps);
        traj->positions[0] = tmp_data.positions[p][0];
        traj->positions[max_time_steps - 1] = tmp_data.positions[p][max_time_steps - 1];
        traj->orientations.resize(max_time_steps);
        traj->orientations[0] = tmp_data.orientations[p][0];
        traj->orientations[max_time_steps - 1] = tmp_data.orientations[p][max_time_steps - 1];
        dynamics.trajs[p] = traj;
    }

    float time = tmp_data.times[0];
    size_t prev = 0;
    size_t next = 0;
    for (size_t t = 1; t < max_time_steps - 1; t++) {
        time += time_tick;

        if (create_equidistant) {
            // create equidistant points in time
            dynamics.times[t] = time;

            while (tmp_data.times[prev + 1] < time) {
                prev++;
            }
            next = prev + 1;

            float diff = tmp_data.times[next] - tmp_data.times[prev];
            // |-------------|------|
            // prev          t      next
            // =============== ratio
            float ratio = (time - tmp_data.times[prev]) / diff;

            for (size_t p = 0; p < tmp_data.positions.size(); p++) {
                dynamics.trajs[p]->positions[t] = (1.0f - ratio) * tmp_data.positions[p][prev]
                                                + ratio * tmp_data.positions[p][next];
                dynamics.trajs[p]->orientations[t] = slerp(tmp_data.orientations[p][prev], tmp_data.orientations[p][next], ratio);
            }
        } else {
            // just copy the data
            dynamics.times[t] = tmp_data.times[t];

            for (size_t p = 0; p < tmp_data.positions.size(); p++) {
                dynamics.trajs[p]->positions[t] = tmp_data.positions[p][t];
                dynamics.trajs[p]->orientations[t] = tmp_data.orientations[p][t];
            }
        }
    }

    // delete tmp data structure
    tmp_data.axes.resize(0);
    tmp_data.times.resize(0);
    for (size_t p = 0; p < tmp_data.positions.size(); p++) {
        tmp_data.positions[p].resize(0);
        tmp_data.orientations[p].resize(0);
    }
    tmp_data.positions.resize(0);
    tmp_data.orientations.resize(0);



    // modify position to let every trajectory start at origin
    if (same_start) {
        // reset boudning box
        b_box.min = vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        b_box.max = vec3(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

        for (size_t p = 0; p < dynamics.trajs.size(); p++) {
            vec3 offset = dynamics.trajs[p]->positions[0];
            for (size_t t = 0; t < dynamics.trajs[p]->positions.size(); t++) {
                dynamics.trajs[p]->positions[t] -= offset;
            }
        }

        // update global bounding box
        compute_data_bounding_box();
    }


    // 6. generate idices for transfering all vertex data to the gpu at once and changing
    // indices for indexed rendering
    std::cout << "  .. generate indices for vertex data" << std::endl;
    unsigned int i = 0; 
    for (size_t p = 0; p < dynamics.trajs.size(); p++) {
        dynamics.trajs[p]->indices_strip.reserve(dynamics.trajs[p]->positions.size());
        dynamics.trajs[p]->indices.reserve(dynamics.trajs[p]->positions.size());
        for (size_t t = 0; t < dynamics.trajs[p]->positions.size(); t++) {
            dynamics.trajs[p]->indices_strip.push_back(i);

            if (t < dynamics.trajs[p]->positions.size() - 1) {
                dynamics.trajs[p]->indices.push_back(i);
                dynamics.trajs[p]->indices.push_back(i + 1);
            } else {
                dynamics.trajs[p]->indices.push_back(i);
                dynamics.trajs[p]->indices.push_back(i);
            }
            
            i++;
        }

        assert(dynamics.trajs[p]->positions.size() == dynamics.trajs[p]->orientations.size()
           && dynamics.trajs[p]->positions.size() == dynamics.trajs[p]->indices_strip.size()
           && "Each data vector for a trajectory has to be of same size");
    }


    // 7. compute bounding box of each trajectory
    std::cout << "  .. compute bounding box of each trajectories" << std::endl;
    for (size_t p = 0; p < dynamics.trajs.size(); p++) {
        dynamics.trajs[p]->b_box.min = vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        dynamics.trajs[p]->b_box.max = vec3(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

        for (size_t t = 0; t < dynamics.trajs[p]->positions.size(); t++) {
            // check for trajectory bounding box
            if (dynamics.trajs[p]->positions[t][0] < dynamics.trajs[p]->b_box.min[0])
                dynamics.trajs[p]->b_box.min[0] = dynamics.trajs[p]->positions[t][0];
            if (dynamics.trajs[p]->positions[t][1] < dynamics.trajs[p]->b_box.min[1])
                dynamics.trajs[p]->b_box.min[1] = dynamics.trajs[p]->positions[t][1];
            if (dynamics.trajs[p]->positions[t][2] < dynamics.trajs[p]->b_box.min[2])
                dynamics.trajs[p]->b_box.min[2] = dynamics.trajs[p]->positions[t][2];
            if (dynamics.trajs[p]->positions[t][0] > dynamics.trajs[p]->b_box.max[0])
                dynamics.trajs[p]->b_box.max[0] = dynamics.trajs[p]->positions[t][0];
            if (dynamics.trajs[p]->positions[t][1] > dynamics.trajs[p]->b_box.max[1])
                dynamics.trajs[p]->b_box.max[1] = dynamics.trajs[p]->positions[t][1];
            if (dynamics.trajs[p]->positions[t][2] > dynamics.trajs[p]->b_box.max[2])
                dynamics.trajs[p]->b_box.max[2] = dynamics.trajs[p]->positions[t][2];
        }

        vec3 traj_diff = dynamics.trajs[p]->b_box.max - dynamics.trajs[p]->b_box.min;
        dynamics.trajs[p]->b_box.center = vec3(dynamics.trajs[p]->b_box.min + (traj_diff / 2));
    }


    // 8. computing linear and angular velocities
    std::cout << "  .. compute linear and angular velocities" << std::endl;
    for (size_t p = 0; p < dynamics.trajs.size(); p++) {
        dynamics.trajs[p]->angular_velocities.resize(dynamics.trajs[p]->orientations.size());
        dynamics.trajs[p]->velocities.resize(dynamics.trajs[p]->orientations.size());
        for (size_t t = 0; t < dynamics.trajs[p]->orientations.size() - 1; t++) {
            // compute angular velocity between current and next time step
            // quaternion that q * q0 = q1 --> q = q1 * conj(q0) (for unit length quaternions)
            vec4 conj_q0 = vec4(-1 * dynamics.trajs[p]->orientations[t][0],
                                -1 * dynamics.trajs[p]->orientations[t][1],
                                -1 * dynamics.trajs[p]->orientations[t][2],
                                dynamics.trajs[p]->orientations[t][3]);
            vec4 quat = quat_mul(dynamics.trajs[p]->orientations[t + 1], conj_q0);
            
            // convert quaternion to axis and angle in radians
            double len = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2]);
            double angle = 2.0f * atan2(len, quat[3]);
            vec3 axis;
            if (len > 0)
                axis = vec3(quat[0], quat[1], quat[2]) / len;
            else
                axis = vec3(1,0,0);

            // rotation velocity
            vec3 w = axis * angle / 1.0f;

            dynamics.trajs[p]->angular_velocities[t] = w;
            dynamics.trajs[p]->velocities[t] = dynamics.trajs[p]->positions[t + 1] - 
                                               dynamics.trajs[p]->positions[t];
        }
        dynamics.trajs[p]->angular_velocities[dynamics.trajs[p]->orientations.size() - 1] = vec3(0.0f, 0.0f, 0.0f);
        dynamics.trajs[p]->velocities[dynamics.trajs[p]->orientations.size() - 1] = vec3(0.0f, 0.0f, 0.0f);
    }


    // 9. computing normal: crossproduct of axis and velocity
    std::cout << "  .. compute normals for each time step" << std::endl;
    for (size_t p = 0; p < dynamics.trajs.size(); p++) {
        dynamics.trajs[p]->main_axis_normals.resize(dynamics.trajs[p]->orientations.size());
        for (size_t t = 0; t < dynamics.trajs[p]->orientations.size() - 1; t++) {
            // compute orientated main axis (assumes longest axis at position 0)
            // TODO: compute for all axis
            vec3 a = quat_rotate(vec3(axes[dynamics.axis_ids[p]][0], 0.0f, 0.0f), dynamics.trajs[p]->orientations[t]);
            vec3 v = dynamics.trajs[p]->velocities[t];
            vec3 n = normalize(cross(v, a));

            dynamics.trajs[p]->main_axis_normals[t] = n;
        }
        dynamics.trajs[p]->main_axis_normals[dynamics.trajs[p]->orientations.size() - 1] = vec3(0.0f, 0.0f, 0.0f);
        // std::cout << dynamics.trajs[p]->main_axis_normals.size() << std::endl;
    }


    // check vector sizes
    assert(dynamics.axis_ids.size() == dynamics.trajs.size()
           && "Each data vector for a particle has to be of same size");

    std::cout << "Data Stats: " << std::endl;
    std::cout << "  number of stationaries particles: " << stationaries.axis_ids.size() << std::endl;
    std::cout << "  number of dynamic particles (trajectories): " << dynamics.axis_ids.size() << " of " << dynamics.trajs.size() - stationaries.axis_ids.size() << " original particles" << std::endl;
}

void data::compute_data_bounding_box()
{
    for (size_t p = 0; p < dynamics.trajs.size(); p++) {
        for (size_t t = 0; t < dynamics.trajs[p]->positions.size(); t++) {
            // global bounding box
            if (dynamics.trajs[p]->positions[t][0] < b_box.min[0])
                b_box.min[0] = dynamics.trajs[p]->positions[t][0];
            if (dynamics.trajs[p]->positions[t][1] < b_box.min[1])
                b_box.min[1] = dynamics.trajs[p]->positions[t][1];
            if (dynamics.trajs[p]->positions[t][2] < b_box.min[2])
                b_box.min[2] = dynamics.trajs[p]->positions[t][2];
            if (dynamics.trajs[p]->positions[t][0] > b_box.max[0])
                b_box.max[0] = dynamics.trajs[p]->positions[t][0];
            if (dynamics.trajs[p]->positions[t][1] > b_box.max[1])
                b_box.max[1] = dynamics.trajs[p]->positions[t][1];
            if (dynamics.trajs[p]->positions[t][2] > b_box.max[2])
                b_box.max[2] = dynamics.trajs[p]->positions[t][2];
        }
    }

    // compute center of bounding box
    vec3 diff = b_box.max - b_box.min;
    b_box.center = vec3(b_box.min + (diff / 2));
}

bool data::read_f90_file(std::string file_name, size_t t, size_t number_particles)
{
    // unformatted Fortran binary files are not flat
    // they store the length of the written record before and after each write
    uint32_t record_length;

    /*
        binary file layout:

        # number of particles in file (int32)
        0400 0000  # record length at begin of record
            db53 0000 --> 21467
        0400 0000  # record length at end of record

        # physical time (double)
        0800 0000
            cff9 12c0 1bc7 9840
        0800 0000

        # x position vector of particle (double[3])
        1800 0000
            0000 0000 0000 e03f -->  0.5
            0b24 287e 8cb9 cb3f -->  0.2166
            0000 0000 0000 f03f -->  1.0
        1800 0000

        # u velocity vector of particle (double[3])
        1800 0000
            0000 0000 0000 0000
            0000 0000 0000 0000
            0000 0000 0000 0000
        1800 0000

        # quaternion (double[4])
        2000 0000
            0000 0000 0000 f03f
            0000 0000 0000 0000
            0000 0000 0000 0000
            0000 0000 0000 0000
        2000 0000

        # a longest axis (double)
        0800 0000
            0000 0000 0000 e03f
        0800 0000

        # b intermediate axis (double)
        0800 0000
            0000 0000 0000 e03f
        0800 0000

        # c smalles axis (double)
        0800 0000
            0000 0000 0000 e03f
        0800 0000

        # nl number of lagrangian points  used to represent particle i (int32)
        0400 0000
            4606 0000
        0400 0000
    */

    std::ifstream file (file_name, std::ios::in | std::ios::binary);

    if (!file.is_open()) {
        return false;
    }

    file.seekg(0, std::ios::end);
    std::streampos end_position = file.tellg();

    // set position to start of file
    file.seekg(0, std::ios::beg);

    // read number of particles
    uint32_t s_number_particles;
    binary_read(file, record_length);
    // std::cout << "record length: " << record_length << std::endl;
    binary_read(file, s_number_particles);
    // std::cout << "   numer_particles: " << s_number_particles << std::endl;
    binary_read(file, record_length);

    // read time
    double s_time;
    binary_read(file, record_length);
    // std::cout << "record length: " << record_length << std::endl;
    binary_read(file, s_time);
    // std::cout << "   time: " << s_time << std::endl;
    binary_read(file, record_length);

    // read data of each particle
    for (size_t i = 0; i < number_particles; i++) {
        double s_x1, s_x2, s_x3;
        double s_u, s_v, s_w;
        double s_q0, s_q1, s_q2, s_q3;
        double s_a;
        double s_b;
        double s_c;
        int32_t _number_lagrangian;
// #pragma omp critical
// {
        // std::cout << "particle_id: " << i << std::endl;
        // position
        binary_read(file, record_length);
        // std::cout << "  record length: " << record_length << std::endl;
        binary_read(file, s_x1);
        binary_read(file, s_x2);
        binary_read(file, s_x3);
        // std::cout << "   x1: " << s_x1 << std::endl;
        // std::cout << "   x2: " << s_x2 << std::endl;
        // std::cout << "   x3: " << s_x3 << std::endl;
        binary_read(file, record_length);

        // velocity
        binary_read(file, record_length);
        // std::cout << "  record length: " << record_length << std::endl;
        binary_read(file, s_u);
        binary_read(file, s_v);
        binary_read(file, s_w);
        // std::cout << "   u: " << s_u << std::endl;
        // std::cout << "   v: " << s_v << std::endl;
        // std::cout << "   w: " << s_w << std::endl;
        binary_read(file, record_length);

        // orientation in quaternions
        binary_read(file, record_length);
        // std::cout << "  record length: " << record_length << std::endl;
        binary_read(file, s_q0);
        binary_read(file, s_q1);
        binary_read(file, s_q2);
        binary_read(file, s_q3);
        // std::cout << "   q0: " << s_q0 << std::endl;
        // std::cout << "   q1: " << s_q1 << std::endl;
        // std::cout << "   q2: " << s_q2 << std::endl;
        // std::cout << "   q3: " << s_q3 << std::endl;
        binary_read(file, record_length);

        // long axis of particle
        binary_read(file, record_length);
        // std::cout << "  record length: " << record_length << std::endl;
        binary_read(file, s_a);
        // std::cout << "   a: " << s_a << std::endl;
        binary_read(file, record_length);

        // medium axis of particle
        binary_read(file, record_length);
        // std::cout << "  record length: " << record_length << std::endl;
        binary_read(file, s_b);
        // std::cout << "   b: " << s_b << std::endl;
        binary_read(file, record_length);

        // smallest axis of particle
        binary_read(file, record_length);
        // std::cout << "  record length: " << record_length << std::endl;
        binary_read(file, s_c);
        // std::cout << "   c: " << s_c << std::endl;
        binary_read(file, record_length);

        //  number of lagrangian points to used to represent particle
        binary_read(file, record_length);
        // std::cout << "  record length: " << record_length << std::endl;
        binary_read(file, _number_lagrangian);
        // std::cout << "   number_lagrangian: " << _number_lagrangian << std::endl;
        binary_read(file, record_length);

        // store this data only once
        if (t == 0) {
            tmp_data.axes[i] = vec3((float)s_a, (float)s_b, (float)s_c);
        }

        tmp_data.positions[i][t] = vec3((float)s_x1,
                                        (float)s_x2,
                                        (float)s_x3);
        tmp_data.orientations[i][t] = vec4((float)s_q0,
                                           (float)s_q1,
                                           (float)s_q2,
                                           (float)s_q3);
    
        // store physical time only for first particle (same for all other)
        if (i == 0)
            tmp_data.times[t] = (float)s_time;
    }
    

    // check if everything was read
    if (!(file.tellg() == end_position)) {
        std::cerr << "ERROR: file could not be read completely" << std::endl;
        return false;
    }

    file.close();

    return true;
}

bool data::generate_random(size_t _number_particles, size_t _time_steps, float start_velocity, int seed, bool cut_trajs, bool same_start)
{
    // parameter
    Bounding_Box box;
    box.min = vec3(0.0f, 0.0f, 0.0f);
    box.max = vec3(100.0f, 100.0f, 100.0f);
    box.center = vec3(box.min + ((box.max - box.min) / 2));
    float step_width = 0.001;

    std::vector<vec3> axes;
    axes.push_back(vec3(0.75, 0.5, 0.33));
    axes.push_back(vec3(0.5, 0.5, 0.5));
    axes.push_back(vec3(1.5, 0.5, 0.5));
    axes.push_back(vec3(0.75, 0.75, 0.33));

    // random generator using Mersenne Twister
    std::random_device rd{};
    int _seed;
    if (seed != 0) {
        _seed = seed;
    }
    else {
        // use abs to be able to set the seed through ui
        // since the input field does not accept negative values (?)
        _seed = abs((int)rd());
    }

    std::mt19937 mt_rand(_seed);
    std::normal_distribution<> gaussian_quat{0, M_PI / 10.0f};
 
    std::cout << "Start generating data ..." << _time_steps << std::endl;
    max_time_steps = _time_steps;
    
    std::cout << "  with seed: " << _seed << std::endl;
    std::cout << "  .. reserve memory" << std::endl;
    tmp_data.axes.resize(_number_particles);
    tmp_data.positions.resize(_number_particles);
    tmp_data.orientations.resize(_number_particles);
    tmp_data.times.resize(max_time_steps);

    for (size_t p = 0; p < _number_particles; p++) {
        std::vector<vec3> pos;
        pos.resize(max_time_steps);
        tmp_data.positions[p] = pos;

        std::vector<vec4> orientation;
        orientation.resize(max_time_steps);
        tmp_data.orientations[p] = orientation;
    }

    std::cout << "  .. generate axes, positions and orientations" << std::endl;

    // for each particle
    for (size_t p = 0; p < _number_particles; p++) {
        // axes of ellipsoid particle
        tmp_data.axes[p] = axes[p % 4];

        // set random start position inside given bounding box
        vec3 prev_position = vec3(mt_rand() % (int)box.max[0] + (int)box.min[0],
                                  mt_rand() % (int)box.max[1] + (int)box.min[1],
                                  mt_rand() % (int)box.max[2] + (int)box.min[2]);
        vec3 prev_direction = vec3(start_velocity, 0.0f, 0.0f);

        vec3 prev_euler_angle = vec3(((mt_rand() % 10) / 100.0f) * 2 * M_PI,
                                     ((mt_rand() % 10) / 100.0f) * 2 * M_PI,
                                     ((mt_rand() % 10) / 100.0f) * 2 * M_PI);

        vec3 angle;
        angle = vec3(gaussian_quat(mt_rand),
                     gaussian_quat(mt_rand),
                     gaussian_quat(mt_rand));

        // for each time step of particle's trajectory
        for (size_t t = 0; t < max_time_steps; t++) {
            tmp_data.times[t] = (float)t;
            // create normal distribution based on previous direction
            // most likely that the direction won't change (mean of gaussian is zero)
            // standard deviation affects the dispersion of generated values from the mean
            std::normal_distribution<> gaussian_x{0, step_width};
            std::normal_distribution<> gaussian_y{0, step_width};
            std::normal_distribution<> gaussian_z{0, step_width};

            vec3 direction = prev_direction + vec3(gaussian_x(mt_rand),
                                                   gaussian_y(mt_rand),
                                                   gaussian_z(mt_rand));
            vec3 position = prev_position + direction;

            tmp_data.positions[p][t] = position;
            prev_position = position;
            prev_direction = direction;

            // compute new orientation
            vec3 euler_angle = prev_euler_angle;
            // prevent to much flickering
            if ((t % 60) == 0) {
                angle = angle + vec3(gaussian_quat(mt_rand),
                                     gaussian_quat(mt_rand),
                                     gaussian_quat(mt_rand));
            }

            euler_angle = euler_angle + angle * 1 / 60.0f;
            
            vec4 orientation = normalize(to_quat(euler_angle[0], euler_angle[1], euler_angle[2]));

            tmp_data.orientations[p][t] = orientation;
            prev_euler_angle = euler_angle;
        }
    }

    post_process(cut_trajs, same_start, false, 0.90);
    return true;
}

bool data::generate_sample()
{
    // parameter
    Bounding_Box box;
    box.min = vec3(0.0f, 0.0f, 0.0f);
    box.max = vec3(50.0f, 10.0f, 20.0f);
    box.center = vec3(box.min + ((box.max - box.min) / 2));
    vec3 axes = vec3(0.75, 0.5, 0.33);

    // random generator using Mersenne Twister
    std::random_device rd{};
    std::mt19937 mt_rand{rd()};
 
    std::cout << "Generating sample data ..." << std::endl;
    size_t _number_particles = 7;
    max_time_steps = 1000;
    
    tmp_data.axes.resize(_number_particles);
    tmp_data.positions.resize(_number_particles);
    tmp_data.orientations.resize(_number_particles);
    tmp_data.times.resize(max_time_steps);

    for (size_t p = 0; p < _number_particles; p++) {
        std::vector<vec3> pos;
        pos.resize(max_time_steps);
        tmp_data.positions[p] = pos;

        std::vector<vec4> orientation;
        orientation.resize(max_time_steps);
        tmp_data.orientations[p] = orientation;
    }

    // 1. straight trajectory
    tmp_data.axes[0] = axes;
    tmp_data.axes[1] = axes;
    tmp_data.axes[2] = axes;
    tmp_data.axes[3] = axes;
    vec3 prev_position = vec3(1.0f, 1.0f, 1.0f);
    for (size_t t = 0; t < max_time_steps; t++) {
        vec3 position = prev_position + vec3(0.01f, 0.0f, 0.0f);
        tmp_data.positions[0][t] = position;
        prev_position = position;
        
        tmp_data.orientations[0][t] = vec4(0.0f, 0.0f, 0.0f, 1.0f);
        tmp_data.times[t] = (float)t;
    }
    prev_position = vec3(1.0f, 1.0f, 1.0f);
    for (size_t t = 0; t < max_time_steps; t++) {
        vec3 position = prev_position + vec3(0.0f, 0.01f, 0.0f);
        tmp_data.positions[1][t] = position;
        prev_position = position;
        
        tmp_data.orientations[1][t] = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    }
    prev_position = vec3(1.0f, 1.0f, 1.0f);
    for (size_t t = 0; t < max_time_steps; t++) {
        vec3 position = prev_position + vec3(0.0f, 0.00f, 0.01f);
        tmp_data.positions[2][t] = position;
        prev_position = position;
        
        tmp_data.orientations[2][t] = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    }
    prev_position = vec3(1.0f, 1.0f, 1.0f);
    for (size_t t = 0; t < max_time_steps; t++) {
        vec3 position = prev_position + vec3(0.01f, 0.01f, 0.01f);
        tmp_data.positions[3][t] = position;
        prev_position = position;
        
        tmp_data.orientations[3][t] = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // 1. rotating trajectory
    tmp_data.axes[4] = axes;
    tmp_data.axes[5] = axes;
    tmp_data.axes[6] = axes;
    prev_position = vec3(1.0f, 1.0f, 1.0f);
    vec3 prev_euler_angle = vec3(0.0f, 0.0f, 0.0f);
    for (size_t t = 0; t < max_time_steps; t++) {
        vec3 position = prev_position + vec3(0.01f, 0.01f, 0.0f);
        tmp_data.positions[4][t] = position;
        prev_position = position;

        vec3 euler_angle = prev_euler_angle + vec3(M_PI / 200.0f, 0.0f, 0.0f);
        
        tmp_data.orientations[4][t] = normalize(to_quat(euler_angle[0],
                                                               euler_angle[1],
                                                               euler_angle[2]));
        prev_euler_angle = euler_angle;
    }
    prev_position = vec3(1.0f, 1.0f, 1.0f);
    prev_euler_angle = vec3(0.0f, 0.0f, 0.0f);
    for (size_t t = 0; t < max_time_steps; t++) {
        vec3 position = prev_position + vec3(0.0f, 0.01f, 0.01f);
        tmp_data.positions[5][t] = position;
        prev_position = position;
        
        vec3 euler_angle = prev_euler_angle + vec3(0.0f, M_PI / 200.0f, 0.0f);
        
        tmp_data.orientations[5][t] = normalize(to_quat(euler_angle[0],
                                                               euler_angle[1],
                                                               euler_angle[2]));
        prev_euler_angle = euler_angle;
    }
    prev_position = vec3(1.0f, 1.0f, 1.0f);
    prev_euler_angle = vec3(0.0f, 0.0f, 0.0f);
    for (size_t t = 0; t < max_time_steps; t++) {
        vec3 position = prev_position + vec3(0.01f, 0.0f, 0.01f);
        tmp_data.positions[6][t] = position;
        prev_position = position;
        
        vec3 euler_angle = prev_euler_angle + vec3(0.0f, 0.0f, M_PI / 200.0f);
        
        tmp_data.orientations[6][t] = normalize(to_quat(euler_angle[0],
                                                               euler_angle[1],
                                                               euler_angle[2]));
        prev_euler_angle = euler_angle;
    }


    post_process(false, false, false, 0.90);

    return true;
}
}
