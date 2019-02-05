#include "math_utils.h"

namespace ellipsoid_trajectory {
    vec3 quat_rotate(vec3 position, vec4 quat)
    {
        vec3 quat_xyz = vec3(quat[0],quat[1],quat[2]);
        vec3 a = cross(quat_xyz, position);
        a = a + (position * quat[3]);
        vec3 b = cross(quat_xyz, a);

        // v = position + 2.0 * cross(quat.xyz, cross(quat.xyz, position) + quat[3] * position)
        return position + (b * 2.0) ;
    }

    vec4 quat_normed(vec4 q)
    {
        vec4 q_norm;
        float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q_norm[0] =  q[0] / norm;
        q_norm[1] =  q[1] / norm;
        q_norm[2] =  q[2] / norm;
        q_norm[3] =  q[3] / norm;
        return q_norm;
    }

    vec4 slerp(vec4 qa, vec4 qb, double t) {
        // quaternion to return
        vec4 qm;
        // Calculate angle between them.
        double cosHalfTheta = qa[3] * qb[3] + qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2];
        // if qa=qb or qa=-qb then theta = 0 and we can return qa
        if (abs(cosHalfTheta) >= 1.0){
            qm[3] = qa[3];qm[0] = qa[0];qm[1] = qa[1];qm[2] = qa[2];
            return qm;
        }
        // Calculate temporary values.
        double halfTheta = acos(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
        // if theta = 180 degrees then result is not fully defined
        // we could rotate around any axis normal to qa or qb
        if (fabs(sinHalfTheta) < 0.001){ // fabs is floating point absolute
            qm[3] = (qa[3] * 0.5 + qb[3] * 0.5);
            qm[0] = (qa[0] * 0.5 + qb[0] * 0.5);
            qm[1] = (qa[1] * 0.5 + qb[1] * 0.5);
            qm[2] = (qa[2] * 0.5 + qb[2] * 0.5);
            return qm;
        }
        double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
        double ratioB = sin(t * halfTheta) / sinHalfTheta; 
        //calculate Quaternion.
        qm[3] = (qa[3] * ratioA + qb[3] * ratioB);
        qm[0] = (qa[0] * ratioA + qb[0] * ratioB);
        qm[1] = (qa[1] * ratioA + qb[1] * ratioB);
        qm[2] = (qa[2] * ratioA + qb[2] * ratioB);
        return qm;
    }

    // euler angle in radian to quaternion
    vec4 to_quat(double pitch, double roll, double yaw)
    {
        vec4 q;
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);

        q[0] = cy * sr * cp - sy * cr * sp;
        q[1] = cy * cr * sp + sy * sr * cp;
        q[2] = sy * cr * cp - cy * sr * sp;
        
        q[3] = cy * cr * cp + sy * sr * sp;
        return q;
    }

    vec4 quat_mul(vec4 q0, vec4 q1)
    {
      return vec4( 
       q0[3] * q1[0] + q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1],                          
       q0[3] * q1[1] + q0[1] * q1[3] + q0[2] * q1[0] - q0[0] * q1[2],
       q0[3] * q1[2] + q0[2] * q1[3] + q0[0] * q1[1] - q0[1] * q1[0],
       q0[3] * q1[3] - q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2]);
    }

    void create_ellipsoid_vertices(std::vector<vec4>& vertices, std::vector<vec4>& normals, std::vector<vec2>& texture_coord, vec3 axes, unsigned int stacks, unsigned int slices, vec4 center)
    {
        float t_step = (M_PI) / (float)slices;
        float s_step = (M_PI) / (float)stacks;

        for(float t = -M_PI / 2; t < M_PI / 2; t += t_step)
        {
            for(float s = -M_PI; s <= M_PI; s += s_step)
            {
                float x = cos(t) * cos(s);
                float y = cos(t) * sin(s);
                float z = sin(t);
                vertices.push_back(center + vec4(axes[0] * x, axes[1] * y, axes[2] * z, 1.0f));
                normals.push_back(center + vec4(x / axes[0], y / axes[1], z / axes[2], 0.0f));

                float texture_x = (t / (M_PI / 2)) / 2 + 0.5f;
                float texture_y = (s / M_PI) / 2 + 0.5f;
                texture_coord.push_back(vec2(texture_x, texture_y));

                x = cos(t + t_step) * cos(s);
                y = cos(t + t_step) * sin(s);
                z = sin(t + t_step);
                vertices.push_back(center + vec4(axes[0] * x, axes[1] * y, axes[2] * z, 1.0f));
                normals.push_back(center + vec4(x / axes[0], y / axes[1], z / axes[2], 0.0f));

                texture_x = ((t + t_step) / (M_PI / 2)) / 2 + 0.5f;
                texture_y = ((s) / M_PI) / 2 + 0.5f;
                texture_coord.push_back(vec2(texture_x, texture_y));
            }
        }
    }

    void create_box_vertices(std::vector<vec3>& vertices, std::vector<unsigned int>& indices, vec3 min, vec3 max, unsigned int restart_id)
    {
        vertices.push_back(vec3(min[0], min[1], max[2]));
        vertices.push_back(vec3(max[0], min[1], max[2]));
        vertices.push_back(vec3(max[0], min[1], min[2]));
        vertices.push_back(vec3(min[0], min[1], min[2]));
        vertices.push_back(vec3(min[0], max[1], max[2]));
        vertices.push_back(vec3(max[0], max[1], max[2]));
        vertices.push_back(vec3(max[0], max[1], min[2]));
        vertices.push_back(vec3(min[0], max[1], min[2]));

        indices.reserve(24);
        indices.push_back(0);
        indices.push_back(1);
        indices.push_back(2);
        indices.push_back(3);
        indices.push_back(0);
        indices.push_back(restart_id);

        indices.push_back(4);
        indices.push_back(5);
        indices.push_back(6);
        indices.push_back(7);
        indices.push_back(4);
        indices.push_back(restart_id);

        indices.push_back(0); indices.push_back(4);
        indices.push_back(restart_id);
        indices.push_back(1); indices.push_back(5);
        indices.push_back(restart_id);
        indices.push_back(2); indices.push_back(6);
        indices.push_back(restart_id);
        indices.push_back(3); indices.push_back(7);
        indices.push_back(restart_id);
    }
}