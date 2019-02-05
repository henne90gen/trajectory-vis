#version 330 core

float norm(vec4 v)
{
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z + v.w*v.w);
}

float norm(vec3 v)
{
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

vec4 quat_normed(vec4 q)
{
    vec4 q_norm;
    float norm = norm(q);
    // float norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    q_norm.x =  q.x / norm;
    q_norm.y =  q.y / norm;
    q_norm.z =  q.z / norm;
    q_norm.w =  q.w / norm;
    return q_norm;
}

vec3 quat_rotate(vec3 pos, vec4 q)
{
    return pos + 2.0 * cross(q.xyz, cross(q.xyz, pos) + q.w * pos);
}

vec3 triangle_normal(vec3 v1, vec3 v2, vec3 v3)
{
    vec3 V = v2 - v1;
    vec3 W = v3 - v1;

    return normalize(cross(V,W));
}