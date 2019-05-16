#version 330 core

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;    
    float shininess;
}; 

struct Light {
    vec3 position;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

// //***** begin interface of view.glsl ***********************************
mat4 get_modelview_matrix();
// //***** end interface of view.glsl ***********************************


uniform Material material;
uniform Light light;

vec3 compute_light(vec3 position_world_space, vec3 normal_world_space)
{
    // ambient
    vec3 ambient = light.ambient * material.ambient;
    
    // diffuse
    vec3 norm = normalize(normal_world_space);
    vec3 lightDir = normalize(light.position - position_world_space);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * (diff * material.diffuse);
    
    // specular
    mat4 IV = inverse(get_modelview_matrix());
    vec3 viewDir = normalize(IV[3].xyz - position_world_space);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * (spec * material.specular);

    return ambient + diffuse + specular;
}

vec3 compute_light_on_color(vec3 position_world_space, vec3 normal_world_space, vec3 color)
{
    // ambient
    // vec3 ambient =  light.ambient * (0.2 * material.ambient + 0.8 * color);
    vec3 ambient =  light.ambient * material.ambient;
    
    // diffuse
    vec3 norm = normalize(normal_world_space);
    vec3 lightDir = normalize(light.position - position_world_space);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * (diff * material.diffuse);
    
    // specular
    mat4 IV = inverse(get_modelview_matrix());
    vec3 viewDir = normalize(IV[3].xyz - position_world_space);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * (spec * material.specular);

    return (ambient + diffuse + specular) * color;
}

float softstep(float halfwidth, float smoothness, float t)
{
    return 1 / (1 + exp(-(t - halfwidth)/smoothness));
}

float softstep_inv(float halfwidth, float smoothness, float t)
{
    return 1 - 1 / (1 + exp(-(t - 1 + halfwidth)/smoothness));
}


float tick(float t)
{
    // sum of partial derivations |dt/dx| + |dt/dy|
    float dt = fwidth(t);

    // smoothness for sigmoid-filter with logistics functions
    float smoothness = 0.25 * dt;
    float halfwidth  = 0.05;
    float t_fract = fract(t);

    return (softstep(halfwidth, smoothness, t_fract) * softstep_inv(halfwidth, smoothness, t_fract));
}
