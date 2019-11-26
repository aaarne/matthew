#version 330

uniform vec3 intensity;
uniform vec3 light_color;
uniform float ambient_term;
uniform float diffuse_term;
uniform float specular_term;
uniform bool broken_normals;
uniform int shininess;

in vec3 fcolor;
in vec3 fnormal;
in vec3 view_dir;
in vec3 light_dir;

out vec4 color;

vec3 diffuse(vec3 N, vec3 L)
{
    return light_color*fcolor*max(0.0, dot(N, L));
}

vec3 ambient() {
    return fcolor;
}

vec3 specular(vec3 N, vec3 V, vec3 L) {
    vec3 R = reflect(-L, N);
    float spec = pow(max(dot(V, R), 0.0), shininess);
    return spec*light_color;
}

void main() {
    vec3 light_pos = normalize(light_dir);
    vec3 N = normalize(fnormal);
    vec3 V = normalize(view_dir);

    vec3 result;

    if (broken_normals) {
        result = ambient_term*ambient()
        + diffuse_term * diffuse(N, light_pos) + diffuse_term * diffuse(-N, light_pos)
        + specular_term * specular(N, V, light_pos) + specular_term * specular(-N, V, light_pos);
    } else {
        result = ambient_term*ambient()
        + diffuse_term * diffuse(N, light_pos)
        + specular_term * specular(N, V, light_pos);
    }

    color = vec4(result, 1.0);
}

