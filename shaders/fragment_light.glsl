#version 330

uniform int color_mode;
uniform int light_model;
uniform vec3 intensity;
uniform vec3 light_color;

in vec3 fcolor;
in vec3 fnormal;
in vec3 view_dir;
in vec3 light_dir;

out vec4 color;

vec3 diffuse(vec3 N, vec3 L)
{
    vec3 nrmN = normalize(N);
    vec3 nrmL = normalize(L);
    return light_color*fcolor*max(0.0, dot(nrmN, nrmL));
}

vec3 ambient() {
    return 0.1 * fcolor;
}

vec3 specular(vec3 N, vec3 V, vec3 L) {
    vec3 R = reflect(-L, N);
    float spec = pow(max(dot(V, R), 0.0), 8);
    return .5*spec*light_color;
}

void main() {
    vec3 light_pos = normalize(light_dir);
    vec3 N = normalize(fnormal);
    vec3 V = normalize(view_dir);

    vec3 result;

    if (light_model == 1) {
        result = ambient() + diffuse(N, light_pos);
    } else if (light_model == 2) {
        result = ambient() + diffuse(N, light_pos) + specular(N, V, light_pos);
    } else if (light_model == 3) {
        result = 2*specular(N, V, light_pos);
    } else if (light_model == 4) {
        result = ambient() + diffuse(N, light_pos) + diffuse(-N, light_pos);
    } else {
        result = fcolor;
    }

    color = vec4(result, 1.0);
}

