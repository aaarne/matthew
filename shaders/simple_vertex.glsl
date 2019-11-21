#version 330

uniform mat4 MV;
uniform mat4 P;
uniform int color_mode;
uniform vec3 intensity;

in vec3 position;
in vec3 colors;
in vec3 normal;

out vec3 fcolor;
out vec3 fnormal;
out vec3 view_dir;
out vec3 light_dir;

void main() {
    vec4 vpoint_mv = MV * vec4(position, 1.0);
    gl_Position = P * vpoint_mv;
    if (color_mode == 1) {
        fcolor = colors;
    } else {
        fcolor = intensity;
    }
    fnormal = mat3(transpose(inverse(MV))) * normal;
    light_dir = vec3(0.0, 3.0, 3.0) - vpoint_mv.xyz;
    view_dir = -vpoint_mv.xyz;
}
