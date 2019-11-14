#version 330

uniform mat4 MV;
uniform mat4 P;
uniform int color_mode;
uniform vec3 intensity;

in vec3 position;
in vec3 valence_color;
in vec3 unicurvature_color;
in vec3 curvature_color;
in vec3 gaussian_curv_color;
in vec3 normal;

out vec3 fcolor;
out vec3 fnormal;
out vec3 view_dir;
out vec3 light_dir;

void main() {
    vec4 vpoint_mv = MV * vec4(position, 1.0);
    gl_Position = P * vpoint_mv;
    if (color_mode == 1) {
        fcolor = valence_color;
    } else if (color_mode == 2) {
        fcolor = unicurvature_color;
    } else if (color_mode == 3) {
        fcolor = curvature_color;
    } else if (color_mode == 4) {
        fcolor = gaussian_curv_color;
    } else {
        fcolor = intensity;
    }
    fnormal = mat3(transpose(inverse(MV))) * normal;
    light_dir = vec3(0.0, 3.0, 3.0) - vpoint_mv.xyz;
    view_dir = -vpoint_mv.xyz;
}
