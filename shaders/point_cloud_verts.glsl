#version 330
uniform mat4 MV;
uniform mat4 P;
uniform vec3 color;
uniform bool ext_color;

in vec3 position;
in vec4 vertexColors;

out vec4 vertColor;

void main() {
    vec4 vpoint_mv = MV * vec4(position, 1.0);
    gl_Position = P * vpoint_mv;

    if (ext_color) {
        vertColor = vertexColors;
    } else {
        vertColor = vec4(color, 1.0);
    }
}

