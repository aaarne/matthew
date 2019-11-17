#version 330
uniform mat4 MV;
uniform mat4 P;

in vec3 position;
//in vec4 color;

out vec4 vertColor;

void main() {
    vec4 vpoint_mv = MV * vec4(position, 1.0);
    gl_Position = P * vpoint_mv;

    vertColor = vec4(1.0, 1.0, 1.0, 1.0);
}

