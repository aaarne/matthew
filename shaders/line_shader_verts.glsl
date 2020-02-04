#version 330
in vec3 position;
uniform mat4 MV;
uniform mat4 P;

void main() {
    gl_Position = P * MV * vec4(position, 1.0);
}
