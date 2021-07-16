#version 330
uniform mat4 MV;
uniform mat4 P;

in vec3 position;

void main() {
    gl_Position = P * MV * vec4(position, 1.0);
}
