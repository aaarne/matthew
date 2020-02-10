#version 330
uniform mat4 MV;
uniform mat4 P;
uniform vec3 line_color;

in vec3 position;
out vec4 vertColor;

void main() {
    gl_Position = P * MV * vec4(position, 1.0);
    vertColor = vec4(line_color, 1.0);
}
