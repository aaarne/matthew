#version 330

out vec4 color;
uniform vec3 line_color;

void main() {
     color = vec4(line_color, 1.0);
}