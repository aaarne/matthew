#version 330
out vec4 color;
uniform vec3 point_color;

void main() {
    color = vec4(point_color, 1.0);
}
