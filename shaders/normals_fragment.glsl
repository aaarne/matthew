#version 330

out vec4 frag_color;

uniform vec3 single_color;

void main() {
    frag_color = vec4(single_color, 1.0);
}
