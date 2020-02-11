#version 330

in vec4 vertColor;
uniform float intensity;
out vec4 color;

void main() {
     color = vec4(vertColor.xyz, 1.0);
}