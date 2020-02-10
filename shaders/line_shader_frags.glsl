#version 330

varying vec4 vertColor;
uniform float intensity;

void main() {
     gl_FragColor = vec4(vertColor.xyz, intensity);
}