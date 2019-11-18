varying vec4 vertColor;
uniform float intensity;

void main() {
    gl_FragColor = intensity * vertColor;
}