#version 330
in vec3 position;
in vec3 normal;
uniform mat4 MV;
uniform mat4 P;
out VS_OUT {
    mat3 normal_mat;
    vec3 normal;
} vs_out;
void main() {
  gl_Position = vec4(position, 1.0);
    vs_out.normal = normal;
    vs_out.normal_mat = mat3(transpose(inverse(MV)));
}
