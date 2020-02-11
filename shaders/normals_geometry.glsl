#version 330

layout (triangles) in;
layout (line_strip, max_vertices = 6) out;

uniform mat4 MV;
uniform mat4 P;
uniform float scaling;

in VS_OUT {
    mat3 normal_mat;
    vec3 normal;
} gs_in[];

void createline(int index) {
    gl_Position = P * MV * gl_in[index].gl_Position;
    EmitVertex();
    vec4 normal_mv = vec4(normalize(gs_in[index].normal_mat *
    gs_in[index].normal), 1.0f);
    float size = length(gs_in[index].normal) * scaling;
    gl_Position = P * (MV * gl_in[index].gl_Position + normal_mv * size);
    EmitVertex();
    EndPrimitive();
}

void main() {
    createline(0);
    createline(1);
    createline(2);
}
