//
// Created by arne on 2/10/20.
//

#include "vectorfield_renderer.h"
#include "shaders_gen.h"

using namespace std;

void VectorfieldRenderer::init() {
    shader.init("vectorfield", shaders::normals_vertex, shaders::normals_fragment, shaders::normals_geometry);
}

void VectorfieldRenderer::do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) {
    shader.bind();
    if (updated) {
        shader.setUniform("scaling", this->scaling);
        shader.uploadAttrib("position", this->points);
        shader.uploadAttrib("normal", this->vecs);
        if (color_prop) shader.uploadAttrib("vec_colors", this->colors);
        else shader.setUniform("single_color", this->color);
        updated = false;
    }
    shader.setUniform("MV", mv);
    shader.setUniform("P", p);
    shader.drawArray(GL_TRIANGLES, 0, this->points.cols());
}

void VectorfieldRenderer::show_vectorfield(const Eigen::MatrixXf &p, const Eigen::MatrixXf &v) {
    Eigen::MatrixXf vecs(v);
    float mean_length = vecs.colwise().norm().mean();
    vecs *= this->scaling/mean_length;
    this->points = p;
    this->vecs = vecs;
    this->updated = true;
}

void VectorfieldRenderer::show_vectorfield(const Eigen::MatrixXf &p, const Eigen::MatrixXf &v,
                                           const Eigen::MatrixXf &colors) {
    show_vectorfield(p, v);
    this->colors = colors;
}

void VectorfieldRenderer::set_color(const Eigen::Vector3f &color) {
    this->updated = true;
    this->color_prop = false;
    this->color = color;
}

void VectorfieldRenderer::set_scaling(float scaling) {
    this->updated = true;
    this->scaling = scaling;
}

