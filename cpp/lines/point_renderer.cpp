//
// Created by sach_ar on 2/6/20.
//

#include "point_renderer.h"
#include "shaders_gen.h"

using namespace std;

void PointRenderer::init() {
    shader.init("single_point_shader", shaders::single_point_verts, shaders::single_point_frags);
    initialized = true;
}

void PointRenderer::setPoint(const surface_mesh::Point &p) {
    this->point = p;
    this->points.push_back(p);
    updated = true;
}

std::vector<surface_mesh::Point> PointRenderer::trace() const {
    return this->points;
}

void PointRenderer::draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) {
    if (updated) {
        Eigen::MatrixXf m(3, 1);
        m.col(0) << point.x, point.y, point.z;
        shader.bind();
        shader.uploadAttrib("position", m);
        updated = false;
    }
    shader.bind();
    shader.setUniform("MV", mv);
    shader.setUniform("P", p);
    glPointSize(10.0f);
    shader.drawArray(GL_POINTS, 0, 1);
}
