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

PointRenderer::PointRenderer(Eigen::Vector3f offset) : offset(offset) {

}

void PointRenderer::do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) {
    if (updated) {
        Eigen::MatrixXf m(3, 1);
        m.col(0) << point.x, point.y, point.z;
        m.colwise() += offset;
        shader.bind();
        shader.uploadAttrib("position", m);
        updated = false;
    }
    if (enabled) {
        shader.bind();
        shader.setUniform("MV", mv);
        shader.setUniform("P", p);
        glPointSize(10.0f);
        shader.drawArray(GL_POINTS, 0, 1);
    }
}

void PointRenderer::clear() {
    points.clear();
}
