//
// Created by arne on 2/10/20.
//

#include <shaders_gen.h>
#include "point_cloud_renderer.h"

using namespace std;

void PointCloudRenderer::init() {
    shader.init("pcd", shaders::point_cloud_verts, shaders::point_cloud_frag);
}

void PointCloudRenderer::do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) {
    shader.bind();
    if (updated) {
        shader.uploadAttrib("position", this->points);
        shader.setUniform("ext_color", (int)vertex_color_prop_available);
        if (vertex_color_prop_available) shader.uploadAttrib("vertexColors", this->colors);
        else shader.setUniform("color", this->color);
        updated = false;
    }
    shader.setUniform("MV", mv);
    shader.setUniform("P", p);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glPointSize(point_size);
    shader.drawArray(GL_POINTS, 0, n);
}

void PointCloudRenderer::show_points(const Eigen::MatrixXf &p) {
    this->points = p;
    this->n = p.cols();
    this->updated = true;
}

PointCloudRenderer::PointCloudRenderer() {

}

void PointCloudRenderer::set_color(const Eigen::Vector3f &c) {
    this->vertex_color_prop_available = false;
    this->color = c;
    this->updated = true;
}

void PointCloudRenderer::set_colors(const Eigen::MatrixXf &c) {
    assert((c.cols() == n));
    this->vertex_color_prop_available = true;
    this->colors = c;
    this->updated = true;
}

void PointCloudRenderer::show_points(const Eigen::MatrixXf &p, const Eigen::MatrixXf &colors) {
    this->show_points(p);
    this->colors = colors;
    this->vertex_color_prop_available = true;
    this->updated = true;
}

void PointCloudRenderer::set_point_size(float size) {
    this->point_size = size;
}
