//
// Created by arne on 7/14/21.
//

#include "frame_renderer.h"
#include <Eigen/Core>

FrameRenderer::FrameRenderer() {
}

void FrameRenderer::do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) {
    for (const auto &r : all) r->draw(mv, p);
    point_renderer.draw(mv, p);
}

void FrameRenderer::init() {
    for (const auto &r :all) r->init();
    point_renderer.init();
    lrx.setColor(surface_mesh::Color(1, 0, 0));
    lry.setColor(surface_mesh::Color(0, 1, 0));
    lrz.setColor(surface_mesh::Color(0, 0, 1));
    point_renderer.setColor(Eigen::Vector3f(0, 0, 0));
    this->show_frame(Eigen::Matrix<float, 4, 4>::Identity());
}

void FrameRenderer::show_frame(const Eigen::Ref<const Eigen::Matrix<float, 4, 4>> frame) {
    this->frame = frame;
    compute();
}

void FrameRenderer::setVisible(bool visible) {
    Renderer::setVisible(visible);
    for (const auto &r : all) r->setVisible(visible);
    point_renderer.setVisible(visible);
}

void FrameRenderer::compute() {
    Eigen::Vector3f x = frame.block<3, 1>(0, 0);
    Eigen::Vector3f y = frame.block<3, 1>(0, 1);
    Eigen::Vector3f z = frame.block<3, 1>(0, 2);
    Eigen::Vector3f p = frame.block<3, 1>(0, 3);

    Eigen::Matrix<float, 3, 2> X, Y, Z;
    X.col(0) << p;
    X.col(1) << p + scaling*x;
    Y.col(0) << p;
    Y.col(1) << p + scaling*y;
    Z.col(0) << p;
    Z.col(1) << p + scaling*z;

    lrx.show_line(X);
    lry.show_line(Y);
    lrz.show_line(Z);
    point_renderer.setPoint(p);
}
