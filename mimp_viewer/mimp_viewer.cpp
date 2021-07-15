//
// Created by arne on 7/15/21.
//

#include "mimp_viewer.h"

MimpViewer::MimpViewer(const std::string &filename) : Meshiew(false) {
    this->load_from_file(filename);
    this->filename = filename;
}

void MimpViewer::initShaders() {
    Meshiew::initShaders();

    tool_frame_renderer = std::make_shared<FrameRenderer>();
    tool_frame_renderer->setScaling(.1);
    tool_frame_renderer->init();
    tool_frame_renderer->setVisible(true);
    this->add_renderer(tool_frame_renderer);

    closest_point_renderer = std::make_shared<PointRenderer>();
    closest_point_renderer->init();
    closest_point_renderer->setVisible(true);
    this->add_renderer(closest_point_renderer);
}

void MimpViewer::displayClosestPoint(const Eigen::Vector3d &point) {
    surface_mesh::Point p(point(0), point(1), point(2));
    closest_point_renderer->setPoint(p);
}

void MimpViewer::displayToolFrame(const Eigen::Ref<const Eigen::Matrix<float, 4, 4>> frame) {
    tool_frame_renderer->show_frame(frame);
}

void MimpViewer::create_gui_elements(nanogui::Window *control, nanogui::Window *info) {
    Meshiew::create_gui_elements(control, info);
    control->setVisible(false);
}

