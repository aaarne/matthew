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
    tool_frame_renderer->init();
    tool_frame_renderer->setVisible(true);
    this->add_renderer(tool_frame_renderer);

    closest_point_renderer = std::make_shared<PointRenderer>();
    closest_point_renderer->init();
    closest_point_renderer->setVisible(true);
    this->add_renderer(closest_point_renderer);
}
