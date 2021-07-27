//
// Created by arne on 7/15/21.
//

#include "mimp_viewer.h"
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/progressbar.h>

#include <Eigen/Core>

using namespace Eigen;
using namespace nanogui;
using namespace std;

MimpViewer::MimpViewer(const std::string &filename) : Meshiew(false) {
    this->load_from_file(filename);
    this->filename = filename;
}

void MimpViewer::initShaders() {
    Meshiew::initShaders();

    tool_frame_renderer = std::make_shared<FrameRenderer>();
    tool_frame_renderer->init();
    tool_frame_renderer->setVisible(true);
    Affine3f id = Affine3f::Identity();
    tool_frame_renderer->show_frame(id.matrix());
    tool_frame_renderer->set_color(Eigen::Vector3f(0, 1, 0));
    this->add_renderer(tool_frame_renderer);

    closest_point_renderer = std::make_shared<PointRenderer>();
    closest_point_renderer->init();
    closest_point_renderer->setVisible(true);
    closest_point_renderer->setColor(Eigen::Vector3f(1, 0, 0));
    this->add_renderer(closest_point_renderer);

    connection_renderer = std::make_shared<LineRenderer>();
    connection_renderer->init();
    connection_renderer->setVisible(true);
    this->add_renderer(connection_renderer);
}

void MimpViewer::displayClosestPoint(const Eigen::Vector3f &point) {
    surface_mesh::Point p(point(0), point(1), point(2));
    closest_point_renderer->setPoint(p);
}

void MimpViewer::displayToolFrame(const Eigen::Ref<const Eigen::Matrix<float, 4, 4>> frame) {
    tool_frame_renderer->show_frame(frame);
}

void
MimpViewer::displayConnectionVector(const Eigen::Ref<const Eigen::Matrix<float, 4, 4>> frame, const Vector3f &closest) {
    Eigen::Matrix<float, 3, 2> l;
    Eigen::Vector3f p = frame.block<3, 1>(0, 3);
    l.col(0) = closest;
    l.col(1) = frame.block<3, 1>(0, 3);
    connection_renderer->show_line(l);
}

void MimpViewer::create_gui_elements(nanogui::Window *control, nanogui::Window *info) {
    Meshiew::create_gui_elements(control, info);
    this->tool_frame_renderer->setScaling(.3f*this->dist_max);
    control->setVisible(false);

    auto coordinates_window = new nanogui::Window(this, "Coordinates");
    coordinates_window->setPosition(Vector2i(1000, 20));

    auto layout = new AdvancedGridLayout({120, 400, 100}, {30, 30, 30}, 15);
    coordinates_window->setLayout(layout);

    const int dfs = 24;

    layout->setAnchor(
            new Label(coordinates_window, "Distance", "sans", dfs),
            AdvancedGridLayout::Anchor(0, 0)
    );
    layout->setAnchor(
            new Label(coordinates_window, "Orientation 1", "sans", dfs),
            AdvancedGridLayout::Anchor(0, 1)
    );
    layout->setAnchor(
            new Label(coordinates_window, "Orientation 2", "sans", dfs),
            AdvancedGridLayout::Anchor(0, 2)
    );

    for (int i = 0; i < 3; i++) {
        auto p = new ProgressBar(coordinates_window);
        p->setValue(.5);
        layout->setAnchor(p, AdvancedGridLayout::Anchor(1, i));
        progressbars.push_back(p);

        auto b = new FloatBox<double>(coordinates_window, 0);
        layout->setAnchor(b, AdvancedGridLayout::Anchor(2, i));
        coordinate_labels.push_back(b);
    }
    initialized = true;
}

const std::vector<std::pair<float, float>> bounds = {
        {-10.0, 10.0},
        {-M_PI, M_PI},
        {-M_PI, M_PI},
};

void MimpViewer::displayCoordinates(const Vector3f &c) {
    if (initialized) {
        for (int i = 0; i<3; i++) {
            coordinate_labels[i]->setValue(c(i));
            progressbars[i]->setValue((c(i) - bounds[i].first) / (bounds[i].second - bounds[i].first));
        }
    }
}


