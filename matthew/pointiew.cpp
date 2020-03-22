//
// Created by arne on 11/16/19.
//

#include <fstream>
#include "pointiew.h"
#include "shaders_gen.h"
#include "pcd_reader.h"
#include "grid.h"
#include <nanogui/opengl.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/colorpicker.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>

using namespace std;

Pointiew::Pointiew(bool fs) : Matthew::Matthew(fs), color(1.0, 1.0, 1.0) {
    setBackground(nanogui::Color(0.f, 0.f, 0.f, 0.f));
    renderer = std::make_shared<PointCloudRenderer>();
}

void Pointiew::initModel() {
    if (has_color) {
        renderer->show_points(points, colors);
    } else {
        renderer->show_points(points);
        renderer->set_color(color);
    }
    renderer->setVisible(true);
    cout << points.rows() << " x " << points.cols() << endl;
}

void Pointiew::initShaders() {
    using namespace shaders;
    renderer->init();
    add_renderer(renderer);
}

void Pointiew::create_gui_elements(nanogui::Window *control, nanogui::Window *info) {
    using namespace nanogui;

    new Label(control, "Base Color:", "sans-bold");
    auto cp = new ColorPicker(control, color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        color << c.r(), c.g(), c.b();
        renderer->set_color(color);
    });
    cp->setEnabled(!has_color);


    new Label(control, "Point Size");
    auto slider = new Slider(control);
    slider->setValue(0.2);
    slider->setCallback([this](float value) {
        renderer->set_point_size(std::max(0.1f, 10.f * value));
    });

    Window *window = info;
    window->setPosition(Vector2i(mFBSize(0)-250, 15));
    auto *grid = new GridLayout(Orientation::Horizontal, 2, Alignment::Minimum, 15, 5);
    grid->setSpacing(0, 10);
    window->setLayout(grid);

    auto info_line = [&](std::string title, int value) {
        new Label(window, title + ":");
        auto *box = new IntBox<int>(window);
        box->setEditable(false);
        box->setFontSize(14);
        box->setValue(value);
    };

    info_line("Points", this->points.cols());

    new Label(window, "Has Color");
    auto *checkbox = new CheckBox(window, "");
    checkbox->setEnabled(false);
    checkbox->setChecked(this->has_color);
}

Eigen::Vector3f Pointiew::get_model_center() {
    return points.rowwise().mean();
}

float Pointiew::get_model_dist_max() {
    Eigen::Vector3f center = points.rowwise().mean();
    Eigen::VectorXf lens = (points.colwise() - center).colwise().norm();
    return lens.maxCoeff();
}

Eigen::Vector3f Pointiew::get_model_dimensions() {
    return points.rowwise().maxCoeff() - points.rowwise().minCoeff();
}

void Pointiew::load_from_file(const std::string &filename) {
    PCDReader reader(filename);
    this->has_color = reader.has_color();
    this->points = reader.get_points();
    if (reader.has_color()) {
        this->colors = reader.get_colors();
    }
}



