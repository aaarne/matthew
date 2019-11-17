//
// Created by arne on 11/16/19.
//

#include <fstream>
#include "pointiew.h"
#include "shaders_gen.h"

using namespace std;

Pointiew::Pointiew() : has_color(false), point_size(2.0), n(0), color(0.0, 0.0, 0.0) {}

void Pointiew::load(std::string filename) {
    std::ifstream in(filename);
    string msg;
    do {
        in >> msg;
    } while (msg != "FIELDS");
    in >> msg;
    int n_fields = 0;
    do {
        in >> msg;
        n_fields++;
    } while (msg != "SIZE");
    cout << "n_fields: " << n_fields << endl;
    do {
        in >> msg;
    } while (msg != "POINTS");
    in >> msg;
    int n;
    stringstream ss(msg);
    ss >> n;
    cout << "Amount of points: " << n << endl;
    do {
        in >> msg;
    } while (msg != "ascii");

    Eigen::MatrixXf points(3, n);
    Eigen::MatrixXf colors(4, n);
    int i = 0;

    if (n_fields == 3) {
        has_color = false;
        while (in >> msg) {
            float v;
            stringstream ss(msg);
            ss >> v;
            points(i % 3, i / 3) = v;
            i++;
        }
    } else if (n_fields == 4) {
        has_color = true;
        while (in >> msg) {
            stringstream ss(msg);
            if (i % 4 == 3) {
                unsigned int c;
                ss >> c;
                colors(0, i / 4) = (c >> 0 & 0xff) / 255.0f;
                colors(1, i / 4) = (c >> 8 & 0xff) / 255.0f;
                colors(2, i / 4) = (c >> 16 & 0xff) / 255.0f;
                colors(3, i / 4) = (c >> 24 & 0xff) / 255.0f;
            } else {
                float v;
                ss >> v;
                points(i % 4, i / 4) = v;
            }
            i++;
        }
    } else {
        cerr << "Unexpected number of fields in point cloud data" << endl;
        exit(1);
    }
    this->n = n;
    in.close();
    this->points = points;
    this->colors = colors;
    pcdShader.bind();
    pcdShader.uploadAttrib("position", points);
    pcdShader.uploadAttrib("vertexColors", colors);
    pcdShader.setUniform("ext_color", (int)has_color);
}

void Pointiew::initShaders() {
    using namespace shaders;
    pcdShader.init("pcd", point_cloud_verts, point_cloud_frag);
}

void Pointiew::create_gui_elements() {
    using namespace nanogui;
    auto *window = new Window(this, "Display Control");
    window->setPosition(Vector2i(15, 15));
    window->setLayout(new GroupLayout());

    new Label(window, "Base Color:", "sans-bold");
    auto cp = new ColorPicker(window, color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        color << c.r(), c.g(), c.b();
    });
    cp->setEnabled(!has_color);

    new Label(window, "Point Size");
    auto *slider = new Slider(window);
    slider->setValue(0.2);
    slider->setCallback([this](float value) {
        this->point_size = std::max(0.1f, 10.0f * value);
    });

    window = new Window(this, "Point Cloud Info");
    window->setPosition(Vector2i(750, 15));
    GridLayout *grid = new GridLayout(Orientation::Horizontal, 2, Alignment::Minimum, 15, 5);
    grid->setSpacing(0, 10);
    window->setLayout(grid);

    new Label(window, "Filename:", "sans-bold");
    new Label(window, filename, "sans");

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

Matthew::Point Pointiew::get_model_center() {
    Eigen::Vector3f center = points.rowwise().mean();
    return {center(0), center(1), center(2)};
}

float Pointiew::get_model_dist_max() {
    Eigen::Vector3f center = points.rowwise().mean();
    Eigen::VectorXf lens = (points.colwise() - center).colwise().norm();
    return lens.maxCoeff();
}

void Pointiew::draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) {
    pcdShader.bind();
    pcdShader.setUniform("MV", mv);
    pcdShader.setUniform("P", p);
    pcdShader.setUniform("color", color);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glPointSize(point_size);
    pcdShader.drawArray(GL_POINTS, 0, n);
}

