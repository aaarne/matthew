//
// Created by arne on 11/16/19.
//

#include <fstream>
#include "pointiew.h"
#include "shaders_gen.h"

using namespace std;

void Pointiew::load(std::string filename) {
    std::ifstream in(filename);
    string msg;
    do {
        in >> msg;
    } while (msg != "POINTS");
    in >> msg;
    int n;
    stringstream ss(msg); ss >> n;
    cout << "Amount of points: " << n << endl;
    do {
        in >> msg;
    } while (msg != "ascii");

    Eigen::MatrixXf points(3, n);
    int i = 0;

    while (in >> msg) {
        float v; stringstream ss(msg); ss >> v;
        points(i%3, i/3) = v;
        i++;
    }
    this->n = n;

    in.close();
    this->points = points;
    pcdShader.bind();
    pcdShader.uploadAttrib("position", points);
}

void Pointiew::initShaders() {
    using namespace shaders;
    pcdShader.init("pcd", point_cloud_verts, point_cloud_frag);
}

void Pointiew::create_gui_elements() {
    using namespace nanogui;
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
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glPointSize(2.0);
    pcdShader.drawArray(GL_POINTS, 0, n);
}


