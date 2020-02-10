//
// Created by arne on 11/18/19.
//

#include <iostream>
#include <Eigen/Eigen>
#include "grid.h"

using namespace std;

Grid::~Grid() {}

Grid::Grid(int lines_per_dim, float scale, const Eigen::Vector3f &origin)
        : LineRenderer::LineRenderer(Eigen::Vector3f::Zero()),
          scale(scale), origin(origin), lines_per_dim(lines_per_dim) {
    int n = lines_per_dim;
    Eigen::MatrixXf points(3, 4 * n + 6);
    int k = 0;

    for (int i = 0; i < n; i++) {
        points.col(k++) << 2 * float(i) / float(n - 1) - 1.0f, -1.0, 0.0;
        points.col(k++) << 2 * float(i) / float(n - 1) - 1.0f, 1.0, 0.0;
    }
    for (int i = 0; i < n; i++) {
        points.col(k++) << -1.0, 2 * float(i) / float(n - 1) - 1.0f, 0.0;
        points.col(k++) << 1.0, 2 * float(i) / float(n - 1) - 1.0f, 0.0;
    }
    points.col(k++) << 0.95, 0.0, 0.0;
    points.col(k++) << 1.05, 0.0, 0.0;
    points.col(k++) << -0.01, 0.95, 0.0;
    points.col(k++) << -0.01, 1.05, 0.0;
    points.col(k++) << 0.01, 0.95, 0.0;
    points.col(k++) << 0.01, 1.05, 0.0;
    points *= scale / 2;
    points.colwise() += origin;
    this->points = points;
    this->show_line_segments(get_points());
}

Eigen::MatrixXf Grid::get_points() const {
    return points;
}

int Grid::n() const {
    return 4 * lines_per_dim + 6;
}

void Grid::setIntensity(float value) {
    Eigen::Vector3f white(1.0, 1.0, 1.0);
    this->setColor(white * value);
}
