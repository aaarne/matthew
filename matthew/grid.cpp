//
// Created by arne on 11/18/19.
//

#include <iostream>
#include "grid.h"

using namespace std;

Grid::Grid(int lines_per_dim, float scale, const Eigen::Vector3f &origin)
        : scale(scale), origin(origin), lines_per_dim(lines_per_dim) {
    int n = lines_per_dim;
    Eigen::MatrixXf points(3, 4 * n);
    int k = 0;

    for (int i = 0; i < n; i++) {
        points.col(k++) << 2 * float(i) / float(n - 1) - 1.0f, -1.0, 0.0;
        points.col(k++) << 2 * float(i) / float(n - 1) - 1.0f, 1.0, 0.0;
    }
    for (int i = 0; i < n; i++) {
        points.col(k++) << -1.0, 2 * float(i) / float(n - 1) - 1.0f, 0.0;
        points.col(k++) << 1.0, 2 * float(i) / float(n - 1) - 1.0f, 0.0;
    }
    points *= scale/2;
    points.colwise() += origin;
    this->points = points;
}

Eigen::MatrixXf Grid::get_points() const {
    return points;
}

int Grid::n() const {
    return 4*lines_per_dim;
}
