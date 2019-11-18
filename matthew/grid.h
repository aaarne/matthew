//
// Created by arne on 11/18/19.
//

#ifndef CROISSANT_GRID_H
#define CROISSANT_GRID_H

#include "nanogui/screen.h"

class Grid {
public:
    Grid(int lines_per_dim, float scale, const Eigen::Vector3f &origin);

    Eigen::MatrixXf get_points() const;

    int n() const;

protected:
    int lines_per_dim;
    float scale;
    Eigen::Vector3f origin;

    Eigen::MatrixXf points;
};


#endif //CROISSANT_GRID_H
