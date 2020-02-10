//
// Created by arne on 11/18/19.
//

#ifndef CROISSANT_GRID_H
#define CROISSANT_GRID_H

#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include "nanogui/screen.h"
#include "line_renderer.h"

class Grid : public LineRenderer {
public:
    Grid(int lines_per_dim, float scale, const Eigen::Vector3f &origin);
    virtual ~Grid();

    void setIntensity(float value);

protected:
    Eigen::MatrixXf get_points() const;
    int n() const;
    int lines_per_dim;
    float scale;
    Eigen::Vector3f origin;
    Eigen::MatrixXf points;
};


#endif //CROISSANT_GRID_H
