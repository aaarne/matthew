//
// Created by arne on 2/10/20.
//

#ifndef MATTHEW_POINT_CLOUD_RENDERER_H
#define MATTHEW_POINT_CLOUD_RENDERER_H


#include <Eigen/Eigen>

class PointCloudRenderer {


private:
    float point_size;
    bool has_color;
    long n;
    Eigen::MatrixXf colors;
    Eigen::Vector3f color;
    Eigen::MatrixXf points;

};


#endif //MATTHEW_POINT_CLOUD_RENDERER_H
