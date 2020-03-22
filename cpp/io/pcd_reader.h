//
// Created by arne on 3/22/20.
//

#ifndef MATTHEW_PCD_READER_H
#define MATTHEW_PCD_READER_H


#include <string>
#include <Eigen/Eigen>

class PCDReader {
public:
    explicit PCDReader(const std::string &filename);

    Eigen::MatrixXf get_points() const { return this->points; }

    Eigen::MatrixXf get_colors() const { return this->colors; }

    bool has_color() const { return this->color; }

private:
    bool color;
    Eigen::MatrixXf points;
    Eigen::MatrixXf colors;

};


#endif //MATTHEW_PCD_READER_H
