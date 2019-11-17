//
// Created by arne on 11/16/19.
//

#ifndef CROISSANT_POINTIEW_H
#define CROISSANT_POINTIEW_H


#include "matthew.h"

class Pointiew : public Matthew {
public:
    explicit Pointiew();

protected:

    void load(std::string filename) override;

    void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) override;

    void initShaders() override;

    void create_gui_elements() override;

    Point get_model_center() override;

    float get_model_dist_max() override;

private:
    float point_size;
    bool has_color;
    long n;
    Eigen::MatrixXf colors;
    Eigen::Vector3f color;
    Eigen::MatrixXf points;

    nanogui::GLShader pcdShader;
};


#endif //CROISSANT_POINTIEW_H