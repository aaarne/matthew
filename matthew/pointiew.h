//
// Created by arne on 11/16/19.
//

#ifndef CROISSANT_POINTIEW_H
#define CROISSANT_POINTIEW_H


#include <point_cloud_renderer.h>
#include "matthew.h"
#include "grid.h"

class Pointiew : public Matthew {
    friend class Matthew;
public:
    explicit Pointiew(bool fs = false);

protected:

    void load_from_file(const std::string &filename) override;

    void initModel() override;

    void initShaders() override;

    void create_gui_elements(nanogui::Window *, nanogui::Window *info) override;

    Eigen::Vector3f get_model_dimensions() override;

    Eigen::Vector3f get_model_center() override;

    float get_model_dist_max() override;

private:
    bool has_color;
    Eigen::MatrixXf colors;
    Eigen::MatrixXf points;
    std::shared_ptr<PointCloudRenderer> renderer;
};


#endif //CROISSANT_POINTIEW_H
