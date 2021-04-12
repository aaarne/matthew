//
// Created by arne on 2/10/20.
//

#ifndef MATTHEW_POINT_CLOUD_RENDERER_H
#define MATTHEW_POINT_CLOUD_RENDERER_H


#include <nanogui/glutil.h>
#include <surface_mesh/types.h>
#include "renderer.h"

class PointCloudRenderer : public Renderer {
public:
    explicit PointCloudRenderer();
    void init() override;

    void show_points(const Eigen::MatrixXf &p);
    void show_points(const Eigen::MatrixXf &p, const Eigen::MatrixXf &colors);
    void set_point_size(float size);
    void set_color(const Eigen::Vector3f &c);
    void set_colors(const Eigen::MatrixXf &c);

    const std::string default_file;
protected:
    void do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) override;

private:
    bool updated = false;
    float point_size = 1.0;
    bool vertex_color_prop_available = false;
    long n;
    Eigen::MatrixXf colors;
    Eigen::Vector3f color;
    Eigen::MatrixXf points;


    nanogui::GLShader shader;

};


#endif //MATTHEW_POINT_CLOUD_RENDERER_H
