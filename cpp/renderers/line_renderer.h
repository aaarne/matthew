//
// Created by sach_ar on 2/4/20.
//

#ifndef CROISSANT_LINERENDERER_H
#define CROISSANT_LINERENDERER_H


#include <nanogui/glutil.h>
#include <surface_mesh/types.h>
#include <surface_mesh/Surface_mesh.h>
#include "renderer.h"

class LineRenderer : public Renderer {
public:
    explicit LineRenderer() : LineRenderer(Eigen::Vector3f::Zero()) {}
    explicit LineRenderer(const Eigen::Vector3f &offset) : LineRenderer(offset, surface_mesh::Color(1.0, 1.0, 1.0)) {}

    explicit LineRenderer(const Eigen::Vector3f &offset, surface_mesh::Color c) : offset(offset), color(c), intensity(1.0) {}

    virtual ~LineRenderer() {}

    void init() override;

    void do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) override;

    void show_line_segments(const std::vector<surface_mesh::Point> &l);

    void show_line_segments(const Eigen::MatrixXf &l);

    void show_line(const std::vector<surface_mesh::Point> &l);
    void show_line(const Eigen::MatrixXf &l);

    void show_isolines(const surface_mesh::Surface_mesh &mesh, const std::string &property_name, int n_intervals);

    void setColor(const surface_mesh::Color &c);
    void setColor(const Eigen::Vector3f &c);

    surface_mesh::Color getColor() const { return color; }

protected:
    void upload_line();

private:
    Eigen::Vector3f offset;
    surface_mesh::Color color;
    nanogui::GLShader lineShader;
    Eigen::MatrixXf line;
    float intensity = 1.0;
    bool strip_mode = false;
    bool updated = false;

};


#endif //CROISSANT_LINERENDERER_H
