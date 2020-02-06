//
// Created by sach_ar on 2/4/20.
//

#ifndef CROISSANT_LINERENDERER_H
#define CROISSANT_LINERENDERER_H


#include <nanogui/glutil.h>
#include <surface_mesh/types.h>
#include <surface_mesh/Surface_mesh.h>

class LineRenderer {
public:
    LineRenderer(Eigen::Vector3f offset) : LineRenderer(offset, surface_mesh::Color(1.0, 1.0, 1.0)) {}

    explicit LineRenderer(Eigen::Vector3f offset, surface_mesh::Color c) : offset(offset), color(c), enabled(true) {}

    void setVisible(bool visible) { this->enabled = visible; }

    void init();

    void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p);

    void show_line_segments(const std::vector<surface_mesh::Point> &l);

    void show_line(const std::vector<surface_mesh::Point> &l);

    void show_isolines(const surface_mesh::Surface_mesh &mesh, const std::string &property_name, int n_intervals);

    void setColor(const surface_mesh::Color &c);

    surface_mesh::Color getColor() const { return color; }

protected:
    void upload_line();

private:
    Eigen::Vector3f offset;
    surface_mesh::Color color;
    nanogui::GLShader lineShader;
    bool enabled;
    std::vector<surface_mesh::Point> line;
    bool strip_mode = false;
    bool apply_offset = true;
    bool updated = false;

};


#endif //CROISSANT_LINERENDERER_H
