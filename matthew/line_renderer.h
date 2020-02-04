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
    LineRenderer() : enabled(true) {}

    void setVisible(bool visible) {this->enabled = visible;}
    void init();
    void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p);

    void show_lines(std::vector<surface_mesh::Point> &l);

    void show_isolines(const surface_mesh::Surface_mesh &mesh, const std::string &property_name, int n_intervals);

protected:
    void upload_line();

private:
    nanogui::GLShader lineShader;
    bool enabled;
    std::vector<surface_mesh::Point> line;

};


#endif //CROISSANT_LINERENDERER_H
