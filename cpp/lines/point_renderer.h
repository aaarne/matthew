//
// Created by sach_ar on 2/6/20.
//

#ifndef CROISSANT_POINT_RENDERER_H
#define CROISSANT_POINT_RENDERER_H


#include <nanogui/glutil.h>
#include <surface_mesh/types.h>
#include <mutex>

class PointRenderer {
public:
    explicit PointRenderer(Eigen::Vector3f offset);
    void init();
    void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p);
    void setPoint(const surface_mesh::Point &p);
    void addPoint(const surface_mesh::Point &p);

    bool is_initialized() const {return this->initialized; }
    void clear();

    std::vector<surface_mesh::Point> trace() const;

private:
    Eigen::Vector3f offset;
    surface_mesh::Point point;
    bool updated = false;
    nanogui::GLShader shader;
    bool initialized = false;
    std::vector<surface_mesh::Point> points;
};


#endif //CROISSANT_POINT_RENDERER_H
