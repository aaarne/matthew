//
// Created by sach_ar on 2/6/20.
//

#ifndef CROISSANT_POINT_RENDERER_H
#define CROISSANT_POINT_RENDERER_H


#include <nanogui/glutil.h>
#include <surface_mesh/types.h>
#include <mutex>
#include "renderer.h"

class PointRenderer : public Renderer {
public:
    explicit PointRenderer();
    explicit PointRenderer(Eigen::Vector3f offset);
    virtual void init() override;
    virtual void do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) override;
    void setPoint(const surface_mesh::Point &p);

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
