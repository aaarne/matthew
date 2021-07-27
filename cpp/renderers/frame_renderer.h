//
// Created by arne on 7/14/21.
//

#ifndef MATTHEW_FRAME_RENDERER_H
#define MATTHEW_FRAME_RENDERER_H


#include <Eigen/Core>
#include <memory>
#include "renderer.h"
#include "line_renderer.h"
#include "point_renderer.h"

class FrameRenderer : public Renderer {
public:
    explicit FrameRenderer();

    virtual void init() override;
    virtual void do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) override;

    void setVisible(bool visible) override;

    void show_frame(const Eigen::Ref<const Eigen::Matrix<float, 4, 4>> frame);

    float getScaling() const {return scaling;}
    void setScaling(float s) {this->scaling = s; compute();}

    void set_color(const Eigen::Vector3f &color) {point_renderer.setColor(color);}

protected:
    virtual void compute();
    Eigen::Matrix<float, 4, 4> frame;
    LineRenderer lrx, lry, lrz;
    std::vector<LineRenderer*> all = {&lrx, &lry, &lrz};
    float scaling = 1.0;

    PointRenderer point_renderer;
};


#endif //MATTHEW_FRAME_RENDERER_H
