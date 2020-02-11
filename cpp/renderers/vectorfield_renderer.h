//
// Created by arne on 2/10/20.
//

#ifndef MATTHEW_VECTORFIELD_RENDERER_H
#define MATTHEW_VECTORFIELD_RENDERER_H


#include <Eigen/Eigen>
#include "renderer.h"
#include <nanogui/glutil.h>

class VectorfieldRenderer : public Renderer {
public:
    explicit VectorfieldRenderer() : VectorfieldRenderer(Eigen::Vector3f(1.0, 0.0, 0.0)) {}
    explicit VectorfieldRenderer(const Eigen::Vector3f &color) : color(color) {}
    void init() override;
    void show_vectorfield(const Eigen::MatrixXf &p, const Eigen::MatrixXf &v);
    void show_vectorfield(const Eigen::MatrixXf &p, const Eigen::MatrixXf &v, const Eigen::MatrixXf &colors);
    void set_color(const Eigen::Vector3f &color);
    void set_scaling(float scaling);
    float get_scaling() const {return this->scaling;}

protected:
    void do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) override;

private:
    bool updated = false;
    bool color_prop = false;
    float scaling = 1.0f;
    Eigen::MatrixXf points;
    Eigen::MatrixXf vecs;
    Eigen::Vector3f color;
    Eigen::MatrixXf colors;
    nanogui::GLShader shader;
};


#endif //MATTHEW_VECTORFIELD_RENDERER_H
