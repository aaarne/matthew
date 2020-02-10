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
    void init() override;

protected:
    void do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) override;

private:
    nanogui::GLShader shader;
};


#endif //MATTHEW_VECTORFIELD_RENDERER_H
