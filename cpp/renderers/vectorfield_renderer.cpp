//
// Created by arne on 2/10/20.
//

#include "vectorfield_renderer.h"

void VectorfieldRenderer::init() {

}

void VectorfieldRenderer::do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) {
    shader.bind();
    shader.setUniform("MV", mv);
    shader.setUniform("P", p);
}

