//
// Created by sach_ar on 2/4/20.
//

#include "line_renderer.h"
#include "shaders_gen.h"

void LineRenderer::init() {
    lineShader.init("line_shader", shaders::line_shader_verts, shaders::line_shader_frags);
}


void LineRenderer::show_line(std::vector<surface_mesh::Point> &l) {
    this->line = l;
    upload_line();
}

void LineRenderer::upload_line() {
    Eigen::MatrixXf p(3, this->line.size());
    int j = 0;
    for (const auto &v : this->line) {
        p.col(j++) << v.x, v.y, v.z;
    }
    lineShader.bind();
    lineShader.uploadAttrib("position", p);
}

void LineRenderer::draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) {
    if (this->enabled) {
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(2.0);
        lineShader.bind();
        lineShader.setUniform("MV", mv);
        lineShader.setUniform("P", p);
        lineShader.drawArray(GL_LINES, 0, line.size());
        glDisable(GL_LINE_SMOOTH);
    }
}
