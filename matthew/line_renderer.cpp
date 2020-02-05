//
// Created by sach_ar on 2/4/20.
//

#include "line_renderer.h"
#include "shaders_gen.h"
#include <cmath>

using namespace surface_mesh;

void LineRenderer::init() {
    lineShader.init("line_shader", shaders::line_shader_verts, shaders::line_shader_frags);
    lineShader.bind();
    Eigen::Vector3f line_color;
    line_color << color.x, color.y, color.z;
    lineShader.setUniform("line_color", line_color);
}


void LineRenderer::show_lines(std::vector<Point> &l) {
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

std::pair<long, long> interval_borders(float a, float b, float l, float size) {
    return {
            ceilf((std::min(a, b) - l) / size),
            floorf((std::max(a, b) - l) / size)
    };
}

long index_of_crossing_border(const std::pair<long, long> &borders01,
                              const std::pair<long, long> &borders02) {
    for (long index = borders01.first; index <= borders01.second; index++) {
        if (index >= borders02.first && index <= borders02.second) {
            return index;
        }
    }
    return -1;
}

std::pair<Point, Point> create_isoline_segment(long index,
                                               const float iso0, const float iso1, const float iso2,
                                               const Point &p0, const Point &p1, const Point &p2,
                                               float l, float size) {
    float ratio1 = (l + index * size - iso0) / (iso1 - iso0);
    float ratio2 = (l + index * size - iso0) / (iso2 - iso0);
    return {
            p0 + (p1 - p0) * ratio1,
            p0 + (p2 - p0) * ratio2
    };
}

void LineRenderer::show_isolines(const Surface_mesh &mesh, const std::string &property_name, int n_intervals) {
    /*
     * Warning: Close your eyes! Ugly code ahead
     */
    auto value = mesh.get_vertex_property<Scalar>(property_name);
    auto p = value.vector();
    float lb = *std::min_element(p.begin(), p.end());
    float ub = *std::max_element(p.begin(), p.end());
    float interval_size = (ub - lb) / ((float) n_intervals);

    std::vector<std::vector<Surface_mesh::Vertex>> triangle_ids;
    std::vector<Point> line_segments;

    for (const auto &f : mesh.faces()) {
        std::vector<Surface_mesh::Vertex> vv(3);
        int k = 0;
        for (const auto &v : mesh.vertices(f)) {
            vv[k++] = v;
        }
        triangle_ids.push_back(vv);
    }

    for (auto &triangle_id : triangle_ids) {
        std::vector<Surface_mesh::Vertex> vv(3);
        for (long k = 0; k < triangle_id.size(); k++) vv[k] = triangle_id[k];

        Scalar iso0, iso1, iso2;
        iso0 = value[vv[0]];
        iso1 = value[vv[1]];
        iso2 = value[vv[2]];

        Point v0 = mesh.position(vv[0]);
        Point v1 = mesh.position(vv[1]);
        Point v2 = mesh.position(vv[2]);

        auto borders01 = interval_borders(iso0, iso1, lb, interval_size);
        auto borders12 = interval_borders(iso1, iso2, lb, interval_size);
        auto borders02 = interval_borders(iso0, iso2, lb, interval_size);

        long i1 = index_of_crossing_border(borders01, borders02);
        long i2 = index_of_crossing_border(borders01, borders12);
        long i3 = index_of_crossing_border(borders02, borders12);

        if (i1 >= 0) {
            auto p = create_isoline_segment(i1, iso0, iso1, iso2, v0, v1, v2, lb, interval_size);
            line_segments.emplace_back(p.first);
            line_segments.emplace_back(p.second);
        }
        if (i2 >= 0) {
            auto p = create_isoline_segment(i2, iso1, iso0, iso2, v1, v0, v2, lb, interval_size);
            line_segments.emplace_back(p.first);
            line_segments.emplace_back(p.second);
        }
        if (i3 >= 0) {
            auto p = create_isoline_segment(i3, iso2, iso0, iso1, v2, v0, v1, lb, interval_size);
            line_segments.emplace_back(p.first);
            line_segments.emplace_back(p.second);
        }
    }

    show_lines(line_segments);
}

void LineRenderer::draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) {
    if (this->enabled) {
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1.0);
        lineShader.bind();
        lineShader.setUniform("MV", mv);
        lineShader.setUniform("P", p);
        lineShader.drawArray(GL_LINES, 0, line.size());
        glDisable(GL_LINE_SMOOTH);
    }
}
