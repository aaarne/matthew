//
// Created by arne on 2/10/20.
//

#ifndef MATTHEW_RENDERER_H
#define MATTHEW_RENDERER_H

#include <surface_mesh/types.h>
#include <vector>

class Renderer {
public:
    virtual void init() = 0;
    virtual void setVisible(bool visible) {this->enabled = visible;}

    virtual void draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) final {
        if (this->enabled) do_draw(mv, p);
    }

protected:
    virtual void do_draw(const Eigen::Matrix4f &mv, const Eigen::Matrix4f &p) = 0;
    bool enabled = false;

    static Eigen::MatrixXf points_to_matrix(const std::vector<surface_mesh::Point> &points) {
        return points_to_matrix(points, Eigen::Vector3f::Zero());
    }

    static Eigen::MatrixXf points_to_matrix(const std::vector<surface_mesh::Point> &points, const Eigen::Vector3f &offset) {
        Eigen::MatrixXf p(3, points.size());
        int j = 0;
        for (const auto &v : points) {
            p.col(j++) << v.x, v.y, v.z;
        }
        p.colwise() += offset;
        return p;
    }
};

#endif //MATTHEW_RENDERER_H
