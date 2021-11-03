//
// Created by sach_ar on 10/8/21.
//

#include <iostream>
#include <csv.h>
#include <Eigen/Core>
#include <nanogui/common.h>
#include "embedding_solver.h"

using namespace std;

void load_problem_from_file(Eigen::MatrixXd &, Eigen::MatrixXi &, Eigen::MatrixXi &, Eigen::VectorXd &);
void toy_problem(Eigen::MatrixXd &, Eigen::MatrixXi &, Eigen::MatrixXi &, Eigen::VectorXd &);

int main(int argc, char **argv) {
    Eigen::MatrixXd points;
    Eigen::MatrixXi faces;
    Eigen::MatrixXi edges;
    Eigen::VectorXd target_lengths;

    load_problem_from_file(points, faces, edges, target_lengths);
//    toy_problem(points, faces, edges, target_lengths);

    nanogui::init();
    auto viewer = new EmbeddingSolver(points, faces, edges, target_lengths);
    viewer->setLazyMode(true);
    matthew::run_app(viewer);
}

void load_problem_from_file(Eigen::MatrixXd &points, Eigen::MatrixXi &faces, Eigen::MatrixXi &edges,
                            Eigen::VectorXd &target_lengths) {
    io::CSVReader<2> vertex_reader("vertices.csv");
    double x, y;
    std::vector<std::tuple<double, double, double>> values;
    while (vertex_reader.read_row(x, y)) {
        double z = 0; //x * x + y * y;
        values.emplace_back(x, y, z);
    }

    io::CSVReader<3> face_reader("faces.csv");
    int v1, v2, v3;
    std::vector<std::tuple<int, int, int>> facesv;
    while (face_reader.read_row(v1, v2, v3)) {
        facesv.emplace_back(v1, v2, v3);
    }

    io::CSVReader<3> edge_reader("target_lengths.csv");
    std::vector<std::tuple<int, int, double>> tlv;
    double l;
    while (edge_reader.read_row(v1, v2, l)) {
        tlv.emplace_back(v1, v2, l);
    }

    points.resize(values.size(), 3);
    faces.resize(facesv.size(), 3);
    edges.resize(tlv.size(), 2);
    target_lengths.resize(tlv.size());

    for (int i = 0; i < values.size(); i++) {
        const auto &r = values[i];
        points.row(i) << std::get<0>(r), std::get<1>(r), std::get<2>(r);
    }

    for (int i = 0; i < facesv.size(); i++) {
        const auto &r = facesv[i];
        faces.row(i) << std::get<0>(r), std::get<1>(r), std::get<2>(r);
    }

    for (int i = 0; i < tlv.size(); i++) {
        const auto &r = tlv[i];
        edges.row(i) << std::get<0>(r), std::get<1>(r);
        target_lengths(i) = std::get<2>(r);
    }

}

void toy_problem(Eigen::MatrixXd &v, Eigen::MatrixXi &f, Eigen::MatrixXi &e, Eigen::VectorXd &tl) {
    v.resize(4, 3);
    f.resize(2, 3);
    e.resize(5, 2);
    tl.resize(4);
    v << 0, 0, 0,
            1, 0.5, 0,
            0, 1, 0,
            -1, 0.5, 0.5;
    f << 0, 1, 2,
            2, 3, 0;
    e << 0, 1,
            1, 2,
            2, 3,
            0, 2,
            3, 0;
    tl << 1, 1, 1, 1, 1;
}
