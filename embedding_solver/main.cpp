//
// Created by sach_ar on 10/8/21.
//

#include <iostream>
#include <csv.h>
#include <Eigen/Core>
#include <nanogui/common.h>
#include "embedding_solver.h"

using namespace std;

int main(int argc, char **argv) {
    io::CSVReader<2> vertex_reader("vertices.csv");
    double x, y;
    std::vector<std::tuple<double, double, double>> values;
    while(vertex_reader.read_row(x, y)) {
//        double z = sqrt(x*x + y*y);
        double z = 0;
        values.emplace_back(x, y, z);
    }

    io::CSVReader<3> face_reader("faces.csv");
    int v1, v2, v3;
    std::vector<std::tuple<int, int, int>> facesv;
    while(face_reader.read_row(v1, v2, v3)) {
        facesv.emplace_back(v1, v2, v3);
    }

    io::CSVReader<3> edge_reader("target_lengths.csv");
    std::vector<std::tuple<int, int, double>> tlv;
    double l;
    while(edge_reader.read_row(v1, v2, l)) {
        tlv.emplace_back(v1, v2, l);
    }

    Eigen::MatrixXd points(values.size(), 3);
    Eigen::MatrixXi faces(facesv.size(), 3);
    Eigen::MatrixXi edges(tlv.size(), 2);
    Eigen::VectorXd target_lengths(tlv.size());

    for (int i = 0; i<values.size(); i++) {
        const auto &r = values[i];
        points.row(i) << std::get<0>(r), std::get<1>(r), std::get<2>(r);
    }

    for (int i = 0; i<facesv.size(); i++) {
        const auto &r = facesv[i];
        faces.row(i) << std::get<0>(r), std::get<1>(r), std::get<2>(r);
    }

    for (int i = 0; i<tlv.size(); i++) {
        const auto &r = tlv[i];
        edges.row(i) << std::get<0>(r), std::get<1>(r);
        target_lengths(i) = std::get<2>(r);
    }

    nanogui::init();
    auto viewer = new EmbeddingSolver(points, faces, edges, target_lengths);
    matthew::run_app(viewer);
}