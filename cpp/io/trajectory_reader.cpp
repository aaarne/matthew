//
// Created by sach_ar on 2/5/20.
//

#include "trajectory_reader.h"
#include <csv.h>

std::vector<surface_mesh::Point> trajectory_reader::read(const std::string &filename, bool no_header) {
    io::CSVReader<3> in(filename);
    if (!no_header) {
        in.read_header(io::ignore_extra_column, "x", "y", "z");
    }
    float x, y, z;
    std::vector<surface_mesh::Point> points;

    while(in.read_row(x, y, z)) {
        points.emplace_back(x, y, z);
    }
    return points;
}

