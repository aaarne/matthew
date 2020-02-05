//
// Created by sach_ar on 2/5/20.
//

#ifndef CROISSANT_TRAJECTORY_READER_H
#define CROISSANT_TRAJECTORY_READER_H


#include <string>
#include <vector>
#include <surface_mesh/types.h>

namespace trajectory_reader {
    std::vector<surface_mesh::Point> read(const std::string &filename, bool no_header = false);
};


#endif //CROISSANT_TRAJECTORY_READER_H
