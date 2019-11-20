//
// Created by arne on 11/20/19.
//

#ifndef CROISSANT_MYLOADMSH_H
#define CROISSANT_MYLOADMSH_H

#include <string>
#include <surface_mesh/Surface_mesh.h>

namespace loadmsh {

    bool load_msh_file(const std::string &filename, surface_mesh::Surface_mesh &mesh);

};


#endif //CROISSANT_MYLOADMSH_H
