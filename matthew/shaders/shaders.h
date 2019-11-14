//
// Created by sach_ar on 11/14/19.
//

#ifndef MATTHEW_SHADERS_H
#define MATTHEW_SHADERS_H

namespace matthew {
    const std::string simple_vertex_source =
            #include "simple_vertex"
            ;

    const std::string simple_fragment_source =
            #include "simple_fragment"
            ;

    const std::string normals_vertex_source =
            #include "normals_vertex"
            ;

    const std::string normals_fragment_source =
            #include "normals_fragment"
            ;

    const std::string normals_geometry_source =
            #include "normals_geometry"
            ;
    }

#endif //MATTHEW_SHADERS_H
