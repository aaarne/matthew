//
// Created by arne on 11/20/19.
//

#include "myloadmsh.h"
#include "pymesh/MshLoader.h"
#include <iostream>
#include <memory>

using namespace std;

bool load(PyMesh::MshLoader *loader, surface_mesh::Surface_mesh &mesh) {
    mesh.clear();

    auto vertices = loader->get_nodes().cast<float>();

    for (long i = 0; i < vertices.size(); i += 3) {
        mesh.add_vertex({vertices(i), vertices(i + 1), vertices(i + 2)});
    }

    auto indices = loader->get_elements();

    for (long i = 0; i < indices.size(); i += loader->get_nodes_per_element()) {
        vector<surface_mesh::Surface_mesh::Vertex> face;
        for (long j = i; j < i + loader->get_nodes_per_element(); j++) {
            face.emplace_back(indices(j));
        }
        mesh.add_face(face);
    }

    for (const auto &name : loader->get_node_field_names()) {
        auto field = loader->get_node_field(name);
        int property_dimension = field.rows() / mesh.n_vertices();
        if (property_dimension == 1) {
            surface_mesh::Surface_mesh::Vertex_property<surface_mesh::Scalar> prop = mesh.vertex_property(name, 0.0f);
            for (const auto &v : mesh.vertices()) {
                prop[v] = field(v.idx());
            }
        } else if (property_dimension == 3) {
            surface_mesh::Surface_mesh::Vertex_property<surface_mesh::Point> prop = mesh.vertex_property(name, surface_mesh::Point(0.0));
            int j = 0;
            for (const auto &v : mesh.vertices()) {
                prop[v] = {static_cast<float>(field(3*v.idx())),
                           static_cast<float>(field(3*v.idx()+1)),
                           static_cast<float>(field(3*v.idx()+2))};
            }
        }
    }
    return true;

}

bool loadmsh::load_msh_file(const std::string &filename, surface_mesh::Surface_mesh &mesh) {
    std::shared_ptr<PyMesh::MshLoader> loader;
    try {
        loader = std::make_shared<PyMesh::MshLoader>(filename);
    } catch (std::exception &e) {
        cerr << e.what() << endl;
        return false;
    }
    return load(loader.get(), mesh);
}

bool loadmsh::load_msh(std::istream &fin, surface_mesh::Surface_mesh &mesh) {
    auto loader = std::make_shared<PyMesh::MshLoader>(fin);
    return load(loader.get(), mesh);
}

