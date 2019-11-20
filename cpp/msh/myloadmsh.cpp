//
// Created by arne on 11/20/19.
//

#include "myloadmsh.h"
#include "MshLoader.h"
#include <iostream>
#include <memory>

using namespace std;

bool loadmsh::load_msh_file(const std::string &filename, surface_mesh::Surface_mesh &mesh) {
    std::shared_ptr<PyMesh::MshLoader> loader;
    try {
        loader = std::make_shared<PyMesh::MshLoader>(filename);
    } catch (std::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

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

    cout << "Mesh has these vertex properties" << endl;
    for (const auto &name : loader->get_node_field_names()) {
        cout << "\t" << name << endl;
        auto field = loader->get_node_field(name);
        surface_mesh::Surface_mesh::Vertex_property<surface_mesh::Scalar> prop = mesh.vertex_property(name, 0.0f);
        for (const auto &v : mesh.vertices()) {
            prop[v] = field(v.idx());
        }
    }
    return true;
}
