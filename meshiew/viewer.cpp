//=============================================================================
//
//   Code framework for the lecture
//
//   "Digital 3D Geometry Processing"
//
//   Gaspard Zoss
//
//   Copyright (C) 2016 by Computer Graphics and Geometry Laboratory,
//         EPF Lausanne
//
//   Edited 2017
//-----------------------------------------------------------------------------
#include "viewer.h"
#include <surface_mesh/Surface_mesh.h>

using std::min;
using std::max;
using namespace surface_mesh;

typedef Surface_mesh Mesh;

// ========================================================================
// NOTE : We've only included the functions you need to implement (or the
//        ones that you will need to use) in the cpp file. This is not the
//        best practice as you normaly would have all the implementation of
//        the functions here and only the declaration in the header file
//        but it allows you to have all the things you need here.
// ========================================================================

void Viewer::calc_weights() {
    calc_edges_weights();
    calc_vertices_weights();
}

void Viewer::calc_edges_weights() {
    Mesh::Halfedge h0, h1, h2;
    Mesh::Vertex v0, v1;
    Point p0, p1, p2, d0, d1;
    Scalar w;
    auto eweight = mesh.edge_property<Scalar>("e:weight", 0);
    for (auto e: mesh.edges()) {
        w = 0.0;

        h0 = mesh.halfedge(e, 0);
        v0 = mesh.to_vertex(h0);
        p0 = mesh.position(v0);

        h1 = mesh.halfedge(e, 1);
        v1 = mesh.to_vertex(h1);
        p1 = mesh.position(v1);

        h2 = mesh.next_halfedge(h0);
        p2 = mesh.position(mesh.to_vertex(h2));
        d0 = normalize(p0 - p2);
        d1 = normalize(p1 - p2);
        w += 1.0 / tan(acos(min(0.99f, max(-0.99f, dot(d0, d1)))));

        h2 = mesh.next_halfedge(h1);
        p2 = mesh.position(mesh.to_vertex(h2));
        d0 = normalize(p0 - p2);
        d1 = normalize(p1 - p2);
        w += 1.0 / tan(acos(min(0.99f, max(-0.99f, dot(d0, d1)))));

        w = max(0.0f, w);
        eweight[e] = w * 0.5f;
    }
}

void Viewer::calc_vertices_weights() {
    Mesh::Face_around_vertex_circulator vf_c, vf_end;
    Mesh::Vertex_around_face_circulator fv_c;
    Scalar area;
    auto vweight = mesh.vertex_property<Scalar>("v:weight", 0);

    for (auto v: mesh.vertices()) {
        area = 0.0;
        vf_c = mesh.faces(v);

        if(!vf_c) {
            continue;
        }

        vf_end = vf_c;

        do {
            fv_c = mesh.vertices(*vf_c);

            const Point& P = mesh.position(*fv_c);  ++fv_c;
            const Point& Q = mesh.position(*fv_c);  ++fv_c;
            const Point& R = mesh.position(*fv_c);

            area += norm(cross(Q-P, R-P)) * 0.5f * 0.3333f;

        } while(++vf_c != vf_end);

        vweight[v] = 0.5f / area;
    }
}

void Viewer::computeValence() {
   Mesh::Vertex_property<Scalar> vertex_valence =
            mesh.vertex_property<Scalar>("v:valence", 0);
    for (auto v: mesh.vertices()) {
        vertex_valence[v] = mesh.valence(v);
    }
}
// ========================================================================
// EXERCISE 1.1
// ========================================================================
void Viewer::computeNormalsWithConstantWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Mesh::Vertex_property<Point> v_cste_weights_n =
            mesh.vertex_property<Point>("v:cste_weights_n", default_normal);

    // ------------- IMPLEMENT HERE ---------
    // Compute the normals for each vertex v in the mesh using the constant weights
    // technique (see .pdf) and store it inside v_cste_weights_n[v]
    // ------------- IMPLEMENT HERE ---------
    for (const auto &v : mesh.vertices()) {
        Vec3 normal(0, 0, 0);
        for (const Mesh::Face &face : mesh.faces(v)) { // this uses the circulator
            normal += mesh.compute_face_normal(face);
        }
        v_cste_weights_n[v] = normal.normalize();
    }
}
// ========================================================================
// EXERCISE 1.2
// ========================================================================
void Viewer::computeNormalsByAreaWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Mesh::Vertex_property<Point> v_area_weights_n =
            mesh.vertex_property<Point>("v:area_weight_n", default_normal);

    // ------------- IMPLEMENT HERE ---------
    // Compute the normals for each vertex v in the mesh using the weights proportionals
    // to the areas technique (see .pdf) and store inside v_area_weights_n[v]
    // ------------- IMPLEMENT HERE ---------
    for (const auto &v : mesh.vertices()) {
        Vec3 normal(0, 0, 0);
        for (const Mesh::Face &face : mesh.faces(v)) {
            vector<Point> vertices;

            for (const auto &vertex : mesh.vertices(face)) {
                vertices.push_back(mesh.position(vertex));
            }

            assert(vertices.size() == 3);

            double A = .5 * norm(cross(
                        vertices[1] - vertices[0],
                        vertices[2] - vertices[0]
                    ));

            normal += A * mesh.compute_face_normal(face);
        }
        v_area_weights_n[v] = normal.normalize();
    }
}
// ========================================================================
// EXERCISE 1.3
// ========================================================================
void Viewer::computeNormalsWithAngleWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Mesh::Vertex_property<Point> v_angle_weights_n =
            mesh.vertex_property<Point>("v:angle_weight_n", default_normal);

    // ------------- IMPLEMENT HERE ---------
    // Compute the normals for each vertex v in the mesh using the weights proportionals
    // to the angles technique (see .pdf) and store it inside v_angle_weights_n[v]
    // ------------- IMPLEMENT HERE ---------
    for (const auto &v : mesh.vertices()) {
        Vec3 normal(0, 0, 0);
        for (const Mesh::Face &face : mesh.faces(v)) {
            vector<Point> vertices;

            for (const auto &vertex : mesh.vertices(face)) {
                vertices.push_back(mesh.position(vertex));
            }

            Vec3 a(vertices[1] - vertices[0]);
            Vec3 b(vertices[2] - vertices[0]);

            double alpha = acos(min(0.99f, max(-0.99f, dot(a, b) / (norm(a) * norm(b)))));

            normal += alpha * mesh.compute_face_normal(face);
        }
        v_angle_weights_n[v] = normal.normalize();
    }
}
// ========================================================================
// EXERCISE 2.1
// ========================================================================
void Viewer::calc_uniform_laplacian() {
    Mesh::Vertex_property<Scalar> v_uniLaplace = mesh.vertex_property<Scalar>("v:uniLaplace", 0);
    Point             laplace(0.0);
    min_uniLaplace = 1000;
    max_uniLaplace = -1000;

    // ------------- IMPLEMENT HERE ---------
    // For each non-boundary vertex, compute uniform Laplacian operator vector
    // and store the vector length in the vertex property of the
    // mesh called v_uniLaplace[v].
    // Store min and max values of v_uniLaplace[v] in min_uniLaplace and max_uniLaplace.
    // ------------- IMPLEMENT HERE ---------
    for (const auto &v : mesh.vertices()) {
        laplace = 0;
        int N = 0;
        for (const auto &v2 : mesh.vertices(v)) {
            laplace += mesh.position(v2) - mesh.position(v);
            N++;
        }
        v_uniLaplace[v] = norm(laplace/N);
    }
    max_uniLaplace = *std::max_element(v_uniLaplace.vector().begin(), v_uniLaplace.vector().end());
    min_uniLaplace = *std::min_element(v_uniLaplace.vector().begin(), v_uniLaplace.vector().end());
}
// ========================================================================
// EXERCISE 2.2
// ========================================================================
void Viewer::calc_mean_curvature() {
    Mesh::Vertex_property<Scalar>  v_curvature = mesh.vertex_property<Scalar>("v:curvature", 0);
    Mesh::Edge_property<Scalar> e_weight = mesh.edge_property<Scalar>("e:weight", 0);
    Mesh::Vertex_property<Scalar>  v_weight = mesh.vertex_property<Scalar>("v:weight", 0);
    Point laplace(0.0);
    min_mean_curvature = 1000;
    max_mean_curvature = -1000;

    // ------------- IMPLEMENT HERE ---------
    // For all non-boundary vertices, approximate the mean curvature using
    // the length of the Laplace-Beltrami approximation.
    // Save your approximation in v_curvature vertex property of the mesh.
    // Use the weights from calc_weights(): e_weight and v_weight
    // ------------- IMPLEMENT HERE ---------

    for (const auto &v : mesh.vertices()) {
        laplace = 0;
        for (const auto &v2 : mesh.vertices(v)) {
            Mesh::Edge e = mesh.find_edge(v, v2);
            laplace += e_weight[e] * (mesh.position(v2) - mesh.position(v));
        }
        laplace *= v_weight[v];
        v_curvature[v] = norm(laplace);
    }
    max_mean_curvature = *std::max_element(v_curvature.vector().begin(), v_curvature.vector().end());
    min_mean_curvature = *std::min_element(v_curvature.vector().begin(), v_curvature.vector().end());
}
// ========================================================================
// EXERCISE 2.3
// ========================================================================
void Viewer::calc_gauss_curvature() {
    Mesh::Vertex_property<Scalar> v_gauss_curvature = mesh.vertex_property<Scalar>("v:gauss_curvature", 0);
    Mesh::Vertex_property<Scalar> v_weight = mesh.vertex_property<Scalar>("v:weight", 0);
    Mesh::Vertex_around_vertex_circulator vv_c, vv_c2, vv_end;
    Point d0, d1;
    Scalar angles, cos_angle;
    Scalar lb(-1.0f), ub(1.0f);
    min_gauss_curvature = 1000;
    max_gauss_curvature = -1000;

    // ------------- IMPLEMENT HERE ---------
    // For each non-boundary vertex, approximate Gaussian curvature,
    // and store it in the vertex property v_gauss_curvature.
    // Hint: When calculating angles out of cross products make sure the value
    // you pass to the acos function is between -1.0 and 1.0.
    // Use the v_weight property for the area weight.
    // ------------- IMPLEMENT HERE ---------

    for (const auto &v : mesh.vertices()) {
        if (mesh.is_boundary(v)) continue;

        angles = 0.0f;

        vv_c = mesh.vertices(v);
        vv_c2 = mesh.vertices(v);

        Vec3 pos = mesh.position(v);
        for (const auto &v1 : vv_c) {
            ++vv_c2; //this is safe as the circulator iterator is implemented circularly. i.e. incrementing end -> begin

            d0 = mesh.position(v1) - pos;
            d1 = mesh.position(*vv_c2) - pos;

            cos_angle = min(ub, max(lb, dot(d0, d1) / (norm(d0) * norm(d1))));
            angles += acos(cos_angle);
        }

        v_gauss_curvature[v] = float(2*M_PI - angles) * 2.0f * v_weight[v];

    }

    max_gauss_curvature = *std::max_element(v_gauss_curvature.vector().begin(), v_gauss_curvature.vector().end());
    min_gauss_curvature = *std::min_element(v_gauss_curvature.vector().begin(), v_gauss_curvature.vector().end());
}

