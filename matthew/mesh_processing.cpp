//
// Created by arne on 11/13/19.
//

#include "mesh_processing.h"

using namespace std;
using namespace Eigen;
using namespace surface_mesh;

void MatthewImpl::calc_weights() {
    calc_edges_weights();
    calc_vertices_weights();
}

void MatthewImpl::calc_edges_weights() {
    Surface_mesh::Halfedge h0, h1, h2;
    Surface_mesh::Vertex v0, v1;
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

void MatthewImpl::calc_vertices_weights() {
    Surface_mesh::Face_around_vertex_circulator vf_c, vf_end;
    Surface_mesh::Vertex_around_face_circulator fv_c;
    Scalar area;
    auto vweight = mesh.vertex_property<Scalar>("v:weight", 0);

    for (auto v: mesh.vertices()) {
        area = 0.0;
        vf_c = mesh.faces(v);

        if (!vf_c) {
            continue;
        }

        vf_end = vf_c;

        do {
            fv_c = mesh.vertices(*vf_c);

            const Point &P = mesh.position(*fv_c);
            ++fv_c;
            const Point &Q = mesh.position(*fv_c);
            ++fv_c;
            const Point &R = mesh.position(*fv_c);

            area += norm(cross(Q - P, R - P)) * 0.5f * 0.3333f;

        } while (++vf_c != vf_end);

        vweight[v] = 0.5f / area;
    }
}

void MatthewImpl::computeValence() {
    Surface_mesh::Vertex_property<Scalar> vertex_valence =
            mesh.vertex_property<Scalar>("v:valence", 0);
    for (auto v: mesh.vertices()) {
        vertex_valence[v] = mesh.valence(v);
    }
}

void MatthewImpl::computeNormalsWithConstantWeights() {
    using namespace surface_mesh;
    Point default_normal(0.0, 1.0, 0.0);
    Surface_mesh::Vertex_property<Point> v_cste_weights_n =
            mesh.vertex_property<Point>("v:cste_weights_n", default_normal);

    for (const auto &v : mesh.vertices()) {
        Vec3 normal(0, 0, 0);
        for (const Surface_mesh::Face &face : mesh.faces(v)) { // this uses the circulator
            normal += mesh.compute_face_normal(face);
        }
        v_cste_weights_n[v] = normal.normalize();
    }
}

void MatthewImpl::computeNormalsByAreaWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Surface_mesh::Vertex_property<Point> v_area_weights_n =
            mesh.vertex_property<Point>("v:area_weight_n", default_normal);

    for (const auto &v : mesh.vertices()) {
        Vec3 normal(0, 0, 0);
        for (const Surface_mesh::Face &face : mesh.faces(v)) {
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

void MatthewImpl::computeNormalsWithAngleWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Surface_mesh::Vertex_property<Point> v_angle_weights_n =
            mesh.vertex_property<Point>("v:angle_weight_n", default_normal);

    for (const auto &v : mesh.vertices()) {
        Vec3 normal(0, 0, 0);
        for (const Surface_mesh::Face &face : mesh.faces(v)) {
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

void MatthewImpl::calc_uniform_laplacian() {
    Surface_mesh::Vertex_property<Scalar> v_uniLaplace = mesh.vertex_property<Scalar>("v:uniLaplace", 0);
    Point laplace(0.0);

    for (const auto &v : mesh.vertices()) {
        laplace = 0;
        int N = 0;
        for (const auto &v2 : mesh.vertices(v)) {
            laplace += mesh.position(v2) - mesh.position(v);
            N++;
        }
        v_uniLaplace[v] = norm(laplace / N);
    }
}

void MatthewImpl::calc_mean_curvature() {
    Surface_mesh::Vertex_property<Scalar> v_curvature = mesh.vertex_property<Scalar>("v:curvature", 0);
    Surface_mesh::Edge_property<Scalar> e_weight = mesh.edge_property<Scalar>("e:weight", 0);
    Surface_mesh::Vertex_property<Scalar> v_weight = mesh.vertex_property<Scalar>("v:weight", 0);
    Point laplace(0.0);

    for (const auto &v : mesh.vertices()) {
        laplace = 0;
        for (const auto &v2 : mesh.vertices(v)) {
            Surface_mesh::Edge e = mesh.find_edge(v, v2);
            laplace += e_weight[e] * (mesh.position(v2) - mesh.position(v));
        }
        laplace *= v_weight[v];
        v_curvature[v] = norm(laplace);
    }
}

void MatthewImpl::calc_gauss_curvature() {
    Surface_mesh::Vertex_property<Scalar> v_gauss_curvature = mesh.vertex_property<Scalar>("v:gauss_curvature", 0);
    Surface_mesh::Vertex_property<Scalar> v_weight = mesh.vertex_property<Scalar>("v:weight", 0);
    Surface_mesh::Vertex_around_vertex_circulator vv_c, vv_c2, vv_end;
    Point d0, d1;
    Scalar angles, cos_angle;
    Scalar lb(-1.0f), ub(1.0f);

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

        v_gauss_curvature[v] = float(2 * M_PI - angles) * 2.0f * v_weight[v];

    }

}

void MatthewImpl::loadMesh(string filename) {
    if (!mesh.read(filename)) {
        std::cerr << "Mesh not found, exiting." << std::endl;
        exit(-1);
    }

    cout << "Mesh " << filename << " loaded." << endl;
    n_vertices = mesh.n_vertices();
    cout << "# of vertices : " << n_vertices << endl;
    n_faces = mesh.n_faces();
    cout << "# of faces : " << n_faces << endl;
    n_edges = mesh.n_edges();
    cout << "# of edges : " << n_edges << endl;

    mesh_center = computeCenter(&mesh);
    float dist_max = 0.0f;
    for (auto v: mesh.vertices()) {
        if (distance(mesh_center, mesh.position(v)) > dist_max) {
            dist_max = distance(mesh_center, mesh.position(v));
        }
    }

    mCamera.arcball = nanogui::Arcball();
    mCamera.arcball.setSize(mSize);
    mCamera.modelZoom = 2 / dist_max;
    mCamera.modelTranslation = -Vector3f(mesh_center.x, mesh_center.y, mesh_center.z);
}

void MatthewImpl::meshProcess() {
    Point default_normal(0.0, 1.0, 0.0);
    Surface_mesh::Vertex_property<Point> vertex_normal =
            mesh.vertex_property<Point>("v:normal");
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    v_color_valence = mesh.vertex_property<Color>("v:color_valence",
                                                  Color(1.0f, 1.0f, 1.0f));
    v_color_unicurvature = mesh.vertex_property<Color>("v:color_unicurvature",
                                                       Color(1.0f, 1.0f, 1.0f));
    v_color_curvature = mesh.vertex_property<Color>("v:color_curvature",
                                                    Color(1.0f, 1.0f, 1.0f));
    v_color_gaussian_curv = mesh.vertex_property<Color>("v:color_gaussian_curv",
                                                        Color(1.0f, 1.0f, 1.0f));

    auto vertex_valence = mesh.vertex_property<Scalar>("v:valence", 0);

    auto v_cste_weights_n = mesh.vertex_property<Point>("v:cste_weights_n");
    auto v_area_weights_n = mesh.vertex_property<Point>("v:area_weight_n");
    auto v_angle_weights_n = mesh.vertex_property<Point>("v:angle_weight_n");

    auto v_uniLaplace = mesh.vertex_property<Scalar>("v:uniLaplace", 0);
    auto v_curvature = mesh.vertex_property<Scalar>("v:curvature", 0);
    auto v_gauss_curvature = mesh.vertex_property<Scalar>("v:gauss_curvature", 0);

    calc_weights();
    calc_uniform_laplacian();
    calc_mean_curvature();
    calc_gauss_curvature();
    computeValence();
    computeNormalsWithConstantWeights();
    computeNormalsByAreaWeights();
    computeNormalsWithAngleWeights();
    color_coding(vertex_valence, &mesh, v_color_valence, 100);
    color_coding(v_uniLaplace, &mesh, v_color_unicurvature);
    color_coding(v_curvature, &mesh, v_color_curvature);
    color_coding(v_gauss_curvature, &mesh, v_color_gaussian_curv);

    int j = 0;
    MatrixXf mesh_points(3, n_vertices);
    MatrixXi indices(3, n_faces);

    for (auto f: mesh.faces()) {
        vector<float> vv(3.0f);
        int k = 0;
        for (auto v: mesh.vertices(f)) {
            vv[k] = v.idx();
            ++k;
        }
        indices.col(j) << vv[0], vv[1], vv[2];
        ++j;
    }

    // Create big matrices to send the data to the GPU with the required
    // format
    MatrixXf color_valence_attrib(3, n_vertices);
    MatrixXf color_unicurvature_attrib(3, n_vertices);
    MatrixXf color_curvature_attrib(3, n_vertices);
    MatrixXf color_gaussian_curv_attrib(3, n_vertices);
    MatrixXf normals_attrib(3, n_vertices);
    MatrixXf normal_cste_weights_attrib(3, n_vertices);
    MatrixXf normal_area_weights_attrib(3, n_vertices);
    MatrixXf normal_angle_weights_attrib(3, n_vertices);

    j = 0;
    for (auto v: mesh.vertices()) {
        mesh_points.col(j) << mesh.position(v).x,
                mesh.position(v).y,
                mesh.position(v).z;
        color_valence_attrib.col(j) << v_color_valence[v].x,
                v_color_valence[v].y,
                v_color_valence[v].z;

        color_unicurvature_attrib.col(j) << v_color_unicurvature[v].x,
                v_color_unicurvature[v].y,
                v_color_unicurvature[v].z;

        color_curvature_attrib.col(j) << v_color_curvature[v].x,
                v_color_curvature[v].y,
                v_color_curvature[v].z;

        color_gaussian_curv_attrib.col(j) << v_color_gaussian_curv[v].x,
                v_color_gaussian_curv[v].y,
                v_color_gaussian_curv[v].z;

        normals_attrib.col(j) << vertex_normal[v].x,
                vertex_normal[v].y,
                vertex_normal[v].z;

        normal_cste_weights_attrib.col(j) << v_cste_weights_n[v].x,
                v_cste_weights_n[v].y,
                v_cste_weights_n[v].z;

        normal_area_weights_attrib.col(j) << v_area_weights_n[v].x,
                v_area_weights_n[v].y,
                v_area_weights_n[v].z;

        normal_angle_weights_attrib.col(j) << v_angle_weights_n[v].x,
                v_angle_weights_n[v].y,
                v_angle_weights_n[v].z;
        ++j;
    }

    mShader.bind();
    mShader.uploadIndices(indices);
    mShader.uploadAttrib("position", mesh_points);
    mShader.uploadAttrib("valence_color", color_valence_attrib);
    mShader.uploadAttrib("unicruvature_color", color_unicurvature_attrib);
    mShader.uploadAttrib("curvature_color", color_curvature_attrib);
    mShader.uploadAttrib("gaussian_curv_color", color_gaussian_curv_attrib);
    //mShader.uploadAttrib("normal", normals_attrib);
    mShader.uploadAttrib("n_cste_weights", normal_cste_weights_attrib);
    mShader.uploadAttrib("n_area_weights", normal_area_weights_attrib);
    mShader.uploadAttrib("n_angle_weights", normal_angle_weights_attrib);
    mShader.setUniform("color_mode", int(color_mode));
    mShader.setUniform("intensity", base_color);

    mShaderNormals.bind();
    mShaderNormals.uploadIndices(indices);
    mShaderNormals.uploadAttrib("position", mesh_points);
    mShaderNormals.uploadAttrib("n_cste_weights", normal_cste_weights_attrib);
    mShaderNormals.uploadAttrib("n_area_weights", normal_area_weights_attrib);
    mShaderNormals.uploadAttrib("n_angle_weights", normal_angle_weights_attrib);
    //mShaderNormals.uploadAttrib("normal", normals_attrib);
}


Point MatthewImpl::computeCenter(Surface_mesh *mesh) {
    Point center = Point(0.0f);

    for (auto v: mesh->vertices()) {
        center += mesh->position(v);
    }

    return center / mesh->n_vertices();
}