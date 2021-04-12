//
// Created by arne on 11/13/19.
//

#include <shaders_gen.h>
#include "meshiew.h"
#include <chrono>
#include <algorithm>
#include "myloadmsh.h"
#include "trajectory_reader.h"
#include <nanogui/opengl.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/colorpicker.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/textbox.h>
#include <nanogui/combobox.h>
#include <nanogui/slider.h>
#include <numeric>
#include "pcd_reader.h"

#define DEFAULT_AMBIENT 0.3
#define DEFAULT_DIFFUSE 1.0
#define DEFAULT_SPECULAR 0.0
#define DEFAULT_OPACITY 1.0

using namespace std;
using namespace Eigen;

using surface_mesh::Surface_mesh;
using surface_mesh::Vec3;
using surface_mesh::Point;

void Meshiew::calc_weights() {
    calc_edges_weights();
    calc_vertices_weights();
}

template<typename T>
static inline double Lerp(T v0, T v1, T t) {
    return (1 - t) * v0 + t * v1;
}


void Meshiew::remesh(const REMESHING_TYPE &remeshing_type,
                     const int &num_iterations) {

    for (int i = 0; i < num_iterations; ++i) {
        cout << "----------- iteration " << i + 1 << " out of " << num_iterations << " -----------" << endl;
        cout << "Amount of vertices before remeshing: " << this->mesh.n_vertices() << "." << endl;

      	calc_target_length(remeshing_type);
      	calc_weights();
      	calc_mean_curvature();
      	calc_uniform_laplacian();
      	calc_gauss_curvature();
              split_long_edges();
              collapse_short_edges();
              equalize_valences();
              tangential_relaxation();
          }

        calc_weights();
        calc_mean_curvature();
        calc_uniform_laplacian();
        calc_gauss_curvature();
        cout << "Amount of vertices after remeshing: " << this->mesh.n_vertices() << "." << endl;
}

void Meshiew::calc_target_length(const REMESHING_TYPE &remeshing_type) {
    Surface_mesh::Vertex_iterator v_it, v_end(mesh.vertices_end());
    Scalar length;

    Surface_mesh::Vertex_property<Scalar> target_length = mesh.vertex_property<Scalar>("v:length", 0);
    Surface_mesh::Vertex_property<Scalar> new_target_length = mesh.vertex_property<Scalar>("v:new_length", 0);

    Surface_mesh::Vertex_property<Scalar> max_curvature = mesh.vertex_property<Scalar>("v:max_curvature", 0);


    std::vector<double> lengths;
    for (const auto &e : mesh.edges()) {
        lengths.push_back(mesh.edge_length(e));
    }

    const float TARGET_LENGTH = std::accumulate(lengths.begin(), lengths.end(), 0.0) / lengths.size();
    cout << "Target length is: " << TARGET_LENGTH << endl;

    if (remeshing_type == AVERAGE) {
        for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
            target_length[*v_it] = TARGET_LENGTH;

    } else if (remeshing_type == CURV) {
        for (const auto &v : mesh.vertices()) {
            length = TARGET_LENGTH;
            if (mesh.is_boundary(v)) {
                length = TARGET_LENGTH;
            } else {
                length = TARGET_LENGTH / max_curvature[v];
            }
            target_length[v] = length;
        }

        const int smoothing_iterations = 5;
        for (int i = 0; i < smoothing_iterations; i++) {
            for (const auto &vi : mesh.vertices()) {
                float local_target_length_avg = 0.f;
                for (const auto &vj : mesh.vertices(vi)) {
                    local_target_length_avg += target_length[vj] - target_length[vi];
                }
                local_target_length_avg /= mesh.valence(vi);
                new_target_length[vi] = target_length[vi] + local_target_length_avg;
                assert(target_length[vi] > 0);
            }

            for (const auto &v : mesh.vertices()) {
                target_length[v] = new_target_length[v];
            }

        }

        float mean_length = 0;
        for (const auto &v : mesh.vertices()) {
            mean_length += target_length[v];
        }
        mean_length /= mesh.n_vertices();

        const double rescale_factor = TARGET_LENGTH / mean_length;
        for (const auto &v : mesh.vertices())
            target_length[v] *= rescale_factor;
    }
}

void Meshiew::split_long_edges() {
    Surface_mesh::Edge_iterator e_it, e_end(mesh.edges_end());
    Surface_mesh::Vertex v0, v1, v;
    bool finished;
    int i;
    int c = 0;

    const double upper_ratio = 4. / 3;

    Surface_mesh::Vertex_property<Point> normals = mesh.vertex_property<Point>("v:normal");
    Surface_mesh::Vertex_property<Scalar> target_length = mesh.vertex_property<Scalar>("v:length", 0);

    for (finished = false, i = 0; !finished && i < 100; ++i) {
        finished = true;

        for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            v0 = mesh.vertex(*e_it, 0);
            v1 = mesh.vertex(*e_it, 1);
            float desired_length = .5f * target_length[v0] + .5f * target_length[v1];
            if (mesh.edge_length(*e_it) > upper_ratio * desired_length) {
                finished = false;
                v = mesh.split(*e_it, mesh.position(v0) + .5 * (mesh.position(v1) - mesh.position(v0)));
                normals[v] = .5 * normals[v0] + .5 * normals[v1];
                target_length[v] = desired_length;
                c++;
            }
        }
    }
    cout << "Split " << c << " long edges in " << i << " iterations." << endl;
}

void Meshiew::collapse_short_edges() {
    Surface_mesh::Edge_iterator e_it, e_end(mesh.edges_end());
    Surface_mesh::Vertex v0, v1;
    Surface_mesh::Halfedge h01, h10, h;
    bool finished;
    int i, c = 0;

    const double lower_ratio = 4. / 5;

    Surface_mesh::Vertex_property<Scalar> target_length = mesh.vertex_property<Scalar>("v:length", 0);

    for (finished = false, i = 0; !finished && i < 100; ++i) {
        finished = true;

        for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            if (!mesh.is_deleted(*e_it)) {
                h01 = mesh.halfedge(*e_it, 0);
                h10 = mesh.halfedge(*e_it, 1);

                // to vertex give the vertex the halfedge points to
                v0 = mesh.to_vertex(h10);
                v1 = mesh.to_vertex(h01);

                float desired_length = .5f * target_length[v0] + .5f * target_length[v1];

                if (mesh.edge_length(*e_it) < lower_ratio * desired_length) {

                    h = Surface_mesh::Halfedge();

                    if (mesh.is_boundary(v0) && !mesh.is_boundary(v1)) {
                        h = h10;
                    } else if (!mesh.is_boundary(v0) && mesh.is_boundary(v1)) {
                        h = h01;
                    } else {
                        if (mesh.valence(v0) < mesh.valence(v1)) {
                            h = h01;
                        } else if (mesh.valence(v0) > mesh.valence(v1)) {
                            h = h10;
                        } else {
                            if (mesh.is_collapse_ok(h01)) h = h01;
                            else if (mesh.is_collapse_ok(h10)) h = h10;
                            else continue;
                        }
                    }

                    if (mesh.is_collapse_ok(h)) {
                        finished = false;
                        mesh.collapse(h);
                        c++;
                    }
                }
            }
        }
    }

    cout << "Collapsed " << c << " short edges in " << i << " iterations." << endl;
    mesh.garbage_collection();

    if (i == 100) std::cerr << "collapse break\n";
}

void Meshiew::equalize_valences() {
    Surface_mesh::Edge_iterator e_it, e_end(mesh.edges_end());
    Surface_mesh::Vertex v0, v1, v2, v3;
    Surface_mesh::Halfedge h;
    int ve0, ve1, ve2, ve3, ve_before, ve_after;
    bool finished;
    int i;
    int c = 0;

    for (finished = false, i = 0; !finished && i < 100; ++i) {
        finished = true;

        for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            if (!mesh.is_boundary(*e_it) && !mesh.is_deleted(*e_it)) {
                auto h01 = mesh.halfedge(*e_it, 0);
                auto h10 = mesh.opposite_halfedge(h01);

                v0 = mesh.to_vertex(h10);
                v1 = mesh.to_vertex(h01);
                v2 = mesh.to_vertex(mesh.next_halfedge(h10));
                v3 = mesh.to_vertex(mesh.next_halfedge(h01));

                assert(mesh.to_vertex(mesh.next_halfedge(mesh.next_halfedge(h10))) == v1);
                assert(mesh.to_vertex(mesh.next_halfedge(mesh.next_halfedge(h01))) == v0);

                auto compute_deviation = [this](Surface_mesh::Vertex &v) -> int {
                    int opt = mesh.is_boundary(v) ? 4 : 6;
                    return mesh.valence(v) - opt;
                };

                ve0 = compute_deviation(v0);
                ve1 = compute_deviation(v1);
                ve2 = compute_deviation(v2);
                ve3 = compute_deviation(v3);

                auto sqr = [](int i) -> int { return i * i; }; //convenience square function
                ve_before = sqr(ve0) + sqr(ve1) + sqr(ve2) + sqr(ve3);
                ve_after = sqr(--ve0) + sqr(--ve1) + sqr(++ve2) + sqr(++ve3);

                if (ve_after < ve_before && mesh.is_flip_ok(*e_it)) {
                    finished = false;
                    mesh.flip(*e_it);
                    c++;
                }
            }
        }
    }

    cout << "Flipped " << c << " edges in " << i << " iterations." << endl;

    if (i == 100) std::cerr << "flip break\n";
}

void Meshiew::tangential_relaxation() {

    Surface_mesh::Vertex_iterator v_it, v_end(mesh.vertices_end());
    Point u, n;
    Point laplace;
    int c = 0;

    Surface_mesh::Vertex_property<Point> normals = mesh.vertex_property<Point>("v:normal");
    Surface_mesh::Vertex_property<Point> update = mesh.vertex_property<Point>("v:update");

// smooth
    for (int iters = 0; iters < 10; ++iters) {
        for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            if (!mesh.is_boundary(*v_it)) {
                laplace = Point(0.0);
                for (const auto &v1 : mesh.vertices(*v_it)) {
                    laplace += mesh.position(v1) - mesh.position(*v_it);
                }
                laplace /= mesh.valence(*v_it);

                Eigen::Vector3d normal, laplacian, proj;
                n = normals[*v_it];
                normal << n.x, n.y, n.z;
                laplacian << laplace.x, laplace.y, laplace.z;

                Eigen::Hyperplane<double, 3> tangentPlane(normal, 0);
                proj = tangentPlane.projection(laplacian);

                u = Point(proj(0), proj(1), proj(2));

                update[*v_it] = .1 * u;
            }
        }

        for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            if (!mesh.is_boundary(*v_it)) {
                c++;
                mesh.position(*v_it) += update[*v_it];
            }
        }
    }
    cout << "Moved " << c << " vertices." << endl;
}

template<typename T>
static inline std::vector<T> Quantile(const std::vector<T> &inData, const std::vector<T> &probs) {
    if (inData.empty()) {
        return std::vector<T>();
    }

    if (1 == inData.size()) {
        return std::vector<T>(1, inData[0]);
    }

    std::vector<T> data = inData;
    std::sort(data.begin(), data.end());
    std::vector<T> quantiles;

    for (size_t i = 0; i < probs.size(); ++i) {
        T poi = Lerp<T>(-0.5, data.size() - 0.5, probs[i]);

        size_t left = std::max(int64_t(std::floor(poi)), int64_t(0));
        size_t right = std::min(int64_t(std::ceil(poi)), int64_t(data.size() - 1));

        T datLeft = data.at(left);
        T datRight = data.at(right);
        T quantile = Lerp<T>(datLeft, datRight, poi - left);

        quantiles.push_back(quantile);
    }

    return quantiles;
}

void Meshiew::color_coding(Surface_mesh::Vertex_property<Scalar> prop, Surface_mesh *mesh,
                           Surface_mesh::Vertex_property<surface_mesh::Color> color_prop, int bound) {
    std::vector<Scalar> values = prop.vector();
    std::sort(values.begin(), values.end());

    std::vector<float> quantiles = show_minmax ? Quantile<float>(values, {
		    0.,
		    0.01,
		    0.33,
		    0.5,
		    0.66,
		    0.99,
		    1.
	    }) : Quantile<float>(values, {
		    0.,
		    0.,
		    0.33,
		    0.5,
		    0.66,
		    1.,
		    1.
	    });

    if (color_coding_window) {
        std::vector<double> v(quantiles.begin(), quantiles.end());
        color_coding_window->update_code(v);
    }
    // map values to colors
    for (auto v: mesh->vertices()) {
        color_prop[v] = value_to_color(prop[v], quantiles[1], quantiles[5], quantiles[0], quantiles[6]);
    }
}

void Meshiew::draw(Eigen::Matrix4f mv, Matrix4f p) {
    using namespace nanogui;
    mShader.bind();
    mShader.setUniform("MV", mv);
    mShader.setUniform("P", p);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0, 1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    mShader.setUniform("intensity", base_color);
    mShader.setUniform("light_color", light_color);
    mShader.setUniform("color_mode", (int) color_mode);
    if (not hide_mesh) {
        mShader.drawIndexed(GL_TRIANGLES, 0, mesh.n_faces());
    }

    if (wireframe) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        mShader.setUniform("color_mode", (int) COLOR_MODE::NORMAL);
        mShader.setUniform("intensity", edge_color);
        mShader.drawIndexed(GL_TRIANGLES, 0, mesh.n_faces());
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        mShader.setUniform("color_mode", (int) color_mode);
    }
}

surface_mesh::Color
Meshiew::value_to_color(Scalar value, Scalar min_value, Scalar max_value, Scalar bound_min, Scalar bound_max) {
    Scalar v0, v1, v2, v3, v4, vout0, vout1;
    vout0 = (min_value == bound_min) ? 1.0 : (value - bound_min) / (min_value - bound_min);
    v0 = min_value + 0.0 / 4.0 * (max_value - min_value);
    v1 = min_value + 1.0 / 4.0 * (max_value - min_value);
    v2 = min_value + 2.0 / 4.0 * (max_value - min_value);
    v3 = min_value + 3.0 / 4.0 * (max_value - min_value);
    v4 = min_value + 4.0 / 4.0 * (max_value - min_value);
    vout1 = (bound_max == max_value) ? 0.0 : (value - max_value) / (bound_max - max_value);

    surface_mesh::Color col(1.0f, 1.0f, 1.0f);

    if (value < v0) {
        col = surface_mesh::Color(0, 0, vout0);
    } else if (value > v4) {
        col = surface_mesh::Color(1, vout1, vout1);
    } else if (value <= v2) {
        if (value <= v1) { // [v0, v1]
            Scalar u = (value - v0) / (v1 - v0);
            col = surface_mesh::Color(0, u, 1);
        } else { // ]v1, v2]
            Scalar u = (value - v1) / (v2 - v1);
            col = surface_mesh::Color(0, 1, 1 - u);
        }
    } else {
        if (value <= v3) { // ]v2, v3]
            Scalar u = (value - v2) / (v3 - v2);
            col = surface_mesh::Color(u, 1, 0);
        } else { // ]v3, v4]
            Scalar u = (value - v3) / (v4 - v3);
            col = surface_mesh::Color(1, 1 - u, 0);
        }
    }
    return col;
}

void Meshiew::meshProcess() {
    using namespace surface_mesh;

    mesh.update_face_normals();
    mesh.update_vertex_normals();

    computeValence();
    calc_weights();
    calc_uniform_laplacian();
    calc_mean_curvature();
    calc_gauss_curvature();
    calc_principal_curvature_directions();
    calc_boundary();
}

void Meshiew::calc_edges_weights() {
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

void Meshiew::calc_vertices_weights() {
    selectable_scalar_properties.emplace_back("Vertex Weight");
    property_map["Vertex Weight"] = "v:weight";
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

void Meshiew::computeValence() {
    selectable_scalar_properties.emplace_back("Valence");
    property_map["Valence"] = "v:valence";
    Surface_mesh::Vertex_property<Scalar> vertex_valence =
            mesh.vertex_property<Scalar>("v:valence", 0);
    for (const auto &v: mesh.vertices()) {
        vertex_valence[v] = mesh.valence(v);
    }

    selectable_scalar_properties.emplace_back("ID");
    property_map["ID"] = "v:id";
    Surface_mesh::Vertex_property<Scalar> vv = mesh.vertex_property<Scalar>("v:id", 0);
    for (const auto &v : mesh.vertices()) {
        vv[v] = v.idx();
    }
}

void Meshiew::calc_uniform_laplacian() {
    Surface_mesh::Vertex_property<Vec3> v_uniLaplace = mesh.vertex_property<Vec3>("v:uniLaplace", Vec3(0, 0, 0));
    Point laplace(0.0);

    for (const auto &v : mesh.vertices()) {
        laplace = 0;
        int N = 0;
        for (const auto &v2 : mesh.vertices(v)) {
            laplace += mesh.position(v2) - mesh.position(v);
            N++;
        }
        v_uniLaplace[v] = laplace / N;
    }
}


void Meshiew::calc_gauss_curvature() {
    selectable_scalar_properties.emplace_back("Gauss Curvature");
    property_map["Gauss Curvature"] = "v:gauss_curvature";
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

void Meshiew::initShaders() {
    using namespace shaders;
    line_renderers = {
            new LineRenderer(model_center, surface_mesh::Color(1.0, 1.0, 1.0)),
            new LineRenderer(model_center, surface_mesh::Color(0.        , 0.39607843, 0.74117647)), // TUM blue
            new LineRenderer(model_center, surface_mesh::Color(0.89019608, 0.44705882, 0.13333333)), // TUM orange
            new LineRenderer(model_center, surface_mesh::Color(0.50196078, 0.50196078, 0.50196078)), // TUM gray
            new LineRenderer(model_center, surface_mesh::Color(0.0, 1.0, 0.0)),
            new LineRenderer(model_center, surface_mesh::Color(1.0, 1.0, 0.0)),
            new LineRenderer(model_center, surface_mesh::Color(1.0, 0.0, 1.0)),
            new LineRenderer(model_center, surface_mesh::Color(0.0, 0.0, 1.0)),
            new LineRenderer(model_center, surface_mesh::Color(1.0, 0.5, 0.0)),
    };

    point_renderers = {
            new PointRenderer(model_center),
    };

    vectorfield_renderers = {
            new VectorfieldRenderer(Eigen::Vector3f(1.0, 0.0, 0.0)),
            new VectorfieldRenderer(Eigen::Vector3f(0.0, 1.0, 0.0)),
            new VectorfieldRenderer(Eigen::Vector3f(0.0, 0.0, 1.0)),
    };

    point_cloud_renderers = {
            new PointCloudRenderer(),
            new PointCloudRenderer(),
            new PointCloudRenderer(),
    };

    for (const auto &lr : line_renderers) {
        lr->setVisible(false);
        line_renderer_settings[lr].n_lines = 20;
        line_renderer_settings[lr].prop_name = "v:id";
        auto c = lr->getColor();
        line_renderer_settings[lr].color << c.x, c.y, c.z;
        line_renderer_settings[lr].point_trace_mode = false;
        line_renderer_settings[lr].point_renderer_id = 0;
    }

    mShader.init("mesh_shader", simple_vertex, fragment_light);

    boundary_renderer = std::make_shared<LineRenderer>();
    boundary_renderer->init();
    this->add_renderer(boundary_renderer);

    normals_renderer = std::make_shared<VectorfieldRenderer>();
    normals_renderer->init();
    this->add_renderer(normals_renderer);
    normals_renderer->setVisible(false);

    for (const auto &lr : line_renderers) {
        lr->setVisible(false);
        lr->init();
        this->add_renderer(lr);
    }
    for (const auto &pr : point_renderers) {
        pr->setVisible(false);
        pr->init();
        this->add_renderer(pr);
    }
    for (const auto &vr : vectorfield_renderers) {
        vr->setVisible(false);
        vr->init();
        this->add_renderer(vr);
    }
    for (const auto &pcr : point_cloud_renderers) {
        pcr->setVisible(false);
        pcr->init();
        this->add_renderer(pcr);
    }
}

void Meshiew::initModel() {
    mesh_center = computeCenter(&mesh);
    dist_max = 0.0f;
    for (auto v: mesh.vertices()) {
        if (distance(mesh_center, mesh.position(v)) > dist_max) {
            dist_max = distance(mesh_center, mesh.position(v));
        }
    }

    meshProcess();

    int j = 0;
    auto vertex_normal = mesh.vertex_property<Point>("v:normal");
    MatrixXf mesh_points(3, mesh.n_vertices());
    MatrixXi indices(3, mesh.n_faces());

    for (const auto &f: mesh.faces()) {
        vector<float> vv(3.0f);
        int k = 0;
        for (auto v: mesh.vertices(f)) {
            vv[k] = v.idx();
            ++k;
        }
        indices.col(j) << vv[0], vv[1], vv[2];
        ++j;
    }

    MatrixXf normals_attrib(3, mesh.n_vertices());

    j = 0;
    for (const auto &v: mesh.vertices()) {
        mesh_points.col(j) << mesh.position(v).x,
                mesh.position(v).y,
                mesh.position(v).z;

        normals_attrib.col(j) << vertex_normal[v].x,
                vertex_normal[v].y,
                vertex_normal[v].z;

        ++j;
    }

    this->mesh_points = mesh_points;
    mShader.bind();
    mShader.uploadIndices(indices);
    mShader.uploadAttrib("position", mesh_points);
    mShader.uploadAttrib("normal", normals_attrib);
    mShader.setUniform("color_mode", int(color_mode));
    mShader.setUniform("intensity", base_color);
    mShader.setUniform("broken_normals", (int) false);
    mShader.setUniform("ambient_term", DEFAULT_AMBIENT);
    mShader.setUniform("diffuse_term", DEFAULT_DIFFUSE);
    mShader.setUniform("specular_term", DEFAULT_SPECULAR);
    mShader.setUniform("opacity", DEFAULT_OPACITY);
    mShader.setUniform("shininess", 8);

    upload_color("v:valence");
    this->normals_renderer->show_vectorfield(mesh_points, normals_attrib);
    this->normals_renderer->set_color(Vector3f(0.0, 1.0, 0.0));
    this->normals_renderer->set_scaling(0.1f);

    auto &scalar_type = mesh.get_vertex_property_type("v:valence");
    auto &vector_type = mesh.get_vertex_property_type("v:normal");

    for (const auto &vprop : mesh.vertex_properties()) {
        if (mesh.get_vertex_property_type(vprop) == scalar_type) {
            if (vprop[0] == 'v' && vprop[1] == ':') continue;
            selectable_scalar_properties.emplace_back(vprop);
            property_map[vprop] = vprop;
        } else if (mesh.get_vertex_property_type(vprop) == vector_type) {
            selectable_vector_properties.emplace_back(vprop);
        }
    }

    for (const auto &p : line_renderer_settings) {
        p.first->show_isolines(mesh, p.second.prop_name, p.second.n_lines);
    }

    for (const auto &vr : vectorfield_renderers) {
        vr->show_vectorfield(mesh_points, normals_attrib);
        vr->set_scaling(0.1f);
    }

    for (const auto &pcr : point_cloud_renderers) {
        pcr->set_color(Eigen::Vector3f(1.0, 1.0, 1.0));
        pcr->show_points(mesh_points);
    }

}

Point Meshiew::computeCenter(Surface_mesh *mesh) {
    Point center = Point(0.0f);
    for (auto v: mesh->vertices()) {
        center += mesh->position(v);
    }
    return center / mesh->n_vertices();
}

void Meshiew::create_gui_elements(nanogui::Window *control, nanogui::Window *info) {
    using namespace nanogui;

    color_coding_window = std::unique_ptr<ColorCodingWindow>(new ColorCodingWindow(this, {
            Eigen::Vector3f(0, 0, 0),
            Eigen::Vector3f(0, 0, 1),
            Eigen::Vector3f(0, 1, 1),
            Eigen::Vector3f(0, 1, 0),
            Eigen::Vector3f(1, 1, 0),
            Eigen::Vector3f(1, 0, 0),
            Eigen::Vector3f(1, 1, 1),
    }));
    color_coding_window->setVisible(false);

    auto vp = mesh.vertex_properties();
    bool mesh_has_color = std::find(vp.begin(), vp.end(), "color") != vp.end();
    Button *b;

    new Label(control, "Property Renderers");
    auto renderer_popup_btn = new PopupButton(control, "Renderers");
    renderer_popup_btn->popup()->setLayout(new GroupLayout());
    auto rr_pp = renderer_popup_btn->popup();


    new Label(control, "Shading");
    auto p = new PopupButton(control, "Color Mode");
    auto c = p->popup();
    c->setLayout(new GroupLayout());

    b = new Button(c, "Normal");
    b->setPushed(!mesh_has_color);
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->color_mode = NORMAL;
        this->color_coding_window->setVisible(false);
    });

    b = new Button(c, "Color Coding");
    b->setPushed(false);
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->color_mode = COLOR_CODE;
        this->color_coding_window->setVisible(true);
    });

    b = new Button(c, "Mesh Color Property");
    b->setPushed(false);
    b->setFlags(Button::RadioButton);
    b->setEnabled(mesh_has_color);
    b->setCallback([this]() {
        this->color_coding_window->setVisible(false);
        auto color = mesh.get_vertex_property<surface_mesh::Color>("color");
        MatrixXf color_mat(3, mesh.n_vertices());

        int j = 0;
        for (auto v: mesh.vertices()) {
            color_mat.col(j) << color[v].x,
                    color[v].y,
                    color[v].z;
            ++j;
        }

        mShader.bind();
        this->color_mode = COLOR_CODE;
        mShader.uploadAttrib("colors", color_mat);
    });
    if (mesh_has_color) {
        b->setPushed(true);
        b->callback()();
    }

    new Label(c, "Normal");
    auto colorPoputBtn = new PopupButton(c, "Colors");
    Popup *colorPopup = colorPoputBtn->popup();
    auto *grid = new GridLayout(Orientation::Horizontal, 2, Alignment::Minimum, 15, 5);
    grid->setSpacing(0, 10);
    colorPopup->setLayout(grid);

    new Label(colorPopup, "Base Color:", "sans-bold");
    auto cp = new ColorPicker(colorPopup, base_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        base_color << c.r(), c.g(), c.b();
    });

    new Label(colorPopup, "Light Color:", "sans-bold");
    cp = new ColorPicker(colorPopup, light_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        light_color << c.r(), c.g(), c.b();
    });

    new Label(colorPopup, "Edge Color:", "sans-bold");
    cp = new ColorPicker(colorPopup, edge_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        edge_color << c.r(), c.g(), c.b();
    });

    new Label(c, "Color Coding (Scalar)");
    (new ComboBox(c, selectable_scalar_properties))->setCallback([this](int index) {
        auto prop = property_map[selectable_scalar_properties[index]];
        this->color_coding_window->setVisible(true);
        upload_color(prop);
    });

    new Label(c, "Color Coding (Vector Length)");
    (new ComboBox(c, selectable_vector_properties))->setCallback([this](int index) {
        auto prop = mesh.vertex_property<Vec3>(selectable_vector_properties[index]);
        const string prop_name = "v:vector_length_tmp";
        auto len_prop = mesh.vertex_property<Scalar>(prop_name);
        for (const auto &v : mesh.vertices()) {
            Eigen::Vector3f vec;
            vec << prop[v].x, prop[v].y, prop[v].z;
            len_prop[v] = vec.norm();
        }
        this->color_coding_window->setVisible(true);
        upload_color(prop_name);
    });

    new Label(c, "Outliers");
    (new CheckBox(c, "Show max/min"))->setCallback([this](bool value) {this->show_minmax = value;});


    Window *window = info;
    auto info_line = [&](const std::string &title, int value) {
        new Label(window, title + ":");
        auto *box = new IntBox<int>(window);
        box->setEditable(false);
        box->setFontSize(14);
        box->setValue(value);
    };

    auto light_model_btn = new PopupButton(control, "LightModel");
    auto light_model_pp = light_model_btn->popup();

    auto add_slider = [this, light_model_pp](const std::string &label, const std::string &key, float def) {
        new Label(light_model_pp, label);
        auto tmp_widget = new Widget(light_model_pp);
        tmp_widget->setLayout(new BoxLayout(Orientation::Horizontal));
        auto slider = new Slider(tmp_widget);
        slider->setValue(def / 2.f);
        slider->setCallback([this, key](float value) {
            mShader.bind();
            mShader.setUniform(key, 2.f * value);
        });
    };

    add_slider("Ambient", "ambient_term", DEFAULT_AMBIENT);
    add_slider("Diffuse", "diffuse_term", DEFAULT_DIFFUSE);
    add_slider("Specular", "specular_term", DEFAULT_SPECULAR);

    new Label(light_model_pp, "Shininess");
    auto box = new IntBox<int>(light_model_pp, 8);
    box->setEditable(true);
    box->setSpinnable(true);
    box->setCallback([this](int value) {
        mShader.bind();
        mShader.setUniform("shininess", value);
    });

    new Label(light_model_pp, "Opacity");
    auto w = new Widget(light_model_pp);
    w->setLayout(new BoxLayout(Orientation::Horizontal));
    auto slider = new Slider(w);
    slider->setValue(DEFAULT_OPACITY);
    slider->setCallback([this](float value) {
        mShader.bind();
        mShader.setUniform("opacity", value);
    });

    auto cb = new CheckBox(light_model_pp, "Broken Normals");
    cb->setCallback([this](bool value) {
        mShader.bind();
        mShader.setUniform("broken_normals", (int) value);
        this->broken_normals = value;
    });
    cb->setChecked(true);
    cb->callback()(true);

    light_model_pp->setLayout(new GroupLayout());
    new Label(control, "Mesh Graph");
    (new CheckBox(control, "Hide Mesh"))->setCallback([this](bool value) {
        this->hide_mesh = value;
    });

    (new CheckBox(control, "Wireframe"))->setCallback([this](bool wireframe) {
        this->wireframe = wireframe;
    });

    (new CheckBox(control, "Boundary"))->setCallback([this](bool value) {
        this->boundary_renderer->setVisible(value);
    });

    (new CheckBox(control, "Normals"))->setCallback([this](bool normals) {
        this->normals_renderer->setVisible(normals);
    });

    auto line_popup_btn = new PopupButton(rr_pp, "Line");
    auto line_popup = line_popup_btn->popup();
    line_popup->setLayout(new GroupLayout());

    int counter = 1;
    auto trajectory_files = this->additional_data_files("traj");

    for (const auto &lr : line_renderers) {
        stringstream ss;
        ss << "Renderer " << counter++;
        auto inner_popup_btn = new PopupButton(line_popup, ss.str());
        auto inner_popup = inner_popup_btn->popup();
        inner_popup->setLayout(new GroupLayout());

        new Label(inner_popup, "Line Rendering");
        auto checkbox = new CheckBox(inner_popup, "Enable");
        checkbox->setChecked(false);
        checkbox->setCallback([this, lr](bool value) {
            lr->setVisible(value);
        });

        new Label(inner_popup, "Isolines");
        auto combo = new ComboBox(inner_popup, selectable_scalar_properties);
        combo->setCallback([this, lr](int index) {
            auto prop = property_map[selectable_scalar_properties[index]];
            line_renderer_settings[lr].prop_name = prop;
            line_renderer_settings[lr].show_raw_data = false;
            lr->show_isolines(mesh, line_renderer_settings[lr].prop_name,
                              line_renderer_settings[lr].n_lines);
        });

        auto n_lines_box = new IntBox<int>(inner_popup, line_renderer_settings[lr].n_lines);
        n_lines_box->setEditable(true);
        n_lines_box->setCallback([this, lr](int value) {
            line_renderer_settings[lr].n_lines = value;
            lr->show_isolines(mesh, line_renderer_settings[lr].prop_name,
                              line_renderer_settings[lr].n_lines);
        });

        new Label(inner_popup, "Trajectories");
        combo = new ComboBox(inner_popup, trajectory_files);
        if (trajectory_files.empty()) {
            combo->setEnabled(false);
            combo->setCaption("No Data");
        }
        auto strip_mode_chk = new CheckBox(inner_popup, "Strip Mode");
        strip_mode_chk->setChecked(false);
        combo->setCallback([this, lr, trajectory_files, strip_mode_chk](int index) {
            auto filename = trajectory_files[index];
            line_renderer_settings[lr].point_trace_mode = false;
            line_renderer_settings[lr].show_raw_data = true;
            vector<Point> points;
            for (const auto &p : trajectory_reader::read(filename, true)) {
                points.emplace_back(p.x, p.y, p.z);
            }
            if (strip_mode_chk->checked())
                lr->show_line(points);
            else
                lr->show_line_segments(points);
        });

        strip_mode_chk->setCallback([this, lr](bool checked) {
            lr->setStripMode(checked);
        });

        new Label(inner_popup, "Point Renderer Traces");
        std::vector<string> renderer_names;
        int counter = 1;
        for (const auto &pr : point_renderers) {
            stringstream ss;
            ss << "Renderer " << counter++;
            renderer_names.push_back(ss.str());
        }

        (new ComboBox(inner_popup, renderer_names))->setCallback([this, lr](int index) {
            line_renderer_settings[lr].point_trace_mode = true;
            line_renderer_settings[lr].show_raw_data = false;
            lr->show_line(point_renderers[index]->trace());
        });

        new Label(inner_popup, "Line Color");
        auto cp = new ColorPicker(inner_popup, line_renderer_settings[lr].color);
        cp->setFixedSize({100, 20});
        cp->setCallback([this, lr](const Color &c) {
            line_renderer_settings[lr].color.x() = c.r();
            line_renderer_settings[lr].color.y() = c.g();
            line_renderer_settings[lr].color.z() = c.b();
            lr->setColor(surface_mesh::Color(c.r(), c.g(), c.b()));
        });
    }

    (new Button(line_popup, "Hide all"))->setCallback([this]() {
        for (const auto &lr : line_renderers) {
            lr->setVisible(false);
        }
    });

    auto pr_popup_btn = new PopupButton(rr_pp, "Point");
    auto pp = pr_popup_btn->popup();
    pp->setLayout(new GroupLayout());

    counter = 1;
    for (const auto &pr : point_renderers) {
        stringstream ss;
        ss << "Point Renderer " << counter++;
        auto btn = new PopupButton(pp, ss.str());
        btn->popup()->setLayout(new GroupLayout());

        (new CheckBox(btn->popup(), "Enabled"))->setCallback([this, pr](bool value) {
            pr->setVisible(value);
        });

        (new Button(btn->popup(), "Clear Trace"))->setCallback([this, pr, counter]() {
            pr->clear();
            for (const auto &lr : line_renderers) {
                if (line_renderer_settings[lr].point_trace_mode
                    && line_renderer_settings[lr].point_renderer_id == (counter - 2)) {
                    lr->show_line(pr->trace());
                }
            }
        });

        auto simulink_btn = new Button(btn->popup(), "Connect to Simulink");
        simulink_btn->setCallback([this, pr, simulink_btn]() {
            simulink_btn->setEnabled(false);
            create_simulink_receiver("localhost", 2222, pr);
        });
    }

    auto pcr_popup_btn = new PopupButton(rr_pp, "Point Cloud");
    pp = pcr_popup_btn->popup();
    pp->setLayout(new GroupLayout());
    counter = 1;
    for (const auto &pcr : point_cloud_renderers) {
        stringstream ss;
        ss << "Point Cloud Renderer " << counter++;
        auto btn = new PopupButton(pp, ss.str());
        btn->popup()->setLayout(new GroupLayout());

        (new CheckBox(btn->popup(), "Enabled"))->setCallback([this, pcr](bool value) {
            pcr->setVisible(value);
        });

        new Label(btn->popup(), "Point Size");
        auto slider = new Slider(btn->popup());
        slider->setValue(0.2);
        slider->setCallback([this, pcr](float value) {
            pcr->set_point_size(std::max(0.1f, 10.f * value));
        });

        new Label(btn->popup(), "Point Color");
        auto cp = new ColorPicker(btn->popup(), Eigen::Vector3f(1.0, 1.0, 1.0));
        cp->setFixedSize({100, 20});
        cp->setCallback([pcr](const Color &c) {
            pcr->set_color(c.head<3>());
        });

        new Label(btn->popup(), "Load from File");
        auto textbox = new TextBox(btn->popup(), pcr->default_file);
        textbox->setEditable(true);

        (new Button(btn->popup(), "Load"))->setCallback([this, textbox, pcr]() {
            PCDReader reader(textbox->value());
            if (reader.has_color()) {
                pcr->show_points(reader.get_points(), reader.get_colors());
            } else {
                pcr->show_points(reader.get_points());
            }
        });
    }

    auto vr_popup_btn = new PopupButton(rr_pp, "Vector Field");
    pp = vr_popup_btn->popup();
    pp->setLayout(new GroupLayout());
    counter = 1;
    for (const auto &vr : vectorfield_renderers) {
        stringstream ss;
        ss << "Vectorfield Renderer " << counter++;
        auto btn = new PopupButton(pp, ss.str());
        btn->popup()->setLayout(new GroupLayout());

        (new CheckBox(btn->popup(), "Enabled"))->setCallback([this, vr](bool value) {
            vr->setVisible(value);
        });

        new Label(btn->popup(), "Scaling");
        auto slider = new Slider(btn->popup());
        slider->setValue(0.1);
        slider->setCallback([vr](float value) {
            vr->set_scaling(value);
        });

        new Label(btn->popup(), "Vector Color");
        auto cp = new ColorPicker(btn->popup(), vr->get_color());
        cp->setFixedSize({100, 20});
        cp->setCallback([vr](const Color &c) {
            vr->set_color(c.head<3>());
        });

        new Label(btn->popup(), "Data Source");
        auto combo = new ComboBox(btn->popup(), selectable_vector_properties);
        combo->setCallback([this, vr](int index) {
            auto prop = mesh.get_vertex_property<Vec3>(selectable_vector_properties[index]);
            long n_boundary_vertices = std::accumulate(mesh.vertices().begin(), mesh.vertices().end(), 0,
                                                       [this](long i, const Vertex &v) {
                                                           return i + (mesh.is_boundary(v) ? 1 : 0);
                                                       });
            Eigen::MatrixXf vecs(3, mesh_points.cols() - n_boundary_vertices);
            Eigen::MatrixXf points(3, mesh_points.cols() - n_boundary_vertices);
            int j = 0;
            for (const auto &v : mesh.vertices()) {
                if (mesh.is_boundary(v)) continue;
                auto p = mesh.position(v);
                points.col(j) << p.x, p.y, p.z;
                vecs.col(j++) << prop[v].x, prop[v].y, prop[v].z;
            }
            vr->show_vectorfield(points, vecs);
        });
    }

    (new Button(pp, "Hide all"))->setCallback([this]() {
        for (const auto &vfr : vectorfield_renderers) {
            vfr->setVisible(false);
        }
    });

    new Label(control, "Remeshing");
    auto remesh_popup_btn = new PopupButton(control, "Remeshing");
    pp = remesh_popup_btn->popup();
    pp->setLayout(new GroupLayout());

    new Label(pp, "Settings");
    std::vector<std::string> remesh_modes = {"Average", "Curv"};
    auto remesh_mode_cmb = new ComboBox(pp, remesh_modes);

    auto n_iter_box = new IntBox<int>(pp, 1);
    n_iter_box->setMinValue(1);
    n_iter_box->setMaxValue(9999);
    n_iter_box->setEditable(true);
    n_iter_box->setSpinnable(true);

    auto allow_remeshing_chkbx = new CheckBox(pp, "Allow Remeshing");
    new Label(pp, "Warning: Remeshing will invalidate vertex properties.");

    new Label(pp, "Do it!");
    auto remesh_btn = new Button(pp, "Remesh");
    remesh_btn->setEnabled(false);
    remesh_btn->setCallback([this, remesh_mode_cmb, n_iter_box]() {
        REMESHING_TYPE mode = AVERAGE;
        switch (remesh_mode_cmb->selectedIndex()) {
          case 0: mode = AVERAGE; break;
          case 1: mode = CURV; break;
        }
        remesh(mode, n_iter_box->value());
        mesh.update_face_normals();
        mesh.update_vertex_normals();
        initModel();
    });

    allow_remeshing_chkbx->setCallback([remesh_btn](bool checked) {
      remesh_btn->setEnabled(checked);
    });

    bool closed = true;
    for (const auto &v : mesh.vertices()) {
        closed &= !mesh.is_boundary(v);
    }
    int euler = mesh.n_vertices() - mesh.n_edges() + mesh.n_faces();

    info_line("Vertices", mesh.n_vertices());
    info_line("Faces", mesh.n_faces());
    info_line("Edges", mesh.n_edges());
    info_line("Euler Characteristic", euler);
    if (closed) {
        info_line("Genus", -int(.5 * euler) + 1);
    }

    new Label(window, "Closed");
    auto *check = new CheckBox(window, "");
    check->setChecked(closed);
    check->setEnabled(false);
}

Meshiew::Meshiew(bool fs) :
        Matthew::Matthew(fs),
        base_color(0, 0.6, 0.15),
        light_color(1, 1, 1),
        edge_color(0, 0, 0) {

}

Meshiew::~Meshiew() {
    mShader.free();
}


Eigen::Vector3f Meshiew::get_model_center() {
    return {mesh_center.x, mesh_center.y, mesh_center.z};
}

float Meshiew::get_model_dist_max() {
    return dist_max;
}

Vector3f Meshiew::get_model_dimensions() {
    return mesh_points.rowwise().maxCoeff() - mesh_points.rowwise().minCoeff();
}

void Meshiew::load_from_file(const std::string &filename) {
    bool success;
    if (has_ending(filename, "msh")) {
        success = loadmsh::load_msh_file(filename, mesh);
    } else if (filename == "-") {
        cout << "Loading from stdin" << endl;
        success = loadmsh::load_msh(std::cin, mesh);
    } else {
        success = mesh.read(filename);
    }

    if (!success) {
        std::cerr << "Mesh not found, exiting." << std::endl;
        exit(-1);
    }
}

void Meshiew::load(Surface_mesh &mesh) {
    this->mesh = mesh;
}

void Meshiew::upload_color(const std::string &prop_name) {
    using namespace surface_mesh;
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    auto color = mesh.vertex_property<Color>("v:tmp_color_coding", Color(1, 1, 1));
    auto prop = mesh.vertex_property<Scalar>(prop_name);
    color_coding(prop, &mesh, color, 20);

    MatrixXf color_mat(3, mesh.n_vertices());

    int j = 0;
    for (auto v: mesh.vertices()) {
        color_mat.col(j) << color[v].x,
                color[v].y,
                color[v].z;
        ++j;
    }

    mShader.bind();
    mShader.uploadAttrib("colors", color_mat);
    mShader.setUniform("intensity", base_color);
}

void Meshiew::calc_boundary() {
    using namespace std::chrono;
    auto t_start = steady_clock::now();
    vector<vector<Vertex>> boundary_loops;
    vector<Vertex> boundary_vertices;

    std::copy_if(mesh.vertices().begin(), mesh.vertices().end(), std::back_inserter(boundary_vertices),
                 [this](const Vertex &v) {
                     return mesh.is_boundary(v);
                 });
    long loop_counter = 0;

    while (!boundary_vertices.empty()) {
        auto v = boundary_vertices[0];
        auto v_last = v;
        auto v_start = v;
        vector<Vertex> loop = {v};
        do {
	    loop_counter++;
            for (const auto &h : mesh.halfedges(v)) {
                bool is_boundary = mesh.is_boundary(h);
                bool goes_back = mesh.to_vertex(h) == v_last;
                if (!goes_back && is_boundary) {
                    v = mesh.to_vertex(h);
                    loop.push_back(v);
                    v_last = v;
                    break;
                }
            }
	    if (loop_counter > mesh.n_vertices()) {
		    std::cerr << "Aborted endless loop when finding closed boundary. Your mesh is broken." << std::endl;
		    break;
            }
        } while (v != v_start);

        auto contained_in_current_loop = [loop](const Vertex &v) {
            return std::find(loop.begin(), loop.end(), v) != loop.end();
        };

        boundary_vertices.erase(
                std::remove_if(boundary_vertices.begin(), boundary_vertices.end(), contained_in_current_loop),
                boundary_vertices.end());
        boundary_loops.push_back(loop);
    }
    cout << "The mesh has " << boundary_loops.size() << " boundary loops." << endl;
    n_boundary_points = std::accumulate(boundary_loops.begin(), boundary_loops.end(), 0,
                                        [](int acc, const vector<Vertex> &v) -> int {
                                            return acc + 2 * v.size() - 2;
                                        });
    MatrixXf points(3, n_boundary_points);
    int j = 0;
    for (const auto &loop : boundary_loops) {
        for (int i = 0; i < loop.size() - 1; i++) {
            auto p1 = mesh.position(loop[i]);
            auto p2 = mesh.position(loop[i + 1]);
            points.col(j++) << p1.x,
                    p1.y,
                    p1.z;
            points.col(j++) << p2.x,
                    p2.y,
                    p2.z;
        }
    }
    cout << "Computing the boundary took " << duration_cast<milliseconds>(steady_clock::now() - t_start).count()
         << "ms." << endl;
    boundary_renderer->show_line_segments(points);
}

void Meshiew::init_timer() {
    t.setInterval([this]() {
        for (const auto &pair : receivers) {
            auto receiver = pair.first;
            auto pr = pair.second;
            auto values = receiver->read_doubles(3);
            surface_mesh::Point p(
                    values[0],
                    values[1],
                    values[2]);
            pair.second->setPoint(p);
        }
        for (const auto &lr : line_renderers) {
            if (line_renderer_settings[lr].point_trace_mode) {
                lr->show_line(point_renderers[line_renderer_settings[lr].point_renderer_id]->trace());
            }
        }
    }, 0);
}

void Meshiew::create_simulink_receiver(const std::string &host, int port, PointRenderer *pr) {
    receivers.emplace_back(std::make_shared<SimulinkReceiver>(host, port), pr);
}

void Meshiew::calc_mean_curvature() {
    selectable_scalar_properties.emplace_back("Mean Curvature");
    property_map["Mean Curvature"] = "v:curvature";
    Surface_mesh::Vertex_property<Scalar> v_curvature = mesh.vertex_property<Scalar>("v:curvature", 0);
    Surface_mesh::Vertex_property<Vec3> v_laplacian = mesh.vertex_property<Vec3>("Laplacian", Vec3(0, 0, 0));
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
        v_laplacian[v] = laplace;
        v_curvature[v] = norm(laplace);
    }
}

void Meshiew::calc_principal_curvature_directions() {
    using namespace std::chrono;
    Surface_mesh::Vertex_property<Vec3> v_principal_direction1 = mesh.vertex_property<Vec3>("v:prince_1",
                                                                                            Vec3(0, 0, 0));
    Surface_mesh::Vertex_property<Vec3> v_principal_direction2 = mesh.vertex_property<Vec3>("v:prince_2",
                                                                                            Vec3(0, 0, 0));
    auto vertex_normal = mesh.vertex_property<Point>("v:normal");
    Surface_mesh::Edge_property<Scalar> e_weight = mesh.edge_property<Scalar>("e:weight", 0);
    Surface_mesh::Vertex_property<Scalar> v_weight = mesh.vertex_property<Scalar>("v:weight", 0);
    auto t_start = steady_clock::now();

    for (const auto &vi : mesh.vertices()) {
        auto xi = mesh.position(vi);
        auto n = vertex_normal[vi];
        auto ni = 0;
        for (const auto &vj : mesh.vertices(vi)) ni++;

        int j = 0;
        Eigen::MatrixXf m(ni, 3);
        Eigen::VectorXf k(ni);
        Eigen::MatrixXf weights = Eigen::MatrixXf::Zero(ni, ni);
        for (const auto &e : mesh.halfedges(vi)) {
            auto vj = mesh.to_vertex(e);
            auto xj = mesh.position(vj);
            double kij = 2 * dot(xi - xj, n) / pow(norm(xi - xj), 2);
            Vec3 d = (xj - xi) - dot(xj - xi, n) * n;
            d /= norm(d);
            k(j) = kij;
            weights(j, j) = 2 * v_weight[vi] * (1. / 8) * e_weight[mesh.edge(e)] * pow(norm(xi - xj), 2);
            if (weights(j, j) == 0) weights(j, j) = 1e-6;
            m.row(j++) << d.x, d.y, d.z;
        }
        Eigen::JacobiSVD<MatrixXf> svd(m, ComputeThinV);
        auto T = svd.matrixV();

        Eigen::MatrixXf d2 = m * T;

        Eigen::MatrixXf data(ni, 3);
        for (int i = 0; i < ni; i++) {
            data(i, 0) = d2(i, 0) * d2(i, 0);
            data(i, 1) = 2 * d2(i, 0) * d2(i, 1);
            data(i, 2) = d2(i, 1) * d2(i, 1);
        }

        Eigen::VectorXf x = (weights * data).householderQr().solve(weights * k);

        Eigen::Matrix2f curvatureTensor;
        curvatureTensor << x(0), x(1), x(1), x(2);

        Eigen::EigenSolver<MatrixXf> eig(curvatureTensor);

        Eigen::MatrixXf principalDirectionsLocal = Eigen::MatrixXf::Zero(3, 3);
        principalDirectionsLocal.block<2, 2>(0, 0) << eig.eigenvectors().real().transpose();

        Eigen::MatrixXf principalDirections = principalDirectionsLocal * T.transpose();

        v_principal_direction1[vi] = Vec3(principalDirections(0, 0), principalDirections(0, 1),
                                          principalDirections(0, 2));
        v_principal_direction2[vi] = Vec3(principalDirections(1, 0), principalDirections(1, 1),
                                          principalDirections(1, 2));
    }

    cout << "Computing the axes of principal curvatures took "
         << duration_cast<milliseconds>(steady_clock::now() - t_start).count()
         << "ms." << endl;

}
