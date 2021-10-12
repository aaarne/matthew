//
// Created by sach_ar on 10/8/21.
//

#include "embedding_solver.h"
#include <nanogui/label.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/textbox.h>
#include <nanogui/checkbox.h>
#include <numeric>
#include <Eigen/Sparse>

using namespace std;
using surface_mesh::Surface_mesh;

EmbeddingSolver::EmbeddingSolver(Eigen::MatrixXd &vertices,
                                 Eigen::MatrixXi &faces,
                                 Eigen::MatrixXi &edges,
                                 Eigen::VectorXd &tl) {
    for (int i = 0; i < vertices.rows(); i++) {
        mesh.add_vertex(surface_mesh::Point(vertices(i, 0), vertices(i, 1), vertices(i, 2)));
    }
    for (int i = 0; i < faces.rows(); i++) {
        vector<Vertex> f;
        f.emplace_back(faces(i, 0));
        f.emplace_back(faces(i, 1));
        f.emplace_back(faces(i, 2));
        mesh.add_face(f);
    }

    Surface_mesh::Edge_property<double> target_length = mesh.add_edge_property("e:target_length", 1e9);
    mesh.add_edge_property("e:length", 1.0);
    mesh.add_edge_property("e:ratio", 1.0);
    mesh.add_vertex_property("Ratio", 1.0f);

    for (int i = 0; i < edges.rows(); i++) {
        Vertex v1(edges(i, 0)), v2(edges(i, 1));
        auto edge = mesh.find_edge(v1, v2);
        target_length[edge] = tl(i);
    }

    const auto[min, max] = std::minmax_element(target_length.vector().begin(), target_length.vector().end());
    cout << "target length: min=" << *min << ", max=" << *max << endl;

    if (*max > 1e6) {
        for (const auto &e: mesh.edges()) {
            if (target_length[e] > 1e6) {
                cout << "Target length for edge "
                     << e << "(" << mesh.vertex(e, 0) << ", " << mesh.vertex(e, 1) << ") is "
                     << target_length[e] << endl;
            }
        }
    }

    update_ratio();
}

void EmbeddingSolver::create_gui_elements(nanogui::Window *control, nanogui::Window *info) {
    Meshiew::create_gui_elements(control, info);

    control->setVisible(false);
    this->color_mode = COLOR_CODE;
    upload_color("Ratio");

    this->wireframe = true;

    using namespace nanogui;
    auto embedding_window = new nanogui::Window(this, "Embedding Solver");
    embedding_window->setPosition(Eigen::Vector2i(1000, 20));
    embedding_window->setLayout(new GroupLayout());

    new Label(embedding_window, "Total Iterations");
    total_iterations_box = new IntBox<long>(embedding_window, 0);

    new Label(embedding_window, "Total Error");
    error_box = new FloatBox<double>(embedding_window, compute_total_error());

    new Label(embedding_window, "Velocity");
    vel_box = new FloatBox<double>(embedding_window, 0.0);

    new Label(embedding_window, "Iterations");
    auto iterations_box = new IntBox<long>(embedding_window, 1000);
    iterations_box->setEditable(true);

    auto auto_detect_convergence_box = new CheckBox(embedding_window, "Auto Detect Convergence");
    auto_detect_convergence_box->setCallback([iterations_box](bool checked) {
        iterations_box->setEnabled(!checked);
    });

    min_vel_box = new FloatBox<double>(embedding_window, 0.001);
    min_vel_box->setEditable(true);

    new Label(embedding_window, "Stiffness");
    stiffness_box = new FloatBox<double>(embedding_window, 1.0);
    stiffness_box->setEditable(true);

    new Label(embedding_window, "Z Force");
    z_force_box = new FloatBox<double>(embedding_window, -0.01);
    z_force_box->setEditable(true);

    new Label(embedding_window, "Integration Rate");
    rate_box = new FloatBox<double>(embedding_window, 0.1);
    rate_box->setEditable(true);

    auto boundary_check = new CheckBox(embedding_window, "Keep Boundary");
    boundary_check->setChecked(true);

    auto visualize_box = new CheckBox(embedding_window, "Visualize");
    visualize_box->setChecked(false);

    auto do_btn = new Button(embedding_window, "Run");
    do_btn->setCallback([this, boundary_check, iterations_box, auto_detect_convergence_box, visualize_box]() {
        this->optimize(rate_box->value(), boundary_check->checked(), z_force_box->value(), stiffness_box->value(),
                       iterations_box->value(), auto_detect_convergence_box->checked(), min_vel_box->value(), !visualize_box->checked());
    });

    new Label(embedding_window, "Automatic Schedule");

    status_box = new TextBox(embedding_window, "NONE");

    auto auto_btn = new Button(embedding_window, "Auto");
    auto_btn->setCallback([this, boundary_check, visualize_box](){
        boundary_check->setChecked(true);
        boundary_check->setEnabled(false);
        rate_box->setEditable(false);
        z_force_box->setEditable(false);
        stiffness_box->setEditable(false);
        min_vel_box->setEditable(false);
        this->optimize_auto(!visualize_box->checked());
    });

}

void EmbeddingSolver::update_ratio() {
    auto length = mesh.get_edge_property<double>("e:length");
    auto ratio = mesh.get_edge_property<double>("e:ratio");
    auto target_length = mesh.get_edge_property<double>("e:target_length");
    for (const auto &e: mesh.edges()) {
        Point p1 = mesh.position(mesh.vertex(e, 0)), p2 = mesh.position(mesh.vertex(e, 1));
        length[e] = norm(p1 - p2);
        ratio[e] = target_length[e] / length[e];
    }

    auto vratio = mesh.get_vertex_property<float>("Ratio");
    for (const auto &v: mesh.vertices()) {
        double acc = 0;
        int n = 0;
        for (const auto &h: mesh.halfedges(v)) {
            n++;
            acc += ratio[mesh.edge(h)];
        }
        vratio[v] = acc / n;
    }
}


void EmbeddingSolver::initShaders() {
    Meshiew::initShaders();
}

double EmbeddingSolver::relax_springs(double rate, bool keep_boundary, double z_force, double stiffness) {
    auto target_length = mesh.get_edge_property<double>("e:target_length");
    Eigen::MatrixXd forces(mesh.n_vertices(), 3);
    forces.setZero();

    for (const auto &e: mesh.edges()) {
        const auto &v1 = mesh.vertex(e, 0);
        const auto &v2 = mesh.vertex(e, 1);
        const auto &ps1 = mesh.position(v1);
        const auto &ps2 = mesh.position(v2);
        Eigen::Vector3d p1(ps1.x, ps1.y, ps1.z), p2(ps2.x, ps2.y, ps2.z);
        double length = stiffness * (p1 - p2).norm();
        double force = length - target_length[e];
        Eigen::Vector3d f = force * (p2 - p1).normalized();
        forces.row(v1.idx()) += f;
        forces.row(v2.idx()) -= f;
    }

    forces.col(2).array() += z_force;

    if (keep_boundary) {
        for (const auto &v: mesh.vertices()) {
            if (mesh.is_boundary(v)) {
                forces.row(v.idx()).setZero();
            }
        }
    }

    for (const auto &v: mesh.vertices()) {
        surface_mesh::Vec3 f(forces(v.idx(), 0), forces(v.idx(), 1), forces(v.idx(), 2));
        mesh.position(v) += rate * f;
    }

    forces.col(2).array() -= z_force;

    iterations++;
    return forces.norm();
}


void EmbeddingSolver::update() {
    update_ratio();
    upload_color("Ratio");
    update_mesh_points();
    error_box->setValue(compute_total_error());
    total_iterations_box->setValue(iterations);
}

void EmbeddingSolver::diffusion(double rate, int iterations) {
    calc_edges_weights();
    auto e_weight = mesh.get_edge_property<Scalar>("e:weight");

    Eigen::SparseMatrix<float> L(mesh.n_vertices(), mesh.n_vertices());
    std::vector<Eigen::Triplet<float>> triplets;

    for (int i = 0; i < mesh.n_vertices(); i++) {
        const auto &v = Vertex(i);
        double sum = 0;
        for (const auto &h: mesh.halfedges(v)) {
            sum += e_weight[mesh.edge(h)];
        }
        for (const auto &h: mesh.halfedges(v)) {
            triplets.emplace_back(i, mesh.to_vertex(h).idx(), e_weight[mesh.edge(h)] / sum);
        }
        triplets.emplace_back(i, i, -1.0);
    }
    L.setFromTriplets(triplets.begin(), triplets.end());

    for (int i = 0; i < iterations; i++) {
        Eigen::MatrixXf laplacian = L * mesh_points.transpose();

        for (const auto &v: mesh.vertices()) {
            surface_mesh::Vec3 l(laplacian(v.idx(), 0), laplacian(v.idx(), 1), laplacian(v.idx(), 2));
            mesh.position(v) += rate * l; // + rate/10 * l2;
        }
    }
}

double EmbeddingSolver::compute_total_error() {
    auto target_length = mesh.get_edge_property<double>("e:target_length");
    std::vector<double> forces;

    for (const auto &e: mesh.edges()) {
        const auto &v1 = mesh.vertex(e, 0);
        const auto &v2 = mesh.vertex(e, 1);
        const auto &ps1 = mesh.position(v1);
        const auto &ps2 = mesh.position(v2);
        Eigen::Vector3d p1(ps1.x, ps1.y, ps1.z), p2(ps2.x, ps2.y, ps2.z);
        double length = (p1 - p2).norm();
        double force = length - target_length[e];
        forces.push_back(force * force);
    }
    return sqrt(std::accumulate(forces.begin(), forces.end(), 0.0));
}

double EmbeddingSolver::optimize(double rate, bool keep_boundary, double z_force, double stiffness, long n_iterations,
                                 bool auto_detect_convergence, double min_vel, bool quiet) {
    long n = 0;
    min_vel *= mesh.n_vertices() / 10000.0;
    Eigen::MatrixXf mp_last, mp;
    if (abs(z_force) > 1e-12) {
        mp_last.setZero(mesh.n_vertices(), 3);
        mp.setZero(mesh.n_vertices(), 3);
    }
    while (true) {
        double err = relax_springs(rate, keep_boundary, z_force, stiffness);
        n++;
        if (!quiet && (n % 10 == 0)) {
            this->update();
            this->drawAll();
        }
        if (auto_detect_convergence) {
            if (abs(z_force) > 1e-12) {
                for (const auto &v : mesh.vertices()) mp.row(v.idx()) << mesh.position(v).x, mesh.position(v).y, mesh.position(v).z;
                double velocity = (mp-mp_last).norm();
                mp_last = mp;
                this->vel_box->setValue(velocity);
                if (velocity < min_vel) break;
            } else {
                if (err < min_vel) break;
            }
        } else {
            if (n > n_iterations) break;
        }

    };
    this->update();
    this->drawAll();
}

double EmbeddingSolver::optimize_auto(bool quiet) {
    this->status_box->setValue("Pulling down");
    const double pulling_down_rate = 0.2;
    const double pulling_down_stiffness = 1.0;
    const double pulling_down_zforce = -0.01;
    this->stiffness_box->setValue(pulling_down_stiffness);
    this->rate_box->setValue(pulling_down_rate);
    this->z_force_box->setValue(pulling_down_zforce);
    optimize(pulling_down_rate, true, pulling_down_zforce, pulling_down_stiffness, -1, true, 1e-2, quiet);

    const double lowering_force_rate = 0.2;
    const double lowering_force_stiffness = 1;
    this->stiffness_box->setValue(lowering_force_stiffness);
    this->rate_box->setValue(lowering_force_rate);
    std::vector<double> zforces = {-1e-2, -5e-3, -1e-3, -5e-4, -1e-4, 5e-5, 1e-5, 5e-6, 1e-6};
    this->status_box->setValue("Lowering Gravity");
    for (const double z_force : zforces) {
        this->z_force_box->setValue(z_force);
        optimize(lowering_force_rate, true, z_force, pulling_down_stiffness, -1, true, 1e-2, quiet);
    }

    const double pre_final_rate = 0.2;
    const double pre_final_stiffness = 1.0;
    this->rate_box->setValue(pre_final_rate);
    this->stiffness_box->setValue(pre_final_stiffness);
    this->status_box->setValue("No Gravity");
    this->optimize(pre_final_rate, true, 0.0, pre_final_stiffness, -1, true, 1e-3, quiet);

    const double final_rate = 0.2;
    const double final_stiffness = 1.0;
    this->rate_box->setValue(final_rate);
    this->stiffness_box->setValue(final_stiffness);
    this->status_box->setValue("Finalize");
    this->optimize(final_rate, true, 0.0, final_stiffness, -1, true, 1e-4, quiet);

    this->update();
    this->drawAll();
}

