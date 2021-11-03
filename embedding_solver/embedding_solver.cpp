//
// Created by sach_ar on 10/8/21.
//

#include <fstream>
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
using namespace surface_mesh;

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

    Vertex boundary_handle;
    Surface_mesh::Halfedge start;
    for (const auto &v: mesh.vertices()) {
        if (mesh.is_boundary(v)) {
            boundary_handle = v;
            break;
        }
    }
    for (const auto &half: mesh.halfedges(boundary_handle)) {
        if (mesh.is_boundary(half)) {
            start = half;
            break;
        }
    }

    Surface_mesh::Halfedge h = start;
    double length = 0.0;
    double eucl_length = 0.0;
    do {
        length += target_length[mesh.edge(h)];
        eucl_length += norm(mesh.position(mesh.from_vertex(h)) - mesh.position(mesh.to_vertex(h)));
        h = mesh.next_halfedge(h);
    } while (mesh.from_vertex(h) != boundary_handle);
    double new_radius = length / eucl_length;

    cout << "Circumference (Riemannian): " << length << endl;
    cout << "Circumference (Euclidean): " << eucl_length << endl;
    for (const auto &v: mesh.vertices()) {
        mesh.position(v) *= new_radius;
    }

    h = start;
    do {
        h = mesh.next_halfedge(h);
        double l = norm(mesh.position(mesh.from_vertex(h)) - mesh.position(mesh.to_vertex(h)));
        double ratio = l / target_length[mesh.edge(h)];
        if ((ratio > 1.001) || (ratio < 0.999)) {
            cerr << "Error: mapping the boundary to a circle does not work for this model!" << endl;
        }
    } while (mesh.from_vertex(h) != boundary_handle);

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
    embedding_window->setLayout(new GroupLayout());

    auto auto_window = new nanogui::Window(this, "Auto");
    auto_window->setLayout(new GroupLayout());

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
        iterations_box->setValue(checked ? 1000000 : 1000);
    });

    min_vel_box = new FloatBox<double>(embedding_window, 0.001);
    min_vel_box->setEditable(true);

    new Label(embedding_window, "Stiffness");
    stiffness_box = new FloatBox<double>(embedding_window, 1.0);
    stiffness_box->setEditable(true);

    new Label(embedding_window, "Z Force");
    z_force_box = new FloatBox<double>(embedding_window, -0.01);
    z_force_box->setEditable(true);

    auto at_center_box = new CheckBox(embedding_window, "Only at Center Vertex");

    new Label(embedding_window, "Normal Force");
    normal_force_box = new FloatBox<double>(embedding_window, 0.0);
    normal_force_box->setEditable(true);

    new Label(embedding_window, "Laplacian Smoothing");
    diffusion_rate_box = new FloatBox<double>(embedding_window, 0.0);
    diffusion_rate_box->setEditable(true);

    new Label(embedding_window, "Smooting Iterations");
    smoothing_iterations_box = new IntBox<int>(embedding_window, 0);
    smoothing_iterations_box->setEditable(true);

    new Label(embedding_window, "Hinge Forces");
    auto hinge_forces_box = new FloatBox<double>(embedding_window, 0.0);
    hinge_forces_box->setEditable(true);

    new Label(embedding_window, "Integration Rate");
    rate_box = new FloatBox<double>(embedding_window, 0.1);
    rate_box->setEditable(true);

    auto boundary_check = new CheckBox(embedding_window, "Keep Boundary");
    boundary_check->setChecked(true);

    auto visualize_box = new CheckBox(embedding_window, "Visualize");
    visualize_box->setChecked(true);

    auto do_btn = new Button(embedding_window, "Run");
    auto run_succ_box = new CheckBox(embedding_window, "Successful");
    run_succ_box->setEnabled(false);

    do_btn->setCallback(
            [this, boundary_check, iterations_box, auto_detect_convergence_box, visualize_box, run_succ_box, at_center_box, hinge_forces_box]() {
                optimization_parameters params;
                params.rate = rate_box->value();
                params.keep_boundary = boundary_check->checked();
                params.z_force = z_force_box->value();
                params.z_force_only_at_center = at_center_box->checked();
                params.normal_force = normal_force_box->value();
                params.stiffness = stiffness_box->value();
                params.n_iterations = iterations_box->value();
                params.auto_detect_convergence = auto_detect_convergence_box->checked();
                params.min_vel = min_vel_box->value();
                params.diffusion_rate = diffusion_rate_box->value();
                params.smooting_iterations = smoothing_iterations_box->value();
                params.hinge_force = hinge_forces_box->value();
                run_succ_box->setChecked(optimize(params, !visualize_box->checked()));
                update();
                drawAll();
            });

    new Label(auto_window, "Automatic Schedule");

    status_box = new TextBox(auto_window, "NONE");

    auto auto_btn = new Button(auto_window, "Auto");
    auto auto_succ_box = new CheckBox(auto_window, "Successful");
    auto_succ_box->setEnabled(false);
    auto_btn->setCallback([this, boundary_check, visualize_box, auto_succ_box]() {
        auto_succ_box->setChecked(this->optimize_auto(!visualize_box->checked()));
    });

    new Label(auto_window, "Error History");
    (new Button(auto_window, "Write CSV"))->setCallback([this](){
        ofstream f("error.csv");
        for (const auto &v : error_history) f << v << ",";
        f << compute_total_error() << endl;
    });

    this->color_coding_window->setVisible(true);

    performLayout();
    embedding_window->setPosition(Eigen::Vector2i(this->size()(0) - embedding_window->size()(0) - 20, 20));
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

double
EmbeddingSolver::relax_springs(double rate, bool keep_boundary, double z_force, bool only_at_center,
                               double normal_force, double stiffness, double hinge_force) {
    auto target_length = mesh.get_edge_property<double>("e:target_length");
    Eigen::MatrixXd internal_forces, additional_forces, total_forces;
    internal_forces.setZero(mesh.n_vertices(), 3);
    additional_forces.setZero(mesh.n_vertices(), 3);

    for (const auto &e: mesh.edges()) {
        const auto &v1 = mesh.vertex(e, 0);
        const auto &v2 = mesh.vertex(e, 1);
        const auto &ps1 = mesh.position(v1);
        const auto &ps2 = mesh.position(v2);
        Eigen::Vector3d p1(ps1.x, ps1.y, ps1.z), p2(ps2.x, ps2.y, ps2.z);
        double length = (p1 - p2).norm();
        double force = stiffness * (length - target_length[e]);
        Eigen::Vector3d f = force * (p2 - p1).normalized();
        internal_forces.row(v1.idx()) += f;
        internal_forces.row(v2.idx()) -= f;
    }

    if (only_at_center) {
        additional_forces(0, 2) += z_force;
    } else {
        additional_forces.col(2).array() += z_force;
    }

    if (normal_force != 0) {
        auto normals = mesh.get_vertex_property<surface_mesh::Vec3>("v:normal");
        for (const auto &v: mesh.vertices()) {
            Eigen::Vector3d n;
            n << normals[v].x, normals[v].y, normals[v].z;
            additional_forces.row(v.idx()) += normal_force * n;
        }
    }

    if (hinge_force != 0.0) {
        additional_forces += hinge_force * hinge_forces();
    }

    total_forces = internal_forces + additional_forces;

    if (keep_boundary) {
        for (const auto &v: mesh.vertices()) {
            if (mesh.is_boundary(v)) {
                total_forces.row(v.idx()).setZero();
            }
        }
    }

    for (const auto &v: mesh.vertices()) {
        surface_mesh::Vec3 f(total_forces(v.idx(), 0), total_forces(v.idx(), 1), total_forces(v.idx(), 2));
        mesh.position(v) += rate * f;
    }

    iterations++;

    return internal_forces.norm();
}


void EmbeddingSolver::update() {
    update_ratio();
    upload_color("Ratio");
    update_mesh_points();
    this->recenter();
    mesh.update_face_normals();
    mesh.update_vertex_normals();
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

        if (mesh.is_boundary(v)) continue;

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

bool EmbeddingSolver::optimize(const optimization_parameters &params, bool quiet) {
    Eigen::MatrixXf mp_last, mp;
    bool use_vel = (abs(params.z_force) > 1e-12) || (abs(params.normal_force) > 1e-12);
    if (use_vel) {
        mp_last.setZero(mesh.n_vertices(), 3);
        mp.setZero(mesh.n_vertices(), 3);
    }
    for (long n = 0; n < params.n_iterations; n++) {
        relax_springs(
                params.rate, params.keep_boundary, params.z_force, params.z_force_only_at_center,
                params.normal_force, params.stiffness, params.hinge_force);
        if (params.smooting_iterations > 0) {
            diffusion(params.diffusion_rate, params.smooting_iterations);
        }
        if (!quiet && (n % 100 == 0)) {
            this->update();
            this->drawAll();
        }

        if ((params.normal_force != 0) && (n % int(1 / params.rate) == 0)) {
            mesh.update_vertex_normals();
        }

        double total_error = compute_total_error();
        error_history.push_back(total_error);

        if (params.auto_detect_convergence) {
            if (use_vel) {
                for (const auto &v: mesh.vertices())
                    mp.row(v.idx()) << mesh.position(v).x, mesh.position(v).y, mesh.position(v).z;
                double velocity = (mp - mp_last).norm();
                mp_last = mp;
                this->vel_box->setValue(velocity);
                if (velocity < params.min_vel) return true;
            } else {
                if (total_error < params.min_vel) return true;
            }
        }
    }
    if (params.auto_detect_convergence) {
        cout << "Didn't converge after " << params.n_iterations << " steps. Aborting optimization." << endl;
    }
    return !params.auto_detect_convergence;
}

bool EmbeddingSolver::optimize_auto(bool quiet) {
    std::vector<ScheduleStep> schedule;

//    schedule.push_back(ScheduleStep("Pulling Down")
//                               .z_force(-0.01)
//                               .min_vel(1e-3));
//
//    schedule.push_back(ScheduleStep("Finalize")
//                               .z_force(-0.001)
//                               .normal_force(-0.001)
//                               .stiffness(20)
//                               .rate(0.01)
//                               .min_vel(5e-6));

    schedule.push_back(ScheduleStep("Freestyle")
                               .normal_force(-0.005)
                               .n_iterations(20000)
                               .keep_boundary(false)
                               .rate(0.2)
                               .min_vel(0.01)
                               .auto_detect_convergence(false));
    for (int i = 0; i < 5; ++i) {
        stringstream ss, sss;
        ss << "Inflating " << i + 1;
        schedule.push_back(ScheduleStep(ss.str())
                                   .normal_force(-0.005)
                                   .n_iterations(60000)
                                   .keep_boundary(true)
                                   .rate(0.2)
                                   .min_vel(0.01)
                                   .auto_detect_convergence(true));


        sss << "Relaxing " << i + 1;
        schedule.push_back(ScheduleStep(sss.str())
                                   .normal_force(0)
                                   .n_iterations(100000)
                                   .keep_boundary(false)
                                   .rate(0.2)
                                   .min_vel(1)
                                   .auto_detect_convergence(true));
    }

    schedule.push_back(ScheduleStep("Lowering Error")
                               .normal_force(0)
                               .n_iterations(100000)
                               .keep_boundary(false)
                               .rate(0.2)
                               .min_vel(.1)
                               .auto_detect_convergence(true));

//    schedule.push_back(
//            ScheduleStep("Hinge")
//                    .normal_force(0)
//                    .n_iterations(100000)
//                    .keep_boundary(false)
//                    .hinge_force(.5)
//                    .rate(0.2)
//                    .min_vel(0.01)
//                    .auto_detect_convergence(true));
//
//    schedule.push_back(
//            ScheduleStep("Finalize")
//                    .normal_force(0)
//                    .n_iterations(100000)
//                    .keep_boundary(false)
//                    .rate(0.2)
//                    .min_vel(1e-2)
//                    .auto_detect_convergence(true));

    bool success = true;

    for (const auto &step: schedule) {
        rate_box->setValue(step.p.rate);
        stiffness_box->setValue(step.p.stiffness);
        status_box->setValue(step.desc);
        z_force_box->setValue(step.p.z_force);
        normal_force_box->setValue(step.p.normal_force);
        min_vel_box->setValue(step.p.min_vel);
        drawAll();

        success &= optimize(step.p, quiet);

        if (!success) break;

        update();
        drawAll();
    }

    return success;
}

double signum(double v) {
    if (v > 0.0) return 1.0;
    else if (v < 0.0) return -1.0;
    else return 0.0;
}

Eigen::MatrixX3d EmbeddingSolver::hinge_forces() {
    Eigen::MatrixX3d f;
    f.setZero(mesh.n_vertices(), 3);

    for (const auto &e: mesh.edges()) {
        if (mesh.is_boundary(e)) continue;
        auto h1 = mesh.halfedge(e, 0);
        auto h2 = mesh.halfedge(e, 1);
        auto n1 = mesh.compute_face_normal(mesh.face(h1));
        auto n2 = mesh.compute_face_normal(mesh.face(h2));
        const double phi = std::acos(surface_mesh::dot(n1, n2) / (norm(n1) * norm(n2)));
        if (isnan(phi)) continue;
        auto v1 = mesh.to_vertex(mesh.next_halfedge(h1));
        auto v2 = mesh.to_vertex(mesh.next_halfedge(h2));
        auto edge_vector = mesh.position(mesh.to_vertex(h1)) - mesh.position(mesh.from_vertex(h1));
        double dedge_v1 =
                norm(cross(mesh.position(mesh.to_vertex(h1)) - mesh.position(v1), edge_vector)) / norm(edge_vector);
        double dedge_v2 =
                norm(cross(mesh.position(mesh.to_vertex(h2)) - mesh.position(v2), edge_vector)) / norm(edge_vector);
        double signed_force1 = signum(dot(cross(edge_vector, n1), n2)) * mesh.edge_length(e) * dedge_v1 * phi * phi;
        double signed_force2 = signum(dot(cross(-edge_vector, n2), n1)) * mesh.edge_length(e) * dedge_v2 * phi * phi;
        Eigen::RowVector3d ne1, ne2;
        ne1 << n1.x, n1.y, n1.z;
        ne2 << n2.x, n2.y, n2.z;
        f.row(v1.idx()) += signed_force1 * ne1;
        f.row(v2.idx()) += signed_force2 * ne2;
    }
    return f;
}

