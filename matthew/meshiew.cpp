//
// Created by arne on 11/13/19.
//

#include <shaders_gen.h>
#include "meshiew.h"

using namespace std;
using namespace Eigen;

using surface_mesh::Surface_mesh;
using surface_mesh::Vec3;
using surface_mesh::Point;

void Meshiew::calc_weights() {
    calc_edges_weights();
    calc_vertices_weights();
}

void Meshiew::set_color(Surface_mesh::Vertex v, const surface_mesh::Color &col,
                            Surface_mesh::Vertex_property<surface_mesh::Color> color_prop) {
    color_prop[v] = col;
}

void Meshiew::color_coding(Surface_mesh::Vertex_property<Scalar> prop, Surface_mesh *mesh,
                               Surface_mesh::Vertex_property<surface_mesh::Color> color_prop, int bound) {
    std::vector<Scalar> values = prop.vector();

    // discard upper and lower bound
    unsigned int n = values.size() - 1;
    unsigned int i = n / bound;
    std::sort(values.begin(), values.end());
    Scalar min_value = values[i], max_value = values[n - 1 - i];

    // map values to colors
    for (auto v: mesh->vertices()) {
        set_color(v, value_to_color(prop[v], min_value, max_value), color_prop);
    }
}

void Meshiew::drawContents() {
    using namespace nanogui;

    /* Draw the window contents using OpenGL */
    mShader.bind();

    Eigen::Matrix4f model, view, proj;
    computeCameraMatrices(model, view, proj);

    Matrix4f mv = view * model;
    Matrix4f p = proj;

    /* MVP uniforms */
    mShader.setUniform("MV", mv);
    mShader.setUniform("P", p);

    /* Setup OpenGL (making sure the GUI doesn't disable these */
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    /* Render everything */
    if (wireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    mShader.setUniform("intensity", base_color);
    mShader.setUniform("light_color", light_color);
    if (color_mode == CURVATURE) {
        mShader.setUniform("color_mode", int(curvature_type));
    } else {
        mShader.setUniform("color_mode", int(color_mode));
    }
    mShader.drawIndexed(GL_TRIANGLES, 0, mesh.n_faces());
//    mShader.drawArray(GL_POINTS, 0, mesh.n_faces());

    if (wireframe) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        mShader.setUniform("intensity", edge_color);
        mShader.drawIndexed(GL_TRIANGLES, 0, mesh.n_faces());
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (normals) {
        mShaderNormals.bind();
        mShaderNormals.setUniform("MV", mv);
        mShaderNormals.setUniform("P", p);
        mShaderNormals.drawIndexed(GL_TRIANGLES, 0, mesh.n_faces());
    }

    LIGHT_MODEL light_model;
    switch (this->color_mode) {
        case SEXY:
            light_model = PHONG;
            break;
        case VALENCE:
        case CURVATURE:
            light_model = LAMBERT;
            break;
        case PLAIN:
        default:
            light_model = NO_LIGHT;
    }
    mShader.setUniform("light_model", int(light_model));
}

surface_mesh::Color Meshiew::value_to_color(Scalar value, Scalar min_value, Scalar max_value) {
    Scalar v0, v1, v2, v3, v4;
    v0 = min_value + 0.0 / 4.0 * (max_value - min_value);
    v1 = min_value + 1.0 / 4.0 * (max_value - min_value);
    v2 = min_value + 2.0 / 4.0 * (max_value - min_value);
    v3 = min_value + 3.0 / 4.0 * (max_value - min_value);
    v4 = min_value + 4.0 / 4.0 * (max_value - min_value);

    surface_mesh::Color col(1.0f, 1.0f, 1.0f);

    if (value < v0) {
        col = surface_mesh::Color(0, 0, 1);
    } else if (value > v4) {
        col = surface_mesh::Color(1, 0, 0);
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

    auto v_uniLaplace = mesh.vertex_property<Scalar>("v:uniLaplace", 0);
    auto v_curvature = mesh.vertex_property<Scalar>("v:curvature", 0);
    auto v_gauss_curvature = mesh.vertex_property<Scalar>("v:gauss_curvature", 0);

    calc_weights();
    calc_uniform_laplacian();
    calc_mean_curvature();
    calc_gauss_curvature();
    computeValence();

    color_coding(vertex_valence, &mesh, v_color_valence, 100);
    color_coding(v_uniLaplace, &mesh, v_color_unicurvature);
    color_coding(v_curvature, &mesh, v_color_curvature);
    color_coding(v_gauss_curvature, &mesh, v_color_gaussian_curv);

    int j = 0;
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

    MatrixXf color_valence_attrib(3, mesh.n_vertices());
    MatrixXf color_unicurvature_attrib(3, mesh.n_vertices());
    MatrixXf color_curvature_attrib(3, mesh.n_vertices());
    MatrixXf color_gaussian_curv_attrib(3, mesh.n_vertices());
    MatrixXf normals_attrib(3, mesh.n_vertices());

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

        ++j;
    }

    mShader.bind();
    mShader.uploadIndices(indices);
    mShader.uploadAttrib("position", mesh_points);
    mShader.uploadAttrib("valence_color", color_valence_attrib);
    mShader.uploadAttrib("unicurvature_color", color_unicurvature_attrib);
    mShader.uploadAttrib("curvature_color", color_curvature_attrib);
    mShader.uploadAttrib("gaussian_curv_color", color_gaussian_curv_attrib);
    mShader.uploadAttrib("normal", normals_attrib);
    mShader.setUniform("color_mode", int(color_mode));
    mShader.setUniform("intensity", base_color);


    mShaderNormals.bind();
    mShaderNormals.uploadIndices(indices);
    mShaderNormals.uploadAttrib("position", mesh_points);
    mShaderNormals.uploadAttrib("normal", normals_attrib);
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
    Surface_mesh::Vertex_property<Scalar> vertex_valence =
            mesh.vertex_property<Scalar>("v:valence", 0);
    for (auto v: mesh.vertices()) {
        vertex_valence[v] = mesh.valence(v);
    }
}

void Meshiew::calc_uniform_laplacian() {
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

void Meshiew::calc_mean_curvature() {
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

void Meshiew::calc_gauss_curvature() {
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
    mShader.init("mesh_shader", simple_vertex, fragment_light);
    mShaderNormals.init("normal_shader", normals_vertex, normals_fragment, normals_geometry);
}

void Meshiew::load(string filename) {
    if (!mesh.read(filename)) {
        std::cerr << "Mesh not found, exiting." << std::endl;
        exit(-1);
    }

    mesh_center = computeCenter(&mesh);
    dist_max = 0.0f;
    for (auto v: mesh.vertices()) {
        if (distance(mesh_center, mesh.position(v)) > dist_max) {
            dist_max = distance(mesh_center, mesh.position(v));
        }
    }

    meshProcess();
}

Point Meshiew::computeCenter(Surface_mesh *mesh) {
    Point center = Point(0.0f);

    for (auto v: mesh->vertices()) {
        center += mesh->position(v);
    }

    return center / mesh->n_vertices();
}

void Meshiew::create_gui_elements() {
    using namespace nanogui;
    window = new Window(this, "Display Control");
    window->setPosition(Vector2i(15, 15));
    window->setLayout(new GroupLayout());

    Button *b;

    new Label(window, "Mesh Graph");
    b = new Button(window, "Wireframe");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool wireframe) {
        this->wireframe = wireframe;
    });
    wireframeBtn = b;

    new Label(window, "Normals");
    b = new Button(window, "Normals");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool normals) {
        this->normals = !this->normals;
    });

    new Label(window, "Color Mode");
    b = new Button(window, "Plain");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->color_mode = PLAIN;
        this->wireframeBtn->setPushed(true);
        this->wireframe = true;
    });

    b = new Button(window, "Sexy");
    b->setPushed(true);
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->color_mode = SEXY;
        this->wireframeBtn->setPushed(false);
        this->wireframe = false;
    });
    b = new Button(window, "Valence");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->color_mode = VALENCE;
    });

    popupCurvature = new PopupButton(window, "Curvature");
    popupCurvature->setFlags(Button::RadioButton);
    Popup *curvPopup = popupCurvature->popup();
    popupCurvature->setCallback([this, curvPopup]() {
        this->color_mode = CURVATURE;
    });
    curvPopup->setLayout(new GroupLayout());
    new Label(curvPopup, "Curvature Type", "sans-bold");
    b = new Button(curvPopup, "Uniform Laplacian");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->curvature_type = UNIMEAN;
    });
    b = new Button(curvPopup, "Laplace-Beltrami");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->curvature_type = LAPLACEBELTRAMI;
    });
    b = new Button(curvPopup, "Gaussian");
    b->setFlags(Button::RadioButton);
    b->setPushed(true);
    b->setCallback([this]() {
        this->curvature_type = GAUSS;
    });

    PopupButton *colorPoputBtn = new PopupButton(window, "Colors");
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

    window = new Window(this, "Mesh Info");
    window->setPosition(Vector2i(750, 15));
    grid = new GridLayout(Orientation::Horizontal, 2, Alignment::Minimum, 15, 5);
    grid->setSpacing(0, 10);
    window->setLayout(grid);

    new Label(window, "Filename:", "sans-bold");
    new Label(window, filename, "sans");

    auto info_line = [&](std::string title, int value) {
        new Label(window, title + ":");
        auto *box = new IntBox<int>(window);
        box->setEditable(false);
        box->setFontSize(14);
        box->setValue(value);
    };

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
    CheckBox *check = new CheckBox(window, "");
    check->setChecked(closed);
    check->setEnabled(false);
}

Meshiew::Meshiew() :
        base_color(0, 0.6, 0.15),
        light_color(1, 1, 1),
        edge_color(0, 0, 0) {

}

Meshiew::~Meshiew() {
    mShader.free();
    mShaderNormals.free();
}

Point Meshiew::get_model_center() {
    return mesh_center;
}

float Meshiew::get_model_dist_max() {
    return dist_max;
}
