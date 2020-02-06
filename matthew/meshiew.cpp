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

void Meshiew::color_coding(Surface_mesh::Vertex_property<Scalar> prop, Surface_mesh *mesh,
                           Surface_mesh::Vertex_property<surface_mesh::Color> color_prop, int bound) {
    std::vector<Scalar> values = prop.vector();

    // discard upper and lower bound
    unsigned int n = values.size() - 1;
    unsigned int i = n / bound;
    std::sort(values.begin(), values.end());
    Scalar min_value = values[i], max_value = values[n - 1 - i];

    cout << "Blue:\t" << min_value + 0.0 / 4.0 * (max_value - min_value) << endl;
    cout << "Cyan:\t" << min_value + 1.0 / 4.0 * (max_value - min_value) << endl;
    cout << "Green:\t" << min_value + 2.0 / 4.0 * (max_value - min_value) << endl;
    cout << "Yellow:\t" << min_value + 3.0 / 4.0 * (max_value - min_value) << endl;
    cout << "Red:\t" << min_value + 4.0 / 4.0 * (max_value - min_value) << endl;

    // map values to colors
    for (auto v: mesh->vertices()) {
        color_prop[v] = value_to_color(prop[v], min_value, max_value);
    }
}

void Meshiew::draw(Eigen::Matrix4f mv, Matrix4f p) {
    using namespace nanogui;
    mShader.bind();
    mShader.setUniform("MV", mv);
    mShader.setUniform("P", p);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    if (wireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    mShader.setUniform("intensity", base_color);
    mShader.setUniform("light_color", light_color);
    mShader.setUniform("color_mode", (int) color_mode);
    mShader.drawIndexed(GL_TRIANGLES, 0, mesh.n_faces());

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

    if (boundary) {
        boundaryShader.bind();
        glEnable(GL_LINE_SMOOTH);
        boundaryShader.setUniform("MV", mv);
        boundaryShader.setUniform("P", p);
        boundaryShader.setUniform("intensity", grid_intensity);
        boundaryShader.drawArray(GL_LINES, 0, n_boundary_points);
        glDisable(GL_LINE_SMOOTH);
    }

    for (const auto &lr : line_renderers) {
        lr->draw(mv, p);
    }

    for (const auto &pr : point_renderers) {
        pr->draw(mv, p);
    }
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

    computeValence();
    calc_weights();
    calc_uniform_laplacian();
    calc_mean_curvature();
    calc_gauss_curvature();
    calc_boundary();

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

    MatrixXf normals_attrib(3, mesh.n_vertices());

    j = 0;
    for (auto v: mesh.vertices()) {
        mesh_points.col(j) << mesh.position(v).x,
                mesh.position(v).y,
                mesh.position(v).z;

        normals_attrib.col(j) << vertex_normal[v].x,
                vertex_normal[v].y,
                vertex_normal[v].z;

        ++j;
    }

    mShader.bind();
    mShader.uploadIndices(indices);
    mShader.uploadAttrib("position", mesh_points);
    mShader.uploadAttrib("normal", normals_attrib);
    mShader.setUniform("color_mode", int(color_mode));
    mShader.setUniform("intensity", base_color);


    mShaderNormals.bind();
    mShaderNormals.uploadIndices(indices);
    mShaderNormals.uploadAttrib("position", mesh_points);
    mShaderNormals.uploadAttrib("normal", normals_attrib);
    this->mesh_points = mesh_points;

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
    selectable_properties.emplace_back("Valence");
    property_map["Valence"] = "v:valence";
    Surface_mesh::Vertex_property<Scalar> vertex_valence =
            mesh.vertex_property<Scalar>("v:valence", 0);
    for (const auto &v: mesh.vertices()) {
        vertex_valence[v] = mesh.valence(v);
    }

    selectable_properties.emplace_back("ID");
    property_map["ID"] = "v:id";
    Surface_mesh::Vertex_property<Scalar> vv = mesh.vertex_property<Scalar>("v:id", 0);
    for (const auto &v : mesh.vertices()) {
        vv[v] = v.idx();
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
    selectable_properties.emplace_back("Mean Curvature");
    property_map["Mean Curvature"] = "v:curvature";
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
    selectable_properties.emplace_back("Gauss Curvature");
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
            new LineRenderer(model_center, surface_mesh::Color(0.0, 0.0, 0.0)),
            new LineRenderer(model_center, surface_mesh::Color(1.0, 0.0, 0.0)),
    };

    point_renderers = {
            new PointRenderer(model_center),
    };

    for (const auto &lr : line_renderers) {
        line_renderer_settings[lr].n_lines = 10;
        line_renderer_settings[lr].prop_name = "v:id";
        auto c = lr->getColor();
        line_renderer_settings[lr].color << c.x, c.y, c.z;
        line_renderer_settings[lr].point_trace_mode = false;
        line_renderer_settings[lr].point_renderer_id = 0;
    }

    for (const auto &pr : point_renderers) {
        pr->setEnabled(false);
    }

    mShader.init("mesh_shader", simple_vertex, fragment_light);
    mShaderNormals.init("normal_shader", normals_vertex, normals_fragment, normals_geometry);
    boundaryShader.init("boundary_shader", grid_verts, grid_frag);
    for (const auto &lr : line_renderers) {
        lr->init();
        lr->setVisible(false);
    }
    for (const auto &pr : point_renderers) {
        pr->init();
    }
}

void Meshiew::initModel() {
    mesh_center = computeCenter(&mesh);
    cout << "Mesh center is at: " << mesh_center << endl;
    dist_max = 0.0f;
    for (auto v: mesh.vertices()) {
        if (distance(mesh_center, mesh.position(v)) > dist_max) {
            dist_max = distance(mesh_center, mesh.position(v));
        }
    }

    meshProcess();
    upload_color("v:valence");
    mShader.bind();
    mShader.setUniform("broken_normals", (int) false);
    mShader.setUniform("ambient_term", DEFAULT_AMBIENT);
    mShader.setUniform("diffuse_term", DEFAULT_DIFFUSE);
    mShader.setUniform("specular_term", DEFAULT_SPECULAR);
    mShader.setUniform("opacity", DEFAULT_OPACITY);
    mShader.setUniform("shininess", 8);

    cout << "Mesh has these vertex properties:" << endl;
    for (const auto &vprop : mesh.vertex_properties()) {
        cout << "\t- " << vprop << endl;
        if (vprop[0] == 'v' && vprop[1] == ':') continue;
        selectable_properties.emplace_back(vprop);
        property_map[vprop] = vprop;
    }

    for (const auto &p : line_renderer_settings) {
        p.first->show_isolines(mesh, p.second.prop_name, p.second.n_lines);
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
    auto vp = mesh.vertex_properties();
    bool mesh_has_color = std::find(vp.begin(), vp.end(), "color") != vp.end();
    Button *b;

    new Label(control, "Mesh Graph");
    (new CheckBox(control, "Wireframe"))->setCallback([this](bool wireframe) {
        this->wireframe = wireframe;
    });

    (new CheckBox(control, "Boundary"))->setCallback([this](bool value) {
        this->boundary = value;
    });

    (new CheckBox(control, "Normals"))->setCallback([this](bool normals) {
        this->normals = !this->normals;
    });


    new Label(control, "Shading");
    auto p = new PopupButton(control, "Color Mode");
    auto c = p->popup();
    c->setLayout(new GroupLayout());

    b = new Button(c, "Normal");
    b->setPushed(!mesh_has_color);
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->color_mode = NORMAL;
    });

    b = new Button(c, "Color Coding");
    b->setPushed(false);
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->color_mode = COLOR_CODE;
    });

    b = new Button(c, "Mesh Color Property");
    b->setPushed(false);
    b->setFlags(Button::RadioButton);
    b->setEnabled(mesh_has_color);
    b->setCallback([this]() {
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

    new Label(c, "Color Coding");
    auto combo = new ComboBox(c, selectable_properties);
    combo->setCallback([this](int index) {
        auto prop = property_map[selectable_properties[index]];
        upload_color(prop);
    });

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

    new Label(control, "Property Renderers");

    auto line_popup_btn = new PopupButton(control, "Line");
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
        checkbox->setCallback([this, lr](bool value) {
            lr->setVisible(value);
        });

        new Label(inner_popup, "Isolines");
        auto combo = new ComboBox(inner_popup, selectable_properties);
        combo->setCallback([this, lr](int index) {
            auto prop = property_map[selectable_properties[index]];
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
        combo->setCallback([this, lr, trajectory_files](int index) {
            auto filename = trajectory_files[index];
            line_renderer_settings[lr].point_trace_mode = false;
            line_renderer_settings[lr].show_raw_data = true;
            vector<Point> points;
            for (const auto &p : trajectory_reader::read(filename, true)) {
                points.emplace_back(p.x, p.y, p.z);
            }
            lr->show_line(points);
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

    auto pr_popup_btn = new PopupButton(control, "Point");
    auto pp = pr_popup_btn->popup();
    pp->setLayout(new GroupLayout());

    counter = 1;
    for (const auto &pr : point_renderers) {
        stringstream ss;
        ss << "Point Renderer " << counter++;
        auto btn = new PopupButton(pp, ss.str());
        btn->popup()->setLayout(new GroupLayout());

        (new CheckBox(btn->popup(), "Enabled"))->setCallback([this, pr](bool value) {
            pr->setEnabled(value);
        });

        (new Button(btn->popup(), "Clear Trace"))->setCallback([this, pr]() {
            pr->clear();
        });
    }


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
    mShaderNormals.free();
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
    color_coding(prop, &mesh, color);

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

    while (!boundary_vertices.empty()) {
        auto v = boundary_vertices[0];
        auto v_last = v;
        auto v_start = v;
        vector<Vertex> loop = {v};
        do {
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
    boundaryShader.bind();
    boundaryShader.uploadAttrib("position", points);
    cout << "Computing the boundary took " << duration_cast<milliseconds>(steady_clock::now() - t_start).count()
         << "ms.";
}

void Meshiew::get_ready_to_run() {
    receiver = new SimulinkReceiver("localhost", 2222);

    t.setInterval([this]() {
        auto values = receiver->read_doubles(3);
        for (const auto &pr : point_renderers) {
            surface_mesh::Point p(
                    values[0],
                    values[1],
                    values[2]);
            pr->setPoint(p);
        }
        for (const auto &lr : line_renderers) {
            if (line_renderer_settings[lr].point_trace_mode) {
                lr->show_line(point_renderers[line_renderer_settings[lr].point_renderer_id]->trace());
            }
        }
    }, 0);
}
