#include "matthew.h"
#include <surface_mesh/Surface_mesh.h>

using std::min;
using std::max;
using namespace surface_mesh;

typedef Surface_mesh Mesh;

void Matthew::calc_weights() {
    calc_edges_weights();
    calc_vertices_weights();
}

void Matthew::calc_edges_weights() {
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

void Matthew::calc_vertices_weights() {
    Mesh::Face_around_vertex_circulator vf_c, vf_end;
    Mesh::Vertex_around_face_circulator fv_c;
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

void Matthew::computeValence() {
    Mesh::Vertex_property <Scalar> vertex_valence =
            mesh.vertex_property<Scalar>("v:valence", 0);
    for (auto v: mesh.vertices()) {
        vertex_valence[v] = mesh.valence(v);
    }
}

void Matthew::computeNormalsWithConstantWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Mesh::Vertex_property <Point> v_cste_weights_n =
            mesh.vertex_property<Point>("v:cste_weights_n", default_normal);

    for (const auto &v : mesh.vertices()) {
        Vec3 normal(0, 0, 0);
        for (const Mesh::Face &face : mesh.faces(v)) { // this uses the circulator
            normal += mesh.compute_face_normal(face);
        }
        v_cste_weights_n[v] = normal.normalize();
    }
}

void Matthew::computeNormalsByAreaWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Mesh::Vertex_property <Point> v_area_weights_n =
            mesh.vertex_property<Point>("v:area_weight_n", default_normal);

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

void Matthew::computeNormalsWithAngleWeights() {
    Point default_normal(0.0, 1.0, 0.0);
    Mesh::Vertex_property <Point> v_angle_weights_n =
            mesh.vertex_property<Point>("v:angle_weight_n", default_normal);

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

void Matthew::calc_uniform_laplacian() {
    Mesh::Vertex_property <Scalar> v_uniLaplace = mesh.vertex_property<Scalar>("v:uniLaplace", 0);
    Point laplace(0.0);
    min_uniLaplace = 1000;
    max_uniLaplace = -1000;

    for (const auto &v : mesh.vertices()) {
        laplace = 0;
        int N = 0;
        for (const auto &v2 : mesh.vertices(v)) {
            laplace += mesh.position(v2) - mesh.position(v);
            N++;
        }
        v_uniLaplace[v] = norm(laplace / N);
    }
    max_uniLaplace = *std::max_element(v_uniLaplace.vector().begin(), v_uniLaplace.vector().end());
    min_uniLaplace = *std::min_element(v_uniLaplace.vector().begin(), v_uniLaplace.vector().end());
}

void Matthew::calc_mean_curvature() {
    Mesh::Vertex_property <Scalar> v_curvature = mesh.vertex_property<Scalar>("v:curvature", 0);
    Mesh::Edge_property <Scalar> e_weight = mesh.edge_property<Scalar>("e:weight", 0);
    Mesh::Vertex_property <Scalar> v_weight = mesh.vertex_property<Scalar>("v:weight", 0);
    Point laplace(0.0);
    min_mean_curvature = 1000;
    max_mean_curvature = -1000;

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

void Matthew::calc_gauss_curvature() {
    Mesh::Vertex_property <Scalar> v_gauss_curvature = mesh.vertex_property<Scalar>("v:gauss_curvature", 0);
    Mesh::Vertex_property <Scalar> v_weight = mesh.vertex_property<Scalar>("v:weight", 0);
    Mesh::Vertex_around_vertex_circulator vv_c, vv_c2, vv_end;
    Point d0, d1;
    Scalar angles, cos_angle;
    Scalar lb(-1.0f), ub(1.0f);
    min_gauss_curvature = 1000;
    max_gauss_curvature = -1000;

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

    max_gauss_curvature = *std::max_element(v_gauss_curvature.vector().begin(), v_gauss_curvature.vector().end());
    min_gauss_curvature = *std::min_element(v_gauss_curvature.vector().begin(), v_gauss_curvature.vector().end());
}

void Matthew::loadMesh(string filename) {
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

    mCamera.arcball = Arcball();
    mCamera.arcball.setSize(mSize);
    mCamera.modelZoom = 2 / dist_max;
    mCamera.modelTranslation = -Vector3f(mesh_center.x, mesh_center.y, mesh_center.z);
}

void Matthew::meshProcess() {
    Point default_normal(0.0, 1.0, 0.0);
    Surface_mesh::Vertex_property<Point> vertex_normal =
            mesh.vertex_property<Point>("v:normal");
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    v_color_valence = mesh.vertex_property<surface_mesh::Color>("v:color_valence",
                                                                surface_mesh::Color(1.0f, 1.0f, 1.0f));
    v_color_unicurvature = mesh.vertex_property<surface_mesh::Color>("v:color_unicurvature",
                                                                     surface_mesh::Color(1.0f, 1.0f, 1.0f));
    v_color_curvature = mesh.vertex_property<surface_mesh::Color>("v:color_curvature",
                                                                  surface_mesh::Color(1.0f, 1.0f, 1.0f));
    v_color_gaussian_curv = mesh.vertex_property<surface_mesh::Color>("v:color_gaussian_curv",
                                                                      surface_mesh::Color(1.0f, 1.0f, 1.0f));

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
    color_coding(vertex_valence, &mesh, v_color_valence, 100 /* bound */);
    color_coding(v_uniLaplace, &mesh, v_color_unicurvature);
    color_coding(v_curvature, &mesh, v_color_curvature);
    color_coding(v_gauss_curvature, &mesh, v_color_gaussian_curv);

    int j = 0;
    MatrixXf mesh_points(3, n_vertices);
    MatrixXu indices(3, n_faces);

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
    mShader.setUniform("intensity", Vector3f(0.98, 0.59, 0.04));

    mShaderNormals.bind();
    mShaderNormals.uploadIndices(indices);
    mShaderNormals.uploadAttrib("position", mesh_points);
    mShaderNormals.uploadAttrib("n_cste_weights", normal_cste_weights_attrib);
    mShaderNormals.uploadAttrib("n_area_weights", normal_area_weights_attrib);
    mShaderNormals.uploadAttrib("n_angle_weights", normal_angle_weights_attrib);
    //mShaderNormals.uploadAttrib("normal", normals_attrib);
}

void Matthew::initGUI() {
    window = new Window(this, "Display Control");
    window->setPosition(Vector2i(15, 15));
    window->setLayout(new GroupLayout());

    PopupButton *popupBtn;
    Popup *popup;
    Button *b;

    new Label(window, "Mesh Graph");
    b = new Button(window, "Wireframe");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool wireframe) {
        this->wireframe = !this->wireframe;
    });
    new Label(window, "Normals");
    b = new Button(window, "Normals");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool normals) {
        this->normals = !this->normals;
    });

    popupBtn = new PopupButton(window, "Normal Weights");
    popup = popupBtn->popup();
    popup->setLayout(new GroupLayout());
    b = new Button(popup, "Constant Weights");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->normals_computation = 0;
    });
    b = new Button(popup, "Area Weights");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->normals_computation = 1;
    });
    b = new Button(popup, "Angle Weights");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->normals_computation = 2;
    });

    new Label(window, "Color Mode");
    b = new Button(window, "Plain");
    b->setCallback([this]() {
        this->color_mode = NORMAL;
    });

    b = new Button(window, "Light");
    b->setCallback([this]() {
        this->color_mode = NORMAL;
    });
    b = new Button(window, "Valence");
    b->setCallback([this]() {
        this->color_mode = VALENCE;
    });

    popupCurvature = new PopupButton(window, "Curvature");
    popup = popupCurvature->popup();
    popupCurvature->setCallback([this]() {
        this->color_mode = CURVATURE;
    });
    popup->setLayout(new GroupLayout());
    new Label(popup, "Curvature Type", "sans-bold");
    b = new Button(popup, "Uniform Laplacian");
    b->setFlags(Button::RadioButton);
    b->setPushed(true);
    b->setCallback([this]() {
        this->curvature_type = UNIMEAN;
        std::cout << "Min Uniform Laplace value is: " << min_uniLaplace << std::endl;
        std::cout << "Max Uniform Laplace value is: " << max_uniLaplace << std::endl;
    });
    b = new Button(popup, "Laplace-Beltrami");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->curvature_type = LAPLACEBELTRAMI;
        std::cout << "Min Laplace-Beltrami curvature value is: " << min_mean_curvature << std::endl;
        std::cout << "Max Laplace-Beltrami curvature value is: " << max_mean_curvature << std::endl;
    });
    b = new Button(popup, "Gaussian");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
        this->curvature_type = GAUSS;
        std::cout << "Min Gauss curvature value is: " << min_gauss_curvature << std::endl;
        std::cout << "Max Gauss curvature value is: " << max_gauss_curvature << std::endl;
    });

    PopupButton *colorPoputBtn = new PopupButton(window, "Colors");
    Popup *colorPopup = colorPoputBtn->popup();
    auto *grid = new GridLayout(Orientation::Horizontal, 2, Alignment::Minimum, 15, 5);
    grid->setSpacing(0, 10);
    colorPopup->setLayout(grid);

    new Label(colorPopup, "Base Color:", "sans-bold");
    auto cp = new ColorPicker(colorPopup, base_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const nanogui::Color &c) {
        base_color << c.r(), c.g(), c.b();
        cout << "New base color: " << base_color.transpose() << endl;
    });

    new Label(colorPopup, "Light Color:", "sans-bold");
    cp = new ColorPicker(colorPopup, light_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const nanogui::Color &c) {
        light_color << c.r(), c.g(), c.b();
        cout << "New light color: " << base_color.transpose() << endl;
    });

    new Label(colorPopup, "Edge Color:", "sans-bold");
    cp = new ColorPicker(colorPopup, edge_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const nanogui::Color &c) {
        edge_color << c.r(), c.g(), c.b();
        cout << "New edge color: " << edge_color.transpose() << endl;
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

    performLayout();
}

void Matthew::initShaders() {
    // Shaders
    mShader.init(
            "a_simple_shader",

            /* Vertex shader */
            "#version 330\n"
            "uniform mat4 MV;\n"
            "uniform mat4 P;\n"
            "uniform int color_mode;\n"
            "uniform vec3 intensity;\n"
            "uniform int normal_selector;\n"

            "in vec3 position;\n"
            "in vec3 valence_color;\n"
            "in vec3 unicruvature_color;\n"
            "in vec3 curvature_color;\n"
            "in vec3 gaussian_curv_color;\n"
            "in vec3 n_cste_weights;\n"
            "in vec3 n_area_weights;\n"
            "in vec3 n_angle_weights;\n"

            "out vec3 fcolor;\n"
            "out vec3 fnormal;\n"
            "out vec3 view_dir;\n"
            "out vec3 light_dir;\n"

            "void main() {\n"
            "    vec4 vpoint_mv = MV * vec4(position, 1.0);\n"
            "    gl_Position = P * vpoint_mv;\n"
            "    if (color_mode == 1) {\n"
            "        fcolor = valence_color;\n"
            "    } else if (color_mode == 2) {\n"
            "        fcolor = unicruvature_color;\n"
            "    } else if (color_mode == 3) {\n"
            "        fcolor = curvature_color;\n"
            "    } else if (color_mode == 4) {\n"
            "        fcolor = gaussian_curv_color;\n"
            "    } else {\n"
            "        fcolor = intensity;\n"
            "    }\n"
            "    if (normal_selector == 0) {\n"
            "       fnormal = mat3(transpose(inverse(MV))) * n_cste_weights;\n"
            "    } else if(normal_selector == 1) {\n"
            "       fnormal = mat3(transpose(inverse(MV))) * n_area_weights;\n"
            "    } else {\n"
            "       fnormal = mat3(transpose(inverse(MV))) * n_angle_weights;\n"
            "    }\n"
            "    light_dir = vec3(0.0, 3.0, 3.0) - vpoint_mv.xyz;\n"
            "    view_dir = -vpoint_mv.xyz;\n"
            "}",

            /* Fragment shader */
            "#version 330\n"
            "uniform int color_mode;\n"
            "uniform vec3 intensity;\n"
            "uniform vec3 light_color;\n"

            "in vec3 fcolor;\n"
            "in vec3 fnormal;\n"
            "in vec3 view_dir;\n"
            "in vec3 light_dir;\n"

            "out vec4 color;\n"

            "void main() {\n"
            "    vec3 c = vec3(0.0);\n"
            "    if (color_mode == 0) {\n"
            "        c += vec3(1.0)*vec3(0.1, 0.1, 0.1);\n"
            "        vec3 n = normalize(fnormal);\n"
            "        vec3 v = normalize(view_dir);\n"
            "        vec3 l = normalize(light_dir);\n"
            "        float lambert = dot(n,l);\n"
            "        if(lambert > 0.0) {\n"
            "            c += vec3(1.0)*light_color*lambert;\n"
            "            vec3 v = normalize(view_dir);\n"
            "            vec3 r = reflect(-l,n);\n"
            "            c += vec3(1.0)*vec3(0.8, 0.8, 0.8)*pow(max(dot(r,v), 0.0), 90.0);\n"
            "        }\n"
            "        c *= fcolor;\n"
            "    } else {\n"
            "       c = fcolor;\n"
            "    }\n"
            "    if (intensity == vec3(0.0)) {\n"
            "        c = intensity;\n"
            "    }\n"
            "    color = vec4(c, 1.0);\n"
            "}"
    );

    mShaderNormals.init(
            "normal_shader",
            /* Vertex shader */
            "#version 330\n\n"
            "in vec3 position;\n"
            "in vec3 n_cste_weights;\n"
            "in vec3 n_area_weights;\n"
            "in vec3 n_angle_weights;\n"
            "uniform mat4 MV;\n"
            "uniform mat4 P;\n"
            "uniform int normal_selector;\n"
            "out VS_OUT {\n"
            "    mat3 normal_mat;\n"
            "    vec3 normal;\n"
            "} vs_out;\n"
            "void main() {\n"
            "  gl_Position = vec4(position, 1.0);\n"
            "    if (normal_selector == 0) {\n"
            "       vs_out.normal = n_cste_weights;\n"
            "    } else if(normal_selector == 1) {\n"
            "       vs_out.normal = n_area_weights;\n"
            "    } else {\n"
            "       vs_out.normal = n_angle_weights;\n"
            "    }\n"
            "    vs_out.normal_mat = mat3(transpose(inverse(MV)));\n"
            "}",
            /* Fragment shader */
            "#version 330\n\n"
            "out vec4 frag_color;\n"
            "void main() {\n"
            "   frag_color = vec4(0.0, 1.0, 0.0, 1.0);\n"
            "}",
            /* Geometry shader */
            "#version 330\n\n"
            "layout (triangles) in;\n"
            "layout (line_strip, max_vertices = 6) out;\n"
            "uniform mat4 MV;\n"
            "uniform mat4 P;\n"
            "in VS_OUT {\n"
            "    mat3 normal_mat;\n"
            "    vec3 normal;\n"
            "} gs_in[];\n"
            "void createline(int index) {\n"
            "   gl_Position = P * MV * gl_in[index].gl_Position;\n"
            "   EmitVertex();\n"
            "   vec4 normal_mv = vec4(normalize(gs_in[index].normal_mat *\n"
            "                                   gs_in[index].normal), 1.0f);\n"
            "   gl_Position = P * (MV * gl_in[index].gl_Position\n"
            "                      + normal_mv * 0.035f);\n"
            "   EmitVertex();\n"
            "   EndPrimitive();\n"
            "}\n"
            "void main() {\n"
            "   createline(0);\n"
            "   createline(1);\n"
            "   createline(2);\n"
            "}"
    );
}

void Matthew::color_coding(Surface_mesh::Vertex_property<Scalar> prop, Surface_mesh *mesh,
                           Surface_mesh::Vertex_property<surface_mesh::Color> color_prop, int bound) {
    // Get the value array
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

surface_mesh::Color Matthew::value_to_color(Scalar value, Scalar min_value, Scalar max_value) {
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

Matthew::~Matthew() {
    mShader.free();
    mShaderNormals.free();
}

Point Matthew::computeCenter(Surface_mesh *mesh) {
    Point center = Point(0.0f);

    for (auto v: mesh->vertices()) {
        center += mesh->position(v);
    }

    return center / mesh->n_vertices();
}

bool Matthew::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
        return true;
    }
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        setVisible(false);
        return true;
    }
    return false;
}

void Matthew::draw(NVGcontext *ctx) {
    /* Draw the user interface */
    Screen::draw(ctx);
}

Vector2f Matthew::getScreenCoord() {
    Vector2i pos = mousePos();
    return Vector2f(2.0f * (float) pos.x() / width() - 1.0f,
                    1.0f - 2.0f * (float) pos.y() / height());
}

void Matthew::repaint() {
    //glfwPostEmptyEvent();
}

void Matthew::drawContents() {
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
    mShader.setUniform("normal_selector", normals_computation);
    if (color_mode == CURVATURE) {
        mShader.setUniform("color_mode", int(curvature_type));
    } else {
        mShader.setUniform("color_mode", int(color_mode));
    }
    mShader.drawIndexed(GL_TRIANGLES, 0, n_faces);

    if (wireframe) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        mShader.setUniform("intensity", edge_color);
        mShader.drawIndexed(GL_TRIANGLES, 0, n_faces);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (normals) {
        mShaderNormals.bind();
        mShaderNormals.setUniform("MV", mv);
        mShaderNormals.setUniform("P", p);
        mShaderNormals.setUniform("normal_selector", normals_computation);
        mShaderNormals.drawIndexed(GL_TRIANGLES, 0, n_faces);
    }
}

bool Matthew::scrollEvent(const Vector2i &p, const Vector2f &rel) {
    if (!Screen::scrollEvent(p, rel)) {
        mCamera.zoom = max(0.1, mCamera.zoom * (rel.y() > 0 ? 1.1 : 0.9));
        repaint();
    }
    return true;
}

bool Matthew::mouseMotionEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers) {
    if (!Screen::mouseMotionEvent(p, rel, button, modifiers)) {
        if (mCamera.arcball.motion(p)) {
            repaint();
        } else if (mTranslate) {
            Eigen::Matrix4f model, view, proj;
            computeCameraMatrices(model, view, proj);
            float zval = nanogui::project(Vector3f(mesh_center.x,
                                                   mesh_center.y,
                                                   mesh_center.z),
                                          view * model, proj, mSize).z();
            Eigen::Vector3f pos1 = nanogui::unproject(
                    Eigen::Vector3f(p.x(), mSize.y() - p.y(), zval), view * model, proj, mSize);
            Eigen::Vector3f pos0 = nanogui::unproject(
                    Eigen::Vector3f(mTranslateStart.x(), mSize.y() -
                                                         mTranslateStart.y(), zval), view * model, proj, mSize);
            mCamera.modelTranslation = mCamera.modelTranslation_start + (pos1 - pos0);
            repaint();

        }
        return true;
    }
}

bool Matthew::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {
    if (!Screen::mouseButtonEvent(p, button, down, modifiers)) {
        if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
            mCamera.arcball.button(p, down);
        } else if (button == GLFW_MOUSE_BUTTON_2 ||
                   (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
            mCamera.modelTranslation_start = mCamera.modelTranslation;
            mTranslate = true;
            mTranslateStart = p;
        }
    }
    if (button == GLFW_MOUSE_BUTTON_1 && !down) {
        mCamera.arcball.button(p, false);
    }
    if (!down) {
        mDrag = false;
        mTranslate = false;
    }
    return true;
}

void Matthew::computeCameraMatrices(Eigen::Matrix4f &model, Eigen::Matrix4f &view, Eigen::Matrix4f &proj) {

    view = nanogui::lookAt(mCamera.eye, mCamera.center, mCamera.up);

    float fH = std::tan(mCamera.viewAngle / 360.0f * M_PI) * mCamera.dnear;
    float fW = fH * (float) mSize.x() / (float) mSize.y();

    proj = nanogui::frustum(-fW, fW, -fH, fH, mCamera.dnear, mCamera.dfar);
    model = mCamera.arcball.matrix();

    model = nanogui::scale(model, Eigen::Vector3f::Constant(mCamera.zoom * mCamera.modelZoom));
    model = nanogui::translate(model, mCamera.modelTranslation);
}

