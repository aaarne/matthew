#include "matthew.h"
#include <surface_mesh/Surface_mesh.h>
#include "shaders_gen.h"

using namespace std;
using namespace Eigen;

typedef surface_mesh::Surface_mesh Mesh;

Matthew::Matthew() :
        nanogui::Screen(Eigen::Vector2i(1024, 768), "Matthew"),
        base_color(0, 0.6, 0.15),
        light_color(1, 1, 1),
        edge_color(0, 0, 0) {

}

void Matthew::run(std::string mesh_file) {
    this->filename = mesh_file;
    loadMesh(filename);
    initGUI();
    initShaders();
    meshProcess();

}

void Matthew::initShaders() {
    using namespace shaders;
    mShader.init("mesh_shader", simple_vertex, simple_fragment);
    mShaderNormals.init("normal_shader", normals_vertex, normals_fragment, normals_geometry);
}

void Matthew::color_coding(Surface_mesh::Vertex_property <Scalar> prop, Surface_mesh *mesh,
                           Surface_mesh::Vertex_property <surface_mesh::Color> color_prop, int bound) {
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
    if (color_mode == CURVATURE) {
        mShader.setUniform("color_mode", int(curvature_type));
    } else {
        mShader.setUniform("color_mode", int(color_mode));
    }
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

void Matthew::set_color(Surface_mesh::Vertex v, const surface_mesh::Color &col,
                        Surface_mesh::Vertex_property <surface_mesh::Color> color_prop) {
    color_prop[v] = col;
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


void Matthew::initGUI() {
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
        this->color_mode = NORMAL;
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

    performLayout();
}

void Matthew::meshProcess() {
    using namespace surface_mesh;
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
