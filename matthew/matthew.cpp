#include "matthew.h"
#include "mesh_processing.h"
#include <surface_mesh/Surface_mesh.h>

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::to_string;
using std::min;
using std::max;
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
        mShaderNormals.setUniform("normal_selector", normals_computation);
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
                        Surface_mesh::Vertex_property<surface_mesh::Color> color_prop) {
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

    PopupButton *popupBtn;
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

    popupBtn = new PopupButton(window, "Normal Weights");
    Popup *popup = popupBtn->popup();
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
        cout << "New base color: " << base_color.transpose() << endl;
    });

    new Label(colorPopup, "Light Color:", "sans-bold");
    cp = new ColorPicker(colorPopup, light_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        light_color << c.r(), c.g(), c.b();
        cout << "New light color: " << base_color.transpose() << endl;
    });

    new Label(colorPopup, "Edge Color:", "sans-bold");
    cp = new ColorPicker(colorPopup, edge_color);
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
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
