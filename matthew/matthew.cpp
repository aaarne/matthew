#include "matthew.h"
#include <surface_mesh/Surface_mesh.h>
#include <nanogui/opengl.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/colorpicker.h>
#include <nanogui/label.h>
#include <iomanip>
#include <nanogui/textbox.h>
#include "shaders_gen.h"
#include "meshiew.h"
#include "pointiew.h"

using namespace std;
using namespace Eigen;

Matthew::Matthew(bool fs) :
        nanogui::Screen(Eigen::Vector2i(1024, 768), "Matthew", true, fs), demo_mode(fs) {
}

bool has_ending(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

Matthew *Matthew::create_matthew(std::string filename, bool fullscreen) {
    Matthew *matthew;
    if (has_ending(filename, "pcd")) {
        matthew = new Pointiew(fullscreen);
    } else if (has_ending(filename, "obj") || has_ending(filename, "off") || has_ending(filename, "stl")) {
        matthew = new Meshiew(fullscreen);
    } else {
        throw std::invalid_argument("Unknown filetype");
    }
    matthew->filename = filename;
    return matthew;
}

void Matthew::run(std::string mesh_file) {
    if (mesh_file != "") {
        this->filename = mesh_file;
    }
    initShaders();
    load(filename);
    mCamera.arcball = nanogui::Arcball();
    mCamera.arcball.setSize(mSize);
    mCamera.modelZoom = 2 / get_model_dist_max();
    model_center = get_model_center();
    mCamera.modelTranslation = -model_center;
    initGUI();
}

bool Matthew::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
        return true;
    }
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        setVisible(false);
        return true;
    }
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        for (const auto &w : windows) {
            w->setVisible(true);
        }
    }
    return false;
}

void Matthew::draw(NVGcontext *ctx) {
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
            float zval = nanogui::project(model_center, view * model, proj, mSize).z();
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


void Matthew::initGUI() {
    using namespace nanogui;
    auto *window = new Window(this, "Display Control");
    window->setPosition(Vector2i(15, 15));
    window->setLayout(new GroupLayout());
    new Label(window, "Background");
    auto cp = new ColorPicker(window, this->background());
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        this->setBackground(c);
    });

    auto *info = new Window(this, "Info");
    info->setPosition(Vector2i(mFBSize(0) - 300, 15));
    auto grid = new GridLayout(Orientation::Horizontal, 2, Alignment::Minimum, 15, 5);
    grid->setSpacing(0, 10);
    info->setLayout(grid);
    new Label(info, "Filename:", "sans-bold");
    new Label(info, filename, "sans");
    create_gui_elements(window, info);

    new Label(window, "Hide Windows");
    Button *b = new Button(window, "Hide");
    b->setCallback([this]() {
        cout << "Press space to reshow." << endl;
        for (const auto &w : this->windows) {
            w->setVisible(false);
        }
    });

    auto vec2widget = [info](const std::string &title, const Eigen::Vector3f &v) {
        new Label(info, title);
        auto *widget = new Widget(info);
        widget->setLayout(new BoxLayout(Orientation::Horizontal));
        auto add_box = [=](float value) {
            auto *box = new FloatBox<float>(widget);
            box->setEditable(false);
            box->setFontSize(14);
            box->setValue(value);
        };
        add_box(v(0));
        add_box(v(1));
        add_box(v(2));
    };

    vec2widget("Center", model_center);
    vec2widget("Dimensions", this->get_model_dimensions());
    this->windows.push_back(window);
    this->windows.push_back(info);

    if (demo_mode) for (const auto &w : windows) w->setVisible(false);

    performLayout();
}

void Matthew::drawContents() {
    Eigen::Matrix4f model, view, projection;
    computeCameraMatrices(model, view, projection);
    draw(view * model, projection);
}

