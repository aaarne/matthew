#include "matthew.h"
#include <surface_mesh/Surface_mesh.h>
#include <nanogui/opengl.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/colorpicker.h>
#include <nanogui/label.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/checkbox.h>
#include "shaders_gen.h"
#include "meshiew.h"
#include "pointiew.h"
#include "dirent.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

Matthew::Matthew(bool fs) :
        nanogui::Screen(Eigen::Vector2i(1680, 1050), "Matthew", true, fs), demo_mode(fs) {
}

bool Matthew::has_ending(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

Matthew *Matthew::create(const std::string &filename, bool fullscreen) {
    Matthew *matthew;
    vector<string> mshtypes = {"obj", "off", "stl", "msh"};
    if (has_ending(filename, "pcd")) {
        matthew = new Pointiew(fullscreen);
    } else if ((filename == "-") || std::any_of(mshtypes.begin(), mshtypes.end(), [&](const string& ending) {return has_ending(filename, ending);})) {
        matthew = new Meshiew(fullscreen);
    } else {
        throw std::invalid_argument("Unknown filetype");
    }
    matthew->load_from_file(filename);
    matthew->filename = filename;
    return matthew;
}

vector<string> files_in_directory(const string& dir) {
    vector<string> files;
    shared_ptr<DIR> directory_ptr(opendir(dir.c_str()), [](DIR* dir){ dir && closedir(dir); });
    struct dirent *dirent_ptr;
    if (!directory_ptr) {
        cerr << "Error opening " << dir << ": " << strerror(errno) << " " << dir << endl;
        return files;
    }

    while ((dirent_ptr = readdir(directory_ptr.get())) != nullptr) {
        files.emplace_back(dir + "/" + dirent_ptr->d_name);
    }
    return files;
}

bool has_ending(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

void Matthew::run() {
    initShaders();
    initModel();

    cam.arcball = nanogui::Arcball();
    cam.arcball.setSize(mSize);
    cam.modelZoom = 2 / get_model_dist_max();
    model_center = get_model_center();
    cam.modelTranslation = -model_center;

    Eigen::Vector3f dim = this->get_model_dimensions();
    grid = std::make_shared<Grid>(11, max(dim(0), dim(1)), get_model_center());
    grid->init();
    grid->setIntensity(1.f);
    this->renderers.push_back(grid);

    initGUI();
    init_timer();
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
        control->setVisible(true);
    }
    return false;
}

Vector2f Matthew::getScreenCoord() {
    Vector2i pos = mousePos();
    return Vector2f(2.0f * (float) pos.x() / width() - 1.0f,
                    1.0f - 2.0f * (float) pos.y() / height());
}

bool Matthew::scrollEvent(const Vector2i &p, const Vector2f &rel) {
    if (!Screen::scrollEvent(p, rel)) {
        cam.zoom = max(0.1, cam.zoom * (rel.y() > 0 ? 1.1 : 0.9));
    }
    return true;
}

bool Matthew::mouseMotionEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers) {
    if (!Screen::mouseMotionEvent(p, rel, button, modifiers)) {
        if (cam.arcball.motion(p)) {
        } else if (mTranslate) {
            Eigen::Matrix4f model, view, proj;
            computeCameraMatrices(model, view, proj);
            float zval = nanogui::project(model_center, view * model, proj, mSize).z();
            Eigen::Vector3f pos1 = nanogui::unproject(
                    Eigen::Vector3f(p.x(), mSize.y() - p.y(), zval), view * model, proj, mSize);
            Eigen::Vector3f pos0 = nanogui::unproject(
                    Eigen::Vector3f(mTranslateStart.x(), mSize.y() -
                                                         mTranslateStart.y(), zval), view * model, proj, mSize);
            cam.modelTranslation = cam.modelTranslation_start + (pos1 - pos0);
        }
        return true;
    }
}

bool Matthew::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {
    if (!Screen::mouseButtonEvent(p, button, down, modifiers)) {
        if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
            cam.arcball.button(p, down);
        } else if (button == GLFW_MOUSE_BUTTON_2 ||
                   (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
            cam.modelTranslation_start = cam.modelTranslation;
            mTranslate = true;
            mTranslateStart = p;
        }
    }
    if (button == GLFW_MOUSE_BUTTON_1 && !down) {
        cam.arcball.button(p, false);
    }
    if (!down) {
        mTranslate = false;
    }
    return true;
}


void Matthew::computeCameraMatrices(Eigen::Matrix4f &model, Eigen::Matrix4f &view, Eigen::Matrix4f &proj) {

    view = nanogui::lookAt(cam.eye, cam.center, cam.up);

    float fH = std::tan(cam.viewAngle / 360.0f * M_PI) * cam.dnear;
    float fW = fH * (float) mSize.x() / (float) mSize.y();

    proj = nanogui::frustum(-fW, fW, -fH, fH, cam.dnear, cam.dfar);
    model = cam.arcball.matrix()
          * nanogui::scale(Eigen::Vector3f::Constant(cam.zoom * cam.modelZoom))
          * nanogui::translate(cam.modelTranslation);
}


void Matthew::initGUI() {
    using namespace nanogui;
    control = new Window(this, "Display Control");
    control->setPosition(Vector2i(15, 15));
    control->setLayout(new GroupLayout());
    new Label(control, "Background");
    auto cp = new ColorPicker(control, this->background());
    cp->setFixedSize({100, 20});
    cp->setCallback([this](const Color &c) {
        this->setBackground(c);
    });

    new Label(control, "Grid");
    auto chkbox = new CheckBox(control, "Show Grid");
    chkbox->setCallback([this](bool value) {
        this->grid->setVisible(value);
    });

    auto *slider = new Slider(control);
    slider->setValue(1.0);
    slider->setCallback([this](float value) {
        this->grid->setIntensity(value);
    });

    info = new Window(this, "Info");
    info->setVisible(false);
    info->setPosition(Vector2i(mFBSize(0) - 300, 15));
    auto grid = new GridLayout(Orientation::Horizontal, 2, Alignment::Minimum, 15, 5);
    grid->setSpacing(0, 10);
    info->setLayout(grid);
    if (!filename.empty()) {
        new Label(info, "Filename:", "sans-bold");
        new Label(info, filename, "sans");
    }
    create_gui_elements(control, info);

    new Label(control, "Windows");
    auto *b = new Button(control, "Info");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool value) {
        info->setVisible(value);
    });
    b = new Button(control, "Hide");
    b->setCallback([this]() {
        cout << "Press space to reshow." << endl;
        control->setVisible(false);
        info->setVisible(false);
    });

    auto vec2widget = [this](const std::string &title, const Eigen::Vector3f &v) {
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

    if (demo_mode) {
        control->setVisible(false);
        info->setVisible(false);
    }

    performLayout();
}

void Matthew::drawContents() {
    Eigen::Matrix4f model, view, projection;
    computeCameraMatrices(model, view, projection);
    Eigen::Matrix4f mv = view * model;
    draw(mv, projection);
    for (const auto &r : renderers) {
        r->draw(mv, projection);
    }
}

Matthew* Matthew::create(surface_mesh::Surface_mesh &mesh, bool fullscreen) {
    auto matt = new Meshiew(fullscreen);
    matt->mesh = mesh;
    return matt;
}

Matthew* Matthew::create(Eigen::VectorXf &points, bool fullscreen) {
    auto matt = new Pointiew(fullscreen);
    matt->points = points;
    matt->has_color = false;
    return matt;
}

Matthew* Matthew::create(Eigen::VectorXf &points, Eigen::VectorXf &colors, bool fullscreen) {
    auto matt = new Pointiew(fullscreen);
    matt->points = points;
    matt->colors = colors;
    matt->has_color = true;
    return matt;
}

bool Matthew::resizeEvent(const Vector2i &i) {
    cam.arcball.setSize(i);
    return true;
}

std::vector<std::string> Matthew::additional_data_files(const string &ending) const {
    std::vector<std::string> filenames;
    if (this->additional_data_folder.empty()) {
        return filenames;
    }
    auto found = files_in_directory(additional_data_folder);

    std::copy_if(found.begin(), found.end(), std::back_inserter(filenames),
            [ending](const std::string &s) {return has_ending(s, ending);});
    return filenames;
}

void matthew::run_app(Matthew *matt) {
    nanogui::ref<Matthew> app = matt;
    app->run();
    app->drawAll();
    app->setVisible(true);
    if (matt->lazy()) {
      nanogui::mainloop(0);
    } else {
      nanogui::mainloop();
    }
    nanogui::shutdown();
}

Matthew *matthew::create_matthew(const std::string &filename, bool fullscreen) {
    return Matthew::create(filename, fullscreen);
}

void matthew::matthew(const std::string &filename, bool fullscreen) {
    nanogui::init();
    run_app(create_matthew(filename, fullscreen));
}

Matthew *matthew::create_show_mesh(surface_mesh::Surface_mesh &mesh, bool fullscreen) {
    return Matthew::create(mesh, fullscreen);
}

void matthew::show_mesh(surface_mesh::Surface_mesh &mesh, bool fullscreen) {
    nanogui::init();
    run_app(create_show_mesh(mesh, fullscreen));
}

Matthew *matthew::create_show_point_cloud(Eigen::VectorXf &points, VectorXf &colors, bool fullscreen) {
    return Matthew::create(points, colors, fullscreen);
}

void matthew::show_point_cloud(Eigen::VectorXf &points, Eigen::VectorXf &colors, bool fullscreen) {
    nanogui::init();
    run_app(create_show_point_cloud(points, colors, fullscreen));
}

Matthew *matthew::create_show_point_cloud(Eigen::VectorXf &points, bool fullscreen) {
    return Matthew::create(points, fullscreen);
}

void matthew::show_point_cloud(Eigen::VectorXf &points, bool fullscreen) {
    nanogui::init();
    run_app(create_show_point_cloud(points, fullscreen));
}

void matthew::matthew(const std::string &filename, bool fullscreen, const std::function<void(Matthew *)>& transformer) {
    nanogui::init();
    auto matt = create_matthew(filename, fullscreen);
    transformer(matt);
    run_app(matt);
}
