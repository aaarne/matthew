#include "matthew.h"
#include <surface_mesh/Surface_mesh.h>
#include "shaders_gen.h"

using namespace std;
using namespace Eigen;

Matthew::Matthew() :
        nanogui::Screen(Eigen::Vector2i(1024, 768), "Matthew") {

}

void Matthew::run(std::string mesh_file) {
    this->filename = mesh_file;
    initShaders();
    load(filename);
    initGUI();

    mCamera.arcball = nanogui::Arcball();
    mCamera.arcball.setSize(mSize);
    mCamera.modelZoom = 2 / get_model_dist_max();
    model_center = get_model_center();
    mCamera.modelTranslation = -Vector3f(model_center.x, model_center.y, model_center.z);

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
            float zval = nanogui::project(Vector3f(model_center.x,
                                                   model_center.y,
                                                   model_center.z),
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
    create_gui_elements();
    performLayout();
}

