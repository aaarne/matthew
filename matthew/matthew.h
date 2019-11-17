#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/colorpicker.h>
#include <nanogui/checkbox.h>
#include <nanogui/popupbutton.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/textbox.h>
#include <nanogui/tabwidget.h>
#include <surface_mesh/Surface_mesh.h>


#ifndef MATTHEW_H
#define MATTHEW_H


class Matthew : public nanogui::Screen {
public:
    explicit Matthew();
    virtual ~Matthew() = default;

    void run(std::string mesh_file);

protected:

    typedef surface_mesh::Surface_mesh Surface_mesh;
    typedef surface_mesh::Scalar Scalar;
    typedef surface_mesh::Point Point;

    virtual void load(std::string filename) = 0;

    void drawContents() final;

    virtual void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) = 0;

    void initGUI();

    virtual void initShaders() = 0;

    virtual void create_gui_elements() = 0;

    virtual Point get_model_center() = 0;
    virtual float get_model_dist_max() = 0;

    std::string filename;

private:
    struct CameraParameters {
        nanogui::Arcball arcball;
        float zoom = 1.0f, viewAngle = 45.0f;
        float dnear = 0.05f, dfar = 100.0f;
        Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
        Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        Eigen::Vector3f modelTranslation = Eigen::Vector3f::Zero();
        Eigen::Vector3f modelTranslation_start = Eigen::Vector3f::Zero();
        float modelZoom = 1.0f;
    };

    CameraParameters mCamera;
    void computeCameraMatrices(Eigen::Matrix4f &model,
                               Eigen::Matrix4f &view,
                               Eigen::Matrix4f &proj);

    Point model_center;
    bool keyboardEvent(int key, int scancode, int action, int modifiers) override;

    void draw(NVGcontext *ctx) override;

    Eigen::Vector2f getScreenCoord();

    void repaint();

    bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) override;

    bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel,
                          int button, int modifiers) override;

    bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override;

    bool mTranslate = false;
    Eigen::Vector2i mTranslateStart = Eigen::Vector2i(0, 0);

};

#endif
