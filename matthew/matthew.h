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
    virtual ~Matthew();

    void run(std::string mesh_file);

protected:

    typedef surface_mesh::Surface_mesh Surface_mesh;
    typedef surface_mesh::Scalar Scalar;
    typedef surface_mesh::Point Point;

    virtual void loadMesh(std::string filename) = 0;

    virtual void meshProcess() = 0;

    void initShaders();
    void initGUI();

    void color_coding(Surface_mesh::Vertex_property<Scalar> prop, Surface_mesh *mesh,
                      Surface_mesh::Vertex_property<surface_mesh::Color> color_prop, int bound = 20);

    static void set_color(Surface_mesh::Vertex v, const surface_mesh::Color &col,
                          Surface_mesh::Vertex_property<surface_mesh::Color> color_prop);

    surface_mesh::Color value_to_color(Scalar value, Scalar min_value, Scalar max_value);


    Eigen::Vector3f base_color;

    enum COLOR_MODE : int {
        NORMAL = 0, VALENCE = 1, CURVATURE = 2, PLAIN = 10
    };

    enum CURVATURE_TYPE : int {
        UNIMEAN = 2, LAPLACEBELTRAMI = 3, GAUSS = 4
    };

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

    COLOR_MODE color_mode = NORMAL;
    CURVATURE_TYPE curvature_type = GAUSS;
    Eigen::Vector3f edge_color;
    std::string filename;
    Eigen::Vector3f light_color;
    CameraParameters mCamera;
    // Variables for the viewer
    nanogui::GLShader mShader;
    nanogui::GLShader mShaderNormals;
    Surface_mesh mesh;
    Point mesh_center = Point(0.0f, 0.0f, 0.0f);
    bool normals = false;
    int normals_computation = 0;
    nanogui::PopupButton *popupCurvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_curvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_gaussian_curv;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_unicurvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_valence;
    nanogui::Window *window;
    // Boolean for the viewer
    bool wireframe = false;
    nanogui::Button *wireframeBtn;

private:
    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);

    virtual void draw(NVGcontext *ctx);

    Eigen::Vector2f getScreenCoord();

    void repaint();

    virtual void drawContents();

    bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel);

    bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel,
                          int button, int modifiers);

    bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers);


    bool mTranslate = false;
    bool mDrag = false;
    Eigen::Vector2i mTranslateStart = Eigen::Vector2i(0, 0);

    void computeCameraMatrices(Eigen::Matrix4f &model,
                               Eigen::Matrix4f &view,
                               Eigen::Matrix4f &proj);

    nanogui::Arcball arcball;

};

#endif
