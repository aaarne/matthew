#include <surface_mesh/Surface_mesh.h>
#include <nanogui/screen.h>
#include <nanogui/glutil.h>
#include <memory>
#include "grid.h"


#ifndef MATTHEW_H
#define MATTHEW_H


class Matthew : public nanogui::Screen {
public:
    explicit Matthew(bool fs);
    ~Matthew() override = default;

    void run();

    static Matthew *create(const std::string &filename, bool fullscreen=false);
    static Matthew *create(surface_mesh::Surface_mesh &mesh, bool fullscreen= false);
    static Matthew *create(Eigen::VectorXf &points, Eigen::VectorXf &colors, bool fullscreen= false);
    static Matthew *create(Eigen::VectorXf &points, bool fullscreen= false);

    void setAdditionalDatafolder(const std::string &s) {additional_data_folder = s;}
    void setLazyMode(const bool &lazy) {lazy_mode = lazy;}

    bool lazy() const {return lazy_mode;}

protected:

    bool resizeEvent(const Eigen::Vector2i &i) override;

    static bool has_ending(std::string const &fullString, std::string const &ending);

    virtual void load_from_file(const std::string &filename) = 0;

    virtual void initModel() = 0;

    void drawContents() override;

    virtual void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) {}

    void initGUI();

    virtual void init_timer() {};

    virtual void initShaders() = 0;

    virtual void create_gui_elements(nanogui::Window *window, nanogui::Window *info) = 0;

    virtual Eigen::Vector3f get_model_center() = 0;
    virtual float get_model_dist_max() = 0;

    virtual Eigen::Vector3f get_model_dimensions() = 0;

    std::vector<std::string> additional_data_files(const std::string &ending) const;

    virtual void recenter();

protected:

    bool lazy_mode = false;
    std::string filename;
    std::string additional_data_folder = "";
    Eigen::Vector3f model_center;

    void add_renderer(const std::shared_ptr<Renderer> &r) {this->renderers.push_back(r);}
    void add_renderer(Renderer *r) {this->renderers.emplace_back(r);}

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

    bool demo_mode = false;

    std::shared_ptr<Grid> grid;
    std::vector<std::shared_ptr<Renderer>> renderers;

    CameraParameters cam;
    nanogui::Window* control;
    nanogui::Window* info;
    void computeCameraMatrices(Eigen::Matrix4f &model,
                               Eigen::Matrix4f &view,
                               Eigen::Matrix4f &proj);

    bool keyboardEvent(int key, int scancode, int action, int modifiers) override;

    Eigen::Vector2f getScreenCoord();

    bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) override;

    bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel,
                          int button, int modifiers) override;

    bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override;

    bool mTranslate = false;
    Eigen::Vector2i mTranslateStart = Eigen::Vector2i(0, 0);
};

namespace matthew {
    void run_app(Matthew *matt);
    Matthew *create_matthew(const std::string &filename, bool fullscreen = false);
    void matthew(const std::string &filename, bool fullscreen = false);
    void matthew(const std::string &filename, bool fullscreen, const std::function<void(Matthew*)>& transformer);
    Matthew *create_show_mesh(surface_mesh::Surface_mesh &mesh, bool fullscreen = false);
    void show_mesh(surface_mesh::Surface_mesh &mesh, bool fullscreen = false);
    Matthew *create_show_point_cloud(Eigen::VectorXf &points, Eigen::VectorXf &colors, bool fullscreen = false);
    void show_point_cloud(Eigen::VectorXf &points, Eigen::VectorXf &colors, bool fullscreen = false);
    Matthew *create_show_point_cloud(Eigen::VectorXf &points, bool fullscreen = false);
    void show_point_cloud(Eigen::VectorXf &points, bool fullscreen = false);
}

#endif
