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

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::to_string;
using std::min;
using std::max;
using namespace surface_mesh;
using namespace nanogui;

class Matthew : public nanogui::Screen {
public:
    Matthew(std::string mesh_file) :
            nanogui::Screen(Eigen::Vector2i(1024, 768), "Matthew"),
            filename(mesh_file),
            base_color(0, 0.6, 0.15),
            light_color(1, 1, 1),
            edge_color(0, 0, 0) {

        loadMesh(mesh_file);
        initGUI();
        initShaders();

        meshProcess();
    }

    void computeValence();

    void computeNormalsWithConstantWeights();

    void computeNormalsByAreaWeights();

    void computeNormalsWithAngleWeights();

    void calc_mean_curvature();

    void calc_uniform_laplacian();

    void calc_gauss_curvature();

    void calc_weights();

    void calc_edges_weights();

    void calc_vertices_weights();

    void loadMesh(string filename);

    void meshProcess();

    void initGUI();

    void initShaders();

    void color_coding(Surface_mesh::Vertex_property<Scalar> prop, Surface_mesh *mesh,
                      Surface_mesh::Vertex_property<surface_mesh::Color> color_prop, int bound = 20);

    static void set_color(Surface_mesh::Vertex v, const surface_mesh::Color &col,
                          Surface_mesh::Vertex_property<surface_mesh::Color> color_prop) {
        color_prop[v] = col;
    }

    surface_mesh::Color value_to_color(Scalar value, Scalar min_value, Scalar max_value);

    ~Matthew();

    static Point computeCenter(Surface_mesh *mesh);

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);

    virtual void draw(NVGcontext *ctx);

    Vector2f getScreenCoord();

    void repaint();

    virtual void drawContents();

    bool scrollEvent(const Vector2i &p, const Vector2f &rel);

    bool mouseMotionEvent(const Vector2i &p, const Vector2i &rel,
                          int button, int modifiers);

    bool mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers);

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
    bool mTranslate = false;
    bool mDrag = false;
    Vector2i mTranslateStart = Vector2i(0, 0);

    void computeCameraMatrices(Eigen::Matrix4f &model,
                               Eigen::Matrix4f &view,
                               Eigen::Matrix4f &proj);

    // Variables for the viewer
    nanogui::GLShader mShader;
    nanogui::GLShader mShaderNormals;
    nanogui::Window *window;
    nanogui::Arcball arcball;

    Point mesh_center = Point(0.0f, 0.0f, 0.0f);
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_valence;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_unicurvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_curvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_gaussian_curv;
    Surface_mesh mesh;

    std::string filename;

    enum COLOR_MODE : int {
        NORMAL = 0, VALENCE = 1, CURVATURE = 2
    };
    enum CURVATURE_TYPE : int {
        UNIMEAN = 2, LAPLACEBELTRAMI = 3, GAUSS = 4
    };

    // Boolean for the viewer
    bool wireframe = false;
    bool normals = false;
    int normals_computation = 0;

    CURVATURE_TYPE curvature_type = UNIMEAN;
    COLOR_MODE color_mode = NORMAL;

    Vector3f base_color, edge_color, light_color;

    PopupButton *popupCurvature;
    FloatBox<float> *coefTextBox;
    IntBox<int> *iterationTextBox;

    // Mesh informations
    int n_vertices = 0;
    int n_faces = 0;
    int n_edges = 0;
    float min_uniLaplace = 1000;
    float max_uniLaplace = -1000;
    float min_mean_curvature = 1000;
    float max_mean_curvature = -1000;
    float min_gauss_curvature = 1000;
    float max_gauss_curvature = -1000;
};
