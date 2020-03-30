//
// Created by arne on 11/13/19.
//

#ifndef MATTHEW_MESH_PROCESSING_H
#define MATTHEW_MESH_PROCESSING_H

#include <point_renderer.h>
#include <timercpp.h>
#include <simulink_receiver.h>
#include <vectorfield_renderer.h>
#include "matthew.h"
#include "line_renderer.h"
#include "point_cloud_renderer.h"
#include "color_window.h"

class Meshiew : public Matthew {
    friend class Matthew;

public:
    explicit Meshiew(bool fs = false);

    virtual ~Meshiew();

protected:
    typedef surface_mesh::Surface_mesh Surface_mesh;
    typedef surface_mesh::Scalar Scalar;
    typedef surface_mesh::Point Point;
    typedef surface_mesh::Surface_mesh::Vertex Vertex;

    void load_from_file(const std::string &filename) override;

    void load(Surface_mesh &mesh);

    void meshProcess();

    void calc_weights();

    void calc_edges_weights();

    enum REMESHING_TYPE : int {
        AVERAGE = 0, CURV = 1
    };

    void remesh(const REMESHING_TYPE &remeshing_type, const int &num_iterations);
    void calc_target_length(const REMESHING_TYPE &remeshing_type);
    void split_long_edges();
    void collapse_short_edges();
    void equalize_valences();

    void tangential_relaxation();

    void calc_principal_curvature_directions();

    void init_timer() override;

    void calc_vertices_weights();

    void computeValence();

    void calc_uniform_laplacian();

    void calc_mean_curvature();

    void calc_gauss_curvature();

    void calc_boundary();

    void create_gui_elements(nanogui::Window *control, nanogui::Window *info) override;

    void create_simulink_receiver(const std::string &host, int port, PointRenderer *pr);

    Eigen::Vector3f get_model_dimensions() override;

    void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) override;

    static Point computeCenter(Surface_mesh *mesh);

    void initModel();

    void initShaders() override;

    Eigen::Vector3f get_model_center() override;

    float get_model_dist_max() override;

    void color_coding(Surface_mesh::Vertex_property <Scalar> prop, Surface_mesh *mesh,
                      Surface_mesh::Vertex_property <surface_mesh::Color> color_prop, int bound = 20);

    surface_mesh::Color value_to_color(Scalar value, Scalar min_value, Scalar max_value, Scalar bound_min, Scalar bound_max);

    void upload_color(const std::string &prop_name);

    enum COLOR_MODE : int {
        NORMAL = 0, COLOR_CODE = 1
    };

    Point mesh_center;
    float dist_max;

    std::vector<std::string> selectable_scalar_properties;
    std::vector<std::string> selectable_vector_properties;
    std::map<std::string, std::string> property_map;

    ColorCodingWindow *color_coding_window;

    Eigen::Vector3f base_color;
    COLOR_MODE color_mode = NORMAL;
    Eigen::Vector3f edge_color;
    Eigen::Vector3f light_color;
    nanogui::GLShader mShader;

    Eigen::MatrixXf mesh_points;
    nanogui::Window *window;
    bool wireframe = false;
    bool broken_normals = false;
    bool hide_mesh = false;
    Surface_mesh mesh;

    int n_boundary_points;

    struct line_renderer_settings {
        bool show_raw_data;
        std::string prop_name;
        int n_lines;
        Eigen::Vector3f color;
        bool point_trace_mode;
        int point_renderer_id;
    };

    std::shared_ptr<LineRenderer> boundary_renderer;
    std::shared_ptr<VectorfieldRenderer> normals_renderer;

    std::vector<LineRenderer*> line_renderers;
    std::map<LineRenderer*, line_renderer_settings> line_renderer_settings;

    std::vector<PointRenderer*> point_renderers;
    std::vector<VectorfieldRenderer*> vectorfield_renderers;
    std::vector<PointCloudRenderer*> point_cloud_renderers;

    Timer t;
    std::vector<std::pair<std::shared_ptr<SimulinkReceiver>, PointRenderer*>> receivers;
};


#endif //MATTHEW_MESH_PROCESSING_H
