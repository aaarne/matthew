//
// Created by arne on 11/13/19.
//

#ifndef MATTHEW_MESH_PROCESSING_H
#define MATTHEW_MESH_PROCESSING_H

#include "matthew.h"

class Meshiew : public Matthew {
    friend class Matthew;

public:
    explicit Meshiew(bool fs = false);

    virtual ~Meshiew();

protected:
    typedef surface_mesh::Surface_mesh Surface_mesh;
    typedef surface_mesh::Scalar Scalar;
    typedef surface_mesh::Point Point;

    void load_from_file(const std::string &filename) override;

    void load(Surface_mesh &mesh);

    void meshProcess();

    void calc_weights();

    void calc_edges_weights();

    void calc_vertices_weights();

    void computeValence();

    void calc_uniform_laplacian();

    void calc_mean_curvature();

    void calc_gauss_curvature();

    void create_gui_elements(nanogui::Window *control, nanogui::Window *info) override;

    Eigen::Vector3f get_model_dimensions() override;

    void draw(Eigen::Matrix4f mv, Eigen::Matrix4f p) override;

    static Point computeCenter(Surface_mesh *mesh);

    void initModel();

    void initShaders() override;

    Eigen::Vector3f get_model_center() override;

    float get_model_dist_max() override;

    void color_coding(Surface_mesh::Vertex_property <Scalar> prop, Surface_mesh *mesh,
                      Surface_mesh::Vertex_property <surface_mesh::Color> color_prop, int bound = 20);

    static void set_color(Surface_mesh::Vertex v, const surface_mesh::Color &col,
                          Surface_mesh::Vertex_property <surface_mesh::Color> color_prop);

    surface_mesh::Color value_to_color(Scalar value, Scalar min_value, Scalar max_value);

    void upload_color(const std::string &prop_name);

    enum COLOR_MODE : int {
        NORMAL = 0, COLOR_CODE = 1, PLAIN = 2
    };

    enum LIGHT_MODEL : int {
        NO_LIGHT = 0, LAMBERT = 1, PHONG = 2, SHINY = 3, SELF_GLOW = 4
    };

    Point mesh_center;
    float dist_max;

    std::vector<std::string> selectable_properties;
    std::map<std::string, std::string> property_map;

    Eigen::Vector3f base_color;
    COLOR_MODE color_mode = NORMAL;
    Eigen::Vector3f edge_color;
    Eigen::Vector3f light_color;
    nanogui::GLShader mShader;
    nanogui::GLShader mShaderNormals;

    Eigen::MatrixXf mesh_points;
    bool normals = false;
    nanogui::PopupButton *popupCurvature;
    nanogui::Window *window;
    bool wireframe = false;
    bool broken_normals = false;
    nanogui::Button *wireframeBtn;
    Surface_mesh mesh;
};


#endif //MATTHEW_MESH_PROCESSING_H
