//
// Created by arne on 11/13/19.
//

#ifndef MATTHEW_MESH_PROCESSING_H
#define MATTHEW_MESH_PROCESSING_H


#include "matthew.h"

class Meshiew : public Matthew {
public:
    explicit Meshiew();
    virtual ~Meshiew();
protected:
    void meshProcess();
    void calc_weights() ;

    void calc_edges_weights();

    void calc_vertices_weights();

    void computeValence();

    void calc_uniform_laplacian();

    void calc_mean_curvature();

    void calc_gauss_curvature();

    void create_gui_elements();

    void drawContents() override ;

    static Point computeCenter(Surface_mesh *mesh);

    void load(std::string filename);

    void initShaders() override;
    Point get_model_center() override;
    float get_model_dist_max() override;

    void color_coding(Surface_mesh::Vertex_property<Scalar> prop, Surface_mesh *mesh,
                      Surface_mesh::Vertex_property<surface_mesh::Color> color_prop, int bound = 20);

    static void set_color(Surface_mesh::Vertex v, const surface_mesh::Color &col,
                          Surface_mesh::Vertex_property<surface_mesh::Color> color_prop);

    surface_mesh::Color value_to_color(Scalar value, Scalar min_value, Scalar max_value);

    enum COLOR_MODE : int {
        SEXY = 0, VALENCE = 1, CURVATURE = 2, PLAIN = 10
    };

    enum CURVATURE_TYPE : int {
        UNIMEAN = 2, LAPLACEBELTRAMI = 3, GAUSS = 4
    };

    enum LIGHT_MODEL : int {
        NO_LIGHT = 0, LAMBERT = 1, PHONG = 2
    };

    int n_faces = 0;
    int n_vertices = 0;
    int n_edges = 0;

    Point mesh_center;
    float dist_max;

    Eigen::Vector3f base_color;
    COLOR_MODE color_mode = SEXY;
    CURVATURE_TYPE curvature_type = GAUSS;
    Eigen::Vector3f edge_color;
    Eigen::Vector3f light_color;
    nanogui::GLShader mShader;
    nanogui::GLShader mShaderNormals;
    Surface_mesh mesh;
    bool normals = false;
    nanogui::PopupButton *popupCurvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_curvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_gaussian_curv;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_unicurvature;
    Surface_mesh::Vertex_property<surface_mesh::Color> v_color_valence;
    nanogui::Window *window;
    bool wireframe = false;
    nanogui::Button *wireframeBtn;

};


#endif //MATTHEW_MESH_PROCESSING_H