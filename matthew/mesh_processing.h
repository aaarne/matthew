//
// Created by arne on 11/13/19.
//

#ifndef MATTHEW_MESH_PROCESSING_H
#define MATTHEW_MESH_PROCESSING_H


#include "matthew.h"

class MatthewImpl : public Matthew {
protected:
    void calc_weights();

    void calc_edges_weights();

    void calc_vertices_weights();

    void computeValence();

    void computeNormalsWithConstantWeights();

    void computeNormalsByAreaWeights();

    void computeNormalsWithAngleWeights();

    void calc_uniform_laplacian();

    void calc_mean_curvature();

    void calc_gauss_curvature();

    static Point computeCenter(Surface_mesh *mesh);

    void loadMesh(std::string filename) override;

    void meshProcess() override;

    int n_faces = 0;
    int n_vertices = 0;
    int n_edges = 0;
};


#endif //MATTHEW_MESH_PROCESSING_H
