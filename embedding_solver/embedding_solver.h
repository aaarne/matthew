//
// Created by sach_ar on 10/8/21.
//

#ifndef MATTHEW_EMDEDDING_SOLVER_H
#define MATTHEW_EMDEDDING_SOLVER_H


#include <meshiew.h>

class EmbeddingSolver : public Meshiew {
public:
    EmbeddingSolver(Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces, Eigen::MatrixXi &edges, Eigen::VectorXd &tl);

protected:
    void update_ratio();

    void update();

    void create_gui_elements(nanogui::Window *control, nanogui::Window *info) override;

    double compute_total_error();

    void initShaders() override;

    double optimize(double rate,
                    bool keep_boundary,
                    double z_force,
                    double stiffness,
                    long n_iterations,
                    bool auto_detect_convergence,
                    double min_vel = 1e-3,
                    bool quiet = false);

    double optimize_auto(bool quiet);

    double relax_springs(double rate, bool keep_boundary, double z_force = 0, double stiffness = 0);



    void diffusion(double rate, int iterations);

private:
    nanogui::FloatBox<double> *error_box, *min_vel_box, *stiffness_box, *z_force_box, *rate_box, *vel_box;
    nanogui::IntBox<long> *total_iterations_box;
    nanogui::TextBox *status_box;

    long iterations = 0;
};


#endif //MATTHEW_EMDEDDING_SOLVER_H
