//
// Created by sach_ar on 10/8/21.
//

#ifndef MATTHEW_EMDEDDING_SOLVER_H
#define MATTHEW_EMDEDDING_SOLVER_H


#include <meshiew.h>

#include <utility>

class EmbeddingSolver : public Meshiew {
public:
    EmbeddingSolver(Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces, Eigen::MatrixXi &edges, Eigen::VectorXd &tl);

protected:
    void update_ratio();

    void update();

    void create_gui_elements(nanogui::Window *control, nanogui::Window *info) override;

    double compute_total_error();

    void initShaders() override;

    struct optimization_parameters {
        double rate = 0.2;
        bool keep_boundary = true;
        double z_force = 0.0;
        bool z_force_only_at_center = false;
        double normal_force = 0.0;
        double stiffness = 1.0;
        long n_iterations = 1000000;
        bool auto_detect_convergence = true;
        double min_vel = 1e-3;
        double diffusion_rate = 0.0;
        int smooting_iterations = 0;
    };

    bool optimize(const optimization_parameters &params, bool quiet = false);

    bool optimize_auto(bool quiet);

    double relax_springs(double rate, bool keep_boundary,
                         double z_force = 0, bool center_z_force = false,
                         double normal_force = 0.0, double stiffness = 1.0);

    void diffusion(double rate, int iterations = 1);


    class ScheduleStep {
    public:
        explicit ScheduleStep(std::string desc) : desc(std::move(desc)) {}

        ScheduleStep& rate(double rate) {
            p.rate = rate;
            return *this;
        }

        ScheduleStep& keep_boundary(bool keep) {
            p.keep_boundary = keep;
            return *this;
        }

        ScheduleStep& z_force(double f) {
            p.z_force = f;
            return *this;
        }

        ScheduleStep& normal_force(double f) {
            p.normal_force = f;
            return *this;
        }

        ScheduleStep& stiffness(double s) {
            p.stiffness = s;
            return *this;
        }

        ScheduleStep& n_iterations(long n) {
            p.n_iterations = n;
            return *this;
        }

        ScheduleStep& auto_detect_convergence(bool v) {
            p.auto_detect_convergence = v;
            return *this;
        }

        ScheduleStep& min_vel(double v) {
            p.min_vel = v;
            return *this;
        }

        ScheduleStep& z_force_only_at_center(bool b) {
            p.z_force_only_at_center = b;
            return *this;
        }

        ScheduleStep& diffusion_rate(double r) {
            p.diffusion_rate = r;
            return *this;
        }

        ScheduleStep& smoothing_iterations(int i) {
            p.smooting_iterations = i;
            return *this;
        }

        std::string desc;
        optimization_parameters p;
    };

private:
    nanogui::FloatBox<double> *error_box, *min_vel_box, *stiffness_box, *z_force_box, *rate_box, *vel_box, *normal_force_box, *diffusion_rate_box;
    nanogui::IntBox<long> *total_iterations_box;
    nanogui::IntBox<int> *smoothing_iterations_box;
    nanogui::TextBox *status_box;

    long iterations = 0;
};


#endif //MATTHEW_EMDEDDING_SOLVER_H
