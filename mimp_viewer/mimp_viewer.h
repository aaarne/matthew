//
// Created by arne on 7/15/21.
//

#ifndef MATTHEW_MIMP_VIEWER_H
#define MATTHEW_MIMP_VIEWER_H


#include <meshiew.h>

class MimpViewer : public Meshiew {
public:
    explicit MimpViewer(const std::string &filename);

    virtual void displayToolFrame(const Eigen::Ref<const Eigen::Matrix<float, 4, 4>> frame);
    virtual void displayClosestPoint(const Eigen::Vector3f &p);
    virtual void displayCoordinates(const Eigen::Vector3f &c);
    virtual void displayConnectionVector(const Eigen::Ref<const Eigen::Matrix<float, 4, 4>> frame,
                                         const Eigen::Vector3f &closest_point);

protected:
    void initShaders() override;
    void create_gui_elements(nanogui::Window *control, nanogui::Window *info) override;

protected:
    std::shared_ptr<FrameRenderer> tool_frame_renderer;
    std::shared_ptr<PointRenderer> closest_point_renderer;
    std::shared_ptr<LineRenderer> connection_renderer;

    std::vector<nanogui::ProgressBar*> progressbars;
    std::vector<nanogui::FloatBox<double>*> coordinate_labels;

    bool initialized = false;
};


#endif //MATTHEW_MIMP_VIEWER_H
