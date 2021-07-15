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
    virtual void displayClosestPoint(const Eigen::Vector3d &p);

protected:
    void initShaders() override;
    void create_gui_elements(nanogui::Window *control, nanogui::Window *info) override;

protected:
    std::shared_ptr<FrameRenderer> tool_frame_renderer;
    std::shared_ptr<PointRenderer> closest_point_renderer;
};


#endif //MATTHEW_MIMP_VIEWER_H
