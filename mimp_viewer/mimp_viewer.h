//
// Created by arne on 7/15/21.
//

#ifndef MATTHEW_MIMP_VIEWER_H
#define MATTHEW_MIMP_VIEWER_H


#include <meshiew.h>

class MimpViewer : public Meshiew {
public:
    explicit MimpViewer(const std::string &filename);

protected:
    void initShaders() override;

protected:
    std::shared_ptr<FrameRenderer> tool_frame_renderer;
    std::shared_ptr<PointRenderer> closest_point_renderer;
};


#endif //MATTHEW_MIMP_VIEWER_H
