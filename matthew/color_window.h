//
// Created by sach_ar on 2/11/20.
//

#ifndef CROISSANT_COLOR_WINDOW_H
#define CROISSANT_COLOR_WINDOW_H


#include <nanogui/window.h>
#include <nanogui/textbox.h>

class ColorCodingWindow : public nanogui::Window {
public:
    explicit ColorCodingWindow(nanogui::Widget *parent, const std::vector<Eigen::Vector3f> &colors);

    void update_code(const std::vector<double> &levels);

private:
    std::vector<nanogui::IntBox<double>*> boxes;


};


#endif //CROISSANT_COLOR_WINDOW_H
