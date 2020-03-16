//
// Created by sach_ar on 2/11/20.
//

#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <iostream>
#include "color_window.h"

using namespace nanogui;

ColorCodingWindow::ColorCodingWindow(nanogui::Widget *parent, const std::vector<Eigen::Vector3f> &colors)
        : nanogui::Window(parent, "Color Code") {
    auto grid = new GridLayout(Orientation::Horizontal, 4, Alignment::Minimum, 15, 5);
    this->setLayout(grid);
    this->setVisible(true);
    this->setPosition(Vector2i(15, 700));

    for (const auto &c : colors) {
        auto btn = new Button(this, "");
        btn->setBackgroundColor(c);
        btn->setEnabled(false);

        auto box = new IntBox<double>(this, 0.0);
        box->setEditable(false);
        this->boxes.push_back(box);
    }

    (new Button(this, "Hide"))->setCallback([this]() {
        this->setVisible(false);
    });

}

void ColorCodingWindow::update_code(const std::vector<double> &levels) {
    if (boxes.size() == levels.size()) {
        for (int i = 0; i<boxes.size(); i++) {
            boxes[i]->setValue(levels[i]);
        }
    } else {
        std::cerr << "Something's wrong, more levels than boxes..." << std::endl;
    }
}
