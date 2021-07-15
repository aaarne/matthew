//
// Created by arne on 7/15/21.
//

#include <string>
#include "mimp_viewer.h"

using namespace std;

int main(int argc, char **argv) {
    nanogui::init();
    std::string filename = "../data/bunny.off";
    MimpViewer viewer(filename);
    matthew::run_app(&viewer);
}