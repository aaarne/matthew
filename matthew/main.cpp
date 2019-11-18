#include <fstream>
#include "meshiew.h"
#include "pointiew.h"
#include "CLI11.hpp"

using namespace std;


int main(int argc, char **argv) {
    CLI::App cli{"Matthew"};

    std::string filename;
    cli.add_option("file,-f,--file", filename, "The mesh/pointcloud to display")->check(CLI::ExistingFile);
    bool fullscreen = false;
    cli.add_flag("--fullscreen,--fs", fullscreen, "Open in fullscreen mode");
    std::vector<float> background_color = {0, 0, 0};
    cli.add_option("--background", background_color, "Background Color (RGB 0..1)")->expected(3);

    CLI11_PARSE(cli, argc, argv);

    if (cli.count("file") == 0) {
        vector<pair<string, string>> filetypes;
        filetypes.emplace_back("obj", "Wavefront OBJ");
        filetypes.emplace_back("off", "Object File Format");
        filetypes.emplace_back("stl", "STL");
        filetypes.emplace_back("pcd", "Point Cloud Data");
        filename = nanogui::file_dialog(filetypes, false);
    }

    Eigen::Vector3f bgcol(background_color[0], background_color[1], background_color[2]);

    nanogui::init();
    nanogui::ref<Matthew> app = Matthew::create_matthew(filename, fullscreen);
    app->setBackground(bgcol);
    app->run();
    app->drawAll();
    app->setVisible(true);
    nanogui::mainloop();
    nanogui::shutdown();

    return 0;
}
