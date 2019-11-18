#include <fstream>
#include "meshiew.h"
#include "pointiew.h"
#include "CLI11.hpp"

using namespace std;

bool has_ending(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char **argv) {
    CLI::App cli{"Matthew"};
    std::string filename;
    bool fullscreen = false;

    cli.add_option("file,-f,--file", filename, "The mesh/pointcloud to display");
    cli.add_flag("--fullscreen,--fs", fullscreen, "Open in fullscreen mode");

    CLI11_PARSE(cli, argc, argv);

    if (cli.count("file") == 0) {
        vector<pair<string, string>> filetypes;
        filetypes.emplace_back("obj", "Wavefront OBJ");
        filetypes.emplace_back("off", "Object File Format");
        filetypes.emplace_back("stl", "STL");
        filetypes.emplace_back("pcd", "Point Cloud Data");
        filename = nanogui::file_dialog(filetypes, false);
    }

    ifstream testfile(filename);

    if (!testfile) {
        cerr << "File " << filename << " not present." << endl;
        exit(2);
    } else {
        testfile.close();
    }

    nanogui::init();
    nanogui::ref<Matthew> app;

    if (has_ending(filename, "pcd")) {
        app = new Pointiew(fullscreen);
    } else if (has_ending(filename, "obj") || has_ending(filename, "off") || has_ending(filename, "stl")) {
        app = new Meshiew(fullscreen);
    } else {
        cerr << "Unknown filetype" << endl;
        exit(1);
    }

    app->run(filename);
    app->drawAll();
    app->setVisible(true);
    nanogui::mainloop();
    nanogui::shutdown();

    return 0;
}
