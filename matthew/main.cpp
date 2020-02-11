#include <fstream>
#include "matthew.h"
#include "CLI11.hpp"

using namespace std;

int main(int argc, char **argv) {
    CLI::App cli{"Matthew"};

    std::string filename;
    cli.add_option("file,-f,--file", filename, "The mesh/pointcloud to display")->check(CLI::ExistingFile);
    bool fullscreen = false;
    bool read_stdin = false;
    std::string additional_data_folder = "";
    cli.add_flag("--fullscreen,--fs", fullscreen, "Open in fullscreen mode");
    cli.add_flag("-s,--stdin", read_stdin, "Read mesh from stdin");
    cli.add_option("-a,--additional_data_folder", additional_data_folder, "Folder for additional data files");
    std::vector<float> background_color = {0.3, 0.3, 0.32};
    cli.add_option("--background", background_color, "Background Color (RGB 0..1)")->expected(3);

    CLI11_PARSE(cli, argc, argv);

    if (!read_stdin && cli.count("file") == 0) {
        vector<pair<string, string>> filetypes;
        filetypes.emplace_back("obj", "Wavefront OBJ");
        filetypes.emplace_back("off", "Object File Format");
        filetypes.emplace_back("stl", "STL");
        filetypes.emplace_back("pcd", "Point Cloud Data");
        filename = nanogui::file_dialog(filetypes, false);
    }

    if (read_stdin) {
        filename = "-";
    }

    Eigen::Vector3f bgcol(background_color[0], background_color[1], background_color[2]);

    matthew::matthew(filename, fullscreen, [&](Matthew *app) {
        app->setBackground(bgcol);
        if (!additional_data_folder.empty()) {
            app->setAdditionalDatafolder(additional_data_folder);
        }
        cout << "Matthew created." << endl;
    });

    return 0;
}
