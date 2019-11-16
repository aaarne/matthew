#include "meshiew.h"

using namespace std;

int main(int argc, char *argv[]) {
    std::string filename;
    if (argc < 2) {
        vector<pair<string, string>> filetypes;
        filetypes.emplace_back("obj", "Wavefront OBJ");
        filetypes.emplace_back("off", "Object File Format");
        filetypes.emplace_back("stl", "STL");
        filename = nanogui::file_dialog(filetypes, false);
    } else {
        filename = argv[1];
    }

    nanogui::init();

    nanogui::ref<Matthew> app = new Meshiew();
    app->run(filename);
    app->drawAll();
    app->setVisible(true);
    nanogui::mainloop();

    nanogui::shutdown();

    return 0;
}
