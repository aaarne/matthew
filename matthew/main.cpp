#include "mesh_processing.h"

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

    try {
        nanogui::init();
        {
            nanogui::ref<Matthew> app = new MatthewImpl();
            app->run(filename);
            app->drawAll();
            app->setVisible(true);
            nanogui::mainloop();
        }

        nanogui::shutdown();

    } catch (const std::runtime_error &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
        std::cerr << error_msg << std::endl;
        return -1;
    }

    return 0;
}
