#include "matthew.h"

int main(int argc, char *argv[]) {
    std::string filename;
    if (argc < 2) {
        filename = nanogui::file_dialog({
                                                {"obj", "Wavefront OBJ"},
                                                {"off", "Object File Format"},
                                                {"stl", "STL"}
                                        }, false);
    } else {
        filename = argv[1];
    }

    try {
        nanogui::init();
        {
            nanogui::ref<Matthew> app = new Matthew(filename);
            app->drawAll();
            app->setVisible(true);
            nanogui::mainloop();
        }

        nanogui::shutdown();
    } catch (const std::runtime_error &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
#if defined(_WIN32)
        MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
#else
        std::cerr << error_msg << std::endl;
#endif
        return -1;
    }

    return 0;
}
