//
// Created by arne on 7/15/21.
//

#include <string>
#include "mimp_viewer.h"
#include <thread>
#include "mimp_state.pb.h"
#include <sockpp/udp_socket.h>

#define HOST "172.31.1.148"
//#define HOST "localhost"
#define PORT 2505

using namespace std;

int main(int argc, char **argv) {
    nanogui::init();
    auto viewer = new MimpViewer(argv[1]);

    sockpp::udp_socket sock;
    sockpp::inet_address addr(HOST, PORT);
    if (!sock.bind(addr)) {
        cerr << "Error binding the UDP socket: " << sock.last_error_str() << endl;
        return 1;
    } else {
        cout << "Listening on: " << sock.address() << endl;
    }
    bool running = true;
    thread t([&running, viewer, &sock]() {
        char buf[1024];
        MimpState state;
        Eigen::Matrix<float, 4, 4> trafo;
        Eigen::Vector3d coordinates, closest;
        while (running) {
            int received = sock.recv(buf, sizeof(buf));
            state.ParseFromArray(buf, received);
            assert(state.tool_trafo_size() == 16);
            assert(state.coordinates_size() == 3);
            assert(state.closest_point_size() == 3);
            for (int row = 0; row < 4; row++) {
                for (int col = 0; col < 4; col++) {
                    trafo(row, col) = state.tool_trafo(4*row+col);
                }
            }
            for (int i = 0; i<3; i++) {
                coordinates(i) = state.coordinates(i);
                closest(i) = state.closest_point(i);
            }
            viewer->displayToolFrame(trafo);
            viewer->displayClosestPoint(closest.cast<float>());
            viewer->displayCoordinates(coordinates.cast<float>());
            viewer->displayConnectionVector(trafo, closest.cast<float>());
        }
    });
    matthew::run_app(viewer);
    running = false;
    t.join();
    delete viewer;
}
