//
// Created by sach_ar on 2/6/20.
//

#include "simulink_receiver.h"

using namespace sockpp;
using namespace std;

SimulinkReceiver::SimulinkReceiver(const std::string &host, int port) : addr(host, port) {
	if (!sock.bind(sockpp::inet_address(host, port))) {
		cerr << "Error binding the UDP v4 socket: " << sock.last_error_str() << endl;
	}
	cout << "Simulink Receiver connected to: " << sock.address() << endl;
}

std::vector<double> SimulinkReceiver::read_doubles(int n) {
    char buf[sizeof(double)*n];
    ssize_t received = sock.recv(buf, sizeof(buf));
    std::vector<double> v;
    for (int i = 0; i<received/sizeof(double); i++) {
        v.push_back(*((double*)buf+i));
    }
    return v;
}
