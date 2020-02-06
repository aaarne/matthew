//
// Created by sach_ar on 2/6/20.
//

#ifndef CROISSANT_SIMULINK_RECEIVER_H
#define CROISSANT_SIMULINK_RECEIVER_H


#include <string>
#include <sockpp/udp_socket.h>

class SimulinkReceiver {
public:
    SimulinkReceiver(const std::string &host, int port);

    std::vector<double> read_doubles(int n);

protected:
    sockpp::udp_socket sock;
    sockpp::inet_address addr;
};


#endif //CROISSANT_SIMULINK_RECEIVER_H
