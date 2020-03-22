//
// Created by arne on 3/22/20.
//

#include "pcd_reader.h"

#include <iostream>
#include <fstream>

using namespace std;

PCDReader::PCDReader(const std::string &filename) {
    std::ifstream in(filename);
    string msg;
    do {
        in >> msg;
    } while (msg != "FIELDS");
    in >> msg;
    int n_fields = 0;
    do {
        in >> msg;
        n_fields++;
    } while (msg != "SIZE");
    cout << "n_fields: " << n_fields << endl;
    do {
        in >> msg;
    } while (msg != "POINTS");
    in >> msg;
    int n;
    stringstream ss(msg);
    ss >> n;
    cout << "Amount of points: " << n << endl;
    do {
        in >> msg;
    } while (msg != "ascii");

    Eigen::MatrixXf points(3, n);
    Eigen::MatrixXf colors(4, n);
    int i = 0;

    if (n_fields == 3) {
        color = false;
        while (in >> msg) {
            float v;
            stringstream ss(msg);
            ss >> v;
            points(i % 3, i / 3) = v;
            i++;
        }
    } else if (n_fields == 4) {
        color = true;
        while (in >> msg) {
            stringstream ss(msg);
            if (i % 4 == 3) {
                unsigned int c;
                ss >> c;
                colors(0, i / 4) = (c >> 0 & 0xff) / 255.0f;
                colors(1, i / 4) = (c >> 8 & 0xff) / 255.0f;
                colors(2, i / 4) = (c >> 16 & 0xff) / 255.0f;
                colors(3, i / 4) = (c >> 24 & 0xff) / 255.0f;
            } else {
                float v;
                ss >> v;
                points(i % 4, i / 4) = v;
            }
            i++;
        }
    } else {
        cerr << "Unexpected number of fields in point cloud data" << endl;
        exit(1);
    }
    in.close();
    this->points = points;
    this->colors = colors;
}
