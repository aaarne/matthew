//
// Created by arne on 11/14/19.
//

#include <iostream>
#include <fstream>
#include <algorithm>
#include <filesystem>

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cerr << "Wrong usage. Proper usage: shader2header headername.h shader_folder" << endl;
        exit(1);
    }

    const string directory = argv[2];
    ofstream out(argv[1]);


}