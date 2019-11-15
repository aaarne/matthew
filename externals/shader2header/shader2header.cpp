//
// Created by arne on 11/14/19.
//

#include <iostream>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <vector>
#include <memory>
#include <cstring>
#include "dirent.h"

using namespace std;

vector<string> files_in_directory(const string& dir) {
  vector<string> files;
  shared_ptr<DIR> directory_ptr(opendir(dir.c_str()), [](DIR* dir){ dir && closedir(dir); });
  struct dirent *dirent_ptr;
  if (!directory_ptr) {
    cerr << "Error opening : " << strerror(errno) << " " << dir << endl;
    return files;
  }

  while ((dirent_ptr = readdir(directory_ptr.get())) != nullptr) {
    files.emplace_back(dir + "/" + dirent_ptr->d_name);
  }
  return files;
}

bool has_ending(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cerr << "Wrong usage. Proper usage: shader2header headername.h shader_folder" << endl;
        exit(1);
    }

    const string directory = argv[2];
    ofstream out(argv[1]);

    out << R"(// Auto-generated header file with shader code string literals

#include <string>

namespace shaders {
)" << endl;

    for (const auto &file : files_in_directory(directory)) {
        if (has_ending(file, ".glsl")) {
            string shader_name = file.substr(file.find_last_of("/\\") + 1);
            shader_name = shader_name.substr(0, shader_name.length() - 5);
            out << "/**" << endl;
            out << " * Shader: " << shader_name << endl;
            out << " * Filename: " << file << endl;
            out << " */" << endl;
            out << "const std::string " << shader_name << " = R\"(" << endl;

            string line;
            ifstream in(file);

            while (getline(in, line)) {
                out << line << endl;
            }

            out << ")\";" << endl << endl;

            in.close();
        }
    }

    out << "}" << endl;
    out.close();
    return 0;
}
