#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_() ,textures_(){
    std::ifstream in;
    in.open (filename, std::ifstream::in);
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (int i=0;i<3;i++) iss >> v[i];
            verts_.push_back(v);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<int> f;
            std::vector<int> t;
            std::vector<int> n;
            int itrash, idx,text,norm;
            iss >> trash;
            int i=0;
            while (iss >> idx >> trash >> text >> trash >> norm) {
                idx--; // in wavefront obj all indices start at 1, not zero
                text--;
                norm--;
                f.push_back(idx);
                t.push_back(text);
                n.push_back(norm);
            }
            textures_.push_back(t);
            faces_.push_back(f);
            normals_.push_back(n);
            
        } else if(!line.compare(0,2,"vt")){
            iss>>trash>>trash;
            Vec2f tt;
            for(int i=0;i<2;i++) iss>> tt[i];
            text_cords_.push_back(tt);
        }
        else if(!line.compare(0,2,"vn")){
            iss>>trash>>trash;
            Vec3f tt;
            for(int i=0;i<3;i++) iss>>tt[i];
            norm_cords_.push_back(tt);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
    return (int)verts_.size();
}

int Model::nfaces() {
    return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
    return faces_[idx];
}
std::vector<int> Model::texture(int idx){
    return textures_[idx];
}
std::vector<int> Model::norm(int idx){
    return normals_[idx];
}

Vec3f Model::normal(int i){
    return norm_cords_[i];
}

Vec3f Model::vert(int i) {
    return verts_[i];
}
Vec2f Model::text(int idx){
    return text_cords_[idx];
}


