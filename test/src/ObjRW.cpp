//
//  ObjRW.cpp
//
//
//  Created by kai chen on 3/24/23.
//
#include "ObjRW.h"
#include <iostream>
#include <fstream>

using namespace SoulIK;

void ObjRW::writeMesh(SoulSkeletonMesh& mesh, std::string path) {
    std::ofstream file;
    file.open(path);

    // v
    for(auto& v : mesh.vertices) {
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }

    // vt

    // vn
    for(auto& v : mesh.normals) {
        file << "vn " << v.x << " " << v.y << " " << v.z << "\n";
    }

    // bone
    for(auto& v : mesh.jointIds) {
        file << "vb " << v.x << " " << v.y << " " << v.z << " " << v.w << "\n";
    }

    // weight
    for(auto& v : mesh.weights) {
        file << "vw " << v.x << " " << v.y << " " << v.z << " " << v.w << "\n";
    }

    // f v/vn/vt
    for (size_t i = 0; i < mesh.indices.size() / 3; i++) {
        file << "f ";
        file << mesh.indices[3*i]+1 << "//" << mesh.indices[3*i]+1 << " ";
        file << mesh.indices[3*i+1]+1 << "//" << mesh.indices[3*i+1]+1 << " ";
        file << mesh.indices[3*i+2]+1 << "//" << mesh.indices[3*i+2]+1 << "\n";
    }
}

