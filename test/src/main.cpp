//
//  main.cpp
//  test libGLVNDGLContext
//
//  Created by kai chen on 2/8/23.
//

//#include    <stdio.h>
//#include    <stdlib.h>
#include    "assimp/Importer.hpp"
//#include    "assimp/scene.h"
//#include    "assimp/postprocess.h"
#include <stdio.h>
#include "SoulTransform.h"
#include "SoulRetargeter.h"

#include "FBXRW.h"















extern "C" 
bool test_libikrigretarget();

int main(int argc, char *argv[]) {

    std::string file_path = __FILE__;
#ifdef _WIN64
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    //std::string inputfile = dir_path + "\\..\\..\\model\\3D_Avatar2_Rig_0723.fbx";
    //std::string outfile = dir_path + "\\..\\..\\model\\3D_Avatar2_Rig_0723_out.fbx";
    std::string inputfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts.fbx";
    std::string outfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts_tiny_out.fbx";
#else
    std::string dir_path = file_path.substr(0, file_path.rfind("/"));
    std::string inputfile = dir_path + "/../../model/3D_Avatar2_Rig_0723.fbx";
    std::string outfile = dir_path + "/../../model/3D_Avatar2_Rig_0723_out.fbx";
#endif

    SoulIK::FBXRW  fbxrw;
    fbxrw.readSkeketonMesh(inputfile);
    fbxrw.writeSkeletonMesh(outfile);


    bool ret = test_libikrigretarget();

    if(ret) {
        printf("test_libikrigretarget success\n");
    } else {
        printf("test_libikrigretarget fail\n");
    }
    
    return 0;
}

