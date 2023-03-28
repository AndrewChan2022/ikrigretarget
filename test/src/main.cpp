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


    SoulIK::FBXRW  fbxrw;
    fbxrw.setPath("d:/data/fbx/3D_Avatar2_Rig_0723.fbx");
    fbxrw.readSkeketonMesh();
    fbxrw.writeSkeletonMesh("d:/data/fbx/3D_Avatar2_Rig_0723_out.fbx");


    bool ret = test_libikrigretarget();

    if(ret) {
        printf("test_libikrigretarget success\n");
    } else {
        printf("test_libikrigretarget fail\n");
    }
    
    return 0;
}

