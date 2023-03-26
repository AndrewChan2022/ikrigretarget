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

namespace Soul {

    class FBXReader {
    public:

        FBXReader() = default;
        ~FBXReader() = default;
        FBXReader(std::string inPath) {
            m_path = inPath;
        }
        void setPath(std::string inPath) {
            m_path = inPath;
        }
        void readSkeketon() {
            m_skeleton = nullptr;

        }

    private:
        std::string m_path;
        std::shared_ptr<USkeleton> m_skeleton;
    };
}
















extern "C" 
bool test_libikrigretarget();

int main(int argc, char *argv[]) {

    bool ret = test_libikrigretarget();

    if(ret) {
        printf("test_libikrigretarget success\n");
    } else {
        printf("test_libikrigretarget fail\n");
    }
    
    return 0;
}

