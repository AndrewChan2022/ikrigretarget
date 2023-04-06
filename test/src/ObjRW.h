//
//  ObjRW.h
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once
#include "SoulScene.hpp"

#include <algorithm>
#include <cmath>

#include "glm/vec3.hpp" // glm::vec3
#include "glm/vec4.hpp" // glm::vec4
#include "glm/gtc/quaternion.hpp"   // glm::quat
#include "glm/mat4x4.hpp" // glm::mat4
#include "glm/ext/matrix_transform.hpp" // glm::translate, glm::rotate, glm::scale
#include "glm/ext/matrix_clip_space.hpp" // glm::perspective
#include "glm/ext/scalar_constants.hpp" // glm::pi
#include "glm/gtx/matrix_decompose.hpp"

namespace SoulIK {

    class ObjRW {
    public:

        ObjRW() = default;
        ~ObjRW() = default;

        // read
        //void readSkeketonMesh(std::string inPath, float scale = 1.0);        

        // write
        void writeMesh(SoulSkeletonMesh& mesh, std::string path);
    private:
    };
}
