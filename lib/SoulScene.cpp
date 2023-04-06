//
//  SoulScene.cpp
//
//
//  Created by kai chen on 3/24/23.
//

#include "SoulScene.hpp"

using namespace SoulIK;

SoulTransform::SoulTransform(glm::mat4& m) {
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(m, scale, rotation, translation, skew, perspective);
}
