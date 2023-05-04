//
//  FBXRW.h
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

    class FBXRWImpl;
    class FBXRW {
    public:

        FBXRW();
        ~FBXRW() = default;

        // read
        void readSkeletonMesh(std::string inPath, float scale = 1.0);
        void readPureSkeletonWithDefualtMesh(std::string inPath, std::string const& rootBoneName, float scale = 1.0);

        // write
        void setScene(std::shared_ptr<SoulScene>& soulScene) { m_soulScene = soulScene; }
        void writeSkeletonMesh(std::string outPath, const SoulMetaData* frameRate = nullptr, const SoulMetaData* customFrameRate = nullptr, float scale = 1.0);

        // other
        bool hasAnimation();
        void printScene();
        std::shared_ptr<SoulScene> getSoulScene() { return m_soulScene; }
    private:
    private:
        std::string m_path;
        std::shared_ptr<FBXRWImpl> pimpl;
        std::shared_ptr<SoulScene> m_soulScene;
    };
}
