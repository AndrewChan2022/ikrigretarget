//
//  FBXRW.h
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
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

    // skeleton
    struct FbxJoint {
        std::string             name;
        uint32_t                parentId;
        glm::mat4               inverseWorldMatrix{ 1.0 };
        glm::mat4               offsetMatrix{ 1.0 };
    };
    struct FbxSkeleton {
        std::vector<FbxJoint>   joints;
    };

    // skeleton animation
    struct FbxVec3Key {
        float       time;
        glm::vec3   value;
    };
    struct FbxQuatKey {
        float       time;
        glm::quat   value;
    };
    struct FbxAniChannel {
        uint32_t    jointId;
        std::vector<FbxVec3Key> PositionKeys;
        std::vector<FbxVec3Key> ScalingKeys;
        std::vector<FbxQuatKey> RotationKeys;
    };
    struct FbxJointAnimation {
        std::string name;
        std::vector<FbxAniChannel> channels;
    };

    // 
    struct FbxSkeletonMesh {
        std::string             name;

        // geom
        std::vector<glm::vec3>  vertices;
        std::vector<glm::vec3>  normals;
        std::vector<glm::vec3>  tangents;
        std::vector<glm::vec2>  uvs;
        std::vector<glm::vec2>  uvs2;
        std::vector<uint32_t>   indices;

        // skin
        std::vector<glm::ivec4> jointIds;
        std::vector<glm::vec4>  weights;
        std::vector<uint8_t>    weightCounts;

        // skeleton
        FbxSkeleton             skeleton;

        // skeleton animation
        FbxJointAnimation       animation;
        
        // morph
        // material and others ...
    };

    class FBXRW {
    public:

        FBXRW() = default;
        ~FBXRW() = default;

        // path
        FBXRW(std::string inPath) {
            m_path = inPath;
        }
        void setPath(std::string inPath) {
            m_path = inPath;
        }

        // read
        void readSkeketonMesh(float scale = 1.0);
        void setSkeleton(std::vector<std::shared_ptr<FbxSkeletonMesh>>& skmeshes) {
            m_skeletonMeshes = skmeshes;
        }

        // write
        void writeSkeletonMesh(std::string outPath, float scale = 1.0);

        bool hasAnimation() {
            for(auto& skm : m_skeletonMeshes) {
                if (skm->animation.channels.size() != 0) { 
                    return true; 
                }
            }
            return false;
        }
    private:
    private:
        std::string m_path;
        std::vector<std::shared_ptr<FbxSkeletonMesh>> m_skeletonMeshes;
    };

}

