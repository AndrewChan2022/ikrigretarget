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

    // meta variant define, similar to std::variant
    enum class FbxMetadataType: uint8_t {
        BOOL = 0,
        INT32 = 1,
        UINT64 = 2,
        FLOAT = 3,
        DOUBLE = 4,
        STRING = 5,
        VEC3 = 6,
        METADATA = 7,
        INT64 = 8,
        UINT32 = 9,
        META_MAX = 10,
    };
    struct FbxMetaData;
    struct FbxMetadataValue {  // cannot be union, not know how to destroy
        bool boolValue;
        int32_t int32Value;
        uint64_t uint64Value;
        float floatValue;
        double doubleValue;
        std::string stringValue;
        glm::vec3 vec3Value;
        std::vector<FbxMetaData> metadataValue;  // no need wrapper of pointer
        int64_t int64Value;
        uint32_t uint32Value;
    };
    struct FbxMetaData {
        std::string key;
        FbxMetadataType type;
        FbxMetadataValue value; // data or more metadata in nestled
    };

    // skeleton
    struct FbxJoint {
        std::string             name;
        uint32_t                parentId;

        // transforms from mesh space to bone space in bind pose
        glm::mat4               inverseWorldMatrix{ 1.0 };
    };
    struct FbxSkeleton {
        std::vector<FbxJoint>   joints;
    };

    // skeleton animation
    struct FbxVec3Key {
        double      time;
        glm::vec3   value;
    };
    struct FbxQuatKey {
        double      time;
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

    // skeleton mesh
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
        std::vector<glm::uvec4> jointIds;
        std::vector<glm::vec4>  weights;
        std::vector<uint8_t>    weightCounts;

        // skeleton
        FbxSkeleton             skeleton;

        // skeleton animation
        FbxJointAnimation       animation;
        
        // morph
        // material and others ...

        // material
        int32_t                 materialIndex{0};
    };

    // node
    struct FbxNode {
        std::string name;

        // tree
        FbxNode* parent;
        std::vector<std::shared_ptr<FbxNode>> children;
        //uint32_t parentIndex;  // todo: rootNode tree or nodes array with parentIndex
        glm::mat4 transform; // transformation relative to parent

        // data
        std::vector<uint32_t> meshes; // meshIndex

        std::vector<FbxMetaData> metaData;
    };

    // material
    struct FbxMaterialProperty {
        std::string key;
        int semantic;       // texture usage semantic, non-texture 0
        int index;          // index of the texture, non-texture 0
        int dataLenght;     // data len
        int dataType;       // data type
        std::vector<char> data; // data buffer
    };
    struct FbxMaterial {
        std::string name;  // property of ?mat.name
        std::vector<std::shared_ptr<FbxMaterialProperty>> properties; // todo: array of member
    };

    struct FbxScene {
        std::string name;

        // node tree
        std::shared_ptr<FbxNode> rootNode; // todo: rootNode tree or nodes array with parentIndex

        // meshes
        std::vector<std::shared_ptr<FbxSkeletonMesh>> skmeshes;

        // materials and textures
        std::vector<std::shared_ptr<FbxMaterial>> materials;

        // skeleton and animation
        std::vector<std::shared_ptr<FbxSkeleton>> skeletons;
        std::vector<std::shared_ptr<FbxJointAnimation>> animations;
    };

    class FBXRWImpl;
    class FBXRW {
    public:

        FBXRW();
        ~FBXRW() = default;

        // read
        void readSkeketonMesh(std::string inPath, float scale = 1.0);        

        // write
        void setScene(std::shared_ptr<FbxScene>& fbxScene) { m_fbxScene = fbxScene; }
        void writeSkeletonMesh(std::string outPath, float scale = 1.0);

        // other
        bool hasAnimation();
        void printScene();
    private:
        std::string m_path;
        std::shared_ptr<FBXRWImpl> pimpl;
        std::shared_ptr<FbxScene> m_fbxScene;
    };
}

