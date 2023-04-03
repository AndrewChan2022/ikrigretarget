//
//  SoulScene.h
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


// SoulScene only for general data represent, not for data process and render
// you should define your native scene data structure for your processor or renderer
namespace SoulIK {

    enum class SoulNodeType: uint8_t {
        None = 0,
        Mesh = 1,
        Locator = 2,
        JointRoot = 3,
        Joint = 4,
        Bone = 5,
        Effector = 6
    };

    // meta variant define, similar to std::variant
    enum class SoulMetadataType: uint8_t {
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
    struct SoulMetaData;
    struct SoulMetadataValue {  // cannot be union, not know how to destroy
        bool boolValue;
        int32_t int32Value;
        uint64_t uint64Value;
        float floatValue;
        double doubleValue;
        std::string stringValue;
        glm::vec3 vec3Value;
        std::vector<SoulMetaData> metadataValue;  // no need wrapper of pointer
        int64_t int64Value;
        uint32_t uint32Value;
    };
    struct SoulMetaData {
        std::string key;
        SoulMetadataType type;
        SoulMetadataValue value; // data or more metadata in nestled
    };

    // skeleton
    struct SoulJoint {
        std::string             name;
        uint32_t                parentId;

        // transforms from mesh space to bone space in bind pose
        glm::mat4               inverseWorldMatrix{ 1.0 };
    };
    struct SoulSkeleton {
        std::vector<SoulJoint>  joints;
    };

    // skeleton animation
    struct SoulTransform {
        glm::vec3 translation;
        glm::quat rotation;
        glm::vec3 scale;
    };
    struct SoulPose {
        std::vector<SoulTransform> transforms;
    };
    struct SoulVec3Key {
        double      time;       // frame
        glm::vec3   value;
    };
    struct SoulQuatKey {
        double      time;       // frame
        glm::quat   value;
    };
    struct SoulAniChannel {
        uint32_t                    jointId;
        std::vector<SoulVec3Key>    PositionKeys;
        std::vector<SoulVec3Key>    ScalingKeys;
        std::vector<SoulQuatKey>    RotationKeys;
    };
    struct SoulJointAnimation {
        std::string name;
        double duration;
        double ticksPerSecond;
        std::vector<SoulAniChannel> channels;
    };

    // skeleton mesh
    struct SoulSkeletonMesh {
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
        SoulSkeleton            skeleton;

        // skeleton animation
        SoulJointAnimation      animation;
        
        // morph
        // material and others ...

        // material
        int32_t                 materialIndex{0};
    };

    // node
    struct SoulNode {
        std::string name;

        // tree
        SoulNode* parent;
        std::vector<std::shared_ptr<SoulNode>> children;
        //uint32_t parentIndex;  // todo: rootNode tree or nodes array with parentIndex
        glm::mat4 transform; // transformation relative to parent

        // data
        std::vector<uint32_t> meshes; // meshIndex

        std::vector<SoulMetaData> metaData;

        // node type
        SoulNodeType nodeType{SoulNodeType::None};
    };

    // material
    struct SoulMaterialProperty {
        std::string key;
        int semantic;       // texture usage semantic, non-texture 0
        int index;          // index of the texture, non-texture 0
        int dataLenght;     // data len
        int dataType;       // data type
        std::vector<char> data; // data buffer
    };
    struct SoulMaterial {
        std::string name;  // property of ?mat.name
        std::vector<std::shared_ptr<SoulMaterialProperty>> properties; // todo: array of member
    };

    struct SoulScene {
        std::string name;

        // node tree
        std::shared_ptr<SoulNode> rootNode; // todo: rootNode tree or nodes array with parentIndex

        // meshes
        std::vector<std::shared_ptr<SoulSkeletonMesh>> skmeshes;

        // materials and textures
        std::vector<std::shared_ptr<SoulMaterial>> materials;

        // skeleton and animation
        std::vector<std::shared_ptr<SoulSkeleton>> skeletons;
        std::vector<std::shared_ptr<SoulJointAnimation>> animations;

        // meta
        std::vector<SoulMetaData> metaData;
    };
}

