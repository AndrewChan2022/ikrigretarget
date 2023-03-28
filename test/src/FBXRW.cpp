//
//  FBXRW.cpp
//
//
//  Created by kai chen on 3/24/23.
//
#include "FBXRW.h"
#include <iostream>
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"
#include "assimp/Exporter.hpp"

using namespace SoulIK;
using namespace Assimp;
#define KINDA_SMALL_NUMBER  (1.e-4f)

class FBXRWImpl {
public:
    static void processNode(aiNode* node, const aiScene* scene,
            std::vector<std::string>& materialNames, 
            std::vector<aiNode*>& nodes, 
            std::vector<std::string>& nodeNames,
            std::vector<std::shared_ptr<FbxSkeletonMesh>>& skeletonMeshes);

    static std::shared_ptr<FbxSkeletonMesh> processMesh(aiMesh* mesh, const aiScene* scene, 
                                                    std::vector<std::string>& materialNames, 
                                                    std::vector<aiNode*>& nodes, 
                                                    std::vector<std::string>& nodeNames);

    static void processSkeleton(aiMesh *mesh, FbxSkeletonMesh &fbxMesh, 
                            std::vector<aiNode*>& nodes, 
                            std::vector<std::string>& nodeNames);

    static void sortJointsByNodeTree(std::vector<FbxJoint> &joints, 
                            std::vector<std::string> &jointNames, 
                            const std::vector<aiNode*>& nodes, 
                            std::vector<std::string> &nodeNames);
    
    static void processSkeletonAnimation(const aiScene* scene, aiMesh *mesh, 
                            FbxSkeletonMesh &fbxMesh,
                            std::vector<aiNode*>& nodes, 
                            std::vector<std::string>& nodeNames);

    static void buildSkeletonTree(std::vector<FbxJoint>& joints, 
                            std::vector<std::string>& jointNames,
                            const std::vector<aiNode*>& nodes, 
                            std::vector<std::string>& nodeNames);

    static void createMeshes(std::vector<std::shared_ptr<FbxSkeletonMesh>>& skeletonMeshes, aiScene* scene);
    static void createMesh(FbxSkeletonMesh& fbxMesh, aiMesh* mesh, aiScene* scene);
    static void createSkeleton(FbxSkeletonMesh &fbxMesh, aiMesh* mesh);
    static void createSkeletonAnimation(std::vector<std::shared_ptr<FbxSkeletonMesh>>& skeletonMeshes, aiScene* scene);
};

static void ____static____(){}

static void traversalAllNodes(aiNode* node, std::vector<aiNode*>& nodes, std::vector<std::string>& nodeNames){
    if (node) {
        nodes.emplace_back(node);
        nodeNames.emplace_back(node->mName.data);
    }

    for (int i = 0; i < node->mNumChildren; ++i) {
        traversalAllNodes(node->mChildren[i], nodes, nodeNames);
    }
}


static inline uint32_t getJointIdByName(std::string& name, const std::vector<std::string>& jointNames) {
    auto pos = std::find(jointNames.begin(), jointNames.end(), name);
    if (pos != jointNames.end()) {
        return pos - jointNames.begin();
    } else {
        return 0xffffffff;
    }
}
static inline uint32_t getJointIdFromSkeleton(std::string& name, FbxSkeleton& skeleton) {

    auto pos = std::find_if(skeleton.joints.begin(), skeleton.joints.end(), [&name](const auto& v) {
        return v.name == name;
    });

    if (pos != skeleton.joints.end()) {
        return pos - skeleton.joints.begin();
    } else {
        return 0xffffffff;
    }
}

// search parent.name or parent.parent.name
static uint32_t getJointIdByNode(aiNode* parentNode, const std::vector<std::string>& jointNames) {
    if (parentNode) {
        std::string name = parentNode->mName.data;
        auto parentPos = std::find(jointNames.begin(), jointNames.end(), name);

        if (parentPos != jointNames.end()) {
            return  parentPos - jointNames.begin();
        } else {
            return getJointIdByNode(parentNode->mParent, jointNames);
        }
    } else {
        return 0xffffffff;
    }
}

static aiNode* getNodeByName(std::string name, const std::vector<aiNode*>& nodes, std::vector<std::string> &nodeNames) {
    auto pos = std::find(nodeNames.begin(), nodeNames.end(), name);
    if (pos != nodeNames.end()) {
        return nodes[pos - nodeNames.begin()];
    } else {
        return nullptr;
    }
}

static void ____FBXRW____(){}

void FBXRW::readSkeketonMesh(float scale) {
    m_skeletonMeshes.clear();

    // read
    Importer importer;
    
    importer.SetPropertyFloat(AI_CONFIG_GLOBAL_SCALE_FACTOR_KEY, 100.0 * scale);  // 1.0f/16.f
    const aiScene* scene = importer.ReadFile(m_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_CalcTangentSpace | aiProcess_FlipUVs | aiProcess_GlobalScale);
    // scene is part of importer, so no need free

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        return;
    }

    // process materials
    std::vector<std::string> materialNames;
    for (int i = 0; i < scene->mNumMaterials; ++i) {
        auto material = scene->mMaterials[i];
        materialNames.emplace_back(material->GetName().data);
    }

    // record nodes
    std::vector<aiNode*> nodes;
    std::vector<std::string> nodeNames;
    traversalAllNodes(scene->mRootNode, nodes, nodeNames);
    
    // process mesh nodes
    FBXRWImpl::processNode(scene->mRootNode, scene, materialNames, nodes, nodeNames, m_skeletonMeshes);


    for (auto& name : nodeNames) {
        printf("name:%s\n", name.c_str());
    }
    printf("process done\n");
}

static void ____FBXRWImpl____(){}

void FBXRWImpl::processNode(aiNode *node, const aiScene* scene, 
                        std::vector<std::string>& materialNames, 
                        std::vector<aiNode*>& nodes, 
                        std::vector<std::string>& nodeNames, 
                        std::vector<std::shared_ptr<FbxSkeletonMesh>>& skeletonMeshes) {
    // mesh
    for(unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        skeletonMeshes.emplace_back(processMesh(mesh, scene, materialNames, nodes, nodeNames));
    }
    
    // child nodes
    for(unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene, materialNames, nodes, nodeNames, skeletonMeshes);
    }
}

std::shared_ptr<FbxSkeletonMesh> FBXRWImpl::processMesh(aiMesh* mesh, const aiScene* scene, 
                                                    std::vector<std::string>& materialNames, 
                                                    std::vector<aiNode*>& nodes, 
                                                    std::vector<std::string>& nodeNames) {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> tangents;
    std::vector<glm::vec2> uvs;

    std::vector<uint32_t> indices;
    //vector<Texture> textures;

    //vertex info
    int vertexCount = mesh->mNumVertices;
    vertices.resize(vertexCount);
    normals.resize(vertexCount);
    tangents.resize(vertexCount);
    uvs.resize(vertexCount, glm::vec2(0.0f, 0.0f));
    for(unsigned int i = 0; i < vertexCount; i++) {
        glm::vec3& vertex = vertices[i];
        glm::vec3& normal = normals[i];
        glm::vec3& tangent = tangents[i];
        glm::vec2& uv = uvs[i];

        // position, normal, tangent, uv 
        vertex.x = mesh->mVertices[i].x; vertex.y = mesh->mVertices[i].y; vertex.z = mesh->mVertices[i].z;
        if(mesh->HasNormals()){
            normal.x = mesh->mNormals[i].x; normal.y = mesh->mNormals[i].y; normal.z = mesh->mNormals[i].z;
        }
        if(mesh->HasTangentsAndBitangents()){
            tangent.x = mesh->mTangents[i].x;
            tangent.y = mesh->mTangents[i].y;
            tangent.z = mesh->mTangents[i].z;
        }
        if(mesh->mTextureCoords[0]) { // if exist uv
            uv.x = mesh->mTextureCoords[0][i].x; uv.y = mesh->mTextureCoords[0][i].y;
        }

        // vertices.emplace_back(std::move(vertex));
        // normals.emplace_back(std::move(normal));
        // tangents.emplace_back(std::move(tangent));
        // uvs.emplace_back(std::move(uv));
    }

    // index info
    int faceCount = mesh->mNumFaces;
    indices.resize(faceCount * 3);
    for(unsigned int i = 0; i < faceCount; i++) {
        aiFace& face = mesh->mFaces[i];
        int startIndex = i * 3;
        for(unsigned int j = 0; j < face.mNumIndices; j++) {
            indices[startIndex + j] = face.mIndices[j];
        }
    }
    // for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
    //     aiFace face = mesh->mFaces[i];
    //     for(unsigned int j = 0; j < face.mNumIndices; j++) {
    //         indices.emplace_back(face.mIndices[j]);
    //     }
    // }

    std::shared_ptr<FbxSkeletonMesh> pFbxMesh = std::make_shared<FbxSkeletonMesh>();
    FbxSkeletonMesh& fbxMesh = *pFbxMesh;
    fbxMesh.name = std::string(mesh->mName.data) + '_' + materialNames[mesh->mMaterialIndex];
    fbxMesh.vertices = std::move(vertices);
    fbxMesh.normals = std::move(normals);
    fbxMesh.tangents = std::move(tangents);
    fbxMesh.uvs = std::move(uvs);
    fbxMesh.indices = std::move(indices);
    
    //calculateNormalArray(fbxMesh.vertices,fbxMesh.indices,fbxMesh.normals);
    //calculateTangentArray(fbxMesh.vertices,fbxMesh.uvs,fbxMesh.normals,fbxMesh.indices,fbxMesh.tangents);

    // material
    /*if(mesh->mMaterialIndex >= 0)
    {
        aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
        vector<Texture> diffuseMaps = loadMaterialTextures(material,
                                            aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
        vector<Texture> specularMaps = loadMaterialTextures(material,
                                            aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    }*/
    //auto ans = Mesh(vertices, indices, textures);
    //

    //blendshape
    // auto aiAnimMeshNums = mesh->mNumAnimMeshes;
    // auto aiAnimMesh = mesh->mAnimMeshes;
    // for (int i = 0; i < aiAnimMeshNums; i++) {
    //     processBlendshape(aiAnimMesh[i], fbxMesh);
    // }

    // skeleton and skin
    processSkeleton(mesh, fbxMesh, nodes, nodeNames);

    // skeleton animation
    processSkeletonAnimation(scene, mesh, fbxMesh, nodes, nodeNames);

    return pFbxMesh;
}

void FBXRWImpl::processSkeleton(aiMesh *mesh, FbxSkeletonMesh &fbxMesh, 
                            std::vector<aiNode*>& nodes, 
                            std::vector<std::string>& nodeNames) {

    std::vector<std::string> jointNames;
    
    // build skeleton
    // skeleton name and matrix
    for (uint32_t i = 0; i < mesh->mNumBones; i++) {
        FbxJoint joint;

        joint.name = std::string(mesh->mBones[i]->mName.data);
        const auto& mat = mesh->mBones[i]->mOffsetMatrix;
        joint.offsetMatrix = glm::mat4(
            mat[0][0], mat[0][1], mat[0][2], mat[0][3],
            mat[1][0], mat[1][1], mat[1][2], mat[1][3],
            mat[2][0], mat[2][1], mat[2][2], mat[2][3],
            mat[3][0], mat[3][1], mat[3][2], mat[3][3]
        );
        joint.inverseWorldMatrix = glm::transpose(glm::mat4(
            mat[0][0], mat[0][1], mat[0][2], mat[0][3],
            mat[1][0], mat[1][1], mat[1][2], mat[1][3],
            mat[2][0], mat[2][1], mat[2][2], mat[2][3],
            mat[3][0], mat[3][1], mat[3][2], mat[3][3]
        ));
        jointNames.emplace_back(joint.name);
        fbxMesh.skeleton.joints.emplace_back(joint);
    }
    buildSkeletonTree(fbxMesh.skeleton.joints, jointNames, nodes, nodeNames);

    // skin
    // weight on joint to weight on vertex
    int vertexCount = fbxMesh.vertices.size();
    fbxMesh.weightCounts.resize(vertexCount);
    fbxMesh.weights.resize(vertexCount);
    fbxMesh.jointIds.resize(vertexCount);
    for (uint32_t i = 0; i < mesh->mNumBones; i++) {
        auto jointName = std::string(mesh->mBones[i]->mName.data);
        auto jointId = getJointIdByName(jointName, jointNames);

        for (uint32_t j = 0; j < mesh->mBones[i]->mNumWeights; j++) {
            uint32_t vertexID = mesh->mBones[i]->mWeights[j].mVertexId;
            float weight = mesh->mBones[i]->mWeights[j].mWeight;

            auto& weightCounts = fbxMesh.weightCounts[vertexID];
            auto& weights = fbxMesh.weights[vertexID];
            auto& jointIds = fbxMesh.jointIds[vertexID];
            if (weightCounts < 4) {
                jointIds[weightCounts] = jointId;
                weights[weightCounts] = weight;
                weightCounts++;
            } else {
                // find min weight of exist and replace it
                int min = 0;
                for (int k = 1; k < 4; ++k) {
                    if (weights[k] < weights[min]) {
                        min = k;
                    }
                }
                if (weights[min] < weight) {
                    jointIds[min] = jointId;
                    weights[min] = weight;
                }
            }
        }
    }
    
    // normalize skin
    for (auto& weights : fbxMesh.weights) {
        float sum = weights.x + weights.y + weights.z + weights.w;
        if (sum > KINDA_SMALL_NUMBER) {
            weights /= sum;
        }
    }
}


/// find parent by name relation:  parentName_childName
void FBXRWImpl::buildSkeletonTree(std::vector<FbxJoint>& joints, 
                              std::vector<std::string>& jointNames,
                              const std::vector<aiNode*>& nodes,
                              std::vector<std::string>& nodeNames) {
    // sort joints by parent
    sortJointsByNodeTree(joints, jointNames, nodes, nodeNames);

    // get parent
    for (auto& joint : joints){
        auto node = getNodeByName(joint.name, nodes, nodeNames);
        if (node != nullptr) {
            auto parentNode = node->mParent;
            auto parentId = getJointIdByNode(parentNode, jointNames);
            joint.parentId = parentId;
        }
    }
}

// use node parent relation to sort skeleton
void FBXRWImpl::sortJointsByNodeTree(std::vector<FbxJoint> &joints, 
                                 std::vector<std::string> &jointNames, 
                                 const std::vector<aiNode*>& nodes, 
                                 std::vector<std::string> &nodeNames) {
    auto count = joints.size();
    for (size_t i = 0; i < count; ++i) {

        // find parent
        aiNode* node = getNodeByName(joints[i].name, nodes, nodeNames);
        auto parentId = getJointIdByNode(node, jointNames);

        // insert before i
        auto j  = parentId;
        if ( j >= i + 1 && j < count ) {
            auto joint = joints[j];
            auto name = jointNames[j];

            // j swap to end and remove, then insert before i
            std::swap(*(joints.begin() + j), *(joints.end() - 1));
            std::swap(*(jointNames.begin() + j), *(jointNames.end() - 1));
            joints.pop_back();
            jointNames.pop_back();
            joints.insert(joints.begin() + i, joint);
            jointNames.insert(jointNames.begin() + i, name);

            // to previous node 
            --i;
        }
    }
}

// first animation as skeleton animation
void FBXRWImpl::processSkeletonAnimation(const aiScene* scene, 
                            aiMesh *mesh, FbxSkeletonMesh &fbxMesh, 
                            std::vector<aiNode*>& nodes, 
                            std::vector<std::string>& nodeNames) {

    auto& joints = fbxMesh.skeleton.joints;
    if (scene->mNumAnimations == 0 || joints.size() == 0) {
        return;
    }
    //m_hasAnimation = true;

    std::unordered_map<std::string, uint32_t> jointNameToId;
    for (int i = 0; i < fbxMesh.skeleton.joints.size(); i++) {
        const FbxJoint& joint = fbxMesh.skeleton.joints[i];
        jointNameToId[joint.name] = i;
    }

    aiAnimation* pAnimation = scene->mAnimations[0];
    fbxMesh.animation.name = pAnimation->mName.data;
    for (int k = 0; k < pAnimation->mNumChannels; k++) {
        
        const aiNodeAnim* pNodeAnim = pAnimation->mChannels[k];
        std::string channelName = pNodeAnim->mNodeName.data;

        if (auto it = jointNameToId.find(channelName); it != jointNameToId.end()) {
            auto jointId = (*it).second;
            fbxMesh.animation.channels.emplace_back();
            FbxAniChannel& fbxChannel = fbxMesh.animation.channels.back();

            fbxChannel.jointId = jointId;
            // scaling key
            fbxChannel.ScalingKeys.resize(pNodeAnim->mNumScalingKeys);
            for (int i = 0; i < pNodeAnim->mNumScalingKeys; i++) {
                const aiVector3D& tmp = pNodeAnim->mScalingKeys[i].mValue;
                fbxChannel.ScalingKeys[i].time = pNodeAnim->mScalingKeys[i].mTime;
                fbxChannel.ScalingKeys[i].value = glm::vec3{ tmp.x,tmp.y,tmp.z };
            }
            // postion key
            fbxChannel.PositionKeys.resize(pNodeAnim->mNumPositionKeys);
            for (int i = 0; i < pNodeAnim->mNumPositionKeys; i++) {
                const aiVector3D& tmp = pNodeAnim->mPositionKeys[i].mValue;
                fbxChannel.PositionKeys[i].time = pNodeAnim->mPositionKeys[i].mTime;
                fbxChannel.PositionKeys[i].value = glm::vec3{ tmp.x,tmp.y,tmp.z };
            }
            // rotation key
            fbxChannel.RotationKeys.resize(pNodeAnim->mNumRotationKeys);
            for (int i = 0; i < pNodeAnim->mNumRotationKeys; i++) {
                const aiQuaternion& tmp = pNodeAnim->mRotationKeys[i].mValue;
                fbxChannel.RotationKeys[i].time = pNodeAnim->mRotationKeys[i].mTime;
                fbxChannel.RotationKeys[i].value = glm::quat{ tmp.w,tmp.x,tmp.y,tmp.z };
            }
        } else {
            printf("cannot find joints:%s\n", channelName.c_str());
        }
    }
}

static void ____write____(){}

void FBXRW::writeSkeletonMesh(std::string outPath, float scale) {

    Exporter exporter;

    // build scene
    std::unique_ptr<aiScene> uscene = std::unique_ptr<aiScene>(new aiScene);
    aiScene* scene = uscene.get();

    // build node tree
    scene->mRootNode = new aiNode();  // transfer owner to scene
    scene->mRootNode->mName.Set("root");

    scene->mRootNode->mNumChildren = m_skeletonMeshes.size();
    scene->mRootNode->mChildren = new aiNode* [scene->mRootNode->mNumChildren]; // transfer owner to parent node
    for(int i = 0; i < m_skeletonMeshes.size(); i++) {
        auto& mesh = *m_skeletonMeshes[i];

        aiNode* child = new aiNode;
        scene->mRootNode->mChildren[i] = child;

        // name
        child->mName.Set(mesh.name);

        // tree
        child->mParent = scene->mRootNode;

        // mesh
        child->mNumMeshes = 1;
        child->mMeshes = new unsigned int[1];
        child->mMeshes[0] = i;

        // todo
        // transform
        //child.mTransformation = ;        
    }

    // build mesh
    FBXRWImpl::createMeshes(m_skeletonMeshes, scene);
    

    // export
    exporter.Export(scene, "fbx", outPath);

    printf("write mesh done\n");
}

void FBXRWImpl::createMeshes(std::vector<std::shared_ptr<FbxSkeletonMesh>>& skeletonMeshes, aiScene* scene) {
    // build meshes
    scene->mNumMeshes = skeletonMeshes.size();
    scene->mMeshes = new aiMesh* [scene->mNumMeshes];
    for(int i = 0; i < skeletonMeshes.size(); i++) {
        auto& fbxMesh = *skeletonMeshes[i];
        aiMesh* mesh = new aiMesh;
        scene->mMeshes[i] = mesh;

        // mesh
        createMesh(fbxMesh, mesh, scene);

        // skeleton
        createSkeleton(fbxMesh, mesh);
    }


    // skeleton animation
    createSkeletonAnimation(skeletonMeshes, scene);
}



void FBXRWImpl::createMesh(FbxSkeletonMesh& fbxMesh, aiMesh* mesh, aiScene* scene) {

    // global
    mesh->mName = fbxMesh.name;

    // geom
    mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    mesh->mNumVertices = fbxMesh.vertices.size();
    mesh->mVertices = new aiVector3D[mesh->mNumVertices];
    mesh->mNormals = new aiVector3D[mesh->mNumVertices];
    mesh->mTangents = new aiVector3D[mesh->mNumVertices];
    mesh->mTextureCoords[0] = new aiVector3D[mesh->mNumVertices];
    mesh->mNumUVComponents[0] = 2;
    for(int i = 0; i < mesh->mNumVertices; i++) {
        aiVector3D& vertex = mesh->mVertices[i];
        aiVector3D& normal = mesh->mNormals[i];
        aiVector3D& tangent = mesh->mTangents[i];
        aiVector3D& uv0 = mesh->mTextureCoords[0][i];

        glm::vec3& tempv = fbxMesh.vertices[i];
        glm::vec3& tempn = fbxMesh.normals[i];
        glm::vec3& tempt = fbxMesh.tangents[i];
        glm::vec2& tempuv = fbxMesh.uvs[i];

        vertex.x = tempv.x; vertex.y = tempv.y; vertex.z = tempv.z;
        normal.x = tempn.x; normal.y = tempn.y; normal.z = tempn.z;
        tangent.x = tempt.x; tangent.y = tempt.y; tangent.z = tempt.z;
        uv0.x = tempuv.x; uv0.y = tempuv.y;
    }

    int faceCount = fbxMesh.indices.size() / 3;
    mesh->mNumFaces = faceCount;
    mesh->mFaces = new aiFace[faceCount];
    for(int i = 0; i < faceCount; i++) {
        aiFace& face = mesh->mFaces[i];
        uint32_t startIndex = i * 3;
        face.mIndices = new unsigned int[3];
        face.mIndices[0] = fbxMesh.indices[startIndex];
        face.mIndices[1] = fbxMesh.indices[startIndex + 1];
        face.mIndices[2] = fbxMesh.indices[startIndex + 2];
    }
}

void FBXRWImpl::createSkeleton(FbxSkeletonMesh &fbxMesh, aiMesh* mesh) {

    FbxSkeleton& sk = fbxMesh.skeleton;
    
    // build skeleton
    mesh->mNumBones = sk.joints.size();
    mesh->mBones = new aiBone* [mesh->mNumBones];

    // build bone
    for(int i = 0; i < mesh->mNumBones; i++) {
        mesh->mBones[i] = new aiBone;
        
        aiBone* bone = mesh->mBones[i];
        FbxJoint& joint = sk.joints[i];

        bone->mName     = joint.name;
        
        // both column major
        aiMatrix4x4& mat = mesh->mBones[i]->mOffsetMatrix;
        auto& jm = joint.offsetMatrix;
        mat[0][0] = jm[0][0]; mat[0][1] = jm[0][1]; mat[0][2] = jm[0][2]; mat[0][3] = jm[0][3];
        mat[1][0] = jm[1][0]; mat[1][1] = jm[1][1]; mat[1][2] = jm[1][2]; mat[1][3] = jm[1][3];
        mat[2][0] = jm[2][0]; mat[2][1] = jm[2][1]; mat[2][2] = jm[2][2]; mat[2][3] = jm[2][3];
        mat[3][0] = jm[3][0]; mat[3][1] = jm[3][1]; mat[3][2] = jm[3][2]; mat[3][3] = jm[3][3];
    }

    // build skin
    std::vector<int> boneWeightCount(mesh->mNumBones, 0);
    std::vector<std::vector<std::tuple<uint32_t, float>>> boneWeights(mesh->mNumBones);
    for(int i = 0; i < mesh->mNumBones; i++) {
        boneWeights[i].resize(fbxMesh.vertices.size());
    }
    auto vertexCount = fbxMesh.vertices.size();
    for(int i = 0; i < vertexCount; i++) {
        int vid = i;
        uint8_t weightCounts = fbxMesh.weightCounts[i];
        glm::ivec4& jointIds = fbxMesh.jointIds[i];
        glm::vec4& weights = fbxMesh.weights[i];

        for(int j = 0; j < weightCounts; j++) {
            auto jointId = jointIds[j];
            auto weight = weights[j];
            if (jointId < mesh->mNumBones) {
                auto& curIndex = boneWeightCount[jointId];
                boneWeights[jointId][curIndex] = {vid, weight};
                curIndex++;
            }
        }
    }
    for(int i = 0; i < mesh->mNumBones; i++) {
        aiBone* bone = mesh->mBones[i];
        bone->mNumWeights = boneWeightCount[i];
        bone->mWeights = new aiVertexWeight[bone->mNumWeights];
        for(int j = 0; j < bone->mNumWeights; j++) {
            bone->mWeights[j].mVertexId = std::get<0>(boneWeights[i][j]);
            bone->mWeights[j].mWeight = std::get<1>(boneWeights[i][j]);
        }
    }
}

// save one animation
void FBXRWImpl::createSkeletonAnimation(std::vector<std::shared_ptr<FbxSkeletonMesh>>& skeletonMeshes, aiScene* scene) {
    
    // find animation count
    int aniCount = 0;
    for(auto& sk : skeletonMeshes) {
        if (sk->animation.channels.size() != 0) {
            aniCount++;
        }
    }
    if (aniCount == 0) {
        return;
    }

    // 
    scene->mNumAnimations = 1;
    scene->mAnimations = new aiAnimation* [1];
    auto& animation = *(scene->mAnimations[0]);
    for(auto& psk : skeletonMeshes) {
        auto& sk = *psk;
        if (sk.animation.channels.size() != 0) {
            
            auto& ani = sk.animation;
            auto& joints = sk.skeleton.joints;
            auto& channels = ani.channels;

            animation.mName = ani.name;
            animation.mNumChannels = channels.size();
            animation.mChannels = new aiNodeAnim* [animation.mNumChannels];

            for(int j = 0; j < animation.mNumChannels; j++) {
                animation.mChannels[j] = new aiNodeAnim;
                
                auto& nodeAnimation = animation.mChannels[j];
                auto& channel = channels[j];
                auto jointId = channel.jointId;

                nodeAnimation->mNodeName =  joints[jointId].name;

                nodeAnimation->mNumPositionKeys = channel.PositionKeys.size();
                nodeAnimation->mPositionKeys = new aiVectorKey[nodeAnimation->mNumPositionKeys];
                for (int k = 0; k < nodeAnimation->mNumPositionKeys; k++) {
                    auto& element = nodeAnimation->mPositionKeys[k];
                    auto& e = channel.PositionKeys[k];
                    element.mTime = e.time;
                    element.mValue = aiVector3D(e.value.x, e.value.y, e.value.z);
                }

                nodeAnimation->mNumRotationKeys = channel.RotationKeys.size();
                nodeAnimation->mRotationKeys = new aiQuatKey[nodeAnimation->mNumRotationKeys];
                for (int k = 0; k < nodeAnimation->mNumRotationKeys; k++) {
                    auto& element = nodeAnimation->mRotationKeys[k];
                    auto& e = channel.RotationKeys[k];
                    element.mTime = e.time;
                    element.mValue = aiQuaterniont(e.value.w, e.value.x, e.value.y, e.value.z);
                }

                nodeAnimation->mNumScalingKeys = channel.ScalingKeys.size();
                nodeAnimation->mScalingKeys = new aiVectorKey[nodeAnimation->mNumScalingKeys];
                for (int k = 0; k < nodeAnimation->mNumScalingKeys; k++) {
                    auto& element = nodeAnimation->mScalingKeys[k];
                    auto& e = channel.ScalingKeys[k];
                    element.mTime = e.time;
                    element.mValue = aiVector3D(e.value.x, e.value.y, e.value.z);
                }
            }
        }
    }
}


