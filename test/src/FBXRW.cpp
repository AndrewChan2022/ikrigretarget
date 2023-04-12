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

typedef std::shared_ptr<SoulSkeletonMesh> SoulSkeletonMeshPtr;

namespace SoulIK {
class FBXRWImpl {
public:

    bool hasAnimation(){return m_hasAnimation; }

    void processNode(aiNode* node, const aiScene* scene, SoulScene& fbxScene,
        SoulNode* parentSoulNode,
        std::vector<std::string>& materialNames, 
        std::vector<aiNode*>& nodes, std::vector<std::string>& nodeNames, std::vector<std::shared_ptr<SoulSkeletonMesh>>& skeletonMeshes);

    void processMetaData(std::vector<SoulMetaData>& fbxMetadata, aiMetadata* aimetaData);
    
    SoulSkeletonMeshPtr processMesh(aiMesh* mesh, const aiScene* scene, std::vector<std::string>& materialNames, 
        std::vector<aiNode*>& nodes, std::vector<std::string>& nodeNames);
    
    void processSkeleton(aiMesh *mesh, SoulSkeletonMesh &fbxMesh, 
        std::vector<aiNode*>& nodes, std::vector<std::string>& nodeNames);

    void sortJointsByNodeTree(std::vector<SoulJoint> &joints, std::vector<std::string> &jointNames, 
        const std::vector<aiNode*>& nodes, std::vector<std::string> &nodeNames);
    
    void processSkeletonAnimation(const aiScene* scene, SoulSkeletonMesh &fbxMesh);

    void buildSkeletonTree(std::vector<SoulJoint>& joints, std::vector<std::string>& jointNames, 
        const std::vector<aiNode*>& nodes, std::vector<std::string>& nodeNames);

    void createMeshes(std::vector<std::shared_ptr<SoulSkeletonMesh>>& skeletonMeshes, aiScene* scene);
    void createDefaultMaterial(aiScene* scene);
    void createNodes(aiScene* scene, SoulScene& fbxScene);
    void createMesh(SoulSkeletonMesh& fbxMesh, aiMesh* mesh, aiScene* scene);
    void createNode(SoulNode* node, aiNode* parentNode, aiScene* scene, int32_t nodeIndex);
    void createSkeleton(SoulSkeletonMesh &fbxMesh, aiMesh* mesh);
    void createSkeletonAnimation(std::vector<std::shared_ptr<SoulSkeletonMesh>>& skeletonMeshes, aiScene* scene);

    bool generateMeshFromPureSkeleton(SoulScene& soulScene, std::string const& rootBoneName);
public:
    Importer importer;
    bool m_hasAnimation{false};
};
}

static void ____static____(){
}

static void traversalAllNodes(aiNode* node, std::vector<aiNode*>& nodes, std::vector<std::string>& nodeNames){
    if (node) {
        nodes.emplace_back(node);
        nodeNames.emplace_back(node->mName.data);
    }

    for (uint32_t i = 0; i < node->mNumChildren; ++i) {
        traversalAllNodes(node->mChildren[i], nodes, nodeNames);
    }
}


static inline int32_t getJointIdByName(std::string& name, const std::vector<std::string>& jointNames) {
    auto pos = std::find(jointNames.begin(), jointNames.end(), name);
    if (pos != jointNames.end()) {
        return static_cast<uint32_t>(pos - jointNames.begin());
    } else {
        return -1;
    }
}
static inline int32_t getJointIdFromSkeleton(std::string& name, SoulSkeleton& skeleton) {

    auto pos = std::find_if(skeleton.joints.begin(), skeleton.joints.end(), [&name](const auto& v) {
        return v.name == name;
    });

    if (pos != skeleton.joints.end()) {
        return static_cast<uint32_t>(pos - skeleton.joints.begin());
    } else {
        return -1;
    }
}

// search parent.name or parent.parent.name
static int32_t getJointIdByNode(aiNode* parentNode, const std::vector<std::string>& jointNames) {
    if (parentNode) {
        std::string name = parentNode->mName.data;
        auto parentPos = std::find(jointNames.begin(), jointNames.end(), name);

        if (parentPos != jointNames.end()) {
            return  static_cast<uint32_t>(parentPos - jointNames.begin());
        } else {
            return getJointIdByNode(parentNode->mParent, jointNames);
        }
    } else {
        return -1;
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

static void ____FBXRW____(){
}

FBXRW::FBXRW() {
    //pimpl = std::make_shared<FBXRWImpl>();
};

bool FBXRW::hasAnimation() {
    return pimpl->hasAnimation();
}

void FBXRW::printScene() {
}

static void ____read____(){}

void FBXRW::readPureSkeletonWithDefualtMesh(std::string inPath, std::string const& rootBoneName, float scale) {
    readSkeletonMesh(inPath, scale);

    bool ret = pimpl->generateMeshFromPureSkeleton(*m_soulScene, rootBoneName);
    if (!ret) {
        printf("error: cannot find skeleton\n");
    }
}

void FBXRW::readSkeletonMesh(std::string inPath, float scale) {
    m_path = inPath;
    m_soulScene = std::make_shared<SoulScene>();

    printf("read model:%s\n", inPath.c_str());

    // read
    pimpl = std::make_shared<FBXRWImpl>();
    Importer& importer = pimpl->importer;
    
    // import as centi-meter by set GlobalScale = 100, which is default
    // maya export setting:  centi-meter
    importer.SetPropertyFloat(AI_CONFIG_GLOBAL_SCALE_FACTOR_KEY, 100.0f * scale);
    // prevent 1 joint split to 3 joints prerotation/rotation/joint
    importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
    
    //importer.SetPropertyBool(AI_CONFIG_IMPORT_REMOVE_EMPTY_BONES, false);
    //importer.SetPropertyBool(AI_CONFIG_IMPORT_NO_SKELETON_MESHES, true);
    //importer.SetPropertyBool(AI_CONFIG_FBX_USE_SKELETON_BONE_CONTAINER, true);
    
    const aiScene* scene = importer.ReadFile(m_path, aiProcess_Triangulate 
                                                   | aiProcess_JoinIdenticalVertices 
                                                   | aiProcess_CalcTangentSpace 
                                                   | aiProcess_FlipUVs
                                                   | aiProcess_GlobalScale
                                                   /*| aiProcess_PopulateArmatureData*/);
    // scene is part of importer, so no need free

    if (!scene || !scene->mRootNode) {
        std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        return;
    }
    if (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) {
        std::cout << "scene incomplete" << std::endl;
    }

    // process materials
    std::vector<std::string> materialNames;
    for (uint32_t i = 0; i < scene->mNumMaterials; ++i) {
        auto material = scene->mMaterials[i];
        /*printf("\nmat:%d name:%s\n", i, material->GetName().data);
        for (uint32_t j = 0; j < material->mNumProperties; j++) {
            aiMaterialProperty& p = *material->mProperties[j];
            printf("key:%s semantic:%d index:%d len:%d type:%d data:\n", p.mKey.data, p.mSemantic, p.mIndex, p.mDataLength, p.mType);
            aiPropertyTypeInfo type = p.mType;
            switch(type) {
                // 1
                case aiPTI_Float: {
                    float* data = (float*)p.mData;
                    printf("float:");
                    int count = p.mDataLength / sizeof(float);
                    for (int i = 0; i < count; i++) {
                        printf("%f ", data[i]);
                    }
                    printf("\n\n");
                    break;
                }
                case aiPTI_Double: {
                    double* data = (double*)p.mData;
                    printf("double:");
                    int count = p.mDataLength / sizeof(double);
                    for (uint32_t i = 0; i < p.mDataLength; i++) {
                        printf("%f ", data[i]);
                    }
                    printf("\n\n");
                    break;
                }
                case aiPTI_String: {
                    aiString* ais = (aiString*)p.mData;
                    std::string s = ais->data;
                    printf("string:");
                    //char* data = (char*)p.mData;
                    //data += 4;  // len
                    printf("%s\n\n", s.c_str());
                    break;
                }
                case aiPTI_Integer: {
                    int32_t* data = (int32_t*)p.mData;
                    printf("int:");
                    int count = p.mDataLength / sizeof(int32_t);
                    for (uint32_t i = 0; i < p.mDataLength; i++) {
                        printf("%d ", data[i]);
                    }
                    printf("\n\n");
                    break;
                }
                // binary buffer
                case aiPTI_Buffer: {
                    char* data = (char*)p.mData;
                    printf("buffer:");
                    printf("\n\n");
                    break;
                }
            }
        }*/
        materialNames.emplace_back(material->GetName().data);
    }

    // record nodes
    std::vector<aiNode*> nodes;
    std::vector<std::string> nodeNames;
    traversalAllNodes(scene->mRootNode, nodes, nodeNames);

    // scene meta
    pimpl->processMetaData(m_soulScene->metaData, scene->mMetaData);
    
    // process mesh nodes
    pimpl->processNode(scene->mRootNode, scene, *m_soulScene, nullptr, materialNames, nodes, nodeNames, m_soulScene->skmeshes);


    // for (auto& name : nodeNames) {
    //     printf("name:%s\n", name.c_str());
    // }
    printf("process done\n");
}

void FBXRWImpl::processNode(aiNode *node, const aiScene* scene,  SoulScene& fbxScene,
                        SoulNode* parentSoulNode,
                        std::vector<std::string>& materialNames, 
                        std::vector<aiNode*>& nodes, 
                        std::vector<std::string>& nodeNames, 
                        std::vector<std::shared_ptr<SoulSkeletonMesh>>& skeletonMeshes) {

    auto curNode = std::make_shared<SoulNode>();
    curNode->name = node->mName.data;

    // node tree and transform
    curNode->parent = parentSoulNode;
    if (parentSoulNode == nullptr) {
        fbxScene.rootNode = curNode;
    } else {
        parentSoulNode->children.push_back(curNode);
    }
    const auto& mat = node->mTransformation;
    // assimp row major, glm column major
    curNode->transform = glm::transpose(glm::mat4(
        mat[0][0], mat[0][1], mat[0][2], mat[0][3],
        mat[1][0], mat[1][1], mat[1][2], mat[1][3],
        mat[2][0], mat[2][1], mat[2][2], mat[2][3],
        mat[3][0], mat[3][1], mat[3][2], mat[3][3]
    ));
    
    // metadata
    if (node->mMetaData != nullptr) {
        // printf("*** meta of %s\n", curNode->name.c_str());
        // if (curNode->name == "Rol01_Torso01HipCtrlJnt_M") {
        //     printf("*** meta of %s\n", curNode->name.c_str());
        // }
        // if (curNode->name == "Hip") {
        //     printf("Hip\n");
        // }
        processMetaData(curNode->metaData, node->mMetaData);
        //aiProcess_PopulateArmatureData
    }

    // mesh
    for(unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        skeletonMeshes.emplace_back(processMesh(mesh, scene, materialNames, nodes, nodeNames));
        curNode->meshes.push_back((uint32_t)skeletonMeshes.size() - 1);
    }
    
    // child nodes
    for(unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene, fbxScene, curNode.get(), materialNames, nodes, nodeNames, skeletonMeshes);
    }
}

static bool aimetadataToSoulMetada( aiMetadataType& aitype, void* aimetadata, SoulMetaData& fbxMetaDataItem) {
    switch(aitype) {
        case AI_BOOL: {
            fbxMetaDataItem.type = SoulMetadataType::BOOL;
            fbxMetaDataItem.value.boolValue = *static_cast<bool*>(aimetadata);
            break;
        }
        case AI_INT32: {
            fbxMetaDataItem.type = SoulMetadataType::INT32;
            fbxMetaDataItem.value.int32Value = *static_cast<int32_t*>(aimetadata);
            break;
        }
        case AI_UINT64: {
            fbxMetaDataItem.type = SoulMetadataType::UINT64;
            fbxMetaDataItem.value.uint64Value = *static_cast<uint64_t*>(aimetadata);
            break;
        }
        case AI_FLOAT: {
            fbxMetaDataItem.type = SoulMetadataType::FLOAT;
            fbxMetaDataItem.value.floatValue = *static_cast<float*>(aimetadata);
            break;
        }
        case AI_DOUBLE: {
            fbxMetaDataItem.type = SoulMetadataType::DOUBLE;
            fbxMetaDataItem.value.doubleValue = *static_cast<double*>(aimetadata);
            break;
        }
        case AI_AISTRING: {
            fbxMetaDataItem.type = SoulMetadataType::STRING;
            fbxMetaDataItem.value.stringValue = (*static_cast<aiString*>(aimetadata)).data;
            break;
        }
        case AI_AIVECTOR3D: {
            fbxMetaDataItem.type = SoulMetadataType::VEC3;
            aiVector3D& v = *static_cast<aiVector3D*>(aimetadata);
            fbxMetaDataItem.value.vec3Value = glm::vec3(v.x, v.y, v.z);
            break;
        }
        case AI_AIMETADATA: {
            // process by outer
            break;
        }
        case AI_INT64: {
            fbxMetaDataItem.type = SoulMetadataType::INT64;
            fbxMetaDataItem.value.int64Value = *static_cast<int64_t*>(aimetadata);
            break;
        }
        case AI_UINT32: {
            fbxMetaDataItem.type = SoulMetadataType::UINT32;
            fbxMetaDataItem.value.uint32Value = *static_cast<uint32_t*>(aimetadata);
            break;
        }
        case AI_META_MAX: {
            return false;
        }
    }
    return true;
}

void FBXRWImpl::processMetaData(std::vector<SoulMetaData>& fbxMetadata, aiMetadata* aimetaData) {

    if (aimetaData == nullptr) {
        return;
    }
    fbxMetadata.resize(aimetaData->mNumProperties);

    for(uint32_t i = 0; i < aimetaData->mNumProperties; i++) {
        aiString key = aimetaData->mKeys[i];
        aiMetadataType aitype = aimetaData->mValues[i].mType;
        void* aimetadataBuf = aimetaData->mValues[i].mData;
        auto& fbxMetaDataItem = fbxMetadata[i];

        fbxMetaDataItem.key = key.data;
        //printf("meta key:%s\n", fbxMetaDataItem.key.c_str());
        if (aitype == AI_AIMETADATA) {
            fbxMetaDataItem.type = SoulMetadataType::METADATA;
            aiMetadata* v = static_cast<aiMetadata*>(aimetadataBuf);
            processMetaData(fbxMetaDataItem.value.metadataValue, v);
        } else {
            aimetadataToSoulMetada(aitype, aimetadataBuf, fbxMetaDataItem);
        }
    }
}

std::shared_ptr<SoulSkeletonMesh> FBXRWImpl::processMesh(aiMesh* mesh, const aiScene* scene, 
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
    uint32_t vertexCount = mesh->mNumVertices;
    vertices.resize(vertexCount);
    normals.resize(vertexCount);
    tangents.resize(vertexCount);
    uvs.resize(vertexCount, glm::vec2(0.0f, 0.0f));
    for(uint32_t i = 0; i < vertexCount; i++) {
        glm::vec3& vertex = vertices[i];
        glm::vec3& normal = normals[i];
        glm::vec3& tangent = tangents[i];
        glm::vec2& uv = uvs[i];

        // position, normal, tangent, uv 
        vertex.x = mesh->mVertices[i].x; vertex.y = mesh->mVertices[i].y; vertex.z = mesh->mVertices[i].z;
        // if (i == 0) {
        //     printf("position: %s, v.y:%f\n", mesh->mName.data, vertex.y);
        // }
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
    uint32_t faceCount = mesh->mNumFaces;
    indices.resize(faceCount * 3);
    for(uint32_t i = 0; i < faceCount; i++) {
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

    std::shared_ptr<SoulSkeletonMesh> pSoulMesh = std::make_shared<SoulSkeletonMesh>();
    SoulSkeletonMesh& fbxMesh = *pSoulMesh;
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

    // material
    fbxMesh.materialIndex = 0;

    // skeleton and skin
    processSkeleton(mesh, fbxMesh, nodes, nodeNames);

    // skeleton animation
    processSkeletonAnimation(scene, fbxMesh);

    return pSoulMesh;
}

void FBXRWImpl::processSkeleton(aiMesh *mesh, SoulSkeletonMesh &fbxMesh, 
                            std::vector<aiNode*>& nodes, std::vector<std::string>& nodeNames) {

    std::vector<std::string> jointNames;
    
    // build skeleton
    // skeleton name and matrix
    for (uint32_t i = 0; i < mesh->mNumBones; i++) {
        SoulJoint joint;

        joint.name = std::string(mesh->mBones[i]->mName.data);
        const auto& mat = mesh->mBones[i]->mOffsetMatrix;
        // assimp row major, glm column major
        joint.inverseBindposeMatrix = glm::transpose(glm::mat4(
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
    uint32_t vertexCount = (uint32_t)fbxMesh.vertices.size();
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
void FBXRWImpl::buildSkeletonTree(std::vector<SoulJoint>& joints, 
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
void FBXRWImpl::sortJointsByNodeTree(std::vector<SoulJoint> &joints, 
                                 std::vector<std::string> &jointNames, 
                                 const std::vector<aiNode*>& nodes, 
                                 std::vector<std::string> &nodeNames) {
    auto count = joints.size();
    for (size_t i = 0; i < count; ++i) {

        // find parent
        aiNode* node = getNodeByName(joints[i].name, nodes, nodeNames);
        auto parentId = getJointIdByNode(node->mParent, jointNames);

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
void FBXRWImpl::processSkeletonAnimation(const aiScene* scene, SoulSkeletonMesh &fbxMesh) {

    auto& joints = fbxMesh.skeleton.joints;
    if (scene->mNumAnimations == 0 || joints.size() == 0) {
        return;
    }
    m_hasAnimation = true;

    std::unordered_map<std::string, uint32_t> jointNameToId;
    for (int i = 0; i < fbxMesh.skeleton.joints.size(); i++) {
        const SoulJoint& joint = fbxMesh.skeleton.joints[i];
        jointNameToId[joint.name] = i;
    }

    aiAnimation* pAnimation = scene->mAnimations[0];
    fbxMesh.animation.name = pAnimation->mName.data;
    fbxMesh.animation.duration = pAnimation->mDuration;
    fbxMesh.animation.ticksPerSecond = pAnimation->mTicksPerSecond;
    for (uint32_t k = 0; k < pAnimation->mNumChannels; k++) {
        
        const aiNodeAnim* pNodeAnim = pAnimation->mChannels[k];
        std::string channelName = pNodeAnim->mNodeName.data;

        if (auto it = jointNameToId.find(channelName); it != jointNameToId.end()) {
            auto jointId = (*it).second;
            fbxMesh.animation.channels.emplace_back();
            SoulAniChannel& fbxChannel = fbxMesh.animation.channels.back();

            fbxChannel.jointId = jointId;
            // scaling key
            fbxChannel.ScalingKeys.resize(pNodeAnim->mNumScalingKeys);
            for (uint32_t i = 0; i < pNodeAnim->mNumScalingKeys; i++) {
                const aiVector3D& tmp = pNodeAnim->mScalingKeys[i].mValue;
                fbxChannel.ScalingKeys[i].time = pNodeAnim->mScalingKeys[i].mTime;
                fbxChannel.ScalingKeys[i].value = glm::vec3{ tmp.x,tmp.y,tmp.z };
            }
            // postion key
            fbxChannel.PositionKeys.resize(pNodeAnim->mNumPositionKeys);
            for (uint32_t i = 0; i < pNodeAnim->mNumPositionKeys; i++) {
                const aiVector3D& tmp = pNodeAnim->mPositionKeys[i].mValue;
                fbxChannel.PositionKeys[i].time = pNodeAnim->mPositionKeys[i].mTime;
                fbxChannel.PositionKeys[i].value = glm::vec3{ tmp.x,tmp.y,tmp.z };
            }
            // rotation key
            fbxChannel.RotationKeys.resize(pNodeAnim->mNumRotationKeys);
            for (uint32_t i = 0; i < pNodeAnim->mNumRotationKeys; i++) {
                const aiQuaternion& tmp = pNodeAnim->mRotationKeys[i].mValue;
                fbxChannel.RotationKeys[i].time = pNodeAnim->mRotationKeys[i].mTime;
                fbxChannel.RotationKeys[i].value = glm::quat{ tmp.w,tmp.x,tmp.y,tmp.z };
            }
        } else {
            printf("cannot find joints:%s\n", channelName.c_str());
        }
    }
}

static void processJointNode(SoulSkeleton& sk, SoulNode* node, SoulTransform const& ParentGlobalTransform, int depth) {
    if (node != nullptr) {

        SoulJoint joint;

        // name
        joint.name = node->name;

        // ibm
        SoulTransform local = SoulTransform(node->transform);
        SoulTransform global =  ParentGlobalTransform * local;
        joint.inverseBindposeMatrix = glm::inverse(global.toMatrix());

        // parent
        joint.parentId = -1;
        if (node->parent != nullptr) {
            joint.parentId = sk.getJointIdByName(node->parent->name);
        }

        sk.joints.push_back(joint);

        // child
        for (auto& child:  node->children) {
            processJointNode(sk, child.get(), global, depth + 1);
        }
    }
}

bool FBXRWImpl::generateMeshFromPureSkeleton(SoulScene& soulScene, std::string const& rootBoneName) {

    const aiScene* aiscene = importer.GetScene();
    
    soulScene.skmeshes.insert(soulScene.skmeshes.begin(), std::make_shared<SoulSkeletonMesh>());
    SoulSkeletonMesh& mesh = *soulScene.skmeshes[0];

    // geom
    mesh.vertices.push_back(glm::vec3(-0.5, 0.0, 0.0));
    mesh.vertices.push_back(glm::vec3(0.5, 0.0, 0.0));
    mesh.vertices.push_back(glm::vec3(0.0, 0.5, 0.0));

    mesh.normals.push_back(glm::vec3(0.0, 0.0, 1.0));
    mesh.normals.push_back(glm::vec3(0.0, 0.0, 1.0));
    mesh.normals.push_back(glm::vec3(0.0, 0.0, 1.0));

    mesh.tangents.push_back(glm::vec3(-0.866, -0.5, 0.0));
    mesh.tangents.push_back(glm::vec3(0.866, 0.5, 0.0));
    mesh.tangents.push_back(glm::vec3(0.0, 1.0, 0.0));

    mesh.uvs.push_back(glm::vec2(0.0, 0.0));
    mesh.uvs.push_back(glm::vec2(1.0, 0.0));
    mesh.uvs.push_back(glm::vec2(0.5, 1.0));

    mesh.indices.push_back(0);
    mesh.indices.push_back(1);
    mesh.indices.push_back(2);

    mesh.jointIds.push_back(glm::uvec4(0, 1, 2, 3));
    mesh.weights.push_back(glm::vec4(0.25, 0.25, 0.25, 0.25));
    mesh.weightCounts.push_back(4);

    mesh.jointIds.push_back(glm::uvec4(0, 1, 2, 3));
    mesh.weights.push_back(glm::vec4(0.25, 0.25, 0.25, 0.25));
    mesh.weightCounts.push_back(4);

    mesh.jointIds.push_back(glm::uvec4(0, 1, 2, 3));
    mesh.weights.push_back(glm::vec4(0.25, 0.25, 0.25, 0.25));
    mesh.weightCounts.push_back(4);

    // skeleton
    mesh.skeleton;
    SoulNode* jointRoot = soulScene.findNodeByName(rootBoneName).get();
    processJointNode(mesh.skeleton, jointRoot, SoulTransform::identity, 0);

    // skeleton animation
    processSkeletonAnimation(aiscene, mesh);

    return true;
}

static void ____write____(){}

void FBXRW::writeSkeletonMesh(std::string outPath, float scale) {


    std::cout << "start write to file:" << outPath << std::endl; 

    Exporter exporter;
    // aiReturn ret2 = exporter.Export(pimpl->importer.GetScene(), "fbx", outPath);
    // if (ret2 != AI_SUCCESS) {
    //     std::cout << "ERROR::ASSIMP::" << exporter.GetErrorString() << std::endl;
    //     return;
    // } else {
    //     return;
    // }

    // build scene
    std::unique_ptr<aiScene> uscene = std::unique_ptr<aiScene>(new aiScene);
    aiScene* scene = uscene.get();
    scene->mFlags = 8;

    // build node tree
    // scene->mRootNode = new aiNode();  // transfer owner to scene
    // scene->mRootNode->mName.Set("RootNode");

    // scene->mRootNode->mNumChildren = (uint32_t)m_soulScene->skmeshes.size();
    // scene->mRootNode->mChildren = new aiNode* [scene->mRootNode->mNumChildren]; // transfer owner to parent node
    // for(int i = 0; i < m_soulScene->skmeshes.size(); i++) {
    //     auto& mesh = *m_soulScene->skmeshes[i];

    //     aiNode* child = new aiNode;
    //     scene->mRootNode->mChildren[i] = child;

    //     // name
    //     child->mName.Set(mesh.name);

    //     // tree
    //     child->mParent = scene->mRootNode;

    //     // mesh
    //     child->mNumMeshes = 1;
    //     child->mMeshes = new unsigned int[1];
    //     child->mMeshes[0] = i;

    //     // todo
    //     // transform
    //     //child.mTransformation = ;        
    // }

    // build material
    pimpl->createDefaultMaterial(scene);

    pimpl->createNodes(scene, *m_soulScene);

    // build mesh
    pimpl->createMeshes(m_soulScene->skmeshes, scene);
    
    // export as meter by set GlobalScale = 1/100, which is default
    aiReturn ret = exporter.Export(scene, "fbx", outPath);
    if (ret != AI_SUCCESS) {
        std::cout << "ERROR::ASSIMP::" << exporter.GetErrorString() << std::endl;
        return;
    }

    printf("write mesh done\n");
}

void FBXRWImpl::createDefaultMaterial(aiScene* scene) {
    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial* [1];
    scene->mMaterials[0] = new aiMaterial;
    aiMaterial& mat = *scene->mMaterials[0];
    mat.mNumProperties = 2;
    mat.mProperties = new aiMaterialProperty* [2];
    mat.mProperties[0] = new aiMaterialProperty;
    mat.mProperties[1] = new aiMaterialProperty;
    auto& pname = *mat.mProperties[0];
    auto& pdiffuse = *mat.mProperties[1];
    
    // mat name
    pname.mKey = aiString("?mat.name");
    pname.mSemantic = 0;
    pname.mSemantic = 0;
    pname.mIndex = 0;

    aiString matName("default");
    pname.mType = aiPTI_String;
    pname.mDataLength = sizeof(matName);
    pname.mData = new char[pname.mDataLength];
    aiString* pdata = (aiString*)pname.mData;
    *pdata = matName;

    // diffuse
    pname.mKey = aiString("$clr.diffuse");
    pname.mSemantic = 0;
    pname.mSemantic = 0;
    pname.mIndex = 0;

    //aiColor3D c(0.8, 0.8, 0.8);
    aiColor3D c(0.4f, 0.4f, 0.4f);
    pname.mType = aiPTI_Float;
    pname.mDataLength = sizeof(float) * 3;
    pname.mData = new char[pname.mDataLength];
    float* pdata2 = (float*)pname.mData;
    pdata2[0] = c.r;
    pdata2[1] = c.g;
    pdata2[2] = c.b;
}

void FBXRWImpl::createNodes(aiScene* scene, SoulScene& fbxScene) {
    createNode(fbxScene.rootNode.get(), nullptr, scene, 0);
}

void FBXRWImpl::createNode(SoulNode* node, aiNode* parentAINode, aiScene* scene, int32_t nodeIndex) {
    
    aiNode* curAINode = new aiNode;
    curAINode->mName = node->name;

    // tree
    curAINode->mParent = parentAINode;
    if(parentAINode == nullptr) {
        scene->mRootNode = curAINode;
    } else {
        parentAINode->mChildren[nodeIndex] = curAINode;
    }
    curAINode->mNumChildren = (uint32_t)node->children.size();
    curAINode->mChildren = new aiNode* [curAINode->mNumChildren];

    // transform
    aiMatrix4x4& mat = curAINode->mTransformation;
    auto& m = node->transform;
    // assimp row major, glm column major
    mat = aiMatrix4x4(m[0][0], m[1][0], m[2][0], m[3][0],
                      m[0][1], m[1][1], m[2][1], m[3][1],
                      m[0][2], m[1][2], m[2][2], m[3][2],
                      m[0][3], m[1][3], m[2][3], m[3][3]);
    
    // meshes
    curAINode->mNumMeshes = (uint32_t)node->meshes.size();
    curAINode->mMeshes = new unsigned int[curAINode->mNumMeshes];
    for(uint32_t i = 0; i < curAINode->mNumMeshes; i++) {
        curAINode->mMeshes[i] = node->meshes[i];
    }

    // recursive child node
    for(int i = 0; i < node->children.size(); i++) {
        createNode(node->children[i].get(), curAINode, scene, i);
    }
}

void FBXRWImpl::createMeshes(std::vector<std::shared_ptr<SoulSkeletonMesh>>& skeletonMeshes, aiScene* scene) {
    // build meshes
    scene->mNumMeshes = (uint32_t)skeletonMeshes.size();
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



void FBXRWImpl::createMesh(SoulSkeletonMesh& fbxMesh, aiMesh* mesh, aiScene* scene) {

    // global
    mesh->mName = fbxMesh.name;

    // geom
    mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    mesh->mNumVertices = (uint32_t)fbxMesh.vertices.size();
    mesh->mVertices = new aiVector3D[mesh->mNumVertices];
    mesh->mNormals = new aiVector3D[mesh->mNumVertices];
    mesh->mTangents = new aiVector3D[mesh->mNumVertices];
    mesh->mTextureCoords[0] = new aiVector3D[mesh->mNumVertices];
    mesh->mNumUVComponents[0] = 2;
    for(uint32_t i = 0; i < mesh->mNumVertices; i++) {
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

    uint32_t faceCount = (uint32_t)fbxMesh.indices.size() / 3;
    mesh->mNumFaces = faceCount;
    mesh->mFaces = new aiFace[faceCount];
    for(uint32_t i = 0; i < faceCount; i++) {
        aiFace& face = mesh->mFaces[i];
        uint32_t startIndex = i * 3;
        face.mNumIndices = 3;
        face.mIndices = new unsigned int[3];
        face.mIndices[0] = fbxMesh.indices[startIndex];
        face.mIndices[1] = fbxMesh.indices[startIndex + 1];
        face.mIndices[2] = fbxMesh.indices[startIndex + 2];
    }

    // material
    mesh->mMaterialIndex = fbxMesh.materialIndex;

}

// create skeleton of mesh will mark world node as joint
void FBXRWImpl::createSkeleton(SoulSkeletonMesh &fbxMesh, aiMesh* mesh) {

    SoulSkeleton& sk = fbxMesh.skeleton;
    
    // build skeleton
    mesh->mNumBones = (uint32_t)sk.joints.size();
    mesh->mBones = new aiBone* [mesh->mNumBones];

    // build bone
    for(uint32_t i = 0; i < mesh->mNumBones; i++) {
        mesh->mBones[i] = new aiBone;
        
        aiBone* bone = mesh->mBones[i];
        SoulJoint& joint = sk.joints[i];

        bone->mName     = joint.name;
        
        // both column major
        aiMatrix4x4& mat = mesh->mBones[i]->mOffsetMatrix;
        auto& jm = joint.inverseBindposeMatrix;
        // assimp row major, glm column major
        mat = aiMatrix4x4(
            jm[0][0], jm[1][0], jm[2][0], jm[3][0],
            jm[0][1], jm[1][1], jm[2][1], jm[3][1],
            jm[0][2], jm[1][2], jm[2][2], jm[3][2],
            jm[0][3], jm[1][3], jm[2][3], jm[3][3]
        );
    }

    // build skin: error
    std::vector<int> boneWeightCount(mesh->mNumBones, 0);
    std::vector<std::vector<std::tuple<uint32_t, float>>> boneWeights(mesh->mNumBones);
    for(uint32_t i = 0; i < mesh->mNumBones; i++) {
        boneWeights[i].resize(fbxMesh.vertices.size());
    }
    auto vertexCount = fbxMesh.vertices.size();
    for(int i = 0; i < vertexCount; i++) {
        int vid = i;
        uint8_t weightCounts = fbxMesh.weightCounts[i];
        glm::uvec4& jointIds = fbxMesh.jointIds[i];
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
    for(uint32_t i = 0; i < mesh->mNumBones; i++) {
        aiBone* bone = mesh->mBones[i];
        bone->mNumWeights = boneWeightCount[i];
        bone->mWeights = new aiVertexWeight[bone->mNumWeights];
        for(uint32_t j = 0; j < bone->mNumWeights; j++) {
            bone->mWeights[j].mVertexId = std::get<0>(boneWeights[i][j]);
            bone->mWeights[j].mWeight = std::get<1>(boneWeights[i][j]);
        }
    }
}

// save one animation
void FBXRWImpl::createSkeletonAnimation(std::vector<std::shared_ptr<SoulSkeletonMesh>>& skeletonMeshes, aiScene* scene) {
    
    // find animation count
    int index = -1;
    for(size_t i = 0; i < skeletonMeshes.size(); i++) {
        auto& sk = skeletonMeshes[i];
        if (sk->animation.channels.size() != 0) {
            index = static_cast<int>(i);
            break;
        }
    }
    if (index == -1) {
        return;
    }
    

    // write
    auto& sk = *skeletonMeshes[index];
    auto& ani = sk.animation;
    auto& joints = sk.skeleton.joints;
    auto& channels = ani.channels;

    scene->mNumAnimations = 1;
    scene->mAnimations = new aiAnimation* [1];
    scene->mAnimations[0] = new aiAnimation;
    aiAnimation& animation = *(scene->mAnimations[0]);

    assert(ani.name != "");  // warning: cannot save if no name
    animation.mName = ani.name;
    animation.mDuration = ani.duration;
    animation.mTicksPerSecond = ani.ticksPerSecond;
    animation.mNumChannels = (uint32_t)channels.size();
    animation.mChannels = new aiNodeAnim* [animation.mNumChannels];

    for(uint32_t j = 0; j < animation.mNumChannels; j++) {
        animation.mChannels[j] = new aiNodeAnim;
        
        auto& nodeAnimation = animation.mChannels[j];
        auto& channel = channels[j];
        auto jointId = channel.jointId;

        nodeAnimation->mNodeName =  joints[jointId].name;

        nodeAnimation->mNumPositionKeys = (uint32_t)channel.PositionKeys.size();
        nodeAnimation->mPositionKeys = new aiVectorKey[nodeAnimation->mNumPositionKeys];
        for (uint32_t k = 0; k < nodeAnimation->mNumPositionKeys; k++) {
            auto& element = nodeAnimation->mPositionKeys[k];
            auto& e = channel.PositionKeys[k];
            element.mTime = e.time / animation.mTicksPerSecond;
            element.mValue = aiVector3D(e.value.x, e.value.y, e.value.z);
        }

        nodeAnimation->mNumRotationKeys = (uint32_t)channel.RotationKeys.size();
        nodeAnimation->mRotationKeys = new aiQuatKey[nodeAnimation->mNumRotationKeys];
        for (uint32_t k = 0; k < nodeAnimation->mNumRotationKeys; k++) {
            auto& element = nodeAnimation->mRotationKeys[k];
            auto& e = channel.RotationKeys[k];
            element.mTime = e.time / animation.mTicksPerSecond;
            element.mValue = aiQuaterniont(e.value.w, e.value.x, e.value.y, e.value.z);
        }

        nodeAnimation->mNumScalingKeys = (uint32_t)channel.ScalingKeys.size();
        nodeAnimation->mScalingKeys = new aiVectorKey[nodeAnimation->mNumScalingKeys];
        for (uint32_t k = 0; k < nodeAnimation->mNumScalingKeys; k++) {
            auto& element = nodeAnimation->mScalingKeys[k];
            auto& e = channel.ScalingKeys[k];
            element.mTime = e.time / animation.mTicksPerSecond;
            element.mValue = aiVector3D(e.value.x, e.value.y, e.value.z);
        }
    }
            
    
}


