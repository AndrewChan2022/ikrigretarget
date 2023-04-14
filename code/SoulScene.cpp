//
//  SoulScene.cpp
//
//
//  Created by kai chen on 3/24/23.
//

#include "SoulScene.hpp"

using namespace SoulIK;

const SoulTransform SoulTransform::identity = SoulTransform();

SoulTransform::SoulTransform(glm::mat4 const& m) {
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(m, scale, rotation, translation, skew, perspective);
}

static void Multiply(SoulTransform* OutTransform, const SoulTransform* A, const SoulTransform* B) {
    // TODO: avoid decompose
    glm::mat4 ma = A->toMatrix();
    glm::mat4 mb = B->toMatrix();
    glm::mat4 mc = ma * mb;
    *OutTransform = SoulTransform(mc);
}

void SoulTransform::LeftDivide(SoulTransform* OutTransform, const SoulTransform* A, const SoulTransform* B) const {
    // TODO: avoid decompose
    glm::mat4 ma = glm::inverse(A->toMatrix());
    glm::mat4 mb = B->toMatrix();
    glm::mat4 mc = ma * mb;
    *OutTransform = SoulTransform(mc);
}

void SoulTransform::Divide(SoulTransform* OutTransform, const SoulTransform* A, const SoulTransform* B) const {
    // TODO: avoid decompose
    glm::mat4 ma = A->toMatrix();
    glm::mat4 mb = B->toMatrix();
    glm::mat4 mc = ma / mb;
    *OutTransform = SoulTransform(mc);
}

SoulTransform SoulTransform::operator*(const SoulTransform& Other) const {
    SoulTransform Output;
    Multiply(&Output, this, &Other);
    return Output;
}

SoulTransform SoulTransform::GetRelativeTransform(SoulTransform const& Other) const {
    // glm: gchild = gparent * lchild => lchild = gparent.inv * gchild 
    SoulTransform Result;
    LeftDivide(&Result, &Other, this);
    return Result;
}

SoulTransform SoulTransform::operator/(const SoulTransform& Other) const {
    SoulTransform Output;
    Divide(&Output, this, &Other);
    return Output;
}

SoulTransform SoulTransform::Inverse() {
    glm::mat4 m = glm::inverse(toMatrix());
    return SoulTransform(m);
}

glm::mat4 SoulTransform::toMatrix() const {
    glm::quat q(rotation.w, rotation.x, rotation.y, rotation.z);
    glm::vec3 t(translation.x, translation.y, translation.z);
    glm::vec3 s(scale.x, scale.y, scale.z);

    glm::mat4 identity(1.0);
    glm::mat4 mt = glm::translate(identity, t);
    glm::mat4 ms = glm::scale(identity, s);
    glm::mat4 mr = glm::mat4_cast(q);
    glm::mat4 OutMatrix = mt * mr * ms;
    return OutMatrix;
}


void SoulSkeletonMesh::calcNormal() {

    // reset to 0
    glm::vec3 zero(0.0);
    for(auto& n : normals) {
        n = zero;
    }

    // calc face normal and plus to its vertices
    size_t faceCount = indices.size() / 3;
    for (size_t i = 0; i < faceCount; i++) {

        uint32_t& i0 = indices[i*3]; 
        uint32_t& i1 = indices[i*3+1]; 
        uint32_t& i2 = indices[i*3+2];

        glm::vec3& v0 = vertices[i0]; 
        glm::vec3& v1 = vertices[i1]; 
        glm::vec3& v2 = vertices[i2];

        glm::vec3 e01 = v1 - v0;
        glm::vec3 e12 = v2 - v1;
        glm::vec3 n = glm::cross(e01, e12);
        n = glm::normalize(n);

        normals[i0] += n;
        normals[i1] += n;
        normals[i2] += n;
    }

    // normalize
    for(auto& n : normals) {
        n = glm::normalize(n);
    }
}

static std::shared_ptr<SoulNode> findNodeByName(std::string const& name, std::shared_ptr<SoulNode> const& node, int depth) {

    if (node->name == name) {
        return node;
    } 

    for(auto& child : node->children) {
        auto ret = findNodeByName(name, child, depth+1);
        if (ret != nullptr) {
            return ret;
        }
    }

    return nullptr;
}

std::shared_ptr<SoulNode> SoulScene::findNodeByName(std::string const& name) const {
    return ::findNodeByName(name, rootNode, 0);
}
