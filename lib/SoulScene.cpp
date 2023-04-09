//
//  SoulScene.cpp
//
//
//  Created by kai chen on 3/24/23.
//

#include "SoulScene.hpp"

using namespace SoulIK;

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

