//
//  SoulRetargetSkeleton.cpp
//  
//
//  Created by kai chen on 3/24/23.
//
#include "SoulTransform.h"

using namespace SoulIK;


const FVector FVector::ZeroVector = FVector(0, 0, 0);
const FVector FVector::OneVector = FVector(1, 1, 1);
// Unit X axis vector (1,0,0)
const FVector XAxisVector = FVector(1, 0, 0);
// Unit Y axis vector (0,1,0) 
const FVector YAxisVector = FVector(0, 1, 0);
// Unit Z axis vector (0,0,1)
const FVector ZAxisVector = FVector(0, 0, 1);

namespace SoulIK {
namespace FText {
    const char* FromName(std::string s) {
        return s.c_str();
    }
}

FVector operator*(double Scale, const FVector& V) 
{
    return V.operator*(Scale);
}
FVector operator-(double Scale, const FVector& V) 
{
    return FVector(Scale) - V;
}

}

const FRotator FRotator::ZeroRotator = FRotator(0, 0, 0);
const FQuat FQuat::Identity = FQuat(0, 0, 0, 1);

const FTransform FTransform::Identity = FTransform();

FRotator::FRotator(const FQuat& q) 
{
    glm::dquat q2(q.w, q.x, q.y, q.z);  // w,x,y,z
    glm::dvec3 euler = glm::eulerAngles(q2);
    x = euler.x;
    y = euler.y;
    z = euler.z;
}

FQuat FRotator::Quaternion() 
{
    glm::dquat q = glm::dquat(glm::dvec3(x, y, z));
    return FQuat(q.x, q.y, q.z, q.w);
}

