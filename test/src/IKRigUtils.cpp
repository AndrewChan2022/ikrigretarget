//
//  IKRigUtils.cpp
//
//
//  Created by kai chen on 3/24/23.
//

#include "IKRigUtils.hpp"

using namespace SoulIK;

bool IKRigUtils::LocalPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose) {  
            
    if (srcCoord == tgtCoord) {
        return;
    }

    // generate transform
    FTransform t = getTransformFromCoord(srcCoord, tgtCoord);

    // transform root
    pose[0] = t * pose[0];
}

bool IKRigUtils::GlobalPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose) {

    if (srcCoord == tgtCoord) {
        return;
    }
    
    // generate transform
    FTransform t = getTransformFromCoord(srcCoord, tgtCoord);

    // transform all
    for(auto& posei : pose) {
        posei = t * posei;
    }
}

void IKRigUtils::FPoseToLocal(SoulSkeleton& sk, std::vector<FTransform>& globalpose, std::vector<FTransform>& localpose) {
    assert(sk.joints.size() == globalpose.size());
    localpose.resize(globalpose.size());
    
    localpose[0] = globalpose[0];
    for(int jointId = 1; jointId < globalpose.size(); jointId++) {
        int parentId = sk.joints[jointId].parentId;
        localpose[jointId] = globalpose[jointId].GetRelativeTransform(globalpose[parentId]);
    }
}

void IKRigUtils::FPoseToGlobal(SoulSkeleton& sk, std::vector<FTransform>& localpose, std::vector<FTransform>& globalpose) {
    assert(sk.joints.size() == localpose.size());
    globalpose.resize(localpose.size());
    
    globalpose[0] = localpose[0];
    for(int jointId = 1; jointId < localpose.size(); jointId++) {
        int parentId = sk.joints[jointId].parentId;
        globalpose[jointId] = localpose[jointId] * globalpose[parentId];
    }
}

FTransform IKRigUtils::getTransformFromCoord(CoordType srcCoord, CoordType tgtCoord) {
    FTransform t = FTransform::Identity;

    // same
    if (srcCoord == tgtCoord) {
        return t;
    }

    // diff
    if (srcCoord == CoordType::RightHandYupZfront && tgtCoord == CoordType::RightHandZupYfront) {
        // maya to 3dsmax  (x,y,z)->(-x,z,y)
        // x 90 then z 180
        double coshalf1 = std::cos(M_PI_4);
        double sinhalf1 = std::sin(M_PI_4);
        double coshalf2 = 0.0;
        double sinhalf2 = 1.0;
        FQuat q1 = FQuat(sinhalf1, 0.0, 0.0, coshalf1);
        FQuat q2 = FQuat(0.0, 0.0, sinhalf2, coshalf2);
        FQuat q = q2 * q1;
        q.Normalize();
        t = FTransform(q);
    } else if (srcCoord == CoordType::RightHandZupYfront && tgtCoord == CoordType::RightHandYupZfront) {
        // 3dsmax to maya   (x,y,z)->(-x,z,y)
        // z -180 then x -90

        double coshalf1 = 0.0;
        double sinhalf1 = 1.0;
        double coshalf2 = std::cos(-M_PI_4);
        double sinhalf2 = std::sin(-M_PI_4);
        FQuat q1 = FQuat(sinhalf1, 0.0, 0.0, coshalf1);
        FQuat q2 = FQuat(0.0, 0.0, sinhalf2, coshalf2);
        FQuat q = q2 * q1;
        q.Normalize();
        t = FTransform(q);
    } else {
        printf("not support\n");
        assert(false);
    }
    return t;
}


void IKRigUtils::SoulPose2FPose(SoulIK::SoulPose& soulpose, std::vector<FTransform>& pose) {
    pose.resize(soulpose.transforms.size());
    for(int i = 0; i < soulpose.transforms.size(); i++) {
        pose[i].Translation = FVector(soulpose.transforms[i].translation);
        pose[i].Rotation = FQuat(soulpose.transforms[i].rotation);
        pose[i].Scale3D = FVector(soulpose.transforms[i].scale);
    }
}

void IKRigUtils::FPose2SoulPose(std::vector<FTransform>& pose, SoulIK::SoulPose& soulpose) {
    soulpose.transforms.resize(pose.size());
    for(int i = 0; i < soulpose.transforms.size(); i++) {
        soulpose.transforms[i].translation = pose[i].Translation;
        soulpose.transforms[i].rotation = pose[i].Rotation;
        soulpose.transforms[i].scale = pose[i].Scale3D;
    }
}

