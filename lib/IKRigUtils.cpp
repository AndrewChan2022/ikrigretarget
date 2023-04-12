//
//  IKRigUtils.cpp
//
//
//  Created by kai chen on 3/24/23.
//

#define _USE_MATH_DEFINES
#include <cmath>
#include "IKRigUtils.hpp"

using namespace SoulIK;

std::shared_ptr<UIKRetargeter> IKRigUtils::createIKRigAsset(SoulIKRigRetargetConfig& config,
    SoulSkeleton& srcsk, SoulSkeleton& tgtsk, USkeleton& srcusk, USkeleton& tgtusk) {

    std::shared_ptr<UIKRetargeter> pInRetargeterAsset = std::make_shared<UIKRetargeter>();
    UIKRetargeter& InRetargeterAsset = *pInRetargeterAsset;

    ///////////////////////////////////////
    // ikrig 1
    InRetargeterAsset.SourceIKRigAsset = std::make_shared<UIKRigDefinition>();
    
    // skeleton
    FIKRigSkeleton rigsk;
    IKRigUtils::USkeleton2RigSkeleton(srcusk, rigsk);
    InRetargeterAsset.SourceIKRigAsset->Skeleton = rigsk;

    // rootbone
    InRetargeterAsset.SourceIKRigAsset->RetargetDefinition.RootBone = config.SourceRootBone;
    InRetargeterAsset.SourceIKRigAsset->RetargetDefinition.GroundBone = config.SourceGroundBone;
    InRetargeterAsset.SourceIKRigAsset->RetargetDefinition.RootType = config.SourceRootType;

    // chain
    InRetargeterAsset.SourceIKRigAsset->RetargetDefinition.BoneChains.resize(config.SourceChains.size());
    for(size_t i = 0; i < config.SourceChains.size(); i++) {
        auto& chain = config.SourceChains[i];
        auto& BoneChain = InRetargeterAsset.SourceIKRigAsset->RetargetDefinition.BoneChains[i];

        BoneChain.ChainName = chain.chainName;
        BoneChain.StartBone.BoneName = chain.startBone;
        BoneChain.StartBone.BoneIndex = srcsk.getJointIdByName(chain.startBone);
        BoneChain.EndBone.BoneName = chain.endBone;
        BoneChain.EndBone.BoneIndex = srcsk.getJointIdByName(chain.endBone);
    }

    ///////////////////////////////////////
    // ikrig 2
    InRetargeterAsset.TargetIKRigAsset = std::make_shared<UIKRigDefinition>();

    // skeleton
    FIKRigSkeleton rigsk2;
    IKRigUtils::USkeleton2RigSkeleton(tgtusk, rigsk2);
    InRetargeterAsset.TargetIKRigAsset->Skeleton = rigsk2;

    // root bone
    InRetargeterAsset.TargetIKRigAsset->RetargetDefinition.RootBone = config.TargetRootBone;
    InRetargeterAsset.TargetIKRigAsset->RetargetDefinition.GroundBone = config.TargetGroundBone;
    InRetargeterAsset.TargetIKRigAsset->RetargetDefinition.RootType = config.TargetRootType;

    // chain
    InRetargeterAsset.TargetIKRigAsset->RetargetDefinition.BoneChains.resize(config.TargetChains.size());
    for(size_t i = 0; i < config.TargetChains.size(); i++) {
        auto& chain = config.TargetChains[i];
        auto& BoneChain = InRetargeterAsset.TargetIKRigAsset->RetargetDefinition.BoneChains[i];

        BoneChain.ChainName = chain.chainName;
        BoneChain.StartBone.BoneName = chain.startBone;
        BoneChain.StartBone.BoneIndex = srcsk.getJointIdByName(chain.startBone);
        BoneChain.EndBone.BoneName = chain.endBone;
        BoneChain.EndBone.BoneIndex = srcsk.getJointIdByName(chain.endBone);
    }

    ///////////////////////////////////////
    // mapping
    for(size_t i = 0; i < config.ChainMapping.size(); i++) {
        std::shared_ptr<URetargetChainSettings> chainSetting = std::make_shared<URetargetChainSettings>();

        chainSetting->Settings.FK.EnableFK = config.ChainMapping[i].EnableFK;
        chainSetting->Settings.IK.EnableIK = config.ChainMapping[i].EnableIK;
        chainSetting->SourceChain = config.ChainMapping[i].SourceChain;
        chainSetting->TargetChain = config.ChainMapping[i].TargetChain;
        InRetargeterAsset.ChainSettings.push_back(chainSetting);
    }

    ///////////////////////////////////////
    // global settting
    InRetargeterAsset.GlobalSettings = std::make_shared<UIKRetargetGlobalSettings>();
    InRetargeterAsset.GlobalSettings->Settings.bEnableRoot = (config.TargetRootType != ERootType::Ignore);

    return pInRetargeterAsset;
}

std::string IKRigUtils::CoordTypeToString(CoordType aCoordType) {
    if(aCoordType == CoordType::RightHandZupYfront) {
        return "RightHandZupYfront";
    } else if (aCoordType == CoordType::RightHandYupZfront) {
        return "RightHandYupZfront";
    }
    return "unkown CoordType";
}

FTransform IKRigUtils::getFTransformFromCoord(CoordType srcCoord, CoordType tgtCoord) {
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

        // double coshalf1 = 0.0;
        // double sinhalf1 = 1.0;
        // double coshalf2 = std::cos(-M_PI_4);
        // double sinhalf2 = std::sin(-M_PI_4);
        // FQuat q1 = FQuat(sinhalf1, 0.0, 0.0, coshalf1);
        // FQuat q2 = FQuat(0.0, 0.0, sinhalf2, coshalf2);
        // FQuat q = q2 * q1;
        // q.Normalize();
        // t = FTransform(q);

        double coshalf1 = std::cos(M_PI_4);
        double sinhalf1 = std::sin(M_PI_4);
        double coshalf2 = 0.0;
        double sinhalf2 = 1.0;
        FQuat q1 = FQuat(sinhalf1, 0.0, 0.0, coshalf1);
        FQuat q2 = FQuat(0.0, 0.0, sinhalf2, coshalf2);
        FQuat q = q2 * q1;
        q = q.Inverse();
        q.Normalize();
        t = FTransform(q);
    } else {
        printf("not support\n");
        assert(false);
    }
    return t;
}

bool IKRigUtils::LocalFPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose) {  
            
    if (srcCoord == tgtCoord) {
        return true;
    }

    // generate transform
    FTransform t = getFTransformFromCoord(srcCoord, tgtCoord);

    // transform root
    pose[0] = pose[0] * t;
    return true;
}

bool IKRigUtils::GlobalFPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose) {

    if (srcCoord == tgtCoord) {
        return true;
    }
    
    // generate transform
    FTransform t = getFTransformFromCoord(srcCoord, tgtCoord);

    // transform all
    for(auto& posei : pose) {
        posei = posei * t;
    }

    return true;
}

void IKRigUtils::USkeletonCoordConvert(CoordType srcCoord, CoordType tgtCoord, USkeleton& sk) {
    if (srcCoord == tgtCoord) {
        return;
    }

    // generate transform
    FTransform t = getFTransformFromCoord(srcCoord, tgtCoord);

    // transform
    sk.refpose[0] = sk.refpose[0] * t;
}

bool IKRigUtils::LocalFPoseCoordConvert(FTransform& t, CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose) {
    if (srcCoord == tgtCoord) {
        return true;
    }

    pose[0] = pose[0] * t;
    return true;
}
bool IKRigUtils::GlobalFPoseCoordConvert(FTransform& t, CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose) {
    if (srcCoord == tgtCoord) {
        return true;
    }

    for(auto& posei : pose) {
        posei = posei * t;
    }
    return true;
}


SoulTransform IKRigUtils::getSoulTransformFromCoord(CoordType srcCoord, CoordType tgtCoord) {
    SoulTransform t = SoulTransform();

    // same
    if (srcCoord == tgtCoord) {
        return t;
    }

    // diff
    if (srcCoord == CoordType::RightHandYupZfront && tgtCoord == CoordType::RightHandZupYfront) {
        // maya to 3dsmax  (x,y,z)->(-x,z,y)
        // x 90 then z 180
        float pi = glm::pi<float>();
        float pi_2 = glm::pi<float>() / 2.0f;
        glm::quat q1 = glm::angleAxis(pi_2, glm::vec3(1.0, 0.0, 0.0));
        glm::quat q2 = glm::angleAxis(pi, glm::vec3(0.0, 0.0, 1.0));
        glm::quat q = q2 * q1;
        q = glm::normalize(q);
        t = SoulTransform(q);

        // double coshalf1 = std::cos(M_PI_4);
        // double sinhalf1 = std::sin(M_PI_4);
        // double coshalf2 = 0.0;
        // double sinhalf2 = 1.0;
        // FQuat q1 = FQuat(sinhalf1, 0.0, 0.0, coshalf1);
        // FQuat q2 = FQuat(0.0, 0.0, sinhalf2, coshalf2);
        // FQuat q = q2 * q1;
        // q.Normalize();
        // t = FTransform(q);
    } else if (srcCoord == CoordType::RightHandZupYfront && tgtCoord == CoordType::RightHandYupZfront) {
        // 3dsmax to maya   (x,y,z)->(-x,z,y)
        // z -180 then x -90

        // double coshalf1 = 0.0;
        // double sinhalf1 = 1.0;
        // double coshalf2 = std::cos(-M_PI_4);
        // double sinhalf2 = std::sin(-M_PI_4);
        // FQuat q1 = FQuat(sinhalf1, 0.0, 0.0, coshalf1);
        // FQuat q2 = FQuat(0.0, 0.0, sinhalf2, coshalf2);
        // FQuat q = q2 * q1;
        // q.Normalize();
        // t = FTransform(q);

        // double coshalf1 = std::cos(M_PI_4);
        // double sinhalf1 = std::sin(M_PI_4);
        // double coshalf2 = 0.0;
        // double sinhalf2 = 1.0;
        // FQuat q1 = FQuat(sinhalf1, 0.0, 0.0, coshalf1);
        // FQuat q2 = FQuat(0.0, 0.0, sinhalf2, coshalf2);
        // FQuat q = q2 * q1;
        // q = q.Inverse();
        // q.Normalize();
        // t = FTransform(q);

        float pi = glm::pi<float>();
        float pi_2 = glm::pi<float>() / 2.0f;
        glm::quat q1 = glm::angleAxis(pi_2, glm::vec3(1.0, 0.0, 0.0));
        glm::quat q2 = glm::angleAxis(pi, glm::vec3(0.0, 0.0, 1.0));
        glm::quat q = q2 * q1;
        q = glm::inverse(q);
        q = glm::normalize(q);
        t = SoulTransform(q);
    } else {
        printf("not support\n");
        assert(false);
    }
    return t;
}

bool IKRigUtils::LocalSoulPoseCoordConvert(SoulTransform& t, CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose) {
    if (srcCoord == tgtCoord) {
        return true;
    }

    pose[0] = t * pose[0];
    return true;
}

bool IKRigUtils::GlobalSoulPoseCoordConvert(SoulTransform& t, CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose) {
    if (srcCoord == tgtCoord) {
        return true;
    }

    for(auto& posei : pose) {
        posei = t * posei;
    }
    return true;
}

bool IKRigUtils::LocalSoulPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose) {
    if (srcCoord == tgtCoord) {
        return true;
    }
    
    // generate transform
    SoulTransform t = getSoulTransformFromCoord(srcCoord, tgtCoord);

    // transform all
    for(auto& posei : pose) {
        posei = t * posei;
    }

    return true;
}

bool IKRigUtils::GlobalSoulPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose) {
    if (srcCoord == tgtCoord) {
        return true;
    }
    
    // generate transform
    SoulTransform t = getSoulTransformFromCoord(srcCoord, tgtCoord);

    // transform all
    for(auto& posei : pose) {
        posei = t * posei;
    }

    return true;
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

void IKRigUtils::SoulPoseToLocal(SoulSkeleton& sk, std::vector<SoulTransform>& globalpose, std::vector<SoulTransform>& localpose) {
    assert(sk.joints.size() == globalpose.size());
    localpose.resize(globalpose.size());
    
    localpose[0] = globalpose[0];
    for(int jointId = 1; jointId < globalpose.size(); jointId++) {
        int parentId = sk.joints[jointId].parentId;
        // gchild = gparent * lchild => lchild = gparent.inv * gchild
        localpose[jointId] =  globalpose[jointId].GetRelativeTransform(globalpose[parentId]);
        //SoulTransform(glm::inverse(globalpose[parentId].toMatrix()) * globalpose[jointId].toMatrix());
    }
}

void IKRigUtils::SoulPoseToGlobal(SoulSkeleton& sk, std::vector<SoulTransform>& localpose, std::vector<SoulTransform>& globalpose) {
    assert(sk.joints.size() == localpose.size());
    assert(sk.joints.size() != 0);
    globalpose.resize(localpose.size());
    
    globalpose[0] = localpose[0];
    for(int jointId = 1; jointId < localpose.size(); jointId++) {
        int parentId = sk.joints[jointId].parentId;
        globalpose[jointId] = globalpose[parentId] * localpose[jointId];
    }
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

SoulTransform IKRigUtils::glmToSoulTransform(glm::mat4& m) {
    return SoulTransform(m);
}

FTransform IKRigUtils::glmToFTransform(glm::mat4& m) {
    return FTransform(m);
}

bool IKRigUtils::alignUSKWithSkeleton(USkeleton& usk, SoulSkeleton const& sk) {

    assert(usk.boneTree.size() == sk.joints.size());

    // sort bone tree and refpose
    for (size_t i = 0; i < sk.joints.size(); i++) {
        auto& name = sk.joints[i].name;
        // if name not same, find and swap with it
        if (usk.boneTree[i].name != name) {
            for (size_t j = i + 1; j < sk.joints.size(); j++) {
                if (usk.boneTree[j].name == name) {
                    std::swap(*(usk.boneTree.begin() + i), *(usk.boneTree.begin() + j));
                    std::swap(*(usk.refpose.begin() + i), *(usk.refpose.begin() + j));
                    break;
                }
            }
        }
    }

    // build parentId
    for (size_t i = 0; i < sk.joints.size(); i++) {
        usk.boneTree[i].parent = sk.joints[i].parentId;
    }

    return true;
}

SoulNode* IKRigUtils::findNodeByName(SoulNode* rootNode, std::string& name) {
    if (rootNode->name == name) {
        return rootNode;
    }

    for(auto& child : rootNode->children) {
        auto ret = findNodeByName(child.get(), name);
        if (ret != nullptr) {
            return ret;
        }
    }
    return nullptr;
}

bool IKRigUtils::getUSkeletonFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, USkeleton& usk, CoordType srcCoord, CoordType tgtCoord) {

    SoulSkeleton& sk = skmesh.skeleton;
    std::vector<SoulJoint>&  joints = sk.joints;

    assert(joints.size() != 0);
    
    // name
    usk.name = scene.skmeshes[0]->name;

    // bone tree
    usk.boneTree.clear();
    for(uint64_t i = 0; i < joints.size(); i++) {
        SoulJoint& joint = joints[i];
        FBoneNode node;
        node.name = joint.name;
        node.parent = joint.parentId;
        usk.boneTree.push_back(node);
    }

    // refpose is local
    usk.refpose.clear();
    for(uint64_t i = 0; i < joints.size(); i++) {
        std::string name = joints[i].name;

        // find node
        auto node = IKRigUtils::findNodeByName(scene.rootNode.get(), name);
        glm::mat4 m = node->transform;
        FTransform t;

        glm::vec3 scale, translation, skew;
        glm::vec4 perspective;
        glm::quat q;
        glm::decompose(m, scale, q, translation, skew, perspective);
        t.Translation = FVector(translation);
        t.Scale3D = FVector(scale);
        t.Rotation = FQuat(q);
        usk.refpose.push_back(t);
    }

    if (srcCoord != tgtCoord && usk.refpose.size() != 0) {
        FTransform t = getFTransformFromCoord(srcCoord, tgtCoord);
        usk.refpose[0] = usk.refpose[0] * t;
    } else if (srcCoord == tgtCoord) {
        // do nothing
    } else {
        printf("no skeleton\n");
    }

    return true;
}

bool IKRigUtils::USkeleton2RigSkeleton(USkeleton& sk, FIKRigSkeleton& rigsk) {
    rigsk.BoneNames.clear();
    rigsk.ParentIndices.clear();
    rigsk.CurrentPoseLocal.clear();
    rigsk.CurrentPoseLocal.clear();
    for (size_t i = 0; i < sk.boneTree.size(); i++) {
        rigsk.BoneNames.push_back(sk.boneTree[i].name);
        rigsk.ParentIndices.push_back(sk.boneTree[i].parent);
    }
    rigsk.CurrentPoseGlobal = sk.refpose;
    rigsk.RefPoseGlobal = sk.refpose;
    for(int i = 0; i < sk.refpose.size(); i++) {
        if (i == 0) {
            rigsk.CurrentPoseLocal.push_back(rigsk.CurrentPoseGlobal[i]);
        } else {
            rigsk.CurrentPoseLocal.push_back(rigsk.CurrentPoseGlobal[i].GetRelativeTransform(rigsk.CurrentPoseGlobal[i]));
        }
    }
    return true;
}

bool IKRigUtils::getSoulPoseFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, std::vector<SoulTransform>& pose) {

    SoulSkeleton& sk = skmesh.skeleton;
    std::vector<SoulJoint>&  joints = sk.joints;
    
    // refpose
    pose.clear();
    for(uint64_t i = 0; i < joints.size(); i++) {
        std::string name = joints[i].name;

        // find node
        auto node = IKRigUtils::findNodeByName(scene.rootNode.get(), name);
        glm::mat4 m = node->transform;
        SoulTransform t;

        glm::vec3 scale, translation, skew;
        glm::vec4 perspective;
        glm::quat q;
        glm::decompose(m, scale, q, translation, skew, perspective);
        t.translation = translation;
        t.scale = scale;
        t.rotation = q;
        pose.push_back(t);
    }

    return true;
}

bool IKRigUtils::getSoulPoseFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, SoulPose& outpose) {

    SoulSkeleton& sk = skmesh.skeleton;
    std::vector<SoulJoint>&  joints = sk.joints;
    
    // refpose is local
    outpose.transforms.clear();
    for(uint64_t i = 0; i < joints.size(); i++) {
        std::string name = joints[i].name;

        // find node
        auto node = IKRigUtils::findNodeByName(scene.rootNode.get(), name);
        glm::mat4 m = node->transform;
        glm::vec3 scale, translation, skew;
        glm::vec4 perspective;
        glm::quat q;
        glm::decompose(m, scale, q, translation, skew, perspective);

        SoulTransform t;
        t.translation = translation;
        t.scale = scale;
        t.rotation = q;
        outpose.transforms.push_back(t);
    }

    return true;
}

void IKRigUtils::debugPrintFPose(SoulSkeleton& sk, std::vector<FTransform>& pose) {
    for (size_t i = 0; i < pose.size(); i++) {
        printf("%zu %s t(%.2f, %.2f, %.2f) s(%.2f, %.2f, %.2f) r.wxyz(%.2f, %.2f, %.2f, %.2f)\n",
            i, 
            sk.joints[i].name.c_str(), 
            pose[i].Translation.x, pose[i].Translation.y, pose[i].Translation.z,
            pose[i].Scale3D.x, pose[i].Scale3D.y, pose[i].Scale3D.z,
            pose[i].Rotation.w, pose[i].Rotation.x, pose[i].Rotation.y, pose[i].Rotation.z
        );
    }
}

void IKRigUtils::debugPrintSoulPose(SoulSkeleton& sk, std::vector<SoulTransform>& pose) {
    for (size_t i = 0; i < pose.size(); i++) {
        printf("%zu %s t(%.2f, %.2f, %.2f) s(%.2f, %.2f, %.2f) r.wxyz(%.2f, %.2f, %.2f, %.2f)\n",
            i, 
            sk.joints[i].name.c_str(), 
            pose[i].translation.x, pose[i].translation.y, pose[i].translation.z,
            pose[i].scale.x, pose[i].scale.y, pose[i].scale.z,
            pose[i].rotation.w, pose[i].rotation.x, pose[i].rotation.y, pose[i].rotation.z
        );
    }
}

void IKRigUtils::debugPrintLocalFPose(SoulSkeleton& sk, std::vector<FTransform>& pose) {

    std::vector<FTransform> gpose = pose;
    IKRigUtils::FPoseToGlobal(sk, pose, gpose);

    for (size_t i = 0; i < gpose.size(); i++) {
        printf("%zd %s t(%.2f, %.2f, %.2f)\n", 
            i, 
            sk.joints[i].name.c_str(), 
            gpose[i].Translation.x,
            gpose[i].Translation.y,
            gpose[i].Translation.z);
    }
}

static void debugPrintNodePoseRecursive(SoulNode* rootNode, const glm::mat4& parentM, int depth) {

    if(rootNode == nullptr) {
        return;
    }

    glm::mat4 gm = rootNode->transform * parentM;
    SoulTransform l = SoulTransform(rootNode->transform);
    SoulTransform g = SoulTransform(gm);
    glm::vec3& lt = l.translation;
    glm::vec3& ls = l.scale;
    glm::vec3& gt = g.translation;

    glm::vec3 eulerAngles = glm::eulerAngles(l.rotation);
    float todegree = 180.0f / static_cast<float>(M_PI);
    glm::vec3 eu = todegree * eulerAngles;  // -y o -z

    for(int i = 0; i < depth; i++) {
        if (i == depth-1) {
            printf("|----");
        } else {
            printf("|    ");
        }
    }
    printf("%s l.t(%.2f %.2f %.2f)q(%.2f %.2f %.2f)s(%.2f %.2f %.2f) g.t(%.2f %.2f %.2f)\n", 
        rootNode->name.c_str(), 
        lt.x, lt.y, lt.z, eu.x, eu.y, eu.z, ls.x, ls.y, ls.z, gt.x, gt.y, gt.z);

    for(auto& child : rootNode->children) {
        debugPrintNodePoseRecursive(child.get(), gm, depth+1);
    }
}

void IKRigUtils::debugPrintNodePose(SoulNode* rootNode) {
    debugPrintNodePoseRecursive(rootNode, rootNode->transform, 0);
}

void IKRigUtils::debugPrintUSKNames(USkeleton& usk) {
    for (size_t i = 0; i < usk.boneTree.size(); i++) {
        printf("i:%zd name:%s parent:%d\n", i, usk.boneTree[i].name.c_str(), usk.boneTree[i].parent);
    }
}

static void debugPrintSkeletonTreeRecursive(std::vector<SoulJointNode>& tree, int i, int depth) {

    for(int i = 0; i < depth; i++) {
        if (i == depth-1) {
            printf("|----");
        } else {
            printf("|    ");
        }
    }
    printf("%d %s\n", i, tree[i].name.c_str());
    for(auto& child : tree[i].children) {
        debugPrintSkeletonTreeRecursive(tree, child, depth+1);
    }
}

void IKRigUtils::debugPrintSkeletonTree(SoulSkeleton& sk) {

    size_t jointCount = sk.joints.size();
    
    std::vector<SoulJointNode> tree(jointCount);

    for(size_t i = 0; i < jointCount; i++) {
        tree[i].name = sk.joints[i].name;
        tree[i].parentId = sk.joints[i].parentId;
        tree[i].inverseBindposeMatrix = sk.joints[i].inverseBindposeMatrix;
    }

    for(size_t jointId = 0; jointId < jointCount; jointId++) {
        // search for child
        for(size_t j = jointId + 1; j < jointCount; j++) {
            if (sk.joints[j].parentId == jointId) {
                tree[jointId].children.push_back(static_cast<int32_t>(j));
            }
        }
    }

    debugPrintSkeletonTreeRecursive(tree, 0, 0);
}

static void debugPrintSkeletonTreeIBMRecursive(std::vector<SoulJointNode>& tree, int i, int depth) {

    for(int i = 0; i < depth; i++) {
        if (i == depth-1) {
            printf("|----");
        } else {
            printf("|    ");
        }
    }

    auto iibm = glm::inverse(tree[i].inverseBindposeMatrix);
    SoulTransform transform(iibm);
    glm::vec3 t = transform.translation;
    
    printf("%d %s t(%.2f %.2f %.2f)\n", i, tree[i].name.c_str(), t.x, t.y, t.z);
    for(auto& child : tree[i].children) {
        debugPrintSkeletonTreeIBMRecursive(tree, child, depth+1);
    }
}

std::vector<SoulJointNode> IKRigUtils::buildJointTree(SoulSkeleton& sk) {
    size_t jointCount = sk.joints.size();
    std::vector<SoulJointNode> tree(jointCount);

    for(size_t i = 0; i < jointCount; i++) {
        tree[i].name = sk.joints[i].name;
        tree[i].parentId = sk.joints[i].parentId;
        tree[i].inverseBindposeMatrix = sk.joints[i].inverseBindposeMatrix;
    }

    for(size_t jointId = 0; jointId < jointCount; jointId++) {
        // search for child
        for(size_t j = jointId + 1; j < jointCount; j++) {
            if (sk.joints[j].parentId == jointId) {
                tree[jointId].children.push_back(static_cast<int32_t>(j));
            }
        }
    }

    return tree;
}

void IKRigUtils::debugPrintSkeletonTreeIBM(SoulSkeleton& sk) {
    
    std::vector<SoulJointNode> tree = buildJointTree(sk);

    debugPrintSkeletonTreeIBMRecursive(tree, 0, 0);
}

static void debugPrintPoseJoints(const std::string& prefix, std::vector<FTransform>& inpose, std::vector<int> joints) {
    for(size_t i = 0; i < joints.size(); i++) {
        printf("%s joint:%zd t(%.2f %.2f %.2f) r(%.2f %.2f %.2f %.2f)\n", prefix.c_str(), i,
            inpose[i].GetTranslation().x, inpose[i].GetTranslation().y, inpose[i].GetTranslation().z,
            inpose[i].GetRotation().x, inpose[i].GetRotation().y, inpose[i].GetRotation().z, inpose[i].GetRotation().w
        );
    }
}

void IKRigUtils::debugPrintPoseJoints(const std::string& prefix, SoulSkeleton& sk, std::vector<FTransform>& inpose, std::vector<std::string> jointNames) {
    std::vector<int> joints;
    for(const auto& name : jointNames) {
        joints.push_back(sk.getJointIdByName(name));
    }
    ::debugPrintPoseJoints(prefix, inpose, joints);
}

void IKRigUtils::debugPrintSkeletonTreeTransform(SoulScene& scene, SoulSkeletonMesh& skmesh) {

    //std::vector<SoulJointNode> tree;
    //tree = buildJointTree(skmesh.skeleton);

    std::vector<SoulTransform> pose;
    getSoulPoseFromMesh(scene, skmesh, pose);

    // name t.x, t.y t.z s.x s.y s.z q.x q.y q.z q.w
    for(size_t i = 0; i < skmesh.skeleton.joints.size(); i++) {
        printf("{\"%s\", {{%f, %f, %f}, {%f, %f, %f}, {%f, %f, %f, %f}}},\n",
            skmesh.skeleton.joints[i].name.c_str(),
            pose[i].translation.x, pose[i].translation.y, pose[i].translation.z,
            pose[i].scale.x, pose[i].scale.y, pose[i].scale.z,
            pose[i].rotation.w, pose[i].rotation.x, pose[i].rotation.y, pose[i].rotation.z
        );
    }
}

void IKRigUtils::debugPrintSkeletonTreeGTransform(SoulScene& scene, SoulSkeletonMesh& skmesh) {
    std::vector<SoulTransform> localpose;
    std::vector<SoulTransform> pose;

    getSoulPoseFromMesh(scene, skmesh, localpose);
    SoulPoseToGlobal(skmesh.skeleton, localpose, pose);

    // name t.x, t.y t.z s.x s.y s.z q.x q.y q.z q.w
    for(size_t i = 0; i < skmesh.skeleton.joints.size(); i++) {
        printf("{\"%s\", {{%f, %f, %f}, {%f, %f, %f}, {%f, %f, %f, %f}}},\n",
            skmesh.skeleton.joints[i].name.c_str(),
            pose[i].translation.x, pose[i].translation.y, pose[i].translation.z,
            pose[i].scale.x, pose[i].scale.y, pose[i].scale.z,
            pose[i].rotation.w, pose[i].rotation.x, pose[i].rotation.y, pose[i].rotation.z
        );
    }
}


void IKRigUtils::debugPrintUSkeletonTreeGTransform(SoulSkeleton& sk, USkeleton& usk) {
    
    std::vector<FTransform> pose;
    FPoseToGlobal(sk, usk.refpose, pose);

    // name t.x, t.y t.z s.x s.y s.z q.x q.y q.z q.w
    for(size_t i = 0; i < usk.boneTree.size(); i++) {

        printf("{\"%s\", {{%f, %f, %f}, {%f, %f, %f}, {%f, %f, %f, %f}}},\n",
            usk.boneTree[i].name.c_str(),
            pose[i].Translation.x, pose[i].Translation.y, pose[i].Translation.z,
            pose[i].Scale3D.x, pose[i].Scale3D.y, pose[i].Scale3D.z,
            pose[i].Rotation.w, pose[i].Rotation.x, pose[i].Rotation.y, pose[i].Rotation.z
        );
    }
}

void IKRigUtils::debugPrintNodeTree(SoulNode* node, int depth) {
    for(int i = 0; i < depth; i++) {
        if (i == depth-1) {
            printf("|----");
        } else {
            printf("|    ");
        }
    }

    SoulTransform tf(node->debugTransformGlobal);
    printf("%s t(%.2f %.2f %.2f) r.xyzw(%.2f %.2f %.2f %.2f)\n",
        node->name.c_str(), 
        tf.translation.x, tf.translation.y, tf.translation.z, 
        tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
    );
    for(auto& child : node->children) {
        debugPrintNodeTree(child.get(), depth+1);
    }
}

void IKRigUtils::debugPrintSKM(std::string const& name, SoulScene& scene, SoulSkeletonMesh& skm, CoordType srccoord, CoordType workcoord) {
    printf("%s:\n", name.c_str());
    SoulPose temppose1, temppose2;
    IKRigUtils::getSoulPoseFromMesh(scene, skm, temppose1);

    std::string srccoordstr = IKRigUtils::CoordTypeToString(srccoord);
    printf("skeleton tree1L on srccoord:%s\n", srccoordstr.c_str());
    IKRigUtils::debugPrintSoulPose(skm.skeleton, temppose1.transforms);
    printf("skeleton tree1G on srccoord:%s\n", srccoordstr.c_str());
    IKRigUtils::SoulPoseToGlobal(skm.skeleton, temppose1.transforms, temppose2.transforms);
    IKRigUtils::debugPrintSoulPose(skm.skeleton, temppose2.transforms);
    printf("skeleton tree1L on srccoord:%s\n", srccoordstr.c_str());
    IKRigUtils::SoulPoseToLocal(skm.skeleton, temppose2.transforms, temppose1.transforms);
    IKRigUtils::debugPrintSoulPose(skm.skeleton, temppose1.transforms);

    IKRigUtils::LocalSoulPoseCoordConvert(srccoord, workcoord, temppose1.transforms);
    printf("skeleton tree1L on workcoord\n");
    IKRigUtils::debugPrintSoulPose(skm.skeleton, temppose1.transforms);
    printf("skeleton tree1G on workcoord\n");
    IKRigUtils::SoulPoseToGlobal(skm.skeleton, temppose1.transforms, temppose2.transforms);
    IKRigUtils::debugPrintSoulPose(skm.skeleton, temppose2.transforms);
    printf("skeleton tree1L on workcoord\n");
    IKRigUtils::SoulPoseToLocal(skm.skeleton, temppose2.transforms, temppose1.transforms);
    IKRigUtils::debugPrintSoulPose(skm.skeleton, temppose1.transforms);
}

void IKRigUtils::debugPrintUSK(std::string const& name, USkeleton& usk, SoulSkeletonMesh& skm, CoordType srccoord, CoordType workcoord) {
    printf("%s:\n", name.c_str());
    std::vector<FTransform> tempfpose1, tempfpose2;
    printf("uskeleton tree1L on workcoord\n");
    tempfpose1 = usk.refpose;
    IKRigUtils::debugPrintFPose(skm.skeleton, tempfpose1);
    printf("uskeleton tree1G on workcoord\n");
    IKRigUtils::FPoseToGlobal(skm.skeleton, tempfpose1, tempfpose2);
    IKRigUtils::debugPrintFPose(skm.skeleton, tempfpose2);
    printf("uskeleton tree1L on workcoord\n");
    IKRigUtils::FPoseToLocal(skm.skeleton, tempfpose2, tempfpose1);
    IKRigUtils::debugPrintFPose(skm.skeleton, tempfpose1);
}

void IKRigUtils::debugPrintIOSoulPose(std::string const& name, SoulPose& pose, SoulSkeletonMesh& skm, int frame) {
    printf("%s:\n", name.c_str());
    printf("skeleton tree1L on srccoord: %d\n", frame);
    IKRigUtils::debugPrintSoulPose(skm.skeleton, pose.transforms);
        
    printf("source skeleton tree1G on srccoord: %d\n", frame);
    SoulPose tempposeG;
    IKRigUtils::SoulPoseToGlobal(skm.skeleton, pose.transforms, tempposeG.transforms);
    IKRigUtils::debugPrintSoulPose(skm.skeleton, tempposeG.transforms);
}

void IKRigUtils::debugPrintIOFPose(std::string const& name, SoulSkeletonMesh& skm, std::vector<FTransform>& inposeLocal, std::vector<FTransform>& inpose, int frame) {
    printf("%s:\n", name.c_str());
    printf("fpose local on workcoord\n");
    IKRigUtils::debugPrintFPose(skm.skeleton, inposeLocal);
    printf("fpose global on workcoord\n");
    IKRigUtils::debugPrintFPose(skm.skeleton, inpose);
}