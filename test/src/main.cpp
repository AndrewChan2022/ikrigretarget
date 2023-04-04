//
//  main.cpp
//  test libGLVNDGLContext
//
//  Created by kai chen on 2/8/23.
//

//#include    <stdio.h>
//#include    <stdlib.h>
#include <stdio.h>
#include "SoulTransform.h"
#include "SoulRetargeter.h"
#include "SoulIKRetargetProcessor.h"

#include "FBXRW.h"
#include "IKRigUtils.hpp"


using namespace SoulIK;

extern "C" 
bool test_libikrigretarget();

SoulNode* findNodeOfTreeByName(SoulNode* rootNode, std::string& name) {
    if (rootNode->name == name) {
        return rootNode;
    }

    for(auto& child : rootNode->children) {
        auto ret = findNodeOfTreeByName(child.get(), name);
        if (ret != nullptr) {
            return ret;
        }
    }
    return nullptr;
}

bool getRefPoseFromMesh(SoulIK::SoulScene& scene, SoulIK::SoulSkeletonMesh& skmesh, std::vector<SoulIK::SoulTransform>& pose) {

    SoulSkeleton& sk = skmesh.skeleton;
    std::vector<SoulJoint>&  joints = sk.joints;

    // refpose
    for(uint64_t i = 0; i < joints.size(); i++) {
        std::string name = joints[i].name;

        // find node
        auto node = findNodeOfTreeByName(scene.rootNode.get(), name);
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
bool getUSkeletonFromMesh(SoulIK::SoulScene& scene, SoulIK::SoulSkeletonMesh& skmesh, SoulIK::USkeleton& usk) {

    SoulSkeleton& sk = skmesh.skeleton;
    std::vector<SoulJoint>&  joints = sk.joints;
    
    // name
    usk.name = scene.skmeshes[0]->name;

    // bone tree
    for(uint64_t i = 0; i < joints.size(); i++) {
        SoulJoint& joint = joints[i];
        FBoneNode node;
        node.name = joint.name;
        node.parent = joint.parentId;
        usk.boneTree.push_back(node);
    }

    // refpose is local
    for(uint64_t i = 0; i < joints.size(); i++) {
        std::string name = joints[i].name;

        // find node
        auto node = findNodeOfTreeByName(scene.rootNode.get(), name);
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

    return true;
}

// generate pose of every joint and every frame
void buildSoulPoseAnimation(SoulIK::SoulScene& scene, 
                        SoulIK::SoulSkeletonMesh& skmesh,
                        SoulIK::SoulJointAnimation& animation,
                        std::vector<SoulIK::SoulPose>& poses) {


    std::vector<SoulIK::SoulTransform> refpose(skmesh.skeleton.joints.size()); 
    getRefPoseFromMesh(scene, skmesh, refpose);

    std::vector<SoulAniChannel> channelsSparse(skmesh.skeleton.joints.size());  // [jointId][frame]
    std::vector<SoulAniChannel> channelsDense(skmesh.skeleton.joints.size());   // [jointId][frame]

    // copy
    for(int i = 0; i < skmesh.animation.channels.size(); i++) {
        SoulAniChannel& channel = skmesh.animation.channels[i];
        uint32_t jointId = channel.jointId;
        channelsSparse[jointId] = channel;
    }

    // interpolation
    std::vector<SoulIK::SoulTransform> curpose = refpose;
    for(int jointId = 0; jointId < skmesh.skeleton.joints.size(); jointId++) {
        
        int prevFrame = 0;
        int curFrame = 0;
        int frameCount = skmesh.animation.duration + 1;
        
        // position
        if (channelsSparse[jointId].PositionKeys.size() == 0) {
            auto& DenseKeys = channelsDense[jointId].PositionKeys;
            SoulVec3Key key{0, refpose[jointId].translation};
            DenseKeys.resize(frameCount, key);
            for(int frame = 0; frame < skmesh.animation.duration; frame++) {
                DenseKeys[frame].time = frame;
            }
        } else {
            auto& SparseKeys = channelsSparse[jointId].PositionKeys;
            auto& DenseKeys = channelsDense[jointId].PositionKeys;
            DenseKeys.resize(frameCount);

            prevFrame = 0;
            glm::vec3 prevValue = SparseKeys[0].value;
            glm::vec3 curValue;
            for (int j = 0; j < SparseKeys.size(); j++) {
                curFrame = SparseKeys[j].time;
                curValue = SparseKeys[j].value;
                for(int frame = prevFrame; frame <= curFrame; frame++) {
                    DenseKeys[frame].time = frame;
                    float alpha = prevFrame == curFrame ? 1 : (frame - prevFrame) / (curFrame - prevFrame); 
                    DenseKeys[frame].value = glm::mix(prevValue, curValue, alpha);
                }

                // next iteration
                prevFrame = curFrame;
                prevValue = curValue;
            }
            // tail
            for(int frame = prevFrame+1; frame < frameCount; frame++) {
                DenseKeys[frame].time = frame;
                DenseKeys[frame].value = curValue;
            }
        }

        // scale
        if (channelsSparse[jointId].ScalingKeys.size() == 0) {
            auto& DenseKeys = channelsDense[jointId].ScalingKeys;
            SoulVec3Key key{0, refpose[jointId].scale};
            DenseKeys.resize(frameCount, key);
            for(int frame = 0; frame < skmesh.animation.duration; frame++) {
                DenseKeys[frame].time = frame;
            }
        } else {
            auto& SparseKeys = channelsSparse[jointId].ScalingKeys;
            auto& DenseKeys = channelsDense[jointId].ScalingKeys;
            DenseKeys.resize(frameCount);

            prevFrame = 0;
            glm::vec3 prevValue = SparseKeys[0].value;
            glm::vec3 curValue;
            for (int j = 0; j < SparseKeys.size(); j++) {
                curFrame = SparseKeys[j].time;
                curValue = SparseKeys[j].value;
                for(int frame = prevFrame; frame <= curFrame; frame++) {
                    DenseKeys[frame].time = frame;
                    float alpha = prevFrame == curFrame ? 1 : (frame - prevFrame) / (curFrame - prevFrame); 
                    DenseKeys[frame].value = glm::mix(prevValue, curValue, alpha);
                }

                // next iteration
                prevFrame = curFrame;
                prevValue = curValue;
            }
            // tail
            for(int frame = prevFrame+1; frame < frameCount; frame++) {
                DenseKeys[frame].time = frame;
                DenseKeys[frame].value = curValue;
            }
        }

        // rotation
        if (channelsSparse[jointId].RotationKeys.size() == 0) {
            auto& DenseKeys = channelsDense[jointId].RotationKeys;
            SoulQuatKey key{0, refpose[jointId].rotation};
            DenseKeys.resize(frameCount, key);
            for(int frame = 0; frame < skmesh.animation.duration; frame++) {
                DenseKeys[frame].time = frame;
            }
        } else {
            auto& SparseKeys = channelsSparse[jointId].RotationKeys;
            auto& DenseKeys = channelsDense[jointId].RotationKeys;
            DenseKeys.resize(frameCount);

            prevFrame = 0;
            glm::quat prevValue = SparseKeys[0].value;
            glm::quat curValue;
            for (int j = 0; j < SparseKeys.size(); j++) {
                curFrame = SparseKeys[j].time;
                curValue = SparseKeys[j].value;
                for(int frame = prevFrame; frame <= curFrame; frame++) {
                    DenseKeys[frame].time = frame;
                    float alpha = prevFrame == curFrame ? 1 : (frame - prevFrame) / (curFrame - prevFrame); 
                    DenseKeys[frame].value = glm::lerp(prevValue, curValue, alpha);
                }

                // next iteration
                prevFrame = curFrame;
                prevValue = curValue;
            }
            // tail
            for(int frame = prevFrame+1; frame < frameCount; frame++) {
                DenseKeys[frame].time = frame;
                DenseKeys[frame].value = curValue;
            }
        }
    }

    // pose
    poses.resize(skmesh.animation.duration);
    int frameCount = skmesh.animation.duration;
    for(int frame = 0; frame < frameCount; frame++) {
        SoulPose& pose = poses[frame];
        pose.transforms.resize(skmesh.skeleton.joints.size());
        for(int jointId = 0; jointId < skmesh.skeleton.joints.size(); jointId++) {
            pose.transforms[jointId].translation = channelsDense[jointId].PositionKeys[frame].value;
            pose.transforms[jointId].scale = channelsDense[jointId].ScalingKeys[frame].value;
            pose.transforms[jointId].rotation = channelsDense[jointId].RotationKeys[frame].value;
        }
    }
}


void writeSoulPosesToAnimation(std::vector<SoulIK::SoulPose>& tempoutposes, SoulIK::SoulSkeletonMesh& tgtskm) {
    tgtskm.animation.channels.clear();
    tgtskm.animation.channels.resize(tgtskm.skeleton.joints.size());
    for(uint32_t jointId = 0; jointId < tgtskm.skeleton.joints.size(); jointId++) {
        tgtskm.animation.channels[jointId].jointId = jointId;
        auto frameCount = tempoutposes.size();
        // position
        tgtskm.animation.channels[jointId].PositionKeys.resize(frameCount);
        for(uint64_t frame = 0; frame < tempoutposes.size(); frame++) {
            tgtskm.animation.channels[jointId].PositionKeys[frame].time = frame;
            tgtskm.animation.channels[jointId].PositionKeys[frame].value =  tempoutposes[frame].transforms[jointId].translation;
        }

        tgtskm.animation.channels[jointId].ScalingKeys.resize(frameCount);
        for(uint64_t frame = 0; frame < tempoutposes.size(); frame++) {
            tgtskm.animation.channels[jointId].ScalingKeys[frame].time = frame;
            tgtskm.animation.channels[jointId].ScalingKeys[frame].value =  tempoutposes[frame].transforms[jointId].scale;
        }

        tgtskm.animation.channels[jointId].RotationKeys.resize(frameCount);
        for(uint64_t frame = 0; frame < tempoutposes.size(); frame++) {
            tgtskm.animation.channels[jointId].RotationKeys[frame].time = frame;
            tgtskm.animation.channels[jointId].RotationKeys[frame].value =  tempoutposes[frame].transforms[jointId].rotation;
        }        
    }
}

bool USkeleton2RigSkeleton(SoulIK::USkeleton& sk, FIKRigSkeleton& rigsk) {
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

std::shared_ptr<UIKRetargeter> buildIKRigRetargetAsset_config1_1chain_lleg(SoulIK::SoulSkeleton& srcsk, SoulIK::SoulSkeleton& tgtsk, SoulIK::USkeleton& srcusk, SoulIK::USkeleton& tgtusk) {

    std::shared_ptr<UIKRetargeter> InRetargeterAsset = std::make_shared<UIKRetargeter>();

    // ikrig1 asset
    InRetargeterAsset->SourceIKRigAsset = std::make_shared<UIKRigDefinition>();
    FIKRigSkeleton rigsk;
    USkeleton2RigSkeleton(srcusk, rigsk);
    InRetargeterAsset->SourceIKRigAsset->Skeleton = rigsk;
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.RootBone = "Hip";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.GroundBone = "RightAnkle_end";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains.resize(1);
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightHip";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("RightHip");
    //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightKnee";
    //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("RightKnee");
    //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightAnkle";
    //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("RightAnkle");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightKnee";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("RightKnee");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "lleg";

    // ikrig2 asset
    InRetargeterAsset->TargetIKRigAsset = std::make_shared<UIKRigDefinition>();
    FIKRigSkeleton rigsk2;
    USkeleton2RigSkeleton(tgtusk, rigsk2);
    InRetargeterAsset->SourceIKRigAsset->Skeleton = rigsk2;
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.RootBone = "Hip";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.GroundBone = "RightAnkle_end";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains.resize(1);
    //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightKnee";
    //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = tgtsk.getJointIdByName("RightKnee");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightHip";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = tgtsk.getJointIdByName("RightHip");
    //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightAnkle";
    //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = tgtsk.getJointIdByName("RightAnkle");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightKnee";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = tgtsk.getJointIdByName("RightKnee");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "lleg";

    // ik rig retarget asset
    std::shared_ptr<URetargetChainSettings> chainSetting = std::make_shared<URetargetChainSettings>();
    chainSetting->Settings.FK.EnableFK = true;
    chainSetting->Settings.IK.EnableIK = false;
    chainSetting->SourceChain = "lleg";
    chainSetting->TargetChain = "lleg";
    InRetargeterAsset->ChainSettings.push_back(chainSetting);

    return InRetargeterAsset;
}

std::shared_ptr<UIKRetargeter> buildIKRigRetargetAsset_config2_6chain(SoulIK::SoulSkeleton& srcsk, SoulIK::SoulSkeleton& tgtsk, SoulIK::USkeleton& srcusk, SoulIK::USkeleton& tgtusk) {

    std::shared_ptr<UIKRetargeter> InRetargeterAsset = std::make_shared<UIKRetargeter>();

    // ikrig1 asset
    InRetargeterAsset->SourceIKRigAsset = std::make_shared<UIKRigDefinition>();
    FIKRigSkeleton rigsk;
    USkeleton2RigSkeleton(srcusk, rigsk);
    InRetargeterAsset->SourceIKRigAsset->Skeleton = rigsk;
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.RootBone = "Hip";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.GroundBone = "RightAnkle_end";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains.resize(6);

    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "Spine";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("Spine");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "Thorax";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("Thorax");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "spine";

    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneName = "RightHip";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneIndex = srcsk.getJointIdByName("RightHip");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneName = "RightAnkle";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneIndex = srcsk.getJointIdByName("RightAnkle");;
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].ChainName = "lleg";

    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneName = "LeftHip";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneIndex = srcsk.getJointIdByName("LeftHip");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneName = "LeftAnkle";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneIndex = srcsk.getJointIdByName("LeftAnkle");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].ChainName = "rleg";

    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneName = "LeftShoulder";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneIndex = srcsk.getJointIdByName("LeftShoulder");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneName = "LeftWrist";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneIndex = srcsk.getJointIdByName("LeftWrist");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].ChainName = "larm";

    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneName = "RightShoulder";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneIndex = srcsk.getJointIdByName("RightShoulder");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneName = "RightWrist";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneIndex = srcsk.getJointIdByName("RightWrist");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].ChainName = "rarm";

    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneName = "Neck";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneIndex = srcsk.getJointIdByName("Neck");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneName = "Head";
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneIndex = srcsk.getJointIdByName("Head");
    InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].ChainName = "head";

    // ikrig2 asset
    InRetargeterAsset->TargetIKRigAsset = std::make_shared<UIKRigDefinition>();
    FIKRigSkeleton rigsk2;
    USkeleton2RigSkeleton(tgtusk, rigsk2);
    InRetargeterAsset->SourceIKRigAsset->Skeleton = rigsk2;
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.RootBone = "Hip";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.GroundBone = "RightAnkle_end";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains.resize(6);

    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "Spine";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("Spine");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "Thorax";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("Thorax");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "spine";

    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneName = "RightHip";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneIndex = srcsk.getJointIdByName("RightHip");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneName = "RightAnkle";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneIndex = srcsk.getJointIdByName("RightAnkle");;
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].ChainName = "lleg";

    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneName = "LeftHip";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneIndex = srcsk.getJointIdByName("LeftHip");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneName = "LeftAnkle";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneIndex = srcsk.getJointIdByName("LeftAnkle");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].ChainName = "rleg";

    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneName = "LeftShoulder";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneIndex = srcsk.getJointIdByName("LeftShoulder");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneName = "LeftWrist";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneIndex = srcsk.getJointIdByName("LeftWrist");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].ChainName = "larm";

    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneName = "RightShoulder";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneIndex = srcsk.getJointIdByName("RightShoulder");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneName = "RightWrist";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneIndex = srcsk.getJointIdByName("RightWrist");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].ChainName = "rarm";

    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneName = "Neck";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneIndex = srcsk.getJointIdByName("Neck");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneName = "Head";
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneIndex = srcsk.getJointIdByName("Head");
    InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].ChainName = "head";
    


    // ik rig retarget asset
    std::shared_ptr<URetargetChainSettings> chainSetting0 = std::make_shared<URetargetChainSettings>();
    chainSetting0->Settings.FK.EnableFK = true;
    chainSetting0->Settings.IK.EnableIK = false;
    chainSetting0->SourceChain = "spine";
    chainSetting0->TargetChain = "spine";
    InRetargeterAsset->ChainSettings.push_back(chainSetting0);

    std::shared_ptr<URetargetChainSettings> chainSetting1 = std::make_shared<URetargetChainSettings>();
    chainSetting1->Settings.FK.EnableFK = true;
    chainSetting1->Settings.IK.EnableIK = false;
    chainSetting1->SourceChain = "lleg";
    chainSetting1->TargetChain = "lleg";
    InRetargeterAsset->ChainSettings.push_back(chainSetting1);

    std::shared_ptr<URetargetChainSettings> chainSetting2 = std::make_shared<URetargetChainSettings>();
    chainSetting2->Settings.FK.EnableFK = true;
    chainSetting2->Settings.IK.EnableIK = false;
    chainSetting2->SourceChain = "rleg";
    chainSetting2->TargetChain = "rleg";
    InRetargeterAsset->ChainSettings.push_back(chainSetting2);

    std::shared_ptr<URetargetChainSettings> chainSetting3 = std::make_shared<URetargetChainSettings>();
    chainSetting3->Settings.FK.EnableFK = true;
    chainSetting3->Settings.IK.EnableIK = false;
    chainSetting3->SourceChain = "larm";
    chainSetting3->TargetChain = "larm";
    InRetargeterAsset->ChainSettings.push_back(chainSetting3);

    std::shared_ptr<URetargetChainSettings> chainSetting4 = std::make_shared<URetargetChainSettings>();
    chainSetting4->Settings.FK.EnableFK = true;
    chainSetting4->Settings.IK.EnableIK = false;
    chainSetting4->SourceChain = "rarm";
    chainSetting4->TargetChain = "rarm";
    InRetargeterAsset->ChainSettings.push_back(chainSetting4);

    std::shared_ptr<URetargetChainSettings> chainSetting5 = std::make_shared<URetargetChainSettings>();
    chainSetting5->Settings.FK.EnableFK = true;
    chainSetting5->Settings.IK.EnableIK = false;
    chainSetting5->SourceChain = "head";
    chainSetting5->TargetChain = "head";
    InRetargeterAsset->ChainSettings.push_back(chainSetting5);

    return InRetargeterAsset;
}

int main(int argc, char *argv[]) {

    std::string file_path = __FILE__;
#ifdef _WIN64
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    //std::string inputfile = dir_path + "\\..\\..\\model\\3D_Avatar2_Rig_0723.fbx";
    //std::string outfile = dir_path + "\\..\\..\\model\\3D_Avatar2_Rig_0723_out.fbx";
    // std::string inputfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts.fbx";
    // std::string outfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts_tiny_out.fbx";
    std::string inputfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts_tiny_ball.fbx";
    std::string inputfile2 = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts_tiny_ball.fbx";
    std::string outfile = dir_path + "\\..\\..\\model\\out.fbx";
    
#else
    std::string dir_path = file_path.substr(0, file_path.rfind("/"));
    std::string inputfile = dir_path + "/../../model/3D_Avatar2_Rig_0723.fbx";
    std::string outfile = dir_path + "/../../model/3D_Avatar2_Rig_0723_out.fbx";
#endif

    SoulIK::FBXRW  fbxrw;
    SoulIK::FBXRW  fbxrw2;
    fbxrw.readSkeketonMesh(inputfile);
    fbxrw2.readSkeketonMesh(inputfile2);
    //fbxrw.writeSkeletonMesh(outfile);


    // build uskeleton of mesh 0
    SoulIK::USkeleton srcusk;
    SoulIK::USkeleton tgtusk;

    SoulIK::SoulScene& srcscene = *fbxrw.getSoulScene();
    SoulIK::SoulScene& tgtscene = *fbxrw2.getSoulScene();
    auto& srcskm = *srcscene.skmeshes[0];
    auto& tgtskm = *tgtscene.skmeshes[0];

    getUSkeletonFromMesh(srcscene, srcskm, srcusk);
    getUSkeletonFromMesh(tgtscene, tgtskm, tgtusk);

    // build input pose of mesh 0
    std::vector<SoulIK::SoulPose> tempposes;
    std::vector<SoulIK::SoulPose> tempoutposes;
    buildSoulPoseAnimation(srcscene, srcskm, srcskm.animation, tempposes);
    int frameCount = tempposes.size();
    tempoutposes.resize(tempposes.size());
    
    // build retargetProcessor
    SoulIK::UIKRetargetProcessor ikretarget;
	std::shared_ptr<UIKRetargeter> InRetargeterAsset = buildIKRigRetargetAsset_config2_6chain(srcskm.skeleton, tgtskm.skeleton, srcusk, tgtusk);
    ikretarget.Initialize(&srcusk, &tgtusk, InRetargeterAsset.get(), false);

    // run retarget
    CoordType srccoord = CoordType::RightHandYupZfront;
    CoordType selfcoord = CoordType::RightHandZupYfront;
    CoordType tgtcoord = CoordType::RightHandYupZfront;
    std::unordered_map<FName, float> SpeedValuesFromCurves;
    float DeltaTime = 0;

    std::vector<FTransform> inpose;
    std::vector<FTransform> inposeLocal;
    std::vector<FTransform> outposeLocal;
    for(int frame = 0; frame < tempposes.size(); frame++) {
        
        // if (frame == 0) { inposeLocal = tgtusk.refpose;}
        // printf("%d: t(%f %f %f) t(%f %f %f) t(%f %f %f) %f %f %f\n", frame, 
        //     inpose[1].Translation.x, inpose[1].Translation.y, inpose[1].Translation.z,
        //     inpose[2].Translation.x, inpose[2].Translation.y, inpose[2].Translation.z,
        //     inpose[3].Translation.x, inpose[3].Translation.y, inpose[3].Translation.z,
        //     inpose[1].Rotation.getAngleDegree(), inpose[2].Rotation.getAngleDegree(), inpose[3].Rotation.getAngleDegree());

        IKRigUtils::SoulPose2FPose(tempposes[frame], inposeLocal);

        // retarget
        IKRigUtils::LocalPoseCoordConvert(srccoord, selfcoord, inposeLocal);
        IKRigUtils::FPoseToGlobal(tgtskm.skeleton, inposeLocal, inpose);
        std::vector<FTransform>& outpose = ikretarget.RunRetargeter(inpose, SpeedValuesFromCurves, DeltaTime);
        IKRigUtils::FPoseToLocal(tgtskm.skeleton, outpose, outposeLocal);
        IKRigUtils::LocalPoseCoordConvert(selfcoord, tgtcoord, outposeLocal);

        //
        IKRigUtils::FPose2SoulPose(outposeLocal, tempoutposes[frame]);
    }

    // output to mesh 0
    writeSoulPosesToAnimation(tempoutposes, tgtskm);
    fbxrw2.writeSkeletonMesh(outfile);

    //bool ret = test_libikrigretarget();
    
    return 0;
}
