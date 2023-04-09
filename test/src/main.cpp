//
//  main.cpp
//  test libGLVNDGLContext
//
//  Created by kai chen on 2/8/23.
//

#include <stdio.h>
#include "SoulRetargeter.h"
#include "SoulIKRetargetProcessor.h"

#include "FBXRW.h"
#include "ObjRW.h"
#include "IKRigUtils.hpp"

using namespace SoulIK;

// generate pose of every joint and every frame
static void buildPoseAnimationByInterpolation(SoulScene& scene, SoulSkeletonMesh& skmesh,
                        SoulJointAnimation& animation, std::vector<SoulIK::SoulPose>& poses) {

    std::vector<SoulIK::SoulTransform> refpose(skmesh.skeleton.joints.size()); 
    IKRigUtils::getSoulPoseFromMesh(scene, skmesh, refpose);

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

static void writePoseAnimationToMesh(std::vector<SoulIK::SoulPose>& tempoutposes, SoulIK::SoulSkeletonMesh& tgtskm, double duration, double ticksPerSecond) {

    // cannot save if no name
    tgtskm.animation.name = "mesh0";
    tgtskm.animation.duration = duration;
    tgtskm.animation.ticksPerSecond = ticksPerSecond;
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

static std::shared_ptr<UIKRetargeter> createIKRigAsset(SoulIKRigRetargetConfig& config,
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

    return pInRetargeterAsset;
}

static SoulPose getMetaTPoseSoulPose(SoulSkeleton& sk) {

    struct PoseItem {
        glm::vec3 t;
        glm::vec3 s;
        glm::quat q;
    };
    std::unordered_map<std::string, PoseItem> posedict = {
        //{"Rol01_Torso01HipCtrlJnt_M", {{0.000000, -11.151259, 692.766296}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Torso01HipCtrlJnt_M", {{0.000000, 692.766296, -11.151259}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Torso01HipCtrlJnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Torso0101Jnt_M", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {-0.018845, 0.706856, 0.706856, 0.018845}}},
        {"Torso0101Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Torso0102Jnt_M", {{34.375542, -0.000000, 0.383953}, {1.000000, 1.000000, 1.000000}, {0.999944, 0.000000, -0.010605, 0.000000}}},
        {"Torso0102Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Torso0103Jnt_M", {{34.376987, -0.000000, 0.267512}, {1.000000, 1.000000, 1.000000}, {0.999974, 0.000000, -0.007215, 0.000000}}},
        {"Torso0103Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Torso0104Jnt_M", {{34.377270, -0.000000, 0.228687}, {1.000000, 1.000000, 1.000000}, {0.999974, 0.000000, -0.007220, 0.000000}}},
        {"Torso0104Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Torso0105Jnt_M", {{34.375942, -0.000000, 0.345904}, {1.000000, 1.000000, 1.000000}, {0.999943, 0.000000, -0.010631, 0.000000}}},
        {"Torso0105Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Up01Jnt_L", {{-70.755814, 65.134995, 6.021814}, {1.000000, 1.000000, 1.000000}, {-0.006912, -0.006912, -0.707073, 0.707073}}},
        {"Leg01Up01Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, -0.000000}}},
        {"Rol01_Leg01Up02Jnt_L", {{139.477142, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Up02Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Up03Jnt_L", {{139.477142, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Up03Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Low01Jnt_L", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.999703, 0.000000, 0.000000, -0.024377}}},
        {"Leg01Low01Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Low02Jnt_L", {{126.577507, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Low02Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Low03Jnt_L", {{126.577507, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Low03Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01AnkleJnt_L", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.706506, -0.706506, -0.029137, 0.029137}}},
        {"Rol01_Leg01FootJnt_L", {{64.388062, 10.310209, 112.016861}, {1.000000, 1.000000, 1.000000}, {0.706362, -0.032439, -0.706362, 0.032439}}},
        {"Rol01_Neck0101Jnt_M", {{130.601349, 0.000000, 29.095982}, {1.000000, 1.000000, 1.000000}, {0.996415, -0.000000, 0.084602, 0.000000}}},
        {"Neck0101Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Neck0102Jnt_M", {{27.701897, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Neck0102Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Neck0103Jnt_M", {{27.701897, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Head_M", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.053466, 0.705083, 0.705083, 0.053466}}},
        {"Neck0103Jnt_M_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {-0.053466, 0.705082, 0.705083, 0.053466}}},
        {"Eye_L", {{37.007999, 78.077209, 51.198513}, {1.000000, 1.000000, 1.000000}, {0.995089, 0.047223, 0.086901, -0.004124}}},
        {"Rol01_Arm01ClavicleStartJnt_L", {{100.361694, 11.303822, -14.647395}, {1.000000, 1.000000, 1.000000}, {-0.176047, 0.618006, 0.740107, 0.198288}}},
        {"Rol01_Arm01Up01Jnt_L", {{87.926872, -0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {0.654214, 0.707379, -0.096248, 0.249711}}},
        {"Arm01Up01Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, -0.000000, -0.000000, 0.000000}}},
        {"Rol01_Arm01Up02Jnt_L", {{89.262054, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Arm01Up02Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Arm01Up03Jnt_L", {{89.262054, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Arm01Up03Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Arm01Low01Jnt_L", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.991170, -0.003312, 0.025363, 0.130109}}},
        {"Arm01Low01Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, -0.000000}}},
        {"Rol01_Arm01Low02Jnt_L", {{79.556503, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, -0.000079, 0.000000, 0.000000}}},
        {"Arm01Low02Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Arm01Low03Jnt_L", {{79.556503, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, -0.000079, 0.000000, 0.000000}}},
        {"Arm01Low03Jnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Arm01ClavicleStartJnt_L_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Arm01ClavicleStartJnt_R", {{100.361694, -11.303822, -14.647395}, {1.000000, 1.000000, 1.000000}, {0.198288, 0.740107, 0.618006, -0.176047}}},
        {"Rol01_Arm01Up01Jnt_R", {{-87.926872, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.712401, -0.652079, 0.235001, 0.109783}}},
        {"Arm01Up01Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Arm01Up02Jnt_R", {{-89.262054, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Arm01Up02Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Arm01ClavicleStartJnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, -0.000000, -0.000000, -0.000000}}},
        {"Rol01_Arm01Up03Jnt_R", {{-89.262054, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Arm01Low01Jnt_R", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.991070, -0.003791, 0.029015, 0.130096}}},
        {"Rol01_Arm01Low02Jnt_R", {{-79.556503, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, -0.000079, 0.000000, 0.000000}}},
        {"Arm01Low02Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Arm01Up03Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Arm01Low01Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, -0.000000}}},
        {"Rol01_Arm01Low03Jnt_R", {{-79.556503, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, -0.000079, 0.000000, 0.000000}}},
        {"Arm01Low03Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Up01Jnt_R", {{-70.755814, -65.134995, 6.021814}, {1.000000, 1.000000, 1.000000}, {0.707073, 0.707073, -0.006912, 0.006912}}},
        {"Rol01_Leg01Up02Jnt_R", {{-139.477142, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Up03Jnt_R", {{-139.477142, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Low01Jnt_R", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.999703, 0.000000, 0.000000, -0.024377}}},
        {"Rol01_Leg01Low02Jnt_R", {{-126.577507, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Low02Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01Low03Jnt_R", {{-126.577507, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01AnkleJnt_R", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.706506, 0.706506, 0.029137, 0.029137}}},
        {"Leg01Low01Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Up01Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, -0.000000, 0.000000, -0.000000}}},
        {"Leg01Up02Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Up03Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Leg01Low03Jnt_R_Scale", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Leg01FootJnt_R", {{-64.388062, 10.310209, 112.016861}, {1.000000, 1.000000, 1.000000}, {0.706362, -0.032439, 0.706362, -0.032439}}},
        {"Rol01_Hand01MasterJnt_R", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.706517, 0.696987, -0.119843, -0.026085}}},
        {"Rol01_Hand01Thumb01FKCtrlJnt_R", {{-16.920595, -5.850204, 25.255711}, {1.000000, 1.000000, 1.000000}, {-0.401013, 0.826006, -0.090787, -0.385566}}},
        {"Rol01_Hand01Thumb02FKCtrlJnt_R", {{-20.364660, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.995840, -0.030879, 0.035645, -0.077959}}},
        {"Rol01_Hand01Thumb03FKCtrlJnt_R", {{-24.680334, -0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Little02FKCtrlJnt_R", {{-58.057373, -2.451173, -20.710228}, {1.000000, 1.000000, 1.000000}, {0.127064, 0.991454, -0.027657, -0.010459}}},
        {"Rol01_Hand01Little03FKCtrlJnt_R", {{-20.926340, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Little04FKCtrlJnt_R", {{-18.014015, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.999347, 0.000000, 0.000000, 0.036137}}},
        {"Rol01_Hand01Middle02FKCtrlJnt_R", {{-61.846611, 3.868438, 5.959052}, {1.000000, 1.000000, 1.000000}, {0.082454, 0.995782, -0.039721, 0.006401}}},
        {"Rol01_Hand01Middle03FKCtrlJnt_R", {{-31.543697, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Eye_R", {{-37.007999, 78.077209, 51.198513}, {1.000000, 1.000000, 1.000000}, {0.995089, 0.047223, -0.086901, 0.004124}}},
        {"Breast01RT101FKCtrlJnt_M_Scale", {{26.904409, -36.319008, -44.162155}, {1.000000, 1.000000, 1.000000}, {0.120786, 0.705796, 0.696714, -0.043040}}},
        {"Neck01HeadUpCtrlJnt_M_Scale", {{0.000000, 123.910492, 9.489252}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01MasterJnt_L", {{0.000000, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {0.696987, -0.706517, 0.026085, -0.119843}}},
        {"Rol01_Hand01Thumb01FKCtrlJnt_L", {{16.920595, -5.850204, 25.255711}, {1.000000, 1.000000, 1.000000}, {0.826005, 0.401014, -0.385566, 0.090787}}},
        {"Rol01_Hand01Middle04FKCtrlJnt_R", {{-20.268305, -0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Breast01LF101FKCtrlJnt_M_Scale", {{26.904409, 36.319008, -44.162155}, {1.000000, 1.000000, 1.000000}, {-0.043042, 0.696714, 0.705796, 0.120788}}},
        {"Rol01_Hand01Index02FKCtrlJnt_R", {{-62.060173, 5.828954, 20.878521}, {1.000000, 1.000000, 1.000000}, {0.017047, 0.999431, -0.019288, 0.021814}}},
        {"Rol01_Hand01Index03FKCtrlJnt_R", {{-28.276159, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Index04FKCtrlJnt_R", {{-18.842459, -0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Ring02FKCtrlJnt_R", {{-60.190514, 0.354814, -7.404934}, {1.000000, 1.000000, 1.000000}, {0.091191, 0.994891, -0.043322, -0.000418}}},
        {"Rol01_Hand01Ring03FKCtrlJnt_R", {{-29.537693, -0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Ring04FKCtrlJnt_R", {{-19.117027, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Ring02FKCtrlJnt_L", {{60.190514, 0.354814, -7.404934}, {1.000000, 1.000000, 1.000000}, {0.994891, -0.091191, -0.000418, 0.043322}}},
        {"Rol01_Hand01Ring03FKCtrlJnt_L", {{29.537693, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Ring04FKCtrlJnt_L", {{19.117027, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Thumb02FKCtrlJnt_L", {{20.364660, -0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {0.995840, -0.030879, 0.035645, -0.077959}}},
        {"Rol01_Hand01Middle02FKCtrlJnt_L", {{61.846611, 3.868438, 5.959052}, {1.000000, 1.000000, 1.000000}, {0.995782, -0.082454, 0.006401, 0.039721}}},
        {"Rol01_Hand01Middle03FKCtrlJnt_L", {{31.543697, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Middle04FKCtrlJnt_L", {{20.268305, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Thumb03FKCtrlJnt_L", {{24.680334, -0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Index02FKCtrlJnt_L", {{62.060173, 5.828954, 20.878521}, {1.000000, 1.000000, 1.000000}, {0.999431, -0.017047, 0.021814, 0.019288}}},
        {"Rol01_Hand01Little02FKCtrlJnt_L", {{58.057373, -2.451173, -20.710228}, {1.000000, 1.000000, 1.000000}, {0.991454, -0.127064, -0.010459, 0.027657}}},
        {"Rol01_Hand01Little03FKCtrlJnt_L", {{20.926340, -0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Little04FKCtrlJnt_L", {{18.014015, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {0.999347, 0.000000, 0.000000, 0.036137}}},
        {"Rol01_Hand01Index03FKCtrlJnt_L", {{28.276159, 0.000000, 0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
        {"Rol01_Hand01Index04FKCtrlJnt_L", {{18.842459, 0.000000, -0.000000}, {1.000000, 1.000000, 1.000000}, {1.000000, 0.000000, 0.000000, 0.000000}}},
    };

    SoulPose soulpose;
    soulpose.transforms.resize(sk.joints.size());
    for(size_t i = 0; i < sk.joints.size(); i++) {
        auto it = posedict.find(sk.joints[i].name);
        
        assert(it != posedict.end());

        if(it != posedict.end()) {
            auto& item = it->second;
            soulpose.transforms[i].translation = item.t;
            soulpose.transforms[i].scale = item.s;
            soulpose.transforms[i].rotation = item.q;
        }
    }
    return soulpose;
}

static std::vector<FTransform> getMetaTPoseFPose(SoulSkeleton& sk, CoordType srcCoord, CoordType tgtCoord) {

    std::vector<FTransform> pose(sk.joints.size());

    auto soulpose = getMetaTPoseSoulPose(sk);
    IKRigUtils::SoulPose2FPose(soulpose, pose);
    IKRigUtils::LocalFPoseCoordConvert(srcCoord, tgtCoord, pose);

    return pose;
}

static SoulIKRigRetargetConfig config1_1chain_lleg() {
    SoulIKRigRetargetConfig config;
    config.SourceCoord  = CoordType::RightHandZupYfront;
    config.WorkCoord    = CoordType::RightHandZupYfront;
    config.TargetCoord  = CoordType::RightHandZupYfront;

    config.SourceRootBone   = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone   = "Hip";
    config.TargetGroundBone = "RightAnkle_end";

    config.SourceChains = {
        // name    start        end
        {"lleg", "RightHip", "RightKnee"}
    };

    config.TargetChains = {
        // name    start        end
        {"lleg", "RightHip", "RightKnee"}
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        {true,  false,  "lleg",         "lleg"}
    };

    return config;
}

static SoulIKRigRetargetConfig config2_6chain() {

    SoulIKRigRetargetConfig config;

    config.SourceCoord      = CoordType::RightHandZupYfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandZupYfront;

    config.SourceRootBone   = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone   = "Hip";
    config.TargetGroundBone = "RightAnkle_end";

    config.SourceChains = {
        // name     start               end
        {"spine",   "Spine",            "Thorax"},
        {"lleg",    "LeftHip",          "LeftAnkle"},
        {"rleg",    "RightHip",         "RightAnkle"},
        {"larm",    "LeftShoulder",     "LeftWrist"},
        {"rram",    "RightShoulder",    "RightWrist"},
        {"head",    "Neck",             "Head"},
    };

    config.TargetChains = {
        // name     start               end
        {"spine",   "Spine",            "Thorax"},
        {"lleg",    "LeftHip",          "LeftAnkle"},
        {"rleg",    "RightHip",         "RightAnkle"},
        {"larm",    "LeftShoulder",     "LeftWrist"},
        {"rram",    "RightShoulder",    "RightWrist"},
        {"head",    "Neck",             "Head"},
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        {true,  false,  "spine",        "spine"},
        {true,  false,  "lleg",         "lleg"},
        {true,  false,  "rleg",         "rleg"},
        {true,  false,  "larm",         "larm"},
        {true,  false,  "rram",         "rram"},
        {true,  false,  "head",         "head"},
    };

    return config;
}

static SoulIKRigRetargetConfig config_to_meta() {
    SoulIKRigRetargetConfig config;
    config.SourceCoord      = CoordType::RightHandZupYfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandYupZfront;

    config.SourceRootBone   = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone   = "Rol01_Torso01HipCtrlJnt_M";
    config.TargetGroundBone = "Rol01_Leg01FootJnt_L";
    
    //config.skipRootBone = true;

    config.SourceChains = {
        // name     start               end
        {"spine",   "Spine",            "Thorax"},
        {"lleg",    "LeftHip",          "LeftAnkle"},
        {"rleg",    "RightHip",         "RightAnkle"},
        {"larm",    "LeftShoulder",     "LeftWrist"},
        {"rram",    "RightShoulder",    "RightWrist"},
        {"head",    "Neck",             "Head"},
    };

    config.TargetChains = {
        // name    start        end
        {"spine",   "Rol01_Torso0102Jnt_M",           "Rol01_Neck0101Jnt_M"},
        {"lleg",    "Rol01_Leg01Up01Jnt_L",          "Rol01_Leg01AnkleJnt_L"},
        {"rleg",    "Rol01_Leg01Up01Jnt_R",         "Rol01_Leg01AnkleJnt_R"},
        {"larm",    "Rol01_Arm01Up01Jnt_L",     "Rol01_Arm01Low03Jnt_L"},
        {"rram",    "Rol01_Arm01Up01Jnt_R",    "Rol01_Arm01Low03Jnt_R"},
        {"head",    "Rol01_Neck0102Jnt_M",             "Head_M"},
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        // {true,  false,  "spine",         "spine"},
        {true,  false,  "lleg",         "lleg"},
        // {true,  false,  "rleg",         "rleg"},
        // {true,  false,  "larm",         "larm"},
        // {true,  false,  "rram",         "rram"},
        // {true,  false,  "head",         "head"},
    };

    return config;
}

static std::string getModelPath() {
    std::string file_path = __FILE__;
    
    #ifdef _WIN64
        std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
        std::string model_path = dir_path + "\\..\\..\\model\\";
    #else
        std::string dir_path = file_path.substr(0, file_path.rfind("/"));
        std::string model_path = dir_path + "/../../model/";
    #endif

    return model_path;
}

void testMetaFBX() {
    std::string modelPath = getModelPath();
    std::string metaAFile = modelPath + "3D_Avatar2_Rig_0723.fbx";
    std::string metaTFile = modelPath + "3D_Avatar2_Rig_0723_tpose1.fbx";

    SoulIK::FBXRW fbxapose, fbxtpose;
    fbxapose.readSkeketonMesh(metaAFile);
    fbxtpose.readSkeketonMesh(metaTFile);


    SoulIK::SoulScene& ascene = *fbxapose.getSoulScene();
    SoulIK::SoulScene& tscene = *fbxtpose.getSoulScene();
    SoulIK::SoulSkeletonMesh& askm = *ascene.skmeshes[7];
    SoulIK::SoulSkeletonMesh& tskm = *tscene.skmeshes[7];


    IKRigUtils::debugPrintSkeletonTreeTransform(tscene, tskm);

    // // print node tree
    // IKRigUtils::debugPrintNodePose(ascene.rootNode.get());
    // IKRigUtils::debugPrintNodePose(tscene.rootNode.get());


    // IKRigUtils::debugPrintSkeletonTreeIBM(askm.skeleton);
    // IKRigUtils::debugPrintSkeletonTreeIBM(tskm.skeleton);

    // // save obj file
    // ObjRW objrw;
    // objrw.writeMesh(askm, modelPath + "a.obj");
    // objrw.writeMesh(tskm, modelPath + "t.obj");

    printf("done");

}

static void getFilePaths(std::string& inputfile, std::string& inputfile2, std::string& outfile, SoulIKRigRetargetConfig& config) {
    std::string modelPath = getModelPath();

    //std::string inputfile = modelPath + "3D_Avatar2_Rig_0723.fbx";
    //std::string outfile = modelPath + "3D_Avatar2_Rig_0723_out.fbx";
    // std::string inputfile = modelPath + "S1_SittingDown_3d_17kpts.fbx";
    // std::string outfile = modelPath + "S1_SittingDown_3d_17kpts_tiny_out.fbx";
    inputfile = modelPath + "S1_SittingDown_3d_17kpts_tiny_ball.fbx";
    if (config.TargetCoord == CoordType::RightHandZupYfront) {
        inputfile2 = modelPath + "S1_SittingDown_3d_17kpts_tiny_ball.fbx";
    }
    else if (config.TargetCoord == CoordType::RightHandYupZfront) {
        //inputfile2 = modelPath + "3D_Avatar2_Rig_0723_tpose3_hik.fbx";
        inputfile2 = modelPath + "3D_Avatar2_Rig_0723.fbx";
        //inputfile2 = modelPath + "T-Pose_02.fbx";
    }
    outfile = modelPath + "out.fbx";
}

int main(int argc, char *argv[]) {

    //testMetaFBX();
    //return 0;

    /////////////////////////////////////////////
    // setting of coord
    //auto config             =  config1_1chain_lleg();
    auto config             =  config_to_meta();
    //auto config             = config2_6chain();
    CoordType srccoord      = config.SourceCoord;
    CoordType workcoord     = config.WorkCoord;
    CoordType tgtcoord      = config.TargetCoord;
    FTransform tsrc2work    = IKRigUtils::getFTransformFromCoord(srccoord, workcoord);
    FTransform twork2tgt    = IKRigUtils::getFTransformFromCoord(workcoord, tgtcoord);
    FTransform ttgt2work    = IKRigUtils::getFTransformFromCoord(tgtcoord, workcoord);
    bool isModelAPose       = true;

    /////////////////////////////////////////////
    // read fbx
    std::string inputfile, inputfile2, outfile;
    SoulIK::FBXRW fbxrw, fbxrw2;
    getFilePaths(inputfile, inputfile2, outfile, config);
    fbxrw.readSkeketonMesh(inputfile);
    fbxrw2.readSkeketonMesh(inputfile2);

    SoulIK::SoulScene& srcscene = *fbxrw.getSoulScene();
    SoulIK::SoulScene& tgtscene = *fbxrw2.getSoulScene();
    SoulIK::SoulSkeletonMesh& srcskm = *srcscene.skmeshes[0];
    SoulIK::SoulSkeletonMesh& tgtskm = *tgtscene.skmeshes[0];

    // printf("skeleton tree1L\n");
    // IKRigUtils::debugPrintSkeletonTreeTransform(srcscene, srcskm);
    // printf("skeleton tree1G\n");
    // IKRigUtils::debugPrintSkeletonTreeGTransform(srcscene, srcskm);
    // printf("skeleton tree1L\n");

    SoulPose temppose1, temppose2;
    IKRigUtils::getSoulPoseFromMesh(tgtscene, tgtskm, temppose1);
    IKRigUtils::LocalSoulPoseCoordConvert(tgtcoord, workcoord, temppose1.transforms);
    printf("skeleton tree1L\n");
    IKRigUtils::debugPrintSoulPose(tgtskm.skeleton, temppose1.transforms);
    printf("skeleton tree1G\n");
    IKRigUtils::SoulPoseToGlobal(tgtskm.skeleton, temppose1.transforms, temppose2.transforms);
    IKRigUtils::debugPrintSoulPose(tgtskm.skeleton, temppose2.transforms);
    printf("skeleton tree1L\n");
    IKRigUtils::SoulPoseToLocal(tgtskm.skeleton, temppose2.transforms, temppose1.transforms);
    IKRigUtils::debugPrintSoulPose(tgtskm.skeleton, temppose1.transforms);

    // printf("skeleton tree2\n");
    // IKRigUtils::debugPrintSkeletonTreeTransform(tgtscene, tgtskm);
    // printf("skeleton tree2G\n");
    // IKRigUtils::debugPrintSkeletonTreeGTransform(tgtscene, tgtskm);

    //IKRigUtils::debugPrintNodePose(tgtscene.rootNode.get());

    /////////////////////////////////////////////
    // init
    SoulIK::USkeleton srcusk;
    SoulIK::USkeleton tgtusk;
    IKRigUtils::getUSkeletonFromMesh(srcscene, srcskm, srcusk, srccoord, workcoord);
    IKRigUtils::getUSkeletonFromMesh(tgtscene, tgtskm, tgtusk, tgtcoord, workcoord);
    if (isModelAPose) {
        // fix to tpose
        tgtusk.refpose = getMetaTPoseFPose(tgtskm.skeleton, CoordType::RightHandYupZfront, CoordType::RightHandZupYfront);
    }
    //srcusk.refpose[0].Rotation = FQuat(); // debug
    //tgtusk.refpose[0].Rotation = FQuat(); // debug


    // printf("uskeleton tree1G\n");
    // IKRigUtils::debugPrintUSkeletonTreeGTransform(srcskm.skeleton, srcusk);
    // printf("uskeleton tree2G\n");
    // IKRigUtils::debugPrintUSkeletonTreeGTransform(tgtskm.skeleton, tgtusk);
    
    std::vector<FTransform> tempfpose1, tempfpose2;
    printf("uskeleton tree1L\n");
    tempfpose1 = tgtusk.refpose;
    IKRigUtils::debugPrintFPose(tgtskm.skeleton, tempfpose1);
    printf("uskeleton tree1G\n");
    IKRigUtils::FPoseToGlobal(tgtskm.skeleton, tempfpose1, tempfpose2);
    IKRigUtils::debugPrintFPose(tgtskm.skeleton, tempfpose2);
    // printf("uskeleton tree1L\n");
    // IKRigUtils::FPoseToLocal(srcskm.skeleton, tempfpose2, tempfpose1);
    // IKRigUtils::debugPrintFPose(srcskm.skeleton, tempfpose1);


    SoulIK::UIKRetargetProcessor ikretarget;
	std::shared_ptr<UIKRetargeter> InRetargeterAsset = createIKRigAsset(config, srcskm.skeleton, tgtskm.skeleton, srcusk, tgtusk);
    ikretarget.Initialize(&srcusk, &tgtusk, InRetargeterAsset.get(), false); // todo: raw pointer not safe
    
    /////////////////////////////////////////////
    // build pose animation form mesh0
    std::vector<SoulIK::SoulPose> tempposes;
    std::vector<SoulIK::SoulPose> tempoutposes;
    buildPoseAnimationByInterpolation(srcscene, srcskm, srcskm.animation, tempposes);
    int frameCount = tempposes.size();
    tempoutposes.resize(tempposes.size());

    /////////////////////////////////////////////
    // run retarget
    std::unordered_map<FName, float> SpeedValuesFromCurves;
    float DeltaTime = 0;

    std::vector<FTransform> initposeLocal = tgtusk.refpose;
    std::vector<FTransform> initposeGlobal;
    IKRigUtils::FPoseToGlobal(tgtskm.skeleton, initposeLocal, initposeGlobal);

    std::vector<FTransform> inpose;
    std::vector<FTransform> inposeLocal;
    std::vector<FTransform> outposeLocal;
    for(int frame = 0; frame < tempposes.size(); frame++) {

        //printf("skeleton tree1L: %d\n", frame);
        //IKRigUtils::debugPrintSoulPose(srcskm.skeleton, tempposes[frame].transforms);
        if(frame < 2) {
            printf("source skeleton tree1G: %d\n", frame);
            SoulPose tempposeG;
            IKRigUtils::SoulPoseToGlobal(srcskm.skeleton, tempposes[frame].transforms, tempposeG.transforms);
            IKRigUtils::debugPrintSoulPose(srcskm.skeleton, tempposeG.transforms);
        }
        
        //if (frame == 0) { inposeLocal = tgtusk.refpose;}
        // printf("%d: t(%f %f %f) t(%f %f %f) t(%f %f %f) %f %f %f\n", frame, 
        //     inpose[1].Translation.x, inpose[1].Translation.y, inpose[1].Translation.z,
        //     inpose[2].Translation.x, inpose[2].Translation.y, inpose[2].Translation.z,
        //     inpose[3].Translation.x, inpose[3].Translation.y, inpose[3].Translation.z,
        //     inpose[1].Rotation.getAngleDegree(), inpose[2].Rotation.getAngleDegree(), inpose[3].Rotation.getAngleDegree());

        // input and cast
        IKRigUtils::SoulPose2FPose(tempposes[frame], inposeLocal);

        //inposeLocal[0].Rotation = FQuat(); // debug


        // retarget
        IKRigUtils::LocalFPoseCoordConvert(tsrc2work, srccoord, workcoord, inposeLocal);
        IKRigUtils::FPoseToGlobal(srcskm.skeleton, inposeLocal, inpose);
        //printf("frame:%d\n", frame);
        //IKRigUtils::debugPrintPoseJoints("initpose", initposeGlobal, {1, 2});
        //IKRigUtils::debugPrintPoseJoints("inpose", inpose, {1, 2});
        std::vector<FTransform>& outpose = ikretarget.RunRetargeter(inpose, SpeedValuesFromCurves, DeltaTime);
        //IKRigUtils::debugPrintPoseJoints("outpose", outpose, {1, 2});
        IKRigUtils::FPoseToLocal(tgtskm.skeleton, outpose, outposeLocal);
        // if (frame == 0) { outposeLocal = tgtusk.refpose;}
        // outposeLocal = tgtusk.refpose; // debug

         if(frame < 2) {
            printf("target skeleton tree1G: %d\n", frame);
            std::vector<FTransform> tempposeG;
            IKRigUtils::FPoseToGlobal(tgtskm.skeleton, outposeLocal, tempposeG);
            IKRigUtils::debugPrintFPose(tgtskm.skeleton, tempposeG);
        }

        IKRigUtils::LocalFPoseCoordConvert(twork2tgt, workcoord, tgtcoord, outposeLocal);

        // cast and output
        IKRigUtils::FPose2SoulPose(outposeLocal, tempoutposes[frame]);


        // test
        //tempoutposes[frame].transforms[0].translation.x += frame;
        // test init fpose
        // std::vector<FTransform> initposeLocal2 = tgtusk.refpose;
        // IKRigUtils::LocalPoseCoordConvert(twork2tgt, initposeLocal2, workcoord, tgtcoord);
        // IKRigUtils::FPose2SoulPose(initposeLocal2, tempoutposes[frame]);
        
        // test init soulpose
        //SoulIK::SoulPose tgtsoulpose;
        //IKRigUtils::getSoulPoseFromMesh(tgtscene, tgtskm, tgtsoulpose);
        //tempoutposes[frame] = tgtsoulpose;

        // test inpose
        //tempoutposes[frame] = tempposes[frame];
    }

    /////////////////////////////////////////////
    // output pose animation to mesh0
    writePoseAnimationToMesh(tempoutposes, tgtskm, srcskm.animation.duration, srcskm.animation.ticksPerSecond);
    fbxrw2.writeSkeletonMesh(outfile);

    return 0;
}
