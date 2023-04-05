//
//  main.cpp
//  test libGLVNDGLContext
//
//  Created by kai chen on 2/8/23.
//

#include <stdio.h>
#include "SoulTransform.h"
#include "SoulRetargeter.h"
#include "SoulIKRetargetProcessor.h"

#include "FBXRW.h"
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

static std::shared_ptr<UIKRetargeter> createIKRigAsset(IKRigRetargetConfig& config,
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

static IKRigRetargetConfig get_config1_1chain_lleg() {
    IKRigRetargetConfig config;
    config.SourceCoord = CoordType::RightHandZupYfront;
    config.WorkCoord = CoordType::RightHandZupYfront;
    config.TargetCoord = CoordType::RightHandZupYfront;

    config.SourceRootBone = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone = "Hip";
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
}

// static std::shared_ptr<UIKRetargeter> buildIKRigRetargetAsset_config1_1chain_lleg(SoulIK::SoulSkeleton& srcsk, SoulIK::SoulSkeleton& tgtsk, SoulIK::USkeleton& srcusk, SoulIK::USkeleton& tgtusk) {

//     std::shared_ptr<UIKRetargeter> InRetargeterAsset = std::make_shared<UIKRetargeter>();

//     // ikrig1 asset
//     InRetargeterAsset->SourceIKRigAsset = std::make_shared<UIKRigDefinition>();
//     FIKRigSkeleton rigsk;
//     IKRigUtils::USkeleton2RigSkeleton(srcusk, rigsk);
//     InRetargeterAsset->SourceIKRigAsset->Skeleton = rigsk;
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.RootBone = "Hip";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.GroundBone = "RightAnkle_end";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains.resize(1);
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightHip";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("RightHip");
//     //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightKnee";
//     //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("RightKnee");
//     //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightAnkle";
//     //InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("RightAnkle");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightKnee";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("RightKnee");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "lleg";

//     // ikrig2 asset
//     InRetargeterAsset->TargetIKRigAsset = std::make_shared<UIKRigDefinition>();
//     FIKRigSkeleton rigsk2;
//     IKRigUtils::USkeleton2RigSkeleton(tgtusk, rigsk2);
//     InRetargeterAsset->TargetIKRigAsset->Skeleton = rigsk2;
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.RootBone = "Hip";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.GroundBone = "RightAnkle_end";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains.resize(1);
//     //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightKnee";
//     //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = tgtsk.getJointIdByName("RightKnee");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "RightHip";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = tgtsk.getJointIdByName("RightHip");
//     //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightAnkle";
//     //InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = tgtsk.getJointIdByName("RightAnkle");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "RightKnee";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = tgtsk.getJointIdByName("RightKnee");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "lleg";

//     // ik rig retarget asset
//     std::shared_ptr<URetargetChainSettings> chainSetting = std::make_shared<URetargetChainSettings>();
//     chainSetting->Settings.FK.EnableFK = true;
//     chainSetting->Settings.IK.EnableIK = false;
//     chainSetting->SourceChain = "lleg";
//     chainSetting->TargetChain = "lleg";
//     InRetargeterAsset->ChainSettings.push_back(chainSetting);

//     return InRetargeterAsset;
// }


static IKRigRetargetConfig config2_6chain() {

    IKRigRetargetConfig config;
    config.SourceCoord = CoordType::RightHandZupYfront;
    config.WorkCoord = CoordType::RightHandZupYfront;
    config.TargetCoord = CoordType::RightHandZupYfront;

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
        {true,  false,  "spine",         "spine"},
        {true,  false,  "lleg",         "lleg"},
        {true,  false,  "rleg",         "rleg"},
        {true,  false,  "larm",         "larm"},
        {true,  false,  "rram",         "rram"},
        {true,  false,  "head",         "head"},
    };
}

static std::shared_ptr<UIKRetargeter> buildIKRigRetargetAsset_config2_6chain(SoulIK::SoulSkeleton& srcsk, SoulIK::SoulSkeleton& tgtsk, SoulIK::USkeleton& srcusk, SoulIK::USkeleton& tgtusk) {

    std::shared_ptr<UIKRetargeter> InRetargeterAsset = std::make_shared<UIKRetargeter>();

    // ikrig1 asset
    InRetargeterAsset->SourceIKRigAsset = std::make_shared<UIKRigDefinition>();
    FIKRigSkeleton rigsk;
    IKRigUtils::USkeleton2RigSkeleton(srcusk, rigsk);
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
    IKRigUtils::USkeleton2RigSkeleton(tgtusk, rigsk2);
    InRetargeterAsset->TargetIKRigAsset->Skeleton = rigsk2;
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

static IKRigRetargetConfig config_to_meta() {
    IKRigRetargetConfig config;
    config.SourceCoord = CoordType::RightHandZupYfront;
    config.WorkCoord = CoordType::RightHandZupYfront;
    config.TargetCoord = CoordType::RightHandYupZfront;

    config.SourceRootBone = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone = "Rol01_Torso01HipCtrlJnt_M";
    config.TargetGroundBone = "Rol01_Leg01FootJnt_L";

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
        {"spine",   "Rol01_Torso01HipCtrlJnt_M",            "Rol01_Neck0101Jnt_M"},
        {"lleg",    "Rol01_Leg01Up01Jnt_L",          "Rol01_Leg01AnkleJnt_L"},
        {"rleg",    "Rol01_Leg01Up01Jnt_R",         "Rol01_Leg01AnkleJnt_R"},
        {"larm",    "Rol01_Arm01Up01Jnt_L",     "Rol01_Arm01Low03Jnt_L"},
        {"rram",    "Rol01_Arm01Up01Jnt_R",    "Rol01_Arm01Low03Jnt_R"},
        {"head",    "Rol01_Neck0102Jnt_M",             "Head_M"},
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        {true,  false,  "spine",         "spine"},
        {true,  false,  "lleg",         "lleg"},
        {true,  false,  "rleg",         "rleg"},
        {true,  false,  "larm",         "larm"},
        {true,  false,  "rram",         "rram"},
        {true,  false,  "head",         "head"},
    };
}

// static std::shared_ptr<UIKRetargeter> buildIKRigRetargetAsset_configc_meta(SoulIK::SoulSkeleton& srcsk, SoulIK::SoulSkeleton& tgtsk, SoulIK::USkeleton& srcusk, SoulIK::USkeleton& tgtusk) {

//     std::shared_ptr<UIKRetargeter> InRetargeterAsset = std::make_shared<UIKRetargeter>();

//     // ikrig1 asset
//     InRetargeterAsset->SourceIKRigAsset = std::make_shared<UIKRigDefinition>();
//     FIKRigSkeleton rigsk;
//     IKRigUtils::USkeleton2RigSkeleton(srcusk, rigsk);
//     InRetargeterAsset->SourceIKRigAsset->Skeleton = rigsk;
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.RootBone = "Hip";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.GroundBone = "RightAnkle_end";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains.resize(6);

//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "Spine";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("Spine");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "Thorax";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("Thorax");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "spine";

//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneName = "LeftHip";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneIndex = srcsk.getJointIdByName("LeftHip");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneName = "LeftAnkle";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneIndex = srcsk.getJointIdByName("LeftAnkle");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[1].ChainName = "lleg";

//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneName = "RightHip";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneIndex = srcsk.getJointIdByName("RightHip");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneName = "RightAnkle";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneIndex = srcsk.getJointIdByName("RightAnkle");;
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[2].ChainName = "rleg";


//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneName = "LeftShoulder";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneIndex = srcsk.getJointIdByName("LeftShoulder");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneName = "LeftWrist";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneIndex = srcsk.getJointIdByName("LeftWrist");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[3].ChainName = "larm";

//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneName = "RightShoulder";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneIndex = srcsk.getJointIdByName("RightShoulder");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneName = "RightWrist";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneIndex = srcsk.getJointIdByName("RightWrist");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[4].ChainName = "rarm";

//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneName = "Neck";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneIndex = srcsk.getJointIdByName("Neck");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneName = "Head";
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneIndex = srcsk.getJointIdByName("Head");
//     InRetargeterAsset->SourceIKRigAsset->RetargetDefinition.BoneChains[5].ChainName = "head";

//     // ikrig2 asset
//     InRetargeterAsset->TargetIKRigAsset = std::make_shared<UIKRigDefinition>();
//     FIKRigSkeleton rigsk2;
//     IKRigUtils::USkeleton2RigSkeleton(tgtusk, rigsk2);
//     InRetargeterAsset->TargetIKRigAsset->Skeleton = rigsk2;
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.RootBone = "Rol01_Torso01HipCtrlJnt_M";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.GroundBone = "Rol01_Leg01FootJnt_L";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains.resize(6);

//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneName = "Rol01_Torso01HipCtrlJnt_M";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].StartBone.BoneIndex = srcsk.getJointIdByName("Rol01_Torso01HipCtrlJnt_M");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneName = "Rol01_Neck0101Jnt_M";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].EndBone.BoneIndex = srcsk.getJointIdByName("Rol01_Neck0101Jnt_M");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[0].ChainName = "spine";

//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneName = "Rol01_Leg01Up01Jnt_L";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].StartBone.BoneIndex = srcsk.getJointIdByName("Rol01_Leg01Up01Jnt_L");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneName = "Rol01_Leg01AnkleJnt_L";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].EndBone.BoneIndex = srcsk.getJointIdByName("Rol01_Leg01AnkleJnt_L");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[1].ChainName = "lleg";

//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneName = "Rol01_Leg01Up01Jnt_R";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].StartBone.BoneIndex = srcsk.getJointIdByName("Rol01_Leg01Up01Jnt_R");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneName = "Rol01_Leg01AnkleJnt_R";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].EndBone.BoneIndex = srcsk.getJointIdByName("Rol01_Leg01AnkleJnt_R");;
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[2].ChainName = "rleg";

    
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneName = "Rol01_Arm01Up01Jnt_L";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].StartBone.BoneIndex = srcsk.getJointIdByName("Rol01_Arm01Up01Jnt_L");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneName = "Rol01_Arm01Low03Jnt_L";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].EndBone.BoneIndex = srcsk.getJointIdByName("Rol01_Arm01Low03Jnt_L");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[3].ChainName = "larm";

//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneName = "Rol01_Arm01Up01Jnt_R";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].StartBone.BoneIndex = srcsk.getJointIdByName("Rol01_Arm01Up01Jnt_R");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneName = "Rol01_Arm01Low03Jnt_R";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].EndBone.BoneIndex = srcsk.getJointIdByName("Rol01_Arm01Low03Jnt_R");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[4].ChainName = "rarm";

//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneName = "Rol01_Neck0102Jnt_M";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].StartBone.BoneIndex = srcsk.getJointIdByName("Rol01_Neck0102Jnt_M");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneName = "Head_M";
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].EndBone.BoneIndex = srcsk.getJointIdByName("Head_M");
//     InRetargeterAsset->TargetIKRigAsset->RetargetDefinition.BoneChains[5].ChainName = "head";

//     // ik rig retarget asset
//     std::shared_ptr<URetargetChainSettings> chainSetting0 = std::make_shared<URetargetChainSettings>();
//     chainSetting0->Settings.FK.EnableFK = true;
//     chainSetting0->Settings.IK.EnableIK = false;
//     chainSetting0->SourceChain = "spine";
//     chainSetting0->TargetChain = "spine";
//     InRetargeterAsset->ChainSettings.push_back(chainSetting0);

//     std::shared_ptr<URetargetChainSettings> chainSetting1 = std::make_shared<URetargetChainSettings>();
//     chainSetting1->Settings.FK.EnableFK = true;
//     chainSetting1->Settings.IK.EnableIK = false;
//     chainSetting1->SourceChain = "lleg";
//     chainSetting1->TargetChain = "lleg";
//     InRetargeterAsset->ChainSettings.push_back(chainSetting1);

//     std::shared_ptr<URetargetChainSettings> chainSetting2 = std::make_shared<URetargetChainSettings>();
//     chainSetting2->Settings.FK.EnableFK = true;
//     chainSetting2->Settings.IK.EnableIK = false;
//     chainSetting2->SourceChain = "rleg";
//     chainSetting2->TargetChain = "rleg";
//     InRetargeterAsset->ChainSettings.push_back(chainSetting2);

//     std::shared_ptr<URetargetChainSettings> chainSetting3 = std::make_shared<URetargetChainSettings>();
//     chainSetting3->Settings.FK.EnableFK = true;
//     chainSetting3->Settings.IK.EnableIK = false;
//     chainSetting3->SourceChain = "larm";
//     chainSetting3->TargetChain = "larm";
//     InRetargeterAsset->ChainSettings.push_back(chainSetting3);

//     std::shared_ptr<URetargetChainSettings> chainSetting4 = std::make_shared<URetargetChainSettings>();
//     chainSetting4->Settings.FK.EnableFK = true;
//     chainSetting4->Settings.IK.EnableIK = false;
//     chainSetting4->SourceChain = "rarm";
//     chainSetting4->TargetChain = "rarm";
//     InRetargeterAsset->ChainSettings.push_back(chainSetting4);

//     std::shared_ptr<URetargetChainSettings> chainSetting5 = std::make_shared<URetargetChainSettings>();
//     chainSetting5->Settings.FK.EnableFK = true;
//     chainSetting5->Settings.IK.EnableIK = false;
//     chainSetting5->SourceChain = "head";
//     chainSetting5->TargetChain = "head";
//     InRetargeterAsset->ChainSettings.push_back(chainSetting5);

//     return InRetargeterAsset;
// }

static void getFilePaths(std::string& inputfile, std::string& inputfile2, std::string& outfile) {
    std::string file_path = __FILE__;
#ifdef _WIN64
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    //std::string inputfile = dir_path + "\\..\\..\\model\\3D_Avatar2_Rig_0723.fbx";
    //std::string outfile = dir_path + "\\..\\..\\model\\3D_Avatar2_Rig_0723_out.fbx";
    // std::string inputfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts.fbx";
    // std::string outfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts_tiny_out.fbx";
    inputfile = dir_path + "\\..\\..\\model\\S1_SittingDown_3d_17kpts_tiny_ball.fbx";
    inputfile2 = dir_path + "\\..\\..\\model\\3D_Avatar2_Rig_0723.fbx";
    outfile = dir_path + "\\..\\..\\model\\out.fbx";
    
#else
    std::string dir_path = file_path.substr(0, file_path.rfind("/"));
    inputfile = dir_path + "/../../model/S1_SittingDown_3d_17kpts_tiny_ball.fbx";
    inputfile2 = dir_path + "/../../model/3D_Avatar2_Rig_0723.fbx";
    outfile = dir_path + "/../../model/out.fbx";
#endif
}

int main(int argc, char *argv[]) {

    /////////////////////////////////////////////
    // setting of coord
    CoordType srccoord = CoordType::RightHandZupYfront;
    CoordType workcoord = CoordType::RightHandZupYfront;
    CoordType tgtcoord = CoordType::RightHandYupZfront;
    FTransform tsrc2work = IKRigUtils::getTransformFromCoord(srccoord, workcoord);
    FTransform twork2tgt = IKRigUtils::getTransformFromCoord(workcoord, tgtcoord);

    /////////////////////////////////////////////
    // read fbx
    std::string inputfile, inputfile2, outfile;
    SoulIK::FBXRW fbxrw, fbxrw2;
    getFilePaths(inputfile, inputfile2, outfile);
    fbxrw.readSkeketonMesh(inputfile);
    fbxrw2.readSkeketonMesh(inputfile2);

    SoulIK::SoulScene& srcscene = *fbxrw.getSoulScene();
    SoulIK::SoulScene& tgtscene = *fbxrw2.getSoulScene();
    SoulIK::SoulSkeletonMesh& srcskm = *srcscene.skmeshes[0];
    SoulIK::SoulSkeletonMesh& tgtskm = *tgtscene.skmeshes[0];

    // IKRigUtils::debugPrintSkeletonTree(srcskm.skeleton);
    // IKRigUtils::debugPrintNodePose(srcscene.rootNode.get());
    // IKRigUtils::debugPrintSkeletonTree(tgtskm.skeleton);
    // IKRigUtils::debugPrintNodePose(tgtscene.rootNode.get());
    //return 0;

    /////////////////////////////////////////////
    // init
    SoulIK::USkeleton srcusk;
    SoulIK::USkeleton tgtusk;
    IKRigUtils::getUSkeletonFromMesh(srcscene, srcskm, srcusk, srccoord, workcoord);
    IKRigUtils::getUSkeletonFromMesh(tgtscene, tgtskm, tgtusk, tgtcoord, workcoord);
    //IKRigUtils::debugPrintLocalFPose(tgtskm.skeleton, tgtusk.refpose);

    SoulIK::UIKRetargetProcessor ikretarget;
	std::shared_ptr<UIKRetargeter> InRetargeterAsset = buildIKRigRetargetAsset_configc_meta(srcskm.skeleton, tgtskm.skeleton, srcusk, tgtusk);
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

    std::vector<FTransform> inpose;
    std::vector<FTransform> inposeLocal;
    std::vector<FTransform> outposeLocal;
    for(int frame = 0; frame < tempposes.size(); frame++) {
        
        //if (frame == 0) { inposeLocal = tgtusk.refpose;}

        // printf("%d: t(%f %f %f) t(%f %f %f) t(%f %f %f) %f %f %f\n", frame, 
        //     inpose[1].Translation.x, inpose[1].Translation.y, inpose[1].Translation.z,
        //     inpose[2].Translation.x, inpose[2].Translation.y, inpose[2].Translation.z,
        //     inpose[3].Translation.x, inpose[3].Translation.y, inpose[3].Translation.z,
        //     inpose[1].Rotation.getAngleDegree(), inpose[2].Rotation.getAngleDegree(), inpose[3].Rotation.getAngleDegree());

        // input and cast
        IKRigUtils::SoulPose2FPose(tempposes[frame], inposeLocal);

        // retarget
        IKRigUtils::LocalPoseCoordConvert(tsrc2work, inposeLocal, srccoord, workcoord);
        IKRigUtils::FPoseToGlobal(srcskm.skeleton, inposeLocal, inpose);
        std::vector<FTransform>& outpose = ikretarget.RunRetargeter(inpose, SpeedValuesFromCurves, DeltaTime);
        IKRigUtils::FPoseToLocal(tgtskm.skeleton, outpose, outposeLocal);
        // if (frame == 0) { outposeLocal = tgtusk.refpose;}
        outposeLocal = tgtusk.refpose;
        IKRigUtils::LocalPoseCoordConvert(twork2tgt, outposeLocal, workcoord, tgtcoord);

        // cast and output
        IKRigUtils::FPose2SoulPose(outposeLocal, tempoutposes[frame]);

        // test
        SoulIK::SoulPose tgtsoulpose;
        IKRigUtils::getSoulPoseFromMesh(tgtscene, tgtskm, tgtsoulpose);
        tempoutposes[frame] = tgtsoulpose;
    }

    /////////////////////////////////////////////
    // output pose animation to mesh0
    writePoseAnimationToMesh(tempoutposes, tgtskm, srcskm.animation.duration, srcskm.animation.ticksPerSecond);
    fbxrw2.writeSkeletonMesh(outfile);

    return 0;
}
