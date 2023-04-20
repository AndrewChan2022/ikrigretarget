//
//  ikrigretargetapi.cpp
//
//
//  Created by kai chen on 3/24/23.
//

#include "ikrigretargetapi.hpp"

#include <stdio.h>

#include "SoulScene.hpp"
#include "SoulRetargeter.h"
#include "IKRigUtils.hpp"
#include "SoulIKRetargetProcessor.h"

#include "FBXRW.h"
#include "ObjRW.h"
#include "InitPoseConvert.h"

using namespace SoulIK;


int myadd(int a, int b) {
    return a + b;
}

// #define DEBUG_POSE_PRINT
// #define DEBUG_POSE_PRINT_EVERY_FRAME

#ifdef DEBUG_POSE_PRINT
    #define DEBUG_PRINT_SKM(name, scene, skm, srccoord, workcoord) IKRigUtils::debugPrintSKM(name, scene, *scene.skmeshes[0], srccoord, workcoord);
    #define DEBUG_PRINT_USK(name, usk, skm, srccoord, workcoord) IKRigUtils::debugPrintUSK(name, usk, skm, srccoord, workcoord);
#else
    #define DEBUG_PRINT_SKM(name, scene, skm, srccoord, workcoord)
    #define DEBUG_PRINT_USK(name, usk, skm, srccoord, workcoord)
#endif

#if defined(DEBUG_POSE_PRINT) &&  defined(DEBUG_POSE_PRINT_EVERY_FRAME)
    //#define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
    #define DEBUG_PRINT(...) printf(__VA_ARGS__)
    #define DEBUG_PRINT_IO_FPOSE(name, skm, poselocal, poseglobal, initInPoseLocal, frame) IKRigUtils::debugPrintIOFPose(name, skm, poselocal, poseglobal, initInPoseLocal, frame)
    #define DEBUG_PRINT_IO_SOULPOSE(name, poselocal, skm, frame) IKRigUtils::debugPrintIOSoulPose(name, poselocal, skm, frame)
#else
    #define DEBUG_PRINT(fmt, ...)
    #define DEBUG_PRINT_IO_FPOSE(name, skm, poselocal, poseglobal, initInPoseLocal, frame)
    #define DEBUG_PRINT_IO_SOULPOSE(name, poselocal, skm, frame)
#endif

// generate pose of every joint and every frame
static void buildPoseAnimationByInterpolation(SoulScene& scene, SoulSkeletonMesh& skmesh,
                        SoulJointAnimation& animation, std::vector<SoulIK::SoulPose>& poses) {

    std::vector<SoulIK::SoulTransform> refpose = IKRigUtils::getSoulPoseTransformFromMesh(scene, skmesh);

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
        double prevTime = 0;
        double curTime = 0;
        int prevFrame = 0;
        int curFrame = 0;
        int frameCount = static_cast<int>(skmesh.animation.duration + 1);
        
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

            prevTime = 0.0;
            prevFrame = 0;
            glm::vec3 prevValue = SparseKeys[0].value;
            glm::vec3 curValue;
            for (int j = 0; j < SparseKeys.size(); j++) {
                curTime = SparseKeys[j].time;
                curFrame = static_cast<int>(round(curTime));
                curValue = SparseKeys[j].value;
                for(int frame = prevFrame; frame <= curFrame; frame++) {
                    DenseKeys[frame].time = frame;
                    float alpha = (curTime - prevTime) < 1e-4 ? 1.0 : (frame - prevFrame) / (curTime - prevTime);
                    DenseKeys[frame].value = glm::mix(prevValue, curValue, alpha);
                }

                // next iteration
                prevTime = curTime;
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

            prevTime = 0.0;
            prevFrame = 0;
            glm::vec3 prevValue = SparseKeys[0].value;
            glm::vec3 curValue;
            for (int j = 0; j < SparseKeys.size(); j++) {
                curTime = SparseKeys[j].time;
                curFrame = static_cast<int>(round(curTime));
                curValue = SparseKeys[j].value;
                for(int frame = prevFrame; frame <= curFrame; frame++) {
                    DenseKeys[frame].time = frame;
                    float alpha = (curTime - prevTime) < 1e-4 ? 1.0 : (frame - prevFrame) / (curTime - prevTime);
                    DenseKeys[frame].value = glm::mix(prevValue, curValue, alpha);
                }

                // next iteration
                prevTime = curTime;
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

            prevTime = 0.0;
            prevFrame = 0;
            glm::quat prevValue = SparseKeys[0].value;
            glm::quat curValue;
            for (int j = 0; j < SparseKeys.size(); j++) {
                curTime = SparseKeys[j].time;
                curFrame = static_cast<int>(round(curTime));
                curValue = SparseKeys[j].value;
                for(int frame = prevFrame; frame <= curFrame; frame++) {
                    DenseKeys[frame].time = frame;
                    float alpha = (curTime - prevTime) < 1e-4 ? 1.0 : (frame - prevFrame) / (curTime - prevTime);
                    DenseKeys[frame].value = glm::mix(prevValue, curValue, alpha);
                    //alpha = glm::clamp(alpha, 0.0f, 1.0f);
                    //DenseKeys[frame].value = glm::lerp(prevValue, curValue, alpha);
                    DenseKeys[frame].value = glm::normalize(DenseKeys[frame].value);
                }

                // next iteration
                prevTime = curTime;
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
    poses.resize(static_cast<int>(skmesh.animation.duration));
    int frameCount = static_cast<int>(skmesh.animation.duration);
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
            tgtskm.animation.channels[jointId].PositionKeys[frame].time = static_cast<double>(frame);
            tgtskm.animation.channels[jointId].PositionKeys[frame].value =  tempoutposes[frame].transforms[jointId].translation;
        }

        tgtskm.animation.channels[jointId].ScalingKeys.resize(frameCount);
        for(uint64_t frame = 0; frame < tempoutposes.size(); frame++) {
            tgtskm.animation.channels[jointId].ScalingKeys[frame].time = static_cast<double>(frame);
            tgtskm.animation.channels[jointId].ScalingKeys[frame].value =  tempoutposes[frame].transforms[jointId].scale;
        }

        tgtskm.animation.channels[jointId].RotationKeys.resize(frameCount);
        for(uint64_t frame = 0; frame < tempoutposes.size(); frame++) {
            tgtskm.animation.channels[jointId].RotationKeys[frame].time = static_cast<double>(frame);
            tgtskm.animation.channels[jointId].RotationKeys[frame].value =  tempoutposes[frame].transforms[jointId].rotation;
        }        
    }
}

static std::vector<FTransform> getMetaTPoseFPose(SoulSkeleton& sk, CoordType srcCoord, CoordType tgtCoord) {

    std::vector<FTransform> pose(sk.joints.size());

    auto soulpose = SoulIK::getMetaTPoseSoulPose(sk);
    IKRigUtils::SoulPose2FPose(soulpose, pose);
    IKRigUtils::LocalFPoseCoordConvert(srcCoord, tgtCoord, pose);

    return pose;
}

// RVO: https://stackoverflow.com/a/10479595/2482283
// static bool configFromName(std::string const& name, SoulIKRigRetargetConfig& config) {
//     typedef SoulIKRigRetargetConfig (*FuncTypeGetConfig)(); // function pointer type
//     std::unordered_map<std::string, FuncTypeGetConfig> configTable = {
//         {"s1_meta", config_s1_meta},
//         {"flair_meta", config_flair_meta}
//     };
//     if (auto it = configTable.find(name); it != configTable.end()) {
//         config = (*it->second)();
//     } else {
//         return false;
//     }
//     return true;
// }


bool retargetFBX(std::string const& srcAnimationFile,
    std::string const& srcTPoseFile,
    std::string const& rootName,
    std::string const& targetFile,
    std::string const& targetTPoseFile,
    std::string const& outfile,
    SoulIKRigRetargetConfig& config) {

    /////////////////////////////////////////////
    // setting of coord
    CoordType srccoord      = config.SourceCoord;
    CoordType workcoord     = config.WorkCoord;
    CoordType tgtcoord      = config.TargetCoord;
    FTransform tsrc2work    = IKRigUtils::getFTransformFromCoord(srccoord, workcoord);
    FTransform twork2tgt    = IKRigUtils::getFTransformFromCoord(workcoord, tgtcoord);
    FTransform ttgt2work    = IKRigUtils::getFTransformFromCoord(tgtcoord, workcoord);

    /////////////////////////////////////////////
    // read fbx
    SoulIK::FBXRW fbxSrcAnimation, fbxSrcTPose, fbxTarget, fbxTargetTPose;
    fbxSrcAnimation.readPureSkeletonWithDefualtMesh(srcAnimationFile, config.SourceRootBone);
    if(srcAnimationFile == srcTPoseFile) {
        fbxSrcTPose = fbxSrcAnimation;
    } else {
        fbxSrcTPose.readPureSkeletonWithDefualtMesh(srcTPoseFile, config.SourceRootBone);
    }
    fbxTarget.readSkeletonMesh(targetFile);
    if (targetFile == targetTPoseFile) {
        fbxTargetTPose = fbxTarget;
    } else {
        fbxTargetTPose.readSkeletonMesh(targetTPoseFile);
    }

    SoulIK::SoulScene& srcscene         = *fbxSrcAnimation.getSoulScene();
    SoulIK::SoulScene& srcTPoseScene    = *fbxSrcTPose.getSoulScene();
    SoulIK::SoulScene& tgtscene         = *fbxTarget.getSoulScene();
    SoulIK::SoulScene& tgtTPosescene    = *fbxTargetTPose.getSoulScene();
    SoulIK::SoulSkeletonMesh& srcskm    = *srcscene.skmeshes[0];
    SoulIK::SoulSkeletonMesh& tgtskm    = *tgtscene.skmeshes[0];

    DEBUG_PRINT_SKM("SrcSoulPose", srcTPoseScene, *srcTPoseScene.skmeshes[0], srccoord, workcoord);
    DEBUG_PRINT_SKM("TgtSoulPose", tgtscene, tgtskm, tgtcoord, workcoord);

    /////////////////////////////////////////////
    // init
    SoulIK::USkeleton srcusk;
    SoulIK::USkeleton tgtusk;
    IKRigUtils::getUSkeletonFromMesh(srcTPoseScene, *srcTPoseScene.skmeshes[0], srcusk, srccoord, workcoord);
    IKRigUtils::alignUSKWithSkeleton(srcusk, srcskm.skeleton, srcTPoseScene, srcscene);
    IKRigUtils::getUSkeletonFromMesh(tgtTPosescene, *tgtTPosescene.skmeshes[0], tgtusk, tgtcoord, workcoord);
    IKRigUtils::alignUSKWithSkeleton(tgtusk, tgtskm.skeleton, tgtTPosescene, tgtscene); //IKRigUtils::debugPrintUSKNames(tgtusk);    
    //if (testCase.isTargetNeedHardCodeTPose) {
    //tgtusk.refpose = getMetaTPoseFPose(tgtskm.skeleton, CoordType::RightHandYupZfront, CoordType::RightHandZupYfront);
    //}

    DEBUG_PRINT_USK("SrcUSK", srcusk, srcskm, srccoord, workcoord);
    DEBUG_PRINT_USK("TgtUSK", tgtusk, tgtskm, tgtcoord, workcoord);

    SoulIK::UIKRetargetProcessor ikretarget;
	std::shared_ptr<UIKRetargeter> InRetargeterAsset = IKRigUtils::createIKRigAsset(config, srcskm.skeleton, tgtskm.skeleton, srcusk, tgtusk);
    ikretarget.Initialize(&srcusk, &tgtusk, InRetargeterAsset.get(), false); // todo: raw pointer not safe
    
    /////////////////////////////////////////////
    // build pose animation form mesh0
    std::vector<SoulIK::SoulPose> tempposes;
    std::vector<SoulIK::SoulPose> tempoutposes;
    buildPoseAnimationByInterpolation(srcscene, srcskm, srcskm.animation, tempposes);
    int frameCount = static_cast<int>(tempposes.size());
    // {
    //     frameCount = 24; // debug
    //     tempposes.resize(frameCount);
    //     for (size_t i = 0; i < frameCount; i++) {
    //         IKRigUtils::getSoulPoseFromMesh(srcscene, srcskm, tempposes[i]);
    //     }
    // }
    tempoutposes.resize(frameCount);

    /////////////////////////////////////////////
    // run retarget
    std::unordered_map<FName, float> SpeedValuesFromCurves;
    float DeltaTime = 0;

    std::vector<FTransform> inpose;
    std::vector<FTransform> inposeLocal;
    std::vector<FTransform> outposeLocal;
    std::vector<FTransform> initInPoseLocal = srcusk.refpose;
    std::vector<FTransform> initOutPoseLocal = tgtusk.refpose;
    //std::vector<SoulTransform> initSoulInPoseLocal;
    //std::vector<SoulTransform> initSoulOutPoseLocal;
    for(int frame = 0; frame < frameCount; frame++) {

        DEBUG_PRINT("frame:%d\n", frame);
        DEBUG_PRINT_IO_SOULPOSE("InSoulPose srccoord", tempposes[frame], srcskm, frame);

        // input and cast
        IKRigUtils::SoulPose2FPose(tempposes[frame], inposeLocal);

        // coord convert
        IKRigUtils::LocalFPoseCoordConvert(tsrc2work, srccoord, workcoord, inposeLocal);

        // to global
        IKRigUtils::FPoseToGlobal(srcskm.skeleton, inposeLocal, inpose);
        DEBUG_PRINT_IO_FPOSE("inFPose workcoord", srcskm, inposeLocal, inpose, initInPoseLocal, frame);

        // retarget
        std::vector<FTransform>& outpose = ikretarget.RunRetargeter(inpose, SpeedValuesFromCurves, DeltaTime);

        // to local
        IKRigUtils::FPoseToLocal(tgtskm.skeleton, outpose, outposeLocal);
        DEBUG_PRINT_IO_FPOSE("outFpose workcoord", tgtskm, outposeLocal, outpose, initOutPoseLocal, frame);

        // coord convert
        IKRigUtils::LocalFPoseCoordConvert(twork2tgt, workcoord, tgtcoord, outposeLocal);

        // cast and output
        IKRigUtils::FPose2SoulPose(outposeLocal, tempoutposes[frame]);
        DEBUG_PRINT_IO_SOULPOSE("outSoulPose tgtcoord", tempoutposes[frame], tgtskm, frame);
    }

    printf("process animation %d keyframes\n", frameCount);

    /////////////////////////////////////////////
    // output pose animation to mesh0
    writePoseAnimationToMesh(tempoutposes, tgtskm, frameCount, srcskm.animation.ticksPerSecond);
    fbxTarget.writeSkeletonMesh(outfile);

    return true;
}

