//
//  IKRigUtils.hpp
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include "SoulScene.hpp"
#include "SoulRetargeter.h"

// SoulScene only for general data represent, not for data process and render
// you should define your native scene data structure for your processor or renderer
namespace SoulIK {

    enum class CoordType: uint8_t {
        RightHandZupYfront,     // 3dsmax
        RightHandYupZfront,     // maya houdini substance marmoset godot OpenGL
        //RightHandZupYback,      // blender
        //LeftHandYupZfront,      // unity c4d zbrush lightwave D3D
        //LeftHandZupYfront       // unreal
    };

    struct IKRigRetargetConfig {
        struct IKRigChain {
            std::string chainName;
            std::string startBone;
            std::string endBone;
        };
        struct IKRigChainMapping {
            bool EnableFK{true};
            bool EnableIK{false};
            std::string SourceChain;
            std::string TargetChain;
        };

        // coordinate system
        CoordType SourceCoord;
        CoordType WorkCoord{CoordType::RightHandZupYfront};
        CoordType TargetCoord;

        // root
        std::string SourceRootBone;
        std::string SourceGroundBone;
        std::string TargetRootBone;
        std::string TargetGroundBone;

        std::vector<IKRigChain> SourceChains;
        std::vector<IKRigChain> TargetChains;
        std::vector<IKRigChainMapping> ChainMapping;
    };


    class IKRigUtils {
    private:
    public:
        // coord convert
        static FTransform getTransformFromCoord(CoordType srcCoord, CoordType tgtCoord);
        static bool LocalPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose);
        static bool GlobalPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose);
        static void USkeletonCoordConvert(CoordType srcCoord, CoordType tgtCoord, USkeleton& sk);
        static bool LocalPoseCoordConvert(FTransform& t, std::vector<FTransform>& pose, CoordType srcCoord, CoordType tgtCoord);
        static bool GlobalPoseCoordConvert(FTransform& t, std::vector<FTransform>& pose, CoordType srcCoord, CoordType tgtCoord);

        // pose local/global
        static void FPoseToLocal(SoulSkeleton& sk, std::vector<FTransform>& globalpose, std::vector<FTransform>& localpose);
        static void FPoseToGlobal(SoulSkeleton& sk, std::vector<FTransform>& localpose, std::vector<FTransform>& globalpose);

        // pose struct cast
        static void SoulPose2FPose(SoulPose& soulpose, std::vector<FTransform>& pose);
        static void FPose2SoulPose(std::vector<FTransform>& pose, SoulPose& soulpose);

        // transform cast
        SoulTransform glmToSoulTransform(glm::mat4& m);
        FTransform glmToFTransform(glm::mat4& m);

        // uskeleton
        static bool getUSkeletonFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, USkeleton& usk, CoordType srcCoord, CoordType tgtCoord);
        static bool USkeleton2RigSkeleton(USkeleton& sk, FIKRigSkeleton& rigsk);

        // search
        static SoulNode* findNodeByName(SoulNode* rootNode, std::string& name);

        // get pose
        static bool getSoulPoseFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, std::vector<SoulTransform>& pose);
        static bool getSoulPoseFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, SoulPose& outpose);

        // debug
        static void debugPrintLocalFPose(SoulSkeleton& sk, std::vector<FTransform>& pose);
        static void debugPrintNodePose(SoulNode* rootNode);
        static void debugPrintSkeletonTree(SoulSkeleton& sk);
    };
}
