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

    struct SoulIKRigRetargetConfig {
        struct SoulIKRigChain {
            std::string chainName;
            std::string startBone;
            std::string endBone;
        };
        struct SoulIKRigChainMapping {
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
        bool SkipRoot{false};       // todo
        bool UseGroundBone{true};   // todo
        std::string SourceRootBone;
        std::string SourceGroundBone;
        std::string TargetRootBone;
        std::string TargetGroundBone;

        std::vector<SoulIKRigChain> SourceChains;
        std::vector<SoulIKRigChain> TargetChains;
        std::vector<SoulIKRigChainMapping> ChainMapping;
    };


    class IKRigUtils {
    private:
    public:
        // coord convert
        
        static void USkeletonCoordConvert(CoordType srcCoord, CoordType tgtCoord, USkeleton& sk);
        
        static FTransform getFTransformFromCoord(CoordType srcCoord, CoordType tgtCoord);
        static bool LocalFPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose);
        static bool GlobalFPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose);
        static bool LocalFPoseCoordConvert(FTransform& t, CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose);
        static bool GlobalFPoseCoordConvert(FTransform& t, CoordType srcCoord, CoordType tgtCoord,  std::vector<FTransform>& pose);

        static SoulTransform getSoulTransformFromCoord(CoordType srcCoord, CoordType tgtCoord);
        static bool LocalSoulPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose);
        static bool GlobalSoulPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose);
        static bool LocalSoulPoseCoordConvert(SoulTransform& t, CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose);
        static bool GlobalSoulPoseCoordConvert(SoulTransform& t, CoordType srcCoord, CoordType tgtCoord, std::vector<SoulTransform>& pose);

        // pose local/global
        static void FPoseToLocal(SoulSkeleton& sk, std::vector<FTransform>& globalpose, std::vector<FTransform>& localpose);
        static void FPoseToGlobal(SoulSkeleton& sk, std::vector<FTransform>& localpose, std::vector<FTransform>& globalpose);
        static void SoulPoseToLocal(SoulSkeleton& sk, std::vector<SoulTransform>& globalpose, std::vector<SoulTransform>& localpose);
        static void SoulPoseToGlobal(SoulSkeleton& sk, std::vector<SoulTransform>& localpose, std::vector<SoulTransform>& globalpose);

        // pose struct cast
        static void SoulPose2FPose(SoulPose& soulpose, std::vector<FTransform>& pose);
        static void FPose2SoulPose(std::vector<FTransform>& pose, SoulPose& soulpose);

        // transform cast
        SoulTransform glmToSoulTransform(glm::mat4& m);
        FTransform glmToFTransform(glm::mat4& m);

        // uskeleton
        static bool getUSkeletonFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, USkeleton& usk, CoordType srcCoord, CoordType tgtCoord);
        static bool USkeleton2RigSkeleton(USkeleton& sk, FIKRigSkeleton& rigsk);
        static std::vector<SoulJointNode> buildJointTree(SoulSkeleton& sk);

        // search
        static SoulNode* findNodeByName(SoulNode* rootNode, std::string& name);

        // get pose
        static bool getSoulPoseFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, std::vector<SoulTransform>& pose);
        static bool getSoulPoseFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh, SoulPose& outpose);

        // debug
        static void debugPrintFPose(SoulSkeleton& sk, std::vector<FTransform>& pose);
        static void debugPrintSoulPose(SoulSkeleton& sk, std::vector<SoulTransform>& pose);
        static void debugPrintLocalFPose(SoulSkeleton& sk, std::vector<FTransform>& pose);
        static void debugPrintNodePose(SoulNode* rootNode);
        // name tree
        static void debugPrintSkeletonTree(SoulSkeleton& sk);
        static void debugPrintSkeletonTreeIBM(SoulSkeleton& sk);
        static void debugPrintSkeletonTreeTransform(SoulScene& scene, SoulSkeletonMesh& skmesh);
        static void debugPrintSkeletonTreeGTransform(SoulScene& scene, SoulSkeletonMesh& skmesh);
        static void debugPrintUSkeletonTreeGTransform(SoulSkeleton& sk, USkeleton& usk);
        // TRS of some joints
        static void debugPrintPoseJoints(const std::string& prefix, SoulSkeleton& sk, std::vector<FTransform>& inpose, std::vector<std::string> jointNames);
    };
}
