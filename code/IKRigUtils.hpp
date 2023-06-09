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

    // class CoordType
    // {
    // public:
    //     enum Value : uint8_t {
    //         RightHandZupYfront,     // 3dsmax
    //         RightHandYupZfront,     // maya houdini substance marmoset godot OpenGL
    //     };
    //     CoordType() = default;
    //     constexpr CoordType(Value aCoordType) : value(aCoordType) { }
    //     #if Enable switch(CoordType) use case:
    //     // Allow switch and comparisons.
    //     constexpr operator Value() const { return value; }
    //     // Prevent usage: if(fruit)
    //     explicit operator bool() const = delete;        
    //     #else
    //     constexpr bool operator==(CoordType a) const { return value == a.value; }
    //     constexpr bool operator!=(CoordType a) const { return value != a.value; }
    //     #endif
    //     constexpr bool ToString() const { 
    //         if(value == RightHandZupYfront) {
    //             return "RightHandZupYfront";
    //         } else if (value == RightHandYupZfront) {
    //             return "RightHandYupZfront";
    //         }
    //     }
    // private:
    //     Value value;
    // };

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

            SoulIKRigChain() = default;
            SoulIKRigChain(std::string const& chainName_, std::string const& startBone_, std::string const& endBone_)
            : chainName(chainName_), startBone(startBone_), endBone(endBone_) {
            }
        };
        struct SoulIKRigChainMapping {
            bool EnableFK{true};
            bool EnableIK{false};
            std::string SourceChain;
            std::string TargetChain;

            SoulIKRigChainMapping() = default;
            SoulIKRigChainMapping(bool EnableFK_, bool EnableIK_, std::string const& SourceChain_, std::string const& TargetChain_) 
            : EnableFK(EnableFK_), EnableIK(EnableIK_),  SourceChain(SourceChain_), TargetChain(TargetChain_) {
            }
        };

        // coordinate system
        CoordType SourceCoord;
        CoordType WorkCoord{CoordType::RightHandZupYfront};
        CoordType TargetCoord;

        // root
        ERootType   SourceRootType{ERootType::RootZMinusGroundZ};
        std::string SourceRootBone;
        std::string SourceGroundBone;
        ERootType   TargetRootType{ERootType::RootZMinusGroundZ};
        std::string TargetRootBone;
        std::string TargetGroundBone;

        std::vector<SoulIKRigChain> SourceChains;
        std::vector<SoulIKRigChain> TargetChains;
        std::vector<SoulIKRigChainMapping> ChainMapping;

        std::vector<int> IntArray; // test python binding

        std::string to_string();
    };


    class IKRigUtils {
    private:
    public:

        static std::shared_ptr<UIKRetargeter> createIKRigAsset(SoulIKRigRetargetConfig& config,
            SoulSkeleton& srcsk, SoulSkeleton& tgtsk, USkeleton& srcusk, USkeleton& tgtusk
        );

        // coord convert
        static std::string CoordTypeToString(CoordType aCoordType);
        static std::string ERootTypeToString(ERootType aRootType);
        
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
        static std::vector<SoulTransform> SoulPoseToGlobal(SoulSkeleton& sk, std::vector<SoulTransform>& localpose);

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
        static bool alignUSKWithSkeleton(USkeleton& usk, SoulSkeleton const& sk, SoulScene& uskScene, SoulScene&skScene);

        // search
        static SoulNode* findNodeByName(SoulNode* rootNode, std::string& name);

        // get pose
        static std::vector<SoulTransform> getSoulPoseTransformFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh);
        static SoulPose getSoulPoseFromMesh(SoulScene& scene, SoulSkeletonMesh& skmesh);

        // debug
        static void debugPrintFPose(SoulSkeleton& sk, std::vector<FTransform>& pose);
        static void debugPrintFPose(SoulSkeleton& sk, std::vector<FTransform>& pose, std::vector<FTransform>& initpose);
        static void debugPrintSoulPose(SoulSkeleton& sk, std::vector<SoulTransform>& pose);
        static void debugPrintLocalFPose(SoulSkeleton& sk, std::vector<FTransform>& pose);
        static void debugPrintNodePose(SoulNode* rootNode);
        static void debugPrintUSKNames(USkeleton& usk);
        // name tree
        static void debugPrintSkeletonTree(SoulSkeleton& sk);
        static void debugPrintSkeletonTreeIBM(SoulSkeleton& sk);
        static void debugPrintSkeletonTreeTransform(SoulScene& scene, SoulSkeletonMesh& skmesh);
        static void debugPrintSkeletonTreeGTransform(SoulScene& scene, SoulSkeletonMesh& skmesh);
        static void debugPrintUSkeletonTreeGTransform(SoulSkeleton& sk, USkeleton& usk);
        static void debugPrintNodeTree(SoulNode* node, int depth = 0);
        // TRS of some joints
        static void debugPrintPoseJoints(const std::string& prefix, SoulSkeleton& sk, std::vector<FTransform>& inpose, std::vector<std::string> jointNames);


        static void debugPrintSKM(std::string const& name, SoulScene& scene, SoulSkeletonMesh& skm, CoordType srccoord, CoordType workcoord);
        static void debugPrintUSK(std::string const& name, USkeleton& usk, SoulSkeletonMesh& skm, CoordType srccoord, CoordType workcoord);
        static void debugPrintIOSoulPose(std::string const& name, SoulPose& pose, SoulSkeletonMesh& skm, int frame);
        static void debugPrintIOFPose(std::string const& name, 
            SoulSkeletonMesh& skm, 
            std::vector<FTransform>& inposeLocal, 
            std::vector<FTransform>& inpose, 
            std::vector<FTransform>& initInPoseLocal, 
            int frame);
    };
}
