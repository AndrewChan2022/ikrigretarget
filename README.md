
# platform

    1. mac 
    2. linux
    3. windows

# windows build and test c++

    //./build.sh 

    rm -rf build
    mkdir build && cd build

    cmake ..
    
    open ikrigretarget.sln with visual studio
    set testikrigretarget as start project
    
    or
    
    cmake --build .
    test/Debug/testikrigretarget.exe

# release files

    release/libikrigretarget.a

# build release
    
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make VERBOSE=1

# prerequisite

# usage:

## working coordinate system: 

    right hand

    z up

    x left

    y front

## source files:

    lib         // retarget implement
    lib/glm     // thirdParty files, need remove if already exist in your project
    test        // test project, including fbx file read write

## header files:

    SoulRetargeter.h                // define retarget asset
    SoulIKRetargetProcessor.h       // retarget processor
    IKRigUtils.h                    // config define, utils for pose convert, coord system convert...
    SoulScene.h                     // scene, mesh, skeleton, animation define

## init of uskeleton:

    class USkeleton {
        std::string name;
        std::vector<FBoneNode> boneTree;    // each item name and parentId
        std::vector<FTransform> refpose;    // coord: Right Hand Z up Y front
        ...
    };
    struct FBoneNode {
        std::string name;
        int32_t parent;
        ...
    };

## init of IKRigRetargetAsset

    // define retarget config
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

    // then create Asset with config
    asset = createIKRigAsset(SoulIKRigRetargetConfig& config,
                    SoulSkeleton& srcsk, SoulSkeleton& tgtsk,
                    USkeleton& srcusk, USkeleton& tgtusk);


## fix to tpose

for test model, its rest pose is A pose, so need fix it to tpose

    if (isTargetMetaAndAPose) {
        // fix to tpose
        tgtusk.refpose = getMetaTPoseFPose(...);
    }

## run retarget:

        // type cast
        IKRigUtils::SoulPose2FPose(inposes[frame], inposeLocal);

        // coord convert
        IKRigUtils::LocalPoseCoordConvert(tsrc2work, inposeLocal, srccoord, workcoord);

        // to global pose
        IKRigUtils::FPoseToGlobal(srcskm.skeleton, inposeLocal, inpose);

        // retarget
        std::vector<FTransform>& outpose = ikretarget.RunRetargeter(inpose, SpeedValuesFromCurves, DeltaTime);
        
        // to local pose
        IKRigUtils::FPoseToLocal(tgtskm.skeleton, outpose, outposeLocal);
        
        // coord convert
        IKRigUtils::LocalPoseCoordConvert(twork2tgt, outposeLocal, workcoord, tgtcoord);

        // type cast
        IKRigUtils::FPose2SoulPose(outposeLocal, outposes[frame]);

## full example:

main.cpp

    #include "SoulScene.h"
    #include "SoulRetargeter.h"
    #include "SoulIKRetargetProcessor.h"
    #include "IKRigUtils.hpp"
    #include "FBXRW.h"

    /////////////////////////////////////////////
    // config
    auto config             = config_to_meta();
    CoordType srccoord      = config.SourceCoord;
    CoordType workcoord     = config.WorkCoord;
    CoordType tgtcoord      = config.TargetCoord;
    bool isModelAPose       = true;

    /////////////////////////////////////////////
    // read fbx
    SoulIK::FBXRW fbxrw, fbxrw2;
    fbxrw.readSkeketonMesh(inputfile);
    fbxrw2.readSkeketonMesh(inputfile2);

    SoulIK::SoulScene& srcscene = *fbxrw.getSoulScene();
    SoulIK::SoulScene& tgtscene = *fbxrw2.getSoulScene();
    SoulIK::SoulSkeletonMesh& srcskm = *srcscene.skmeshes[0];
    SoulIK::SoulSkeletonMesh& tgtskm = *tgtscene.skmeshes[0];

    /////////////////////////////////////////////
    // init
    SoulIK::USkeleton srcusk;
    SoulIK::USkeleton tgtusk;
    IKRigUtils::getUSkeletonFromMesh(srcscene, srcskm, srcusk, srccoord, workcoord);
    IKRigUtils::getUSkeletonFromMesh(tgtscene, tgtskm, tgtusk, tgtcoord, workcoord);
    if (isModelAPose) { 
        // fix to tpose
        tgtusk.refpose = getMetaTPoseFPose(tgtskm.skeleton, tgtcoord, workcoord);
    }

    SoulIK::UIKRetargetProcessor ikretarget;
	auto InRetargeterAsset = createIKRigAsset(config, srcskm.skeleton, tgtskm.skeleton, srcusk, tgtusk);
    ikretarget.Initialize(&srcusk, &tgtusk, InRetargeterAsset.get(), false);
    
    /////////////////////////////////////////////
    // build pose animation
    std::vector<SoulIK::SoulPose> inposes;
    std::vector<SoulIK::SoulPose> outposes;
    ... 

    /////////////////////////////////////////////
    // run retarget
    for(int frame = 0; frame < inposes.size(); frame++) {
        // type cast
        IKRigUtils::SoulPose2FPose(inposes[frame], inposeLocal);

        // coord convert
        IKRigUtils::LocalPoseCoordConvert(tsrc2work, inposeLocal, srccoord, workcoord);

        // to global pose
        IKRigUtils::FPoseToGlobal(srcskm.skeleton, inposeLocal, inpose);

        // retarget
        std::vector<FTransform>& outpose = ikretarget.RunRetargeter(inpose, SpeedValuesFromCurves, DeltaTime);
        
        // to local pose
        IKRigUtils::FPoseToLocal(tgtskm.skeleton, outpose, outposeLocal);
        
        // coord convert
        IKRigUtils::LocalPoseCoordConvert(twork2tgt, outposeLocal, workcoord, tgtcoord);

        // type cast
        IKRigUtils::FPose2SoulPose(outposeLocal, outposes[frame]);
    }

    /////////////////////////////////////////////
    // write animation to fbx
    ...

# feature work

develop maya plugin based on this lib

render the skeleton and animation

# Acknowledgements

this repo copy from   https://github.com/EpicGames/UnrealEngine

path:  Engine/Plugins/Animation/IKRig

I use glm to implement Unreal math, and keep coordinate system right hand, z up, y front

this is different with Unreal, which is left hand, z up, y front
