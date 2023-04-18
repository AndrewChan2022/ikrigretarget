
- [todo and issues](#todo-and-issues)
- [algorithm](#algorithm)
  * [coordinate hand](#coordinate-hand)
  * [transform represent](#transform-represent)
  * [store order](#store-order)
  * [transform in our code](#transform-in-our-code)
  * [quaternion order](#quaternion-order)
  * [skeleton pose transform](#skeleton-pose-transform)
  * [root retarget](#root-retarget)
  * [chain FK retarget](#chain-fk-retarget)
  * [chain IK retarget](#chain-ik-retarget)
  * [Pole Match retarget](#pole-match-retarget)
- [platform](#platform)
- [cmake option](#cmake-option)
- [macos](#macos)
- [windows build and test c++](#windows-build-and-test-c--)
- [release files](#release-files)
- [build release](#build-release)
- [prerequisite](#prerequisite)
- [usage](#usage)
  * [working coordinate system](#working-coordinate-system)
  * [source files](#source-files)
  * [header files](#header-files)
  * [full example](#full-example)
  * [input model](#input-model)
  * [coordtype convert](#coordtype-convert)
  * [init of uskeleton](#init-of-uskeleton)
  * [root retarget config](#root-retarget-config)
  * [chain retarget config](#chain-retarget-config)
  * [put config all to IKRigRetargetAsset](#put-config-all-to-ikrigretargetasset)
  * [tpose](#tpose)
  * [init of processor](#init-of-processor)
  * [run retarget:](#run-retarget-)
- [feature work](#feature-work)
- [Release notes](#release-notes)
- [Acknowledgements](#acknowledgements)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# todo and issues

1. python interface
2. ik part
3. fbx sdk
4. render part

# algorithm

## coordinate hand

all the coodinate system is right hand.

## transform represent

https://fgiesen.wordpress.com/2012/02/12/row-major-vs-column-major-row-vectors-vs-column-vectors/

reference:  Game Engine Architexture chapter 5.3.2

Points and vectors can be represented as row matrices (1× n) or column matrices (n × 1)

row represent:&nbsp;&nbsp;&nbsp; 

$$v1=\begin{bmatrix}x & y & z\end{bmatrix}$$

col represent:&nbsp;&nbsp;&nbsp; 

$$ v2=\begin{bmatrix}
x \\
y \\
z \\
\end{bmatrix} = v1^T $$

the choice between column and row vectors affect the order of matrix multiply

apply matrix to vector:

row vector:&nbsp;&nbsp;&nbsp; $$v1_{1 \times n} = v1_{1 \times n} \times M_{n \times n}$$

col vector:&nbsp;&nbsp;&nbsp; $$v2_{n \times 1} = M_{n \times n} \times v2_{n \times 1}$$

multiple matrix concatenate, apply M1 first, then M2: 

$$v1 = v1 \times M1 \times M2$$

$$v2 = M2 \times M1 \times v2$$

the represent also affect the element order of matrix. 

example of translation matrix for hogogeneous coordinate:

row vector:

$$M_{translation}=
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
tx & ty & tz & 1 \\
\end{bmatrix}$$

col vector:

$$M_{translation}=
\begin{bmatrix}
1 & 0 & 0 & tx \\
0 & 1 & 0 & ty \\
0 & 0 & 1 & tz \\
0 & 0 & 0 & 1 \\
\end{bmatrix}$$


## storage order

the matrix can store in row major or col major.

the store order does not affect the represent, but it affect the element order of memory

to access element:

row major:  m[row][col]

col major:  m[col][row]

## transform in our code

    SoulScene.h
        glm::mat4/glm::dmat4:
            represent: col vector
            store order: col major
        
    SoulFTransform SoulRetargeter.h SoulIKRetargetProcessor.h
        https://docs.unrealengine.com/4.27/en-US/API/Runtime/Core/Math/FTransform/
        Unreal FTransform 
            represent: row vector
            store order: row major
    lib ASSIMP
        aiMatrix4x4:
            represent: col vector
            store order: row major

## quaternion order

order of all quat: left is first

## skeleton pose transform

reference:  game engine architecture chapter 12.3.3

    
joint local space: the space of joint

global space: in world space or model space, because we does not care about transform outside model, so we choose model space.

to transform local pose to global pose(model space), only walking the skeleton hierarchy from current joint all the way to root.

denote transform from joint j local space to its parent space:

$$P_{j->p(j)}$$

row represent:  

$$P_{5->M} = P_{5->4} \times P_{4->3} \times P_{3->0} \times P_{0->M}$$

col represent:

$$P_{5->M} =  P_{0->M} \times P_{3->0} \times P_{4->3} \times P_{5->4} $$

![pose transform](/img/posetransform.png "pose transform")



## root retarget
root retarget: retarget position by height ratio

    init:
        source.InitialHeightInverse = 1/ root.z
        target.initialHeight = root.z
    retarget:
        target.root.translation = source.root.translation *  target.initialHeight * source.InitialHeightInverse

## chain FK retarget
chain FK retarget: copy global rotation delta

    init:
        foreach chain:
            foreach joint:
                record initialPoseGlobal, initialPoseLocal
                reset currentPoseGlobal
    retarget(inputPoseGlobal, outposeGlobal):
        foreach chain:
            foreach joint:

                // apply parent transform to child to get position
                currentPositionGlobal = apply parrent.currentPoseGlobal to initialPoseLocal

                // copy global rotation delta
                deltaRotationGlobal = inputPoseGlobal.currentRotationGlboal / source.initalRotationGlboal
                currentRotationGlboal = initialRotationGlobal * deltaRotationGlobal

                // copy global scale delta
                currentScaleGlobal = TargetInitialScaleGlobal + (SourceCurrentScaleGlobal - SourceInitialScaleGlobal);

                // pose from position and rotation
                currentPoseGlobal = (currentPositionGlobal, currentRotationGlboal)
                outposeGlobal[boneIndex] =  currentPoseGlobal

## chain IK retarget
    // chain IK retarget
    todo

## Pole Match retarget
    // pole match retarget
    todo


# platform

    1. mac 
    2. linux
    3. windows

# cmake option

    // with assimp project embedded
    -DEMBED_ASSIMP=ON

    // release
    --DCMAKE_BUILD_TYPE=Release

# macos

    rm -rf build
    mkdir build && cd build

    // for xcode project
    cmake .. -GXcode

    // without xcode
    cmake .. --DCMAKE_BUILD_TYPE=Release
    make -j16


# windows build and test c++

    //./build.sh 

    rm -rf build
    mkdir build && cd build

    cmake ..
    
    // with visual studio
    open ikrigretarget.sln with visual studio
    set testikrigretarget as start project
    
    // without visual studio
    cmake --build .
    test/Debug/testikrigretarget.exe

# release files

    release/libikrigretarget.a

# build release
    
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make VERBOSE=1

# prerequisite

# usage

## working coordinate system

    right hand

    z up

    x left

    y front

## source files

    lib         // retarget implement
    lib/glm     // thirdParty files, need remove if already exist in your project
    test        // test project, including fbx file read write

## header files

    SoulRetargeter.h                // define retarget asset
    SoulIKRetargetProcessor.h       // retarget processor
    IKRigUtils.h                    // config define, utils for pose convert, coord system convert...
    SoulScene.h                     // scene, mesh, skeleton, animation define

## full example

main.cpp

    #include "SoulScene.h"
    #include "SoulRetargeter.h"
    #include "SoulIKRetargetProcessor.h"
    #include "IKRigUtils.hpp"
    #include "FBXRW.h"

    /////////////////////////////////////////////
    // config
    TestCase testCase       = case_Flair();
    auto config             = testCase.config;
    CoordType srccoord      = config.SourceCoord;
    CoordType workcoord     = config.WorkCoord;
    CoordType tgtcoord      = config.TargetCoord;

    /////////////////////////////////////////////
    // read fbx
    std::string srcAnimationFile, srcTPoseFile, targetFile, targetTPoseFile, outfile;
    getFilePaths(srcAnimationFile, srcTPoseFile, targetFile, targetTPoseFile, outfile, testCase);

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

    /////////////////////////////////////////////
    // init
    SoulIK::USkeleton srcusk;
    SoulIK::USkeleton tgtusk;
    IKRigUtils::getUSkeletonFromMesh(srcTPoseScene, *srcTPoseScene.skmeshes[0], srcusk, srccoord, workcoord);
    IKRigUtils::alignUSKWithSkeleton(srcusk, srcskm.skeleton);
    IKRigUtils::getUSkeletonFromMesh(tgtTPosescene, *tgtTPosescene.skmeshes[0], tgtusk, tgtcoord, workcoord);
    IKRigUtils::alignUSKWithSkeleton(tgtusk, tgtskm.skeleton); 

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

## input model

    source animation    : including skeleton animation
    source tpose        : tpose skeleton for source animation model
    target animation    : save animation based on this model
    target tpose        : tpose skeleton for target animation model

    
    because animation model and tpose model may have different skeleton order
    so need align them:

    IKRigUtils::alignUSKWithSkeleton(sourceTPoseUSKeleton, sourceAnimationSkeletonMesh.skeleton);

## coordtype convert

    enum class CoordType: uint8_t {
        RightHandZupYfront,
        RightHandYupZfront,
    };

    wording coord   : CoordType::RightHandZupYfront
    from maya       : CoordType::RightHandYupZfront

## init of uskeleton

    class USkeleton {
        std::string name;
        std::vector<FBoneNode> boneTree;    // each item name and parentId
        std::vector<FTransform> refpose;    // coord: Right Hand Z up Y front, local
        ...
    };
    struct FBoneNode {
        std::string name;                   // joint name
        int32_t parent;                     // joint tree relationship
        ...
    };

## root retarget config

    1. define root type
    2. define root name
    3. if need ground bone, define ground bone type

    enum class ERootType: uint8_t {
	    RootZ = 0,          // height = root.translation.z
	    RootZMinusGroundZ,  // height = root.translation.z - ground.translation.z
	    Ignore              // skip
    };

    // root
    ERootType   SourceRootType{ERootType::RootZMinusGroundZ};
    std::string SourceRootBone;
    std::string SourceGroundBone;
    ERootType   TargetRootType{ERootType::RootZMinusGroundZ};
    std::string TargetRootBone;
    std::string TargetGroundBone;


## chain retarget config

    1. source skeleton define many chains
    2. target skeleton define may chains
    3. define chain mapping

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

    std::vector<SoulIKRigChain> SourceChains;
    std::vector<SoulIKRigChain> TargetChains;
    std::vector<SoulIKRigChainMapping> ChainMapping;

## put config all to IKRigRetargetAsset

    /////////////////////////////////////////////
    // define config
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
        ERootType   SourceRootType{ERootType::RootZMinusGroundZ};
        std::string SourceRootBone;
        std::string SourceGroundBone;
        ERootType   TargetRootType{ERootType::RootZMinusGroundZ};
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


    /////////////////////////////////////////////
    // example config:

    SoulIKRigRetargetConfig config;
    config.SourceCoord      = CoordType::RightHandZupYfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandYupZfront;

    config.SourceRootType   = ERootType::RootZMinusGroundZ;
    config.TargetRootType   = ERootType::RootZ;

    config.SourceRootBone   = "mixamorig:Hips";
    config.SourceGroundBone = "mixamorig:LeftToe_End";
    config.TargetRootBone   = "Rol01_Torso01HipCtrlJnt_M";
    config.TargetGroundBone = "Rol01_Leg01FootJnt_L";
    
    //config.skipRootBone = true;

    config.SourceChains = {
        // name     start               end
        // spine
        {"spine",   "Spine",            "Thorax"},

        // head
        {"head",    "Neck",             "Head"},

        //{"lleg",    "LeftHip",          "LeftAnkle"},
        {"lleg1",   "LeftHip",          "LeftHip"},
        {"lleg2",   "LeftKnee",         "LeftKnee"},
        {"lleg3",   "LeftAnkle",        "LeftAnkle"},

        //{"rleg",    "RightHip",         "RightAnkle"},
        {"rleg1",   "RightHip",         "RightHip"},
        {"rleg2",   "RightKnee",        "RightKnee"},
        {"rleg3",   "RightAnkle",       "RightAnkle"},

        //{"larm",    "LeftShoulder",     "LeftWrist"},
        {"larm1",   "LeftShoulder",     "LeftShoulder"},
        {"larm2",   "LeftElbow",        "LeftElbow"},
        {"larm3",   "LeftWrist",        "LeftWrist"},
        
        //{"rram",    "RightShoulder",    "RightWrist"},
        {"rram1",   "RightShoulder",    "RightShoulder"},
        {"rram2",   "RightElbow",       "RightElbow"},
        {"rram3",   "RightWrist",       "RightWrist"},

    };

    config.TargetChains = {
        // name    start        end
        // spine
        {"spine",   "Rol01_Torso0102Jnt_M",     "Rol01_Neck0101Jnt_M"},

        // head
        {"head",    "Rol01_Neck0102Jnt_M",      "Head_M"},

        //{"lleg",    "Rol01_Leg01Up01Jnt_L",     "Rol01_Leg01AnkleJnt_L"},
        {"lleg1",   "Rol01_Leg01Up01Jnt_L",     "Rol01_Leg01Up01Jnt_L"},
        {"lleg2",   "Rol01_Leg01Low01Jnt_L",    "Rol01_Leg01Low01Jnt_L"},
        {"lleg3",   "Rol01_Leg01AnkleJnt_L",    "Rol01_Leg01AnkleJnt_L"},

        //{"rleg",    "Rol01_Leg01Up01Jnt_R",     "Rol01_Leg01AnkleJnt_R"},
        {"rleg1",   "Rol01_Leg01Up01Jnt_R",     "Rol01_Leg01Up01Jnt_R"},
        {"rleg2",   "Rol01_Leg01Low01Jnt_R",    "Rol01_Leg01Low01Jnt_R"},
        {"rleg3",   "Rol01_Leg01AnkleJnt_R",    "Rol01_Leg01AnkleJnt_R"},

        //{"larm",    "Rol01_Arm01Up01Jnt_L",     "Rol01_Arm01Low03Jnt_L"},
        {"larm1",   "Rol01_Arm01Up01Jnt_L",     "Rol01_Arm01Up01Jnt_L"},
        {"larm2",   "Rol01_Arm01Low01Jnt_L",    "Rol01_Arm01Low01Jnt_L"},
        {"larm3",   "Rol01_Hand01MasterJnt_L",  "Rol01_Hand01MasterJnt_L"},

        //{"rram",    "Rol01_Arm01Up01Jnt_R",    "Rol01_Arm01Low03Jnt_R"},
        {"rram1",   "Rol01_Arm01Up01Jnt_R",     "Rol01_Arm01Up01Jnt_R"},
        {"rram2",   "Rol01_Arm01Low01Jnt_R",    "Rol01_Arm01Low01Jnt_R"},
        {"rram3",   "Rol01_Hand01MasterJnt_R",  "Rol01_Hand01MasterJnt_R"},        
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        
        // spine
        {true,  false,  "spine",        "spine"},

        // head
        {true,  false,  "head",         "head"},

        // lleg
        {true,  false,  "lleg1",        "lleg1"},
        {true,  false,  "lleg2",        "lleg2"},
        {true,  false,  "lleg3",        "lleg3"},
        
        // rleg
        {true,  false,  "rleg1",        "rleg1"},
        {true,  false,  "rleg2",        "rleg2"},
        {true,  false,  "rleg3",        "rleg3"},

        // larm
        {true,  false,  "larm1",        "larm1"},
        {true,  false,  "larm2",        "larm2"},
        {true,  false,  "larm3",        "larm3"},
        
        // rarm
        {true,  false,  "rram1",        "rram1"},
        {true,  false,  "rram2",        "rram2"},
        {true,  false,  "rram3",        "rram3"},
    };


## tpose

    better both source and target initial pose on tpose

    but other initial pose also fine

    rule: source inital pose == target initial pose

## init of processor

    SoulIK::UIKRetargetProcessor ikretarget;
    ikretarget.Initialize(&srcusk, &tgtusk, asset, false);    

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





# feature work

develop maya plugin based on this lib

render the skeleton and animation so easy debug

# Release notes

version 1.0.4: 2023.4.14

    1. update macos assimp lib
    2. fix macos bug:
        create createIKRigAsset targetBoneIndex error
        SoulIKRetargetProcessor sort chain error: 
            std::sort(ChainPairsFK.begin(), ChainPairsFK.end(), ChainsSorter);

            return A.TargetBoneChainName.compare(B.TargetBoneChainName) <= 0;
            =>
            return A.TargetBoneChainName.compare(B.TargetBoneChainName) < 0;
    3. change /lib to /code
    4. remove embedded assimp project if not -DEMBED_ASSIMP=ON

version 1.0.3: 2023.4.13

    fix animation jump:  quaternion not normalize.

version 1.0.2: 2023.4.12

    fix flair to meta retarget:  change coordtype and rootBoneName

    add ERootType:
        RootZ               : height = root.translation.z
        RootZMinusGroundZ   : height = root.translation.z - ground.translation.z
        Ignore              : skip root retarget

version 1.0.1: 2023.4.12

    input file:  sourceAnimation sourceTPose targetAnimation targetTPose
        need align tpose uskeleton to animation skeleton
    add testcase struct

    fix uskeleton coordtype convert
    fix tpose and animation pose alignment

    add macos support with release assimp lib
    windows change assimp from debug to release

version 1.0.0: 2023.4.10:
    read source animation fbx file
    read source tpose fbx file
    read target meta fbx file

    run retarget from source animation to target meta file

    retarget config:
        s1 to meta
        flair to meta

# Acknowledgements

this repo copy from   https://github.com/EpicGames/UnrealEngine

path:  Engine/Plugins/Animation/IKRig

I use glm to implement Unreal math, and keep coordinate system right hand, z up, y front

this is different with Unreal, which is left hand, z up, y front
