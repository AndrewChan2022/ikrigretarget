
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

coordinate system: 

    right hand

    z up

    x left
    
    y front

example code:


    // init skeleton
    SoulIK::USkeleton srcusk = ...;
    SoulIK::USkeleton tgtusk = ...;

    // init IKRigAsset and IKRigRetargetAsset
    std::shared_ptr<UIKRetargeter> InRetargeterAsset;
    InRetargeterAsset  = buildIKRigRetargetAsset(srcskm.skeleton, tgtskm.skeleton, srcusk, tgtusk);

    // init ikretarget with Skeleton, IKRigAsset and IKRigRetargetAsset
    SoulIK::UIKRetargetProcessor ikretarget;
    ikretarget.Initialize(&srcusk, &tgtusk, InRetargeterAsset.get(), false);

    // run retargeting every frame
    std::unordered_map<FName, float> SpeedValuesFromCurves;
    float DeltaTime = 0;
    for(int frame = 0; frame < frameCount; frame++) {

        // inpose is global
        std::vector<FTransform> inposeLocal = ...; // get local pose from animation;
        std::vector<FTransform> inposeGlobal;
        inposeGlobal = ...; // LocalToGlobal(inposeLocal, inposeGlobal);

        // run retarget
        std::vector<FTransform>& outpose = ikretarget.RunRetargeter(inposeGlobal, SpeedValuesFromCurves, DeltaTime);

        // out pose to local
        std::vector<FTransform> outposeLocal;
        outposeLocal = ...; //FPoseToLocal(outpose, outposeLocal);
    }