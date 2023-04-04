//
//  IKRigUtils.hpp
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include "SoulScene.hpp"
#include "SoulRetargeter.h"
#include <math.h>

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

    class IKRigUtils {
    private:
        static FTransform getTransformFromCoord(CoordType srcCoord, CoordType tgtCoord);
    public:
        static bool LocalPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose);
        static bool GlobalPoseCoordConvert(CoordType srcCoord, CoordType tgtCoord, std::vector<FTransform>& pose);

        static void FPoseToLocal(SoulSkeleton& sk, std::vector<FTransform>& globalpose, std::vector<FTransform>& localpose);
        static void FPoseToGlobal(SoulSkeleton& sk, std::vector<FTransform>& localpose, std::vector<FTransform>& globalpose);

        static void SoulPose2FPose(SoulIK::SoulPose& soulpose, std::vector<FTransform>& pose);
        static void FPose2SoulPose(std::vector<FTransform>& pose, SoulIK::SoulPose& soulpose);
    };
}
