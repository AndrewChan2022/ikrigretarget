//
//  InitPoseConvert.h
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include <stdio.h>


#include "SoulScene.hpp"
#include "SoulRetargeter.h"
#include "IKRigUtils.hpp"
#include "SoulIKRetargetProcessor.h"

namespace SoulIK {
    void testProject();

    void convertToTPose();
    void readAndWriteFBX(std::string const& inputfile, std::string const& outputfile);
    SoulPose getMetaTPoseSoulPose(SoulSkeleton& sk);
}
