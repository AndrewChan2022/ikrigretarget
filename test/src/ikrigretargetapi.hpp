//
//  ikrigretargetapi.hpp
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include "SoulScene.hpp"
#include "SoulRetargeter.h"
#include "IKRigUtils.hpp"
#include "SoulIKRetargetProcessor.h"


int myadd(int a, int b);

bool retargetFBX(std::string const& srcAnimationFile,
    std::string const& srcTPoseFile,
    std::string const& rootName,
    std::string const& targetFile,
    std::string const& targetTPoseFile,
    std::string const& outfile,
    SoulIK::SoulIKRigRetargetConfig& config);
