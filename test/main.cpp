//
//  main.cpp
//  test ikrigretarget
//
//  Created by kai chen on 2/8/23.
//

#include <stdio.h>

#include "SoulScene.hpp"
#include "SoulRetargeter.h"
#include "IKRigUtils.hpp"
#include "SoulIKRetargetProcessor.h"

#include "FBXRW.h"
#include "ObjRW.h"
#include "InitPoseConvert.h"

#include "ikrigretargetapi.hpp"

using namespace SoulIK;



static SoulIKRigRetargetConfig config1_1chain_lleg() {
    SoulIKRigRetargetConfig config;
    config.SourceCoord  = CoordType::RightHandZupYfront;
    config.WorkCoord    = CoordType::RightHandZupYfront;
    config.TargetCoord  = CoordType::RightHandZupYfront;

    config.SourceRootType   = ERootType::RootZMinusGroundZ;
    config.TargetRootType   = ERootType::RootZ;

    config.SourceRootBone   = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone   = "Hip";
    config.TargetGroundBone = "RightAnkle_end";

    config.SourceChains = {
        // name    start        end
        {"lleg", "RightHip", "RightKnee"}
    };

    config.TargetChains = {
        // name    start        end
        {"lleg", "RightHip", "RightKnee"}
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        {true,  false,  "lleg",         "lleg"}
    };

    return config;
}

static SoulIKRigRetargetConfig config2_6chain() {

    SoulIKRigRetargetConfig config;

    config.SourceCoord      = CoordType::RightHandZupYfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandZupYfront;

    config.SourceRootType   = ERootType::RootZMinusGroundZ;
    config.TargetRootType   = ERootType::RootZ;

    config.SourceRootBone   = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone   = "Hip";
    config.TargetGroundBone = "RightAnkle_end";

    config.SourceChains = {
        // name     start               end
        {"spine",   "Spine",            "Thorax"},
        {"lleg",    "LeftHip",          "LeftAnkle"},
        {"rleg",    "RightHip",         "RightAnkle"},
        {"larm",    "LeftShoulder",     "LeftWrist"},
        {"rram",    "RightShoulder",    "RightWrist"},
        {"head",    "Neck",             "Head"},
    };

    config.TargetChains = {
        // name     start               end
        {"spine",   "Spine",            "Thorax"},
        {"lleg",    "LeftHip",          "LeftAnkle"},
        {"rleg",    "RightHip",         "RightAnkle"},
        {"larm",    "LeftShoulder",     "LeftWrist"},
        {"rram",    "RightShoulder",    "RightWrist"},
        {"head",    "Neck",             "Head"},
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        {true,  false,  "spine",        "spine"},
        {true,  false,  "lleg",         "lleg"},
        {true,  false,  "rleg",         "rleg"},
        {true,  false,  "larm",         "larm"},
        {true,  false,  "rram",         "rram"},
        {true,  false,  "head",         "head"},
    };

    return config;
}

static SoulIKRigRetargetConfig config_s1_meta_error() {
    SoulIKRigRetargetConfig config;
    config.SourceCoord      = CoordType::RightHandZupYfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandYupZfront;

    config.SourceRootType   = ERootType::RootZMinusGroundZ;
    config.TargetRootType   = ERootType::RootZ;

    config.SourceRootBone   = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
    config.TargetRootBone   = "Rol01_Torso01HipCtrlJnt_M";
    config.TargetGroundBone = "Rol01_Leg01FootJnt_L";
    
    //config.skipRootBone = true;

    config.SourceChains = {
        // name     start               end
        {"spine",   "Spine",            "Thorax"},
        {"head",    "Neck",             "Head"},
        {"lleg",    "LeftHip",          "LeftAnkle"},
        {"rleg",    "RightHip",         "RightAnkle"},
        {"larm",    "LeftShoulder",     "LeftWrist"},        
        {"rram",    "RightShoulder",    "RightWrist"},

    };

    config.TargetChains = {
        // name    start        end
        {"spine",   "Rol01_Torso0102Jnt_M",     "Rol01_Neck0101Jnt_M"},
        {"head",    "Rol01_Neck0102Jnt_M",      "Head_M"},
        {"lleg",    "Rol01_Leg01Up01Jnt_L",     "Rol01_Leg01AnkleJnt_L"},
        {"rleg",    "Rol01_Leg01Up01Jnt_R",     "Rol01_Leg01AnkleJnt_R"},
        {"larm",    "Rol01_Arm01Up01Jnt_L",     "Rol01_Arm01Low03Jnt_L"},
        {"rram",    "Rol01_Arm01Up01Jnt_R",    "Rol01_Arm01Low03Jnt_R"},
    };

    config.ChainMapping = {
        // fk   ik      sourceChain     targetChain
        {true,  false,  "spine",        "spine"},
        {true,  false,  "head",         "head"},
        {true,  false,  "lleg",         "lleg"},
        {true,  false,  "rleg",         "rleg"},
        {true,  false,  "larm",         "larm"},
        {true,  false,  "rram",         "rram"},
    };

    return config;
}


static SoulIKRigRetargetConfig config_s1_meta() {
    SoulIKRigRetargetConfig config;
    config.SourceCoord      = CoordType::RightHandZupYfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandYupZfront;

    config.SourceRootType   = ERootType::RootZMinusGroundZ;
    config.TargetRootType   = ERootType::RootZ;

    config.SourceRootBone   = "Hip";
    config.SourceGroundBone = "RightAnkle_end";
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

    return config;
}

static SoulIKRigRetargetConfig config_flair_meta() {
    SoulIKRigRetargetConfig config;
    config.SourceCoord      = CoordType::RightHandYupZfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandYupZfront;

    config.SourceRootType   = ERootType::RootZ;
    config.TargetRootType   = ERootType::RootZ;

    config.SourceRootBone   = "mixamorig:Hips";
    config.SourceGroundBone = "mixamorig:LeftToeBase";
    config.TargetRootBone   = "Rol01_Torso01HipCtrlJnt_M";
    config.TargetGroundBone = "Rol01_Leg01FootJnt_L";
    
    //config.skipRootBone = true;

    config.SourceChains = {
        // name     start                               end
        // spine
        {"spine",       "mixamorig:Spine",              "mixamorig:Spine2"},

        // head
        {"head",        "mixamorig:Neck",               "mixamorig:Head"},

        // lleg
        {"lleg1",       "mixamorig:LeftUpLeg",          "mixamorig:LeftUpLeg"},
        {"lleg2",       "mixamorig:LeftLeg",            "mixamorig:LeftLeg"},
        {"lleg3",       "mixamorig:LeftFoot",           "mixamorig:LeftFoot"},

        // rleg
        {"rleg1",       "mixamorig:RightUpLeg",         "mixamorig:RightUpLeg"},
        {"rleg2",       "mixamorig:RightLeg",           "mixamorig:RightLeg"},
        {"rleg3",       "mixamorig:RightFoot",          "mixamorig:RightFoot"},

        // larm
        {"larm0",       "mixamorig:LeftShoulder",       "mixamorig:LeftShoulder"},
        {"larm1",       "mixamorig:LeftArm",            "mixamorig:LeftArm"},
        {"larm2",       "mixamorig:LeftForeArm",        "mixamorig:LeftForeArm"},
        {"larm3",       "mixamorig:LeftHand",           "mixamorig:LeftHand"},
        
        // lhand
        {"lfinger1",    "mixamorig:LeftHandThumb1",     "mixamorig:LeftHandThumb3"},
        {"lfinger2",    "mixamorig:LeftHandIndex1",     "mixamorig:LeftHandIndex3"},
        {"lfinger3",    "mixamorig:LeftHandMiddle1",    "mixamorig:LeftHandMiddle3"},
        {"lfinger4",    "mixamorig:LeftHandRing1",      "mixamorig:LeftHandRing3"},
        {"lfinger5",    "mixamorig:LeftHandPinky1",     "mixamorig:LeftHandPinky3"},

        // rarm
        {"rram0",       "mixamorig:RightShoulder",      "mixamorig:RightShoulder"},
        {"rram1",       "mixamorig:RightArm",           "mixamorig:RightArm"},
        {"rram2",       "mixamorig:RightForeArm",       "mixamorig:RightForeArm"},
        {"rram3",       "mixamorig:RightHand",          "mixamorig:RightHand"},

        // rhand
        {"rfinger1",    "mixamorig:RightHandThumb1",    "mixamorig:RightHandThumb3"},
        {"rfinger2",    "mixamorig:RightHandIndex1",    "mixamorig:RightHandIndex3"},
        {"rfinger3",    "mixamorig:RightHandMiddle1",   "mixamorig:RightHandMiddle3"},
        {"rfinger4",    "mixamorig:RightHandRing1",     "mixamorig:RightHandRing3"},
        {"rfinger5",    "mixamorig:RightHandPinky1",    "mixamorig:RightHandPinky3"},
    };

    config.TargetChains = {
        // name     start                           end
        // spine
        {"spine",       "Rol01_Torso0102Jnt_M",             "Rol01_Neck0101Jnt_M"},

        // head
        {"head",        "Rol01_Neck0102Jnt_M",              "Head_M"},

        // lleg
        {"lleg1",       "Rol01_Leg01Up01Jnt_L",             "Rol01_Leg01Up01Jnt_L"},
        {"lleg2",       "Rol01_Leg01Low01Jnt_L",            "Rol01_Leg01Low01Jnt_L"},
        {"lleg3",       "Rol01_Leg01AnkleJnt_L",            "Rol01_Leg01AnkleJnt_L"},

        // rleg
        {"rleg1",       "Rol01_Leg01Up01Jnt_R",             "Rol01_Leg01Up01Jnt_R"},
        {"rleg2",       "Rol01_Leg01Low01Jnt_R",            "Rol01_Leg01Low01Jnt_R"},
        {"rleg3",       "Rol01_Leg01AnkleJnt_R",            "Rol01_Leg01AnkleJnt_R"},

        // larm
        {"larm0",       "Rol01_Arm01ClavicleStartJnt_L",    "Rol01_Arm01ClavicleStartJnt_L"},
        {"larm1",       "Rol01_Arm01Up01Jnt_L",             "Rol01_Arm01Up01Jnt_L"},
        {"larm2",       "Rol01_Arm01Low01Jnt_L",            "Rol01_Arm01Low01Jnt_L"},
        {"larm3",       "Rol01_Hand01MasterJnt_L",          "Rol01_Hand01MasterJnt_L"},

        // lhand
        {"lfinger1",    "Rol01_Hand01Thumb01FKCtrlJnt_L",   "Rol01_Hand01Thumb03FKCtrlJnt_L"},
        {"lfinger2",    "Rol01_Hand01Index02FKCtrlJnt_L",   "Rol01_Hand01Index04FKCtrlJnt_L"},
        {"lfinger3",    "Rol01_Hand01Middle02FKCtrlJnt_L",  "Rol01_Hand01Middle04FKCtrlJnt_L"},
        {"lfinger4",    "Rol01_Hand01Ring02FKCtrlJnt_L",    "Rol01_Hand01Ring04FKCtrlJnt_L"},
        {"lfinger5",    "Rol01_Hand01Little02FKCtrlJnt_L",  "Rol01_Hand01Little04FKCtrlJnt_L"},

        // rarm
        {"rram0",       "Rol01_Arm01ClavicleStartJnt_R",    "Rol01_Arm01ClavicleStartJnt_R"},
        {"rram1",       "Rol01_Arm01Up01Jnt_R",             "Rol01_Arm01Up01Jnt_R"},
        {"rram2",       "Rol01_Arm01Low01Jnt_R",            "Rol01_Arm01Low01Jnt_R"},
        {"rram3",       "Rol01_Hand01MasterJnt_R",          "Rol01_Hand01MasterJnt_R"},

        // rhand
        {"rfinger1",    "Rol01_Hand01Thumb01FKCtrlJnt_R",   "Rol01_Hand01Thumb03FKCtrlJnt_R"},
        {"rfinger2",    "Rol01_Hand01Index02FKCtrlJnt_R",   "Rol01_Hand01Index04FKCtrlJnt_R"},
        {"rfinger3",    "Rol01_Hand01Middle02FKCtrlJnt_R",  "Rol01_Hand01Middle04FKCtrlJnt_R"},
        {"rfinger4",    "Rol01_Hand01Ring02FKCtrlJnt_R",    "Rol01_Hand01Ring04FKCtrlJnt_R"},
        {"rfinger5",    "Rol01_Hand01Little02FKCtrlJnt_R",  "Rol01_Hand01Little04FKCtrlJnt_R"},
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
        {true,  false,  "larm0",        "larm0"},
        {true,  false,  "larm1",        "larm1"},
        {true,  false,  "larm2",        "larm2"},
        {true,  false,  "larm3",        "larm3"},

        // lhand
        {true,  false,  "lfinger1",     "lfinger1"},
        {true,  false,  "lfinger2",     "lfinger2"},
        {true,  false,  "lfinger3",     "lfinger3"},
        {true,  false,  "lfinger4",     "lfinger4"},
        {true,  false,  "lfinger5",     "lfinger5"},
        
        // rarm
        {true,  false,  "rram0",        "rram0"},
        {true,  false,  "rram1",        "rram1"},
        {true,  false,  "rram2",        "rram2"},
        {true,  false,  "rram3",        "rram3"},

        // rhand
        {true,  false,  "rfinger1",     "rfinger1"},
        {true,  false,  "rfinger2",     "rfinger2"},
        {true,  false,  "rfinger3",     "rfinger3"},
        {true,  false,  "rfinger4",     "rfinger4"},
        {true,  false,  "rfinger5",     "rfinger5"},
    };

    return config;
}

static SoulIKRigRetargetConfig config_gpt_meta() {
    SoulIKRigRetargetConfig config;
    config.SourceCoord      = CoordType::RightHandZupYfront;
    config.WorkCoord        = CoordType::RightHandZupYfront;
    config.TargetCoord      = CoordType::RightHandYupZfront;

    config.SourceRootType   = ERootType::RootZMinusGroundZ;
    config.TargetRootType   = ERootType::RootZ;

    config.SourceRootBone   = "Pelvis";
    config.SourceGroundBone = "Left_foot";
    config.TargetRootBone   = "Rol01_Torso01HipCtrlJnt_M";
    config.TargetGroundBone = "Rol01_Leg01FootJnt_L";
    
    //config.skipRootBone = true;

    config.SourceChains = {
        // name         start                 end
        // spine
        {"spine",       "Spine1",             "Spine3"},

        // head
        {"head",        "Neck",               "Head"},

        // lleg
        {"lleg1",       "Left_hip",          "Left_hip"},
        {"lleg2",       "Left_knee",         "Left_knee"},
        {"lleg3",       "Left_ankle",        "Left_ankle"},

        // rleg
        {"rleg1",       "Right_hip",         "Right_hip"},
        {"rleg2",       "Right_knee",        "Right_knee"},
        {"rleg3",       "Right_ankle",       "Right_ankle"},

        // larm
        {"larm0",       "Left_collar",       "Left_collar"},
        {"larm1",       "Left_shoulder",     "Left_shoulder"},
        {"larm2",       "Left_elbow",        "Left_elbow"},
        {"larm3",       "Left_wrist",        "Left_wrist"},
        
        // rarm
        {"rram0",       "Right_collar",      "Right_collar"},
        {"rram1",       "Right_shoulder",    "Right_shoulder"},
        {"rram2",       "Right_elbow",       "Right_elbow"},
        {"rram3",       "Right_wrist",       "Right_wrist"},
    };

    config.TargetChains = {
        // name     start                           end
        // spine
        {"spine",       "Rol01_Torso0102Jnt_M",             "Rol01_Neck0101Jnt_M"},

        // head
        {"head",        "Rol01_Neck0102Jnt_M",              "Head_M"},

        // lleg
        {"lleg1",       "Rol01_Leg01Up01Jnt_L",             "Rol01_Leg01Up01Jnt_L"},
        {"lleg2",       "Rol01_Leg01Low01Jnt_L",            "Rol01_Leg01Low01Jnt_L"},
        {"lleg3",       "Rol01_Leg01AnkleJnt_L",            "Rol01_Leg01AnkleJnt_L"},

        // rleg
        {"rleg1",       "Rol01_Leg01Up01Jnt_R",             "Rol01_Leg01Up01Jnt_R"},
        {"rleg2",       "Rol01_Leg01Low01Jnt_R",            "Rol01_Leg01Low01Jnt_R"},
        {"rleg3",       "Rol01_Leg01AnkleJnt_R",            "Rol01_Leg01AnkleJnt_R"},

        // larm
        {"larm0",       "Rol01_Arm01ClavicleStartJnt_L",    "Rol01_Arm01ClavicleStartJnt_L"},
        {"larm1",       "Rol01_Arm01Up01Jnt_L",             "Rol01_Arm01Up01Jnt_L"},
        {"larm2",       "Rol01_Arm01Low01Jnt_L",            "Rol01_Arm01Low01Jnt_L"},
        {"larm3",       "Rol01_Hand01MasterJnt_L",          "Rol01_Hand01MasterJnt_L"},

        // rarm
        {"rram0",       "Rol01_Arm01ClavicleStartJnt_R",    "Rol01_Arm01ClavicleStartJnt_R"},
        {"rram1",       "Rol01_Arm01Up01Jnt_R",             "Rol01_Arm01Up01Jnt_R"},
        {"rram2",       "Rol01_Arm01Low01Jnt_R",            "Rol01_Arm01Low01Jnt_R"},
        {"rram3",       "Rol01_Hand01MasterJnt_R",          "Rol01_Hand01MasterJnt_R"},
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
        {true,  false,  "larm0",        "larm0"},
        {true,  false,  "larm1",        "larm1"},
        {true,  false,  "larm2",        "larm2"},
        {true,  false,  "larm3",        "larm3"},
        
        // rarm
        {true,  false,  "rram0",        "rram0"},
        {true,  false,  "rram1",        "rram1"},
        {true,  false,  "rram2",        "rram2"},
        {true,  false,  "rram3",        "rram3"},
    };

    return config;
}

struct TestCase {
    SoulIKRigRetargetConfig  config;
    std::string srcAnimationFile;
    std::string srcTPoseFile;
    std::string targetFile;
    std::string targetTPoseFile;
    std::string outFile;
    bool isTargetNeedHardCodeTPose{ false };
};

TestCase case_S1SittingDown() {
    
    TestCase testCase;

    testCase.config = config_s1_meta();
    testCase.srcAnimationFile = "S1_SittingDown_3d_17kpts.fbx";
    testCase.srcTPoseFile = "S1_SittingDown_3d_17kpts.fbx";
    testCase.targetFile = "3D_Avatar2_Rig_0723.fbx";
    testCase.targetTPoseFile = "3D_Avatar2_Rig_0723_itpose.fbx";
    testCase.outFile = "out.fbx";
    testCase.isTargetNeedHardCodeTPose = false;

    return testCase;
}

TestCase case_S1Walking() {
    
    TestCase testCase;

    testCase.config = config_s1_meta();
    testCase.srcAnimationFile = "S1_Walking_3d_17kpts.fbx";
    testCase.srcTPoseFile = "S1_Walking_3d_17kpts.fbx";
    testCase.targetFile = "3D_Avatar2_Rig_0723.fbx";
    testCase.targetTPoseFile = "3D_Avatar2_Rig_0723_itpose.fbx";
    testCase.outFile = "out.fbx";
    testCase.isTargetNeedHardCodeTPose = false;

    return testCase;
}

TestCase case_Flair() {
    
    TestCase testCase;

    testCase.config = config_flair_meta(); 
    testCase.srcAnimationFile = "Flair.fbx";
    testCase.srcTPoseFile = "Y_Bot.fbx";
    testCase.targetFile = "3D_Avatar2_Rig_0723.fbx";
    testCase.targetTPoseFile = "3D_Avatar2_Rig_0723_itpose.fbx";
    testCase.outFile = "out.fbx";
    testCase.isTargetNeedHardCodeTPose = false;
    return testCase;
}

TestCase case_gpt() {
    
    TestCase testCase;

    testCase.config = config_gpt_meta(); 
    testCase.srcAnimationFile = "gpt_motion_smpl.fbx";
    testCase.srcTPoseFile = "GPT_T-Pose.fbx";
    testCase.targetFile = "3D_Avatar2_Rig_0723.fbx";
    testCase.targetTPoseFile = "3D_Avatar2_Rig_0723_itpose.fbx";
    testCase.outFile = "out.fbx";
    testCase.isTargetNeedHardCodeTPose = false;
    return testCase;
}

static std::string getModelPath() {
    std::string file_path = __FILE__;
    
    #ifdef _WIN64
        std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
        std::string model_path = dir_path + "\\..\\model\\";
    #else
        std::string dir_path = file_path.substr(0, file_path.rfind("/"));
        std::string model_path = dir_path + "/../model/";
    #endif

    return model_path;
}

static void getFilePaths(std::string& srcAnimationFile, 
    std::string& srcTPoseFile, 
    std::string& targetFile,
    std::string& targetTPoseFile,
    std::string& outfile, 
    TestCase& testcase) {
    
    std::string modelPath = getModelPath();

    srcAnimationFile    = modelPath + testcase.srcAnimationFile;
    srcTPoseFile        = modelPath + testcase.srcTPoseFile;
    targetFile          = modelPath + testcase.targetFile;
    targetTPoseFile     = modelPath + testcase.targetTPoseFile;
    outfile             = modelPath + testcase.outFile;
}

int main(int argc, char *argv[]) {

    /////////////////////////////////////////////
    // setting of coord
    //TestCase testCase       = case_Flair(); 
    //TestCase testCase       = case_S1Walking();
    TestCase testCase       = case_gpt();
    auto config             = testCase.config;

    // printf("%s", config.to_string().c_str());
    // return 0;

    std::string srcAnimationFile, srcTPoseFile, targetFile, targetTPoseFile, outfile;
    getFilePaths(srcAnimationFile, srcTPoseFile, targetFile, targetTPoseFile, outfile, testCase);

    bool ret = retargetFBX(srcAnimationFile, srcTPoseFile, config.SourceRootBone,
        targetFile, targetTPoseFile, outfile, config);

    if (!ret) {
        printf("retargetFBX error\n");
    }

    return 0;
}
