
import sys, os
import ikrigretarget as irr

# print(irr.myadd(1,2))

def config_gpt_meta():

    config = irr.SoulIKRigRetargetConfig()

    config.SourceCoord      = irr.CoordType.RightHandZupYfront
    config.WorkCoord        = irr.CoordType.RightHandZupYfront
    config.TargetCoord      = irr.CoordType.RightHandYupZfront

    config.SourceRootType   = irr.ERootType.RootZMinusGroundZ
    config.TargetRootType   = irr.ERootType.RootZ

    config.SourceRootBone   = "Pelvis"
    config.SourceGroundBone = "Left_foot"
    config.TargetRootBone   = "Rol01_Torso01HipCtrlJnt_M"
    config.TargetGroundBone = "Rol01_Leg01FootJnt_L"


    ### source chain
    # name         start                 end
    config.SourceChains.append(irr.SoulIKRigChain("spine",       "Spine1",             "Spine3"))
    config.SourceChains.append(irr.SoulIKRigChain("head",        "Neck",               "Head"))

    # lleg
    config.SourceChains.append(irr.SoulIKRigChain("lleg1",       "Left_hip",          "Left_hip"))
    config.SourceChains.append(irr.SoulIKRigChain("lleg2",       "Left_knee",         "Left_knee"))
    config.SourceChains.append(irr.SoulIKRigChain("lleg3",       "Left_ankle",        "Left_ankle"))

    # rleg
    config.SourceChains.append(irr.SoulIKRigChain("rleg1",       "Right_hip",         "Right_hip"))
    config.SourceChains.append(irr.SoulIKRigChain("rleg2",       "Right_knee",        "Right_knee"))
    config.SourceChains.append(irr.SoulIKRigChain("rleg3",       "Right_ankle",       "Right_ankle"))

    # larm
    config.SourceChains.append(irr.SoulIKRigChain("larm0",       "Left_collar",       "Left_collar"))
    config.SourceChains.append(irr.SoulIKRigChain("larm1",       "Left_shoulder",     "Left_shoulder"))
    config.SourceChains.append(irr.SoulIKRigChain("larm2",       "Left_elbow",        "Left_elbow"))
    config.SourceChains.append(irr.SoulIKRigChain("larm3",       "Left_wrist",        "Left_wrist"))

    # rarm
    config.SourceChains.append(irr.SoulIKRigChain("larm0",       "Left_collar",       "Left_collar"))
    config.SourceChains.append(irr.SoulIKRigChain("rram1",       "Right_shoulder",    "Right_shoulder"))
    config.SourceChains.append(irr.SoulIKRigChain("rram2",       "Right_elbow",       "Right_elbow"))
    config.SourceChains.append(irr.SoulIKRigChain("rram3",       "Right_wrist",       "Right_wrist"))

    ### source chain
    config.TargetChains.append(irr.SoulIKRigChain("spine",       "Rol01_Torso0102Jnt_M",             "Rol01_Neck0101Jnt_M"))
    config.TargetChains.append(irr.SoulIKRigChain("head",        "Rol01_Neck0102Jnt_M",              "Head_M"))

    # lleg
    config.TargetChains.append(irr.SoulIKRigChain("lleg1",       "Rol01_Leg01Up01Jnt_L",             "Rol01_Leg01Up01Jnt_L"))
    config.TargetChains.append(irr.SoulIKRigChain("lleg2",       "Rol01_Leg01Low01Jnt_L",            "Rol01_Leg01Low01Jnt_L"))
    config.TargetChains.append(irr.SoulIKRigChain("lleg3",       "Rol01_Leg01AnkleJnt_L",            "Rol01_Leg01AnkleJnt_L"))

    # rleg
    config.TargetChains.append(irr.SoulIKRigChain("rleg1",       "Rol01_Leg01Up01Jnt_R",             "Rol01_Leg01Up01Jnt_R"))
    config.TargetChains.append(irr.SoulIKRigChain("rleg2",       "Rol01_Leg01Low01Jnt_R",            "Rol01_Leg01Low01Jnt_R"))
    config.TargetChains.append(irr.SoulIKRigChain("rleg3",       "Rol01_Leg01AnkleJnt_R",            "Rol01_Leg01AnkleJnt_R"))
    
    # larm
    config.TargetChains.append(irr.SoulIKRigChain("larm0",       "Rol01_Arm01ClavicleStartJnt_L",    "Rol01_Arm01ClavicleStartJnt_L"))
    config.TargetChains.append(irr.SoulIKRigChain("larm1",       "Rol01_Arm01Up01Jnt_L",             "Rol01_Arm01Up01Jnt_L"))
    config.TargetChains.append(irr.SoulIKRigChain("larm2",       "Rol01_Arm01Low01Jnt_L",            "Rol01_Arm01Low01Jnt_L"))
    config.TargetChains.append(irr.SoulIKRigChain("larm3",       "Rol01_Hand01MasterJnt_L",          "Rol01_Hand01MasterJnt_L"))

    # rarm
    config.TargetChains.append(irr.SoulIKRigChain("rram0",       "Rol01_Arm01ClavicleStartJnt_R",    "Rol01_Arm01ClavicleStartJnt_R"))
    config.TargetChains.append(irr.SoulIKRigChain("rram1",       "Rol01_Arm01Up01Jnt_R",             "Rol01_Arm01Up01Jnt_R"))
    config.TargetChains.append(irr.SoulIKRigChain("rram2",       "Rol01_Arm01Low01Jnt_R",            "Rol01_Arm01Low01Jnt_R"))
    config.TargetChains.append(irr.SoulIKRigChain("rram3",       "Rol01_Hand01MasterJnt_R",          "Rol01_Hand01MasterJnt_R"))

    ### chain mapping
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "spine",        "spine"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "head",         "head"))

    # lleg
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "lleg1",        "lleg1"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "lleg2",        "lleg2"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "lleg3",        "lleg3"))

    # rleg
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "rleg1",        "rleg1"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "rleg2",        "rleg2"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "rleg3",        "rleg3"))

    # larm
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "larm0",        "larm0"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "larm1",        "larm1"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "larm2",        "larm2"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "larm3",        "larm3"))

    # rarm
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "rarm0",        "rarm0"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "rarm1",        "rarm1"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "rarm2",        "rarm2"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True,  False,  "rarm3",        "rarm3"))

    return config

def test():

    a = irr.CoordType.RightHandZupYfront
    print("a:", a)


    config = irr.SoulIKRigRetargetConfig()

    # coord
    config.SourceCoord = irr.CoordType.RightHandZupYfront
    config.WorkCoord = irr.CoordType.RightHandZupYfront
    config.TargetCoord = irr.CoordType.RightHandZupYfront

    # root 
    config.SourceRootType = irr.ERootType.RootZ
    config.SourceRootBone = "hip"
    config.SourceGroundBone = ""
    config.TargetRootType = irr.ERootType.RootZ
    config.TargetRootBone = "hip"
    config.TargetGroundBone = ""

    # source chain
    config.SourceChains.append(irr.SoulIKRigChain("name0", "start0", "end1"))
    config.SourceChains.append(irr.SoulIKRigChain("name1", "start1", "end1"))

    # config.SourceChains = [
    #     irr.SoulIKRigChain("name0", "start0", "end1"),
    #     irr.SoulIKRigChain("name1", "start1", "end1")
    # ]

    # target chain
    config.TargetChains.append(irr.SoulIKRigChain("name0", "start0", "end1"))
    config.TargetChains.append(irr.SoulIKRigChain("name1", "start1", "end1"))

    # chain mapping
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True, False, "larm", "larm"))
    config.ChainMapping.append(irr.SoulIKRigChainMapping(True, False, "rarm", "rarm"))


    config.IntArray.append(1)
    config.IntArray.append(2)
    config.IntArray.append(3)

    print(config)


    print("IntArray:", len(config.IntArray))
    for i in config.IntArray:
        print(i)

def get_model_path():
    curdir = os.path.dirname(os.path.abspath(__file__))
    curdir = os.path.join(curdir, "../model")
    return curdir

def to_model_path(filename):
    return os.path.join(get_model_path(), filename)

if __name__ == "__main__":
    
    config = config_gpt_meta()

    srcAnimationFile = to_model_path("gpt_motion_smpl.fbx")
    srcTPoseFile = to_model_path("GPT_T-Pose.fbx")
    targetFile = to_model_path("3D_Avatar2_Rig_0723.fbx")
    targetTPoseFile = to_model_path("3D_Avatar2_Rig_0723_itpose.fbx")
    outFile = to_model_path("out.fbx")

    # irr.retargetFBX(srcAnimationFile, srcTPoseFile, config.SourceRootBone, targetFile, targetTPoseFile, outfile, config)

