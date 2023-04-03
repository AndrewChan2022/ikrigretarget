//
//  SoulRetargetSkeleton.h
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include "SoulTransform.h"

//#region MARK: -Interfaces
//#endregion
// MARK: - retargeter
#pragma region retargeter

namespace SoulIK
{
    class UObject {
    };

    struct FBoneNode
    {
        std::string name;
        int32_t parent;
        int32_t mode{-1};       // not ignore it
    };

    ////////////////////////////////////////////////////////////////////////
    //   skeleton define

    class USkeleton
    {
        public:
        std::string name;
        std::vector<FBoneNode> boneTree;
        std::vector<FTransform> refpose;    // important: local transform

        std::string GetName() { return name; }
        int32_t GetNum() const {
            return (int32_t)boneTree.size();
        }
        std::string GetBoneName(int32_t idx) const {
            return boneTree[idx].name;
        }
        int32_t GetParentIndex(int32_t idx) const {
            return boneTree[idx].parent;
        }

        std::vector<FTransform>& GetRefBonePose() {
            return refpose;
        }
    };

    ////////////////////////////////////////////////////////////////////////
    //   setting
    enum class ERetargetRotationMode : uint8_t
    {
        
        Interpolated, 
        OneToOne, 
        OneToOneReversed, 
        None
    };

    enum class ERetargetTranslationMode : uint8_t
    {
        None,
        GloballyScaled,
        Absolute,
    };
    struct FTargetChainFKSettings
    {
        bool EnableFK = true;
        ERetargetRotationMode RotationMode = ERetargetRotationMode::Interpolated;
        float RotationAlpha = 1.0f;
        ERetargetTranslationMode TranslationMode = ERetargetTranslationMode::None;
        float TranslationAlpha = 1.0f;
        float PoleVectorMatching = 0.0f;
        float PoleVectorOffset = 0.0f;
        //bool operator==(const FTargetChainFKSettings& Other) const;
    };
    struct FTargetChainIKSettings
    {
        bool EnableIK = true;
        float BlendToSource = 0.0f;
        FVector BlendToSourceWeights = FVector::OneVector;
        FVector StaticOffset = FVector::ZeroVector;
        FVector StaticLocalOffset = FVector::ZeroVector;
        //FRotator StaticRotationOffset = FRotator::ZeroRotator;
        float Extension = 1.0f;
        bool bAffectedByIKWarping = true;
        //bool operator==(const FTargetChainIKSettings& Other) const;
    };
    struct FTargetChainSpeedPlantSettings
    {
        // The name of the curve on the source animation that contains the speed of the end effector bone
        // 0.0 ~ 100.0
        bool EnableSpeedPlanting = false;
        
        // The name of the curve on the source animation that contains the speed of the end effector bone.
        // 0.0 ~ 100.0
        FName SpeedCurveName;

        // Range 0 to 1000. Default 15. The maximum speed a source bone can be moving while being considered 'planted'.
	    // The target IK goal will not be allowed to move whenever the source bone speed drops below this threshold speed
        // 0 ~ 100
        float SpeedThreshold = 15.0f;

        // How stiff the spring model is that smoothly pulls the IK position after unplanting (more stiffness means more oscillation around the target value)
        // 0 ~ inf
        float UnplantStiffness = 250.0f;

        // How much damping to apply to the spring (0 means no damping, 1 means critically damped which means no oscillation)
        // 0 ~ 10
        float UnplantCriticalDamping = 1.0f;
        //bool operator==(const FTargetChainSpeedPlantSettings& Other) const;
    };
    struct FTargetChainSettings
    {
    public:
        FTargetChainFKSettings FK;
        FTargetChainIKSettings IK;
        FTargetChainSpeedPlantSettings SpeedPlanting;
        
        //void CopySettingsFromAsset(const URetargetChainSettings* AssetChainSettings);
        //bool operator==(const FTargetChainSettings& Other) const;
    };
    class URetargetChainSettings : public UObject
    {
    public:
        URetargetChainSettings() = default;
        URetargetChainSettings(const FName& TargetChain) : TargetChain(TargetChain){}
        
        FName SourceChain;
        FName TargetChain;
        FTargetChainSettings Settings;
    };

    struct FTargetRootSettings
    {
    public:
        float RotationAlpha = 1.0f;
        float TranslationAlpha = 1.0f;
        float BlendToSource = 0.0f;
        FVector BlendToSourceWeights = FVector::OneVector;
        float ScaleHorizontal = 1.0f;
        float ScaleVertical = 1.0f;
        FVector TranslationOffset = FVector::ZeroVector;
        FRotator RotationOffset = FRotator::ZeroRotator;
        float AffectIKHorizontal = 1.0f;
        float AffectIKVertical = 0.0f;
    };
    class URetargetRootSettings: public UObject {
        FTargetRootSettings Settings;
    private:
        bool RetargetRootTranslation_DEPRECATED = true;
        float GlobalScaleHorizontal_DEPRECATED = 1.0f;
        float GlobalScaleVertical_DEPRECATED = 1.0f;
        FVector BlendToSource_DEPRECATED = FVector::ZeroVector;
        FVector StaticOffset_DEPRECATED = FVector::ZeroVector;
        FRotator StaticRotationOffset_DEPRECATED = FRotator::ZeroRotator;
    };    
    struct FRetargetGlobalSettings
    {
        enum class EBasicAxis {
            X,
            Y,
            Z,
            NegX,
            NegY,
            NegZ
        };

        enum class EWarpingDirectionSource {
            Goals,
            Chain,
        };

        bool bEnableRoot = true;
        bool bEnableFK = true;
        bool bEnableIK = true;
        bool bWarping = false;

        EWarpingDirectionSource DirectionSource = EWarpingDirectionSource::Goals;
        EBasicAxis ForwardDirection = EBasicAxis::Y;

        FName DirectionChain;
        float WarpForwards = 1.0f;
        float SidewaysOffset = 0.0f;
        float WarpSplay = 1.0f;

        static FVector GetAxisVector(const EBasicAxis& Axis)
        {
            switch (Axis)
            {
            case EBasicAxis::X:
                return FVector::XAxisVector;
            case EBasicAxis::Y:
                return FVector::YAxisVector;
            case EBasicAxis::Z:
                return FVector::ZAxisVector;
            case EBasicAxis::NegX:
                return -FVector::XAxisVector;
            case EBasicAxis::NegY:
                return -FVector::YAxisVector;
            case EBasicAxis::NegZ:
                return -FVector::ZAxisVector;
            default:
                checkNoEntry();
                return FVector::ZeroVector;
            }
        }
    };

    class UIKRetargetGlobalSettings: public UObject
    {
    public:
        FRetargetGlobalSettings Settings;
    };

    ////////////////////////////////////////////////////////////////////////
    //   retargeter define
    struct FIKRigSkeleton
    {
        TArray<FName> BoneNames;
        TArray<int32> ParentIndices;
        TArray<FName> ExcludedBones;
        TArray<FTransform> CurrentPoseGlobal;
        TArray<FTransform> CurrentPoseLocal;
        TArray<FTransform> RefPoseGlobal;
    };

    struct FBoneReference
    {
        FBoneReference()
            : BoneName(""), BoneIndex(-1), bUseSkeletonIndex(false)
        {
        }
        FBoneReference(std::string name)
            : BoneName(name), BoneIndex(-1), bUseSkeletonIndex(false)
        {
        }
        FBoneReference(std::string name, int32_t index)
            : BoneName(name), BoneIndex(index), bUseSkeletonIndex(false)
        {
        }

        std::string BoneName;
        int32_t BoneIndex : 31;
        uint32_t bUseSkeletonIndex : 1;
    };

    struct FBoneChain
    {
        FBoneChain()
            : ChainName(""), StartBone(""), EndBone(""), IKGoalName("")
        {
        }
        FBoneChain(std::string InName, const std::string &InStartBone, const std::string &InEndBone)
            : ChainName(InName), StartBone(InStartBone), EndBone(InEndBone), IKGoalName("")
        {
        }
        FName ChainName;
        FBoneReference StartBone;
        FBoneReference EndBone;
        std::string IKGoalName;
    };

    class URetargetDefinition
    {
    public:
        std::string RootBone;
        std::string GroundBone;         // RootBone.z - GroundBone.z = height
        TArray<FBoneChain> BoneChains;

        friend class UIKRigDefinition;
    };

    class UIKRigDefinition
    {
    public:
        FIKRigSkeleton Skeleton;
        // TArray<TObjectPtr<UIKRigEffectorGoal>> Goals;
        // TArray<TObjectPtr<UIKRigSolver>> Solvers;
        URetargetDefinition RetargetDefinition;

        public:
        const TArray<FBoneChain>& GetRetargetChains() const { return RetargetDefinition.BoneChains; };
        const FName& GetRetargetRoot() const { return RetargetDefinition.RootBone; };
        const FName& GetRetargetGround() const { return RetargetDefinition.GroundBone; };
        const FBoneChain* UIKRigDefinition::GetRetargetChainByName(FName ChainName) const
        {
            for (const FBoneChain& Chain : RetargetDefinition.BoneChains)
            {
                if (Chain.ChainName == ChainName)
                {
                    return &Chain;
                }
            }
            return nullptr;
        }

    };

    class UIKRetargeter
    {
    public:
        std::shared_ptr<UIKRigDefinition> SourceIKRigAsset; // skeleton/root/ground/bonechains
        std::shared_ptr<UIKRigDefinition> TargetIKRigAsset; // skeleton/root/ground/bonechains

        URetargetRootSettings RootSetting;                              // root setting
        TArray<std::shared_ptr<URetargetChainSettings>> ChainSettings;  // chain mappings
        std::shared_ptr<UIKRetargetGlobalSettings> GlobalSettings;


        UIKRigDefinition* GetSourceIKRig() {
            return SourceIKRigAsset.get();
        }
        UIKRigDefinition* GetTargetIKRig() {
            return TargetIKRigAsset.get();
        }
        const TArray<std::shared_ptr<URetargetChainSettings>>& GetAllChainSettings() const { return ChainSettings; };
    };
}