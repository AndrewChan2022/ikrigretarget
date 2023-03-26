//
//  SoulIKRetargetProcessor.h
//  
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include <stdarg.h>
#include "SoulRetargeter.h"


namespace Soul {



struct FIKRigLogger {

	// c style
	void LogError(char* fmt, ...) { 
		va_list arguments;
    	va_start(arguments, fmt);
		// You cannot pass the variadic arguments to a variadic function
		//  Instead, you must call a function that takes a va_list as argument.
		vprintf(fmt, arguments);
		printf("\n");
		va_end(arguments);
	}
	
	// c++ style
	template <class ... Args>
	void LogWarning(const char* format, Args ... args) {
		printf(format, args...);
		printf("\n");
	}

	// macro style #define myprintf(fmt, ...)  printf(fmt, __VA_ARGS__)
};



////////////////////////////////////////////////////////////////////////
//   retarget skeleton
struct FIKRetargetPose
{
public:
	FIKRetargetPose() = default;
	FQuat GetDeltaRotationForBone(const FName BoneName) const;
	void SetDeltaRotationForBone(FName BoneName, const FQuat& RotationDelta);
	const std::unordered_map<FName, FQuat>& GetAllDeltaRotations() const { return BoneRotationOffsets; };

	FVector GetRootTranslationDelta() const;
	void SetRootTranslationDelta(const FVector& TranslationDelta);
	void AddToRootTranslationDelta(const FVector& TranslationDelta);
	
	void SortHierarchically(const FIKRigSkeleton& Skeleton);

private:
	FVector RootTranslationOffset = FVector::ZeroVector;
	std::unordered_map<FName, FQuat> BoneRotationOffsets;
};

struct FRetargetSkeleton
{
	std::vector<FName> BoneNames;				// list of all bone names in ref skeleton order
	std::vector<int32_t> ParentIndices;			// per-bone indices of parent bones (the hierarchy)
	std::vector<FTransform> RetargetLocalPose;	// local space retarget pose
	std::vector<FTransform> RetargetGlobalPose;	// global space retarget pose
	FName RetargetPoseName;						// the name of the retarget pose this was initialized with
	USkeleton* Skeleton;							// the skeletal mesh this was initialized with
	std::vector<FName> ChainThatContainsBone;	// record which chain is actually controlling each bone

	void Initialize(
		USkeleton* InSkeleton,
		const std::vector<FBoneChain>& BoneChains,
		const FName InRetargetPoseName,
		const FIKRetargetPose* RetargetPose,
		const FName RetargetRootBone);

	void Reset();

	void GenerateRetargetPose(
		const FName InRetargetPoseName,
		const FIKRetargetPose* InRetargetPose,
		const FName RetargetRootBone);

	int32_t FindBoneIndexByName(const FName InName) const;

	int32_t GetParentIndex(const int32_t BoneIndex) const;

	void UpdateGlobalTransformsBelowBone(
		const int32_t StartBoneIndex,
		const std::vector<FTransform>& InLocalPose,
		std::vector<FTransform>& OutGlobalPose) const;

	void UpdateLocalTransformsBelowBone(
		const int32_t StartBoneIndex,
		std::vector<FTransform>& OutLocalPose,
		const std::vector<FTransform>& InGlobalPose) const;
	
	void UpdateGlobalTransformOfSingleBone(
		const int32_t BoneIndex,
		const std::vector<FTransform>& InLocalPose,
		std::vector<FTransform>& OutGlobalPose) const;
	
	void UpdateLocalTransformOfSingleBone(
		const int32_t BoneIndex,
		std::vector<FTransform>& OutLocalPose,
		const std::vector<FTransform>& InGlobalPose) const;

	FTransform GetGlobalRefPoseOfSingleBone(
		const int32_t BoneIndex,
		const std::vector<FTransform>& InGlobalPose) const;

	int32_t GetCachedEndOfBranchIndex(const int32_t InBoneIndex) const;

	void GetChildrenIndices(const int32_t BoneIndex, std::vector<int32_t>& OutChildren) const;

	void GetChildrenIndicesRecursive(const int32_t BoneIndex, std::vector<int32_t>& OutChildren) const;
	
	bool IsParentOfChild(const int32_t PotentialParentIndex, const int32_t ChildBoneIndex) const;

private:
	
	/** One index per-bone. Lazy-filled on request. Stores the last element of the branch below the bone.
	 * You can iterate between in the indices stored here and the bone in question to iterate over all children recursively */
	mutable std::vector<int32_t> CachedEndOfBranchIndices;
};

struct FTargetSkeleton : public FRetargetSkeleton
{
	std::vector<FTransform> OutputGlobalPose;
	// true for bones that are in a target chain that is ALSO mapped to a source chain
	// ie, bones that are actually posed based on a mapped source chain
	std::vector<bool> IsBoneRetargeted;

	void Initialize(
		USkeleton* InSkeletalMesh,
		const std::vector<FBoneChain>& BoneChains,
		const FName InRetargetPoseName,
		const FIKRetargetPose* RetargetPose,
		const FName RetargetRootBone);

	void Reset();

	void SetBoneIsRetargeted(const int32_t BoneIndex, const bool IsRetargeted);

	void UpdateGlobalTransformsAllNonRetargetedBones(std::vector<FTransform>& InOutGlobalPose);
};


////////////////////////////////////////////////////////////////////////
//   root retargeter

// resolving an FBoneChain to an actual skeleton, used to validate compatibility and get all chain indices
struct FResolvedBoneChain
{
	FResolvedBoneChain(const FBoneChain& BoneChain, const FRetargetSkeleton& Skeleton, TArray<int32> &OutBoneIndices);

	/* Does the START bone exist in the skeleton? */
	bool bFoundStartBone = false;
	/* Does the END bone exist in the skeleton? */
	bool bFoundEndBone = false;
	/* Is the END bone equals or a child of the START bone? */
	bool bEndIsStartOrChildOfStart  = false;

	bool IsValid() const
	{
		return bFoundStartBone && bFoundEndBone && bEndIsStartOrChildOfStart;
	}
};

struct FRootSource
{
	FName BoneName;
	int32 BoneIndex;
	FQuat InitialRotation;
	float InitialHeightInverse;
	FVector InitialPosition;
	FVector CurrentPosition;
	FVector CurrentPositionNormalized;
	FQuat CurrentRotation;
};

struct FRootTarget
{
	FName BoneName;
	int32 BoneIndex;
	FVector InitialPosition;
	FQuat InitialRotation;
	float InitialHeight;

	FVector RootTranslationDelta;
	FQuat RootRotationDelta;
};

struct FRootRetargeter
{	
	FRootSource Source;
	FRootTarget Target;
	FTargetRootSettings Settings;

	void Reset();
	
	bool InitializeSource(
		const FName SourceRootBoneName,
		const FRetargetSkeleton& SourceSkeleton,
		FIKRigLogger& Log);
	
	bool InitializeTarget(
		const FName TargetRootBoneName,
		const FTargetSkeleton& TargetSkeleton,
		FIKRigLogger& Log);

	void EncodePose(const TArray<FTransform> &SourceGlobalPose);
	
	void DecodePose(TArray<FTransform> &OutTargetGlobalPose);

	FVector GetGlobalScaleVector() const
	{
		return GlobalScaleFactor * FVector(Settings.ScaleHorizontal, Settings.ScaleHorizontal, Settings.ScaleVertical);
	}

private:
	FVector GlobalScaleFactor;
};


////////////////////////////////////////////////////////////////////////
//   chain retargeter
struct FChainFK
{
	TArray<FTransform> InitialGlobalTransforms;

	TArray<FTransform> InitialLocalTransforms;

	TArray<FTransform> CurrentGlobalTransforms;

	TArray<float> Params;
	TArray<int32_t> BoneIndices;

	int32_t ChainParentBoneIndex;
	FTransform ChainParentInitialGlobalTransform;

	bool Initialize(
		const FRetargetSkeleton& Skeleton,
		const TArray<int32_t>& InBoneIndices,
		const TArray<FTransform> &InitialGlobalPose,
		FIKRigLogger& Log);

private:
	
	bool CalculateBoneParameters(FIKRigLogger& Log);

protected:

	static void FillTransformsWithLocalSpaceOfChain(
		const FRetargetSkeleton& Skeleton,
		const TArray<FTransform>& InGlobalPose,
		const TArray<int32>& BoneIndices,
		TArray<FTransform>& OutLocalTransforms);

	void PutCurrentTransformsInRefPose(
		const TArray<int32_t>& BoneIndices,
		const FRetargetSkeleton& Skeleton,
		const TArray<FTransform>& InCurrentGlobalPose);
};

struct FChainEncoderFK : public FChainFK
{
	TArray<FTransform> CurrentLocalTransforms;

	FTransform ChainParentCurrentGlobalTransform;
	
	void EncodePose(
		const FRetargetSkeleton& SourceSkeleton,
		const TArray<int32>& SourceBoneIndices,
		const TArray<FTransform> &InSourceGlobalPose);

	void TransformCurrentChainTransforms(const FTransform& NewParentTransform);
};

struct FChainDecoderFK : public FChainFK
{
	void InitializeIntermediateParentIndices(
		const int32_t RetargetRootBoneIndex,
		const int32_t ChainRootBoneIndex,
		const FTargetSkeleton& TargetSkeleton);
	
	void DecodePose(
		const FRootRetargeter& RootRetargeter,
		const FTargetChainSettings& Settings,
		const TArray<int32_t>& TargetBoneIndices,
		FChainEncoderFK& SourceChain,
		const FTargetSkeleton& TargetSkeleton,
		TArray<FTransform> &InOutGlobalPose);

	void MatchPoleVector(
		const FTargetChainSettings& Settings,
		const TArray<int32_t>& TargetBoneIndices,
		FChainEncoderFK& SourceChain,
		const FTargetSkeleton& TargetSkeleton,
		TArray<FTransform> &InOutGlobalPose);

private:
	
	FTransform GetTransformAtParam(
		const TArray<FTransform>& Transforms,
		const TArray<float>& InParams,
		const float& Param) const;
	
	void UpdateIntermediateParents(
		const FTargetSkeleton& TargetSkeleton,
		TArray<FTransform> &InOutGlobalPose);

	TArray<int32> IntermediateParentIndices;
};

struct FDecodedIKChain
{
	FVector EndEffectorPosition = FVector::ZeroVector;
	FQuat EndEffectorRotation = FQuat::Identity;
	FVector PoleVectorPosition = FVector::ZeroVector;
};

struct FSourceChainIK
{
	int32 BoneIndexA = INDEX_NONE;
	int32 BoneIndexB = INDEX_NONE;
	int32 BoneIndexC = INDEX_NONE;
	
	FVector InitialEndPosition = FVector::ZeroVector;
	FQuat InitialEndRotation = FQuat::Identity;
	float InvInitialLength = 1.0f;

	// results after encoding...
	FVector PreviousEndPosition = FVector::ZeroVector;
	FVector CurrentEndPosition = FVector::ZeroVector;
	FVector CurrentEndDirectionNormalized = FVector::ZeroVector;
	FQuat CurrentEndRotation = FQuat::Identity;
	float CurrentHeightFromGroundNormalized = 0.0f;
	FVector PoleVectorDirection = FVector::ZeroVector;
};

struct FTargetChainIK
{
	int32 BoneIndexA = INDEX_NONE;
	int32 BoneIndexC = INDEX_NONE;
	
	float InitialLength = 1.0f;
	FVector InitialEndPosition = FVector::ZeroVector;
	FQuat InitialEndRotation = FQuat::Identity;
	FVector PrevEndPosition = FVector::ZeroVector;
};

struct FChainRetargeterIK
{
	FSourceChainIK Source;
	FTargetChainIK Target;
	
	FDecodedIKChain Results;

	bool ResetThisTick;
	//FVectorSpringState PlantingSpringState;

	bool InitializeSource(
		const TArray<int32>& BoneIndices,
		const TArray<FTransform> &SourceInitialGlobalPose,
		FIKRigLogger& Log);
	
	bool InitializeTarget(
		const TArray<int32>& BoneIndices,
		const TArray<FTransform> &TargetInitialGlobalPose,
		FIKRigLogger& Log);

	void EncodePose(const TArray<FTransform> &SourceInputGlobalPose);
	
	void DecodePose(
		const FTargetChainSettings& Settings,
		const FRootRetargeter& RootRetargeter,
		const std::unordered_map<FName, float>& SpeedValuesFromCurves,
		const float DeltaTime,
		const TArray<FTransform>& InGlobalPose);

	void SaveDebugInfo(const TArray<FTransform>& InGlobalPose);
};

struct FRetargetChainPair
{
	FTargetChainSettings Settings;
	
	TArray<int32> SourceBoneIndices;
	TArray<int32> TargetBoneIndices;
	
	FName SourceBoneChainName;
	FName TargetBoneChainName;

	virtual ~FRetargetChainPair() = default;
	
	virtual bool Initialize(
		const FBoneChain& SourceBoneChain,
		const FBoneChain& TargetBoneChain,
		const FRetargetSkeleton& SourceSkeleton,
		const FTargetSkeleton& TargetSkeleton,
		FIKRigLogger& Log);

private:

	bool ValidateBoneChainWithSkeletalMesh(
		const bool IsSource,
		const FBoneChain& BoneChain,
		const FRetargetSkeleton& RetargetSkeleton,
		FIKRigLogger& Log);
};

struct FRetargetChainPairFK : FRetargetChainPair
{
	FChainEncoderFK FKEncoder;
	FChainDecoderFK FKDecoder;
	//FPoleVectorMatcher PoleVectorMatcher;
	
	virtual bool Initialize(
        const FBoneChain& SourceBoneChain,
        const FBoneChain& TargetBoneChain,
        const FRetargetSkeleton& SourceSkeleton,
        const FTargetSkeleton& TargetSkeleton,
        FIKRigLogger& Log) override;
};

struct FRetargetChainPairIK : FRetargetChainPair
{
	//FChainRetargeterIK IKChainRetargeter;
	FName IKGoalName;
	FName PoleVectorGoalName;

	virtual bool Initialize(
        const FBoneChain& SourceBoneChain,
        const FBoneChain& TargetBoneChain,
        const FRetargetSkeleton& SourceSkeleton,
        const FTargetSkeleton& TargetSkeleton,
        FIKRigLogger& Log) override;
};


class UIKRetargetProcessor : public UObject
{
public:

	UIKRetargetProcessor();
	
	// Initialize the retargeter to enable running it.
	// @param SourceSkeleton - the skeletal mesh to poses FROM
	// @param TargetSkeleton - the skeletal mesh to poses TO
	// @param InRetargeterAsset - the source asset to use for retargeting settings
	// @param bSuppressWarnings - if true, will not output warnings during initialization
	// @warning - Initialization does a lot of validation and can fail for many reasons. Check bIsLoadedAndValid afterwards.
	void Initialize(
		USkeleton *SourceSkeleton,
		USkeleton *TargetSkeleton,
		UIKRetargeter* InRetargeterAsset,
		const bool bSuppressWarnings=false);

	
	// Run the retarget to generate a new pose.
	// @param InSourceGlobalPose -  is the source mesh input pose in Component/Global space
	// @return The retargeted Component/Global space pose for the target skeleton
	TArray<FTransform>& RunRetargeter(
		const TArray<FTransform>& InSourceGlobalPose,
		const std::unordered_map<FName, float>& SpeedValuesFromCurves,
		const float DeltaTime);

	// logging system
	FIKRigLogger Log;

private:

	// init
	bool InitializeRoots();
	bool InitializeBoneChainPairs();
	bool InitializeIKRig(UObject* Outer, const USkeleton* InSkeleton);
	
	// run
	void RunRootRetarget(const TArray<FTransform>& InGlobalTransforms, TArray<FTransform>& OutGlobalTransforms);
	void RunFKRetarget(const TArray<FTransform>& InGlobalTransforms, TArray<FTransform>& OutGlobalTransforms);
	void RunIKRetarget(
		const TArray<FTransform>& InSourceGlobalPose,
		TArray<FTransform>& OutTargetGlobalPose,
		const std::unordered_map<FName, float>& SpeedValuesFromCurves,
		const float DeltaTime);
	void RunPoleVectorMatching(const TArray<FTransform>& InGlobalTransforms, TArray<FTransform>& OutGlobalTransforms);
	// Runs in the after the base IK retarget to apply stride warping to IK goals.
	void RunStrideWarping(const TArray<FTransform>& InTargeGlobalPose);

private:

	bool bIsInitialized = false;
	bool bRootsInitialized = false;
	bool bAtLeastOneValidBoneChainPair = false;
	bool bIKRigInitialized = false;
	UIKRetargeter* RetargeterAsset = nullptr;

	// data structure for retarget
	FRootRetargeter RootRetargeter;
	FRetargetSkeleton SourceSkeleton;
	FTargetSkeleton TargetSkeleton;
	TArray<FRetargetChainPairFK> ChainPairsFK;
	TArray<FRetargetChainPairIK> ChainPairsIK;
	//TObjectPtr<UIKRigProcessor> IKRigProcessor = nullptr;

	// setting
	FRetargetGlobalSettings GlobalSettings;
};

}
