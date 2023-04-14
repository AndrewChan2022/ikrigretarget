//
//  IKRetargetProcessor.cpp
//  
//
//  Created by kai chen on 3/24/23.
//

#include "SoulIKRetargetProcessor.h"
#include <algorithm>
#include <tuple>

// #define DEBUG_POSE_LOG
// #define DEBUG_POSE_LOG_CHAINFK
// #define DEBUG_POSE_LOG_ROOT

#define RETARGETSKELETON_INVALID_BRANCH_INDEX -2

using namespace SoulIK;


FVector FIKRetargetPose::GetRootTranslationDelta() const
{
	return RootTranslationOffset;
}

void FRetargetSkeleton::Initialize(
	USkeleton* InSkeleton,
	const TArray<FBoneChain>& BoneChains,
	const FName InRetargetPoseName,
	const FIKRetargetPose* RetargetPose,
	const FName RetargetRootBone)
{
	// reset all skeleton data
	Reset();
	
	// record which skeletal mesh this is running on
	Skeleton = InSkeleton;
	
	// copy names and parent indices into local storage
	const USkeleton* RefSkeleton = Skeleton;
	for (int32 BoneIndex=0; BoneIndex<RefSkeleton->GetNum(); ++BoneIndex)
	{
		BoneNames.push_back(RefSkeleton->GetBoneName(BoneIndex));
		ParentIndices.push_back(RefSkeleton->GetParentIndex(BoneIndex));	
	}

	// determine set of bones referenced by one of the retarget bone chains
	// this is the set of bones that will be affected by the retarget pose
	ChainThatContainsBone.resize(BoneNames.size(), NAME_None);
	for (const FBoneChain& BoneChain : BoneChains)
	{
		TArray<int32> BonesInChain;
		if (FResolvedBoneChain(BoneChain, *this, BonesInChain).IsValid())
		{
			for (const int32 BoneInChain : BonesInChain)
			{
				ChainThatContainsBone[BoneInChain] = BoneChain.ChainName;
			}
		}
	}

	// initialize branch caching
	//CachedEndOfBranchIndices.Init(RETARGETSKELETON_INVALID_BRANCH_INDEX, ParentIndices.Num());

	// update retarget pose to reflect custom offsets (applies stored offsets)
	// NOTE: this must be done AFTER generating IsBoneInAnyTargetChain array above
	GenerateRetargetPose(InRetargetPoseName, RetargetPose, RetargetRootBone);
}

void FRetargetSkeleton::Reset()
{
	BoneNames.clear();
	ParentIndices.clear();
	RetargetLocalPose.clear();
	RetargetGlobalPose.clear();
	Skeleton = nullptr;
}

void FRetargetSkeleton::GenerateRetargetPose(
	const FName InRetargetPoseName,
	const FIKRetargetPose* InRetargetPose,
	const FName RetargetRootBone)
{
	// record the name of the retarget pose (prevents re-initialization if profile swaps it)
	RetargetPoseName = InRetargetPoseName;
	
	// initialize retarget pose to the skeletal mesh reference pose
	RetargetLocalPose = Skeleton->GetRefBonePose();
	// copy local pose to global
	RetargetGlobalPose = RetargetLocalPose;
	// convert to global space
	UpdateGlobalTransformsBelowBone(-1, RetargetLocalPose, RetargetGlobalPose);

	// no retarget pose specified (will use default pose from skeletal mesh with no offsets)
	if (InRetargetPose==nullptr  || RetargetRootBone == NAME_None)
	{
		return;
	}

	// apply retarget pose offsets (retarget pose is stored as offset relative to reference pose)
	const TArray<FTransform>& RefPoseLocal = Skeleton->GetRefBonePose();
	
	// apply root translation offset
	const int32 RootBoneIndex = FindBoneIndexByName(RetargetRootBone);
	if (RootBoneIndex != INDEX_NONE)
	{
		FTransform& RootTransform = RetargetGlobalPose[RootBoneIndex];
		RootTransform.AddToTranslation(InRetargetPose->GetRootTranslationDelta());
		UpdateLocalTransformOfSingleBone(RootBoneIndex, RetargetLocalPose, RetargetGlobalPose);
	}

	// apply bone rotation offsets
	for (const auto& BoneDelta : InRetargetPose->GetAllDeltaRotations())
	{
		const int32 BoneIndex = FindBoneIndexByName(BoneDelta.first);
		if (BoneIndex == INDEX_NONE)
		{
			// this can happen if a retarget pose recorded a bone offset for a bone that is not present in the
			// target skeleton; ie, the retarget pose was generated from a different Skeletal Mesh with extra bones
			continue;
		}

		const FQuat LocalBoneRotation = RefPoseLocal[BoneIndex].GetRotation() * BoneDelta.second;
		RetargetLocalPose[BoneIndex].SetRotation(LocalBoneRotation);
	}

	UpdateGlobalTransformsBelowBone(-1, RetargetLocalPose, RetargetGlobalPose);
}

int32 FRetargetSkeleton::FindBoneIndexByName(const FName InName) const
{
	auto it = std::find(BoneNames.begin(), BoneNames.end(), InName);
	if (it == BoneNames.end()) {
		return -1;
	} else {
		return static_cast<int32_t>(std::distance(BoneNames.begin(), it));
	}
}

void FRetargetSkeleton::UpdateGlobalTransformsBelowBone(
	const int32 StartBoneIndex,
	const TArray<FTransform>& InLocalPose,
	TArray<FTransform>& OutGlobalPose) const
{
	//check(BoneNames.IsValidIndex(StartBoneIndex+1));
	//check(BoneNames.Num() == InLocalPose.Num());
	//check(BoneNames.Num() == OutGlobalPose.Num());
	
	for (int32 BoneIndex=StartBoneIndex+1; BoneIndex<OutGlobalPose.size(); ++BoneIndex)
	{
		UpdateGlobalTransformOfSingleBone(BoneIndex,InLocalPose,OutGlobalPose);
	}
}

void FRetargetSkeleton::UpdateLocalTransformsBelowBone(
	const int32 StartBoneIndex,
	TArray<FTransform>& OutLocalPose,
	const TArray<FTransform>& InGlobalPose) const
{
	//check(BoneNames.IsValidIndex(StartBoneIndex));
	//check(BoneNames.Num() == OutLocalPose.Num());
	//check(BoneNames.Num() == InGlobalPose.Num());
	
	for (int32 BoneIndex=StartBoneIndex+1; BoneIndex<InGlobalPose.size(); ++BoneIndex)
	{
		UpdateLocalTransformOfSingleBone(BoneIndex, OutLocalPose, InGlobalPose);
	}
}

void FRetargetSkeleton::UpdateGlobalTransformOfSingleBone(
	const int32 BoneIndex,
	const TArray<FTransform>& InLocalPose,
	TArray<FTransform>& OutGlobalPose) const
{
	const int32 ParentIndex = ParentIndices[BoneIndex];
	if (ParentIndex == INDEX_NONE)
	{
		// root always in global space already, no conversion required
		OutGlobalPose[BoneIndex] = InLocalPose[BoneIndex];
		return; 
	}
	const FTransform& ChildLocalTransform = InLocalPose[BoneIndex];
	const FTransform& ParentGlobalTransform = OutGlobalPose[ParentIndex];
	OutGlobalPose[BoneIndex] = ChildLocalTransform * ParentGlobalTransform;
}

void FRetargetSkeleton::UpdateLocalTransformOfSingleBone(
	const int32 BoneIndex,
	TArray<FTransform>& OutLocalPose,
	const TArray<FTransform>& InGlobalPose) const
{
	const int32 ParentIndex = ParentIndices[BoneIndex];
	if (ParentIndex == INDEX_NONE)
	{
		// root bone, so just set the local pose to the global pose
		OutLocalPose[BoneIndex] = InGlobalPose[BoneIndex];
		return;
	}
	const FTransform& ChildGlobalTransform = InGlobalPose[BoneIndex];
	const FTransform& ParentGlobalTransform = InGlobalPose[ParentIndex];
	OutLocalPose[BoneIndex] = ChildGlobalTransform.GetRelativeTransform(ParentGlobalTransform);
}

FTransform FRetargetSkeleton::GetGlobalRefPoseOfSingleBone(
	const int32 BoneIndex,
	const TArray<FTransform>& InGlobalPose) const
{
	const int32 ParentIndex = ParentIndices[BoneIndex];
	if (ParentIndex == INDEX_NONE)
	{
		return RetargetLocalPose[BoneIndex]; // root always in global space
	}
	const FTransform& ChildLocalTransform = RetargetLocalPose[BoneIndex];
	const FTransform& ParentGlobalTransform = InGlobalPose[ParentIndex];
	return ChildLocalTransform * ParentGlobalTransform;
}

int32 FRetargetSkeleton::GetCachedEndOfBranchIndex(const int32 InBoneIndex) const
{
	if ( InBoneIndex >= CachedEndOfBranchIndices.size())
	{
		return INDEX_NONE;
	}

	// already cached
	if (CachedEndOfBranchIndices[InBoneIndex] != RETARGETSKELETON_INVALID_BRANCH_INDEX)
	{
		return CachedEndOfBranchIndices[InBoneIndex];
	}

	const int32 NumBones = static_cast<int32_t>(BoneNames.size());
	
	// if we're asking for root's branch, get the last bone  
	if (InBoneIndex == 0)
	{
		CachedEndOfBranchIndices[InBoneIndex] = NumBones-1;
		return CachedEndOfBranchIndices[InBoneIndex];
	}

	CachedEndOfBranchIndices[InBoneIndex] = INDEX_NONE;
	const int32 StartParentIndex = GetParentIndex(InBoneIndex);
	int32 BoneIndex = InBoneIndex + 1;
	int32 ParentIndex = GetParentIndex(BoneIndex);

	// if next child bone's parent is less than or equal to StartParentIndex,
	// we are leaving the branch so no need to go further
	while (ParentIndex > StartParentIndex && BoneIndex < NumBones)
	{
		CachedEndOfBranchIndices[InBoneIndex] = BoneIndex;
				
		BoneIndex++;
		ParentIndex = GetParentIndex(BoneIndex);
	}

	return CachedEndOfBranchIndices[InBoneIndex];
}

void FRetargetSkeleton::GetChildrenIndices(const int32 BoneIndex, TArray<int32>& OutChildren) const
{
	const int32 LastBranchIndex = GetCachedEndOfBranchIndex(BoneIndex);
	if (LastBranchIndex == INDEX_NONE)
	{
		// no children (leaf bone)
		return;
	}
	
	for (int32 ChildBoneIndex = BoneIndex + 1; ChildBoneIndex <= LastBranchIndex; ChildBoneIndex++)
	{
		if (GetParentIndex(ChildBoneIndex) == BoneIndex)
		{
			OutChildren.push_back(ChildBoneIndex);
		}
	}
}

void FRetargetSkeleton::GetChildrenIndicesRecursive(const int32 BoneIndex, TArray<int32>& OutChildren) const
{
	const int32 LastBranchIndex = GetCachedEndOfBranchIndex(BoneIndex);
	if (LastBranchIndex == INDEX_NONE)
	{
		// no children (leaf bone)
		return;
	}
	
	for (int32 ChildBoneIndex = BoneIndex + 1; ChildBoneIndex <= LastBranchIndex; ChildBoneIndex++)
	{
		OutChildren.push_back(ChildBoneIndex);
	}
}

bool FRetargetSkeleton::IsParentOfChild(const int32 PotentialParentIndex, const int32 ChildBoneIndex) const
{
	int32 ParentIndex = GetParentIndex(ChildBoneIndex);
	while (ParentIndex != INDEX_NONE)
	{
		if (ParentIndex == PotentialParentIndex)
		{
			return true;
		}
		
		ParentIndex = GetParentIndex(ParentIndex);
	}
	
	return false;
}

int32 FRetargetSkeleton::GetParentIndex(const int32 BoneIndex) const
{
	if (BoneIndex < 0 || BoneIndex>=ParentIndices.size() || BoneIndex == INDEX_NONE)
	{
		return INDEX_NONE;
	}

	return ParentIndices[BoneIndex];
}

void FTargetSkeleton::Initialize(
	USkeleton* InSkeleton,
	const TArray<FBoneChain>& BoneChains,
	const FName InRetargetPoseName,
	const FIKRetargetPose* RetargetPose,
	const FName RetargetRootBone)
{
	Reset();
	
	FRetargetSkeleton::Initialize(InSkeleton, BoneChains, InRetargetPoseName, RetargetPose, RetargetRootBone);

	// make storage for per-bone "Is Retargeted" flag (used for hierarchy updates)
	// these are bones that are in a target chain that is mapped to a source chain (ie, will actually be retargeted)
	// these flags are actually set later in init phase when bone chains are mapped together
	IsBoneRetargeted.resize(BoneNames.size(), false);

	// initialize storage for output pose (the result of the retargeting)
	OutputGlobalPose = RetargetGlobalPose;
}

void FTargetSkeleton::Reset()
{
	FRetargetSkeleton::Reset();
	OutputGlobalPose.clear();
	IsBoneRetargeted.clear();
}

void FTargetSkeleton::UpdateGlobalTransformsAllNonRetargetedBones(TArray<FTransform>& InOutGlobalPose)
{
	//check(IsBoneRetargeted.Num() == InOutGlobalPose.Num());
	
	for (int32 BoneIndex=0; BoneIndex<InOutGlobalPose.size(); ++BoneIndex)
	{
		if (!IsBoneRetargeted[BoneIndex])
		{
			UpdateGlobalTransformOfSingleBone(BoneIndex, RetargetLocalPose, InOutGlobalPose);
		}
	}
}

FResolvedBoneChain::FResolvedBoneChain(
	const FBoneChain& BoneChain,
	const FRetargetSkeleton& Skeleton,
	TArray<int32>& OutBoneIndices)
{
	// validate start and end bones exist and are not the root
	const int32 StartIndex = Skeleton.FindBoneIndexByName(BoneChain.StartBone.BoneName);
	const int32 EndIndex = Skeleton.FindBoneIndexByName(BoneChain.EndBone.BoneName);
	bFoundStartBone = StartIndex > INDEX_NONE;
	bFoundEndBone = EndIndex > INDEX_NONE;

	// no need to build the chain if start/end indices are wrong 
	const bool bIsWellFormed = bFoundStartBone && bFoundEndBone && EndIndex >= StartIndex;
	if (bIsWellFormed)
	{
		// init array with end bone 
		OutBoneIndices = {EndIndex};

		// if only one bone in the chain
		if (EndIndex == StartIndex)
		{
			// validate end bone is child of start bone ?
			bEndIsStartOrChildOfStart = true;
			return;
		}

		// record all bones in chain while walking up the hierarchy (tip to root of chain)
		int32 ParentIndex = Skeleton.GetParentIndex(EndIndex);
		while (ParentIndex > INDEX_NONE && ParentIndex >= StartIndex)
		{
			OutBoneIndices.push_back(ParentIndex);
			ParentIndex = Skeleton.GetParentIndex(ParentIndex);
		}

		// if we walked up till the start bone
		if (OutBoneIndices.back() == StartIndex)
		{
			// validate end bone is child of start bone
			bEndIsStartOrChildOfStart = true;
			// reverse the indices (we want root to tip order)
			std::reverse(OutBoneIndices.begin(), OutBoneIndices.end());
			return;
		}
      
		// oops, we walked all the way up without finding the start bone
		OutBoneIndices.clear();
	}
}

void FTargetSkeleton::SetBoneIsRetargeted(const int32 BoneIndex, const bool IsRetargeted)
{
	//check(IsBoneRetargeted.IsValidIndex(BoneIndex));
	IsBoneRetargeted[BoneIndex] = IsRetargeted;
}

bool FChainFK::Initialize(
	const FRetargetSkeleton& Skeleton,
	const TArray<int32>& InBoneIndices,
	const TArray<FTransform>& InitialGlobalPose,
	FIKRigLogger& Log)
{
	//check(!InBoneIndices.IsEmpty());

	// store for debugging purposes
	BoneIndices = InBoneIndices;

	// store all the initial bone transforms in the bone chain
	InitialGlobalTransforms.clear();
	for (int32 Index=0; Index < BoneIndices.size(); ++Index)
	{
		const int32 BoneIndex = BoneIndices[Index];
		if (BoneIndex < InitialGlobalPose.size())
		{
			InitialGlobalTransforms.emplace_back(InitialGlobalPose[BoneIndex]);
		}
	}

	// initialize storage for current bones
	CurrentGlobalTransforms = InitialGlobalTransforms;

	// get the local space of the chain in retarget pose
	InitialLocalTransforms.resize(InitialGlobalTransforms.size());
	FillTransformsWithLocalSpaceOfChain(Skeleton, InitialGlobalPose, BoneIndices, InitialLocalTransforms);

	// store chain parent data
	ChainParentBoneIndex = Skeleton.GetParentIndex(BoneIndices[0]);
	ChainParentInitialGlobalTransform = FTransform::Identity;
	if (ChainParentBoneIndex != INDEX_NONE)
	{
		ChainParentInitialGlobalTransform = InitialGlobalPose[ChainParentBoneIndex];
	}

#ifdef DEBUG_POSE_LOG
	printf("encode init\n");
//	printf("InitialGlobalPose[1]:  t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n",
//		InitialGlobalPose[1].Translation.x,
//		InitialGlobalPose[1].Translation.y,
//		InitialGlobalPose[1].Translation.z,
//		InitialGlobalPose[1].Rotation.x,
//		InitialGlobalPose[1].Rotation.y,
//		InitialGlobalPose[1].Rotation.z,
//		InitialGlobalPose[1].Rotation.w
//	);
//	printf("InitialGlobalPose[2]:  t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n",
//		InitialGlobalPose[2].Translation.x,
//		InitialGlobalPose[2].Translation.y,
//		InitialGlobalPose[2].Translation.z,
//		InitialGlobalPose[2].Rotation.x,
//		InitialGlobalPose[2].Rotation.y,
//		InitialGlobalPose[2].Rotation.z,
//		InitialGlobalPose[2].Rotation.w
//	);

	printf("InitialGlobalTransforms[0]:  t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n", 
		InitialGlobalTransforms[0].Translation.x,
		InitialGlobalTransforms[0].Translation.y,
		InitialGlobalTransforms[0].Translation.z,
		InitialGlobalTransforms[0].Rotation.x,
		InitialGlobalTransforms[0].Rotation.y,
		InitialGlobalTransforms[0].Rotation.z,
		InitialGlobalTransforms[0].Rotation.w
	);
//	printf("InitialGlobalTransforms[1]:  t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n",
//		InitialGlobalTransforms[1].Translation.x,
//		InitialGlobalTransforms[1].Translation.y,
//		InitialGlobalTransforms[1].Translation.z,
//		InitialGlobalTransforms[1].Rotation.x,
//		InitialGlobalTransforms[1].Rotation.y,
//		InitialGlobalTransforms[1].Rotation.z,
//		InitialGlobalTransforms[1].Rotation.w
//	);
#endif

	// calculate parameter of each bone, normalized by the length of the bone chain
	return CalculateBoneParameters(Log);
}

bool FChainFK::CalculateBoneParameters(FIKRigLogger& Log)
{
	Params.clear();
	
	// special case, a single-bone chain
	if (InitialGlobalTransforms.size() == 1)
	{
		Params.push_back(1.0f);
		return true;
	}

	// calculate bone lengths in chain and accumulate total length
	TArray<float> BoneDistances;
	float TotalChainLength = 0.0f;
	BoneDistances.push_back(0.0f);
	for (int32 i=1; i<InitialGlobalTransforms.size(); ++i)
	{
		TotalChainLength += static_cast<float>((InitialGlobalTransforms[i].GetTranslation() - InitialGlobalTransforms[i-1].GetTranslation()).Size());
		BoneDistances.push_back(TotalChainLength);
	}

	// cannot retarget chain if all the bones are sitting directly on each other
	if (TotalChainLength <= KINDA_SMALL_NUMBER)
	{
		Log.LogWarning("TinyBoneChain, IK Retargeter bone chain length is too small to reliably retarget.", "");
		return false;
	}

	// calc each bone's param along length
	for (int32 i=0; i<InitialGlobalTransforms.size(); ++i)
	{
		Params.push_back(BoneDistances[i] / TotalChainLength); 
	}

	return true;
}

void FChainFK::FillTransformsWithLocalSpaceOfChain(
	const FRetargetSkeleton& Skeleton,
	const TArray<FTransform>& InGlobalPose,
	const TArray<int32>& BoneIndices,
	TArray<FTransform>& OutLocalTransforms)
{
	//check(BoneIndices.Num() == OutLocalTransforms.Num())
	
	for (int32 ChainIndex=0; ChainIndex<BoneIndices.size(); ++ChainIndex)
	{
		const int32 BoneIndex = BoneIndices[ChainIndex];
		const int32 ParentIndex = Skeleton.GetParentIndex(BoneIndex);
		if (ParentIndex == INDEX_NONE)
		{
			// root is always in "global" space
			OutLocalTransforms[ChainIndex] = InGlobalPose[BoneIndex];
			continue;
		}

		const FTransform& ChildGlobalTransform = InGlobalPose[BoneIndex];
		const FTransform& ParentGlobalTransform = InGlobalPose[ParentIndex];
		OutLocalTransforms[ChainIndex] = ChildGlobalTransform.GetRelativeTransform(ParentGlobalTransform);
	}
}

void FChainFK::PutCurrentTransformsInRefPose(
	const TArray<int32>& InBoneIndices,
	const FRetargetSkeleton& Skeleton,
	const TArray<FTransform>& InCurrentGlobalPose)
{
	// update chain current transforms to the retarget pose in global space
	for (int32 ChainIndex=0; ChainIndex<InBoneIndices.size(); ++ChainIndex)
	{
		// update first bone in chain based on the incoming parent
		if (ChainIndex == 0)
		{
			const int32 BoneIndex = InBoneIndices[ChainIndex];
			CurrentGlobalTransforms[ChainIndex] = Skeleton.GetGlobalRefPoseOfSingleBone(BoneIndex, InCurrentGlobalPose);
		}
		else
		{
			// all subsequent bones in chain are based on previous parent
			const int32 BoneIndex = InBoneIndices[ChainIndex];
			const FTransform& ParentGlobalTransform = CurrentGlobalTransforms[ChainIndex-1];
			const FTransform& ChildLocalTransform = Skeleton.RetargetLocalPose[BoneIndex];
			CurrentGlobalTransforms[ChainIndex] = ChildLocalTransform * ParentGlobalTransform;
		}
	}
}

void FChainEncoderFK::EncodePose(
	const FRetargetSkeleton& SourceSkeleton,
	const TArray<int32>& SourceBoneIndices,
    const TArray<FTransform> &InSourceGlobalPose)
{
	//check(SourceBoneIndices.Num() == CurrentGlobalTransforms.Num());

#ifdef DEBUG_POSE_LOG
	printf("encode\n");
	printf("InSourceGlobalPose[1]:  t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n", 
		InSourceGlobalPose[1].Translation.x, 
		InSourceGlobalPose[1].Translation.y,
		InSourceGlobalPose[1].Translation.z,
		InSourceGlobalPose[1].Rotation.x,
		InSourceGlobalPose[1].Rotation.y,
		InSourceGlobalPose[1].Rotation.z,
		InSourceGlobalPose[1].Rotation.w
	);
	printf("InSourceGlobalPose[2]:  t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n", 
		InSourceGlobalPose[2].Translation.x, 
		InSourceGlobalPose[2].Translation.y,
		InSourceGlobalPose[2].Translation.z,
		InSourceGlobalPose[2].Rotation.x,
		InSourceGlobalPose[2].Rotation.y,
		InSourceGlobalPose[2].Rotation.z,
		InSourceGlobalPose[2].Rotation.w
	);
#endif 
	
	// copy the global input pose for the chain
	for (int32 ChainIndex=0; ChainIndex<SourceBoneIndices.size(); ++ChainIndex)
	{
		const int32 BoneIndex = SourceBoneIndices[ChainIndex];
		CurrentGlobalTransforms[ChainIndex] = InSourceGlobalPose[BoneIndex];
	}

	CurrentLocalTransforms.resize(SourceBoneIndices.size());
	FillTransformsWithLocalSpaceOfChain(SourceSkeleton, InSourceGlobalPose, SourceBoneIndices, CurrentLocalTransforms);

	if (ChainParentBoneIndex != INDEX_NONE)
	{
		ChainParentCurrentGlobalTransform = InSourceGlobalPose[ChainParentBoneIndex];
	}
}

void FChainEncoderFK::TransformCurrentChainTransforms(const FTransform& NewParentTransform)
{
	for (int32 ChainIndex=0; ChainIndex<CurrentGlobalTransforms.size(); ++ChainIndex)
	{
		if (ChainIndex == 0)
		{
			CurrentGlobalTransforms[ChainIndex] = CurrentLocalTransforms[ChainIndex] * NewParentTransform;
		}
		else
		{
			CurrentGlobalTransforms[ChainIndex] = CurrentLocalTransforms[ChainIndex] * CurrentGlobalTransforms[ChainIndex-1];
		}
	}
}

static void printTargetNodeTransform(
	TArray<FTransform> &InOutGlobalPose, 				// curG
	const FTargetSkeleton& TargetSkeleton, 				// init
	FQuat const& RotationDelta,
	int ChainIndex, int BoneIndex, int ParentIndex) {

	// each node: 
    //		init TRS len/angle/L len/angle/G deltaR
    //		cur  TRS len/angle/L len/angle/G deltaR
	
	double len = -1;

	printf("decode target ChainIndex: %d, boneIndex:%d name:%s\n", 
		ChainIndex, BoneIndex, TargetSkeleton.BoneNames[BoneIndex].c_str()
	);

	printf("init L len:%.2f angle:%.2f t(%.2f %.2f %.2f) r.xyzwd(%.2f %.2f %.2f %.2f)%.2f\n",
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetTranslation().Length(),
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetRotation().AngleDegree(),
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetTranslation().x,
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetTranslation().y,
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetTranslation().z,
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetRotation().x,
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetRotation().y,
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetRotation().z,
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetRotation().w,
		TargetSkeleton.RetargetLocalPose[BoneIndex].GetRotation().Size()
	);

	if (ParentIndex != -1) {
		len = (TargetSkeleton.RetargetGlobalPose[ParentIndex].GetTranslation() 
			-  TargetSkeleton.RetargetGlobalPose[BoneIndex].GetTranslation()).Length();
	}
	printf("init G len:%.2lf angle:%.2lf t(%.2lf %.2lf %.2lf) r.xyzwd(%.2lf %.2lf %.2lf %.2lf)%.2f\n",
		len,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetRotation().AngleDegree(),
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetTranslation().x,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetTranslation().y,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetTranslation().z,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetRotation().x,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetRotation().y,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetRotation().z,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetRotation().w,
		TargetSkeleton.RetargetGlobalPose[BoneIndex].GetRotation().Size()
	);

	printf("RotationDelta r.xyzwd:(%.2f %.2f %.2f %.2f)%.2f alpha:%.2f\n",
		RotationDelta.x,
		RotationDelta.y,
		RotationDelta.z,
		RotationDelta.w,
		RotationDelta.Size(),
		RotationDelta.AngleDegree()
	);

	FTransform curL;
	if (ParentIndex != -1) {
		curL = InOutGlobalPose[BoneIndex].GetRelativeTransform(InOutGlobalPose[ParentIndex]);
	} else {
		curL = InOutGlobalPose[BoneIndex];
	}
	printf("cur L len:%.2f angle:%.2f t(%.2f %.2f %.2f) r.xyzwd(%.2f %.2f %.2f %.2f)%.2f\n",
		curL.GetTranslation().Length(),
		curL.GetRotation().AngleDegree(),
		curL.GetTranslation().x,
		curL.GetTranslation().y,
		curL.GetTranslation().z,
		curL.GetRotation().x,
		curL.GetRotation().y,
		curL.GetRotation().z,
		curL.GetRotation().w,
		curL.GetRotation().Size()
	);

	if (ParentIndex != -1) {
		len = (InOutGlobalPose[ParentIndex].GetTranslation() 
			-  InOutGlobalPose[BoneIndex].GetTranslation()).Length();
	}
	printf("cur G len:%.2f angle:%.2f t(%.2f %.2f %.2f) r.xyzwd(%.2f %.2f %.2f %.2f)%.2f\n",
		len,
		InOutGlobalPose[BoneIndex].GetRotation().AngleDegree(),
		InOutGlobalPose[BoneIndex].GetTranslation().x,
		InOutGlobalPose[BoneIndex].GetTranslation().y,
		InOutGlobalPose[BoneIndex].GetTranslation().z,
		InOutGlobalPose[BoneIndex].GetRotation().x,
		InOutGlobalPose[BoneIndex].GetRotation().y,
		InOutGlobalPose[BoneIndex].GetRotation().z,
		InOutGlobalPose[BoneIndex].GetRotation().w,
		InOutGlobalPose[BoneIndex].GetRotation().Size()
	);
}

void FChainDecoderFK::DecodePose(
	const FRootRetargeter& RootRetargeter,
	const FTargetChainSettings& Settings,
	const TArray<int32>& TargetBoneIndices,
    FChainEncoderFK& SourceChain,
    const FTargetSkeleton& TargetSkeleton,
    TArray<FTransform> &InOutGlobalPose)
{
	//check(TargetBoneIndices.Num() == CurrentGlobalTransforms.Num());
	//check(TargetBoneIndices.Num() == Params.Num());

	// Before setting this chain pose, we need to ensure that any
	// intermediate (between chains) NON-retargeted parent bones have had their
	// global transforms updated.
	// 
	// For example, if this chain is retargeting a single head bone, AND the spine was
	// retargeted in the prior step, then the neck bones will need updating first.
	// Otherwise the neck bones will remain at their location prior to the spine update.
	UpdateIntermediateParents(TargetSkeleton,InOutGlobalPose);

	// transform entire source chain from it's root to match target's current root orientation (maintaining offset from retarget pose)
	// this ensures children are retargeted in a "local" manner free from skewing that will happen if source and target
	// become misaligned as can happen if parent chains were not retargeted
	FTransform SourceChainParentInitialDelta = SourceChain.ChainParentInitialGlobalTransform.GetRelativeTransform(ChainParentInitialGlobalTransform);
	FTransform TargetChainParentCurrentGlobalTransform = ChainParentBoneIndex == INDEX_NONE ? FTransform::Identity : InOutGlobalPose[ChainParentBoneIndex]; 
	FTransform SourceChainParentTransform = SourceChainParentInitialDelta * TargetChainParentCurrentGlobalTransform;

	// apply delta to the source chain's current transforms before transferring rotations to the target
	SourceChain.TransformCurrentChainTransforms(SourceChainParentTransform);

	// if FK retargeting has been disabled for this chain, then simply set it to the retarget pose
	if (!Settings.FK.EnableFK)
	{
		// put the chain in the global ref pose (globally rotated by parent bone in it's currently retargeted state)
		PutCurrentTransformsInRefPose(TargetBoneIndices, TargetSkeleton, InOutGlobalPose);
		
		for (int32 ChainIndex=0; ChainIndex<TargetBoneIndices.size(); ++ChainIndex)
		{
			const int32 BoneIndex = TargetBoneIndices[ChainIndex];
			InOutGlobalPose[BoneIndex] = CurrentGlobalTransforms[ChainIndex];
		}

		return;
	}

	const int32 NumBonesInSourceChain = static_cast<int32>(SourceChain.CurrentGlobalTransforms.size());
	const int32 NumBonesInTargetChain = static_cast<int32>(TargetBoneIndices.size());
	const int32 TargetStartIndex = std::max(0, NumBonesInTargetChain - NumBonesInSourceChain);
	const int32 SourceStartIndex = std::max(0,NumBonesInSourceChain - NumBonesInTargetChain);

	// now retarget the pose of each bone in the chain, copying from source to target
	for (int32 ChainIndex=0; ChainIndex<TargetBoneIndices.size(); ++ChainIndex)
	{
		const int32 BoneIndex = TargetBoneIndices[ChainIndex];
		const FTransform& TargetInitialTransform = InitialGlobalTransforms[ChainIndex];
		FTransform SourceCurrentTransform;
		FTransform SourceInitialTransform;

		// get source current / initial transforms for this bone
		switch (Settings.FK.RotationMode)
		{
			case ERetargetRotationMode::Interpolated:
			{
				// get the initial and current transform of source chain at param
				// this is the interpolated transform along the chain
				const float Param = Params[ChainIndex];
					
				SourceCurrentTransform = GetTransformAtParam(
					SourceChain.CurrentGlobalTransforms,
					SourceChain.Params,
					Param);

				SourceInitialTransform = GetTransformAtParam(
					SourceChain.InitialGlobalTransforms,
					SourceChain.Params,
					Param);
			}
			break;
			case ERetargetRotationMode::OneToOne:
			{
				if (ChainIndex < NumBonesInSourceChain)
				{
					SourceCurrentTransform = SourceChain.CurrentGlobalTransforms[ChainIndex];
					SourceInitialTransform = SourceChain.InitialGlobalTransforms[ChainIndex];
				}else
				{
					SourceCurrentTransform = SourceChain.CurrentGlobalTransforms.back();
					SourceInitialTransform = SourceChain.InitialGlobalTransforms.back();
				}
			}
			break;
			case ERetargetRotationMode::OneToOneReversed:
			{
				if (ChainIndex < TargetStartIndex)
				{
					SourceCurrentTransform = SourceChain.InitialGlobalTransforms[0];
					SourceInitialTransform = SourceChain.InitialGlobalTransforms[0];
				}
				else
				{
					const int32 SourceChainIndex = SourceStartIndex + (ChainIndex - TargetStartIndex);
					SourceCurrentTransform = SourceChain.CurrentGlobalTransforms[SourceChainIndex];
					SourceInitialTransform = SourceChain.InitialGlobalTransforms[SourceChainIndex];
				}
			}
			break;
			case ERetargetRotationMode::None:
			{
				SourceCurrentTransform = SourceChain.InitialGlobalTransforms.back();
				SourceInitialTransform = SourceChain.InitialGlobalTransforms.back();
			}
			break;
			default:
				checkNoEntry();
			break;
		}

		#ifdef DEBUG_POSE_LOG_CHAINFK
		printf("decode source ChainIndex:%d jointId:%d jointName:%s\n", ChainIndex, BoneIndex, 
			TargetSkeleton.BoneNames[BoneIndex].c_str()
		);
		printf("SourceInitialTransform: t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n", 
			SourceInitialTransform.Translation.x, 
			SourceInitialTransform.Translation.y,
			SourceInitialTransform.Translation.z,
			SourceInitialTransform.Rotation.x,
			SourceInitialTransform.Rotation.y,
			SourceInitialTransform.Rotation.z,
			SourceInitialTransform.Rotation.w
		);
		printf("SourceCurrentTransform: t.xyz:(%.2f %.2f %.2f) r.xyzw:(%.2f %.2f %.2f %.2f)\n", 
			SourceCurrentTransform.Translation.x, 
			SourceCurrentTransform.Translation.y,
			SourceCurrentTransform.Translation.z,
			SourceCurrentTransform.Rotation.x,
			SourceCurrentTransform.Rotation.y,
			SourceCurrentTransform.Rotation.z,
			SourceCurrentTransform.Rotation.w
		);
		#endif
		
		// apply rotation offset to the initial target rotation
		const FQuat SourceCurrentRotation = SourceCurrentTransform.GetRotation();
		const FQuat SourceInitialRotation = SourceInitialTransform.GetRotation();
		FQuat RotationDelta = SourceCurrentRotation * SourceInitialRotation.Inverse();
		//printf("rot0:%f rotnow:%f rotationDelta:%f\n", SourceInitialRotation.getAngleDegree(), SourceCurrentRotation.getAngleDegree(), RotationDelta.getAngleDegree());
		const FQuat TargetInitialRotation = TargetInitialTransform.GetRotation();
		FQuat OutRotation = RotationDelta * TargetInitialRotation; // copy Global rotation
		// OutRotation.Normalize(); // chenkai bugfix, not bug

		#ifdef DEBUG_POSE_LOG_CHAINFK
		printf("TargetInitialRotation.xyzwd:(%.2f %.2f %.2f %.2f)%.2f angle:%.2f\n", 
			TargetInitialRotation.x,
			TargetInitialRotation.y,
			TargetInitialRotation.z,
			TargetInitialRotation.w,
			TargetInitialRotation.Size(),
			TargetInitialRotation.AngleDegree()
		);
		printf("RotationDelta.xyzwd:(%.2f %.2f %.2f %.2f)%.2f angle:%.2f\n", 
			RotationDelta.x,
			RotationDelta.y,
			RotationDelta.z,
			RotationDelta.w,
			RotationDelta.Size(),
			RotationDelta.AngleDegree()
		);
		printf("OutRotation.xyzwd:(%.2f %.2f %.2f %.2f)%.2f angle:%.2f\n", 
			OutRotation.x,
			OutRotation.y,
			OutRotation.z,
			OutRotation.w,
			OutRotation.Size(),
			OutRotation.AngleDegree()
		);
		#endif

		// calculate output POSITION based on translation mode setting
		FTransform ParentGlobalTransform = FTransform::Identity;
		const int32 ParentIndex = TargetSkeleton.ParentIndices[BoneIndex];
		if (ParentIndex != INDEX_NONE)
		{
			ParentGlobalTransform = InOutGlobalPose[ParentIndex];
		}
		FVector OutPosition;
		switch (Settings.FK.TranslationMode)
		{
			case ERetargetTranslationMode::None:
				{
					const FVector InitialLocalOffset = TargetSkeleton.RetargetLocalPose[BoneIndex].GetTranslation();
					OutPosition = ParentGlobalTransform.TransformPosition(InitialLocalOffset);
				}
				break;
			case ERetargetTranslationMode::GloballyScaled:
				{
					OutPosition = SourceCurrentTransform.GetTranslation() * RootRetargeter.GetGlobalScaleVector();
				}
				break;
			case ERetargetTranslationMode::Absolute:
				OutPosition = SourceCurrentTransform.GetTranslation();
				break;
			default:
				checkNoEntry();
				break;
		}

		// calculate output SCALE
		const FVector SourceCurrentScale = SourceCurrentTransform.GetScale3D();
		const FVector SourceInitialScale = SourceInitialTransform.GetScale3D();
		const FVector TargetInitialScale = TargetInitialTransform.GetScale3D();
		const FVector OutScale = SourceCurrentScale + (TargetInitialScale - SourceInitialScale);
		
		// apply output transform
		CurrentGlobalTransforms[ChainIndex] = FTransform(OutRotation, OutPosition, OutScale);
		InOutGlobalPose[BoneIndex] = CurrentGlobalTransforms[ChainIndex];

		#ifdef DEBUG_POSE_LOG_CHAINFK
		if (ChainIndex == 0) {
			int ParentParentIndex = TargetSkeleton.ParentIndices[ParentIndex];
			printTargetNodeTransform(InOutGlobalPose, TargetSkeleton, FQuat(), -1, ParentIndex, ParentParentIndex);
		}
		printTargetNodeTransform(InOutGlobalPose, TargetSkeleton, RotationDelta, ChainIndex, BoneIndex, ParentIndex);
		#endif
	}

	// apply final blending between retarget pose of chain and newly retargeted pose
	// blend must be done in local space, so we do it in a separate loop after full chain pose is generated
	// (skipped if the alphas are not near 1.0)
	if (!IsNearlyEqual(Settings.FK.RotationAlpha, 1.0f) || !IsNearlyEqual(Settings.FK.TranslationAlpha, 1.0f))
	{
		TArray<FTransform> NewLocalTransforms;
		NewLocalTransforms.resize(InitialLocalTransforms.size());
		FillTransformsWithLocalSpaceOfChain(TargetSkeleton, InOutGlobalPose, TargetBoneIndices, NewLocalTransforms);

		for (int32 ChainIndex=0; ChainIndex<InitialLocalTransforms.size(); ++ChainIndex)
		{
			// blend between current local pose and initial local pose
			FTransform& NewLocalTransform = NewLocalTransforms[ChainIndex];
			const FTransform& RefPoseLocalTransform = InitialLocalTransforms[ChainIndex];
			NewLocalTransform.SetTranslation(FVector::lerp(RefPoseLocalTransform.GetTranslation(), NewLocalTransform.GetTranslation(), Settings.FK.TranslationAlpha));
			NewLocalTransform.SetRotation(FQuat::FastLerp(RefPoseLocalTransform.GetRotation(), NewLocalTransform.GetRotation(), Settings.FK.RotationAlpha).GetNormalized());

			// put blended transforms back in global space and store in final output pose
			const int32 BoneIndex = TargetBoneIndices[ChainIndex];
			const int32 ParentIndex = TargetSkeleton.ParentIndices[BoneIndex];
			const FTransform& ParentGlobalTransform = ParentIndex == INDEX_NONE ? FTransform::Identity : InOutGlobalPose[ParentIndex];
			InOutGlobalPose[BoneIndex] = NewLocalTransform * ParentGlobalTransform;
		}
	}
}

void FChainDecoderFK::InitializeIntermediateParentIndices(
	const int32 RetargetRootBoneIndex,
	const int32 ChainRootBoneIndex,
	const FTargetSkeleton& TargetSkeleton)
{
	IntermediateParentIndices.clear();
	int32 ParentBoneIndex = TargetSkeleton.ParentIndices[ChainRootBoneIndex];
	while (true)
	{
		if (ParentBoneIndex < 0 || ParentBoneIndex == RetargetRootBoneIndex)
		{
			break; // reached root of skeleton
		}

		if (TargetSkeleton.IsBoneRetargeted[ParentBoneIndex])
		{
			break; // reached the start of another retargeted chain
		}

		IntermediateParentIndices.push_back(ParentBoneIndex);
		ParentBoneIndex = TargetSkeleton.ParentIndices[ParentBoneIndex];
	}

	std::reverse(IntermediateParentIndices.begin(), IntermediateParentIndices.end());
}

void FChainDecoderFK::UpdateIntermediateParents(
	const FTargetSkeleton& TargetSkeleton,
	TArray<FTransform>& InOutGlobalPose)
{
	for (const int32& ParentIndex : IntermediateParentIndices)
	{
		TargetSkeleton.UpdateGlobalTransformOfSingleBone(ParentIndex, TargetSkeleton.RetargetLocalPose, InOutGlobalPose);
	}
}

FTransform FChainDecoderFK::GetTransformAtParam(
	const TArray<FTransform>& Transforms,
	const TArray<float>& InParams,
	const float& Param) const
{
	if (InParams.size() == 1)
	{
		#ifdef DEBUG_POSE_LOG_CHAINFK
		printf("get front chain:%d\n", 0);
		#endif
		return Transforms[0];
	}
	
	if (Param < KINDA_SMALL_NUMBER)
	{
		#ifdef DEBUG_POSE_LOG_CHAINFK
		printf("get front chain:%d\n", 0);
		#endif
		return Transforms[0];
	}

	if (Param > 1.0f - KINDA_SMALL_NUMBER)
	{
		#ifdef DEBUG_POSE_LOG_CHAINFK
		printf("get back chain:%d\n", Transforms.size()-1);
		#endif
		return Transforms.back();
	}

	for (int32 ChainIndex=1; ChainIndex<InParams.size(); ++ChainIndex)
	{
		const float CurrentParam = InParams[ChainIndex];
		if (CurrentParam <= Param)
		{
			continue;
		}
		
		const float PrevParam = InParams[ChainIndex-1];
		const float PercentBetweenParams = (Param - PrevParam) / (CurrentParam - PrevParam);
		const FTransform& Prev = Transforms[ChainIndex-1];
		const FTransform& Next = Transforms[ChainIndex];
		const FVector Position = FVector::lerp(Prev.GetTranslation(), Next.GetTranslation(), PercentBetweenParams);
		FQuat Rotation = FQuat::FastLerp(Prev.GetRotation(), Next.GetRotation(), PercentBetweenParams).GetNormalized();
		const FVector Scale = FVector::lerp(Prev.GetScale3D(), Next.GetScale3D(), PercentBetweenParams);
		
		#ifdef DEBUG_POSE_LOG_CHAINFK
		//Rotation = FQuat::Identity;
		printf("get blend chain:%d %d %.2f t:(%.2f %.2f %.2f) r.xyzw(%.2f %.2f %.2f %.2f)\n", 
				ChainIndex-1, ChainIndex, PercentBetweenParams,
				Position.x, Position.y, Position.z,
				Rotation.x, Rotation.y, Rotation.z, Rotation.w
			);
		#endif
		return FTransform(Rotation,Position, Scale);
	}

	checkNoEntry();
	return FTransform::Identity;
}


bool FChainRetargeterIK::InitializeSource(
	const TArray<int32>& BoneIndices,
	const TArray<FTransform>& SourceInitialGlobalPose,
	FIKRigLogger& Log)
{
	// todo
	return true;
}

void FChainRetargeterIK::EncodePose(const TArray<FTransform>& InSourceGlobalPose)
{
	// todo
}

bool FChainRetargeterIK::InitializeTarget(
	const TArray<int32>& BoneIndices,
	const TArray<FTransform> &TargetInitialGlobalPose,
	FIKRigLogger& Log)
{
	// todo
	return true;
}
	
void FChainRetargeterIK::DecodePose(
	const FTargetChainSettings& Settings,
	const FRootRetargeter& RootRetargeter,
	const std::unordered_map<FName, float>& SpeedValuesFromCurves,
	const float DeltaTime,
    const TArray<FTransform>& InGlobalPose)
{
	// todo
}

// MARK: - chain pair


bool FRetargetChainPair::Initialize(
    const FBoneChain& SourceBoneChain,
    const FBoneChain& TargetBoneChain,
    const FRetargetSkeleton& SourceSkeleton,
    const FTargetSkeleton& TargetSkeleton,
    FIKRigLogger& Log)
{
	// validate source bone chain is compatible with source skeletal mesh
	const bool bIsSourceValid = ValidateBoneChainWithSkeletalMesh(true, SourceBoneChain, SourceSkeleton, Log);
	if (!bIsSourceValid)
	{
		Log.LogWarning("IncompatibleSourceChain, IK Retargeter source bone chain, '%s', is not compatible with Skeletal Mesh: '%s'",
			SourceBoneChain.ChainName.c_str(), SourceSkeleton.Skeleton->GetName().c_str());
		return false;
	}

	// validate target bone chain is compatible with target skeletal mesh
	const bool bIsTargetValid = ValidateBoneChainWithSkeletalMesh(false, TargetBoneChain, TargetSkeleton, Log);
	if (!bIsTargetValid)
    {
		Log.LogWarning("IncompatibleTargetChain, IK Retargeter target bone chain, '{%s}', is not compatible with Skeletal Mesh: '{%s}'",
			TargetBoneChain.ChainName.c_str(), TargetSkeleton.Skeleton->GetName().c_str());
		return false;
    }

	// store attributes of chain
	SourceBoneChainName = SourceBoneChain.ChainName;
	TargetBoneChainName = TargetBoneChain.ChainName;
	
	return true;
}

bool FRetargetChainPair::ValidateBoneChainWithSkeletalMesh(
    const bool IsSource,
    const FBoneChain& BoneChain,
    const FRetargetSkeleton& RetargetSkeleton,
    FIKRigLogger& Log)
{
	// record the chain indices
	TArray<int32>& BoneIndices = IsSource ? SourceBoneIndices : TargetBoneIndices;
	
	// resolve the bone bone to the skeleton
	const FResolvedBoneChain ResolvedChain = FResolvedBoneChain(BoneChain, RetargetSkeleton, BoneIndices);
	
	// warn if START bone not found
	if (!ResolvedChain.bFoundStartBone)
	{
		Log.LogWarning("MissingStartBone, IK Retargeter bone chain, {%s}, could not find start bone, {%s} in mesh {%s}",
			BoneChain.ChainName.c_str(),
			BoneChain.StartBone.BoneName.c_str(),
			RetargetSkeleton.Skeleton->GetName().c_str());
	}
	
	// warn if END bone not found
	if (!ResolvedChain.bFoundEndBone)
	{
		Log.LogWarning("MissingEndBone, IK Retargeter bone chain, {%s}, could not find end bone, {%s} in mesh {%s}",
			FText::FromName(BoneChain.ChainName), 
			FText::FromName(BoneChain.EndBone.BoneName), 
			FText::FromName(RetargetSkeleton.Skeleton->GetName()));
	}

	// warn if END bone was not a child of START bone
	if (ResolvedChain.bFoundEndBone && !ResolvedChain.bEndIsStartOrChildOfStart)
	{
		Log.LogWarning("EndNotChildtOfStart, IK Retargeter bone chain, {%s}, end bone, '{%s}' was not a child of the start bone '{%s}'.",
			FText::FromName(BoneChain.ChainName), FText::FromName(BoneChain.EndBone.BoneName), FText::FromName(BoneChain.StartBone.BoneName));
	}
	
	return ResolvedChain.IsValid();
}

bool FRetargetChainPairFK::Initialize(
	const FBoneChain& SourceBoneChain,
	const FBoneChain& TargetBoneChain,
	const FRetargetSkeleton& SourceSkeleton,
	const FTargetSkeleton& TargetSkeleton,
	FIKRigLogger& Log)
{
	const bool bChainInitialized = FRetargetChainPair::Initialize(SourceBoneChain, TargetBoneChain, SourceSkeleton, TargetSkeleton, Log);
	if (!bChainInitialized)
	{
		return false;
	}

	// initialize SOURCE FK chain encoder with retarget pose
	const bool bFKEncoderInitialized = FKEncoder.Initialize(SourceSkeleton, SourceBoneIndices, SourceSkeleton.RetargetGlobalPose, Log);
	if (!bFKEncoderInitialized)
	{
		Log.LogWarning( "BadFKEncoder, IK Retargeter failed to initialize FK encoder, '{%s}', on Skeletal Mesh: '{%s}'",
			FText::FromName(SourceBoneChainName), FText::FromName(SourceSkeleton.Skeleton->GetName()));
		return false;
	}

	// initialize TARGET FK chain decoder with retarget pose
	const bool bFKDecoderInitialized = FKDecoder.Initialize(TargetSkeleton, TargetBoneIndices, TargetSkeleton.RetargetGlobalPose, Log);
	if (!bFKDecoderInitialized)
	{
		Log.LogWarning("BadFKDecoder, IK Retargeter failed to initialize FK decoder, '{%s}', on Skeletal Mesh: '{%s}'",
			FText::FromName(TargetBoneChainName), FText::FromName(TargetSkeleton.Skeleton->GetName()));
		return false;
	}

	// initialize the pole vector matcher for this chain
	// const bool bPoleVectorMatcherInitialized = PoleVectorMatcher.Initialize(
	// 	SourceBoneIndices,
	// 	TargetBoneIndices,
	// 	SourceSkeleton.RetargetGlobalPose,
	// 	TargetSkeleton.RetargetGlobalPose,
	// 	TargetSkeleton);
	// if (!bPoleVectorMatcherInitialized)
	// {
	// 	Log.LogWarning( FText::Format(
	// 		LOCTEXT("BadPoleVectorMatcher", "IK Retargeter failed to initialize pole matching for chain, '{0}', on Skeletal Mesh: '{1}'"),
	// 		FText::FromName(TargetBoneChainName), FText::FromString(TargetSkeleton.SkeletalMesh->GetName())));
	// 	return false;
	// }

	return true;
}



bool FRetargetChainPairIK::Initialize(
	const FBoneChain& SourceBoneChain,
	const FBoneChain& TargetBoneChain,
	const FRetargetSkeleton& SourceSkeleton,
	const FTargetSkeleton& TargetSkeleton,
	FIKRigLogger& Log)
{
	// todo

	return true;
}


// MARK: - root retargeter

bool FRootRetargeter::InitializeSource(
	const ERootType SourceRootType,
	const FName SourceRootBoneName,
	const FName SourceGroundBoneName,
	const FRetargetSkeleton& SourceSkeleton,
	FIKRigLogger& Log)
{
	// validate target root bone exists
	Source.RootType = SourceRootType;
	Source.BoneName = SourceRootBoneName;
	Source.BoneIndex = SourceSkeleton.FindBoneIndexByName(SourceRootBoneName);
	if (Source.BoneIndex == INDEX_NONE)
	{
		//Log.LogWarning("MissingSourceRoot", "IK Retargeter could not find source root bone, {0} in mesh {1}"),
		//	FText::FromName(SourceRootBoneName), FText::FromString(SourceSkeleton.SkeletalMesh->GetName())));
		return false;
	}
	
	// record initial root data
	const FTransform InitialTransform = SourceSkeleton.RetargetGlobalPose[Source.BoneIndex];
	float InitialHeight = static_cast<float>(InitialTransform.GetTranslation().z);
	if (Source.RootType == ERootType::RootZMinusGroundZ) {
		int32 groundBoneIndex = SourceSkeleton.FindBoneIndexByName(SourceGroundBoneName);
		const FTransform InitialTransformGround = SourceSkeleton.RetargetGlobalPose[groundBoneIndex]; 
		if (groundBoneIndex == -1) {
			Log.LogError("NoGroundBone", "must set GroundBone");
			return false;
		}
		InitialHeight = static_cast<float>(InitialTransform.GetTranslation().z - InitialTransformGround.GetTranslation().z);
	}
	Source.InitialRotation = InitialTransform.GetRotation();

	#ifdef DEBUG_POSE_LOG_ROOT
	printf("init srcroot: root.t(%.2f %.2f %.2f) ground.t(%.2f %.2f %.2f)\n",
		InitialTransform.GetTranslation().x, InitialTransform.GetTranslation().y, InitialTransform.GetTranslation().z, 
		InitialTransformGround.GetTranslation().x, InitialTransformGround.GetTranslation().y, InitialTransformGround.GetTranslation().z
	);
	#endif

	// ensure root height is not at origin, this happens if user sets root to ACTUAL skeleton root and not pelvis
	if (InitialHeight < KINDA_SMALL_NUMBER)
	{
		// warn user and push it up slightly to avoid divide by zero
		Log.LogError("BadRootHeight, The source retarget root bone is very near the ground plane. This will cause the target to be moved very far. To resolve this, please create a retarget pose with the retarget root at the correct height off the ground.");
		InitialHeight = 1.0f;
	}

	// invert height
	Source.InitialHeightInverse = 1.0f / InitialHeight;

	return true;
}

bool FRootRetargeter::InitializeTarget(
	const ERootType TargetRootType,
	const FName TargetRootBoneName,
	const FName TargetGroundBoneName,
	const FTargetSkeleton& TargetSkeleton,
	FIKRigLogger& Log)
{
	// validate target root bone exists
	Target.RootType = TargetRootType;
	Target.BoneName = TargetRootBoneName;
	Target.BoneIndex = TargetSkeleton.FindBoneIndexByName(TargetRootBoneName);
	if (Target.BoneIndex == INDEX_NONE)
	{
		//Log.LogWarning( FText::Format(
		//	LOCTEXT("CountNotFindRootBone", "IK Retargeter could not find target root bone, {0} in mesh {1}"),
		//	FText::FromName(TargetRootBoneName), FText::FromString(TargetSkeleton.SkeletalMesh->GetName())));
		return false;
	}

	const FTransform TargetInitialTransform = TargetSkeleton.RetargetGlobalPose[Target.BoneIndex];
	Target.InitialHeight = static_cast<float>(TargetInitialTransform.GetTranslation().z);
	if (Target.RootType == ERootType::RootZMinusGroundZ) {
		int32 groundBoneIndex = TargetSkeleton.FindBoneIndexByName(TargetGroundBoneName);
		if (groundBoneIndex == -1) {
			Log.LogError("NoGroundBone", "must set GroundBone");
			return false;
		}
		const FTransform TargetTransformGround = TargetSkeleton.RetargetGlobalPose[groundBoneIndex]; 
		Target.InitialHeight = static_cast<float>(TargetInitialTransform.GetTranslation().z - TargetTransformGround.GetTranslation().z);
	}
	Target.InitialRotation = TargetInitialTransform.GetRotation();
	Target.InitialPosition = TargetInitialTransform.GetTranslation();

	#ifdef DEBUG_POSE_LOG_ROOT
	printf("init tgtroot: root.t(%.2f %.2f %.2f) ground.t(%.2f %.2f %.2f)\n",
		TargetInitialTransform.GetTranslation().x, TargetInitialTransform.GetTranslation().y, TargetInitialTransform.GetTranslation().z, 
		TargetTransformGround.GetTranslation().x, TargetTransformGround.GetTranslation().y, TargetTransformGround.GetTranslation().z
	);
	#endif

	// initialize the global scale factor
	const float ScaleFactor = Source.InitialHeightInverse * Target.InitialHeight;
	GlobalScaleFactor.Set(ScaleFactor, ScaleFactor, ScaleFactor);
	
	return true;
}

void FRootRetargeter::Reset()
{
	Source = FRootSource();
	Target = FRootTarget();
}

void FRootRetargeter::EncodePose(const TArray<FTransform>& SourceGlobalPose)
{
	const FTransform& SourceTransform = SourceGlobalPose[Source.BoneIndex];
	Source.CurrentPosition = SourceTransform.GetTranslation();
	Source.CurrentPositionNormalized = Source.CurrentPosition * Source.InitialHeightInverse;
	Source.CurrentRotation = SourceTransform.GetRotation();	

	#ifdef DEBUG_POSE_LOG_ROOT
	printf("root encode: t(%.2f %.2f %.2f) inverseHeight:%.2f\n", 
		Source.CurrentPosition.x, Source.CurrentPosition.y, Source.CurrentPosition.z, Source.InitialHeightInverse
	);
	#endif
}

void FRootRetargeter::DecodePose(TArray<FTransform>& OutTargetGlobalPose)
{
	// retarget position
	FVector Position;
	{
		// generate basic retarget root position by scaling the normalized position by root height
		const FVector RetargetedPosition = Source.CurrentPositionNormalized * Target.InitialHeight;

		#ifdef DEBUG_POSE_LOG_ROOT
		printf("root decode RetargetedPosition: t(%.2f %.2f %.2f) height:%.2f\n", 
			RetargetedPosition.x, RetargetedPosition.y, RetargetedPosition.z, Target.InitialHeight
		);
		#endif
		
		// blend the retarget root position towards the source retarget root position
		Position = FVector::lerp(RetargetedPosition, Source.CurrentPosition, Settings.BlendToSource*Settings.BlendToSourceWeights);

		// apply vertical / horizontal scaling of motion
		FVector ScaledRetargetedPosition = Position;
		ScaledRetargetedPosition.z *= Settings.ScaleVertical;
		const FVector HorizontalOffset = (ScaledRetargetedPosition - Target.InitialPosition) * FVector(Settings.ScaleHorizontal, Settings.ScaleHorizontal, 1.0f);
		Position = Target.InitialPosition + HorizontalOffset;
		
		// apply a static offset
		Position += Settings.TranslationOffset;

		// blend with alpha
		Position = FVector::lerp(Target.InitialPosition, Position, Settings.TranslationAlpha);

		// record the delta created by all the modifications made to the root translation
		Target.RootTranslationDelta = Position - RetargetedPosition;
	}

	// retarget rotation
	FQuat Rotation;
	{
		// calc offset between initial source/target root rotations
		const FQuat RotationDelta = Source.CurrentRotation * Source.InitialRotation.Inverse();
		// add retarget pose delta to the current source rotation
		const FQuat RetargetedRotation = RotationDelta * Target.InitialRotation;

		// add static rotation offset
		Rotation = RetargetedRotation * Settings.RotationOffset.Quaternion();

		// blend with alpha
		Rotation = FQuat::FastLerp(Target.InitialRotation, Rotation, Settings.RotationAlpha);
		Rotation.Normalize();

		// record the delta created by all the modifications made to the root rotation
		Target.RootRotationDelta = RetargetedRotation * Target.InitialRotation.Inverse();
	}

	// apply to target
	FTransform& TargetRootTransform = OutTargetGlobalPose[Target.BoneIndex];
	TargetRootTransform.SetTranslation(Position);
	TargetRootTransform.SetRotation(Rotation);
}




UIKRetargetProcessor::UIKRetargetProcessor()
{
}

// MARK: - init
/* #region MARK: -init */

void UIKRetargetProcessor::Initialize(
		USkeleton* InSourceSkeleton,
		USkeleton* InTargetSkeleton,
		UIKRetargeter* InRetargeterAsset,
		const bool bSuppressWarnings)
{
	// reset all initialized flags
	bIsInitialized = false;
	bRootsInitialized = false;
	bAtLeastOneValidBoneChainPair = false;
	bIKRigInitialized = false;
	
	// record source asset
	RetargeterAsset = InRetargeterAsset;
	if (InRetargeterAsset->GlobalSettings != nullptr) {
		GlobalSettings = InRetargeterAsset->GlobalSettings->Settings;
	}

	const UIKRigDefinition* SourceIKRig = RetargeterAsset->GetSourceIKRig();
	const UIKRigDefinition* TargetIKRig = RetargeterAsset->GetTargetIKRig();

	// check prerequisite assets
	if (!InSourceSkeleton) {
		Log.LogError("MissingSourceSkeleton");
		return;
	}
	if (!InTargetSkeleton) {
		Log.LogError("MissingTargetSkeleton");
		return;
	}
	if (!SourceIKRig) {
		Log.LogError("MissingSourceIKRig");
		return;
	}
	if (!TargetIKRig) {
		Log.LogError("MissingTargetIKRig");
		return;
	}
	

	// ref pose on ref pose, no retarget pose
	FName InRetargetPoseName = "None";
	FIKRetargetPose* RetargetPose = nullptr;

	// initialize skeleton data for source and target
	SourceSkeleton.Initialize(
		InSourceSkeleton,
		SourceIKRig->GetRetargetChains(),
		InRetargetPoseName,
		RetargetPose,
		//RetargeterAsset->GetCurrentRetargetPoseName(ERetargetSourceOrTarget::Source),
		//RetargeterAsset->GetCurrentRetargetPose(ERetargetSourceOrTarget::Source),
		SourceIKRig->GetRetargetRoot());
	
	TargetSkeleton.Initialize(
		InTargetSkeleton,
		TargetIKRig->GetRetargetChains(),
		InRetargetPoseName,//RetargeterAsset->GetCurrentRetargetPoseName(ERetargetSourceOrTarget::Target),
		RetargetPose,//RetargeterAsset->GetCurrentRetargetPose(ERetargetSourceOrTarget::Target),
		TargetIKRig->GetRetargetRoot());

	// initialize roots
	bRootsInitialized = InitializeRoots();
	if (!bRootsInitialized) {
		bIsInitialized = false;
		return;
	}

	// initialize pairs of bone chains
	bAtLeastOneValidBoneChainPair = InitializeBoneChainPairs();
	if (!bAtLeastOneValidBoneChainPair)
	{
		// couldn't match up any BoneChain pairs, no limb retargeting possible
		// Log.LogWarning( 
		// 	"NoMappedChains, unable to map any bone chains between source, %s and target, %s",
		// 	SourceSkeletalMesh->GetName(), TargetSkeletalMesh->GetName());
	}

	// initialize the IKRigProcessor for doing IK decoding
	bIKRigInitialized = InitializeIKRig(this, InTargetSkeleton);
	if (!bIKRigInitialized)
	{
		// couldn't initialize the IK Rig, we don't disable the retargeter in this case, just warn the user
		// Log.LogWarning( 
        //     "CouldNotInitializeIKRig, unable to initialize the IK Rig, %s for the Skeletal Mesh %s.",
		// 	TargetIKRig->GetName(), TargetSkeletalMesh->GetName());
	}

	// must have a mapped root bone OR at least a single mapped chain to be able to do any retargeting at all
	if (bRootsInitialized && bAtLeastOneValidBoneChainPair)
	{
		// confirm for the user that the IK Rig was initialized successfully
		// Log.LogInfo(
        //     "SuccessfulInit, Success! ready to transfer animation from the source, %s to the target, %s",
		// 		SourceSkeleton->GetName(), TargetSkeleton->GetName());
	}

	// copy the initial settings from the asset
	//ApplySettingsFromAsset();
	
	bIsInitialized = true;
}

bool UIKRetargetProcessor::InitializeRoots()
{
	// reset root data
	RootRetargeter.Reset();
	
	// initialize root encoder
	const FName SourceRootBoneName = RetargeterAsset->GetSourceIKRig()->GetRetargetRoot();
	const FName SourceGroundBoneName = RetargeterAsset->GetSourceIKRig()->GetRetargetGround();
	const ERootType SourceRootType = RetargeterAsset->GetSourceIKRig()->GetRootType();
	const bool bRootEncoderInit = RootRetargeter.InitializeSource(SourceRootType, SourceRootBoneName, SourceGroundBoneName, SourceSkeleton, Log);
	if (!bRootEncoderInit)
	{
		//Log.LogWarning( FText::Format(
		//	LOCTEXT("NoSourceRoot", "IK Retargeter unable to initialize source root, '{0}' on skeletal mesh: '{1}'"),
		//	FText::FromName(SourceRootBoneName), FText::FromString(SourceSkeleton.SkeletalMesh->GetName())));
	}

	// initialize root decoder
	const FName TargetRootBoneName = RetargeterAsset->GetTargetIKRig()->GetRetargetRoot();
	const FName TargetGroundBoneName = RetargeterAsset->GetTargetIKRig()->GetRetargetGround();
	const ERootType TargetRootType = RetargeterAsset->GetTargetIKRig()->GetRootType();
	const bool bRootDecoderInit = RootRetargeter.InitializeTarget(TargetRootType, TargetRootBoneName, TargetGroundBoneName, TargetSkeleton, Log);
	if (!bRootDecoderInit)
	{
		// Log.LogWarning( FText::Format(
		// 	LOCTEXT("NoTargetRoot", "IK Retargeter unable to initialize target root, '{0}' on skeletal mesh: '{1}'"),
		// 	FText::FromName(TargetRootBoneName), FText::FromString(TargetSkeleton.SkeletalMesh->GetName())));
	}

	return bRootEncoderInit && bRootDecoderInit;
}

bool UIKRetargetProcessor::InitializeBoneChainPairs()
{
	ChainPairsFK.clear();
	ChainPairsIK.clear();
	
	const UIKRigDefinition* TargetIKRig = RetargeterAsset->GetTargetIKRig();
	const UIKRigDefinition* SourceIKRig = RetargeterAsset->GetSourceIKRig();
	
	//check(SourceIKRig && TargetIKRig);

	// check that chains are available in both IKRig assets before sorting them based on StartBone index
	const TArray<std::shared_ptr<URetargetChainSettings>>& ChainMapping = RetargeterAsset->GetAllChainSettings();	
	for (std::shared_ptr<URetargetChainSettings> ChainMap : ChainMapping)
	{
		// get target bone chain
		const FBoneChain* TargetBoneChain = RetargeterAsset->GetTargetIKRig()->GetRetargetChainByName(ChainMap->TargetChain);
		if (!TargetBoneChain)
		{
			// Log.LogWarning( FText::Format(
			// LOCTEXT("MissingTargetChain", "IK Retargeter missing target bone chain: {0}. Please update the mapping."),
			// FText::FromString(ChainMap->TargetChain.ToString())));
			continue;
		}
		
		// user opted to not map this to anything, we don't need to spam a warning about it
		if (ChainMap->SourceChain == NAME_None)
		{
			continue; 
		}
		
		// get source bone chain
		const FBoneChain* SourceBoneChain = RetargeterAsset->GetSourceIKRig()->GetRetargetChainByName(ChainMap->SourceChain);
		if (!SourceBoneChain)
		{
			// Log.LogWarning( FText::Format(
			// LOCTEXT("MissingSourceChain", "IK Retargeter missing source bone chain: {0}"),
			// FText::FromString(ChainMap->SourceChain.ToString())));
			continue;
		}

		// all chains are loaded as FK (giving IK better starting pose)
		FRetargetChainPairFK ChainPair;
		if (ChainPair.Initialize(*SourceBoneChain, *TargetBoneChain, SourceSkeleton, TargetSkeleton, Log))
		{
			ChainPairsFK.push_back(ChainPair);
		}
		
		// load IK chain
		if (ChainMap->Settings.IK.EnableIK)
		{
			FRetargetChainPairIK ChainPairIK;
			if (ChainPairIK.Initialize(*SourceBoneChain, *TargetBoneChain, SourceSkeleton, TargetSkeleton, Log))
			{
				ChainPairsIK.push_back(ChainPairIK);
			}
		}

		// todo ik
		// warn user if IK goal is not on the END bone of the target chain. It will still work, but may produce bad results.
		// const TArray<UIKRigEffectorGoal*> AllGoals = TargetIKRig->GetGoalArray();
		// for (const UIKRigEffectorGoal*Goal : AllGoals)
		// {
		// 	if (Goal->GoalName == TargetBoneChain->IKGoalName)
		// 	{
		// 		if (Goal->BoneName != TargetBoneChain->EndBone.BoneName)
		// 		{
		// 		// 	Log.LogWarning( FText::Format(
		// 		// LOCTEXT("TargetIKNotOnEndBone", "Retarget chain, '{0}' has an IK goal that is not on the End Bone of the chain."),
		// 		// 	FText::FromString(TargetBoneChain->ChainName.ToString())));
		// 		}
		// 		break;
		// 	}
		// }
	}

	// sort the chains based on their StartBone's index
	auto ChainsSorter = [this](const FRetargetChainPair& A, const FRetargetChainPair& B)
	{
		const int32 IndexA = A.TargetBoneIndices.size() > 0 ? A.TargetBoneIndices[0] : INDEX_NONE;
		const int32 IndexB = B.TargetBoneIndices.size() > 0 ? B.TargetBoneIndices[0] : INDEX_NONE;
		if (IndexA == IndexB)
		{
			// fallback to sorting alphabetically
			return A.TargetBoneChainName.compare(B.TargetBoneChainName) < 0; // bugfix: <= -> <
		}
		return IndexA < IndexB;
	};
	
	std::sort(ChainPairsFK.begin(), ChainPairsFK.end(), ChainsSorter);
	std::sort(ChainPairsIK.begin(), ChainPairsIK.end(), ChainsSorter);
	
	// record which bones in the target skeleton are being retargeted
	for (const FRetargetChainPairFK& FKChainPair : ChainPairsFK)
	{
		for (const int32 BoneIndex : FKChainPair.TargetBoneIndices)
		{
			TargetSkeleton.SetBoneIsRetargeted(BoneIndex, true);
		}
	}

	// record intermediate bones (non-retargeted bones located BETWEEN FK chains on the target skeleton)
	for (FRetargetChainPairFK& FKChainPair : ChainPairsFK)
	{
		FKChainPair.FKDecoder.InitializeIntermediateParentIndices(
			RootRetargeter.Target.BoneIndex,
			FKChainPair.TargetBoneIndices[0],
			TargetSkeleton);
	}

	// root is updated before IK as well
	if (bRootsInitialized)
	{
		TargetSkeleton.SetBoneIsRetargeted(RootRetargeter.Target.BoneIndex, true);	
	}

	// return true if at least 1 pair of bone chains were initialized
	return !(ChainPairsIK.empty() && ChainPairsFK.empty());
}

bool UIKRetargetProcessor::InitializeIKRig(UObject* Outer, const USkeleton* InSkeleton)
{
	// TODO	
	return true;
}


/* #endregion */



// MARK: - run retarget
/* #region MARK: -run retarget */

TArray<FTransform>&  UIKRetargetProcessor::RunRetargeter(
	const TArray<FTransform>& InSourceGlobalPose,
	const std::unordered_map<FName, float>& SpeedValuesFromCurves,
	const float DeltaTime)
{
	//check(bIsInitialized);
		
	// start from retarget pose
	TargetSkeleton.OutputGlobalPose = TargetSkeleton.RetargetGlobalPose;
	//return TargetSkeleton.OutputGlobalPose;

	// ROOT retargeting
	if (GlobalSettings.bEnableRoot && bRootsInitialized)
	{
		RunRootRetarget(InSourceGlobalPose, TargetSkeleton.OutputGlobalPose);
		// update global transforms below root
		TargetSkeleton.UpdateGlobalTransformsBelowBone(RootRetargeter.Target.BoneIndex, TargetSkeleton.RetargetLocalPose, TargetSkeleton.OutputGlobalPose);
	}
	
	// FK CHAIN retargeting
	if (GlobalSettings.bEnableFK && bAtLeastOneValidBoneChainPair)
	{
		RunFKRetarget(InSourceGlobalPose, TargetSkeleton.OutputGlobalPose);
		// update all the bones that are not controlled by FK chains or root
		TargetSkeleton.UpdateGlobalTransformsAllNonRetargetedBones(TargetSkeleton.OutputGlobalPose);
	}
	
	// IK CHAIN retargeting
	if (GlobalSettings.bEnableIK && bAtLeastOneValidBoneChainPair && bIKRigInitialized)
	{
		RunIKRetarget(InSourceGlobalPose, TargetSkeleton.OutputGlobalPose, SpeedValuesFromCurves, DeltaTime);
	}

	// Pole Vector matching between source / target chains
	if (GlobalSettings.bEnableFK && bAtLeastOneValidBoneChainPair)
	{
		RunPoleVectorMatching(InSourceGlobalPose, TargetSkeleton.OutputGlobalPose);
	}

	return TargetSkeleton.OutputGlobalPose;
}

void UIKRetargetProcessor::RunRootRetarget(
	const TArray<FTransform>& InGlobalTransforms,
    TArray<FTransform>& OutGlobalTransforms)
{
	RootRetargeter.EncodePose(InGlobalTransforms);
	RootRetargeter.DecodePose(OutGlobalTransforms);
}

void UIKRetargetProcessor::RunFKRetarget(
	const TArray<FTransform>& InGlobalTransforms,
    TArray<FTransform>& OutGlobalTransforms)
{
	// spin through chains and encode/decode them all using the input pose
	for (FRetargetChainPairFK& ChainPair : ChainPairsFK)
	{
		ChainPair.FKEncoder.EncodePose(
			SourceSkeleton,
			ChainPair.SourceBoneIndices,
			InGlobalTransforms);
		
		ChainPair.FKDecoder.DecodePose(
			RootRetargeter,
			ChainPair.Settings,
			ChainPair.TargetBoneIndices,
			ChainPair.FKEncoder,
			TargetSkeleton,
			OutGlobalTransforms);
	}
}

void UIKRetargetProcessor::RunIKRetarget(
	const TArray<FTransform>& InSourceGlobalPose,
    TArray<FTransform>& OutTargetGlobalPose,
    const std::unordered_map<FName, float>& SpeedValuesFromCurves,
    const float DeltaTime)
{
	// todo
}

void UIKRetargetProcessor::RunPoleVectorMatching(const TArray<FTransform>& InGlobalTransforms, TArray<FTransform>& OutGlobalTransforms) {
	// todo
}

void UIKRetargetProcessor::RunStrideWarping(const TArray<FTransform>& InTargetGlobalPose)
{
	// todo
}

/* #endregion */
