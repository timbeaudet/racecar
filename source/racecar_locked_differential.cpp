///
/// @file
/// @details This is a basic simulation for a locked differential, where both wheels spin same speed.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_locked_differential.h"

//--------------------------------------------------------------------------------------------------------------------//

Racecar::LockedDifferential::LockedDifferential(const Real& momentOfInertia, const Real& finalDriveRatio) :
	RotatingBody(momentOfInertia),
	mFinalDriveJoint(finalDriveRatio)
{
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::LockedDifferential::~LockedDifferential(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnSimulate(const Real& fixedTime)
{
	RotatingBody::OnSimulate(fixedTime);

	//const RotatingBody* inputSource(GetInputSource());
	//if (nullptr != inputSource)
	//{
	//	error_if(fabs(GetAngularVelocity() - (inputSource->GetAngularVelocity() / mFinalDriveRatio)) > Racecar::kElipson,
	//		"The wheel speed does not match the input velocity after Simulate().");
	//	//SetAngularVelocity(inputSource->GetAngularVelocity() / mFinalDriveRatio);
	//}
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::LockedDifferential::ComputeDownstreamInertia(void) const
{
	return RotatingBody::ComputeDownstreamInertia() / mFinalDriveJoint.GetGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::LockedDifferential::ComputeUpstreamInertia(void) const
{
	const RotatingBody* inputSource(GetInputSource());
	const Real upstreamInertia((nullptr == inputSource) ? 0.0 : inputSource->ComputeUpstreamInertia());
	return GetInertia() + upstreamInertia * mFinalDriveJoint.GetGearRatio();

	//return RotatingBody::ComputeUpstreamInertia(fromSource) * mFinalDriveJoint.GetGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	//const Real upstreamInertia((nullptr == GetInputSource()) ? 0.0 : GetInputSource()->ComputeUpstreamInertia(fromSource));
	//const Real downstreamInertia(RotatingBody::ComputeDownstreamInertia(fromSource));
	//const Real inputImpulse(changeInAngularVelocity * (upstreamInertia + (downstreamInertia / mFinalDriveJoint.GetGearRatio())));
	//const Real outputAngularVelocityChange(inputImpulse / (upstreamInertia + (downstreamInertia * mFinalDriveJoint.GetGearRatio())));

	//RotatingBody::OnDownstreamAngularVelocityChange(outputAngularVelocityChange, fromSource);
	RotatingBody::OnDownstreamAngularVelocityChange(changeInAngularVelocity / mFinalDriveJoint.GetGearRatio());
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	//const Real upstreamInertia((nullptr == GetInputSource()) ? 0.0 : GetInputSource()->ComputeUpstreamInertia(fromSource));
	//const Real downstreamInertia(RotatingBody::ComputeDownstreamInertia(fromSource));
	//const Real inputImpulse(changeInAngularVelocity * (upstreamInertia + (downstreamInertia * mFinalDriveJoint.GetGearRatio())));
	//const Real outputAngularVelocityChange(inputImpulse / (upstreamInertia + (downstreamInertia / mFinalDriveJoint.GetGearRatio())));

	//SetAngularVelocity(GetAngularVelocity() + changeInAngularVelocity);
	//
	//RotatingBody* inputSource(GetInputSource());
	//if (nullptr != inputSource)
	//{
	//	inputSource->OnUpstreamAngularVelocityChange(outputAngularVelocityChange, fromSource);
	//}



	SetAngularVelocity(GetAngularVelocity() + changeInAngularVelocity);

	RotatingBody* inputSource(GetInputSource());
	if (nullptr != inputSource)
	{
		inputSource->OnUpstreamAngularVelocityChange(changeInAngularVelocity * mFinalDriveJoint.GetGearRatio());
	}
}

//--------------------------------------------------------------------------------------------------------------------//
