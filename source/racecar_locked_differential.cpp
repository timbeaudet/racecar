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

void Racecar::LockedDifferential::Simulate(RacecarControllerInterface& racecarController, const Real& fixedTime)
{
	((void)racecarController);
	((void)fixedTime);

	RotatingBody::Simulate(fixedTime);

	//const RotatingBody* inputSource(GetInputSource());
	//if (nullptr != inputSource)
	//{
	//	error_if(fabs(GetAngularVelocity() - (inputSource->GetAngularVelocity() / mFinalDriveRatio)) > Racecar::kElipson,
	//		"The wheel speed does not match the input velocity after Simulate().");
	//	//SetAngularVelocity(inputSource->GetAngularVelocity() / mFinalDriveRatio);
	//}
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::LockedDifferential::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	return RotatingBody::ComputeDownstreamInertia(fromSource) / mFinalDriveJoint.GetGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::LockedDifferential::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	return RotatingBody::ComputeUpstreamInertia(fromSource) * mFinalDriveJoint.GetGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource)
{
	RotatingBody::OnDownstreamAngularVelocityChange(changeInAngularVelocity / mFinalDriveJoint.GetGearRatio(), fromSource);
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource)
{
	RotatingBody::OnUpstreamAngularVelocityChange(changeInAngularVelocity * mFinalDriveJoint.GetGearRatio(), fromSource);
}

//--------------------------------------------------------------------------------------------------------------------//
