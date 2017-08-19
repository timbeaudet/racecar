///
/// @file
/// @details This is a basic simulation for a locked differential, where both wheels spin same speed.
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_locked_differential.h"

//--------------------------------------------------------------------------------------------------------------------//

Racecar::LockedDifferential::LockedDifferential(const Real finalDriveRatio, const Real momentOfInertia) :
	RotatingBody(momentOfInertia),
	mFinalDriveRatio(finalDriveRatio)
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

	const RotatingBody* inputSource(GetInputSource());
	if (nullptr != inputSource)
	{
		SetAngularVelocity(inputSource->GetAngularVelocity() / mFinalDriveRatio);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::LockedDifferential::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	return RotatingBody::ComputeDownstreamInertia(fromSource);// / mFinalDriveRatio;
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::LockedDifferential::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	//return GetExpectedInputSource().ComputeUpstreamInertia(fromSource);// / mFinalDriveRatio + GetInertia();
	//return RotatingBody::ComputeUpstreamInertia(fromSource);
	return RotatingBody::ComputeUpstreamInertia(fromSource) * mFinalDriveRatio;
	//return GetInertia() + GetExpectedInputSource().ComputeUpstreamInertia(fromSource) * mFinalDriveRatio;
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnApplyDownstreamAcceleration(const Real changeInAcceleration, const RotatingBody& fromSource)
{
	AddAngularAcceleration(changeInAcceleration);
	for (RotatingBody* output : GetOutputSources())
	{
		output->OnApplyDownstreamAcceleration(changeInAcceleration / mFinalDriveRatio, fromSource);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnApplyUpstreamAcceleration(const Real changeInAcceleration, const RotatingBody& fromSource)
{
	AddAngularAcceleration(changeInAcceleration);

	RotatingBody* inputBody(GetInputSource());
	if (nullptr != inputBody)
	{
		inputBody->OnApplyUpstreamAcceleration(changeInAcceleration, fromSource);
	}
}

//--------------------------------------------------------------------------------------------------------------------//
