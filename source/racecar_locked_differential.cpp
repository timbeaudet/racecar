///
/// @file
/// @details This is a basic simulation for a locked differential, where both wheels spin same speed.
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_locked_differential.h"
#include "../drive_train_simulation.h" //for kFixedStep

//--------------------------------------------------------------------------------------------------------------------//

Racecar::LockedDifferential::LockedDifferential(const float finalDriveRatio, const float momentOfInertia) :
	RotatingBody(momentOfInertia),
	mFinalDriveRatio(finalDriveRatio)
{
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::LockedDifferential::~LockedDifferential(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::Simulate(RacecarControllerInterface& racecarController)
{
	((void)racecarController);
	SetAngularVelocity(GetExpectedInputSource().GetAngularVelocity() / mFinalDriveRatio);
}

//--------------------------------------------------------------------------------------------------------------------//

float Racecar::LockedDifferential::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	return RotatingBody::ComputeDownstreamInertia(fromSource);// / mFinalDriveRatio;
}

//--------------------------------------------------------------------------------------------------------------------//

float Racecar::LockedDifferential::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	//return GetExpectedInputSource().ComputeUpstreamInertia(fromSource);// / mFinalDriveRatio + GetInertia();
	return RotatingBody::ComputeUpstreamInertia(fromSource);
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource)
{
	AddAngularAcceleration(changeInAcceleration);
	for (RotatingBody* output : GetOutputSources())
	{
		output->OnApplyDownstreamAcceleration(changeInAcceleration / mFinalDriveRatio, fromSource);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::OnApplyUpstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource)
{
	AddAngularAcceleration(changeInAcceleration * mFinalDriveRatio);

	RotatingBody* inputBody(GetInputSource());
	if (nullptr != inputBody)
	{
		inputBody->OnApplyUpstreamAcceleration(changeInAcceleration * mFinalDriveRatio, fromSource);
	}
}

//--------------------------------------------------------------------------------------------------------------------//
