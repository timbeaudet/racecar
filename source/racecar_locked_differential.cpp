///
/// @file
/// @details This is a basic simulation for a locked differential, where both wheels spin same speed.
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_locked_differential.h"
#include "../drive_train_simulation.h" //for kFixedStep

//--------------------------------------------------------------------------------------------------------------------//

Racecar::LockedDifferential::LockedDifferential(const float finalDriveRatio) :
	mFinalDriveRatio(finalDriveRatio)
{
	SetInertia(Racecar::ComputeInertia(5.0f, 3.0f));
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::LockedDifferential::~LockedDifferential(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::LockedDifferential::Simulate(void)
{
	SetAngularVelocity(GetExpectedInputSource().GetAngularVelocity() / mFinalDriveRatio);
}

//--------------------------------------------------------------------------------------------------------------------//
