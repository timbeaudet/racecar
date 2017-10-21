///
/// @file
/// @details This is a basic simulation for a 5 speed h-shifter transmission.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_transmission.h"
#include "racecar_controller.h"

//--------------------------------------------------------------------------------------------------------------------//

constexpr Racecar::Gear UpshiftGear(const Racecar::Gear& gear)
{
	return
		(Racecar::Gear::Reverse == gear) ? Racecar::Gear::Neutral :
		(Racecar::Gear::Neutral == gear) ? Racecar::Gear::First :
		(Racecar::Gear::First == gear) ? Racecar::Gear::Second :
		(Racecar::Gear::Second == gear) ? Racecar::Gear::Third :
		(Racecar::Gear::Third == gear) ? Racecar::Gear::Fourth : Racecar::Gear::Fifth;
}

constexpr Racecar::Gear DownshiftGear(const Racecar::Gear& gear)
{
	return
		(Racecar::Gear::Fifth == gear) ? Racecar::Gear::Fourth :
		(Racecar::Gear::Fourth == gear) ? Racecar::Gear::Third :
		(Racecar::Gear::Third == gear) ? Racecar::Gear::Second :
		(Racecar::Gear::Second == gear) ? Racecar::Gear::First :
		(Racecar::Gear::First == gear) ? Racecar::Gear::Neutral : Racecar::Gear::Reverse;
}

//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

Racecar::Transmission::Transmission(const Real momentOfInertia, const std::array<Real, 8>& gearRatios) :
	RotatingBody(momentOfInertia),
	mInputShaftSpeed(0.0),
	mOutputShaftSpeed(0.0),
	mSelectedGear(Gear::Neutral),
	mHasClearedShift(true),
	mGearJoints{ GearJoint(100.0), GearJoint(gearRatios[1]), GearJoint(gearRatios[2]), GearJoint(gearRatios[3]), 
		GearJoint(gearRatios[4]), GearJoint(gearRatios[5]), GearJoint(gearRatios[6]), GearJoint(fabs(gearRatios[7]) < kElipson ? 100.0 : gearRatios[7]) }
{
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Transmission::~Transmission(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::Simulate(const RacecarControllerInterface& racecarController, const Real& fixedTime)
{
	((void)fixedTime);

	SimulateShiftLogic(racecarController);

	RotatingBody& inputSource(GetExpectedInputSource());
	mInputShaftSpeed = inputSource.GetAngularVelocity();
	
	if (Gear::Neutral == mSelectedGear)
	{
		//Do nothing.
	}
	else
	{
		mOutputShaftSpeed = mInputShaftSpeed / GetSelectedGearRatio();
		SetAngularVelocity(mOutputShaftSpeed);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Transmission::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	if (Gear::Neutral == mSelectedGear)
	{
		return 0.0;
	}

	return RotatingBody::ComputeDownstreamInertia(fromSource) / GetSelectedGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Transmission::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	if (Gear::Neutral == mSelectedGear)
	{
		return 0.0;
	}

	return RotatingBody::ComputeUpstreamInertia(fromSource) * GetSelectedGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource)
{
	if (Gear::Neutral != mSelectedGear)
	{
		RotatingBody::OnApplyDownstreamAcceleration(changeInAcceleration / GetSelectedGearRatio(), fromSource);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource)
{
	if (Gear::Neutral != mSelectedGear)
	{
		RotatingBody::OnApplyUpstreamAcceleration(changeInAcceleration * GetSelectedGearRatio(), fromSource);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::SimulateShiftLogic(const RacecarControllerInterface& racecarController)
{
	if (true == mHasClearedShift)
	{		
		if (true == racecarController.IsUpshift())
		{	//Upshift
			mSelectedGear = UpshiftGear(mSelectedGear);
			mHasClearedShift = false;
		}
		else if (true == racecarController.IsDownshift())
		{	//Downshift
			mSelectedGear = DownshiftGear(mSelectedGear);
			mHasClearedShift = false;
		}
	}
	else
	{
		if (false == racecarController.IsUpshift() && false == racecarController.IsDownshift())
		{
			mHasClearedShift = true;
		}
	}
}

Racecar::Real Racecar::Transmission::GetSelectedGearRatio(void) const
{
	error_if(Gear::Neutral == mSelectedGear, "Cannot use this while in neutral.");
	return mGearJoints[static_cast<int>(mSelectedGear)].GetGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

Racecar::GearJoint::GearJoint(Real gearRatio) :
	mGearRatio(gearRatio)
{
	error_if(fabs(mGearRatio) < 0.01, "Error: gearRatio too close to zero.");
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::GearJoint::~GearJoint(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//
