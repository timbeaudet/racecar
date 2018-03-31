///
/// @file
/// @details This is a basic simulation for a 5 speed h-shifter transmission.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_transmission.h"
#include "racecar_controller.h"

#include "racecar_clutch.h" //For the ClutchJoint to perform a frictional synchromesh.

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

Racecar::Transmission::Transmission(const Real momentOfInertia, const std::array<Real, 6>& gearRatios, const Real& reverseRatio) :
	RotatingBody(momentOfInertia),
	mSelectedGear(Gear::Neutral),
	mHasClearedShift(true),
	mIsSynchromeshBox(false),
	mHasUsedShifter(false),
	mGearJoints{ GearJoint(100.0), GearJoint(gearRatios[0]), GearJoint(gearRatios[1]), GearJoint(gearRatios[2]),
		GearJoint(gearRatios[3]), GearJoint(gearRatios[4]), GearJoint(fabs(gearRatios[5]) < kEpsilon ? 100.0 : gearRatios[5]),
		GearJoint(reverseRatio > -kEpsilon ? 100.0 : reverseRatio) }
{
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Transmission::~Transmission(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnControllerChange(const RacecarControllerInterface& racecarController)
{
	if (racecarController.GetShifterPosition() != Gear::Neutral)
	{
		mHasUsedShifter = true;
	}

	if (false == mHasUsedShifter)
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
	else
	{
		mSelectedGear = racecarController.GetShifterPosition();
	}
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnSimulate(const Real& fixedTime)
{
	if (Gear::Neutral == mSelectedGear)
	{
		//Do nothing.
	}
	else
	{
		RotatingBody& inputSource(GetExpectedInputSource());
		GearJoint& currentGearJoint(mGearJoints[static_cast<size_t>(GetSelectedGear())]);
		
		const Real matchImpulse = currentGearJoint.ComputeTorqueImpulse(inputSource, *this);
		if (true == mIsSynchromeshBox)
		{
			const Real frictionImpulse = 10 * 0.45 * fixedTime;
			const Real appliedImpulse((fabs(matchImpulse) > frictionImpulse) ? frictionImpulse * Sign(matchImpulse) : matchImpulse);
			GetExpectedInputSource().ApplyUpstreamAngularImpulse(appliedImpulse);
			ApplyDownstreamAngularImpulse(-appliedImpulse);
		}
		else
		{	//It appears the "dog collar" boxes just slam into required speeds.
			GetExpectedInputSource().ApplyUpstreamAngularImpulse(matchImpulse);
			ApplyDownstreamAngularImpulse(-matchImpulse);
//			error_if(fabs(GetAngularVelocity() - GetExpectedInputSource().GetAngularVelocity() / GetSelectedGearRatio()) > Racecar::kEpsilon,
//				"InternalError: The output shaft is not rotating at the correct speed.");
		}
	}

	RotatingBody::OnSimulate(fixedTime);
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Transmission::ComputeDownstreamInertia(void) const
{
	if (Gear::Neutral == mSelectedGear)
	{
		return 0.0;
	}

	//return RotatingBody::ComputeDownstreamInertia() / fabs(GetSelectedGearRatio());

	//https://www.servo2go.com/support/files/Smart%20Motion%20Cheat%20Sheet%20Rev3.pdf
	const Real oneOverRatioSquared((1 / GetSelectedGearRatio()) * (1 / GetSelectedGearRatio()));
	return RotatingBody::ComputeDownstreamInertia() * oneOverRatioSquared;
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Transmission::ComputeUpstreamInertia(void) const
{
	if (Gear::Neutral == mSelectedGear)
	{
		return GetInertia();
	}

	//return GetInertia() + upstreamInertia * fabs(GetSelectedGearRatio());

	const RotatingBody* inputSource(GetInputSource());
	const Real upstreamInertia((nullptr == inputSource) ? 0.0 : inputSource->ComputeUpstreamInertia());
	const Real ratioSquared(GetSelectedGearRatio() * GetSelectedGearRatio());
	return GetInertia() + upstreamInertia * ratioSquared;
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	if (Gear::Neutral != mSelectedGear)
	{
		RotatingBody::OnDownstreamAngularVelocityChange(changeInAngularVelocity / GetSelectedGearRatio());
	}
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	SetAngularVelocity(GetAngularVelocity() + changeInAngularVelocity);

	if (Gear::Neutral != mSelectedGear)
	{
		RotatingBody* inputSource(GetInputSource());
		if (nullptr != inputSource)
		{
			inputSource->OnUpstreamAngularVelocityChange(changeInAngularVelocity * GetSelectedGearRatio());
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

Racecar::Real Racecar::GearJoint::ComputeTorqueImpulse(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep)
{
	((void)fixedTimeStep);
	return ComputeTorqueImpulseToMatchVelocity(input, output);
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::GearJoint::ComputeTorqueImpulseToMatchVelocity(const RotatingBody& input, const RotatingBody& output)
{
	//   J = (Io * Ii * (Wo * gr - Wi)) / (Io + Ii * gr)
	//
	// where J is an angular impulse (kg*m^s / s)
	//       Io / Ii is Inertia of output / input bodies respectively.
	//       Wo / Wi is Angular Velocity (rad/sec) of output / input bodies.
	//       gr is the gear ratio.
	//
	//See: gear_ratio_impulse_proof.png for actual train of thought.

	const Real ratio(GetGearRatio());
	error_if(ratio < kEpsilon && ratio > -kEpsilon, "Expected gear ratio non-zero value.");

	const Real inputInertia(input.ComputeUpstreamInertia());
	const Real outputInertia(output.ComputeDownstreamInertia() * fabs(ratio));
	error_if(inputInertia < kEpsilon || outputInertia < kEpsilon, "Expected input and output inertia to be greater than zero.");

	const Real numerator = (inputInertia * outputInertia * Sign(ratio)) * (ratio * output.GetAngularVelocity() - input.GetAngularVelocity()); //Io*Ii*(r*Wo - Wi)
	const Real denominator = outputInertia + inputInertia * ratio; //Io + Ii * gr
	const Real torqueImpulse = numerator / denominator;
	return torqueImpulse * Sign(ratio);

	////
	//const Real ratio(GetGearRatio());
	//error_if(ratio < kEpsilon && ratio > -kEpsilon, "Expected gear ratio non-zero value.");

	//const Real inputInertia(input.ComputeUpstreamInertia());
	//const Real outputInertia(output.ComputeDownstreamInertia());
	//error_if(inputInertia < kEpsilon || outputInertia < kEpsilon, "Expected input and output inertia to be greater than zero.");

	//const Real numerator = (inputInertia * outputInertia) * (ratio * output.GetAngularVelocity() - input.GetAngularVelocity()); //Io*Ii*(r*Wo - Wi)
	//const Real denominator = outputInertia + inputInertia * ratio; //Io + Ii * gr
	//const Real torqueImpulse = numerator / denominator;
	//return torqueImpulse;
}

//--------------------------------------------------------------------------------------------------------------------//
