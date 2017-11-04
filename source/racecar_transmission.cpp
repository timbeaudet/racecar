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
	mInputShaftSpeed(0.0),
	mOutputShaftSpeed(0.0),
	mSelectedGear(Gear::Neutral),
	mHasClearedShift(true),
	//mGearJoints{ GearJoint(100.0), GearJoint(gearRatios[1]), GearJoint(gearRatios[2]), GearJoint(gearRatios[3]), 
	//	GearJoint(gearRatios[4]), GearJoint(gearRatios[5]), GearJoint(gearRatios[6]), GearJoint(fabs(gearRatios[7]) < kElipson ? 100.0 : gearRatios[7]) }
	mGearJoints{ GearJoint(100.0), GearJoint(gearRatios[0]), GearJoint(gearRatios[1]), GearJoint(gearRatios[2]),
		GearJoint(gearRatios[3]), GearJoint(gearRatios[4]), GearJoint(fabs(gearRatios[5]) < kElipson ? 100.0 : gearRatios[5]),
		GearJoint(reverseRatio > -kElipson ? 100.0 : reverseRatio) }
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
	mOutputShaftSpeed = GetAngularVelocity();
	//if (false == GetOutputSources().empty())
	//{
	//	mOutputShaftSpeed = GetExpectedOutputSource(0).GetAngularVelocity();
	//}

	if (Gear::Neutral == mSelectedGear)
	{
		//Do nothing.
	}
	else
	{
		//mOutputShaftSpeed = mInputShaftSpeed / GetSelectedGearRatio();
		//SetAngularVelocity(mOutputShaftSpeed);

		//This may not be 100% accurate, the syncrhomesh friction should only happen prior to the gear being selected.
		//In theory the output and input shaft speeds should be perfectly matched here after a shift is complete anyway,
		//so likely this will be good enough for now.
		if (fabs(mOutputShaftSpeed - mInputShaftSpeed) > kElipson)
		{
			//RotatingBody input(ComputeUpstreamInertia(*this) * GetSelectedGearRatio());
			//input.SetAngularVelocity(mInputShaftSpeed / GetSelectedGearRatio());

			//Real outputInertia(0.0);
			//const std::vector<RotatingBody*>& outputSource = GetOutputSources();
			//for (RotatingBody* body : outputSource)
			//{
			//	outputInertia += body->ComputeDownstreamInertia(*this);
			//}

			//RotatingBody output(outputInertia);
			//output.SetAngularVelocity(mOutputShaftSpeed);

			//ClutchJoint synchromeshFrictionJoint(0.6, 0.4);
			//synchromeshFrictionJoint.SetNormalForce(1000000.0);
			//Real frictionalImpulse = synchromeshFrictionJoint.ComputeTorqueImpulse(input, output, fixedTime);
			//ApplyUpstreamAngularImpulse(frictionalImpulse, *this);
			//ApplyDownstreamAngularImpulse(-frictionalImpulse, *this);

			//if (false == GetOutputSources().empty())
			//{
			//	mOutputShaftSpeed = GetExpectedOutputSource(0).GetAngularVelocity();
			//	SetAngularVelocity(mOutputShaftSpeed);
			//}



			///The more correct way:
			//if (false == GetOutputSources().empty())
			//{
			//	Real matchImpulse = mGearJoints[static_cast<size_t>(GetSelectedGear())].ComputeTorqueImpulse(GetExpectedInputSource(), *this);
			//	GetExpectedInputSource().ApplyUpstreamAngularImpulse(-matchImpulse, *this);
			//	ApplyDownstreamAngularImpulse(matchImpulse, *this);
			//}


			///The absolute hack, this HAS TO WORK.
			const Real desiredOutputSpeed(mInputShaftSpeed / GetSelectedGearRatio());
			const Real differenceToDesired(desiredOutputSpeed - mOutputShaftSpeed); //rad / sec
			const Real downstreamInertia(ComputeDownstreamInertia(*this)); //kg*m^2
			ApplyDownstreamAngularImpulse(differenceToDesired * downstreamInertia, *this);

			//This one doesn't work, what?
			//const Real desiredInputSpeed(mOutputShaftSpeed * GetSelectedGearRatio());
			//const Real differenceToDesired(desiredInputSpeed - mInputShaftSpeed); //rad / sec
			//const Real upstreamInertia(ComputeUpstreamInertia(*this)); //kg*m^2
			//ApplyUpstreamAngularImpulse(differenceToDesired * upstreamInertia, *this);
		}
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

	const RotatingBody* inputSource(GetInputSource());
	const Real upstreamInertia((nullptr == inputSource) ? 0.0 : inputSource->ComputeUpstreamInertia(fromSource));
	return GetInertia() + upstreamInertia * GetSelectedGearRatio();
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource)
{
	if (Gear::Neutral != mSelectedGear)
	{
		RotatingBody::OnDownstreamAngularVelocityChange(changeInAngularVelocity / GetSelectedGearRatio(), fromSource);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource)
{
	if (Gear::Neutral != mSelectedGear)
	{
		SetAngularVelocity(GetAngularVelocity() + changeInAngularVelocity);

		RotatingBody* inputSource(GetInputSource());
		if (nullptr != inputSource)
		{
			inputSource->OnUpstreamAngularVelocityChange(changeInAngularVelocity * GetSelectedGearRatio(), fromSource);
		}
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

Racecar::Real Racecar::GearJoint::ComputeTorqueImpulse(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep)
{
	//Wi = Wo / ratio
	//input.ComputeUpstreamInertia();

	((void)fixedTimeStep);
	return ComputeTorqueImpulseToMatchVelocity(input, output);
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::GearJoint::ComputeTorqueImpulseToMatchVelocity(const RotatingBody& input, const RotatingBody& output)
{
	const Real angularVelocityDifference(output.GetAngularVelocity() - (input.GetAngularVelocity()));
	const Real inputInertia(input.ComputeUpstreamInertia(output) / GetGearRatio());
	const Real outputInertia(output.ComputeDownstreamInertia(output) * GetGearRatio());

	const Real ratio(GetGearRatio());
	const Real denominator = outputInertia - inputInertia * (ratio * ratio * ratio); //Io = Ii * r^3
	const Real torqueImpulse = (inputInertia * outputInertia) * (ratio * angularVelocityDifference) / denominator;
	return torqueImpulse;
}

//--------------------------------------------------------------------------------------------------------------------//
