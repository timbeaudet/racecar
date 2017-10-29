///
/// @file
/// @details This is a simple simulation for a clutch, that applies forces to two separate rotating bodies via
///   a frictional clutch disk.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_clutch.h"
#include "racecar_controller.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::ClutchJoint::ClutchJoint(Real staticFrictionCoefficient, Real kineticFrictionCoefficient) :
	mStaticFrictionCoefficient(staticFrictionCoefficient),
	mKineticFrictionCoefficient(kineticFrictionCoefficient)
{
}

Racecar::ClutchJoint::~ClutchJoint(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::ClutchJoint::ComputeTorqueImpulse(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep)
{
	const Real frictionImpulse(ComputeTorqueImpulseFromFriction(input, output, fixedTimeStep));
	const Real matchingImpulse(ComputeTorqueImpulseToMatchVelocity(input, output));
	if (fabs(matchingImpulse) > frictionImpulse)
	{
		return frictionImpulse * Racecar::Sign(matchingImpulse);
	}

	return matchingImpulse;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::ClutchJoint::ComputeTorqueImpulseFromFriction(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep)
{
	const Real angularVelocityDifference(output.GetAngularVelocity() - input.GetAngularVelocity());
	const Real& frictionCoefficient((fabs(angularVelocityDifference) > 0.1) ? mKineticFrictionCoefficient: mStaticFrictionCoefficient);
	return mNormalForce * frictionCoefficient * fixedTimeStep;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::ClutchJoint::ComputeTorqueImpulseToMatchVelocity(const RotatingBody& input, const RotatingBody& output)
{
	const Real angularVelocityDifference(output.GetAngularVelocity() - input.GetAngularVelocity());
	const Real inputInertia(input.ComputeUpstreamInertia(output));
	const Real outputInertia(output.ComputeDownstreamInertia(output));

	const Real torqueImpulse = (inputInertia * outputInertia * angularVelocityDifference) / (inputInertia + outputInertia);
	return torqueImpulse;
}

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

Racecar::Clutch::Clutch(const Real& momentOfInertia, const Real& maximumNormalForce, 
	const Real& staticFrictionCoefficient, const Real& kineticFrictionCoefficient) :
	RotatingBody(momentOfInertia),
	mClutchEngagement(0.0),
	mMaximumNormalForce(maximumNormalForce),
	mClutchJoint(staticFrictionCoefficient, kineticFrictionCoefficient)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Clutch::~Clutch(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Clutch::OnControllerChange(const Racecar::RacecarControllerInterface& racecarController)
{
	mClutchEngagement = ClutchPedalToClutchForce(racecarController.GetClutchPosition());
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Clutch::Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime)
{
	RotatingBody& inputSource(GetExpectedInputSource());

	mClutchEngagement = ClutchPedalToClutchForce(racecarController.GetClutchPosition());

	const Real actualNormalForce(mClutchEngagement * mMaximumNormalForce); //N //See above comment for where this comes from!
	mClutchJoint.SetNormalForce(actualNormalForce);

	const Real frictionalImpulse = mClutchJoint.ComputeTorqueImpulse(inputSource, *this, fixedTime);
	if (fabs(frictionalImpulse) > kElipson)	//Make sure not zero!
	{
		inputSource.ApplyUpstreamAngularImpulse(frictionalImpulse, *this);
		ApplyDownstreamAngularImpulse(-frictionalImpulse, *this);
	}

	RotatingBody::Simulate(fixedTime);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Clutch::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	//When not engaged: return 0
	//When completely engaged: return RotatingBody::ComputeDownstreamInertia()
	//When partially engaged: Do black magic! ????

	if (mClutchEngagement < Racecar::PercentTo(0.5f))
	{
		return 0.0;
	}
	
	//May we need to do something special if the clutch is partially engaged / spinning different speeds than input?
	return RotatingBody::ComputeDownstreamInertia(fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Clutch::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	if (mClutchEngagement < Racecar::PercentTo(0.5))
	{
		return GetInertia();
	}

	return RotatingBody::ComputeUpstreamInertia(fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Clutch::OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource)
{
	if (mClutchEngagement < Racecar::PercentTo(0.5))
	{
		return;
	}

	RotatingBody::OnDownstreamAngularVelocityChange(changeInAngularVelocity, fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Clutch::OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource)
{
	if (mClutchEngagement < Racecar::PercentTo(0.5))
	{
		SetAngularVelocity(GetAngularVelocity() + changeInAngularVelocity);
		return;
	}

	RotatingBody::OnUpstreamAngularVelocityChange(changeInAngularVelocity, fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Clutch::ClutchPedalToClutchForce(const float pedalInput)
{
	const Real kClutchFullyEngaged(0.4);
	const Real kClutchDisengaged(0.6);

	if (pedalInput < kClutchFullyEngaged) { return 1.0; }
	if (pedalInput > kClutchDisengaged) { return 0.0; }
	const Real value(1.0 - ((pedalInput - kClutchFullyEngaged) / (kClutchDisengaged - kClutchFullyEngaged)));
	return (value < 0.0f) ? 0.0f : (1.0f < value) ? 1.0f : value;
}

//-------------------------------------------------------------------------------------------------------------------//
