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
		ApplyUpstreamAngularImpulse(-frictionalImpulse, *this);
	}

	////
	//// This is a bit of a hack to only apply the frictional forces if the there is a large enough difference in
	//// angular velocities, which prevents bouncing back and forth like crazy. Ultimately the ComputeFrictionalTorque
	//// function should be accounting for the velocities getting close enough to just match speeds.
	//if (fabs(inputSource.GetAngularVelocity() - GetAngularVelocity()) > Racecar::RevolutionsMinuteToRadiansSecond(250))
	//{
	//	const Real frictionalTorque(ComputeFrictionalTorque());
	//	inputSource.ApplyUpstreamTorque(-frictionalTorque, *this);
	//	ApplyDownstreamTorque(frictionalTorque, *this);
	//}

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
		//RotatingBody::OnUpstreamAngularVelocityChange(changeInAngularVelocity, fromSource);
		//RotatingBody* inputSource(GetInputSource());
		//if (nullptr != inputSource)
		//{	//Negate any changes we just passed upstream since they shouldn't go upstream in the first place.
		//	//There is currently no way with the API to set inputSource to null, or modify velocity directly,
		//	//so although this adds a bit of computation, and computational inaccuracies, it should work.
		//	//inputSource->OnUpstreamAngularVelocityChange(-changeInAngularVelocity, fromSource);
		//}

		
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

//Racecar::Real Racecar::Clutch::ComputeFrictionalTorque(void) const
//{
//	if (mClutchEngagement < PercentTo<float>(0.5))
//	{	//Clutch disengaged. Transfer NO forces, either way.
//		return 0.0;
//	}
//
//	const RotatingBody& inputSource(GetExpectedInputSource());
//
//	//http://x-engineer.org/automotive-engineering/drivetrain/coupling-devices/calculate-torque-capacity-clutch/
//
//	const Real engineVelocity(inputSource.GetAngularVelocity());
//	const Real clutchVelocity(GetAngularVelocity());
//
//	const Real frictionCoefficient((fabs(engineVelocity - clutchVelocity) < 0.1) ? mStaticFrictionCoefficient : mKineticFrictionCoefficient);
//	const Real actualNormalForce(mClutchEngagement * mMaximumNormalForce); //N //See above comment for where this comes from!
//	const Real maximumTorqueAmount(actualNormalForce * frictionCoefficient * (3.75 * 0.0254)); //Nm   (3.75in to meters)
//	
//
//	//engineVelocity * engineInertia = clutchVelocity * clutchInertia
//
//		//inputBody; momentumEngine = inertiaEngine * velocityEngine;   //inertiaEngine = inputSource->ComputeUpstreamInertia()
//		//outputBody; momentumClutch = inertiaClutch * velocityClutch;  //inertiaClutch = ComputeDownstreamInertia()
//		//momentumEngine + momentumClutch = X;
//
//		//X = inertiaClutch + inertiaEngine
//
//		//velDifference = velocityEngine - velocityClutch; //positive when engine spins faster
//		//value = inertiaEngine / inertiaClutch
//		//			100				1000 = .1
//		//velocityClutch += velDifference * value;
//		//velocityEngine -= velDifference * (1.0f / value)
//
//		//torqueClutch = (velDifference * value) * inertiaClutch
//		//torqueEngine = -(velDifference * (1.0f / value)) * inertiaEngine
//
//		//torqueClutch SHOULD EQUAL -torqueEngine
//
//
//
//	////////////////  Conservation of Momentum
//	//(engineInertia * engineVelocity) + (clutchInertia * clutchVelocity) = inertiaTotal + velocityTotal
//	//inertiaTotal = engineInertia + clutchInertia
//	//velocityTotal = (engineInertia * engineVelocity) + (clutchInertia * clutchVelocity) / inertiaTotal
//	//
//	//velocityTotal = engineVelocity + clutchVelocity
//	//inertiaTotal = (engineInertia * engineVelocity) + (clutchInertia * clutchVelocity) / velocityTotal
//	////////////////
//
//
//
//
////	before:
////		clutch:1000 Nm  @  2000RPM
////		engine:100 Nm   @  3000RPM
////	after:
////		clutch and engine wind up at ~2100RPM (I think)
//
//
//
//
//
//	////engineInertia * 5000 + wheelInertia * 1000 = (engineInertia + wheelInertia) engineSpeed + wheelSpeed
//	//const float velocityDifference(engineVelocity - clutchVelocity); //Positive if flywheel spinning faster than clutch disc.
//	//const float engineInertia(inputSource.ComputeUpstreamInertia(*this));
//	//const float clutchInertia(ComputeDownstreamIntertia(*this));
//	//							//   2000rpm             1000rpm             100             1000  =       2100
//	//const float targetVelocity = clutchVelocity + (velocityDifference * ((engineInertia * engineVelocity) / (clutchInertia * clutchVelocity)));
//	//const float torqueOnClutch((targetVelocity - clutchVelocity) * clutchInertia);
//	//const float torqueOnEngine(-(targetVelocity - engineVelocity) * engineInertia);
//
//	////const float value = engineInertia / clutchInertia; //.1
//	////const float torqueOnClutch = (velocityDifference * (value)) * clutchInertia; //100,000
//	////const float torqueOnEngine = -(velocityDifference * (1.0f / value)) * engineInertia; //1,000,000
//
//	//rand();
//
//	//return (torqueOnClutch + torqueOnEngine) / -2.0f;
//
//
//	//velocityDifference * DriveTrainSimulation::kFixedTime
//	//const float torqueAmount((velocityDifference > maximumTorqueAmount) ? maximumTorqueAmount : velocityDifference);
//
//	//return torqueAmount;
//
//	return (engineVelocity > clutchVelocity) ? maximumTorqueAmount : -maximumTorqueAmount;
//}

//-------------------------------------------------------------------------------------------------------------------//
