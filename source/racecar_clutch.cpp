///
/// @file
/// @details This is a simple simulation for a clutch, that applies forces to two separate rotating bodies via
///   a frictional clutch disk.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_clutch.h"
#include "racecar_controller.h"

#include "../turtle_brains/tb_math_kit.h"
#include "../turtle_brains/tb_debug_kit.h"

#include "../gameplay_scene.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Clutch::Clutch(void) :
	mClutchEngagement(0.0f)
{
	SetInertia(Racecar::ComputeInertia(140.0f, 2.5f));
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Clutch::~Clutch(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Clutch::Simulate(const Racecar::RacecarControllerInterface& racecarController)
{
	RotatingBody& inputSource(GetExpectedInputSource());

	if (true == tbGame::Input::IsKeyDown(tbApplication::tbKey2))
	{
		SetAngularVelocity(inputSource.GetAngularVelocity());
	}

	mClutchEngagement = ClutchPedalToClutchForce(racecarController.GetClutchPosition());

	if (fabsf(inputSource.GetAngularVelocity() - GetAngularVelocity()) > Racecar::RevolutionsMinuteToDegreesSecond(100))
	{
		const float frictionalTorque(ComputeFrictionalTorque());
		inputSource.ApplyUpstreamTorque(-frictionalTorque, *this);
		ApplyDownstreamTorque(frictionalTorque, *this);
	}

	RotatingBody::Simulate();
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Clutch::ComputeDownstreamIntertia(const RotatingBody& fromSource) const
{
	if (this == &fromSource)
	{
		return RotatingBody::ComputeDownstreamIntertia(fromSource);
	}

	//return 0.0f;


	//if (this != &fromSource && false == tbMath::IsEqual(GetAngularVelocity(), GetExpectedInputSource().GetAngularVelocity(), 1.0f))
	//{	//The clutch is currently slipping, so it doesn't really apply inertia to things upstream.
	//	return 0.0f;
	//}

	//When not engaged: return 0
	//When completely engaged: return RotatingBody::ComputeDownstreamInertia()
	//When partially engaged: Do black magic!

	if (mClutchEngagement < Racecar::PercentTo(0.5f))
	{
		return 0.0f;
	}
	else if (mClutchEngagement > Racecar::PercentTo(99.5f))
	{
		return RotatingBody::ComputeDownstreamIntertia(fromSource);
	}
	
	//Time for black magic!
	return ComputeFrictionalTorque();
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Clutch::OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource)
{
	if (this == &fromSource)
	{
		RotatingBody::OnApplyDownstreamAcceleration(changeInAcceleration, fromSource);
		return;
	}

	//if (this != &fromSource && false == tbMath::IsEqual(GetAngularVelocity(), GetExpectedInputSource().GetAngularVelocity(), 1.0f))
	//{	//The clutch is currently slipping, so it does NOT apply acceleration to itself or downstream sources.
	//	return;
	//}

	if (mClutchEngagement < Racecar::PercentTo(0.5f))
	{
		return;
	}
	else if (mClutchEngagement > Racecar::PercentTo(99.5f))
	{
		RotatingBody::OnApplyDownstreamAcceleration(changeInAcceleration, fromSource);
		return;
	}

	//Somewhere in between; continue the black magic from above.
	RotatingBody::OnApplyDownstreamAcceleration(changeInAcceleration, fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Clutch::ClutchPedalToClutchForce(const float pedalInput)
{
	const float kClutchFullyEngaged(0.4f);
	const float kClutchDisengaged(0.6f);

	if (pedalInput < kClutchFullyEngaged) { return 1.0f; }
	if (pedalInput > kClutchDisengaged) { return 0.0f; }
	return tbMath::Clamp(1.0f - ((pedalInput - kClutchFullyEngaged) / (kClutchDisengaged - kClutchFullyEngaged)), 0.0f, 1.0f);
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Clutch::ComputeFrictionalTorque(void) const
{
	if (mClutchEngagement < PercentTo<float>(0.5f))
	{	//Clutch disengaged. Transfer NO forces, either way.
		return 0.0f;
	}

	const RotatingBody& inputSource(GetExpectedInputSource());

	//http://x-engineer.org/automotive-engineering/drivetrain/coupling-devices/calculate-torque-capacity-clutch/
	const float clutchStaticFrictionCoefficient(0.6f); //static steel on steel from: http://www.school-for-champions.com/science/friction_equation.htm#.WBSr1fkrLZI
	const float clutchKineticFrictionCoefficient(0.4f); //kinetic steel on steel

	const float engineVelocity(inputSource.GetAngularVelocity());
	const float transmissionVelocity(GetAngularVelocity());

	//http://www.thecartech.com/subjects/design/Automobile_clutchs.htm
	//318ft-lbs is the torque capacity of a 'race' miata clutch: https://www.flyinmiata.com/fm-level-1-clutch.html
	//8 1/2 inch diameter for clutch: http://www.autozone.com/drivetrain/clutch-set/duralast-clutch-set/245070_123121_4121
	//GUESSTIMATE: radius of effect - 3.75" 

	//7000N * 0.4 = 2800N --> Nm?
	//2800N * 0.09525m =  266 Nm
	//318ft-lbs ~= 432 Nm    / 0.09525m (3.75") / 0.6 (coefficient of friction)  = 7567 Nm

	const float frictionCoefficient(tbMath::IsEqual(engineVelocity, transmissionVelocity, 0.1f) ? clutchStaticFrictionCoefficient : clutchKineticFrictionCoefficient);
	const float forceNormal = mClutchEngagement * 7567.0f; //See above comment for where this comes from!
	const float maximumTorqueAmount(forceNormal * frictionCoefficient * tbMath::Convert::InchesToMeters(3.75f)); //Nm

	//engineVelocity * engineInertia = clutchVelocity * clutchInertia


	//engineInertia * 5000 + wheelInertia * 1000 = (engineInertia + wheelInertia) engineSpeed + wheelSpeed
	//const float velocityDifference(tbMath::Convert::DegreesToRadians(engineVelocity - transmissionVelocity)); //Positive if flywheel spinning faster than clutch disc.
	//velocityDifference * DriveTrainSimulation::kFixedTime
	//const float torqueAmount((velocityDifference > maximumTorqueAmount) ? maximumTorqueAmount : velocityDifference);

	//return torqueAmount;

	return (engineVelocity > transmissionVelocity) ? maximumTorqueAmount : -maximumTorqueAmount;
}

//-------------------------------------------------------------------------------------------------------------------//
