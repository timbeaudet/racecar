///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_wheel.h"
#include "racecar_controller.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::Wheel(const Real momentOfInertia) :
	RotatingBody(momentOfInertia)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::~Wheel(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real fixedTime)
{
	const RotatingBody* inputSource(GetInputSource());
	if (nullptr != inputSource)
	{
		SetAngularVelocity(inputSource->GetAngularVelocity());
	}

	if (racecarController.GetBrakePosition() > Racecar::PercentTo<float>(1.0f))
	{
		//The brake can apply negative force - need to clamp it
		//HELL - Need to do it correctly!!
		ApplyUpstreamTorque(-GetAngularVelocity() * (0.83f * racecarController.GetBrakePosition()) * fixedTime, *this);
	}

	RotatingBody::Simulate();
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::GetWheelSpeedMPH(void) const
{
	const Real speedFeetPerSecond(GetAngularVelocity() * (22.0 / 2.0) / 12.0);
	const Real speedMPH(speedFeetPerSecond * 60 * 60 / 5280.0);
	return fabs(speedMPH);
}

//-------------------------------------------------------------------------------------------------------------------//
