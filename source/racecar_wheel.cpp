///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_wheel.h"
#include "racecar_transmission.h"
#include "racecar_controller.h"

#include "../turtle_brains/tb_math_kit.h"
#include "../turtle_brains/tb_debug_kit.h"

#include <cmath>

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::Wheel(void)
{
	SetInertia(Racecar::ComputeInertia(5.0f, 3.0f));
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::~Wheel(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::Simulate(const Racecar::RacecarControllerInterface& racecarController)
{
	RotatingBody& inputSource(GetExpectedInputSource());

	if (racecarController.GetBrakePosition() > Racecar::PercentTo<float>(0.5f))
	{
		ApplyUpstreamTorque(-GetAngularVelocity() * 0.23f, *this);
	}

	SetAngularVelocity(inputSource.GetAngularVelocity());

	RotatingBody::Simulate();
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Wheel::GetWheelSpeedMPH(void) const
{
	const float speedFeetPerSecond(tbMath::Convert::DegreesToRadians(GetAngularVelocity()) * (22.0f / 2.0f) / 12.0f);
	const float speedMPH(speedFeetPerSecond * 60 * 60 / 5280.0f);
	return fabsf(speedMPH);
}

//-------------------------------------------------------------------------------------------------------------------//
