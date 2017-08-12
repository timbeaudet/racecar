///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_wheel.h"
#include "racecar_transmission.h"
#include "racecar_controller.h"
#include "../drive_train_simulation.h" //For kFixedTime

#include "../turtle_brains/tb_math_kit.h"
#include "../turtle_brains/tb_debug_kit.h"

#include <cmath>

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::Wheel(void)
{
	SetInertia(Racecar::ComputeInertia(40.0f, 6.5f));
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::~Wheel(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::Simulate(const Racecar::RacecarControllerInterface& racecarController)
{
	RotatingBody& inputSource(GetExpectedInputSource());

	SetAngularVelocity(inputSource.GetAngularVelocity());
	if (racecarController.GetBrakePosition() > Racecar::PercentTo<float>(0.5f))
	{
		//The brake can apply negative force - need to clamp it
		//HELL - Need to do it correctly!!
		ApplyUpstreamTorque(-GetAngularVelocity() * (0.83f * racecarController.GetBrakePosition()) * DriveTrainSimulation::kFixedTime, *this);
	}

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
