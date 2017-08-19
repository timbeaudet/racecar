///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "linear_motion_test.h"
#include "test_kit.h"

#include "../player_racecar_controller.h"
#include "../racecar/racecar.h"
#include "../racecar/racecar_wheel.h"
#include "../racecar/racecar_locked_differential.h"

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::WheelWithLinearMotion(void)
{
	Racecar::DoNothingController racecarController;
	Racecar::Wheel wheel(8.0, 0.25);
	wheel.SetOnGround(true);

	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamTorque(200.0, wheel); //200nm torque
		wheel.Simulate(racecarController); //Simulates 10ms of action.
	}

	const Real expectedLinearVelocity(100.0);  //100m/s
	const Real& finalLinearVelocity(wheel.GetLinearVelocity());
	if (fabs(finalLinearVelocity - expectedLinearVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}
	
	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

