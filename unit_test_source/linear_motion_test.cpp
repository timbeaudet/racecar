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
#include "../racecar/racecar_body.h"
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

bool Racecar::UnitTests::RacecarWithLinearMotion(void)
{
	Racecar::DoNothingController racecarController;
	Racecar::RacecarBody racecarBody(92.0);
	Racecar::Wheel wheel(8.0, 0.25);
	racecarBody.SetWheel(0, &wheel);
	wheel.SetRacecarBody(&racecarBody);
	wheel.SetOnGround(true);

	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamTorque(200.0, wheel); //200nm torque
		wheel.Simulate(racecarController); //Simulates 10ms of action.
		racecarBody.Simulate(racecarController);
	}

	const Real expectedLinearVelocity(8.0);  //m/s
	const Real& finalLinearVelocity(wheel.GetLinearVelocity());
	if (fabs(finalLinearVelocity - expectedLinearVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::FlyingCarHitsTrack(void)
{
	const Real initialVelocity(40.0); //m/s

	Racecar::DoNothingController racecarController;
	Racecar::RacecarBody racecarBody(92.0);
	Racecar::Wheel wheel(8.0, 0.25);
	racecarBody.SetWheel(0, &wheel);
	racecarBody.SetLinearVelocity(initialVelocity); //m/s
	wheel.SetRacecarBody(&racecarBody);
	wheel.SetLinearVelocity(racecarBody.GetLinearVelocity());

	//Simulate 1 second of applying a no torques.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.Simulate(racecarController); //Simulates 10ms of action.
		racecarBody.Simulate(racecarController);
	}

	const Real& finalLinearVelocity(wheel.GetLinearVelocity());
	if (fabs(finalLinearVelocity - initialVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	if (fabs(wheel.GetAngularVelocity()) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	//Still flying in the air, no change in angular or linear velocities.
	wheel.SetOnGround(true); //Land on the ground.
	wheel.Simulate(racecarController); //Simulates 10ms of action.
	racecarBody.Simulate(racecarController);

	const Real expectedLinearVelocity(37.037037037);
	const Real expectedAngularVelocity(148.148148148);
	const Real finalRacecarVelocity(racecarBody.GetLinearVelocity());
	const Real finalWheelLinearVelocity(wheel.GetLinearVelocity());
	const Real finalWheelAngularVelocity(wheel.GetAngularVelocity());
	if (fabs(expectedLinearVelocity - finalRacecarVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	if (fabs(expectedLinearVelocity - finalWheelLinearVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	if (fabs(expectedAngularVelocity - finalWheelAngularVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

