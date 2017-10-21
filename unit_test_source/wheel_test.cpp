///
/// @file
/// @details A handful of test functions for testing the wheel component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "wheel_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_wheel.h"

#include <array>

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::WheelBrakingTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::Wheel wheel(8.0, 0.25); //0.5kg-m^2

	wheel.SetAngularVelocity(40.0); //rad/s
	wheel.SetMaximumBrakingTorque(20.0); //Nm

	//5Nm, should lose 0.1rad/s per time step.
	racecarController.SetBrakePosition(0.25);
	wheel.Simulate(racecarController, 0.01);
	{	//Wheel speed should now be 40.0 - 0.1 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.9) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//10Nm, should lose 0.2rad/s per time step.
	racecarController.SetBrakePosition(0.5);
	wheel.Simulate(racecarController, 0.01);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.7) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//20Nm, should lose 0.4rad/s per time step.
	racecarController.SetBrakePosition(1.0);
	wheel.Simulate(racecarController, 0.01);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 - 0.4 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.3) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//After 0.99 seconds the wheel should be going .4 + .3 + .2 rad/s due to above tests.
	for (int timer(30); timer < 1010; timer += 10)
	{
		wheel.Simulate(racecarController, 0.01);
	}
	{	//Wheel speed should now be  .1 rad/s due to above tests and 1.01 second simulation time.
		if (fabs(wheel.GetAngularVelocity() - 0.1) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	wheel.Simulate(racecarController, 0.01);
	wheel.Simulate(racecarController, 0.01);
	{	//Wheel speed should stay at 0.
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//
