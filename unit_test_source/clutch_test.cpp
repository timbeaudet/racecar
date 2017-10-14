///
/// @file
/// @details A handful of test functions for testing the clutch component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "clutch_test.h"
#include "test_kit.h"

#include "../player_racecar_controller.h"
#include "../racecar/racecar.h"
#include "../racecar/racecar_clutch.h"
#include "../racecar/racecar_wheel.h"

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::ClutchInputTest(void)
{
	Racecar::ProgrammaticController racecarController;

	const Real initialWheelAngularVelocity(40.0); //rad/s
	Racecar::Wheel inputWheel(8.0, 0.25);
	inputWheel.SetAngularVelocity(initialWheelAngularVelocity);

	Racecar::Clutch clutch(inputWheel.GetInertia()); //kg*m^2
	clutch.SetAngularVelocity(-initialWheelAngularVelocity);

	clutch.SetInputSource(&inputWheel);
	inputWheel.AddOutputSource(&clutch);

	//Simulate 2 seconds with the clutch disengaged, nothing should change.
	racecarController.SetClutchPosition(1.0f); //Disengage the clutch!
	for (int timer(10); timer < 2000; timer += 10)
	{
		clutch.Simulate(racecarController, 0.01);
		inputWheel.Simulate(racecarController, 0.01);
	}

	{	//Everything should be in the same state it started in initially.
		if (fabs(inputWheel.GetAngularVelocity() - initialWheelAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() + initialWheelAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//Simulate 2 seconds with clutch engaged.
	racecarController.SetClutchPosition(0.0f); //Engage the clutch!
	for (int timer(10); timer < 20000; timer += 10)
	{
		clutch.Simulate(racecarController, 0.01);
		inputWheel.Simulate(racecarController, 0.01);
	}

	{	//Everything should be in the same state it started in initially.
		if (fabs(inputWheel.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//
