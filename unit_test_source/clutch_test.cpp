///
/// @file
/// @details A handful of test functions for testing the clutch component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "clutch_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_engine.h"
#include "../source/racecar_clutch.h"
#include "../source/racecar_wheel.h"

#include <array>

//--------------------------------------------------------------------------------------------------------------------//

struct ClutchTestBlob
{
	Racecar::Real staticFriction;
	Racecar::Real kineticFriction;
	Racecar::Real maximumNormalForce;
	Racecar::Real inputInertia;
	Racecar::Real inputAngularVelocity;
	Racecar::Real outputInertia;
	Racecar::Real outputAngularVelocity;
	Racecar::Real mExpectedInputAngularVelocity[2]; //After a single step, after all steps.
	Racecar::Real mExpectedOutputAngularVelocity[2]; //After a single step, after all steps.
	int mTestTime;
};

bool Racecar::UnitTests::ClutchInputTest(void)
{
	std::array<ClutchTestBlob, 2> tests{
		ClutchTestBlob{ 0.6, 0.4, 1000,     10.0, 40.0,     10.0, -40.0,     { 0.0, 0.0 }, { 0.0, 0.0 }, 1000 },
		ClutchTestBlob{ 0.6, 0.4, 1000,     10.0, 40.0,     10.0,   0.0,     { 0.0, 20.0 }, { 0.0, 20.0 }, 1000 },
	};

	for (const ClutchTestBlob& test : tests)
	{
		Racecar::ProgrammaticController racecarController;

		//Racecar::Wheel inputWheel(8.0, 0.25);
		Racecar::RotatingBody inputBody(test.inputInertia);
		inputBody.SetAngularVelocity(test.inputAngularVelocity);

		Racecar::Clutch clutch(test.outputInertia, 1000, 0.6, 0.4); //kg*m^2
		clutch.SetAngularVelocity(test.outputAngularVelocity);

		clutch.SetInputSource(&inputBody);
		inputBody.AddOutputSource(&clutch);

		//Simulate 2 seconds with the clutch disengaged, nothing should change.
		racecarController.SetClutchPosition(1.0f); //Disengage the clutch!
		for (int timer(10); timer < 2000; timer += 10)
		{
			clutch.Simulate(racecarController, 0.01);
			inputBody.Simulate(0.01);
		}

		{	//Everything should be in the same state it started in initially.
			if (fabs(inputBody.GetAngularVelocity() - test.inputAngularVelocity) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(clutch.GetAngularVelocity() - test.outputAngularVelocity) > UnitTests::kTestElipson)
			{
				return false;
			}
		}

		//TODO: Compute and test a single time-step.

		//Simulate 2 seconds with clutch engaged.
		racecarController.SetClutchPosition(0.0f); //Engage the clutch!
		for (int timer(10); timer < 20000; timer += 10)
		{
			clutch.Simulate(racecarController, 0.01);
			inputBody.Simulate(0.01);
		}

		{	//Everything should be in the same state it started in initially.
			if (fabs(inputBody.GetAngularVelocity() - test.mExpectedInputAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(clutch.GetAngularVelocity() - test.mExpectedOutputAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::SlippingClutchTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::ConstantEngine engine(10.0, 1000);
	Racecar::Clutch clutch(10, 100, 0.6, 0.4);
	Racecar::Wheel wheel(1000, 1.0);

	engine.AddOutputSource(&clutch);
	clutch.SetInputSource(&engine);
	clutch.AddOutputSource(&wheel);
	wheel.SetInputSource(&clutch);

	racecarController.SetThrottlePosition(1.0);
	racecarController.SetClutchPosition(0.0);

	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, 0.01);
		clutch.Simulate(racecarController, 0.01);
		wheel.Simulate(racecarController, 0.01);
	}

	//Should differ, by an amount that Tim was too lazy to do the math for, this will prove it works, but not works perfectly!
	if (fabs(engine.GetAngularVelocity() - wheel.GetAngularVelocity()) < UnitTests::kTestElipson)
	{
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//
