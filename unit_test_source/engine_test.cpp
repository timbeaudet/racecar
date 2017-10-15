///
/// @file
/// @details A handful of test functions for testing the engine component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "engine_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_engine.h"
#include "../source/racecar_wheel.h"

#include <array>

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::BasicEngineTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::ConstantEngine engine(10, 100);

	//Simulate 1 seconds with no throttle input, nothing should change.
	racecarController.SetThrottlePosition(0.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, 0.01);
	}

	{	//Everything should be in the same state it started in initially.
		if (fabs(engine.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//Compute and test a single time-step of constant engine torque.
	racecarController.SetThrottlePosition(1.0f);
	engine.Simulate(racecarController, 0.01);

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Single step.
		if (fabs(engine.GetAngularVelocity() - 0.1) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//Simulate 1 seconds with the constant engine torque of 100Nm.
	racecarController.SetThrottlePosition(1.0f);
	for (int timer(10); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, 0.01);
	}

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Multiple-steps.
		if (fabs(engine.GetAngularVelocity() - 10.0) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::EngineWithConnectionTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::ConstantEngine engine(10, 100);
	Racecar::Wheel wheel(10, 1.0);
	engine.AddOutputSource(&wheel);
	wheel.SetInputSource(&engine);

	//Simulate 1 seconds with no throttle input, nothing should change.
	racecarController.SetThrottlePosition(0.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, 0.01);
		wheel.Simulate(racecarController, 0.01);
	}

	{	//Everything should be in the same state it started in initially.
		if (fabs(engine.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//Compute and test a single time-step of constant engine torque.
	racecarController.SetThrottlePosition(1.0f);
	engine.Simulate(racecarController, 0.01);
	wheel.Simulate(racecarController, 0.01);

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Single step.
		if (fabs(engine.GetAngularVelocity() - 0.05) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - 0.05) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//Simulate 1 seconds with the constant engine torque of 100Nm.
	racecarController.SetThrottlePosition(1.0f);
	for (int timer(10); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, 0.01);
		wheel.Simulate(racecarController, 0.01);
	}

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Multiple-steps.
		if (fabs(engine.GetAngularVelocity() - 5.0) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - 5.0) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//
