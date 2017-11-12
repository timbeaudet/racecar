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
	Racecar::ConstantEngine engine(10.0, 100.0, 50.0);

	//Simulate 1 seconds with no throttle input, nothing should change.
	racecarController.SetThrottlePosition(0.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
	}

	{	//Everything should be in the same state it started in initially.
		if (fabs(engine.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//Compute and test a single time-step of constant engine torque.
	racecarController.SetThrottlePosition(1.0f);
	engine.ControllerChange(racecarController);
	engine.Simulate(kTestFixedTimeStep);

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Single step.
		if (fabs(engine.GetAngularVelocity() - 0.1) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//Simulate 1 seconds with the constant engine torque of 100Nm.
	racecarController.SetThrottlePosition(1.0f);
	for (int timer(10); timer < 1000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
	}

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Multiple-steps.
		if (fabs(engine.GetAngularVelocity() - 10.0) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//Compute and test a single time-step of constant engine resistance torque.
	racecarController.SetThrottlePosition(0.0f);
	engine.ControllerChange(racecarController);
	engine.Simulate(kTestFixedTimeStep);

	{	//Should slow down the engine a little bit.
		if (fabs(engine.GetAngularVelocity() - 9.95) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//Simulate ~2 seconds with the constant lower engine RESISTANCE of 50Nm.
	racecarController.SetThrottlePosition(0.0f);
	for (int timer(10); timer < 1980; timer += 10)
	{
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
	}

	{	//Engine should be slowed down a lot.
		if (fabs(engine.GetAngularVelocity() - 0.1) > UnitTests::kTestEpsilon)
		{
			return false;
		}

		//Get the engine to hit a zero velocity state.
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
		if (fabs(engine.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}

		//Finally ensure that over simulating won't bounce back and forth, and should remain 0.
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
		engine.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);

		if (fabs(engine.GetAngularVelocity()) > UnitTests::kTestEpsilon)
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
	Racecar::ConstantEngine engine(10.0, 100.0, 0.0);
	Racecar::Wheel wheel(10, 1.0);
	engine.AddOutputSource(&wheel);
	wheel.SetInputSource(&engine);

	//Simulate 1 seconds with no throttle input, nothing should change.
	racecarController.SetThrottlePosition(0.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);
	}

	{	//Everything should be in the same state it started in initially.
		if (fabs(engine.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//Compute and test a single time-step of constant engine torque.
	racecarController.SetThrottlePosition(1.0f);
	engine.ControllerChange(racecarController);
	wheel.ControllerChange(racecarController);
	engine.Simulate(kTestFixedTimeStep);
	wheel.Simulate(kTestFixedTimeStep);

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Single step.
		if (fabs(engine.GetAngularVelocity() - 0.05) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - 0.05) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//Simulate 1 seconds with the constant engine torque of 100Nm.
	racecarController.SetThrottlePosition(1.0f);
	for (int timer(10); timer < 1000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);
		engine.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);
	}

	{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Multiple-steps.
		if (fabs(engine.GetAngularVelocity() - 5.0) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - 5.0) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::EngineTorqueTest(void)
{
	Racecar::TorqueCurve torqueCurve;
	torqueCurve.AddPlotPoint(0.0, 100.0);
	torqueCurve.AddPlotPoint(10000.1, 10100.0);
	torqueCurve.NormalizeTorqueCurve();

	for (int rpm = 0; rpm < 10000; ++rpm)
	{
		const Real expectedTorque(rpm + 100.0);
		if (false == ExpectedValueWithin(torqueCurve.GetOutputTorque(static_cast<Real>(rpm)), expectedTorque, 0.1, "Curve: OutputTorque does not match expected value."))
		{
			return false;
		}
	}

	const Racecar::Real constantTorque(100.0);
	Racecar::TorqueCurve torqueLine;
	torqueLine.AddPlotPoint(0.0, constantTorque);
	torqueLine.AddPlotPoint(10000.1, constantTorque);
	torqueLine.NormalizeTorqueCurve();

	for (int rpm = 0; rpm < 10000; ++rpm)
	{
		if (false == ExpectedValueWithin(torqueLine.GetOutputTorque(static_cast<float>(rpm)), constantTorque, 0.1, "Constant: OutputTorque does not match expected value."))
		{
			return false;
		}
	}

	//Racecar::ProgrammaticController racecarController;
	//racecarController.SetThrottlePosition(1.0);

	//Racecar::Engine engine(100, torqueLine); //100kg*m^2 inertia with 100Nm torque engine.
	//for (int timer(0); timer < 10000; timer += 10)
	//{
	//	engine.ControllerChange(racecarController);
	//	engine.Simulate(kTestFixedTimeStep);
	//}

	//{
	//	if (fabs(engine.GetAngularVelocity() - 10.0) > UnitTests::kTestEpsilon)
	//	{
	//		return false;
	//	}
	//}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

//bool EnginePowerTest(void);

//--------------------------------------------------------------------------------------------------------------------//
