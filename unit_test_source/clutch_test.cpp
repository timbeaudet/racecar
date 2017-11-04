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

//--------------------------------------------------------------------------------------------------------------------//

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
			clutch.Simulate(racecarController, kTestFixedTimeStep);
			inputBody.Simulate(kTestFixedTimeStep);
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
			clutch.Simulate(racecarController, kTestFixedTimeStep);
			inputBody.Simulate(kTestFixedTimeStep);
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
	//Racecar::ProgrammaticController racecarController;
	//Racecar::ConstantEngine engine(10.0, 1000.0, 0.0);
	//Racecar::Clutch clutch(10.0, 100, 0.6, 0.4);
	//Racecar::Wheel wheel(10.0, 1.0);

	//engine.AddOutputSource(&clutch);
	//clutch.SetInputSource(&engine);
	//clutch.AddOutputSource(&wheel);
	//wheel.SetInputSource(&clutch);

	//racecarController.SetThrottlePosition(1.0f);
	//racecarController.SetBrakePosition(0.0f);
	//racecarController.SetClutchPosition(0.0f);
	//for (int timer(0); timer < 1000; timer += 10)
	//{
	//	clutch.OnControllerChange(racecarController);
	//	engine.Simulate(racecarController, 0.01);
	//	clutch.Simulate(racecarController, 0.01);
	//	wheel.Simulate(racecarController, 0.01);
	//}

	//{	//Should differ, by an amount that Tim was too lazy to do the math for, this will prove it works, but not works perfectly!
	//	if (fabs(engine.GetAngularVelocity() - wheel.GetAngularVelocity()) < UnitTests::kTestElipson)
	//	{
	//		return false;
	//	}
	//	if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
	//	{	//Should always, ALWAYS, match clutch and wheel speed - (unless there is gear reduction).
	//		return false;
	//	}
	//}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

struct EngineClutchWheelTestBlob
{
	Racecar::Real mEngineInertia;
	Racecar::Real mEngineAngularVelocity;
	Racecar::Real mClutchInertia;
	Racecar::Real mClutchAngularVelocity;
	Racecar::Real mWheelInertia;

	Racecar::Real mExpectedEngineAngularVelocity[2]; //After a single step, after all steps.
	Racecar::Real mExpectedClutchAngularVelocity[2]; //After a single step, after all steps.
	Racecar::Real mExpectedWheelAngularVelocity[2]; //After a single step, after all steps.
};

bool Racecar::UnitTests::EngineClutchWheelThrottleTest(void)
{
	std::array<EngineClutchWheelTestBlob, 1> tests{
		EngineClutchWheelTestBlob{ 10.0, 0.0,   1.0, 0.0,   10.0,   { 0.47619047619, 47.619047619 }, { 0.47619047619, 47.619047619 }, { 0.47619047619, 47.619047619 } }
	};

	for (const EngineClutchWheelTestBlob& test : tests)
	{
		Racecar::ProgrammaticController racecarController;
		Racecar::ConstantEngine engine(test.mEngineInertia, 1000.0, 0.0);
		Racecar::Clutch clutch(test.mClutchInertia, 100, 0.6, 0.4);
		Racecar::Wheel wheel(test.mWheelInertia, 1.0);

		wheel.SetMaximumBrakingTorque(500.0);

		engine.AddOutputSource(&clutch);
		clutch.SetInputSource(&engine);
		clutch.AddOutputSource(&wheel);
		wheel.SetInputSource(&clutch);

		engine.SetAngularVelocity(test.mEngineAngularVelocity);
		clutch.SetAngularVelocity(test.mClutchAngularVelocity);
		wheel.SetAngularVelocity(clutch.GetAngularVelocity());

		racecarController.SetThrottlePosition(1.0f);
		racecarController.SetBrakePosition(0.0f);
		racecarController.SetClutchPosition(0.0f);

		//Simulate a single time step.
		clutch.OnControllerChange(racecarController);

		engine.Simulate(racecarController, kTestFixedTimeStep);
		clutch.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);

		{
			if (fabs(engine.GetAngularVelocity() - test.mExpectedEngineAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(clutch.GetAngularVelocity() - test.mExpectedClutchAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - test.mExpectedWheelAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
		}

		for (int timer(10); timer < 1000; timer += 10)
		{
			engine.Simulate(racecarController, kTestFixedTimeStep);
			clutch.Simulate(racecarController, kTestFixedTimeStep);
			wheel.Simulate(racecarController, kTestFixedTimeStep);
		}

		{
			if (fabs(engine.GetAngularVelocity() - test.mExpectedEngineAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(clutch.GetAngularVelocity() - test.mExpectedClutchAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - test.mExpectedWheelAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::EngineClutchWheelBrakingTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::ConstantEngine engine(10.0, 1000.0, 0.0);
	Racecar::Clutch clutch(1.0, 100, 0.6, 0.4);
	Racecar::Wheel wheel(10.0, 1.0);

	wheel.SetMaximumBrakingTorque(100.0);

	engine.AddOutputSource(&clutch);
	clutch.SetInputSource(&engine);
	clutch.AddOutputSource(&wheel);
	wheel.SetInputSource(&clutch);

	racecarController.SetThrottlePosition(1.0f);
	racecarController.SetBrakePosition(0.0f);
	racecarController.SetClutchPosition(0.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		clutch.OnControllerChange(racecarController);
		engine.Simulate(racecarController, kTestFixedTimeStep);
		clutch.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);
	}

	{
		if (fabs(engine.GetAngularVelocity() - 47.619047619) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(engine.GetAngularVelocity() - clutch.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	const Racecar::Real engineAngularVelocity(engine.GetAngularVelocity());
	const Racecar::Real clutchAngularVelocity(clutch.GetAngularVelocity());
	const Racecar::Real wheelAngularVelocity(wheel.GetAngularVelocity());
	
	//Release the throttle, but don't yet hit the brakes, everything SHOULD be spinning the same velocities as the,
	//ConstantEngine was setup without any resistance.
	racecarController.SetThrottlePosition(0.0f);
	racecarController.SetBrakePosition(0.0f);
	racecarController.SetClutchPosition(0.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		clutch.OnControllerChange(racecarController);
		engine.Simulate(racecarController, kTestFixedTimeStep);
		clutch.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);
	}

	{
		if (fabs(engine.GetAngularVelocity() - engineAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - clutchAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - wheelAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//Release the throttle and disengage the clutch, but don't yet hit the brakes.
	//everything SHOULD be spinning the same velocities as the ConstantEngine was setup without any resistance.
	racecarController.SetThrottlePosition(0.0f);
	racecarController.SetBrakePosition(0.0f);
	racecarController.SetClutchPosition(1.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		clutch.OnControllerChange(racecarController);
		engine.Simulate(racecarController, kTestFixedTimeStep);
		clutch.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);
	}

	{
		if (fabs(engine.GetAngularVelocity() - engineAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - clutchAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - wheelAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - engine.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	//Press the brake. The wheel and clutch should slow down, keep their velocities matched- but the engine
	//should remain identical speed it started at.
	racecarController.SetThrottlePosition(0.0f);
	racecarController.SetBrakePosition(1.0f);
	racecarController.SetClutchPosition(1.0f);
	for (int timer(0); timer < 1000; timer += 10)
	{
		clutch.OnControllerChange(racecarController);
		engine.Simulate(racecarController, kTestFixedTimeStep);
		clutch.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);
	}

	{
		if (fabs(engine.GetAngularVelocity() - engineAngularVelocity) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - (clutchAngularVelocity - 9.09090909)) > UnitTests::kTestElipson)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

struct EngineClutchWheelMismatchTestBlob
{
	Racecar::Real mEngineInertia;
	Racecar::Real mEngineAngularVelocity;
	Racecar::Real mClutchInertia;
	Racecar::Real mClutchAngularVelocity;
	Racecar::Real mClutchNormalForce;
	Racecar::Real mWheelInertia;

	Racecar::Real mExpectedEngineAngularVelocity[2]; //After a single step, after all steps.
	Racecar::Real mExpectedClutchAngularVelocity[2]; //After a single step, after all steps.
	Racecar::Real mExpectedWheelAngularVelocity[2]; //After a single step, after all steps.
};

bool Racecar::UnitTests::EngineClutchWheelMismatchTest(void)
{
	std::array<EngineClutchWheelMismatchTestBlob, 1> tests{
		EngineClutchWheelMismatchTestBlob{ 10.0, Racecar::RevolutionsMinuteToRadiansSecond(1000),   1.0, 0.0, 1000.0,   10.0, }
	};

	for (const EngineClutchWheelMismatchTestBlob& test : tests)
	{
		Racecar::ProgrammaticController racecarController;
		Racecar::ConstantEngine engine(test.mEngineInertia, 1000.0, 0.0);
		Racecar::Clutch clutch(test.mClutchInertia, test.mClutchNormalForce, 0.6, 0.4);
		Racecar::Wheel wheel(test.mWheelInertia, 1.0);

		wheel.SetMaximumBrakingTorque(100.0);

		engine.AddOutputSource(&clutch);
		clutch.SetInputSource(&engine);
		clutch.AddOutputSource(&wheel);
		wheel.SetInputSource(&clutch);

		engine.SetAngularVelocity(test.mEngineAngularVelocity);
		clutch.SetAngularVelocity(test.mClutchAngularVelocity);
		wheel.SetAngularVelocity(clutch.GetAngularVelocity());

		racecarController.SetThrottlePosition(0.0f);
		racecarController.SetBrakePosition(0.0f);
		racecarController.SetClutchPosition(1.0f);
		for (int timer(0); timer < 1000; timer += 10)
		{
			clutch.OnControllerChange(racecarController);
			engine.Simulate(racecarController, kTestFixedTimeStep);
			clutch.Simulate(racecarController, kTestFixedTimeStep);
			wheel.Simulate(racecarController, kTestFixedTimeStep);
		}

		{	//Just performed multiple time-steps with clutch disengaged, should remain in initial state.
			if (fabs(engine.GetAngularVelocity() - test.mEngineAngularVelocity) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(clutch.GetAngularVelocity() - test.mClutchAngularVelocity) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
		}


		//Release the clutch pedal so the clutch will engage and start spinning with the engine.
		racecarController.SetThrottlePosition(0.0f);
		racecarController.SetBrakePosition(0.0f);
		racecarController.SetClutchPosition(0.0f);

		clutch.OnControllerChange(racecarController);
		engine.Simulate(racecarController, kTestFixedTimeStep);
		clutch.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);

		{	//A single time-step of the clutch being engaged has now occurred, engine should slow, clutch and wheel should move faster and matched speeds.
			if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
		}

		//Now let it simulate for a bit of time.
		for (int timer(10); timer < 2000; timer += 10)
		{
			clutch.OnControllerChange(racecarController);
			engine.Simulate(racecarController, kTestFixedTimeStep);
			clutch.Simulate(racecarController, kTestFixedTimeStep);
			wheel.Simulate(racecarController, kTestFixedTimeStep);
		}

		{
			if (fabs(clutch.GetAngularVelocity() - engine.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(clutch.GetAngularVelocity() - wheel.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//
